//! Perception DST scenarios and the invariant each one checks.
//!
//! Every scenario is a function of its seed only. The seed drives all noise,
//! localization error, delays and network faults; geometry is fixed or derived
//! from the seed. A failure message always names the scenario, the seed, the
//! simulated time and the agent, and the CLI prints the command that replays it.
//!
//! What a passing scenario establishes is stated per scenario: it shows the
//! property held for that seed's realization of noise and faults, not that it
//! holds universally. Statistical checks (NEES) pool independent sub-runs and
//! use conservative bounds so that 100 CI seeds do not produce false alarms.
//!
//! Phrases such as "tracks every object", "no misses, no ghosts" and
//! "sub-meter error" in the invariants below mean [`TRACKING_CRITERION`].
//! Final convergence checks ("agents converge") are exact: bit-identical
//! digests after a reliable synchronization round.

use super::bus::{LinkFaults, Topology};
use super::harness::{
    run, AdversarySpec, AgentSpec, Behavior, BoxSensorSpec, LocalizationSpec, ObjectSpec,
    PartitionSpec, RunRecord, ScenarioSpec, SensorSpec, TruthSample,
};
use super::metrics::mean_chi2_bounds;
use godview_core::consistency::SourceStatus;
use godview_core::extent::yaw_residual;
use godview_core::frames::{EnuFrame, Geodetic};
use godview_core::godview_space::SpatialEngine;
use godview_core::perception_node::RejectReason;
use godview_core::report::{ReportError, ReportKey};
use godview_core::world_model::IdRule;
use h3o::Resolution;
use nalgebra::Vector3;
use rand::{Rng, SeedableRng};
use rand_chacha::ChaCha8Rng;
use std::collections::BTreeSet;

/// What "tracks every object" means in the invariants, sampled every 0.5 s.
pub const TRACKING_CRITERION: &str = "At each sample every expected object is matched (within 5 m) by exactly one \
world object with error under the scenario's bound. Gating is statistical, so an isolated error may occur (for \
example one report briefly outside its gate shows as a duplicate). An error attributed to the same object at two \
consecutive samples fails, as do more than one isolated error event (counted once per sample and object, \
however many agents show it) and more than 2% of expected object-samples.";

/// The result of running one catalog scenario with one seed.
#[derive(Debug, Clone)]
pub struct ScenarioOutcome {
    /// Scenario name.
    pub name: &'static str,
    /// Seed.
    pub seed: u64,
    /// The invariant or expected behavior checked.
    pub invariant: &'static str,
    /// Failed checks, each with time and agent context. Empty means passed.
    pub failures: Vec<String>,
    /// Key metrics for the report.
    pub metrics: Vec<(&'static str, String)>,
    /// Run digest (reproducibility).
    pub digest: u64,
    /// Simulated seconds.
    pub duration_s: f64,
}

impl ScenarioOutcome {
    /// True if every check passed.
    pub fn passed(&self) -> bool {
        self.failures.is_empty()
    }
}

/// A catalog entry.
#[derive(Clone, Copy)]
pub struct Entry {
    /// Scenario name (CLI identifier).
    pub name: &'static str,
    /// What the scenario checks.
    pub invariant: &'static str,
    /// Runs the scenario with a seed.
    pub run: fn(u64) -> ScenarioOutcome,
}

/// All perception scenarios.
pub fn catalog() -> Vec<Entry> {
    vec![
        Entry {
            name: "two_views",
            invariant: TWO_VIEWS,
            run: two_views,
        },
        Entry {
            name: "frame_invariance",
            invariant: FRAME_INVARIANCE,
            run: frame_invariance,
        },
        Entry {
            name: "localization_uncertainty",
            invariant: LOC_UNCERTAINTY,
            run: localization_uncertainty,
        },
        Entry {
            name: "localization_fusion",
            invariant: LOC_FUSION,
            run: localization_fusion,
        },
        Entry {
            name: "pose_drift",
            invariant: POSE_DRIFT,
            run: pose_drift,
        },
        Entry {
            name: "localization_dropout",
            invariant: LOC_DROPOUT,
            run: localization_dropout,
        },
        Entry {
            name: "flash_mob",
            invariant: H3_BOUNDARY,
            run: h3_boundary,
        },
        Entry {
            name: "network_faults",
            invariant: NETWORK_FAULTS,
            run: network_faults,
        },
        Entry {
            name: "slow_loris",
            invariant: SLOW_LORIS,
            run: slow_loris,
        },
        Entry {
            name: "biased_source",
            invariant: BIASED_SOURCE,
            run: biased_source,
        },
        Entry {
            name: "crossing_targets",
            invariant: CROSSING,
            run: crossing_targets,
        },
        Entry {
            name: "large_objects",
            invariant: LARGE_OBJECTS,
            run: large_objects,
        },
        Entry {
            name: "handoff_ids",
            invariant: HANDOFF_IDS,
            run: handoff_ids,
        },
        Entry {
            name: "object_expiry",
            invariant: EXPIRY,
            run: object_expiry,
        },
        Entry {
            name: "echo_chamber",
            invariant: ECHO,
            run: echo_chamber,
        },
        Entry {
            name: "split_brain",
            invariant: SPLIT_BRAIN,
            run: split_brain,
        },
        Entry {
            name: "agent_restart",
            invariant: RESTART,
            run: agent_restart,
        },
        Entry {
            name: "byzantine",
            invariant: BYZANTINE,
            run: byzantine,
        },
        Entry {
            name: "time_warp",
            invariant: TIME_WARP,
            run: time_warp,
        },
        Entry {
            name: "time_tornado",
            invariant: TIME_TORNADO,
            run: time_tornado,
        },
    ]
}

/// Looks up a scenario by name.
pub fn find(name: &str) -> Option<Entry> {
    catalog().into_iter().find(|e| e.name == name)
}

// ============================================================================
// SHARED CHECKS
// ============================================================================

const CUTOFF_M: f64 = 5.0;

struct Checker {
    name: &'static str,
    seed: u64,
    failures: Vec<String>,
}

impl Checker {
    fn new(name: &'static str, seed: u64) -> Self {
        Self {
            name,
            seed,
            failures: Vec::new(),
        }
    }

    fn fail(&mut self, msg: String) {
        // Keep reports readable: the first failures carry the diagnosis.
        if self.failures.len() < 12 {
            self.failures
                .push(format!("[{} seed={}] {}", self.name, self.seed, msg));
        }
    }

    fn check(&mut self, ok: bool, msg: impl FnOnce() -> String) {
        if !ok {
            self.fail(msg());
        }
    }

    /// [`TRACKING_CRITERION`] for the agents in `agents` at the samples in
    /// `[from_s, to_s)`, with matched error under `max_rms`.
    ///
    /// Each error is attributed to an object: a missed object to itself, a
    /// false estimate (ghost or duplicate) to the nearest expected object, and
    /// an accuracy violation to the agent. An error with the same attribution at
    /// two consecutive samples is a failure. Isolated errors are counted as
    /// events, one per (sample, object): a duplicate local track at one source
    /// shows in every agent's world at once, and is one event, not one per
    /// agent. More than 2% of expected object-samples (and more than one) is a
    /// failure. Every defect found while building this harness (wrong frames,
    /// duplicate local tracks, ghosts from stale data, split clusters under
    /// constant pose error, starved young tracks) lasted many consecutive samples.
    fn tracked_world(
        &mut self,
        rec: &RunRecord,
        agents: &[usize],
        from_s: f64,
        to_s: f64,
        max_rms: f64,
        expected: impl Fn(&TruthSample) -> bool + Copy,
    ) {
        const ACCURACY: u64 = u64::MAX;
        let mut events: BTreeSet<(usize, u64)> = BTreeSet::new();
        let mut object_samples = 0usize;
        let mut previous: Vec<BTreeSet<u64>> = vec![BTreeSet::new(); rec.ids.len()];
        for (si, s) in rec
            .samples
            .iter()
            .enumerate()
            .filter(|(_, s)| s.t_s >= from_s && s.t_s < to_s)
        {
            let truths: Vec<&TruthSample> = s.truths.iter().filter(|t| expected(t)).collect();
            object_samples += truths.len().max(1);
            for &a in agents {
                let sc = rec.score(s, a, CUTOFF_M, expected);
                let objects = &s.agents[a].world.objects;
                let mut keys: BTreeSet<u64> =
                    sc.unmatched_truths.iter().map(|&i| truths[i].id).collect();
                for &e in &sc.unmatched_estimates {
                    let p = objects[e].position();
                    let nearest = truths
                        .iter()
                        .min_by(|x, y| x.position.distance(&p).total_cmp(&y.position.distance(&p)))
                        .map_or(ACCURACY - 1, |t| t.id);
                    keys.insert(nearest);
                }
                if sc.rms_matched > max_rms {
                    keys.insert(ACCURACY - 2 - a as u64);
                }
                if !keys.is_empty() {
                    events.extend(keys.iter().map(|k| (si, *k)));
                    let repeated: Vec<u64> = keys.intersection(&previous[a]).copied().collect();
                    if !repeated.is_empty() {
                        self.fail(format!(
                            "t={:.2}s agent {a}: error persisted for two consecutive samples (objects {:?}; \
                             missed={} false={} matched={} rms={:.3} m, objects in world: {})",
                            s.t_s,
                            repeated,
                            sc.missed,
                            sc.false_estimates,
                            sc.matched,
                            sc.rms_matched,
                            objects.len()
                        ));
                    }
                }
                previous[a] = keys;
            }
        }
        let n = events.len();
        self.check(n <= 1 || n * 50 <= object_samples, || {
            let at: Vec<String> = events
                .iter()
                .take(6)
                .map(|(si, k)| format!("t={:.2}s object {k}", rec.samples[*si].t_s))
                .collect();
            format!(
                "{n} isolated error events over {object_samples} expected object-samples \
                 (more than 2%): {}",
                at.join(", ")
            )
        });
    }

    /// Every world object of every agent in `agents` satisfies `ok` (on its set
    /// of source agents) at the samples in `[from_s, to_s)`, under the same
    /// isolated-error tolerance as [`TRACKING_CRITERION`].
    ///
    /// A pair's position statistic can exceed its gate at one instant, which
    /// splits one cluster for one snapshot (seen in CI: 21.5 against 21.11,
    /// with fused snapshots on both sides). A violation by the same agent at
    /// two consecutive samples fails, as do more than one violating sample and
    /// more than 2% of checked object-samples.
    fn fused_members(
        &mut self,
        rec: &RunRecord,
        agents: &[usize],
        from_s: f64,
        to_s: f64,
        what: &str,
        ok: impl Fn(&BTreeSet<usize>) -> bool,
    ) {
        let mut bad_samples = 0usize;
        let mut checked = 0usize;
        let mut previous: BTreeSet<usize> = BTreeSet::new();
        for s in rec
            .samples
            .iter()
            .filter(|s| s.t_s >= from_s && s.t_s < to_s)
        {
            let mut now_bad = BTreeSet::new();
            for &a in agents {
                let members = members_by_agent(rec, s, a);
                checked += members.len();
                if let Some(m) = members.iter().find(|m| !ok(m)) {
                    now_bad.insert(a);
                    if previous.contains(&a) {
                        self.fail(format!(
                            "t={:.2}s agent {a}: {what} for two consecutive samples (members {m:?})",
                            s.t_s
                        ));
                    }
                }
            }
            if !now_bad.is_empty() {
                bad_samples += 1;
            }
            previous = now_bad;
        }
        self.check(bad_samples <= 1 || bad_samples * 50 <= checked, || {
            format!("{what} at {bad_samples} samples over {checked} checked object-samples (more than 2%)")
        });
    }

    /// After the final reliable round, every agent computes the same world, and
    /// it equals the world built from each source's own reports alone.
    fn converged(&mut self, rec: &RunRecord) {
        let Some(reference) = &rec.source_only_world else {
            self.fail("no final synchronization round recorded".into());
            return;
        };
        for (a, w) in rec.final_worlds.iter().enumerate() {
            if rec.ids.get(a).is_none() {
                continue;
            }
            self.check(w.digest() == reference.digest(), || {
                format!(
                    "agent {a} final world differs from source-only world ({} vs {} objects)",
                    w.objects.len(),
                    reference.objects.len()
                )
            });
        }
    }

    fn finish(
        self,
        rec: Option<&RunRecord>,
        invariant: &'static str,
        metrics: Vec<(&'static str, String)>,
    ) -> ScenarioOutcome {
        ScenarioOutcome {
            name: self.name,
            seed: self.seed,
            invariant,
            failures: self.failures,
            metrics,
            digest: rec.map_or(0, |r| r.digest()),
            duration_s: rec.map_or(0.0, |r| r.samples.last().map_or(0.0, |s| s.t_s)),
        }
    }
}

fn globally_observed(t: &TruthSample) -> bool {
    !t.global_observers.is_empty()
}

fn members_by_agent(rec: &RunRecord, s: &super::harness::Sample, a: usize) -> Vec<BTreeSet<usize>> {
    s.agents[a]
        .world
        .objects
        .iter()
        .map(|o| {
            o.members
                .iter()
                .filter_map(|((src, _), _)| rec.ids.iter().position(|id| id == src))
                .collect()
        })
        .collect()
}

fn mean_rms(rec: &RunRecord, agents: &[usize], from_s: f64) -> f64 {
    let mut v = Vec::new();
    for s in rec.samples.iter().filter(|s| s.t_s >= from_s) {
        for &a in agents {
            let sc = rec.score(s, a, CUTOFF_M, globally_observed);
            if sc.matched > 0 {
                v.push(sc.rms_matched);
            }
        }
    }
    if v.is_empty() {
        0.0
    } else {
        v.iter().sum::<f64>() / v.len() as f64
    }
}

fn pooled_nees(rec: &RunRecord, agents: &[usize], from_s: f64) -> (f64, usize) {
    let mut v = Vec::new();
    for s in rec.samples.iter().filter(|s| s.t_s >= from_s) {
        for &a in agents {
            v.extend(rec.score(s, a, CUTOFF_M, globally_observed).nees);
        }
    }
    (
        if v.is_empty() {
            0.0
        } else {
            v.iter().sum::<f64>() / v.len() as f64
        },
        v.len(),
    )
}

/// `n` agents on a circle, each with its own rotated and offset odometry frame.
pub fn ring(n: usize, radius: f64) -> Vec<AgentSpec> {
    (0..n)
        .map(|i| {
            let th = i as f64 / n as f64 * std::f64::consts::TAU + 0.3;
            let mut a = AgentSpec::at(Vector3::new(radius * th.cos(), radius * th.sin(), 2.0));
            // Every agent gets its own odometry frame: rotated and offset.
            a.odom_yaw_rad = 0.7 * i as f64 - 1.1;
            a.odom_offset_enu = Vector3::new(3.0 * i as f64, -2.0 * i as f64, 0.5);
            a
        })
        .collect()
}

fn seeded_objects(seed: u64, n: usize, half_extent: f64, max_speed: f64) -> Vec<ObjectSpec> {
    let mut rng = ChaCha8Rng::seed_from_u64(seed ^ 0x0b1e_c75e);
    (0..n)
        .map(|_| {
            ObjectSpec::moving(
                Vector3::new(
                    rng.gen_range(-half_extent..half_extent),
                    rng.gen_range(-half_extent..half_extent),
                    1.0,
                ),
                Vector3::new(
                    rng.gen_range(-max_speed..max_speed),
                    rng.gen_range(-max_speed..max_speed),
                    0.0,
                ),
            )
        })
        .collect()
}

/// `n` seeded objects at least `min_sep` apart for the whole run (rejection sampling).
pub fn separated_objects(
    seed: u64,
    n: usize,
    half_extent: f64,
    max_speed: f64,
    duration: f64,
    min_sep: f64,
) -> Vec<ObjectSpec> {
    let mut out: Vec<ObjectSpec> = Vec::new();
    let mut salt = 0;
    while out.len() < n && salt < 10_000 {
        salt += 1;
        let cand = seeded_objects(seed.wrapping_add(salt), 1, half_extent, max_speed)[0];
        let ok = out.iter().all(|o| {
            (0..=((duration * 4.0) as usize)).all(|k| {
                let t = k as f64 / 4.0;
                let a = o.position_enu + o.velocity_enu * t;
                let b = cand.position_enu + cand.velocity_enu * t;
                (a - b).norm() >= min_sep
            })
        });
        if ok {
            out.push(cand);
        }
    }
    assert_eq!(
        out.len(),
        n,
        "seed {seed}: could not place {n} objects {min_sep} m apart"
    );
    out
}

// ============================================================================
// SCENARIOS
// ============================================================================

const TWO_VIEWS: &str = "Two agents in different odometry frames observing the same objects produce exactly one shared \
world object per physical object, each fused from both sources, with sub-meter error; all agents converge to the \
world computed from the sources' own reports.";

fn two_views(seed: u64) -> ScenarioOutcome {
    let mut spec = ScenarioSpec::new("two_views", seed);
    spec.duration_s = 8.0;
    let mut a = AgentSpec::at(Vector3::new(-40.0, -10.0, 2.0));
    a.odom_yaw_rad = 0.5;
    a.odom_offset_enu = Vector3::new(5.0, 5.0, 0.0);
    let mut b = AgentSpec::at(Vector3::new(45.0, 30.0, 2.0));
    b.odom_yaw_rad = -2.2;
    b.odom_offset_enu = Vector3::new(-20.0, 4.0, 0.0);
    spec.agents = vec![a, b];
    spec.objects = separated_objects(seed, 3, 25.0, 3.0, spec.duration_s, 12.0);
    let rec = run(&spec);

    let mut c = Checker::new("two_views", seed);
    c.tracked_world(&rec, &[0, 1], 2.0, f64::INFINITY, 0.5, globally_observed);
    c.fused_members(
        &rec,
        &[0, 1],
        2.0,
        f64::INFINITY,
        "an object not fused from both sources",
        |m| *m == BTreeSet::from([0, 1]),
    );
    let (nees, n) = pooled_nees(&rec, &[0, 1], 2.0);
    // Sanity bound only: samples within one run are correlated in time.
    c.check(n > 0 && nees < 9.0, || {
        format!("pooled fused NEES {nees:.2} over {n} samples exceeds 3x the dimension")
    });
    c.converged(&rec);
    let metrics = vec![
        ("mean_rms_m", format!("{:.3}", mean_rms(&rec, &[0, 1], 2.0))),
        ("pooled_nees", format!("{nees:.2} (n={n})")),
    ];
    c.finish(Some(&rec), TWO_VIEWS, metrics)
}

const FRAME_INVARIANCE: &str =
    "The same physical scene observed through differently rotated and offset odometry frames \
yields the same shared world state (positions within 1 mm, same cardinality at every sample).";

fn frame_invariance(seed: u64) -> ScenarioOutcome {
    let build = |rotated: bool| {
        let mut spec = ScenarioSpec::new("frame_invariance", seed);
        spec.duration_s = 6.0;
        spec.agents = ring(3, 50.0);
        for (i, a) in spec.agents.iter_mut().enumerate() {
            if rotated {
                a.odom_yaw_rad = [1.3, -2.7, 0.4][i];
                a.odom_offset_enu = Vector3::new(
                    [100.0, -40.0, 7.0][i],
                    [-50.0, 12.0, 300.0][i],
                    [2.0, 0.0, -1.0][i],
                );
            } else {
                a.odom_yaw_rad = 0.0;
                a.odom_offset_enu = Vector3::zeros();
            }
        }
        spec.objects = separated_objects(seed, 4, 30.0, 4.0, spec.duration_s, 10.0);
        spec
    };
    let plain = run(&build(false));
    let rotated = run(&build(true));
    let mut c = Checker::new("frame_invariance", seed);
    let mut worst = 0.0f64;
    for (sp, sr) in plain.samples.iter().zip(rotated.samples.iter()) {
        for a in 0..3 {
            let (wp, wr) = (&sp.agents[a].world, &sr.agents[a].world);
            c.check(wp.objects.len() == wr.objects.len(), || {
                format!(
                    "t={:.2}s agent {a}: {} objects in ENU-aligned frames vs {} in rotated frames",
                    sp.t_s,
                    wp.objects.len(),
                    wr.objects.len()
                )
            });
            for o in &wp.objects {
                let d = wr
                    .objects
                    .iter()
                    .map(|q| q.position().distance(&o.position()))
                    .fold(f64::INFINITY, f64::min);
                worst = worst.max(d);
            }
        }
    }
    c.check(worst < 1e-3, || {
        format!("same object differs by {worst:.2e} m between frame choices")
    });
    c.converged(&rotated);
    c.finish(
        Some(&rotated),
        FRAME_INVARIANCE,
        vec![("max_position_difference_m", format!("{worst:.2e}"))],
    )
}

const LOC_UNCERTAINTY: &str = "Published reports include the publisher's localization and timing uncertainty: across \
60 independent runs the NEES of reports against truth is chi-square consistent (mean within a 4-sigma band around 3), \
and report covariance grows with range as heading error predicts.";

fn localization_uncertainty(seed: u64) -> ScenarioOutcome {
    let runs = 60;
    let mut nees = Vec::with_capacity(runs);
    let mut c = Checker::new("localization_uncertainty", seed);
    let mut trace_by_range: [Vec<f64>; 3] = [Vec::new(), Vec::new(), Vec::new()];
    let ranges = [20.0, 50.0, 90.0];
    let mut last = None;
    for r in 0..runs {
        let sub_seed = seed.wrapping_mul(1_000_003).wrapping_add(r as u64);
        let mut spec = ScenarioSpec::new("localization_uncertainty", sub_seed);
        spec.duration_s = 2.5;
        spec.final_sync = false;
        let mut a = AgentSpec::at(Vector3::new(0.0, 0.0, 2.0));
        a.odom_yaw_rad = 0.8;
        a.sensor = SensorSpec {
            sigma_m: 0.2,
            ..SensorSpec::default()
        };
        a.localization = LocalizationSpec {
            sigma_h_m: 0.5,
            sigma_v_m: 0.3,
            sigma_yaw_rad: 1f64.to_radians(),
            sigma_tilt_rad: 0.2f64.to_radians(),
            ..LocalizationSpec::default()
        };
        spec.agents = vec![a];
        spec.objects = ranges
            .iter()
            .enumerate()
            .map(|(k, rng_m)| {
                let th = 0.9 + 2.1 * k as f64;
                ObjectSpec::moving(
                    Vector3::new(rng_m * th.cos(), rng_m * th.sin(), 1.0),
                    Vector3::zeros(),
                )
            })
            .collect();
        let rec = run(&spec);
        let last_t = rec
            .report_checks
            .iter()
            .map(|x| x.t_s)
            .fold(f64::MIN, f64::max);
        let finals: Vec<_> = rec
            .report_checks
            .iter()
            .filter(|x| (x.t_s - last_t).abs() < 1e-9)
            .collect();
        for x in &finals {
            let k = ranges
                .iter()
                .position(|r| (x.range_m - r).abs() < 5.0)
                .unwrap_or(0);
            trace_by_range[k].push(x.pos_trace);
        }
        // One sample per run (rotating through the objects) keeps NEES samples independent:
        // reports from one run share the run's localization error.
        let want = ranges[r % 3];
        match finals.iter().find(|x| (x.range_m - want).abs() < 5.0) {
            Some(x) => nees.push(x.nees),
            None => c.fail(format!(
                "run {r} (seed {sub_seed}) published no report for the object at {want} m"
            )),
        }
        last = Some(rec);
    }
    let mean = nees.iter().sum::<f64>() / nees.len().max(1) as f64;
    let (lo, hi) = mean_chi2_bounds(3, nees.len().max(1), 4.0);
    c.check(mean >= lo && mean <= hi, || {
        format!(
            "mean report NEES {mean:.2} over {} independent runs outside [{lo:.2}, {hi:.2}]",
            nees.len()
        )
    });
    let avg = |v: &Vec<f64>| v.iter().sum::<f64>() / v.len().max(1) as f64;
    let (t20, t50, t90) = (
        avg(&trace_by_range[0]),
        avg(&trace_by_range[1]),
        avg(&trace_by_range[2]),
    );
    c.check(t20 < t50 && t50 < t90, || {
        format!("covariance trace does not grow with range: {t20:.3} / {t50:.3} / {t90:.3} m²")
    });
    let metrics = vec![
        ("mean_nees", format!("{mean:.2} in [{lo:.2}, {hi:.2}]")),
        (
            "pos_trace_20_50_90m",
            format!("{t20:.3} / {t50:.3} / {t90:.3}"),
        ),
    ];
    c.finish(last.as_ref(), LOC_UNCERTAINTY, metrics)
}

const LOC_FUSION: &str = "Agents with realistic localization error (0.4 m, 0.5 deg) still associate their reports of \
the same object, because pose uncertainty is in the covariance: no duplicates, no misses, bounded error.";

fn localization_fusion(seed: u64) -> ScenarioOutcome {
    let mut spec = ScenarioSpec::new("localization_fusion", seed);
    spec.duration_s = 8.0;
    spec.agents = ring(4, 45.0);
    for a in &mut spec.agents {
        a.localization = LocalizationSpec {
            sigma_h_m: 0.4,
            sigma_v_m: 0.2,
            sigma_yaw_rad: 0.5f64.to_radians(),
            sigma_tilt_rad: 0.1f64.to_radians(),
            ..LocalizationSpec::default()
        };
    }
    spec.objects = separated_objects(seed, 5, 30.0, 3.0, spec.duration_s, 12.0);
    let rec = run(&spec);
    let mut c = Checker::new("localization_fusion", seed);
    c.tracked_world(
        &rec,
        &[0, 1, 2, 3],
        2.0,
        f64::INFINITY,
        2.0,
        globally_observed,
    );
    c.converged(&rec);
    let (nees, n) = pooled_nees(&rec, &[0, 1, 2, 3], 2.0);
    c.check(nees < 12.0, || {
        format!("pooled fused NEES {nees:.2} (n={n}) far above dimension")
    });
    c.finish(
        Some(&rec),
        LOC_FUSION,
        vec![
            (
                "mean_rms_m",
                format!("{:.3}", mean_rms(&rec, &[0, 1, 2, 3], 2.0)),
            ),
            ("pooled_nees", format!("{nees:.2}")),
        ],
    )
}

const LOC_DROPOUT: &str = "An agent that loses global localization publishes only local-frame reports; peers stop \
fusing it within one validity period, keep tracking with the remaining sources (no misses, no ghosts), and resume \
fusing it after recovery.";

fn localization_dropout(seed: u64) -> ScenarioOutcome {
    let mut spec = ScenarioSpec::new("localization_dropout", seed);
    spec.duration_s = 10.0;
    spec.agents = ring(3, 50.0);
    spec.agents[2].localization.outages = vec![(3.0, 6.0)];
    spec.objects = separated_objects(seed, 3, 25.0, 2.0, spec.duration_s, 12.0);
    let rec = run(&spec);
    let mut c = Checker::new("localization_dropout", seed);
    c.check(
        rec.global_frame_publish_times[2]
            .iter()
            .all(|t| *t < 3.0 || *t >= 6.0),
        || "agent 2 published global-frame reports during its outage".into(),
    );
    c.check(
        rec.local_frame_publish_times[2]
            .iter()
            .any(|t| (3.0..6.0).contains(t)),
        || "agent 2 published nothing in its local frame during the outage".into(),
    );
    c.tracked_world(&rec, &[0, 1], 2.0, f64::INFINITY, 0.6, |t| {
        !t.observers.is_empty()
    });
    for s in rec.samples.iter() {
        for a in [0usize, 1] {
            let has2 = members_by_agent(&rec, s, a).iter().any(|m| m.contains(&2));
            if (4.6..6.0).contains(&s.t_s) {
                c.check(!has2, || {
                    format!(
                        "t={:.2}s agent {a} still fuses agent 2 without a global pose",
                        s.t_s
                    )
                });
            }
            if s.t_s >= 7.0 {
                c.check(has2, || {
                    format!("t={:.2}s agent {a} has not resumed fusing agent 2", s.t_s)
                });
            }
        }
    }
    c.converged(&rec);
    c.finish(
        Some(&rec),
        LOC_DROPOUT,
        vec![(
            "local_batches_agent2",
            rec.local_frame_publish_times[2].len().to_string(),
        )],
    )
}

const H3_BOUNDARY: &str = "Objects crossing H3 cell edges (including six that straddle the edges of one cell) stay one \
world object each, keep a stable id, and enter the spatial engine with their exact continuous position and \
uncertainty.";

fn h3_boundary(seed: u64) -> ScenarioOutcome {
    let mut spec = ScenarioSpec::new("flash_mob", seed);
    spec.duration_s = 10.0;
    let site = EnuFrame::new(spec.site);
    let res = Resolution::Ten;
    let cell = h3o::LatLng::new(spec.site.lat_deg, spec.site.lon_deg)
        .expect("site")
        .to_cell(res);
    let center: h3o::LatLng = cell.into();
    let center_enu = site.from_ecef(
        &Geodetic::new(center.lat(), center.lng(), spec.site.height_m)
            .expect("valid")
            .to_ecef(),
    );
    let boundary = cell.boundary();
    let mut objects = Vec::new();
    for k in 0..boundary.len() {
        let (a, b) = (boundary[k], boundary[(k + 1) % boundary.len()]);
        let mid = Geodetic::new(
            (a.lat() + b.lat()) / 2.0,
            (a.lng() + b.lng()) / 2.0,
            spec.site.height_m,
        )
        .expect("valid");
        let m = site.from_ecef(&mid.to_ecef());
        let out = Vector3::new(m.x - center_enu.x, m.y - center_enu.y, 0.0).normalize();
        // Start 12 m inside the edge, cross it at t = 2 s.
        objects.push(ObjectSpec::moving(
            Vector3::new(m.x, m.y, 1.0) - out * 12.0,
            out * 6.0,
        ));
    }
    // A moving grid: many more crossings of res-10 cells (66 m edges).
    for i in 0..6 {
        for j in 0..6 {
            let dir = if (i + j) % 2 == 0 { 1.0 } else { -1.0 };
            objects.push(ObjectSpec::moving(
                Vector3::new(-75.0 + 30.0 * i as f64, -75.0 + 30.0 * j as f64, 1.0),
                Vector3::new(7.0 * dir, 2.5 * dir, 0.0),
            ));
        }
    }
    // Sensor resolution is not under test here: drop any object that would pass
    // within 5 m of another during the run.
    let mut kept: Vec<ObjectSpec> = Vec::new();
    for o in objects {
        let clear = kept.iter().all(|k| {
            (0..=40).all(|s| {
                let t = s as f64 * spec.duration_s / 40.0;
                ((o.position_enu + o.velocity_enu * t) - (k.position_enu + k.velocity_enu * t))
                    .norm()
                    >= 5.0
            })
        });
        if clear {
            kept.push(o);
        }
    }
    spec.objects = kept;
    spec.agents = ring(4, 140.0);
    for a in &mut spec.agents {
        a.sensor.range_m = 400.0;
    }
    let rec = run(&spec);
    let mut c = Checker::new("flash_mob", seed);
    c.tracked_world(
        &rec,
        &[0, 1, 2, 3],
        2.0,
        f64::INFINITY,
        0.6,
        globally_observed,
    );

    // The scenario must actually exercise boundaries.
    let cell_of =
        |p: &godview_core::frames::Ecef| p.to_geodetic().to_latlng().map(|l| l.to_cell(res)).ok();
    let (first, last) = (rec.samples.first(), rec.samples.last());
    let crossed = match (first, last) {
        (Some(f), Some(l)) => f
            .truths
            .iter()
            .filter(|t| {
                l.truths
                    .iter()
                    .find(|u| u.id == t.id)
                    .is_some_and(|u| cell_of(&u.position) != cell_of(&t.position))
            })
            .count(),
        _ => 0,
    };
    c.check(crossed >= 20, || {
        format!("only {crossed} objects changed H3 cell; scenario does not exercise boundaries")
    });

    // Stable identity: the world object matched to each truth keeps its id.
    let mut churn = 0;
    let mut pairs = 0;
    for w in rec.samples.windows(2).filter(|w| w[0].t_s >= 2.0) {
        for t in &w[0].truths {
            let id_at = |s: &super::harness::Sample, pos: &godview_core::frames::Ecef| {
                s.agents[0]
                    .world
                    .objects
                    .iter()
                    .min_by(|a, b| {
                        a.position()
                            .distance(pos)
                            .total_cmp(&b.position().distance(pos))
                    })
                    .filter(|o| o.position().distance(pos) < CUTOFF_M)
                    .map(|o| o.id)
            };
            let Some(t1) = w[1].truths.iter().find(|u| u.id == t.id) else {
                continue;
            };
            if let (Some(a), Some(b)) = (id_at(&w[0], &t.position), id_at(&w[1], &t1.position)) {
                pairs += 1;
                if a != b {
                    churn += 1;
                }
            }
        }
    }
    c.check(churn * 50 <= pairs, || {
        format!("world ids changed {churn} times over {pairs} consecutive matches (>2%)")
    });

    // Shared state flows into the 3D spatial engine without losing precision.
    if let Some(world) = rec.final_worlds.first() {
        let mut engine = SpatialEngine::new(res);
        for o in &world.objects {
            let _ = engine.update_entity(o.to_entity());
            let _ = engine.update_entity(o.to_entity()); // updates must replace, not append
        }
        c.check(engine.stats().total_entities == world.objects.len(), || {
            format!(
                "spatial engine holds {} entities for {} world objects",
                engine.stats().total_entities,
                world.objects.len()
            )
        });
        for o in &world.objects {
            let e = o.to_entity();
            let back = Geodetic::new(e.position[0], e.position[1], e.position[2])
                .expect("valid")
                .to_ecef();
            c.check(back.distance(&o.position()) < 1e-6, || {
                format!(
                    "entity position drifted {:.2e} m",
                    back.distance(&o.position())
                )
            });
            let hits = engine.query_radius(e.position, 0.5);
            c.check(
                hits.iter()
                    .any(|h| h.id == e.id && h.position_cov_enu.is_some()),
                || {
                    format!(
                        "world object {:?} not found at its exact position with uncertainty",
                        o.id.1
                    )
                },
            );
        }
    }
    c.converged(&rec);
    c.finish(
        Some(&rec),
        H3_BOUNDARY,
        vec![
            ("objects_crossing_cells", crossed.to_string()),
            ("id_changes", format!("{churn}/{pairs}")),
        ],
    )
}

const NETWORK_FAULTS: &str = "Under 30% loss, 20% duplication, 5% corruption and jitter that reorders packets, corrupted \
batches are rejected, duplicates and stale versions change nothing, every agent tracks every object with no ghosts, \
and agents converge once delivery is reliable.";

fn network_faults(seed: u64) -> ScenarioOutcome {
    let mut spec = ScenarioSpec::new("network_faults", seed);
    spec.duration_s = 12.0;
    spec.agents = ring(4, 50.0);
    spec.objects = separated_objects(seed, 5, 30.0, 3.0, spec.duration_s, 12.0);
    spec.faults = LinkFaults {
        latency_ms: 30,
        jitter_ms: 150,
        loss: 0.3,
        duplicate: 0.2,
        corrupt: 0.05,
    };
    let rec = run(&spec);
    let mut c = Checker::new("network_faults", seed);
    let b = rec.bus;
    c.check(
        b.reordered > 0 && b.lost > 0 && b.duplicated > 0 && b.corrupted > 0,
        || format!("faults not exercised: {b:?}"),
    );
    let invalid: u64 = rec
        .node_stats
        .iter()
        .flat_map(|s| s.rejected.iter())
        .filter(|(r, _)| matches!(r, RejectReason::Invalid(_)))
        .map(|(_, n)| n)
        .sum();
    let dup: u64 = rec.node_stats.iter().map(|s| s.duplicate).sum();
    let stale: u64 = rec.node_stats.iter().map(|s| s.stale).sum();
    c.check(invalid > 0, || "no corrupted batch was rejected".into());
    // A corrupted message may also be duplicated in flight, so each corruption can
    // cause up to two rejections; nothing else may be rejected.
    c.check(invalid <= 2 * b.corrupted, || {
        format!("{invalid} rejections but only {} corruptions", b.corrupted)
    });
    c.check(dup > 0 && stale > 0, || {
        format!("duplicate={dup} stale={stale}: dedup/out-of-order paths not exercised")
    });
    c.tracked_world(
        &rec,
        &[0, 1, 2, 3],
        2.0,
        f64::INFINITY,
        0.6,
        globally_observed,
    );
    c.converged(&rec);
    c.finish(
        Some(&rec),
        NETWORK_FAULTS,
        vec![
            ("rejected_corrupt", invalid.to_string()),
            ("duplicates", dup.to_string()),
            ("stale", stale.to_string()),
            ("reordered", b.reordered.to_string()),
        ],
    )
}

const SLOW_LORIS: &str =
    "With 50% packet loss, soft-state republishing keeps every object in every agent's world (no \
misses, no ghosts) and agents converge once delivery is reliable.";

fn slow_loris(seed: u64) -> ScenarioOutcome {
    let mut spec = ScenarioSpec::new("slow_loris", seed);
    spec.duration_s = 10.0;
    spec.agents = ring(4, 50.0);
    spec.objects = separated_objects(seed, 4, 30.0, 3.0, spec.duration_s, 12.0);
    spec.faults = LinkFaults {
        latency_ms: 25,
        jitter_ms: 50,
        loss: 0.5,
        duplicate: 0.0,
        corrupt: 0.0,
    };
    let rec = run(&spec);
    let mut c = Checker::new("slow_loris", seed);
    let loss = rec.bus.lost as f64 / rec.bus.sent.max(1) as f64;
    c.check((0.4..0.6).contains(&loss), || {
        format!("realized loss {loss:.2} not near 0.5")
    });
    c.tracked_world(
        &rec,
        &[0, 1, 2, 3],
        2.0,
        f64::INFINITY,
        0.6,
        globally_observed,
    );
    c.converged(&rec);
    c.finish(
        Some(&rec),
        SLOW_LORIS,
        vec![("realized_loss", format!("{loss:.3}"))],
    )
}

const POSE_DRIFT: &str = "Each agent's position error drifts by up to 2 m horizontally over a 30 s cycle while it \
declares 0.3 m, as the two real cars of V2V4Real did. With relative alignment from shared objects, every object is \
tracked once with no persistent duplicates, the estimated relative offsets follow the injected drift, and agents \
converge. The same run without alignment must show duplicates or worlds emptied by the trust monitor excluding \
every source (issue #5).";

fn pose_drift(seed: u64) -> ScenarioOutcome {
    let build = |align: bool| {
        let mut spec = ScenarioSpec::new("pose_drift", seed);
        spec.duration_s = 20.0;
        spec.agents = ring(4, 45.0);
        for a in &mut spec.agents {
            a.localization = LocalizationSpec {
                sigma_h_m: 0.3,
                sigma_v_m: 0.2,
                sigma_yaw_rad: 0.1f64.to_radians(),
                sigma_tilt_rad: 0.05f64.to_radians(),
                drift_amp_enu: Vector3::new(2.0, 2.0, 0.3),
                drift_period_s: 30.0,
                ..LocalizationSpec::default()
            };
        }
        spec.objects = separated_objects(seed, 6, 30.0, 2.0, spec.duration_s, 12.0);
        spec.node.world.align.enabled = align;
        spec
    };
    let spec = build(true);
    let rec = run(&spec);
    let agents = [0, 1, 2, 3];
    let mut c = Checker::new("pose_drift", seed);
    c.tracked_world(&rec, &agents, 2.0, f64::INFINITY, 3.0, globally_observed);
    c.converged(&rec);
    // Estimated relative offsets against the injected drift. The agents'
    // constant errors (0.3 m declared) and heading errors are not included in
    // the reference, so agreement is only expected to within about a metre.
    let r = EnuFrame::new(spec.site).ecef_from_enu();
    let drift = |i: usize, t: f64| {
        let l = &spec.agents[i].localization;
        let ph = 2.0 * std::f64::consts::PI * t / l.drift_period_s + 2.1 * i as f64;
        r * Vector3::from_fn(|k, _| l.drift_amp_enu[k] * (ph + k as f64).sin())
    };
    let mut residuals = Vec::new();
    for smp in rec.samples.iter().filter(|s| s.t_s >= 2.0) {
        let al = &smp.agents[0].world.alignment;
        for x in al {
            for y in al {
                let (Some(i), Some(j)) = (
                    rec.ids.iter().position(|id| *id == x.source),
                    rec.ids.iter().position(|id| *id == y.source),
                ) else {
                    continue;
                };
                if i < j {
                    let e =
                        (x.translation - y.translation) + (drift(i, smp.t_s) - drift(j, smp.t_s));
                    residuals.push(e.norm());
                }
            }
        }
    }
    let mean_residual = residuals.iter().sum::<f64>() / residuals.len().max(1) as f64;
    c.check(!residuals.is_empty() && mean_residual < 1.0, || {
        format!(
            "estimated relative offsets miss the injected drift by {mean_residual:.2} m on average"
        )
    });
    // Ablation: no alignment, same drift.
    let ablation = run(&build(false));
    let dups = ablation
        .samples
        .iter()
        .filter(|s| s.t_s >= 2.0)
        .flat_map(|s| agents.iter().map(move |a| (s, *a)))
        .filter(|(s, a)| {
            ablation
                .score(s, *a, CUTOFF_M, globally_observed)
                .false_estimates
                > 0
        })
        .count();
    // Without alignment the trust monitor also sees every source disagree with
    // every other and eventually excludes them all (issue #5); count the
    // samples where an agent's world is empty although objects are in view.
    let emptied = ablation
        .samples
        .iter()
        .filter(|s| s.t_s >= 2.0)
        .flat_map(|s| agents.iter().map(move |a| (s, *a)))
        .filter(|(s, a)| {
            s.agents[*a].world.objects.is_empty() && s.truths.iter().any(globally_observed)
        })
        .count();
    c.check(dups + emptied > 0, || {
        "without alignment no duplicate or emptied world appeared; drift not exercised".into()
    });
    c.finish(
        Some(&rec),
        POSE_DRIFT,
        vec![
            ("mean_rms_m", format!("{:.3}", mean_rms(&rec, &agents, 2.0))),
            ("mean_offset_residual_m", format!("{mean_residual:.3}")),
            ("ablation_duplicate_agent_samples", dups.to_string()),
            ("ablation_emptied_world_agent_samples", emptied.to_string()),
            (
                "ablation_mean_rms_m",
                format!("{:.3}", mean_rms(&ablation, &agents, 2.0)),
            ),
        ],
    )
}

const BIASED_SOURCE: &str = "A source with an unreported 6 m localization bias is detected by every honest agent and \
excluded from fusion, no honest source is ever flagged, and honest worlds have no ghosts afterwards; the same run \
with trust disabled shows the ghosts that exclusion removes.";

fn biased_source(seed: u64) -> ScenarioOutcome {
    let build = |trust: bool| {
        let mut spec = ScenarioSpec::new("biased_source", seed);
        spec.duration_s = 14.0;
        spec.agents = ring(5, 55.0);
        spec.agents[4].localization.unmodeled_bias_enu = Vector3::new(6.0, 0.0, 0.0);
        spec.objects = separated_objects(seed, 6, 30.0, 2.0, spec.duration_s, 15.0);
        spec.node.disable_trust = !trust;
        spec
    };
    let rec = run(&build(true));
    let ablation = run(&build(false));
    let mut c = Checker::new("biased_source", seed);
    let honest = [0usize, 1, 2, 3];
    for s in &rec.samples {
        for &a in &honest {
            for (&src, st) in &s.agents[a].statuses {
                if src != 4 {
                    c.check(
                        !matches!(st, SourceStatus::Inflated | SourceStatus::Excluded),
                        || {
                            format!(
                                "t={:.2}s agent {a} falsely flagged honest agent {src} as {st:?}",
                                s.t_s
                            )
                        },
                    );
                }
            }
            if s.t_s >= 6.0 {
                let st = s.agents[a].statuses.get(&4).copied();
                c.check(st == Some(SourceStatus::Excluded), || {
                    format!("t={:.2}s agent {a} judges the biased agent {st:?}", s.t_s)
                });
            }
        }
    }
    c.tracked_world(&rec, &honest, 6.0, f64::INFINITY, 0.5, globally_observed);
    // Ablation: without trust the biased reports cannot associate (6 m off with
    // decimeter covariance) and appear as ghosts for the whole run.
    let ghost_samples = ablation
        .samples
        .iter()
        .filter(|s| s.t_s >= 6.0)
        .filter(|s| {
            honest.iter().all(|a| {
                ablation
                    .score(s, *a, CUTOFF_M, globally_observed)
                    .false_estimates
                    > 0
            })
        })
        .count();
    let late = ablation.samples.iter().filter(|s| s.t_s >= 6.0).count();
    c.check(ghost_samples == late, || {
        format!("with trust disabled only {ghost_samples}/{late} samples show ghosts; the scenario cannot show exclusion matters")
    });
    let first_excluded = rec
        .samples
        .iter()
        .find(|s| {
            honest
                .iter()
                .all(|a| s.agents[*a].statuses.get(&4) == Some(&SourceStatus::Excluded))
        })
        .map(|s| s.t_s);
    c.finish(
        Some(&rec),
        BIASED_SOURCE,
        vec![
            ("excluded_by_all_at_s", format!("{first_excluded:?}")),
            ("ablation_ghost_samples", format!("{ghost_samples}/{late}")),
        ],
    )
}

const CROSSING: &str = "Two same-class objects passing within 1.5 m of each other are not merged and keep their \
identities: each agent has both objects before and after the crossing, and the world id that follows each object \
does not swap.";

fn crossing_targets(seed: u64) -> ScenarioOutcome {
    let mut spec = ScenarioSpec::new("crossing_targets", seed);
    spec.duration_s = 10.0;
    let mut a = AgentSpec::at(Vector3::new(0.0, -60.0, 2.0));
    a.odom_yaw_rad = 0.3;
    let mut b = AgentSpec::at(Vector3::new(60.0, 0.0, 2.0));
    b.odom_yaw_rad = 2.0;
    spec.agents = vec![a, b];
    spec.objects = vec![
        ObjectSpec::moving(Vector3::new(-30.0, 0.0, 1.0), Vector3::new(6.0, 0.0, 0.0)),
        ObjectSpec::moving(Vector3::new(1.5, -30.0, 1.0), Vector3::new(0.0, 6.0, 0.0)),
    ];
    let rec = run(&spec);
    let mut c = Checker::new("crossing_targets", seed);
    c.tracked_world(&rec, &[0, 1], 2.0, 4.0, 0.5, globally_observed);
    c.tracked_world(&rec, &[0, 1], 7.0, f64::INFINITY, 0.5, globally_observed);
    let mut bad = 0;
    let mut total = 0;
    for s in rec.samples.iter().filter(|s| s.t_s >= 2.0) {
        for a in 0..2 {
            total += 1;
            let sc = rec.score(s, a, CUTOFF_M, globally_observed);
            if sc.missed + sc.false_estimates > 0 {
                bad += 1;
            }
        }
    }
    c.check(bad * 4 <= total, || {
        format!("{bad}/{total} samples with cardinality errors around the crossing")
    });
    let id_near = |t_s: f64, truth_idx: usize, a: usize| {
        let s = rec
            .samples
            .iter()
            .min_by(|x, y| (x.t_s - t_s).abs().total_cmp(&(y.t_s - t_s).abs()))?;
        let t = s.truths.get(truth_idx)?;
        s.agents[a]
            .world
            .objects
            .iter()
            .filter(|o| o.position().distance(&t.position) < CUTOFF_M)
            .min_by(|x, y| {
                x.position()
                    .distance(&t.position)
                    .total_cmp(&y.position().distance(&t.position))
            })
            .map(|o| o.id)
    };
    for a in 0..2 {
        for k in 0..2 {
            let (before, after) = (id_near(3.5, k, a), id_near(8.5, k, a));
            c.check(before.is_some() && before == after, || {
                format!(
                    "agent {a} object {k}: world id before crossing {:?} vs after {:?}",
                    before.map(|i| i.1),
                    after.map(|i| i.1)
                )
            });
        }
    }
    c.converged(&rec);
    c.finish(
        Some(&rec),
        CROSSING,
        vec![("samples_with_errors", format!("{bad}/{total}"))],
    )
}

const LARGE_OBJECTS: &str = "Cars, vans, trucks and buses are reported as oriented boxes by detectors that see only \
part of an object (the measured center moves up to a quarter of the length toward the sensor, with that uncertainty \
reported), that get each object's size and heading wrong by a fixed amount per agent (0.4 m per dimension and 0.05 \
rad, standard deviation, as the human annotators of the V2V4Real dataset do), and that cannot tell front from back. Each \
object is still one world object, fused from every agent that sees \
it from whatever side; every world object carries a box whose size is consistent with its reported uncertainty \
(pooled NEES below 6) and whose heading is within five declared standard deviations of the truth (at least 15 and at most 30 degrees), modulo 180 degrees unless the object \
moves, in which case front and back are resolved. The same run without declaring the detector's systematic center \
error must show duplicates, and so must the run without declaring its systematic size and heading errors.";

fn large_objects(seed: u64) -> ScenarioOutcome {
    let mut spec = ScenarioSpec::new("large_objects", seed);
    spec.duration_s = 10.0;
    spec.agents = ring(4, 45.0);
    let partial_view = 0.5;
    let (size_bias_m, yaw_bias_rad) = (0.4, 0.05);
    for a in &mut spec.agents {
        a.sensor.boxes = Some(BoxSensorSpec {
            partial_view,
            size_bias_m,
            yaw_bias_rad,
            ..BoxSensorSpec::default()
        });
    }
    // The detector's systematic errors, declared so they are published.
    spec.node.tracker.box_center_sys_frac = partial_view / 2.0;
    spec.node.tracker.box_size_sys_m = size_bias_m;
    spec.node.tracker.box_yaw_sys_rad = yaw_bias_rad;
    let sizes = [
        (4.5, 1.8, 1.5),
        (12.0, 2.5, 3.5),
        (12.0, 2.6, 3.2),
        (5.5, 2.0, 2.2),
    ];
    spec.objects = separated_objects(seed, 4, 25.0, 2.0, spec.duration_s, 16.0)
        .into_iter()
        .zip(sizes)
        .map(|(o, (l, w, h))| o.with_size(l, w, h))
        .collect();
    let rec = run(&spec);
    // Ablations: the same run without declaring the systematic center error,
    // and without declaring the systematic size and heading errors. Each must
    // split objects into duplicates, which shows the threat is exercised and
    // that the declaration is what handles it.
    let duplicate_samples = |r: &RunRecord| {
        r.samples
            .iter()
            .filter(|s| s.t_s >= 2.5)
            .flat_map(|s| (0..4).map(move |a| (s, a)))
            .filter(|(s, a)| r.score(s, *a, CUTOFF_M, globally_observed).false_estimates > 0)
            .count()
    };
    let mut blind = spec.clone();
    blind.node.tracker.box_center_sys_frac = 0.0;
    let ablation_dups = duplicate_samples(&run(&blind));
    let mut blind_size = spec.clone();
    blind_size.node.tracker.box_size_sys_m = 0.0;
    blind_size.node.tracker.box_yaw_sys_rad = 0.0;
    let size_ablation_dups = duplicate_samples(&run(&blind_size));
    let mut c = Checker::new("large_objects", seed);
    c.check(ablation_dups > 0, || {
        "without the declared systematic center error no duplicate appeared; partial views not exercised"
            .into()
    });
    c.check(size_ablation_dups > 0, || {
        "without the declared systematic size and heading errors no duplicate appeared; biases not exercised"
            .into()
    });
    c.tracked_world(
        &rec,
        &[0, 1, 2, 3],
        2.5,
        f64::INFINITY,
        2.0,
        globally_observed,
    );
    let mut size_nees = Vec::new();
    let mut worst_heading = 0.0f64;
    let mut heading_nees = Vec::new();
    for s in rec.samples.iter().filter(|s| s.t_s >= 3.0) {
        let truths: Vec<&TruthSample> = s.truths.iter().filter(|t| globally_observed(t)).collect();
        for a in 0..4 {
            let sc = rec.score(s, a, CUTOFF_M, globally_observed);
            let objects = &s.agents[a].world.objects;
            for &(e, ti) in &sc.pairs {
                let (Some(b), Some((size, heading))) =
                    (objects[e].object_box, truths[ti].object_box)
                else {
                    c.fail(format!(
                        "t={:.2}s agent {a}: world object without a box",
                        s.t_s
                    ));
                    continue;
                };
                size_nees.push(
                    (0..3)
                        .map(|i| (b.size[i] - size[i]).powi(2) / b.size_var[i])
                        .sum::<f64>(),
                );
                let err = yaw_residual(b.yaw, heading, b.yaw_ambiguous).abs();
                worst_heading = worst_heading.max(err);
                heading_nees.push(err * err / b.yaw_var);
                // Within five declared standard deviations, never tighter than
                // 15 degrees and never looser than 30: a fused heading comes
                // from one source, so it carries that source's whole bias, and
                // a 4-sigma bias draw (seen in CI) must not fail an honest
                // estimate, while a backwards or grossly wrong heading must.
                let bound = (5.0 * b.yaw_var.sqrt()).clamp(15f64.to_radians(), 30f64.to_radians());
                c.check(err < bound, || {
                    format!(
                        "t={:.2}s agent {a}: heading {:.1} deg off, bound {:.1} (ambiguous: {})",
                        s.t_s,
                        err.to_degrees(),
                        bound.to_degrees(),
                        b.yaw_ambiguous
                    )
                });
            }
        }
    }
    let pooled = size_nees.iter().sum::<f64>() / size_nees.len().max(1) as f64;
    c.check(!size_nees.is_empty() && pooled < 6.0, || {
        format!(
            "pooled size NEES {pooled:.2} over {} estimates",
            size_nees.len()
        )
    });
    c.converged(&rec);
    c.finish(
        Some(&rec),
        LARGE_OBJECTS,
        vec![
            (
                "pooled_size_nees",
                format!("{pooled:.2} (n={})", size_nees.len()),
            ),
            (
                "worst_heading_deg",
                format!("{:.1}", worst_heading.to_degrees()),
            ),
            (
                "pooled_heading_nees",
                format!(
                    "{:.2} (expect 1)",
                    heading_nees.iter().sum::<f64>() / heading_nees.len().max(1) as f64
                ),
            ),
            (
                "mean_rms_m",
                format!("{:.3}", mean_rms(&rec, &[0, 1, 2, 3], 2.5)),
            ),
            (
                "ablation_duplicate_agent_samples",
                ablation_dups.to_string(),
            ),
            (
                "size_ablation_duplicate_agent_samples",
                size_ablation_dups.to_string(),
            ),
        ],
    )
}

const HANDOFF_IDS: &str = "Objects drive past a row of agents with short sensor range, in both directions, so \
sources keep joining and leaving each object. With names carried in the reports (lineage), each object keeps one ID \
for its whole drive past (at most two changes per run are tolerated, for a cluster that splits for one snapshot), \
and no ID is ever given to two physical objects. In the same run under the oldest-track rule every change is its \
holder leaving or an older track joining and every object passes through at least three holders (handoffs are \
exercised), and the smallest-key rule renames objects on a join. All agents converge to the same world, IDs \
included.";

/// For each agent and object, the IDs it shows over time and the changes the
/// ID rule cannot explain (not "holder left" and not "an older track joined").
fn id_history(
    rec: &RunRecord,
    agents: &[usize],
) -> (usize, usize, Vec<String>, BTreeSet<(u64, ReportKey)>) {
    type Seen = (ReportKey, godview_env::Timestamp, BTreeSet<ReportKey>);
    let mut changes = 0;
    let mut unexplained = 0;
    let mut first = Vec::new();
    let mut holders = BTreeSet::new();
    for &a in agents {
        let mut prev: std::collections::BTreeMap<u64, (usize, Seen)> = Default::default();
        for (si, s) in rec.samples.iter().enumerate() {
            let truths: Vec<&TruthSample> =
                s.truths.iter().filter(|t| globally_observed(t)).collect();
            let sc = rec.score(s, a, CUTOFF_M, globally_observed);
            for &(e, ti) in &sc.pairs {
                let o = &s.agents[a].world.objects[e];
                let id = truths[ti].id;
                let members: BTreeSet<ReportKey> = o.members.iter().map(|(k, _)| *k).collect();
                holders.insert((id, o.id));
                if let Some((pi, (pid, pborn, _))) = prev.get(&id) {
                    if *pi + 1 == si && *pid != o.id {
                        changes += 1;
                        let holder_left = !members.contains(pid);
                        let older_joined = o.born < *pborn;
                        if !holder_left && !older_joined {
                            unexplained += 1;
                            if first.len() < 4 {
                                first.push(format!(
                                    "t={:.2}s agent {a} object {id}: renamed while its holder stayed and the new \
                                     holder is younger",
                                    s.t_s
                                ));
                            }
                        }
                    }
                }
                prev.insert(id, (si, (o.id, o.born, members)));
            }
        }
    }
    (changes, unexplained, first, holders)
}

fn handoff_ids(seed: u64) -> ScenarioOutcome {
    let mut spec = ScenarioSpec::new("handoff_ids", seed);
    // Long enough for the leading objects to pass all four agents, short
    // enough that none leaves the covered road (the tracking checks would then
    // see its coasting track as a false estimate, which is not this test).
    spec.duration_s = 26.0;
    spec.agents = (0..4)
        .map(|i| {
            let mut a = AgentSpec::at(Vector3::new(40.0 * i as f64, -12.0, 2.0));
            a.odom_yaw_rad = 0.9 * i as f64 - 1.3;
            a.odom_offset_enu = Vector3::new(-2.0 * i as f64, 3.0 * i as f64, 0.5);
            a.sensor.range_m = 32.0;
            a
        })
        .collect();
    // Eastbound on y = 0, westbound on y = 6, both directions so that every
    // pair of neighbors has a joiner with the smaller key in one direction.
    let mut rng = ChaCha8Rng::seed_from_u64(seed ^ 0x1d5);
    let mut objects = Vec::new();
    for (x0, y, dir) in [
        (-40.0, 0.0, 1.0),
        (-72.0, 0.0, 1.0),
        (160.0, 6.0, -1.0),
        (192.0, 6.0, -1.0),
    ] {
        let speed = rng.gen_range(6.5..7.5);
        let x0 = x0 + rng.gen_range(-2.0..2.0);
        objects.push(ObjectSpec::moving(
            Vector3::new(x0, y, 1.0),
            Vector3::new(dir * speed, 0.0, 0.0),
        ));
    }
    spec.objects = objects;
    let rec = run(&spec);
    let agents = [0, 1, 2, 3];
    let mut c = Checker::new("handoff_ids", seed);
    // Lineage (the default rule): IDs travel with the reports' names. An ID
    // must never move to a different physical object.
    let (changes, _, _, named) = id_history(&rec, &agents);
    let mut objects_per_id: std::collections::BTreeMap<ReportKey, BTreeSet<u64>> =
        Default::default();
    for (truth, id) in &named {
        objects_per_id.entry(*id).or_default().insert(*truth);
    }
    c.check(changes <= 2, || {
        format!("lineage: {changes} ID changes; objects should keep one ID while they stay covered")
    });
    for (id, truths) in &objects_per_id {
        c.check(truths.len() == 1, || {
            format!(
                "ID {}:{} was given to objects {truths:?}",
                id.0.short(),
                id.1 .0
            )
        });
    }
    // Oldest-track rule: every change must be its holder leaving or an older
    // track joining, and every object must pass through at least three
    // holders, which shows the run exercises handoffs.
    let mut oldest = spec.clone();
    oldest.node.world.id_rule = IdRule::OldestTrack;
    let oldest_rec = run(&oldest);
    let (oldest_changes, unexplained, first, holders) = id_history(&oldest_rec, &agents);
    for f in first {
        c.fail(f);
    }
    c.check(unexplained == 0, || {
        format!("oldest-track rule: {unexplained} of {oldest_changes} ID changes not explained by the holder leaving or an older track joining")
    });
    for id in 0..spec.objects.len() as u64 {
        let n = holders.iter().filter(|(o, _)| *o == id).count();
        c.check(n >= 3, || {
            format!(
                "object {id} had {n} holders under the oldest-track rule; handoffs not exercised"
            )
        });
    }
    // Smallest-key rule: renames objects when a source joins.
    let mut old = spec.clone();
    old.node.world.id_rule = IdRule::SmallestKey;
    let old_rec = run(&old);
    let (old_changes, old_unexplained, _, _) = id_history(&old_rec, &agents);
    c.check(old_unexplained > 0, || {
        "the smallest-key rule never renamed an object on a join; joins not exercised".into()
    });
    let rms = mean_rms(&rec, &agents, 2.0);
    c.check(rms < 1.0, || format!("mean matched RMS {rms:.3} m"));
    c.converged(&rec);
    c.finish(
        Some(&rec),
        HANDOFF_IDS,
        vec![
            ("id_changes", changes.to_string()),
            ("oldest_track_rule_changes", oldest_changes.to_string()),
            (
                "smallest_key_rule_changes",
                format!("{old_changes} ({old_unexplained} on a join)"),
            ),
            ("mean_rms_m", format!("{rms:.3}")),
        ],
    )
}

const EXPIRY: &str = "When an object disappears, every agent drops it within coast + validity time (via tombstones \
or expiry) and never shows it again; a newly appearing object is present after the confirmation latency.";

fn object_expiry(seed: u64) -> ScenarioOutcome {
    let mut spec = ScenarioSpec::new("object_expiry", seed);
    spec.duration_s = 10.0;
    spec.agents = ring(3, 45.0);
    // The late object comes from the same separated set: a hand-placed one could
    // pass within the sensor's resolution of another object (it did, 0.44 m, for
    // one seed), which tests resolution rather than expiry.
    let mut objs = separated_objects(seed, 5, 25.0, 2.0, spec.duration_s, 12.0);
    objs[0].remove_s = Some(4.0);
    objs[4].spawn_s = 5.0;
    spec.objects = objs;
    let rec = run(&spec);
    let mut c = Checker::new("object_expiry", seed);
    c.tracked_world(&rec, &[0, 1, 2], 2.0, 4.0, 0.5, globally_observed);
    // Coast (1.5 s) + one publish period + latency, rounded up to the next sample.
    c.tracked_world(&rec, &[0, 1, 2], 6.0, f64::INFINITY, 0.5, globally_observed);
    c.converged(&rec);
    c.finish(Some(&rec), EXPIRY, vec![])
}

const ECHO: &str = "Five agents in a line relay every batch they receive (with duplication and jitter). Reports reach \
every agent over multiple hops, repeated delivery of already-fused information changes nothing, and every agent's \
world equals the one computed from each source's own reports exactly (no uncertainty reduction from echoes).";

fn echo_chamber(seed: u64) -> ScenarioOutcome {
    let mut spec = ScenarioSpec::new("echo_chamber", seed);
    spec.duration_s = 8.0;
    spec.topology = Topology::Line;
    spec.faults = LinkFaults {
        latency_ms: 15,
        jitter_ms: 40,
        loss: 0.0,
        duplicate: 0.3,
        corrupt: 0.0,
    };
    spec.agents = (0..5)
        .map(|i| {
            let mut a = AgentSpec::at(Vector3::new(-80.0 + 40.0 * i as f64, -40.0, 2.0));
            a.relay = true;
            a.odom_yaw_rad = 0.4 * i as f64;
            a.sensor.range_m = 250.0;
            a
        })
        .collect();
    spec.objects = separated_objects(seed, 3, 30.0, 2.0, spec.duration_s, 12.0);
    let rec = run(&spec);
    let mut c = Checker::new("echo_chamber", seed);
    for (a, st) in rec.node_stats.iter().enumerate().take(4).skip(1) {
        c.check(st.duplicate > 0, || {
            format!("agent {a} saw no duplicate reports; echoes not exercised")
        });
    }
    c.fused_members(
        &rec,
        &[0, 4],
        3.0,
        f64::INFINITY,
        "an object not fused from all 5 sources via relays",
        |m| m.len() == 5,
    );
    c.tracked_world(&rec, &[0, 2, 4], 3.0, f64::INFINITY, 0.5, globally_observed);
    c.converged(&rec);
    let dup: u64 = rec.node_stats.iter().map(|s| s.duplicate).sum();
    c.finish(
        Some(&rec),
        ECHO,
        vec![("duplicate_reports_absorbed", dup.to_string())],
    )
}

const SPLIT_BRAIN: &str = "During a partition each side fuses only its own sources (others expire within one validity \
period) and keeps tracking; after healing every agent fuses all sources again and all agents converge to one world.";

fn split_brain(seed: u64) -> ScenarioOutcome {
    let mut spec = ScenarioSpec::new("split_brain", seed);
    spec.duration_s = 12.0;
    spec.agents = ring(4, 50.0);
    spec.partitions = vec![PartitionSpec {
        start_s: 3.0,
        end_s: 7.0,
        group_a: vec![0, 1],
        group_b: vec![2, 3],
    }];
    spec.objects = separated_objects(seed, 3, 25.0, 2.0, spec.duration_s, 12.0);
    let rec = run(&spec);
    let mut c = Checker::new("split_brain", seed);
    c.tracked_world(
        &rec,
        &[0, 1, 2, 3],
        2.0,
        f64::INFINITY,
        0.6,
        globally_observed,
    );
    for s in &rec.samples {
        for a in 0..4usize {
            let side: BTreeSet<usize> = if a < 2 {
                BTreeSet::from([0, 1])
            } else {
                BTreeSet::from([2, 3])
            };
            for m in members_by_agent(&rec, s, a) {
                if (4.6..7.0).contains(&s.t_s) {
                    c.check(m.is_subset(&side), || {
                        format!(
                            "t={:.2}s agent {a} fuses across the partition: {m:?}",
                            s.t_s
                        )
                    });
                }
            }
        }
    }
    // Never fusing across the partition is checked strictly above: no message
    // crosses it, so a violation cannot be a statistical tail.
    c.fused_members(
        &rec,
        &[0, 1, 2, 3],
        8.0,
        f64::INFINITY,
        "an object not fused from all 4 sources after healing",
        |m| m.len() == 4,
    );
    c.converged(&rec);
    c.finish(
        Some(&rec),
        SPLIT_BRAIN,
        vec![("blocked_messages", rec.bus.blocked.to_string())],
    )
}

const RESTART: &str = "An agent disconnected and restarted (new epoch, track ids reused from 1) is accepted again on \
reconnection: its new-epoch reports replace its old ones, peers fuse it again, and no ghosts remain.";

fn agent_restart(seed: u64) -> ScenarioOutcome {
    let mut spec = ScenarioSpec::new("agent_restart", seed);
    spec.duration_s = 11.0;
    spec.agents = ring(3, 50.0);
    spec.agents[2].restart_at_s = Some(4.5);
    spec.partitions = vec![PartitionSpec {
        start_s: 3.0,
        end_s: 6.0,
        group_a: vec![2],
        group_b: vec![0, 1],
    }];
    spec.objects = separated_objects(seed, 3, 25.0, 2.0, spec.duration_s, 12.0);
    let rec = run(&spec);
    let mut c = Checker::new("agent_restart", seed);
    c.fused_members(
        &rec,
        &[0, 1],
        8.0,
        f64::INFINITY,
        "an object not fused with the restarted agent",
        |m| m.contains(&2),
    );
    c.tracked_world(&rec, &[0, 1, 2], 8.0, f64::INFINITY, 0.6, globally_observed);
    c.converged(&rec);
    c.finish(Some(&rec), RESTART, vec![])
}

const BYZANTINE: &str = "A validly signed agent fabricating objects creates ghosts until it is revoked, after which \
honest worlds are exact and its traffic is rejected; replayed and forged batches are rejected and leave every honest \
agent's world bit-identical to a control run without them.";

fn byzantine(seed: u64) -> ScenarioOutcome {
    let build = |attack: bool| {
        let mut spec = ScenarioSpec::new("byzantine", seed);
        spec.duration_s = 10.0;
        spec.agents = ring(4, 50.0);
        spec.agents[3].behavior = Behavior::Fabricator { count: 5 };
        // This scenario isolates the cryptographic and revocation layer. With
        // trust enabled the consistency monitor often excludes the fabricator
        // first (its fake objects contradict honest partners); biased_source
        // covers that layer.
        spec.node.disable_trust = true;
        spec.objects = separated_objects(seed, 3, 25.0, 2.0, spec.duration_s, 12.0);
        spec.faults = LinkFaults {
            corrupt: 0.05,
            ..LinkFaults::default()
        };
        spec.adversary = AdversarySpec {
            revoke: vec![(3, 5.0)],
            replay_at_s: if attack { vec![6.0, 8.0] } else { vec![] },
            forge_at_s: if attack { vec![7.0] } else { vec![] },
        };
        spec
    };
    let attacked = run(&build(true));
    let control = run(&build(false));
    let mut c = Checker::new("byzantine", seed);
    let honest = [0usize, 1, 2];
    let ghosts_before = attacked
        .samples
        .iter()
        .filter(|s| (2.0..5.0).contains(&s.t_s))
        .any(|s| {
            honest.iter().any(|a| {
                attacked
                    .score(s, *a, CUTOFF_M, globally_observed)
                    .false_estimates
                    >= 3
            })
        });
    c.check(ghosts_before, || {
        "fabricated objects never appeared before revocation; threat not exercised".into()
    });
    c.tracked_world(
        &attacked,
        &honest,
        5.2,
        f64::INFINITY,
        0.6,
        globally_observed,
    );
    let count = |rec: &RunRecord, a: usize, f: &dyn Fn(&RejectReason) -> bool| -> u64 {
        rec.node_stats[a]
            .rejected
            .iter()
            .filter(|(r, _)| f(r))
            .map(|(_, n)| *n)
            .sum()
    };
    for a in 1..3 {
        c.check(
            count(&attacked, a, &|r| *r == RejectReason::Revoked) > 0,
            || format!("agent {a} never rejected the revoked source"),
        );
        c.check(
            count(&attacked, a, &|r| {
                *r == RejectReason::Invalid(ReportError::BadSignature)
            }) > 0,
            || format!("agent {a} did not reject the forged batch"),
        );
        let replayed = attacked.node_stats[a].stale
            + attacked.node_stats[a].below_floor
            + attacked.node_stats[a].duplicate;
        let base = control.node_stats[a].stale
            + control.node_stats[a].below_floor
            + control.node_stats[a].duplicate;
        c.check(replayed > base, || {
            format!("agent {a}: replays not observed ({replayed} vs control {base})")
        });
        for (sa, sc) in attacked.samples.iter().zip(control.samples.iter()) {
            c.check(
                sa.agents[a].neutral_digest == sc.agents[a].neutral_digest,
                || {
                    format!(
                        "t={:.2}s agent {a}: world differs from control run under replay/forgery",
                        sa.t_s
                    )
                },
            );
        }
    }
    c.converged(&attacked);
    let rej: u64 = attacked
        .node_stats
        .iter()
        .flat_map(|s| s.rejected.values())
        .sum();
    c.finish(
        Some(&attacked),
        BYZANTINE,
        vec![("rejected_batches", rej.to_string())],
    )
}

const TIME_WARP: &str =
    "Detections delayed by 0-400 ms (out of order, inside the 0.5 s retrodiction window) are all \
fused (none rejected as stale) and every agent tracks every object with sub-meter error.";

fn time_warp(seed: u64) -> ScenarioOutcome {
    let mut spec = ScenarioSpec::new("time_warp", seed);
    spec.duration_s = 10.0;
    spec.agents = ring(3, 50.0);
    for a in &mut spec.agents {
        a.sensor.max_delay_ms = 400;
    }
    spec.objects = separated_objects(seed, 4, 30.0, 5.0, spec.duration_s, 12.0);
    let rec = run(&spec);
    let mut c = Checker::new("time_warp", seed);
    let stale: usize = rec.tracker_rejections.iter().map(|r| r.0).sum();
    c.check(stale == 0, || {
        format!("{stale} in-window delayed detections rejected as stale")
    });
    c.tracked_world(&rec, &[0, 1, 2], 2.5, f64::INFINITY, 0.6, globally_observed);
    c.converged(&rec);
    c.finish(
        Some(&rec),
        TIME_WARP,
        vec![(
            "mean_rms_m",
            format!("{:.3}", mean_rms(&rec, &[0, 1, 2], 2.5)),
        )],
    )
}

const TIME_TORNADO: &str =
    "Detections delayed by up to 5 s mostly fall outside the retrodiction window: they are \
rejected rather than applied to the wrong time, stale data never creates ghost objects (estimates no real object \
explains within their own 99.99% uncertainty), and the reported \
covariance covers the larger errors of rarely updated tracks (pooled NEES below 6, every error under 3 m).";

fn time_tornado(seed: u64) -> ScenarioOutcome {
    let mut spec = ScenarioSpec::new("time_tornado", seed);
    spec.duration_s = 15.0;
    spec.agents = ring(3, 50.0);
    for a in &mut spec.agents {
        a.sensor.max_delay_ms = 5000;
    }
    spec.objects = separated_objects(seed, 3, 25.0, 3.0, spec.duration_s, 15.0);
    let rec = run(&spec);
    let mut c = Checker::new("time_tornado", seed);
    let stale: usize = rec.tracker_rejections.iter().map(|r| r.0).sum();
    let total: u64 = rec.detections.iter().sum();
    c.check(stale as f64 > 0.3 * total as f64, || {
        format!("only {stale}/{total} detections rejected as stale; delays not exercised")
    });
    let mut nees = Vec::new();
    for s in rec.samples.iter().filter(|s| s.t_s >= 5.0) {
        for a in 0..3 {
            let sc = rec.score(s, a, CUTOFF_M, |_| true);
            // A ghost is an estimate that no real object explains, given the
            // estimate's own uncertainty: beyond the 5 m cutoff and outside its
            // 99.99% position ellipsoid around every truth. A just-confirmed
            // track of a real object can be a few meters off with an honest
            // multi-meter covariance (seen at seed 48: 5.35 m off with about
            // 4 m of uncertainty, 0.29 m off a tenth of a second later); it is
            // not a ghost. The ghost this check exists for drifted to 22 m off
            // while claiming about 1.2 m.
            let objects = &s.agents[a].world.objects;
            let ghosts: Vec<String> = sc
                .unmatched_estimates
                .iter()
                .filter_map(|&e| {
                    let o = &objects[e];
                    let p: nalgebra::Matrix3<f64> = o.cov.fixed_view::<3, 3>(0, 0).into_owned();
                    let chol = p.cholesky()?;
                    let explained = s.truths.iter().any(|t| {
                        let d = o.position().0 - t.position.0;
                        d.dot(&chol.solve(&d)) < 21.11
                    });
                    let nearest = s
                        .truths
                        .iter()
                        .map(|t| t.position.distance(&o.position()))
                        .fold(f64::INFINITY, f64::min);
                    (!explained).then(|| {
                        format!(
                            "{nearest:.1} m from the nearest object, sigma {:.1} m",
                            p.trace().sqrt()
                        )
                    })
                })
                .collect();
            c.check(ghosts.is_empty(), || {
                format!(
                    "t={:.2}s agent {a}: {} ghost objects from stale data ({})",
                    s.t_s,
                    ghosts.len(),
                    ghosts.join("; ")
                )
            });
            // Only about a tenth of the detections arrive inside the retrodiction
            // window, so tracks are updated at roughly 1 Hz and errors of 1-2 m
            // are expected between updates. What must hold is that the reported
            // covariance covers the error (pooled NEES below), and that no
            // estimate is grossly wrong.
            c.check(sc.matched == 0 || sc.rms_matched < 3.0, || {
                format!("t={:.2}s agent {a}: rms {:.2} m", s.t_s, sc.rms_matched)
            });
            nees.extend(sc.nees);
        }
    }
    let pooled = nees.iter().sum::<f64>() / nees.len().max(1) as f64;
    c.check(!nees.is_empty() && pooled < 6.0, || {
        format!(
            "pooled NEES {pooled:.2} over {} estimates: covariance does not cover the error",
            nees.len()
        )
    });
    c.converged(&rec);
    c.finish(
        Some(&rec),
        TIME_TORNADO,
        vec![
            ("stale_rejected", format!("{stale}/{total}")),
            ("pooled_nees", format!("{pooled:.2} (n={})", nees.len())),
        ],
    )
}
