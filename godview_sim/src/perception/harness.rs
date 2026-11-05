//! The perception DST harness: runs a [`ScenarioSpec`] deterministically and
//! records what every agent believed, for scenario checks to judge.
//!
//! What it models, and how this differs from the legacy runner:
//!
//! * **Independent sensing.** Each agent draws its own measurement noise
//!   from its own seeded stream, only for objects within its sensor range, and
//!   receives position-only detections with no ground-truth id. (The legacy
//!   runner cloned one noisy reading per object to every agent, with the true
//!   velocity and the oracle id, which made association trivial and hid
//!   correlated errors; audit findings F-26, F-27.)
//! * **Frames and localization.** Each agent has its own odometry frame
//!   (rotated and offset from the site ENU frame). Its localization estimate
//!   of `ECEF ← odometry` carries an error drawn once from the covariance it
//!   reports, plus an optional unmodeled bias that it does not report.
//!   Outage windows make the agent publish in its local frame.
//! * **Real bytes over a faulty network.** Agents exchange signed report
//!   batches through [`SimBus`] (latency, jitter, loss, duplication,
//!   corruption, partitions), optionally relaying what they receive.
//! * **Adversaries.** Replays of captured batches, forged batches signed by a
//!   non-agent key, fabricating agents, and revocation.
//!
//! Every random draw comes from a stream derived from the scenario seed, and
//! all iteration is over ordered collections, so `(scenario, seed)` fully
//! reproduces a run. [`RunRecord::digest`] summarizes a run for that check.

use super::bus::{BusStats, LinkFaults, SimBus, Topology};
use super::metrics::{score, Score};
use crate::context::SimContext;
use crate::keys::DeterministicKeyProvider;
use crate::oracle::Oracle;
use godview_core::consistency::SourceStatus;
use godview_core::extent::{wrap_pi, ObjectBox};
use godview_core::frames::{rotate_state_cov, Ecef, EnuFrame, Geodetic, Pose3, PoseWithCov};
use godview_core::local_tracker::Detection;
use godview_core::localization::{LocalizationStatus, PoseEstimate};
use godview_core::perception_node::{NodeConfig, NodeStats, PerceptionNode};
use godview_core::report::{
    quantize_covariance, AgentId, FrameRef, LocalTrackId, ReportBatch, TrackLifecycle, TrackReport,
};
use godview_core::report_store::ReportStore;
use godview_core::world_model::{TrustView, WorldModel, WorldSnapshot};
use godview_env::{GodViewContext, Timestamp};
use nalgebra::{Matrix3, Matrix6, Vector3, Vector6};
use rand::{Rng, SeedableRng};
use rand_chacha::ChaCha8Rng;
use std::collections::{BTreeMap, BTreeSet, VecDeque};
use std::time::Duration;

/// How a sensor reports object extent (oriented boxes).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BoxSensorSpec {
    /// Size noise, each of length, width, height (m).
    pub sigma_size_m: f64,
    /// Heading noise (rad).
    pub sigma_yaw_rad: f64,
    /// The detector cannot tell front from back: headings are reported
    /// modulo π, randomly flipped.
    pub ambiguous: bool,
    /// Partial view: the measured center moves toward the sensor along the
    /// object's length by a uniform amount in `[0, partial_view · length / 2]`
    /// when seen end-on (the far end is not seen), scaled by the cosine
    /// between the length axis and the direction to the sensor, so a
    /// broadside view has no shift. Half of that range is a systematic offset
    /// that repeats every scan; the detection's covariance carries only the
    /// per-scan spread (variance `a² / 12` for range `a`), and the systematic
    /// part must be declared through the tracker's `box_center_sys_frac`
    /// (`partial_view / 2` matches it exactly).
    pub partial_view: f64,
    /// Systematic size error (m): `size_bias_m · g` is added to each
    /// dimension, with `g ~ N(0, 1)` drawn once per (agent, object, dimension)
    /// and kept for the whole run, like an annotator or detector that always
    /// gets one vehicle's size wrong the same way. It is not in the
    /// detection's variances; declare it through the tracker's
    /// `box_size_sys_m` (equal values match exactly).
    pub size_bias_m: f64,
    /// Systematic heading error (rad), drawn the same way; declare it through
    /// the tracker's `box_yaw_sys_rad`.
    pub yaw_bias_rad: f64,
}

impl Default for BoxSensorSpec {
    fn default() -> Self {
        Self {
            sigma_size_m: 0.2,
            sigma_yaw_rad: 0.05,
            ambiguous: true,
            partial_view: 0.0,
            size_bias_m: 0.0,
            yaw_bias_rad: 0.0,
        }
    }
}

/// The fixed bias draws `[g_l, g_w, g_h, g_yaw]` of one (agent, object) pair.
/// Seeded from the pair alone, so they do not disturb any other noise stream.
fn bias_draws(seed: u64, agent: usize, object: u64) -> [f64; 4] {
    let mut rng = ChaCha8Rng::seed_from_u64(
        seed ^ 0xb1a5_0000_0000_0000 ^ ((agent as u64) << 40) ^ object.wrapping_mul(0x9e37_79b9),
    );
    [0; 4].map(|_| randn(&mut rng))
}

/// Sensor model of an agent.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SensorSpec {
    /// Detection range from the agent's position (m).
    pub range_m: f64,
    /// Isotropic position noise standard deviation (m).
    pub sigma_m: f64,
    /// Probability each in-range object is detected per scan.
    pub p_detect: f64,
    /// Detections reach the tracker after a uniform delay in `[min, max]` (ms).
    pub min_delay_ms: u64,
    /// See `min_delay_ms`.
    pub max_delay_ms: u64,
    /// Report oriented boxes for objects that have a size (`None`: points only).
    pub boxes: Option<BoxSensorSpec>,
}

impl Default for SensorSpec {
    fn default() -> Self {
        Self {
            range_m: 150.0,
            sigma_m: 0.3,
            p_detect: 1.0,
            min_delay_ms: 0,
            max_delay_ms: 0,
            boxes: None,
        }
    }
}

/// Localization error model of an agent.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct LocalizationSpec {
    /// Horizontal position standard deviation (m), reported and realized.
    pub sigma_h_m: f64,
    /// Vertical position standard deviation (m).
    pub sigma_v_m: f64,
    /// Yaw standard deviation (rad).
    pub sigma_yaw_rad: f64,
    /// Roll/pitch standard deviation (rad).
    pub sigma_tilt_rad: f64,
    /// Error added to the estimate but NOT reported in its covariance (ENU, m).
    pub unmodeled_bias_enu: Vector3<f64>,
    /// Windows `[start, end)` in seconds with no global localization.
    pub outages: Vec<(f64, f64)>,
}

/// How an agent behaves.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Behavior {
    /// Tracks with its sensors and publishes honestly.
    Honest,
    /// Publishes validly signed reports of `count` fabricated objects.
    Fabricator {
        /// Number of fake objects.
        count: usize,
    },
}

/// One simulated agent.
#[derive(Debug, Clone, PartialEq)]
pub struct AgentSpec {
    /// Starting position in the site ENU frame (m).
    pub position_enu: Vector3<f64>,
    /// Constant velocity in ENU (m/s).
    pub velocity_enu: Vector3<f64>,
    /// Yaw of the agent's odometry frame relative to ENU (rad).
    pub odom_yaw_rad: f64,
    /// Odometry origin relative to the starting position (ENU, m).
    pub odom_offset_enu: Vector3<f64>,
    /// Sensor model.
    pub sensor: SensorSpec,
    /// Localization model.
    pub localization: LocalizationSpec,
    /// Publish period (ms).
    pub publish_period_ms: u64,
    /// Forward accepted peer batches to neighbors (multi-hop gossip).
    pub relay: bool,
    /// Honest or adversarial.
    pub behavior: Behavior,
    /// Restart (lose local state, bump epoch) at this time (s).
    pub restart_at_s: Option<f64>,
}

impl AgentSpec {
    /// An honest agent at `position_enu` with default sensing.
    pub fn at(position_enu: Vector3<f64>) -> Self {
        Self {
            position_enu,
            velocity_enu: Vector3::zeros(),
            odom_yaw_rad: 0.0,
            odom_offset_enu: Vector3::zeros(),
            sensor: SensorSpec::default(),
            localization: LocalizationSpec::default(),
            publish_period_ms: 100,
            relay: false,
            behavior: Behavior::Honest,
            restart_at_s: None,
        }
    }
}

/// One physical object.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ObjectSpec {
    /// Position at spawn time (ENU, m).
    pub position_enu: Vector3<f64>,
    /// Constant velocity (ENU, m/s).
    pub velocity_enu: Vector3<f64>,
    /// Class id.
    pub class_id: u8,
    /// Spawn time (s).
    pub spawn_s: f64,
    /// Removal time (s), if any.
    pub remove_s: Option<f64>,
    /// Length, width, height (m), if the object has a box. Its heading is the
    /// direction of travel (East if stationary).
    pub size: Option<Vector3<f64>>,
}

impl ObjectSpec {
    /// An object present from the start.
    pub fn moving(position_enu: Vector3<f64>, velocity_enu: Vector3<f64>) -> Self {
        Self {
            position_enu,
            velocity_enu,
            class_id: 1,
            spawn_s: 0.0,
            remove_s: None,
            size: None,
        }
    }

    /// The same object with a box of the given length, width and height.
    pub fn with_size(mut self, length: f64, width: f64, height: f64) -> Self {
        self.size = Some(Vector3::new(length, width, height));
        self
    }

    /// Heading of the object's length axis in ENU (rad).
    pub fn heading_enu(&self) -> f64 {
        let v = self.velocity_enu;
        if v.x.hypot(v.y) > 1e-9 {
            v.y.atan2(v.x)
        } else {
            0.0
        }
    }
}

/// A network partition window.
#[derive(Debug, Clone, PartialEq)]
pub struct PartitionSpec {
    /// Start (s).
    pub start_s: f64,
    /// End (s).
    pub end_s: f64,
    /// One side.
    pub group_a: Vec<usize>,
    /// Other side.
    pub group_b: Vec<usize>,
}

/// Adversarial actions.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct AdversarySpec {
    /// Re-send agent 0's earliest captured batches to every agent at these times (s).
    pub replay_at_s: Vec<f64>,
    /// Inject a batch signed by a non-agent key but claiming to be agent 0, at these times (s).
    pub forge_at_s: Vec<f64>,
    /// Every other agent revokes agent `.0` at time `.1` (s).
    pub revoke: Vec<(usize, f64)>,
}

/// A complete, reproducible scenario.
#[derive(Debug, Clone)]
pub struct ScenarioSpec {
    /// Scenario identifier.
    pub name: &'static str,
    /// Master seed.
    pub seed: u64,
    /// Site anchor.
    pub site: Geodetic,
    /// Simulated duration (s).
    pub duration_s: f64,
    /// Simulation step (ms).
    pub tick_ms: u64,
    /// Sampling period for the record (ms).
    pub sample_every_ms: u64,
    /// No samples before this time (s).
    pub warmup_s: f64,
    /// Agents.
    pub agents: Vec<AgentSpec>,
    /// Objects.
    pub objects: Vec<ObjectSpec>,
    /// Topology.
    pub topology: Topology,
    /// Default link faults.
    pub faults: LinkFaults,
    /// Partition windows.
    pub partitions: Vec<PartitionSpec>,
    /// Adversary.
    pub adversary: AdversarySpec,
    /// Node configuration for every agent.
    pub node: NodeConfig,
    /// After the run, heal the network, publish once more reliably and drain
    /// the bus, then record each agent's trust-neutral world at the end time.
    pub final_sync: bool,
    /// Time after spawn before an object is expected in the world state (s).
    pub confirm_latency_s: f64,
}

impl ScenarioSpec {
    /// A spec with defaults: San Francisco site, 50 ms ticks, 500 ms samples.
    pub fn new(name: &'static str, seed: u64) -> Self {
        Self {
            name,
            seed,
            site: Geodetic::new(37.7749, -122.4194, 10.0).expect("valid site"),
            duration_s: 10.0,
            tick_ms: 50,
            sample_every_ms: 500,
            warmup_s: 1.5,
            agents: Vec::new(),
            objects: Vec::new(),
            topology: Topology::FullMesh,
            faults: LinkFaults::default(),
            partitions: Vec::new(),
            adversary: AdversarySpec::default(),
            node: NodeConfig::default(),
            final_sync: true,
            confirm_latency_s: 0.8,
        }
    }
}

/// Ground truth of one object at a sample.
#[derive(Debug, Clone, PartialEq)]
pub struct TruthSample {
    /// Oracle id.
    pub id: u64,
    /// Position in ECEF.
    pub position: Ecef,
    /// Honest agents that can currently see it (in range, past confirmation latency).
    pub observers: BTreeSet<usize>,
    /// Of those, observers with a global pose at this time.
    pub global_observers: BTreeSet<usize>,
    /// True box `(length, width, height, heading in ENU)`, if the object has one.
    pub object_box: Option<([f64; 3], f64)>,
}

/// One agent's state at a sample.
#[derive(Debug, Clone, PartialEq)]
pub struct AgentSample {
    /// World state with this agent's trust view applied.
    pub world: WorldSnapshot,
    /// Digest of the trust-neutral world state.
    pub neutral_digest: u64,
    /// This agent's judgment of each other agent.
    pub statuses: BTreeMap<usize, SourceStatus>,
}

/// Everything recorded at one sample time.
#[derive(Debug, Clone, PartialEq)]
pub struct Sample {
    /// Time.
    pub t: Timestamp,
    /// Seconds since scenario start.
    pub t_s: f64,
    /// Ground truth.
    pub truths: Vec<TruthSample>,
    /// Per agent.
    pub agents: Vec<AgentSample>,
}

/// A published report of an agent's own track compared with truth.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ReportCheck {
    /// Publishing agent.
    pub agent: usize,
    /// Time.
    pub t_s: f64,
    /// Range from the agent to the matched truth (m).
    pub range_m: f64,
    /// NEES of the report position against truth.
    pub nees: f64,
    /// Trace of the report's position covariance (m²).
    pub pos_trace: f64,
    /// Position error (m).
    pub error_m: f64,
}

/// What a scenario run produced.
#[derive(Debug, Clone)]
pub struct RunRecord {
    /// Scenario name.
    pub name: &'static str,
    /// Seed.
    pub seed: u64,
    /// Agent ids by index.
    pub ids: Vec<AgentId>,
    /// Samples in time order.
    pub samples: Vec<Sample>,
    /// End time.
    pub end: Timestamp,
    /// Trust-neutral world per agent after the final synchronization round.
    pub final_worlds: Vec<WorldSnapshot>,
    /// World computed from every source's own latest reports only.
    pub source_only_world: Option<WorldSnapshot>,
    /// Node counters.
    pub node_stats: Vec<NodeStats>,
    /// Detections the trackers rejected as stale / duplicate, per agent.
    pub tracker_rejections: Vec<(usize, usize)>,
    /// Detections generated per agent.
    pub detections: Vec<u64>,
    /// Bus counters.
    pub bus: BusStats,
    /// Published-report accuracy checks (honest agents, global frame).
    pub report_checks: Vec<ReportCheck>,
    /// Batches published in the local frame per agent, with the time window.
    pub local_frame_publish_times: Vec<Vec<f64>>,
    /// Global-frame publish times per agent.
    pub global_frame_publish_times: Vec<Vec<f64>>,
}

impl RunRecord {
    /// Deterministic digest of the run (for reproducibility checks).
    pub fn digest(&self) -> u64 {
        let mut h: u64 = 0xcbf2_9ce4_8422_2325;
        let mut mix = |v: u64| {
            for b in v.to_le_bytes() {
                h ^= b as u64;
                h = h.wrapping_mul(0x0000_0100_0000_01b3);
            }
        };
        for s in &self.samples {
            mix(s.t.as_nanos() as u64);
            for a in &s.agents {
                mix(a.neutral_digest);
                mix(a.world.digest());
            }
        }
        for w in &self.final_worlds {
            mix(w.digest());
        }
        mix(self.bus.delivered);
        mix(self.bus.lost);
        h
    }

    /// Scores agent `a`'s world at sample `s` against the truths selected by `expected`.
    pub fn score(
        &self,
        s: &Sample,
        a: usize,
        cutoff_m: f64,
        expected: impl Fn(&TruthSample) -> bool,
    ) -> Score {
        let est: Vec<(Ecef, Option<Matrix3<f64>>)> = s.agents[a]
            .world
            .objects
            .iter()
            .map(|o| {
                (
                    o.position(),
                    Some(o.cov.fixed_view::<3, 3>(0, 0).into_owned()),
                )
            })
            .collect();
        let truths: Vec<Ecef> = s
            .truths
            .iter()
            .filter(|t| expected(t))
            .map(|t| t.position)
            .collect();
        score(&est, &truths, cutoff_m)
    }
}

fn stream(seed: u64, stream: u64, index: u64) -> ChaCha8Rng {
    // SplitMix64 finalizer over the combined key: independent, reproducible streams.
    let mut z = seed
        ^ stream.wrapping_mul(0x9e37_79b9_7f4a_7c15)
        ^ index.wrapping_mul(0xbf58_476d_1ce4_e5b9);
    z = (z ^ (z >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
    ChaCha8Rng::seed_from_u64(z ^ (z >> 31))
}

fn randn(rng: &mut impl Rng) -> f64 {
    let u1: f64 = rng.gen_range(1e-12..1.0);
    let u2: f64 = rng.gen();
    (-2.0 * u1.ln()).sqrt() * (2.0 * std::f64::consts::PI * u2).cos()
}

struct SimAgent {
    spec: AgentSpec,
    key: ed25519_dalek::SigningKey,
    node: PerceptionNode,
    /// ENU ← odometry (exact).
    enu_from_odom: Pose3,
    /// The agent's estimate of ECEF ← odometry (with its realized error).
    est_link: PoseWithCov,
    sensor_rng: ChaCha8Rng,
    delay_rng: ChaCha8Rng,
    fab_rng: ChaCha8Rng,
    pending: VecDeque<(i64, Detection)>,
    next_obs: u64,
    relay_seen: BTreeSet<u64>,
    restarted: bool,
    fab_seq: u64,
    stale: usize,
    duplicate: usize,
}

impl SimAgent {
    fn position_enu(&self, t_s: f64) -> Vector3<f64> {
        self.spec.position_enu + self.spec.velocity_enu * t_s
    }

    fn in_outage(&self, t_s: f64) -> bool {
        self.spec
            .localization
            .outages
            .iter()
            .any(|(a, b)| t_s >= *a && t_s < *b)
    }

    fn pose(&self, t: Timestamp, t_s: f64) -> PoseEstimate {
        PoseEstimate {
            t,
            ecef_from_odom: self.est_link,
            status: if self.in_outage(t_s) {
                LocalizationStatus::LocalOnly
            } else {
                LocalizationStatus::Global
            },
        }
    }
}

fn fnv(bytes: &[u8]) -> u64 {
    let mut h: u64 = 0xcbf2_9ce4_8422_2325;
    for b in bytes {
        h ^= *b as u64;
        h = h.wrapping_mul(0x0000_0100_0000_01b3);
    }
    h
}

/// Runs a scenario and records the outcome.
pub fn run(spec: &ScenarioSpec) -> RunRecord {
    let site = EnuFrame::new(spec.site);
    let ecef_from_enu = site.ecef_from_enu_pose();
    let r_ecef_enu = site.ecef_from_enu();
    let mut keys = DeterministicKeyProvider::new(spec.seed);
    let ctx = SimContext::new(spec.seed);
    let t0 = ctx.timestamp();

    let mut agents: Vec<SimAgent> = spec
        .agents
        .iter()
        .enumerate()
        .map(|(i, a)| {
            let i64_ = i as u64;
            let enu_from_odom = Pose3::from_yaw(a.odom_yaw_rad, a.position_enu + a.odom_offset_enu);
            let true_link = ecef_from_enu.compose(&enu_from_odom);
            let l = &a.localization;
            let mut cov_enu = Matrix6::zeros();
            cov_enu[(0, 0)] = l.sigma_h_m.powi(2);
            cov_enu[(1, 1)] = l.sigma_h_m.powi(2);
            cov_enu[(2, 2)] = l.sigma_v_m.powi(2);
            cov_enu[(3, 3)] = l.sigma_tilt_rad.powi(2);
            cov_enu[(4, 4)] = l.sigma_tilt_rad.powi(2);
            cov_enu[(5, 5)] = l.sigma_yaw_rad.powi(2);
            let cov = rotate_state_cov(&r_ecef_enu, &cov_enu);
            let mut loc_rng = stream(spec.seed, 3, i64_);
            let delta_enu = Vector6::from_fn(|k, _| randn(&mut loc_rng) * cov_enu[(k, k)].sqrt());
            let mut delta = Vector6::zeros();
            delta
                .fixed_rows_mut::<3>(0)
                .copy_from(&(r_ecef_enu * delta_enu.fixed_rows::<3>(0)));
            delta
                .fixed_rows_mut::<3>(3)
                .copy_from(&(r_ecef_enu * delta_enu.fixed_rows::<3>(3)));
            // The estimate is the truth moved by -δ, so truth = estimate ⊕ δ with δ ~ N(0, cov).
            let mut est = true_link.perturbed(&(-delta));
            est.translation += r_ecef_enu * l.unmodeled_bias_enu;
            let key = keys.agent_key(i64_);
            SimAgent {
                spec: a.clone(),
                node: PerceptionNode::new(key.clone(), 1, spec.node),
                key,
                enu_from_odom,
                est_link: PoseWithCov { pose: est, cov },
                sensor_rng: stream(spec.seed, 1, i64_),
                delay_rng: stream(spec.seed, 2, i64_),
                fab_rng: stream(spec.seed, 4, i64_),
                pending: VecDeque::new(),
                next_obs: 0,
                relay_seen: BTreeSet::new(),
                restarted: false,
                fab_seq: 0,
                stale: 0,
                duplicate: 0,
            }
        })
        .collect();
    let ids: Vec<AgentId> = agents.iter().map(|a| a.node.id()).collect();
    let index_of: BTreeMap<AgentId, usize> =
        ids.iter().enumerate().map(|(i, id)| (*id, i)).collect();
    let attacker = keys.agent_key(1_000_000);

    let mut bus = SimBus::new(
        agents.len(),
        spec.topology.clone(),
        spec.faults,
        stream(spec.seed, 5, 0).gen(),
    );
    let mut oracle = Oracle::new(spec.seed.wrapping_mul(0x9e37_79b9_7f4a_7c15));
    oracle.set_position_noise(0.0);
    let mut spawned: BTreeMap<usize, u64> = BTreeMap::new(); // spec index -> oracle id
    let mut class_of: BTreeMap<u64, u8> = BTreeMap::new();
    let mut box_of: BTreeMap<u64, ([f64; 3], f64)> = BTreeMap::new();
    let mut spawn_time: BTreeMap<u64, f64> = BTreeMap::new();

    let mut record = RunRecord {
        name: spec.name,
        seed: spec.seed,
        ids: ids.clone(),
        samples: Vec::new(),
        end: t0,
        final_worlds: Vec::new(),
        source_only_world: None,
        node_stats: Vec::new(),
        tracker_rejections: Vec::new(),
        detections: Vec::new(),
        bus: BusStats::default(),
        report_checks: Vec::new(),
        local_frame_publish_times: vec![Vec::new(); agents.len()],
        global_frame_publish_times: vec![Vec::new(); agents.len()],
    };
    let mut captured: Vec<Vec<u8>> = Vec::new();
    let mut partitioned: BTreeSet<usize> = BTreeSet::new();
    let mut revoked: BTreeSet<usize> = BTreeSet::new();

    let ticks = (spec.duration_s * 1000.0 / spec.tick_ms as f64).round() as u64;
    let dt = spec.tick_ms as f64 / 1000.0;
    for k in 0..=ticks {
        if k > 0 {
            ctx.advance_time(Duration::from_millis(spec.tick_ms));
            oracle.step(dt);
        }
        let t = ctx.timestamp();
        let t_s = t.secs_since(t0);
        let ms = k * spec.tick_ms;

        // --- scheduled world events -------------------------------------------
        for (i, o) in spec.objects.iter().enumerate() {
            if !spawned.contains_key(&i) && t_s + 1e-9 >= o.spawn_s {
                let id = oracle.spawn_entity(o.position_enu, o.velocity_enu, "object");
                spawned.insert(i, id);
                class_of.insert(id, o.class_id);
                if let Some(sz) = o.size {
                    box_of.insert(id, ([sz.x, sz.y, sz.z], o.heading_enu()));
                }
                spawn_time.insert(id, t_s);
            }
            if let (Some(r), Some(id)) = (o.remove_s, spawned.get(&i)) {
                if t_s + 1e-9 >= r {
                    oracle.remove_entity(*id);
                }
            }
        }
        for (pi, p) in spec.partitions.iter().enumerate() {
            if t_s + 1e-9 >= p.start_s && t_s < p.end_s && partitioned.insert(pi) {
                let a: Vec<_> = p.group_a.iter().map(|i| bus.node(*i)).collect();
                let b: Vec<_> = p.group_b.iter().map(|i| bus.node(*i)).collect();
                bus.controller().partition(a, b);
            }
            if t_s + 1e-9 >= p.end_s && partitioned.remove(&pi) {
                // Rebuild remaining partitions (the controller can only heal all).
                bus.controller().heal_all();
                for pj in &partitioned {
                    let q = &spec.partitions[*pj];
                    let a: Vec<_> = q.group_a.iter().map(|i| bus.node(*i)).collect();
                    let b: Vec<_> = q.group_b.iter().map(|i| bus.node(*i)).collect();
                    bus.controller().partition(a, b);
                }
            }
        }
        for (ai, at) in &spec.adversary.revoke {
            if t_s + 1e-9 >= *at && revoked.insert(*ai) {
                let victim = ids[*ai];
                for (j, ag) in agents.iter_mut().enumerate() {
                    if j != *ai {
                        ag.node.revoke(victim);
                    }
                }
            }
        }
        for ag in agents.iter_mut() {
            if let Some(r) = ag.spec.restart_at_s {
                if !ag.restarted && t_s + 1e-9 >= r {
                    ag.restarted = true;
                    let e = ag.node.epoch() + 1;
                    ag.node.restart(e);
                    ag.pending.clear();
                }
            }
        }

        // --- sensing ------------------------------------------------------------
        let truth_now: Vec<(u64, Vector3<f64>)> = oracle
            .active_entities()
            .iter()
            .map(|e| (e.id, e.position))
            .collect();
        for (ai, ag) in agents.iter_mut().enumerate() {
            if ag.spec.behavior != Behavior::Honest {
                continue;
            }
            let here = ag.position_enu(t_s);
            let odom_from_enu = ag.enu_from_odom.inverse();
            for (id, p) in &truth_now {
                if (p - here).norm() > ag.spec.sensor.range_m {
                    continue;
                }
                // Draw unconditionally so detection probability does not shift the noise stream.
                let u: f64 = ag.sensor_rng.gen();
                let n = Vector3::new(
                    randn(&mut ag.sensor_rng),
                    randn(&mut ag.sensor_rng),
                    randn(&mut ag.sensor_rng),
                ) * ag.spec.sensor.sigma_m;
                if u >= ag.spec.sensor.p_detect {
                    continue;
                }
                // Box measurement. Random numbers are drawn only when the sensor
                // reports boxes, so point-only scenarios keep their noise streams.
                let mut shift = Vector3::zeros();
                let mut r_enu = Matrix3::identity() * ag.spec.sensor.sigma_m.powi(2);
                let mut object_box = None;
                if let (Some(bs), Some((sz, heading))) = (ag.spec.sensor.boxes, box_of.get(id)) {
                    let ns = [
                        randn(&mut ag.sensor_rng),
                        randn(&mut ag.sensor_rng),
                        randn(&mut ag.sensor_rng),
                    ];
                    let ny = randn(&mut ag.sensor_rng);
                    let flip: f64 = ag.sensor_rng.gen();
                    let part: f64 = ag.sensor_rng.gen();
                    let axis = Vector3::new(heading.cos(), heading.sin(), 0.0);
                    if bs.partial_view > 0.0 {
                        // Seen end-on, the far end is hidden and the center moves
                        // toward the sensor; seen broadside, the whole side is
                        // visible. The shift scales with the cosine between the
                        // length axis and the direction to the sensor.
                        let view = (here - p).normalize().dot(&axis);
                        let a = bs.partial_view * sz[0] / 2.0 * view;
                        shift = axis * (part * a);
                        r_enu += axis * axis.transpose() * (a * a / 12.0);
                    }
                    let bias = if bs.size_bias_m > 0.0 || bs.yaw_bias_rad > 0.0 {
                        bias_draws(spec.seed, ai, *id)
                    } else {
                        [0.0; 4]
                    };
                    let sz = [0, 1, 2].map(|i| (sz[i] + bs.size_bias_m * bias[i]).max(0.2));
                    let mut yaw = heading + bias[3] * bs.yaw_bias_rad + ny * bs.sigma_yaw_rad
                        - ag.spec.odom_yaw_rad;
                    if bs.ambiguous && flip < 0.5 {
                        yaw += std::f64::consts::PI;
                    }
                    let v = bs.sigma_size_m * bs.sigma_size_m;
                    object_box = Some(ObjectBox {
                        size: [0, 1, 2].map(|i| (sz[i] + ns[i] * bs.sigma_size_m).max(0.1)),
                        size_var: [v; 3],
                        yaw: wrap_pi(yaw),
                        yaw_var: bs.sigma_yaw_rad * bs.sigma_yaw_rad,
                        yaw_ambiguous: bs.ambiguous,
                    });
                }
                ag.next_obs += 1;
                let s = &ag.spec.sensor;
                let delay = if s.max_delay_ms > s.min_delay_ms {
                    ag.delay_rng.gen_range(s.min_delay_ms..=s.max_delay_ms)
                } else {
                    s.min_delay_ms
                };
                let det = Detection {
                    obs_id: ag.next_obs,
                    t,
                    // Noise is drawn in ENU and rotated with the point, so a rotated
                    // odometry frame sees the same physical error.
                    z: odom_from_enu.transform_point(&(p + n + shift)),
                    r: odom_from_enu.rotation.matrix()
                        * r_enu
                        * odom_from_enu.rotation.matrix().transpose(),
                    class_id: class_of.get(id).copied().unwrap_or(1),
                    class_prob: 0.9,
                    object_box,
                };
                ag.pending
                    .push_back((t.add_millis(delay as i64).as_nanos(), det));
            }
            // Deliver due detections to the tracker (possibly late and out of order).
            let mut due: Vec<Detection> = Vec::new();
            let mut keep = VecDeque::new();
            while let Some((at, d)) = ag.pending.pop_front() {
                if at <= t.as_nanos() {
                    due.push(d);
                } else {
                    keep.push_back((at, d));
                }
            }
            ag.pending = keep;
            if !due.is_empty() {
                let out = ag.node.ingest(&due);
                ag.stale += out.stale;
                ag.duplicate += out.duplicate;
            }
            ag.node.maintain(t);
        }

        // --- publishing -----------------------------------------------------------
        for (i, ag) in agents.iter_mut().enumerate() {
            let period = ag.spec.publish_period_ms.max(spec.tick_ms);
            if !ms.is_multiple_of(period) {
                continue;
            }
            let bytes = match ag.spec.behavior {
                Behavior::Honest => {
                    let pose = ag.pose(t, t_s);
                    let out = ag.node.publish(t, Some(&pose));
                    if out.is_some() {
                        if pose.status == LocalizationStatus::Global {
                            record.global_frame_publish_times[i].push(t_s);
                        } else {
                            record.local_frame_publish_times[i].push(t_s);
                        }
                    }
                    out
                }
                Behavior::Fabricator { count } => Some(fabricate(ag, count, &site, t)),
            };
            let Some(bytes) = bytes else { continue };
            if i == 0 && captured.len() < 3 {
                captured.push(bytes.clone());
            }
            if ag.spec.behavior == Behavior::Honest
                && ag.pose(t, t_s).status == LocalizationStatus::Global
            {
                check_reports(
                    &bytes,
                    i,
                    t_s,
                    &*ag,
                    &truth_now,
                    &site,
                    &mut record.report_checks,
                );
            }
            bus.broadcast(i, &bytes, t, &BTreeSet::new());
        }

        // --- adversary --------------------------------------------------------------
        for at in &spec.adversary.replay_at_s {
            if (t_s - at).abs() < dt / 2.0 {
                for bytes in &captured {
                    for to in 1..agents.len() {
                        bus.inject(0, to, bytes.clone(), t);
                    }
                }
            }
        }
        for at in &spec.adversary.forge_at_s {
            if (t_s - at).abs() < dt / 2.0 && !captured.is_empty() {
                // Re-sign a captured batch with the attacker's key, then claim agent 0 as source.
                if let Ok(mut b) = ReportBatch::verify_and_decode(&captured[0]) {
                    let forger = AgentId::from_signing_key(&attacker);
                    b.source = forger;
                    for r in &mut b.reports {
                        r.source = forger;
                        r.seq += 1_000_000;
                        r.pos[0] += 50.0;
                    }
                    let mut forged = b.sign(&attacker);
                    forged[4..36].copy_from_slice(&ids[0].0);
                    for to in 1..agents.len() {
                        bus.inject(0, to, forged.clone(), t);
                    }
                }
            }
        }

        // --- delivery and relaying ---------------------------------------------------
        deliver(&mut bus, &mut agents, t);

        // --- sampling ----------------------------------------------------------------
        let is_sample = ms.is_multiple_of(spec.sample_every_ms) && t_s + 1e-9 >= spec.warmup_s;
        let is_publish_tick = ms.is_multiple_of(100);
        if is_publish_tick || is_sample {
            let mut agent_samples = Vec::new();
            for ag in agents.iter_mut() {
                // Feed the consistency monitor at 10 Hz even between samples.
                let world = ag.node.world(t);
                if is_sample {
                    let statuses = ag
                        .node
                        .monitor()
                        .statuses()
                        .into_iter()
                        .filter_map(|(id, s)| index_of.get(&id).map(|i| (*i, s)))
                        .collect();
                    agent_samples.push(AgentSample {
                        neutral_digest: ag.node.world_neutral(t).digest(),
                        world,
                        statuses,
                    });
                }
            }
            if is_sample {
                let truths = truth_now
                    .iter()
                    .map(|(id, p)| {
                        let age = t_s - spawn_time.get(id).copied().unwrap_or(0.0);
                        let mut observers = BTreeSet::new();
                        let mut global_observers = BTreeSet::new();
                        for (j, ag) in agents.iter().enumerate() {
                            if ag.spec.behavior == Behavior::Honest
                                && age >= spec.confirm_latency_s
                                && (p - ag.position_enu(t_s)).norm() <= ag.spec.sensor.range_m
                                && ag.spec.sensor.p_detect > 0.0
                            {
                                observers.insert(j);
                                if !ag.in_outage(t_s) {
                                    global_observers.insert(j);
                                }
                            }
                        }
                        TruthSample {
                            id: *id,
                            position: site.to_ecef(p),
                            observers,
                            global_observers,
                            object_box: box_of.get(id).copied(),
                        }
                    })
                    .collect();
                record.samples.push(Sample {
                    t,
                    t_s,
                    truths,
                    agents: agent_samples,
                });
            }
        }
        record.end = t;
    }

    // --- final synchronization round ---------------------------------------------------
    if spec.final_sync {
        let t_end = record.end;
        let t_end_s = t_end.secs_since(t0);
        bus.controller().heal_all();
        bus.set_lossless(true);
        for (i, ag) in agents.iter_mut().enumerate() {
            if ag.spec.behavior != Behavior::Honest {
                continue;
            }
            let pose = ag.pose(t_end, t_end_s);
            if let Some(bytes) = ag.node.publish(t_end, Some(&pose)) {
                bus.broadcast(i, &bytes, t_end, &BTreeSet::new());
            }
        }
        let mut now = t_end;
        let mut guard = 0;
        while !bus.is_idle() && guard < 10_000 {
            now = now.add_millis(spec.tick_ms as i64);
            deliver(&mut bus, &mut agents, now);
            guard += 1;
        }
        record.final_worlds = agents.iter().map(|a| a.node.world_neutral(t_end)).collect();
        let mut own = ReportStore::new();
        for a in &agents {
            for r in a.node.store().iter().filter(|r| r.source == a.node.id()) {
                own.insert(*r);
            }
        }
        record.source_only_world =
            Some(WorldModel::new(spec.node.world).snapshot(&own, t_end, &TrustView::default()));
    }

    record.node_stats = agents.iter().map(|a| a.node.stats().clone()).collect();
    record.tracker_rejections = agents.iter().map(|a| (a.stale, a.duplicate)).collect();
    record.detections = agents.iter().map(|a| a.next_obs).collect();
    record.bus = bus.stats();
    record
}

/// Delivers due messages, letting relaying agents forward accepted batches once.
fn deliver(bus: &mut SimBus, agents: &mut [SimAgent], now: Timestamp) {
    loop {
        let batch = bus.deliver_until(now);
        if batch.is_empty() {
            break;
        }
        for d in batch {
            let ag = &mut agents[d.to];
            let accepted = matches!(ag.node.receive(&d.bytes, now), Ok(s) if !s.own_echo);
            if accepted && ag.spec.relay && ag.relay_seen.insert(fnv(&d.bytes)) {
                bus.broadcast(d.to, &d.bytes, now, &BTreeSet::from([d.from]));
            }
        }
    }
}

/// Builds a validly signed batch describing objects that do not exist.
fn fabricate(ag: &mut SimAgent, count: usize, site: &EnuFrame, t: Timestamp) -> Vec<u8> {
    let key = ag.key.clone();
    let source = AgentId::from_signing_key(&key);
    let p = rotate_state_cov(
        &site.ecef_from_enu(),
        &Matrix6::from_diagonal(&Vector6::new(0.04, 0.04, 0.04, 0.25, 0.25, 0.25)),
    );
    let reports = (0..count)
        .map(|c| {
            let enu = Vector3::new(
                ag.fab_rng.gen_range(-100.0..100.0),
                ag.fab_rng.gen_range(-100.0..100.0),
                1.0,
            );
            let pos = site.to_ecef(&enu).0;
            ag.fab_seq += 1;
            TrackReport {
                source,
                epoch: ag.node.epoch(),
                seq: ag.fab_seq,
                track: LocalTrackId(c as u64 + 1),
                t_valid: t,
                frame: FrameRef::Ecef,
                pos: [pos.x, pos.y, pos.z],
                vel: [0.0; 3],
                cov: quantize_covariance(&p),
                class_id: 1,
                class_prob: 0.99,
                p_exist: 0.99,
                lifecycle: TrackLifecycle::Confirmed,
                valid_for_ms: 1500,
                born: t,
                name: None,
                object_box: None,
            }
        })
        .collect();
    ReportBatch {
        source,
        epoch: ag.node.epoch(),
        sent_at: t,
        reports,
    }
    .sign(&key)
}

/// Compares an honest agent's just-published global reports with truth.
fn check_reports(
    bytes: &[u8],
    agent: usize,
    t_s: f64,
    ag: &SimAgent,
    truth: &[(u64, Vector3<f64>)],
    site: &EnuFrame,
    out: &mut Vec<ReportCheck>,
) {
    let Ok(batch) = ReportBatch::verify_and_decode(bytes) else {
        return;
    };
    let here = ag.position_enu(t_s);
    for r in batch
        .reports
        .iter()
        .filter(|r| r.frame == FrameRef::Ecef && r.lifecycle != TrackLifecycle::Terminated)
    {
        let pos = Ecef(r.position());
        let Some((_, p_enu)) = truth.iter().map(|(id, p)| (id, *p)).min_by(|a, b| {
            site.to_ecef(&a.1)
                .distance(&pos)
                .total_cmp(&site.to_ecef(&b.1).distance(&pos))
        }) else {
            continue;
        };
        let tp = site.to_ecef(&p_enu);
        let e = pos.0 - tp.0;
        if e.norm() > 10.0 {
            continue;
        }
        let pc: Matrix3<f64> = r.covariance().fixed_view::<3, 3>(0, 0).into_owned();
        if let Some(ch) = pc.cholesky() {
            out.push(ReportCheck {
                agent,
                t_s,
                range_m: (p_enu - here).norm(),
                nees: e.dot(&ch.solve(&e)),
                pos_trace: pc.trace(),
                error_m: e.norm(),
            });
        }
    }
}
