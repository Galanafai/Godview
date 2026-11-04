//! Statistical consistency of peers: who is reporting positions that do not
//! match what everyone else sees?
//!
//! Replaces the legacy usefulness counters, which rewarded novelty and
//! punished redundancy, so honest neighbors that saw the same objects were
//! silenced while fabricated ids counted as useful (audit finding F-23).
//!
//! For each live global report of source `s`, the monitor finds, for every
//! other source, that source's nearest report of the same class within
//! `partner_radius_m`. With at least `min_partners` partners it computes the
//! pairwise NIS against each partner (`dᵀ (P_s + P_j)⁻¹ d`, position only)
//! and records the **median**. A median over three or more partners is not
//! moved by one inconsistent partner, so an honest source next to a biased
//! one is not blamed, while the biased source sees large values against every
//! honest partner. The window mean of these samples drives the status.
//!
//! What this can and cannot establish:
//! * It detects a source whose reports are inconsistent with a majority of
//!   co-observing sources given everyone's stated covariances (bias, bad
//!   calibration, broken localization, under-reported uncertainty).
//! * It cannot attribute blame with fewer than `min_partners` co-observers,
//!   and it cannot see a bias shared by every source.
//! * It cannot detect fabricated objects that no other source observes:
//!   absence of evidence is not modeled. Capability tokens and revocation
//!   handle that class of attack.
//! * Samples taken close in time are correlated, so the window mean is a
//!   detection statistic with a conservative threshold, not a calibrated test.

use crate::frames::Ecef;
use crate::report::AgentId;
use crate::report_store::ReportStore;
use crate::spatial_index::SpatialIndex;
use crate::world_model::{TrustView, WorldModel};
use godview_env::Timestamp;
use nalgebra::{Matrix3, Vector3};
use std::collections::{BTreeMap, VecDeque};

/// Monitor parameters.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ConsistencyConfig {
    /// Samples kept per source.
    pub window: usize,
    /// Samples needed before a source is judged.
    pub min_samples: usize,
    /// Co-observing sources needed to attribute a sample.
    pub min_partners: usize,
    /// Search radius for a partner's report of the same object (m).
    pub partner_radius_m: f64,
    /// Window-mean NIS above which a source's covariance is inflated.
    pub inflate_above: f64,
    /// Window-mean NIS above which a source is excluded from fusion.
    pub exclude_above: f64,
    /// A flagged source is cleared only below `inflate_above * clear_ratio` (hysteresis).
    pub clear_ratio: f64,
    /// Covariance multiplier for inflated sources.
    pub inflate_factor: f64,
}

impl Default for ConsistencyConfig {
    fn default() -> Self {
        Self {
            window: 40,
            min_samples: 10,
            min_partners: 3,
            partner_radius_m: 20.0,
            // chi²(3) at 99.9%: a consistent source's median pairwise NIS sits near 2.4.
            inflate_above: 16.27,
            exclude_above: 60.0,
            clear_ratio: 0.5,
            inflate_factor: 4.0,
        }
    }
}

/// Current judgment of a source.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum SourceStatus {
    /// Not enough co-observed samples yet.
    Unknown,
    /// Consistent with its peers.
    Consistent,
    /// Inconsistent; fused with inflated covariance.
    Inflated,
    /// Grossly inconsistent; not fused.
    Excluded,
}

#[derive(Debug, Clone)]
struct SourceStats {
    nis: VecDeque<f64>,
    residual: VecDeque<Vector3<f64>>,
    status: SourceStatus,
}

/// Tracks per-source consistency from co-observed reports.
#[derive(Debug, Clone)]
pub struct ConsistencyMonitor {
    config: ConsistencyConfig,
    model: WorldModel,
    sources: BTreeMap<AgentId, SourceStats>,
}

impl ConsistencyMonitor {
    /// Creates a monitor using `model` for selection and time alignment.
    pub fn new(config: ConsistencyConfig, model: WorldModel) -> Self {
        Self {
            config,
            model,
            sources: BTreeMap::new(),
        }
    }

    /// Adds samples from the reports live at `t`. Returns the number of samples taken.
    ///
    /// Uses raw (uninflated) covariances and includes excluded sources, so a
    /// source can recover once its reports become consistent again.
    pub fn observe(&mut self, store: &ReportStore, t: Timestamp) -> usize {
        let (items, _) = self.model.prepare(store, t, &TrustView::default());
        let mut index = SpatialIndex::new(self.model.config.resolution);
        for (i, it) in items.iter().enumerate() {
            let _ = index.upsert(i, Ecef(it.x.fixed_rows::<3>(0).into_owned()));
        }
        let mut taken = 0;
        for (i, m) in items.iter().enumerate() {
            let pm = m.x.fixed_rows::<3>(0).into_owned();
            // Nearest report per other source.
            let mut nearest: BTreeMap<AgentId, (f64, usize)> = BTreeMap::new();
            for j in index.query_radius(&Ecef(pm), self.config.partner_radius_m) {
                let o = &items[j];
                if j == i || o.key.0 == m.key.0 || o.class_id != m.class_id {
                    continue;
                }
                let d = (o.x.fixed_rows::<3>(0) - pm).norm();
                let e = nearest.entry(o.key.0).or_insert((d, j));
                if d < e.0 {
                    *e = (d, j);
                }
            }
            if nearest.len() < self.config.min_partners {
                continue;
            }
            let mut nis = Vec::with_capacity(nearest.len());
            let mut residuals = Vec::with_capacity(nearest.len());
            for (_, j) in nearest.values() {
                let o = &items[*j];
                let d = pm - o.x.fixed_rows::<3>(0);
                let s: Matrix3<f64> =
                    (m.p.fixed_view::<3, 3>(0, 0) + o.p.fixed_view::<3, 3>(0, 0)).into_owned();
                if let Some(c) = s.cholesky() {
                    nis.push(d.dot(&c.solve(&d)));
                    residuals.push(d);
                }
            }
            if nis.len() < self.config.min_partners {
                continue;
            }
            let sample = median(&mut nis);
            let resid = Vector3::new(
                median(&mut residuals.iter().map(|r| r.x).collect::<Vec<_>>()),
                median(&mut residuals.iter().map(|r| r.y).collect::<Vec<_>>()),
                median(&mut residuals.iter().map(|r| r.z).collect::<Vec<_>>()),
            );
            let st = self.sources.entry(m.key.0).or_insert_with(|| SourceStats {
                nis: VecDeque::new(),
                residual: VecDeque::new(),
                status: SourceStatus::Unknown,
            });
            if st.nis.len() == self.config.window {
                st.nis.pop_front();
                st.residual.pop_front();
            }
            st.nis.push_back(sample);
            st.residual.push_back(resid);
            taken += 1;
        }
        let cfg = self.config;
        for st in self.sources.values_mut() {
            st.status = next_status(&cfg, st);
        }
        taken
    }

    /// Current status of a source.
    pub fn status(&self, source: &AgentId) -> SourceStatus {
        self.sources
            .get(source)
            .map_or(SourceStatus::Unknown, |s| s.status)
    }

    /// Window-mean NIS of a source, if any samples exist.
    pub fn mean_nis(&self, source: &AgentId) -> Option<f64> {
        let s = self.sources.get(source)?;
        (!s.nis.is_empty()).then(|| s.nis.iter().sum::<f64>() / s.nis.len() as f64)
    }

    /// Mean residual (source minus partners, ECEF meters): an estimate of a constant bias.
    pub fn bias_estimate(&self, source: &AgentId) -> Option<Vector3<f64>> {
        let s = self.sources.get(source)?;
        (!s.residual.is_empty()).then(|| {
            s.residual.iter().fold(Vector3::zeros(), |a, r| a + r) / s.residual.len() as f64
        })
    }

    /// Statuses of all sources seen.
    pub fn statuses(&self) -> BTreeMap<AgentId, SourceStatus> {
        self.sources.iter().map(|(k, v)| (*k, v.status)).collect()
    }

    /// The trust view to apply in fusion.
    pub fn trust_view(&self) -> TrustView {
        let mut view = TrustView::default();
        for (src, st) in &self.sources {
            match st.status {
                SourceStatus::Inflated => {
                    view.inflate.insert(*src, self.config.inflate_factor);
                }
                SourceStatus::Excluded => {
                    view.excluded.insert(*src);
                }
                _ => {}
            }
        }
        view
    }
}

fn next_status(cfg: &ConsistencyConfig, st: &SourceStats) -> SourceStatus {
    if st.nis.len() < cfg.min_samples {
        return st.status;
    }
    let mean = st.nis.iter().sum::<f64>() / st.nis.len() as f64;
    let flagged = matches!(st.status, SourceStatus::Inflated | SourceStatus::Excluded);
    if mean > cfg.exclude_above {
        SourceStatus::Excluded
    } else if mean > cfg.inflate_above {
        SourceStatus::Inflated
    } else if flagged && mean > cfg.inflate_above * cfg.clear_ratio {
        // Hysteresis: stay inflated until clearly consistent again.
        SourceStatus::Inflated
    } else {
        SourceStatus::Consistent
    }
}

fn median(v: &mut [f64]) -> f64 {
    v.sort_by(f64::total_cmp);
    let n = v.len();
    if n % 2 == 1 {
        v[n / 2]
    } else {
        (v[n / 2 - 1] + v[n / 2]) / 2.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::frames::{rotate_state_cov, EnuFrame, Geodetic};
    use crate::report::{quantize_covariance, FrameRef, LocalTrackId, TrackLifecycle, TrackReport};
    use nalgebra::{Matrix6, Vector6};
    use rand::{Rng, SeedableRng};

    fn randn(rng: &mut impl Rng) -> f64 {
        let u1: f64 = rng.gen_range(1e-12..1.0);
        let u2: f64 = rng.gen();
        (-2.0 * u1.ln()).sqrt() * (2.0 * std::f64::consts::PI * u2).cos()
    }

    /// Five sources observe eight objects; source 5 adds an unmodeled 6 m east bias.
    fn run(bias_m: f64, rounds: usize) -> ConsistencyMonitor {
        let site = EnuFrame::new(Geodetic::new(37.7749, -122.4194, 0.0).unwrap());
        let mut rng = rand::rngs::StdRng::seed_from_u64(77);
        let mut mon = ConsistencyMonitor::new(ConsistencyConfig::default(), WorldModel::default());
        let sigma = 0.5;
        for round in 0..rounds {
            let mut store = ReportStore::new();
            let t = Timestamp::from_millis(round as i64 * 100);
            for src in 1..=5u8 {
                for obj in 0..8u64 {
                    let truth = Vector3::new(obj as f64 * 40.0, 10.0, 0.0);
                    let mut enu = truth + Vector3::from_fn(|_, _| randn(&mut rng) * sigma);
                    if src == 5 {
                        enu.x += bias_m;
                    }
                    let p = rotate_state_cov(
                        &site.ecef_from_enu(),
                        &Matrix6::from_diagonal(&Vector6::new(0.25, 0.25, 0.25, 1.0, 1.0, 1.0)),
                    );
                    let pos = site.to_ecef(&enu).0;
                    store.insert(TrackReport {
                        source: AgentId([src; 32]),
                        epoch: 0,
                        seq: round as u64 * 100 + obj,
                        track: LocalTrackId(obj),
                        t_valid: t,
                        frame: FrameRef::Ecef,
                        pos: [pos.x, pos.y, pos.z],
                        vel: [0.0; 3],
                        cov: quantize_covariance(&p),
                        class_id: 1,
                        class_prob: 0.9,
                        p_exist: 0.9,
                        lifecycle: TrackLifecycle::Confirmed,
                        valid_for_ms: 1000,
                        object_box: None,
                    });
                }
            }
            mon.observe(&store, t);
        }
        mon
    }

    #[test]
    fn flags_the_biased_source_and_only_it() {
        let mon = run(6.0, 10);
        assert_eq!(mon.status(&AgentId([5; 32])), SourceStatus::Excluded);
        for src in 1..=4u8 {
            assert_eq!(
                mon.status(&AgentId([src; 32])),
                SourceStatus::Consistent,
                "honest source {src}"
            );
        }
        let view = mon.trust_view();
        assert!(view.excluded.contains(&AgentId([5; 32])));
        assert_eq!(view.excluded.len(), 1);
    }

    #[test]
    fn honest_sources_stay_consistent() {
        let mon = run(0.0, 10);
        for src in 1..=5u8 {
            assert_eq!(mon.status(&AgentId([src; 32])), SourceStatus::Consistent);
            let m = mon.mean_nis(&AgentId([src; 32])).unwrap();
            assert!(m < 5.0, "mean median-NIS {m}");
        }
    }

    #[test]
    fn moderate_bias_is_inflated_and_estimated() {
        // Pairwise innovation std is sqrt(0.25 + 0.25) = 0.71 m per axis, so a 3.5 m bias
        // gives a median NIS near 3.5² / 0.5 + 3 ≈ 27: inside the inflate band (16.27, 60].
        let mon = run(3.5, 10);
        assert_eq!(mon.status(&AgentId([5; 32])), SourceStatus::Inflated);
        assert!(mon.trust_view().inflate.contains_key(&AgentId([5; 32])));
        let b = mon.bias_estimate(&AgentId([5; 32])).unwrap();
        assert!((b.norm() - 3.5).abs() < 0.5, "bias estimate {b}");
    }

    #[test]
    fn detection_limit_is_where_the_threshold_says() {
        // A 2.5 m bias (NIS ≈ 15.5) sits just under the 99.9% threshold: not flagged.
        // This documents the sensitivity rather than hiding it.
        let mon = run(2.5, 10);
        assert_eq!(mon.status(&AgentId([5; 32])), SourceStatus::Consistent);
        let m = mon.mean_nis(&AgentId([5; 32])).unwrap();
        assert!((10.0..16.27).contains(&m), "mean NIS {m}");
    }

    #[test]
    fn needs_enough_partners_to_judge() {
        let mut mon = ConsistencyMonitor::new(
            ConsistencyConfig {
                min_partners: 5,
                ..Default::default()
            },
            WorldModel::default(),
        );
        let store = ReportStore::new();
        assert_eq!(mon.observe(&store, Timestamp::ZERO), 0);
        assert_eq!(mon.status(&AgentId([1; 32])), SourceStatus::Unknown);
    }
}
