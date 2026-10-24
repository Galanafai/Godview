//! L3: the shared world state, computed from replicated reports.
//!
//! [`WorldModel::snapshot`] is a pure function of `(report store, query time,
//! trust view)`. Any two agents holding the same reports and trust view get
//! bit-identical snapshots, whatever order the reports arrived in. That is
//! the convergence guarantee; it says nothing about accuracy, which comes
//! from the estimation below.
//!
//! Steps:
//!
//! 1. **Select** live reports: global (ECEF) frame, not terminated, not past
//!    `t_valid + valid_for`, not from the future, not from excluded sources.
//!    Local-frame reports are replicated but never fused globally.
//! 2. **Predict** each report to the query time with a constant-velocity
//!    model whose process noise is defined in local East-North-Up at the
//!    object and rotated into ECEF.
//! 3. **Gate** pairs from different sources with the Mahalanobis distance of
//!    their difference (`S = P_a + P_b`), position (3 DOF) and velocity
//!    (3 DOF) separately. Candidates come from an H3-partitioned index.
//! 4. **Cluster** in canonical order: gated pairs sorted by distance; two
//!    clusters merge only if they share no source and their fused estimates
//!    gate against each other. One cluster holds at most one track per source.
//!    Gating fused estimates (rather than requiring every pair to gate) keeps
//!    four sources of one object together when one pair's localization errors
//!    happen to fall outside a 99% gate, while still refusing to merge
//!    clusters that disagree as wholes.
//! 5. **Fuse** each cluster with Covariance Intersection, members in key
//!    order, weights chosen to minimize the trace of the fused covariance.
//!    CI is used here, and only here, because the cross-correlation between
//!    different sources' tracks of one object is unknown (shared process
//!    noise, correlated environment). Within a source, the Kalman update is
//!    correct and CI would throw information away.
//!
//! Nothing produced here is ever inserted into a report store or a local
//! filter. That missing feedback path is what rules out rumor propagation.

use crate::frames::{cv_process_noise, cv_transition, ecef_from_enu_rotation, symmetrize, Ecef};
use crate::godview_space::Entity;
use crate::report::{AgentId, FrameRef, ReportKey, TrackLifecycle};
use crate::report_store::{Fnv, ReportStore};
use crate::spatial_index::SpatialIndex;
use godview_env::Timestamp;
use h3o::Resolution;
use nalgebra::{Matrix3, Matrix6, Vector3, Vector6};
use std::collections::{BTreeMap, BTreeSet};

/// Parameters of the shared world model.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WorldModelConfig {
    /// H3 resolution of the association index.
    pub resolution: Resolution,
    /// Acceleration PSD in local East, North, Up at the object (m²/s³).
    pub accel_psd_enu: Vector3<f64>,
    /// Position gate (chi² with 3 DOF; 21.11 = 99.99%).
    ///
    /// Looser than the local tracker's 99% gate on purpose. A source's
    /// localization error is constant for long periods, so a pair of sources
    /// whose errors happen to point apart would otherwise stay split for the
    /// whole period; and with several merges per object per snapshot at 10 Hz,
    /// a 99.9% gate makes world objects flicker. A false merge still needs the
    /// position and the velocity gate to pass against conservative (CI)
    /// covariances, so distinct objects are not merged unless they are
    /// indistinguishable in both.
    pub gate_pos_chi2: f64,
    /// Velocity gate (chi² with 3 DOF; 21.11 = 99.99%).
    pub gate_vel_chi2: f64,
    /// Reports stamped more than this far after the query time are ignored (s).
    pub max_future_skew_s: f64,
}

impl Default for WorldModelConfig {
    fn default() -> Self {
        Self {
            resolution: Resolution::Ten,
            accel_psd_enu: Vector3::new(1.0, 1.0, 0.25),
            gate_pos_chi2: 21.11,
            gate_vel_chi2: 21.11,
            max_future_skew_s: 0.5,
        }
    }
}

/// How a receiver currently treats each source (from the consistency monitor).
#[derive(Debug, Clone, Default, PartialEq)]
pub struct TrustView {
    /// Covariance inflation factor per source (>= 1).
    pub inflate: BTreeMap<AgentId, f64>,
    /// Sources whose reports are not fused.
    pub excluded: BTreeSet<AgentId>,
}

/// One fused object in the shared world state.
#[derive(Debug, Clone, PartialEq)]
pub struct WorldObject {
    /// Identity: the smallest member key. Derived, never negotiated; stable
    /// while that member's track lives.
    pub id: ReportKey,
    /// Contributing reports `(key, version)` in key order (provenance).
    pub members: Vec<(ReportKey, (u32, u64))>,
    /// Query time.
    pub t: Timestamp,
    /// Fused `[p, v]` in ECEF.
    pub state: Vector6<f64>,
    /// Fused covariance.
    pub cov: Matrix6<f64>,
    /// Class id of the most confident member.
    pub class_id: u8,
    /// That member's class probability.
    pub class_prob: f32,
    /// Highest member existence probability.
    pub p_exist: f32,
}

impl WorldObject {
    /// Fused position.
    pub fn position(&self) -> Ecef {
        Ecef(self.state.fixed_rows::<3>(0).into_owned())
    }

    /// Fused velocity in ECEF.
    pub fn velocity(&self) -> Vector3<f64> {
        self.state.fixed_rows::<3>(3).into_owned()
    }

    /// Distinct contributing sources.
    pub fn sources(&self) -> BTreeSet<AgentId> {
        self.members.iter().map(|((s, _), _)| *s).collect()
    }

    /// A UUID derived from the world id, for consumers that key by UUID.
    pub fn uuid(&self) -> uuid::Uuid {
        let mut b = [0u8; 16];
        b[..8].copy_from_slice(&self.id.0 .0[..8]);
        b[8..].copy_from_slice(&self.id.1 .0.to_le_bytes());
        uuid::Uuid::from_bytes(b)
    }

    /// Converts to a spatial-engine entity, preserving the continuous
    /// position and its uncertainty (rotated into local ENU).
    pub fn to_entity(&self) -> Entity {
        let g = self.position().to_geodetic();
        let r_enu = ecef_from_enu_rotation(g.lat_deg, g.lon_deg).transpose();
        let v_enu = r_enu * self.velocity();
        let pc = r_enu * self.cov.fixed_view::<3, 3>(0, 0) * r_enu.transpose();
        Entity {
            id: self.uuid(),
            position: [g.lat_deg, g.lon_deg, g.height_m],
            velocity: [v_enu.x, v_enu.y, v_enu.z],
            entity_type: format!("class_{}", self.class_id),
            timestamp: self.t.as_millis(),
            confidence: self.p_exist,
            position_cov_enu: Some([
                pc[(0, 0)] as f32,
                pc[(0, 1)] as f32,
                pc[(0, 2)] as f32,
                pc[(1, 1)] as f32,
                pc[(1, 2)] as f32,
                pc[(2, 2)] as f32,
            ]),
        }
    }
}

/// The shared world state at one query time.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct WorldSnapshot {
    /// Query time.
    pub t: Timestamp,
    /// Fused objects ordered by id.
    pub objects: Vec<WorldObject>,
    /// Live reports skipped because they are in a source-local frame.
    pub skipped_local_frame: usize,
    /// Reports skipped because they expired before the query time.
    pub skipped_expired: usize,
    /// Reports skipped because their source is excluded.
    pub skipped_untrusted: usize,
    /// Reports skipped because they are stamped in the future.
    pub skipped_future: usize,
}

impl WorldSnapshot {
    /// A deterministic digest of the fused state (ids, membership, bit patterns).
    pub fn digest(&self) -> u64 {
        let mut h = Fnv::new();
        h.u64(self.t.as_nanos() as u64);
        for o in &self.objects {
            for ((src, tr), (e, s)) in &o.members {
                h.bytes(&src.0);
                h.u64(tr.0);
                h.u64(*e as u64);
                h.u64(*s);
            }
            for v in o.state.iter().chain(o.cov.iter()) {
                h.u64(v.to_bits());
            }
        }
        h.finish()
    }
}

/// A report predicted to the query time.
#[derive(Debug, Clone, Copy)]
pub(crate) struct Prepared {
    pub key: ReportKey,
    pub version: (u32, u64),
    pub x: Vector6<f64>,
    pub p: Matrix6<f64>,
    pub class_id: u8,
    pub class_prob: f32,
    pub p_exist: f32,
}

/// The L3 fusion engine.
#[derive(Debug, Clone, Default)]
pub struct WorldModel {
    /// Parameters.
    pub config: WorldModelConfig,
}

impl WorldModel {
    /// Creates a world model.
    pub fn new(config: WorldModelConfig) -> Self {
        Self { config }
    }

    /// Selects live global reports and predicts them to `t`.
    ///
    /// Returns prepared items in key order and the skip counts. Trust
    /// inflation is applied; excluded sources are skipped.
    pub(crate) fn prepare(
        &self,
        store: &ReportStore,
        t: Timestamp,
        trust: &TrustView,
    ) -> (Vec<Prepared>, WorldSnapshot) {
        let mut snap = WorldSnapshot {
            t,
            ..Default::default()
        };
        let mut items = Vec::new();
        for r in store.iter() {
            if r.lifecycle == TrackLifecycle::Terminated {
                continue;
            }
            if r.frame != FrameRef::Ecef {
                snap.skipped_local_frame += 1;
                continue;
            }
            if t > r.expires_at() {
                snap.skipped_expired += 1;
                continue;
            }
            if r.t_valid.secs_since(t) > self.config.max_future_skew_s {
                snap.skipped_future += 1;
                continue;
            }
            if trust.excluded.contains(&r.source) {
                snap.skipped_untrusted += 1;
                continue;
            }
            let (x, mut p) = self.predict(&r.state(), &r.covariance(), t.secs_since(r.t_valid));
            if let Some(f) = trust.inflate.get(&r.source) {
                p *= f.max(1.0);
            }
            items.push(Prepared {
                key: r.key(),
                version: r.version(),
                x,
                p,
                class_id: r.class_id,
                class_prob: r.class_prob,
                p_exist: r.p_exist,
            });
        }
        (items, snap)
    }

    /// Constant-velocity prediction in ECEF with ENU-defined process noise.
    pub fn predict(
        &self,
        x: &Vector6<f64>,
        p: &Matrix6<f64>,
        dt: f64,
    ) -> (Vector6<f64>, Matrix6<f64>) {
        if dt == 0.0 {
            return (*x, *p);
        }
        let g = Ecef(x.fixed_rows::<3>(0).into_owned()).to_geodetic();
        let axes = ecef_from_enu_rotation(g.lat_deg, g.lon_deg);
        let f = cv_transition(dt);
        let q = cv_process_noise(dt, &self.config.accel_psd_enu, &axes);
        (f * x, symmetrize(&(f * p * f.transpose() + q)))
    }

    /// Computes the shared world state at `t` (see the module documentation).
    pub fn snapshot(&self, store: &ReportStore, t: Timestamp, trust: &TrustView) -> WorldSnapshot {
        let (items, mut snap) = self.prepare(store, t, trust);
        let clusters = self.cluster(&items);
        snap.objects = clusters
            .into_iter()
            .filter_map(|members| fuse_cluster(&items, &members, t))
            .collect();
        snap.objects.sort_by_key(|o| o.id);
        snap
    }

    /// Whether two prepared reports may describe the same object.
    pub(crate) fn gates(&self, a: &Prepared, b: &Prepared) -> Option<f64> {
        if a.key.0 == b.key.0 || a.class_id != b.class_id {
            return None;
        }
        self.gate_states(&a.x, &a.p, &b.x, &b.p)
    }

    /// Position and velocity gate on the difference of two estimates
    /// (`S = P_a + P_b`). Returns the position distance if both gates pass.
    pub(crate) fn gate_states(
        &self,
        xa: &Vector6<f64>,
        pa: &Matrix6<f64>,
        xb: &Vector6<f64>,
        pb: &Matrix6<f64>,
    ) -> Option<f64> {
        let d = xa - xb;
        let s = pa + pb;
        let dp = d.fixed_rows::<3>(0).into_owned();
        let sp: Matrix3<f64> = s.fixed_view::<3, 3>(0, 0).into_owned();
        let pos = dp.dot(&sp.cholesky()?.solve(&dp));
        // NaN fails the gate.
        if pos.is_nan() || pos >= self.config.gate_pos_chi2 {
            return None;
        }
        let dv = d.fixed_rows::<3>(3).into_owned();
        let sv: Matrix3<f64> = s.fixed_view::<3, 3>(3, 3).into_owned();
        let vel = dv.dot(&sv.cholesky()?.solve(&dv));
        (vel < self.config.gate_vel_chi2).then_some(pos)
    }

    /// Deterministic constrained complete-linkage clustering.
    pub(crate) fn cluster(&self, items: &[Prepared]) -> Vec<Vec<usize>> {
        let mut index = SpatialIndex::new(self.config.resolution);
        let mut max_trace = 0.0f64;
        for (i, it) in items.iter().enumerate() {
            let _ = index.upsert(i, Ecef(it.x.fixed_rows::<3>(0).into_owned()));
            max_trace = max_trace.max(it.p.fixed_view::<3, 3>(0, 0).trace());
        }
        // Gated pairs. The search radius bounds the Mahalanobis gate:
        // d² <= gate * λmax(S) <= gate * (tr P_i + tr P_max).
        let mut pairs: Vec<(f64, usize, usize)> = Vec::new();
        for (i, a) in items.iter().enumerate() {
            let tr_a = a.p.fixed_view::<3, 3>(0, 0).trace();
            let radius = (self.config.gate_pos_chi2 * (tr_a + max_trace)).sqrt();
            let center = Ecef(a.x.fixed_rows::<3>(0).into_owned());
            for j in index.query_radius(&center, radius) {
                if j <= i {
                    continue;
                }
                if let Some(d2) = self.gates(a, &items[j]) {
                    pairs.push((d2, i, j));
                }
            }
        }
        pairs.sort_by(|x, y| x.0.total_cmp(&y.0).then(x.1.cmp(&y.1)).then(x.2.cmp(&y.2)));

        let mut cluster_of: Vec<usize> = (0..items.len()).collect();
        let mut members: Vec<Vec<usize>> = (0..items.len()).map(|i| vec![i]).collect();
        let mut fused: Vec<(Vector6<f64>, Matrix6<f64>)> =
            items.iter().map(|it| (it.x, it.p)).collect();
        for (_, i, j) in pairs {
            let (ci, cj) = (cluster_of[i], cluster_of[j]);
            if ci == cj {
                continue;
            }
            let sources_i: BTreeSet<AgentId> =
                members[ci].iter().map(|m| items[*m].key.0).collect();
            let disjoint = members[cj]
                .iter()
                .all(|m| !sources_i.contains(&items[*m].key.0));
            if !disjoint {
                continue;
            }
            let (a, b) = (&fused[ci], &fused[cj]);
            if self.gate_states(&a.0, &a.1, &b.0, &b.1).is_none() {
                continue;
            }
            let (keep, gone) = if ci < cj { (ci, cj) } else { (cj, ci) };
            let moved = std::mem::take(&mut members[gone]);
            for m in &moved {
                cluster_of[*m] = keep;
            }
            members[keep].extend(moved);
            members[keep].sort_unstable();
            fused[keep] = fuse_members(items, &members[keep]);
        }
        members.into_iter().filter(|m| !m.is_empty()).collect()
    }
}

/// CI-fuses members (indices in key order) in key order.
fn fuse_members(items: &[Prepared], members: &[usize]) -> (Vector6<f64>, Matrix6<f64>) {
    let first = &items[members[0]];
    let (mut x, mut p) = (first.x, first.p);
    for m in &members[1..] {
        let it = &items[*m];
        let (nx, np) = covariance_intersection(&x, &p, &it.x, &it.p);
        x = nx;
        p = np;
    }
    (x, p)
}

/// Fuses one cluster (member indices in key order) into a world object.
fn fuse_cluster(items: &[Prepared], members: &[usize], t: Timestamp) -> Option<WorldObject> {
    let first = items.get(*members.first()?)?;
    let (x, p) = fuse_members(items, members);
    let best = members.iter().map(|m| &items[*m]).max_by(|a, b| {
        a.class_prob
            .total_cmp(&b.class_prob)
            .then(b.key.cmp(&a.key))
    })?;
    Some(WorldObject {
        id: first.key,
        members: members
            .iter()
            .map(|m| (items[*m].key, items[*m].version))
            .collect(),
        t,
        state: x,
        cov: p,
        class_id: best.class_id,
        class_prob: best.class_prob,
        p_exist: members
            .iter()
            .map(|m| items[*m].p_exist)
            .fold(0.0, f32::max),
    })
}

/// Covariance Intersection with the weight minimizing the fused trace.
///
/// `P⁻¹ = ω A⁻¹ + (1 - ω) B⁻¹`, `x = P (ω A⁻¹ a + (1 - ω) B⁻¹ b)`.
/// The fused covariance is consistent for any unknown cross-correlation
/// between the inputs; fusing an estimate with itself returns it unchanged.
/// The weight is found by golden-section search (the trace is convex in ω).
pub fn covariance_intersection(
    a: &Vector6<f64>,
    pa: &Matrix6<f64>,
    b: &Vector6<f64>,
    pb: &Matrix6<f64>,
) -> (Vector6<f64>, Matrix6<f64>) {
    let (Some(ia), Some(ib)) = (
        pa.cholesky().map(|c| c.inverse()),
        pb.cholesky().map(|c| c.inverse()),
    ) else {
        return if pa.trace() <= pb.trace() {
            (*a, *pa)
        } else {
            (*b, *pb)
        };
    };
    let trace_at = |w: f64| {
        (ia * w + ib * (1.0 - w))
            .cholesky()
            .map_or(f64::INFINITY, |c| c.inverse().trace())
    };
    let (mut lo, mut hi) = (0.0f64, 1.0f64);
    let g = (5f64.sqrt() - 1.0) / 2.0;
    let mut c = hi - g * (hi - lo);
    let mut d = lo + g * (hi - lo);
    let (mut fc, mut fd) = (trace_at(c), trace_at(d));
    for _ in 0..60 {
        if fc < fd {
            hi = d;
            d = c;
            fd = fc;
            c = hi - g * (hi - lo);
            fc = trace_at(c);
        } else {
            lo = c;
            c = d;
            fc = fd;
            d = lo + g * (hi - lo);
            fd = trace_at(d);
        }
    }
    let mut w = (lo + hi) / 2.0;
    // The optimum may sit on a boundary (one input dominates).
    for edge in [0.0, 1.0] {
        if trace_at(edge) < trace_at(w) {
            w = edge;
        }
    }
    // When the trace is flat in ω (for example equal covariances) every weight is
    // optimal; prefer the symmetric one so fusion does not favor whichever input
    // happens to sort first.
    if trace_at(0.5) <= trace_at(w) * (1.0 + 1e-9) {
        w = 0.5;
    }
    let info = ia * w + ib * (1.0 - w);
    let Some(p) = info.cholesky().map(|c| c.inverse()) else {
        return if pa.trace() <= pb.trace() {
            (*a, *pa)
        } else {
            (*b, *pb)
        };
    };
    let x = p * (ia * a * w + ib * b * (1.0 - w));
    (x, symmetrize(&p))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::frames::{EnuFrame, Geodetic};
    use crate::report::{quantize_covariance, LocalTrackId, TrackReport};
    use crate::report_store::ReportStore;
    use rand::seq::SliceRandom;
    use rand::SeedableRng;

    fn site() -> EnuFrame {
        EnuFrame::new(Geodetic::new(37.7749, -122.4194, 0.0).unwrap())
    }

    fn report(
        src: u8,
        track: u64,
        seq: u64,
        enu: Vector3<f64>,
        var: f64,
        t_ms: i64,
    ) -> TrackReport {
        let s = site();
        let p_enu = Matrix6::from_diagonal(&Vector6::new(var, var, var, 0.5, 0.5, 0.5));
        let r = s.ecef_from_enu();
        let cov = crate::frames::rotate_state_cov(&r, &p_enu);
        let pos = s.to_ecef(&enu).0;
        TrackReport {
            source: AgentId([src; 32]),
            epoch: 0,
            seq,
            track: LocalTrackId(track),
            t_valid: Timestamp::from_millis(t_ms),
            frame: FrameRef::Ecef,
            pos: [pos.x, pos.y, pos.z],
            vel: [0.0; 3],
            cov: quantize_covariance(&cov),
            class_id: 1,
            class_prob: 0.9,
            p_exist: 0.9,
            lifecycle: TrackLifecycle::Confirmed,
            valid_for_ms: 1500,
        }
    }

    fn store(rs: &[TrackReport]) -> ReportStore {
        let mut s = ReportStore::new();
        for r in rs {
            s.insert(*r);
        }
        s
    }

    #[test]
    fn two_sources_one_object_fuse_into_one() {
        let rs = [
            report(1, 10, 1, Vector3::new(10.0, 5.0, 0.0), 0.25, 1000),
            report(2, 20, 1, Vector3::new(10.4, 5.2, 0.0), 0.25, 1000),
            report(2, 21, 2, Vector3::new(60.0, 5.0, 0.0), 0.25, 1000),
        ];
        let snap = WorldModel::default().snapshot(
            &store(&rs),
            Timestamp::from_millis(1000),
            &TrustView::default(),
        );
        assert_eq!(snap.objects.len(), 2);
        let fused = snap.objects.iter().find(|o| o.members.len() == 2).unwrap();
        assert_eq!(
            fused.id,
            (AgentId([1; 32]), LocalTrackId(10)),
            "id is the smallest member key"
        );
        let tr = fused.cov.fixed_view::<3, 3>(0, 0).trace();
        assert!(tr <= 0.75 + 1e-6, "fused no worse than either input");
        let mid = site().to_ecef(&Vector3::new(10.2, 5.1, 0.0));
        assert!(fused.position().distance(&mid) < 1e-3);
    }

    #[test]
    fn one_source_never_merges_with_itself() {
        // Two tracks from one source 0.3 m apart stay two objects (one-per-source rule).
        let rs = [
            report(1, 10, 1, Vector3::new(0.0, 0.0, 0.0), 1.0, 0),
            report(1, 11, 2, Vector3::new(0.3, 0.0, 0.0), 1.0, 0),
        ];
        let snap =
            WorldModel::default().snapshot(&store(&rs), Timestamp::ZERO, &TrustView::default());
        assert_eq!(snap.objects.len(), 2);
    }

    #[test]
    fn snapshot_is_independent_of_arrival_order() {
        // I5: same report set, any order or duplication, same bits.
        let mut rng = rand::rngs::StdRng::seed_from_u64(4);
        let mut rs: Vec<TrackReport> = Vec::new();
        for src in 1..=4u8 {
            for obj in 0..6u64 {
                let jitter = Vector3::new(src as f64 * 0.11, -(src as f64) * 0.07, 0.0);
                rs.push(report(
                    src,
                    obj,
                    obj + 1,
                    Vector3::new(obj as f64 * 25.0, 3.0, 1.0) + jitter,
                    0.3,
                    500,
                ));
            }
        }
        let wm = WorldModel::default();
        let t = Timestamp::from_millis(700);
        let reference = wm.snapshot(&store(&rs), t, &TrustView::default());
        assert_eq!(reference.objects.len(), 6);
        assert!(reference.objects.iter().all(|o| o.members.len() == 4));
        for _ in 0..50 {
            let mut shuffled = rs.clone();
            shuffled.extend(rs.iter().take(7).copied());
            shuffled.shuffle(&mut rng);
            assert_eq!(
                wm.snapshot(&store(&shuffled), t, &TrustView::default())
                    .digest(),
                reference.digest()
            );
        }
    }

    #[test]
    fn repeated_exchange_cannot_shrink_uncertainty() {
        // I4: delivering the same reports again (relays, echoes) leaves the fused state identical,
        // and CI of an estimate with itself does not reduce covariance.
        let rs = [
            report(1, 1, 1, Vector3::new(0.0, 0.0, 0.0), 0.5, 0),
            report(2, 1, 1, Vector3::new(0.2, 0.0, 0.0), 0.5, 0),
        ];
        let wm = WorldModel::default();
        let once = wm.snapshot(&store(&rs), Timestamp::ZERO, &TrustView::default());
        let mut echo = store(&rs);
        for _ in 0..100 {
            for r in &rs {
                echo.insert(*r);
            }
        }
        assert_eq!(
            wm.snapshot(&echo, Timestamp::ZERO, &TrustView::default()),
            once
        );
        let x = Vector6::new(1.0, 2.0, 3.0, 0.0, 0.0, 0.0);
        let p = Matrix6::from_diagonal(&Vector6::new(2.0, 1.0, 0.5, 1.0, 1.0, 1.0));
        let (_, pf) = covariance_intersection(&x, &p, &x, &p);
        assert!((pf - p).norm() < 1e-9);
    }

    #[test]
    fn ci_is_consistent_for_correlated_inputs() {
        // Monte Carlo: two estimates sharing a common error component (correlation 0.8).
        // A naive Kalman combination is overconfident; CI must not be.
        let mut rng = rand::rngs::StdRng::seed_from_u64(8);
        let p = Matrix6::identity();
        let mut ci_nees = 0.0;
        let mut kf_nees = 0.0;
        let n = 1500;
        let randn = |rng: &mut rand::rngs::StdRng| {
            use rand::Rng;
            let u1: f64 = rng.gen_range(1e-12..1.0);
            let u2: f64 = rng.gen();
            (-2.0 * u1.ln()).sqrt() * (2.0 * std::f64::consts::PI * u2).cos()
        };
        for _ in 0..n {
            let common = Vector6::from_fn(|_, _| randn(&mut rng) * 0.8f64.sqrt());
            let a = common + Vector6::from_fn(|_, _| randn(&mut rng) * 0.2f64.sqrt());
            let b = common + Vector6::from_fn(|_, _| randn(&mut rng) * 0.2f64.sqrt());
            let (x, pf) = covariance_intersection(&a, &p, &b, &p);
            ci_nees += x.dot(&(pf.try_inverse().unwrap() * x));
            let xk = (a + b) / 2.0;
            kf_nees += xk.dot(&(xk * 2.0)); // naive fused covariance P/2
        }
        let (ci, kf) = (ci_nees / n as f64, kf_nees / n as f64);
        assert!(
            ci <= 6.0 * 1.1,
            "CI mean NEES {ci} should not exceed the state dimension"
        );
        assert!(
            kf > 6.0 * 1.5,
            "naive fusion is overconfident ({kf}); the test must be able to fail"
        );
    }

    #[test]
    fn local_frame_expired_terminated_and_excluded_reports_are_not_fused() {
        let mut local = report(3, 1, 1, Vector3::new(0.0, 0.0, 0.0), 0.5, 1000);
        local.frame = FrameRef::Local { frame_id: 1 };
        let mut dead = report(4, 1, 1, Vector3::new(0.0, 0.0, 0.0), 0.5, 1000);
        dead.lifecycle = TrackLifecycle::Terminated;
        let rs = [
            report(1, 1, 1, Vector3::new(0.0, 0.0, 0.0), 0.5, 1000),
            report(2, 1, 1, Vector3::new(0.0, 0.0, 0.0), 0.5, 0), // expires at 1500 ms
            local,
            dead,
            report(5, 1, 1, Vector3::new(0.0, 0.0, 0.0), 0.5, 1000),
        ];
        let mut trust = TrustView::default();
        trust.excluded.insert(AgentId([5; 32]));
        let snap =
            WorldModel::default().snapshot(&store(&rs), Timestamp::from_millis(1600), &trust);
        assert_eq!(snap.objects.len(), 1);
        assert_eq!(snap.objects[0].members.len(), 1);
        assert_eq!(
            (
                snap.skipped_expired,
                snap.skipped_local_frame,
                snap.skipped_untrusted
            ),
            (1, 1, 1)
        );
    }

    #[test]
    fn prediction_aligns_reports_from_different_times() {
        // A report 0.5 s old of an object moving at 10 m/s east must associate with a fresh one.
        let s = site();
        let mut old = report(1, 1, 1, Vector3::new(0.0, 0.0, 0.0), 0.2, 0);
        let v = s.ecef_from_enu() * Vector3::new(10.0, 0.0, 0.0);
        old.vel = [v.x as f32, v.y as f32, v.z as f32];
        let mut new = report(2, 1, 1, Vector3::new(5.0, 0.0, 0.0), 0.2, 500);
        new.vel = old.vel;
        let snap = WorldModel::default().snapshot(
            &store(&[old, new]),
            Timestamp::from_millis(500),
            &TrustView::default(),
        );
        assert_eq!(snap.objects.len(), 1, "time-aligned estimates associate");
    }

    #[test]
    fn entity_conversion_preserves_position_and_uncertainty() {
        let rs = [report(1, 1, 1, Vector3::new(12.0, -4.0, 30.0), 0.64, 0)];
        let snap =
            WorldModel::default().snapshot(&store(&rs), Timestamp::ZERO, &TrustView::default());
        let e = snap.objects[0].to_entity();
        let back = Geodetic::new(e.position[0], e.position[1], e.position[2])
            .unwrap()
            .to_ecef();
        assert!(back.distance(&snap.objects[0].position()) < 1e-6);
        let c = e.position_cov_enu.unwrap();
        assert!(
            (c[0] - 0.64).abs() < 1e-3 && (c[3] - 0.64).abs() < 1e-3 && (c[5] - 0.64).abs() < 1e-3
        );
    }
}
