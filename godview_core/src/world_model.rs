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
//! 5. **Name** each cluster (see [`IdRule::Lineage`]). Every report may
//!    carry a `name`: the ID its track belonged to in its source's latest
//!    world. A cluster takes the name carried by its oldest member (its own
//!    key if it carries none), so an ID started by one track passes to the
//!    tracks that were fused with it and outlives the track that started it.
//!    A name that is the key of a live track in another cluster is not
//!    allowed (that track's cluster owns it); two clusters claiming the same
//!    name of an ended track are settled in a fixed order, and a cluster that
//!    loses falls back to its next claim and finally to its oldest member's
//!    key, which no other cluster can take. The result depends only on the
//!    reports and the trust view, so agents with the same reports agree on
//!    IDs as they agree on everything else.
//! 6. **Fuse** each cluster with Covariance Intersection, members in key
//!    order, weights chosen to minimize the trace of the fused covariance.
//!    CI is used here, and only here, because the cross-correlation between
//!    different sources' tracks of one object is unknown (shared process
//!    noise, correlated environment). Within a source, the Kalman update is
//!    correct and CI would throw information away.
//!
//! Nothing produced here is ever inserted into a report store or a local
//! filter. That missing feedback path is what rules out rumor propagation.
//! The one exception is the name label of step 5: a node publishes the ID its
//! own track belongs to. Names never enter gating, clustering or fusion, so
//! they cannot change any estimate or covariance; they only choose IDs.

use crate::extent::{fuse_boxes, BoxGate, BoxModel, ObjectBox};
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
    /// Box consistency required to fuse two reports or two clusters.
    pub box_gate: BoxGate,
    /// Box process noise used when predicting reports to the query time.
    pub box_model: BoxModel,
    /// Which member names a fused object.
    pub id_rule: IdRule,
}

/// How a fused object's ID is chosen.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum IdRule {
    /// The name carried by the oldest member, so an ID survives handoffs
    /// (see the module documentation, step 5).
    #[default]
    Lineage,
    /// The key of the member whose track was created first (ties: smaller
    /// key). Names are ignored: the ID changes when that track leaves.
    OldestTrack,
    /// The smallest member key. The earlier rule, kept for comparison: the
    /// ID changes whenever a source with a smaller key joins.
    SmallestKey,
}

impl Default for WorldModelConfig {
    fn default() -> Self {
        Self {
            resolution: Resolution::Ten,
            accel_psd_enu: Vector3::new(1.0, 1.0, 0.25),
            gate_pos_chi2: 21.11,
            gate_vel_chi2: 21.11,
            max_future_skew_s: 0.5,
            box_gate: BoxGate::default(),
            box_model: BoxModel::default(),
            id_rule: IdRule::default(),
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
    /// Identity, a report key (see [`IdRule`]). Derived from the reports,
    /// never negotiated, and unique within a snapshot.
    pub id: ReportKey,
    /// When the oldest current member's track was created (its source's clock).
    pub born: Timestamp,
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
    /// Fused box, heading counterclockwise from local East at the object.
    pub object_box: Option<ObjectBox>,
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
            oriented_box: self.object_box.map(|b| {
                [
                    b.size[0] as f32,
                    b.size[1] as f32,
                    b.size[2] as f32,
                    b.yaw as f32,
                ]
            }),
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
            h.bytes(&o.id.0 .0);
            h.u64(o.id.1 .0);
            h.u64(o.born.as_nanos() as u64);
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
    /// When the source created the track.
    pub born: Timestamp,
    /// The name the source gives this track's object (see [`IdRule::Lineage`]).
    pub name: Option<ReportKey>,
    /// Box with heading in ENU at the object, predicted to the query time.
    pub object_box: Option<ObjectBox>,
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
            let dt = t.secs_since(r.t_valid);
            let (x, mut p) = self.predict(&r.state(), &r.covariance(), dt);
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
                born: r.born,
                name: r.name,
                object_box: r.object_box.map(|b| self.config.box_model.predict(&b, dt)),
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
        let ids = name_clusters(&items, &clusters, self.config.id_rule);
        snap.objects = clusters
            .iter()
            .zip(ids)
            .filter_map(|(members, id)| fuse_cluster(&items, members, t, id))
            .collect();
        snap.objects.sort_by_key(|o| o.id);
        snap
    }

    /// Whether two prepared reports may describe the same object.
    pub(crate) fn gates(&self, a: &Prepared, b: &Prepared) -> Option<f64> {
        if a.key.0 == b.key.0
            || a.class_id != b.class_id
            || !self
                .config
                .box_gate
                .passes(a.object_box.as_ref(), b.object_box.as_ref())
        {
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
        let mut fused: Vec<(Vector6<f64>, Matrix6<f64>, Option<ObjectBox>)> =
            items.iter().map(|it| (it.x, it.p, it.object_box)).collect();
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
            if self.gate_states(&a.0, &a.1, &b.0, &b.1).is_none()
                || !self.config.box_gate.passes(a.2.as_ref(), b.2.as_ref())
            {
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
fn fuse_members(
    items: &[Prepared],
    members: &[usize],
) -> (Vector6<f64>, Matrix6<f64>, Option<ObjectBox>) {
    let first = &items[members[0]];
    let (mut x, mut p, mut b) = (first.x, first.p, first.object_box);
    for m in &members[1..] {
        let it = &items[*m];
        let (nx, np) = covariance_intersection(&x, &p, &it.x, &it.p);
        x = nx;
        p = np;
        b = match (b, it.object_box) {
            (Some(a), Some(c)) => Some(fuse_boxes(&a, &c)),
            (None, c) => c,
            (a, None) => a,
        };
    }
    (x, p, b)
}

/// Members of a cluster from oldest to youngest track (ties: smaller key).
fn by_age(items: &[Prepared], members: &[usize]) -> Vec<usize> {
    let mut v = members.to_vec();
    v.sort_by(|a, b| {
        let (a, b) = (&items[*a], &items[*b]);
        a.born.cmp(&b.born).then(a.key.cmp(&b.key))
    });
    v
}

/// Chooses each cluster's ID (one per cluster, unique within the snapshot).
fn name_clusters(items: &[Prepared], clusters: &[Vec<usize>], rule: IdRule) -> Vec<ReportKey> {
    let aged: Vec<Vec<usize>> = clusters.iter().map(|c| by_age(items, c)).collect();
    match rule {
        IdRule::SmallestKey => clusters
            .iter()
            .map(|c| {
                c.iter()
                    .map(|m| items[*m].key)
                    .min()
                    .expect("non-empty cluster")
            })
            .collect(),
        IdRule::OldestTrack => aged.iter().map(|c| items[c[0]].key).collect(),
        IdRule::Lineage => {
            // Which cluster each live track is in: a live track's key may only
            // name its own cluster.
            let owner: BTreeMap<ReportKey, usize> = clusters
                .iter()
                .enumerate()
                .flat_map(|(ci, c)| c.iter().map(move |m| (items[*m].key, ci)))
                .collect();
            // Settle clusters in order of their oldest member.
            let mut order: Vec<usize> = (0..clusters.len()).collect();
            order.sort_by(|a, b| {
                let (a, b) = (&items[aged[*a][0]], &items[aged[*b][0]]);
                a.born.cmp(&b.born).then(a.key.cmp(&b.key))
            });
            let mut taken: BTreeSet<ReportKey> = BTreeSet::new();
            let mut ids: Vec<ReportKey> = aged.iter().map(|c| items[c[0]].key).collect();
            for ci in order {
                let fallback = items[aged[ci][0]].key;
                let id = aged[ci]
                    .iter()
                    .map(|m| items[*m].name.unwrap_or(items[*m].key))
                    .find(|n| owner.get(n).is_none_or(|o| *o == ci) && !taken.contains(n))
                    .unwrap_or(fallback);
                taken.insert(id);
                ids[ci] = id;
            }
            ids
        }
    }
}

/// Fuses one cluster (member indices in key order) into a world object named `id`.
fn fuse_cluster(
    items: &[Prepared],
    members: &[usize],
    t: Timestamp,
    id: ReportKey,
) -> Option<WorldObject> {
    let oldest = &items[*by_age(items, members).first()?];
    let (x, p, object_box) = fuse_members(items, members);
    let best = members.iter().map(|m| &items[*m]).max_by(|a, b| {
        a.class_prob
            .total_cmp(&b.class_prob)
            .then(b.key.cmp(&a.key))
    })?;
    Some(WorldObject {
        id,
        born: oldest.born,
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
        object_box,
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
            born: Timestamp::from_millis(0),
            name: None,
            object_box: None,
        }
    }

    fn store(rs: &[TrackReport]) -> ReportStore {
        let mut s = ReportStore::new();
        for r in rs {
            s.insert(*r);
        }
        s
    }

    fn boxed(mut r: TrackReport, length: f64, yaw: f64) -> TrackReport {
        r.object_box = Some(ObjectBox {
            size: [length, 2.0, 2.0],
            size_var: [0.04; 3],
            yaw,
            yaw_var: 0.0025,
            yaw_ambiguous: true,
        });
        r
    }

    #[test]
    fn box_gate_keeps_a_car_and_a_truck_apart_when_position_cannot() {
        // Two sources with poor localization (1 m) each report a car and a truck
        // 3 m apart. Position alone gates every cross pair; with source 2's
        // errors pushing its car toward source 1's truck, position-only
        // clustering pairs the wrong ones. Boxes refuse those pairs.
        let car_1 = boxed(
            report(1, 1, 1, Vector3::new(0.0, 0.0, 0.0), 1.0, 1000),
            4.5,
            0.0,
        );
        let truck_1 = boxed(
            report(1, 2, 2, Vector3::new(0.0, 3.0, 0.0), 1.0, 1000),
            12.0,
            0.0,
        );
        let car_2 = boxed(
            report(2, 1, 1, Vector3::new(0.0, 2.2, 0.0), 1.0, 1000),
            4.6,
            3.1,
        );
        let truck_2 = boxed(
            report(2, 2, 2, Vector3::new(0.0, 5.0, 0.0), 1.0, 1000),
            11.8,
            0.05,
        );
        let st = store(&[car_1, truck_1, car_2, truck_2]);
        let t = Timestamp::from_millis(1000);
        let members = |cfg: WorldModelConfig| -> Vec<Vec<u64>> {
            let snap = WorldModel::new(cfg).snapshot(&st, t, &TrustView::default());
            snap.objects
                .iter()
                .map(|o| {
                    o.members
                        .iter()
                        .map(|((s, tr), _)| s.0[0] as u64 * 10 + tr.0)
                        .collect()
                })
                .collect()
        };
        let with = members(WorldModelConfig::default());
        assert_eq!(
            with,
            vec![vec![11, 21], vec![12, 22]],
            "cars with cars, trucks with trucks"
        );
        let without = members(WorldModelConfig {
            box_gate: BoxGate::disabled(),
            ..WorldModelConfig::default()
        });
        assert!(
            without.contains(&vec![12, 21]),
            "position alone pairs truck 1 with car 2: {without:?}"
        );
        let snap = WorldModel::default().snapshot(&st, t, &TrustView::default());
        let truck = snap
            .objects
            .iter()
            .find(|o| o.members.len() == 2 && o.object_box.unwrap().size[0] > 8.0)
            .unwrap();
        let b = truck.object_box.unwrap();
        assert!(
            (b.size[0] - 11.9).abs() < 0.11,
            "fused truck length {}",
            b.size[0]
        );
        assert_eq!(
            truck.to_entity().oriented_box.map(|o| o[0] > 8.0),
            Some(true)
        );
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
            "tracks born together: the smaller key names the object"
        );
        let tr = fused.cov.fixed_view::<3, 3>(0, 0).trace();
        assert!(tr <= 0.75 + 1e-6, "fused no worse than either input");
        let mid = site().to_ecef(&Vector3::new(10.2, 5.1, 0.0));
        assert!(fused.position().distance(&mid) < 1e-3);
    }

    fn born_at(mut r: TrackReport, born_ms: i64) -> TrackReport {
        r.born = Timestamp::from_millis(born_ms);
        r
    }

    #[test]
    fn a_source_joining_later_does_not_rename_the_object() {
        let wm = WorldModel::default();
        let old_rule = WorldModel::new(WorldModelConfig {
            id_rule: IdRule::SmallestKey,
            ..WorldModelConfig::default()
        });
        let at = Vector3::new(10.0, 5.0, 0.0);
        let first = born_at(report(9, 90, 1, at, 0.25, 5000), 1000);
        let t = Timestamp::from_millis(5000);
        let alone = wm.snapshot(&store(&[first]), t, &TrustView::default());
        assert_eq!(alone.objects[0].id, first.key());
        // A source with a smaller key starts tracking the same object later.
        let joiner = born_at(
            report(1, 10, 1, at + Vector3::new(0.3, 0.0, 0.0), 0.25, 5000),
            4000,
        );
        let both = store(&[first, joiner]);
        let snap = wm.snapshot(&both, t, &TrustView::default());
        assert_eq!(snap.objects.len(), 1);
        assert_eq!(snap.objects[0].members.len(), 2);
        assert_eq!(
            snap.objects[0].id,
            first.key(),
            "the oldest track keeps the ID"
        );
        assert_eq!(snap.objects[0].born, first.born);
        // The earlier rule renames the object as soon as the smaller key joins.
        let snap = old_rule.snapshot(&both, t, &TrustView::default());
        assert_eq!(snap.objects[0].id, joiner.key());
        // The same reports give the same IDs whatever order they arrived in.
        let reversed = store(&[joiner, first]);
        assert_eq!(
            wm.snapshot(&reversed, t, &TrustView::default()).digest(),
            wm.snapshot(&both, t, &TrustView::default()).digest()
        );
    }

    fn named(mut r: TrackReport, name: ReportKey) -> TrackReport {
        r.name = Some(name);
        r
    }

    #[test]
    fn a_name_outlives_the_track_that_started_it() {
        let wm = WorldModel::default();
        let at = Vector3::new(10.0, 5.0, 0.0);
        let t = Timestamp::from_millis(5000);
        let a = born_at(report(5, 50, 1, at, 0.25, 5000), 1000);
        // b and c were fused with a and carry its ID as their name.
        let b = named(
            born_at(
                report(7, 70, 1, at + Vector3::new(0.2, 0.0, 0.0), 0.25, 5000),
                2000,
            ),
            a.key(),
        );
        let c = born_at(
            report(3, 30, 1, at + Vector3::new(0.0, 0.2, 0.0), 0.25, 5000),
            3000,
        );
        let snap = wm.snapshot(&store(&[a, b, c]), t, &TrustView::default());
        assert_eq!(snap.objects[0].id, a.key());
        // a's track ends: the object keeps a's ID through b's name.
        let mut gone = a;
        gone.seq += 1;
        gone.lifecycle = TrackLifecycle::Terminated;
        let snap = wm.snapshot(&store(&[a, gone, b, c]), t, &TrustView::default());
        assert_eq!((snap.objects.len(), snap.objects[0].id), (1, a.key()));
        assert_eq!(
            snap.objects[0].born, b.born,
            "born is the oldest current member"
        );
        // Without the name, the oldest-track rule hands it to b.
        let oldest = WorldModel::new(WorldModelConfig {
            id_rule: IdRule::OldestTrack,
            ..WorldModelConfig::default()
        });
        let snap = oldest.snapshot(&store(&[a, gone, b, c]), t, &TrustView::default());
        assert_eq!(snap.objects[0].id, b.key());
    }

    #[test]
    fn names_choose_ids_and_change_nothing_else() {
        let wm = WorldModel::default();
        let t = Timestamp::from_millis(5000);
        let p = Vector3::new(10.0, 5.0, 0.0);
        let q = Vector3::new(60.0, 5.0, 0.0);
        let a = born_at(report(5, 50, 1, p, 0.25, 5000), 1000);
        let b = born_at(report(6, 60, 1, q, 0.25, 5000), 1000);
        // c, the oldest member near q, names a, whose track is alive in
        // another object: the name is refused and the next member's claim
        // (b's own key) names the object.
        let c = named(
            born_at(
                report(9, 90, 1, q + Vector3::new(0.2, 0.0, 0.0), 0.25, 5000),
                500,
            ),
            a.key(),
        );
        let snap = wm.snapshot(&store(&[a, b, c]), t, &TrustView::default());
        let ids: BTreeSet<ReportKey> = snap.objects.iter().map(|o| o.id).collect();
        assert_eq!(ids, [a.key(), b.key()].into_iter().collect());
        // Two objects claiming the same ended track's name get distinct IDs:
        // the one with the older member keeps it.
        let dead = (AgentId([1; 32]), LocalTrackId(1));
        let a2 = named(a, dead);
        let b2 = named(b, dead);
        let snap = wm.snapshot(&store(&[a2, b2]), t, &TrustView::default());
        let ids: Vec<ReportKey> = snap.objects.iter().map(|o| o.id).collect();
        assert_eq!(ids.len(), 2);
        assert!(ids.contains(&dead) && ids.contains(&b.key()), "{ids:?}");
        // Names never change an estimate.
        let plain = wm.snapshot(&store(&[a, b]), t, &TrustView::default());
        for (x, y) in plain.objects.iter().zip(&snap.objects) {
            assert_eq!((x.state, x.cov, &x.members), (y.state, y.cov, &y.members));
        }
    }

    #[test]
    fn the_id_passes_to_the_next_oldest_member_when_its_holder_leaves() {
        let wm = WorldModel::new(WorldModelConfig {
            id_rule: IdRule::OldestTrack,
            ..WorldModelConfig::default()
        });
        let at = Vector3::new(10.0, 5.0, 0.0);
        let t = Timestamp::from_millis(5000);
        let a = born_at(report(5, 50, 1, at, 0.25, 5000), 1000);
        let b = born_at(
            report(7, 70, 1, at + Vector3::new(0.2, 0.0, 0.0), 0.25, 5000),
            2000,
        );
        let c = born_at(
            report(3, 30, 1, at + Vector3::new(0.0, 0.2, 0.0), 0.25, 5000),
            3000,
        );
        let snap = wm.snapshot(&store(&[a, b, c]), t, &TrustView::default());
        assert_eq!((snap.objects.len(), snap.objects[0].id), (1, a.key()));
        // The holder's track ends: the next-oldest member takes over, not the smallest key.
        let mut gone = a;
        gone.seq += 1;
        gone.lifecycle = TrackLifecycle::Terminated;
        let snap = wm.snapshot(&store(&[a, gone, b, c]), t, &TrustView::default());
        assert_eq!((snap.objects.len(), snap.objects[0].id), (1, b.key()));
        // Excluding the holder's source has the same effect.
        let trust = TrustView {
            excluded: [AgentId([5; 32])].into_iter().collect(),
            ..TrustView::default()
        };
        let snap = wm.snapshot(&store(&[a, b, c]), t, &trust);
        assert_eq!(snap.objects[0].id, b.key());
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
