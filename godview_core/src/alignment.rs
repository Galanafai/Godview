//! Relative alignment of sources from objects they both see.
//!
//! Each source's reports carry its own localization error. Much of it is a
//! slowly varying offset common to every object that source reports (in the
//! V2V4Real replay, two real cars disagreed by 0.5 to 4 m, drifting over tens
//! of seconds). A fixed declared uncertainty cannot fit that: too tight, and
//! objects both sources see are split in two; too loose, and every fused
//! position is less certain than it needs to be.
//!
//! This module estimates, at each snapshot, a translation for each source that
//! brings its reports into agreement with the other sources, using only the
//! objects they both report:
//!
//! 1. For each pair of sources, every pair of their reports within
//!    `max_offset_m` of each other (same class, compatible box sizes) is a
//!    candidate. Each candidate's difference is tried as the pair's offset; the
//!    one that the most candidates agree with (within `inlier_tol_m`, each report
//!    used once) wins. A row of parked cars shifted by one car does not win,
//!    because the other objects disagree with that shift. The offset is the
//!    mean of its inliers, and its covariance comes from their scatter, not
//!    from the reports' covariances: those include the very pose error being
//!    estimated.
//! 2. The per-source translations are solved jointly by least squares over all
//!    pair offsets, and anchored to the median source of each group of sources
//!    connected by shared objects. One biased source among several is
//!    corrected itself rather than pulling the others.
//! 3. Each report of a source is moved by that source's translation, and the
//!    translation's covariance is added to the report's covariance. The
//!    declared covariance is kept: the correction removes disagreement between
//!    sources, not the error they all share.
//!
//! The estimate is a pure function of the prepared reports, so agents with the
//! same reports compute the same alignment, and nothing is fed back into any
//! source's reports or filters. Only translation is estimated; a heading
//! offset between sources is not.

use crate::report::AgentId;
use godview_env::Timestamp;
use nalgebra::{DMatrix, DVector, Matrix3, Vector3};
use std::collections::{BTreeMap, BTreeSet};

/// Alignment parameters.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AlignConfig {
    /// Whether to align at all. Off by default.
    pub enabled: bool,
    /// Largest offset between two sources that is searched for (m).
    pub max_offset_m: f64,
    /// A candidate agrees with an offset hypothesis within this distance (m).
    pub inlier_tol_m: f64,
    /// Fewest shared objects for a pair of sources to be aligned.
    pub min_pairs: usize,
    /// Floor on the per-object scatter used for an offset's covariance (m).
    pub scatter_floor_m: f64,
}

impl Default for AlignConfig {
    fn default() -> Self {
        Self {
            enabled: false,
            max_offset_m: 8.0,
            inlier_tol_m: 1.0,
            min_pairs: 3,
            scatter_floor_m: 0.1,
        }
    }
}

/// One report as alignment sees it.
#[derive(Debug, Clone, Copy)]
pub(crate) struct AlignItem {
    pub source: AgentId,
    pub position: Vector3<f64>,
    pub class_id: u8,
    /// Length and width of the box, if any.
    pub size: Option<[f64; 2]>,
}

/// The estimated translation of one source.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SourceOffset {
    /// The source.
    pub source: AgentId,
    /// Translation added to the source's positions (ECEF, m).
    pub translation: Vector3<f64>,
    /// Covariance of the translation (ECEF, m²).
    pub cov: Matrix3<f64>,
    /// Shared objects behind it, over all pairs with this source.
    pub shared_objects: usize,
    /// Sources solved together (connected by shared objects) have the same group
    /// number; offsets of different groups are not related to each other.
    pub group: usize,
}

/// An offset between two sources: positions of `b` minus positions of `a`.
#[derive(Debug, Clone, Copy)]
struct PairOffset {
    a: usize,
    b: usize,
    offset: Vector3<f64>,
    cov: Matrix3<f64>,
    n: usize,
}

fn sizes_compatible(a: Option<[f64; 2]>, b: Option<[f64; 2]>) -> bool {
    match (a, b) {
        (Some(x), Some(y)) => {
            (0..2).all(|k| (x[k] - y[k]).abs() <= (0.3 * x[k].max(y[k])).max(1.5))
        }
        _ => true,
    }
}

/// Estimates the offset `positions(b) - positions(a)` from shared objects.
fn pair_offset(
    items: &[AlignItem],
    ia: &[usize],
    ib: &[usize],
    cfg: &AlignConfig,
) -> Option<(Vector3<f64>, Matrix3<f64>, usize)> {
    // Candidates (index in a, index in b, difference), in a fixed order.
    let mut cands: Vec<(usize, usize, Vector3<f64>)> = Vec::new();
    for &i in ia {
        for &j in ib {
            let (x, y) = (&items[i], &items[j]);
            if x.class_id != y.class_id || !sizes_compatible(x.size, y.size) {
                continue;
            }
            let d = y.position - x.position;
            if d.norm() <= cfg.max_offset_m {
                cands.push((i, j, d));
            }
        }
    }
    if cands.len() < cfg.min_pairs {
        return None;
    }
    // Inliers of a hypothesis: one-to-one, closest first.
    let inliers = |h: &Vector3<f64>| -> Vec<Vector3<f64>> {
        let mut by_err: Vec<(f64, usize)> = cands
            .iter()
            .enumerate()
            .map(|(k, c)| ((c.2 - h).norm(), k))
            .filter(|(e, _)| *e <= cfg.inlier_tol_m)
            .collect();
        by_err.sort_by(|p, q| p.0.total_cmp(&q.0).then(p.1.cmp(&q.1)));
        let (mut used_a, mut used_b) = (Vec::new(), Vec::new());
        let mut out = Vec::new();
        for (_, k) in by_err {
            let (i, j, d) = cands[k];
            if used_a.contains(&i) || used_b.contains(&j) {
                continue;
            }
            used_a.push(i);
            used_b.push(j);
            out.push(d);
        }
        out
    };
    let score = |v: &[Vector3<f64>], h: &Vector3<f64>| {
        (
            v.len(),
            -v.iter().map(|d| (d - h).norm_squared()).sum::<f64>(),
        )
    };
    let mut best: Option<(usize, f64, Vector3<f64>)> = None;
    for c in &cands {
        let v = inliers(&c.2);
        let (n, s) = score(&v, &c.2);
        if best.is_none_or(|(bn, bs, _)| n > bn || (n == bn && s > bs)) {
            best = Some((n, s, c.2));
        }
    }
    let (_, _, h) = best?;
    // Refine: mean of the inliers, then the inliers of the mean.
    let first = inliers(&h);
    let mean = first.iter().sum::<Vector3<f64>>() / first.len() as f64;
    let v = inliers(&mean);
    if v.len() < cfg.min_pairs {
        return None;
    }
    let n = v.len() as f64;
    let offset = v.iter().sum::<Vector3<f64>>() / n;
    let mut scatter = Matrix3::zeros();
    for d in &v {
        let e = d - offset;
        scatter += e * e.transpose();
    }
    scatter /= (n - 1.0).max(1.0);
    // With few inliers the deviations from their mean span fewer than three
    // directions, so the scatter is singular. Add the floor in every
    // direction (not only on the diagonal) so the inverse stays bounded.
    let floor = cfg.scatter_floor_m * cfg.scatter_floor_m;
    scatter += Matrix3::identity() * floor;
    Some((offset, scatter / n, v.len()))
}

/// Estimates a translation per source (see the module documentation).
pub(crate) fn estimate(items: &[AlignItem], cfg: &AlignConfig) -> Vec<SourceOffset> {
    if !cfg.enabled {
        return Vec::new();
    }
    let sources: Vec<AgentId> = {
        let mut s: Vec<AgentId> = items.iter().map(|i| i.source).collect();
        s.sort();
        s.dedup();
        s
    };
    let by_source: Vec<Vec<usize>> = sources
        .iter()
        .map(|s| {
            (0..items.len())
                .filter(|&i| items[i].source == *s)
                .collect()
        })
        .collect();
    let mut pairs = Vec::new();
    for a in 0..sources.len() {
        for b in a + 1..sources.len() {
            if let Some((offset, cov, n)) = pair_offset(items, &by_source[a], &by_source[b], cfg) {
                pairs.push(PairOffset {
                    a,
                    b,
                    offset,
                    cov,
                    n,
                });
            }
        }
    }
    if pairs.is_empty() {
        return Vec::new();
    }
    // Groups of sources connected by shared objects.
    let mut group: Vec<usize> = (0..sources.len()).collect();
    fn root(g: &mut [usize], mut i: usize) -> usize {
        while g[i] != i {
            g[i] = g[g[i]];
            i = g[i];
        }
        i
    }
    for p in &pairs {
        let (ra, rb) = (root(&mut group, p.a), root(&mut group, p.b));
        group[ra.max(rb)] = ra.min(rb);
    }
    let mut groups: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
    for s in 0..sources.len() {
        if pairs.iter().any(|p| p.a == s || p.b == s) {
            let r = root(&mut group, s);
            groups.entry(r).or_default().push(s);
        }
    }
    let mut out = Vec::new();
    for (group_no, members) in groups.values().enumerate() {
        let idx: BTreeMap<usize, usize> =
            members.iter().enumerate().map(|(k, s)| (*s, k)).collect();
        let m = members.len();
        // Translations t_s with t_a - t_b = offset(a -> b) for every pair, so
        // that corrected positions agree: (x_a + t_a) = (x_b + t_b).
        let mut h = DMatrix::<f64>::zeros(3 * m, 3 * m);
        let mut g = DVector::<f64>::zeros(3 * m);
        for p in pairs.iter().filter(|p| idx.contains_key(&p.a)) {
            let w = p.cov.try_inverse().unwrap_or_else(Matrix3::identity);
            let (ka, kb) = (3 * idx[&p.a], 3 * idx[&p.b]);
            let wo = w * p.offset;
            for r in 0..3 {
                for c in 0..3 {
                    h[(ka + r, ka + c)] += w[(r, c)];
                    h[(kb + r, kb + c)] += w[(r, c)];
                    h[(ka + r, kb + c)] -= w[(r, c)];
                    h[(kb + r, ka + c)] -= w[(r, c)];
                }
                g[ka + r] += wo[r];
                g[kb + r] -= wo[r];
            }
        }
        // Minimum-norm solution: the group's mean translation is zero. The
        // tolerance is relative, so rounding in the gauge direction is
        // treated as zero rather than inverted.
        let tol = 1e-9 * h.amax().max(1.0);
        let Ok(hp) = h.clone().pseudo_inverse(tol) else {
            continue;
        };
        let t = &hp * &g;
        if t.iter().any(|v| !v.is_finite()) {
            continue;
        }
        // Anchor to the median source, axis by axis.
        let mut shift = Vector3::zeros();
        for axis in 0..3 {
            let mut v: Vec<f64> = (0..m).map(|k| t[3 * k + axis]).collect();
            v.sort_by(f64::total_cmp);
            shift[axis] = if m % 2 == 1 {
                v[m / 2]
            } else {
                (v[m / 2 - 1] + v[m / 2]) / 2.0
            };
        }
        for (k, s) in members.iter().enumerate() {
            let translation = Vector3::new(t[3 * k], t[3 * k + 1], t[3 * k + 2]) - shift;
            let cov = hp.fixed_view::<3, 3>(3 * k, 3 * k).into_owned();
            let shared_objects = pairs
                .iter()
                .filter(|p| p.a == *s || p.b == *s)
                .map(|p| p.n)
                .sum();
            out.push(SourceOffset {
                source: sources[*s],
                translation,
                cov: (cov + cov.transpose()) / 2.0,
                shared_objects,
                group: group_no,
            });
        }
    }
    out
}

/// Parameters of the offset filter (see [`OffsetBank`]); off by default. The
/// defaults are the rule's values from the V2V4Real val inputs
/// (`docs/results/phase_d_d1_plan.md`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct OffsetFilterConfig {
    /// Whether the ego view uses the filter instead of the per-frame estimate.
    pub enabled: bool,
    /// Random-walk power spectral density of the translation, horizontal (m²/s).
    pub q_h: f64,
    /// The same, vertical.
    pub q_v: f64,
    /// Standard deviation of a shared-object offset measurement, horizontal (m).
    pub sigma_obj_h: f64,
    /// The same, vertical.
    pub sigma_obj_v: f64,
    /// Standard deviation of an agent-agent link measurement, horizontal (m).
    pub sigma_link_h: f64,
    /// The same, vertical (the self position's height is only the sensor's).
    pub sigma_link_v: f64,
    /// Mahalanobis gate on a measurement against the predicted translation
    /// (chi-square, 3 degrees of freedom).
    pub gate_chi2: f64,
    /// Standard deviation of the prior of a pair with no estimate (m, per axis).
    pub init_sigma_m: f64,
    /// An estimate older than this without a measurement is dropped (s).
    pub max_age_s: f64,
    /// Largest offset a first estimate searches for (m).
    pub max_offset_m: f64,
    /// A candidate agrees with a first-estimate hypothesis within this (m).
    pub inlier_tol_m: f64,
}

impl Default for OffsetFilterConfig {
    fn default() -> Self {
        Self {
            enabled: false,
            q_h: 0.023,
            q_v: 0.066,
            sigma_obj_h: 0.37,
            sigma_obj_v: 0.57,
            sigma_link_h: 0.33,
            sigma_link_v: 1.0,
            gate_chi2: 16.27,
            init_sigma_m: 8.0 / 3.0,
            max_age_s: 10.0,
            max_offset_m: 8.0,
            inlier_tol_m: 1.0,
        }
    }
}

/// A report as the offset filter sees it.
#[derive(Debug, Clone, Copy)]
pub(crate) struct FilterItem {
    pub source: AgentId,
    pub position: Vector3<f64>,
    pub class_id: u8,
    pub size: Option<[f64; 2]>,
}

/// A direct measurement of the translation to add to `other`'s reports.
#[derive(Debug, Clone, Copy)]
pub(crate) struct LinkMeasurement {
    pub other: AgentId,
    pub z: Vector3<f64>,
}

#[derive(Debug, Clone, Copy)]
struct FilterState {
    /// Translation (local East, North, Up) and its covariance.
    t: Vector3<f64>,
    p: Matrix3<f64>,
    /// Time of the last prediction and of the last accepted measurement.
    at: Timestamp,
    last_meas: Timestamp,
    n_meas: usize,
}

/// One agent's translation estimates for every other agent, over time.
///
/// Per other agent B: the translation to add to B's reports so that they land in
/// the ego agent's frame, a random walk observed through (a) shared objects (any
/// number per frame, gated against the prediction) and (b) agent-agent links
/// (a report of B on the ego's self footprint, or the ego's report on B's). A
/// stateful companion of the ego view; the shared world never uses it.
#[derive(Debug, Clone)]
pub struct OffsetBank {
    cfg: OffsetFilterConfig,
    states: BTreeMap<AgentId, FilterState>,
    /// Rotation from local East-North-Up to ECEF at the first position seen.
    r_enu: Option<Matrix3<f64>>,
}

impl OffsetBank {
    /// Creates an empty bank.
    pub fn new(cfg: OffsetFilterConfig) -> Self {
        Self {
            cfg,
            states: BTreeMap::new(),
            r_enu: None,
        }
    }

    /// The filter parameters.
    pub fn config(&self) -> &OffsetFilterConfig {
        &self.cfg
    }

    fn meas_cov(&self, h: f64, v: f64) -> Matrix3<f64> {
        Matrix3::from_diagonal(&Vector3::new(h * h, h * h, v * v))
    }

    fn update_one(&self, st: &mut FilterState, z: &Vector3<f64>, r: &Matrix3<f64>, t: Timestamp) -> bool {
        let s = st.p + r;
        let Some(chol) = s.cholesky() else {
            return false;
        };
        let y = z - st.t;
        if y.dot(&chol.solve(&y)) >= self.cfg.gate_chi2 {
            return false;
        }
        let k = st.p * chol.inverse();
        st.t += k * y;
        st.p = crate::frames::symmetrize(&((Matrix3::identity() - k) * st.p));
        st.last_meas = t;
        st.n_meas += 1;
        true
    }

    /// Updates every pair with this frame's measurements and returns the current
    /// translations (ECEF) of the others that have an estimate, with the ego's own
    /// at zero.
    pub(crate) fn update(
        &mut self,
        t: Timestamp,
        ego: AgentId,
        reference: Vector3<f64>,
        ego_items: &[FilterItem],
        other_items: &[FilterItem],
        links: &[LinkMeasurement],
    ) -> Vec<SourceOffset> {
        // `reference` (ECEF, any point near the agents) fixes the East-North-Up axes
        // of the state the first time.
        let r = *self.r_enu.get_or_insert_with(|| {
            let g = crate::frames::Ecef(reference).to_geodetic();
            crate::frames::ecef_from_enu_rotation(g.lat_deg, g.lon_deg)
        });
        let to_enu = |v: &Vector3<f64>| r.transpose() * v;
        // Predict, and drop stale states.
        let cfg = self.cfg;
        let q = Matrix3::from_diagonal(&Vector3::new(cfg.q_h, cfg.q_h, cfg.q_v));
        self.states.retain(|_, st| t.secs_since(st.last_meas) <= cfg.max_age_s);
        for st in self.states.values_mut() {
            let dt = t.secs_since(st.at).max(0.0);
            st.p += q * dt;
            st.at = t;
        }
        let mut others: Vec<AgentId> = other_items.iter().map(|i| i.source).collect();
        others.extend(links.iter().map(|l| l.other));
        others.sort();
        others.dedup();
        let r_obj = self.meas_cov(cfg.sigma_obj_h, cfg.sigma_obj_v);
        let r_link = self.meas_cov(cfg.sigma_link_h, cfg.sigma_link_v);
        for b in others.into_iter().filter(|b| *b != ego) {
            let mut state = self.states.get(&b).copied();
            // (b) links: direct measurements.
            for l in links.iter().filter(|l| l.other == b) {
                let z = to_enu(&l.z);
                match state.as_mut() {
                    Some(st) => {
                        self.update_one(st, &z, &r_link, t);
                    }
                    None => {
                        state = Some(FilterState { t: z, p: r_link, at: t, last_meas: t, n_meas: 1 });
                    }
                }
            }
            // (a) shared objects.
            let cands: Vec<(usize, usize, Vector3<f64>)> = ego_items
                .iter()
                .enumerate()
                .flat_map(|(i, x)| {
                    other_items
                        .iter()
                        .enumerate()
                        .filter(move |(_, y)| y.source == b && x.class_id == y.class_id && sizes_compatible(x.size, y.size))
                        .map(move |(j, y)| (i, j, to_enu(&(x.position - y.position))))
                })
                .filter(|(_, _, z)| z.norm() <= cfg.max_offset_m + state.map_or(0.0, |s| s.t.norm()))
                .collect();
            if state.is_none() && !cands.is_empty() {
                // First estimate: the offset most candidates agree with, one-to-one.
                let inliers = |h: &Vector3<f64>| -> Vec<Vector3<f64>> {
                    let mut order: Vec<usize> = (0..cands.len()).collect();
                    order.sort_by(|a, b| {
                        (cands[*a].2 - h).norm().total_cmp(&(cands[*b].2 - h).norm()).then(a.cmp(b))
                    });
                    let (mut ui, mut uj, mut out) = (BTreeSet::new(), BTreeSet::new(), Vec::new());
                    for k in order {
                        let (i, j, z) = cands[k];
                        if (z - h).norm() <= cfg.inlier_tol_m && !ui.contains(&i) && !uj.contains(&j) {
                            ui.insert(i);
                            uj.insert(j);
                            out.push(z);
                        }
                    }
                    out
                };
                let mut best: Option<(usize, Vector3<f64>)> = None;
                for (_, _, h) in &cands {
                    let v = inliers(h);
                    let better = best.map_or(true, |(n, bh)| v.len() > n || (v.len() == n && h.norm() < bh.norm()));
                    if better {
                        best = Some((v.len(), *h));
                    }
                }
                if let Some((n, h)) = best {
                    let v = inliers(&h);
                    let mean = v.iter().fold(Vector3::zeros(), |a, z| a + z) / v.len() as f64;
                    state = Some(FilterState { t: mean, p: r_obj / n as f64, at: t, last_meas: t, n_meas: n });
                }
            } else if let Some(st) = state.as_mut() {
                // Gate against the prediction, closest first, one-to-one.
                let s = st.p + r_obj;
                if let Some(chol) = s.cholesky() {
                    let mut scored: Vec<(f64, usize, usize, Vector3<f64>)> = cands
                        .iter()
                        .filter_map(|(i, j, z)| {
                            let y = z - st.t;
                            let d = y.dot(&chol.solve(&y));
                            (d < cfg.gate_chi2).then_some((d, *i, *j, *z))
                        })
                        .collect();
                    scored.sort_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)).then(a.2.cmp(&b.2)));
                    let (mut ui, mut uj) = (BTreeSet::new(), BTreeSet::new());
                    for (_, i, j, z) in scored {
                        if ui.contains(&i) || uj.contains(&j) {
                            continue;
                        }
                        if self.update_one(st, &z, &r_obj, t) {
                            ui.insert(i);
                            uj.insert(j);
                        }
                    }
                }
            }
            match state {
                Some(st) => {
                    self.states.insert(b, st);
                }
                None => {
                    self.states.remove(&b);
                }
            }
        }
        let mut out = vec![SourceOffset {
            source: ego,
            translation: Vector3::zeros(),
            cov: Matrix3::zeros(),
            shared_objects: 0,
            group: 0,
        }];
        for (b, st) in &self.states {
            out.push(SourceOffset {
                source: *b,
                translation: r * st.t,
                cov: r * st.p * r.transpose(),
                shared_objects: st.n_meas,
                group: 0,
            });
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn item(src: u8, x: f64, y: f64) -> AlignItem {
        AlignItem {
            source: AgentId([src; 32]),
            position: Vector3::new(x, y, 0.0),
            class_id: 1,
            size: Some([4.5, 1.8]),
        }
    }

    fn on() -> AlignConfig {
        AlignConfig {
            enabled: true,
            ..AlignConfig::default()
        }
    }

    fn shifted(src: u8, pts: &[(f64, f64)], dx: f64, dy: f64) -> Vec<AlignItem> {
        pts.iter().map(|(x, y)| item(src, x + dx, y + dy)).collect()
    }

    const SCENE: [(f64, f64); 6] = [
        (0.0, 0.0),
        (12.0, 3.0),
        (25.0, -4.0),
        (31.0, 9.0),
        (-8.0, 15.0),
        (5.0, -12.0),
    ];

    fn translation(out: &[SourceOffset], src: u8) -> Vector3<f64> {
        out.iter()
            .find(|o| o.source == AgentId([src; 32]))
            .unwrap()
            .translation
    }

    /// Issue found on V2V4Real: with exactly three matched objects the
    /// deviations from their mean sum to zero, so the scatter matrix is
    /// singular. A floor on the diagonal alone left it nearly singular, its
    /// inverse huge, and the solve moved cars by up to about 1e15 m.
    #[test]
    fn three_shared_objects_never_blow_up() {
        let mut state: u64 = 0x9E37_79B9_7F4A_7C15;
        let mut unit = || {
            state = state
                .wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1_442_695_040_888_963_407);
            (state >> 11) as f64 / (1u64 << 53) as f64 * 2.0 - 1.0
        };
        let cfg = on();
        for _ in 0..2000 {
            let offset = Vector3::new(3.0 * unit(), 3.0 * unit(), 0.5 * unit());
            let mut items = Vec::new();
            for (x, y) in SCENE.iter().take(3) {
                let p = Vector3::new(*x, *y, 0.0);
                let noise = Vector3::new(0.3 * unit(), 0.3 * unit(), 0.3 * unit());
                items.push(AlignItem {
                    position: p,
                    ..item(1, 0.0, 0.0)
                });
                items.push(AlignItem {
                    position: p + offset + noise,
                    ..item(2, 0.0, 0.0)
                });
            }
            for o in estimate(&items, &cfg) {
                assert!(
                    o.translation.iter().all(|v| v.is_finite())
                        && o.translation.norm() <= cfg.max_offset_m,
                    "translation {:?} for offset {:?}",
                    o.translation,
                    offset
                );
            }
        }
    }

    #[test]
    fn two_sources_meet_halfway() {
        let mut items = shifted(1, &SCENE, 0.0, 0.0);
        items.extend(shifted(2, &SCENE, 2.0, -1.0));
        let out = estimate(&items, &on());
        assert!((translation(&out, 1) - Vector3::new(1.0, -0.5, 0.0)).norm() < 1e-6);
        assert!((translation(&out, 2) - Vector3::new(-1.0, 0.5, 0.0)).norm() < 1e-6);
        assert_eq!(out[0].shared_objects, 6);
    }

    #[test]
    fn one_biased_source_among_four_is_corrected_alone() {
        let mut items = Vec::new();
        for s in 1..=3 {
            items.extend(shifted(s, &SCENE, 0.0, 0.0));
        }
        items.extend(shifted(4, &SCENE, 3.0, 2.0));
        let out = estimate(&items, &on());
        for s in 1..=3 {
            assert!(
                translation(&out, s).norm() < 1e-6,
                "honest source {s} moved"
            );
        }
        assert!((translation(&out, 4) - Vector3::new(-3.0, -2.0, 0.0)).norm() < 1e-6);
    }

    #[test]
    fn a_row_shifted_by_one_car_does_not_win() {
        // A row of parked cars 5 m apart, plus other objects. True offset: 2 m
        // along the row, so shifting by one car (-3 m) also matches the row.
        let mut scene: Vec<(f64, f64)> = (0..6).map(|k| (5.0 * k as f64, 0.0)).collect();
        scene.extend([(10.0, 20.0), (-15.0, -18.0), (30.0, 25.0), (-20.0, 14.0)]);
        let mut items = shifted(1, &scene, 0.0, 0.0);
        items.extend(shifted(2, &scene, 2.0, 0.0));
        let out = estimate(&items, &on());
        assert!((translation(&out, 2) - Vector3::new(-1.0, 0.0, 0.0)).norm() < 1e-6);
    }

    #[test]
    fn too_few_shared_objects_or_disabled_means_no_correction() {
        let mut items = shifted(1, &SCENE[..2], 0.0, 0.0);
        items.extend(shifted(2, &SCENE[..2], 1.0, 0.0));
        assert!(estimate(&items, &on()).is_empty());
        let mut items = shifted(1, &SCENE, 0.0, 0.0);
        items.extend(shifted(2, &SCENE, 1.0, 0.0));
        assert!(estimate(&items, &AlignConfig::default()).is_empty());
    }

    #[test]
    fn a_truck_does_not_align_with_a_car() {
        let mut items = shifted(1, &SCENE, 0.0, 0.0);
        let mut other = shifted(2, &SCENE, 1.0, 0.0);
        for o in &mut other {
            o.size = Some([12.0, 2.5]);
        }
        items.extend(other);
        assert!(estimate(&items, &on()).is_empty());
    }

    // ---- the offset filter -------------------------------------------------

    use rand::{Rng, SeedableRng};

    fn site() -> crate::frames::EnuFrame {
        crate::frames::EnuFrame::new(crate::frames::Geodetic::new(37.7749, -122.4194, 0.0).unwrap())
    }

    fn fitem(src: u8, enu: Vector3<f64>) -> FilterItem {
        FilterItem {
            source: AgentId([src; 32]),
            position: site().to_ecef(&enu).0,
            class_id: 1,
            size: Some([4.5, 1.8]),
        }
    }

    fn gauss(rng: &mut rand::rngs::StdRng, sigma: f64) -> f64 {
        let (u1, u2): (f64, f64) = (rng.gen_range(1e-12..1.0), rng.gen());
        sigma * (-2.0 * u1.ln()).sqrt() * (2.0 * std::f64::consts::PI * u2).cos()
    }

    fn noise3(rng: &mut rand::rngs::StdRng, s: f64) -> Vector3<f64> {
        Vector3::new(gauss(rng, s), gauss(rng, s), gauss(rng, 0.5 * s))
    }

    fn bank() -> OffsetBank {
        OffsetBank::new(OffsetFilterConfig { enabled: true, ..OffsetFilterConfig::default() })
    }

    /// The estimate for agent 2 in ENU relative to the ego (agent 1), if any.
    fn translation_of(out: &[SourceOffset]) -> Option<Vector3<f64>> {
        let r = site().ecef_from_enu();
        out.iter().find(|o| o.source == AgentId([2; 32])).map(|o| r.transpose() * o.translation)
    }

    fn ts(k: i64) -> Timestamp {
        Timestamp::from_millis(k * 100)
    }

    #[test]
    fn the_filter_converges_with_one_shared_object_per_frame() {
        let mut rng = rand::rngs::StdRng::seed_from_u64(1);
        let truth = Vector3::new(1.2, -0.7, 0.3);
        let mut b = bank();
        let mut last = None;
        for k in 0..60 {
            let obj = Vector3::new(20.0 + k as f64, 5.0, 0.0);
            let a = vec![fitem(1, obj + noise3(&mut rng, 0.25))];
            let o = vec![fitem(2, obj - truth + noise3(&mut rng, 0.25))];
            last = translation_of(&b.update(ts(k), AgentId([1; 32]), site().to_ecef(&Vector3::zeros()).0, &a, &o, &[]));
        }
        let e = last.expect("an estimate after the first frame");
        assert!((e - truth).norm() < 0.2, "{e:?}");
    }

    #[test]
    fn the_filter_follows_a_slow_drift() {
        let mut rng = rand::rngs::StdRng::seed_from_u64(2);
        let mut b = bank();
        let mut worst: f64 = 0.0;
        for k in 0..150 {
            // 0.15 m/s East drift: 15 m over the run is out of reach of a static estimate.
            let truth = Vector3::new(1.0 + 0.015 * k as f64, 0.5, 0.0);
            let objs = [Vector3::new(20.0, 5.0, 0.0), Vector3::new(35.0, -8.0, 0.0), Vector3::new(10.0, 12.0, 0.0)];
            let a: Vec<_> = objs.iter().map(|o| fitem(1, *o + noise3(&mut rng, 0.2))).collect();
            let o: Vec<_> = objs.iter().map(|o| fitem(2, *o - truth + noise3(&mut rng, 0.2))).collect();
            if let Some(e) = translation_of(&b.update(ts(k), AgentId([1; 32]), site().to_ecef(&Vector3::zeros()).0, &a, &o, &[])) {
                if k > 30 {
                    worst = worst.max((e - truth).norm());
                }
            }
        }
        assert!(worst < 0.5, "worst error after the first 3 s: {worst}");
    }

    #[test]
    fn the_filter_rejects_a_wrong_match() {
        let mut rng = rand::rngs::StdRng::seed_from_u64(3);
        let truth = Vector3::new(0.8, 0.6, 0.0);
        let mut b = bank();
        for k in 0..30 {
            let objs = [Vector3::new(20.0, 5.0, 0.0), Vector3::new(30.0, -6.0, 0.0)];
            let a: Vec<_> = objs.iter().map(|o| fitem(1, *o + noise3(&mut rng, 0.2))).collect();
            let o: Vec<_> = objs.iter().map(|o| fitem(2, *o - truth + noise3(&mut rng, 0.2))).collect();
            b.update(ts(k), AgentId([1; 32]), site().to_ecef(&Vector3::zeros()).0, &a, &o, &[]);
        }
        let before = translation_of(&b.update(ts(30), AgentId([1; 32]), site().to_ecef(&Vector3::zeros()).0, &[fitem(1, Vector3::new(20.0, 5.0, 0.0))], &[fitem(2, Vector3::new(20.0, 5.0, 0.0) - truth)], &[])).unwrap();
        let n_before = b.states[&AgentId([2; 32])].n_meas;
        // Frames with only a decoy: B reports a car 4 m from the one A reports.
        for k in 31..41 {
            let a = vec![fitem(1, Vector3::new(20.0, 5.0, 0.0))];
            let o = vec![fitem(2, Vector3::new(24.0, 5.0, 0.0) - truth)];
            b.update(ts(k), AgentId([1; 32]), site().to_ecef(&Vector3::zeros()).0, &a, &o, &[]);
        }
        let after = translation_of(&b.update(ts(41), AgentId([1; 32]), site().to_ecef(&Vector3::zeros()).0, &[], &[], &[])).unwrap();
        assert_eq!(b.states[&AgentId([2; 32])].n_meas, n_before, "the decoys were not accepted");
        assert!((after - before).norm() < 0.1, "{after:?} against {before:?}");
        assert!((after - truth).norm() < 0.2);
        // A link far off the prediction is rejected too.
        let wrong = LinkMeasurement { other: AgentId([2; 32]), z: site().ecef_from_enu() * (truth + Vector3::new(5.0, 0.0, 0.0)) };
        b.update(ts(42), AgentId([1; 32]), site().to_ecef(&Vector3::zeros()).0, &[], &[], &[wrong]);
        assert_eq!(b.states[&AgentId([2; 32])].n_meas, n_before);
    }

    #[test]
    fn two_agents_with_only_agent_agent_links_converge() {
        let mut rng = rand::rngs::StdRng::seed_from_u64(4);
        let truth = Vector3::new(-1.1, 0.9, 0.2);
        let r = site().ecef_from_enu();
        let mut b = bank();
        let mut last = None;
        for k in 0..40 {
            // No shared object at all: B's report on A's footprint (translation x_selfA - x_B)
            // and A's report on B's (x_A - x_selfB), each with 0.3 m noise.
            let links = vec![
                LinkMeasurement { other: AgentId([2; 32]), z: r * (truth + noise3(&mut rng, 0.3)) },
                LinkMeasurement { other: AgentId([2; 32]), z: r * (truth + noise3(&mut rng, 0.3)) },
            ];
            last = translation_of(&b.update(ts(k), AgentId([1; 32]), site().to_ecef(&Vector3::zeros()).0, &[], &[], &links));
        }
        let e = last.expect("an estimate from links alone");
        assert!((e - truth).norm() < 0.15, "{e:?}");
    }

    #[test]
    fn an_estimate_goes_stale_and_restarts_from_consensus() {
        let mut b = bank();
        let truth = Vector3::new(1.0, 0.0, 0.0);
        let obj = Vector3::new(20.0, 5.0, 0.0);
        assert!(translation_of(&b.update(ts(0), AgentId([1; 32]), site().to_ecef(&Vector3::zeros()).0, &[fitem(1, obj)], &[fitem(2, obj - truth)], &[])).is_some());
        // Nothing measured for longer than the max age: the estimate is gone.
        assert!(translation_of(&b.update(ts(200), AgentId([1; 32]), site().to_ecef(&Vector3::zeros()).0, &[], &[], &[])).is_none());
    }

}
