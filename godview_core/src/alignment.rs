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
use nalgebra::{DMatrix, DVector, Matrix3, Vector3};
use std::collections::BTreeMap;

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
    for members in groups.values() {
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
            });
        }
    }
    out
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
}
