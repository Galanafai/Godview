//! Scoring estimates against ground truth.
//!
//! Replaces the legacy RMS metric, which scored only tracks still named with
//! the agent's own UUID, returned 0.0 when none were, and ignored ghosts and
//! misses (audit finding F-29).
//!
//! * **GOSPA** (generalized optimal sub-pattern assignment, α = 2, p = 2):
//!   one number combining localization error of matched pairs with a fixed
//!   penalty `c²/2` for every missed object and every false (ghost) estimate.
//!   Matching is optimal (Hungarian), so it does not depend on track names.
//! * **NEES** (normalized estimation error squared) of matched pairs:
//!   `eᵀ P⁻¹ e` over the 3-D position. For a consistent estimator its mean is
//!   3. Within one run errors are correlated in time (for example a fixed
//!   localization bias), so a single run's mean NEES is a sanity bound, not a
//!   calibrated test; calibrated checks use many independent seeds.

use godview_core::assignment::assign;
use godview_core::frames::Ecef;
use nalgebra::Matrix3;

/// Result of scoring one set of estimates against one set of truths.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Score {
    /// GOSPA distance (m).
    pub gospa: f64,
    /// Root mean square position error over matched pairs (m).
    pub rms_matched: f64,
    /// Matched pairs.
    pub matched: usize,
    /// Truths without an estimate within the cutoff.
    pub missed: usize,
    /// Estimates without a truth within the cutoff (ghosts and duplicates).
    pub false_estimates: usize,
    /// NEES of each matched pair (only when covariances were supplied).
    pub nees: Vec<f64>,
    /// Indices of truths left unmatched.
    pub unmatched_truths: Vec<usize>,
    /// Indices of estimates left unmatched.
    pub unmatched_estimates: Vec<usize>,
}

/// Scores estimates (position and optional position covariance) against truths.
pub fn score(estimates: &[(Ecef, Option<Matrix3<f64>>)], truths: &[Ecef], cutoff_m: f64) -> Score {
    let c2 = cutoff_m * cutoff_m;
    let cost: Vec<Vec<f64>> = estimates
        .iter()
        .map(|(e, _)| truths.iter().map(|t| (e.0 - t.0).norm_squared()).collect())
        .collect();
    let pairs = if estimates.is_empty() || truths.is_empty() {
        Vec::new()
    } else {
        assign(&cost, c2)
    };
    let mut s = Score::default();
    let mut sq = 0.0;
    for (ei, ti) in &pairs {
        let d2 = cost[*ei][*ti];
        sq += d2;
        if let Some(p) = estimates[*ei].1 {
            let e = estimates[*ei].0 .0 - truths[*ti].0;
            if let Some(ch) = p.cholesky() {
                s.nees.push(e.dot(&ch.solve(&e)));
            }
        }
    }
    s.unmatched_truths = (0..truths.len())
        .filter(|t| !pairs.iter().any(|(_, ti)| ti == t))
        .collect();
    s.unmatched_estimates = (0..estimates.len())
        .filter(|e| !pairs.iter().any(|(ei, _)| ei == e))
        .collect();
    s.matched = pairs.len();
    s.missed = truths.len() - pairs.len();
    s.false_estimates = estimates.len() - pairs.len();
    s.rms_matched = if s.matched > 0 {
        (sq / s.matched as f64).sqrt()
    } else {
        0.0
    };
    s.gospa = (sq + c2 / 2.0 * (s.missed + s.false_estimates) as f64).sqrt();
    s
}

/// Two-sided bounds for the mean of `n` independent `χ²(k)` samples, via the
/// Wilson-Hilferty approximation, at standard-normal quantile `z`
/// (2.576 ≈ 99%, 4.0 ≈ 99.994%).
pub fn mean_chi2_bounds(k: usize, n: usize, z: f64) -> (f64, f64) {
    let dof = (k * n) as f64;
    let q = |z: f64| {
        let a = 2.0 / (9.0 * dof);
        dof * (1.0 - a + z * a.sqrt()).powi(3)
    };
    (q(-z) / n as f64, q(z) / n as f64)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn p(x: f64) -> Ecef {
        Ecef::new(x, 0.0, 0.0)
    }

    #[test]
    fn perfect_match_scores_zero() {
        let s = score(&[(p(0.0), None), (p(10.0), None)], &[p(10.0), p(0.0)], 5.0);
        assert_eq!(
            (s.gospa, s.matched, s.missed, s.false_estimates),
            (0.0, 2, 0, 0)
        );
    }

    #[test]
    fn ghosts_and_misses_are_penalized() {
        // One correct, one ghost far away, one truth missed.
        let s = score(&[(p(0.0), None), (p(100.0), None)], &[p(0.0), p(50.0)], 5.0);
        assert_eq!((s.matched, s.missed, s.false_estimates), (1, 1, 1));
        assert_eq!(
            (
                s.unmatched_truths.as_slice(),
                s.unmatched_estimates.as_slice()
            ),
            (&[1usize][..], &[1usize][..])
        );
        assert!(
            (s.gospa - (25.0f64).sqrt()).abs() < 1e-12,
            "c²/2 per miss and per ghost"
        );
    }

    #[test]
    fn duplicate_estimate_counts_as_false() {
        let s = score(&[(p(0.0), None), (p(0.5), None)], &[p(0.0)], 5.0);
        assert_eq!((s.matched, s.false_estimates), (1, 1));
    }

    #[test]
    fn nees_uses_covariance() {
        let s = score(&[(p(2.0), Some(Matrix3::identity() * 4.0))], &[p(0.0)], 5.0);
        assert!((s.nees[0] - 1.0).abs() < 1e-12);
    }

    #[test]
    fn chi2_bounds_bracket_the_mean() {
        let (lo, hi) = mean_chi2_bounds(3, 100, 2.576);
        assert!(lo < 3.0 && hi > 3.0 && hi < 4.0 && lo > 2.0, "({lo}, {hi})");
    }
}
