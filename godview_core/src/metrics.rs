//! GodView Metrics Module
//! =======================
//!
//! Implements the Deep Inspection metrics from `sensor_fusion_viz.md`:
//! - **Ghost Score**: Quantifies likelihood a track is a duplicate "ghost"
//! - **Entropy**: Measures uncertainty (covariance health) of a track
//! - **Tension**: Measures contradiction between local detection and fused belief
//!
//! These metrics drive the "Ghost Hunter" visualization mode.

use nalgebra::{Matrix6, Vector6};
use std::f64::consts::PI;

/// Metrics calculated for each track during inspection
#[derive(Debug, Clone, Default)]
pub struct TrackMetrics {
    /// Ghost likelihood score [0, 1]. Higher = more likely a ghost.
    pub ghost_score: f64,
    /// Differential entropy of the track's covariance (bits)
    pub entropy: f64,
    /// Tension with nearest detection (Normalized Innovation Squared)
    pub tension: f64,
    /// ID of the nearest neighbor track (for ghost detection)
    pub nearest_neighbor: Option<uuid::Uuid>,
    /// Mahalanobis distance to nearest neighbor
    pub nearest_distance: f64,
}

// =============================================================================
// GHOST SCORE CALCULATION
// =============================================================================

/// Configuration for ghost score calculation
#[derive(Debug, Clone)]
pub struct GhostScoreConfig {
    /// Weight for spatial proximity component
    pub w_proximity: f64,
    /// Weight for consensus weakness component
    pub w_consensus: f64,
    /// Weight for covariance inflation component
    pub w_covariance: f64,
    /// Gating threshold for "ambiguity zone"
    pub gating_threshold: f64,
    /// Maximum expected covariance trace (for normalization)
    pub max_trace: f64,
}

impl Default for GhostScoreConfig {
    fn default() -> Self {
        Self {
            w_proximity: 0.5,
            w_consensus: 0.3,
            w_covariance: 0.2,
            gating_threshold: 9.21, // Chi-squared 99% for 3 DOF
            max_trace: 100.0,
        }
    }
}

/// Calculate Ghost Score for a track relative to all other tracks.
///
/// Uses spatial neighbors for O(k) performance instead of O(N²).
///
/// # Formula (from sensor_fusion_viz.md Section 3.1)
/// ```text
/// S_ghost = max(w1 * p_ij + w2 * (1 - R_supp) + w3 * Trace(P)/Trace(P_limit))
/// ```
///
/// Where:
/// - `p_ij` = proximity score via Mahalanobis distance
/// - `R_supp` = support ratio (agents observing / agents capable)
/// - `Trace(P)` = covariance trace (uncertainty magnitude)
pub fn calculate_ghost_score(
    track_position: &[f64; 3],
    track_velocity: &[f64; 3],
    track_covariance: &Matrix6<f64>,
    supporting_agents: usize,
    total_agents: usize,
    neighbors: &[([f64; 3], [f64; 3], Matrix6<f64>)], // (pos, vel, cov) of neighbors
    config: &GhostScoreConfig,
) -> (f64, Option<usize>, f64) {
    // Component 1: Spatial Proximity (max across all neighbors)
    let mut max_proximity = 0.0;
    let mut nearest_idx = None;
    let mut nearest_dist = f64::MAX;

    for (idx, (n_pos, n_vel, n_cov)) in neighbors.iter().enumerate() {
        let mahal_dist = mahalanobis_distance_3d(track_position, n_pos, track_covariance, n_cov);

        if mahal_dist < nearest_dist {
            nearest_dist = mahal_dist;
            nearest_idx = Some(idx);
        }

        // Proximity score peaks in "ambiguity zone" near gating threshold
        // Using modified Gaussian that peaks at threshold
        let sigma = config.gating_threshold / 2.0;
        let proximity = if mahal_dist < config.gating_threshold {
            // Inside gate: high proximity, peaks at threshold
            (-((mahal_dist - config.gating_threshold).powi(2)) / (2.0 * sigma * sigma)).exp()
        } else {
            // Outside gate: decays rapidly
            (-(mahal_dist.powi(2)) / (2.0 * sigma * sigma)).exp() * 0.5
        };

        if proximity > max_proximity {
            max_proximity = proximity;
        }
    }

    // Component 2: Consensus Weakness (1 - support ratio)
    let support_ratio = if total_agents > 0 {
        supporting_agents as f64 / total_agents as f64
    } else {
        1.0 // No agents = no weakness
    };
    let consensus_weakness = 1.0 - support_ratio;

    // Component 3: Covariance Inflation
    let position_cov = track_covariance.fixed_view::<3, 3>(0, 0);
    let cov_trace = position_cov.trace();
    let covariance_penalty = (cov_trace / config.max_trace).min(1.0);

    // Composite score
    let ghost_score = (config.w_proximity * max_proximity
        + config.w_consensus * consensus_weakness
        + config.w_covariance * covariance_penalty)
        .clamp(0.0, 1.0);

    (ghost_score, nearest_idx, nearest_dist)
}

/// Calculate Mahalanobis distance between two 3D positions.
///
/// Uses the combined covariance: `D² = (x-y)' * (P_x + P_y)^-1 * (x-y)`
pub fn mahalanobis_distance_3d(
    pos_a: &[f64; 3],
    pos_b: &[f64; 3],
    cov_a: &Matrix6<f64>,
    cov_b: &Matrix6<f64>,
) -> f64 {
    // Extract position covariance (top-left 3x3)
    let p_a = cov_a.fixed_view::<3, 3>(0, 0);
    let p_b = cov_b.fixed_view::<3, 3>(0, 0);

    // Combined covariance
    let p_combined = p_a + p_b;

    // Difference vector
    let delta = nalgebra::Vector3::new(
        pos_a[0] - pos_b[0],
        pos_a[1] - pos_b[1],
        pos_a[2] - pos_b[2],
    );

    // Mahalanobis distance squared: delta' * P^-1 * delta
    match p_combined.try_inverse() {
        Some(p_inv) => {
            let d_squared = delta.transpose() * p_inv * delta;
            d_squared[(0, 0)].sqrt()
        }
        None => {
            // Singular matrix, fall back to Euclidean
            delta.norm()
        }
    }
}

// =============================================================================
// ENTROPY CALCULATION
// =============================================================================

/// Calculate differential entropy of a multivariate Gaussian.
///
/// # Formula (from sensor_fusion_viz.md Section 3.2)
/// ```text
/// H(P) = 0.5 * ln((2πe)^d * det(P))
/// ```
///
/// For dimension d=6 (position + velocity):
/// ```text
/// H = 0.5 * (6 * ln(2πe) + ln(det(P)))
/// ```
pub fn calculate_entropy(covariance: &Matrix6<f64>) -> f64 {
    let d = 6.0; // State dimension
    let det = covariance.determinant();

    if det <= 0.0 {
        // Non-positive definite, return high entropy as warning
        return f64::MAX;
    }

    // H = 0.5 * (d * ln(2πe) + ln(det))
    let two_pi_e = 2.0 * PI * std::f64::consts::E;
    0.5 * (d * two_pi_e.ln() + det.ln())
}

/// Calculate entropy reduction from a fusion step.
///
/// # Formula
/// ```text
/// H_delta = H(P_prior) - H(P_posterior) = 0.5 * ln(det(P_prior) / det(P_posterior))
/// ```
///
/// Returns:
/// - Positive: Information was gained
/// - Zero: Redundant update
/// - Negative: Divergence (bad)
pub fn calculate_entropy_reduction(prior_cov: &Matrix6<f64>, posterior_cov: &Matrix6<f64>) -> f64 {
    let det_prior = prior_cov.determinant();
    let det_posterior = posterior_cov.determinant();

    if det_prior <= 0.0 || det_posterior <= 0.0 {
        return 0.0;
    }

    0.5 * (det_prior / det_posterior).ln()
}

// =============================================================================
// TENSION CALCULATION
// =============================================================================

/// Calculate tension (Normalized Innovation Squared) between a detection and fused belief.
///
/// # Formula (from sensor_fusion_viz.md Section 3.3)
/// ```text
/// T = (z - Hx)' * S^-1 * (z - Hx)
/// ```
///
/// Where:
/// - `z` = local detection position
/// - `x` = global fused state
/// - `S` = innovation covariance = H*P*H' + R
///
/// High tension indicates the agent contradicts the consensus.
pub fn calculate_tension(
    detection_pos: &[f64; 3],
    fused_pos: &[f64; 3],
    fused_covariance: &Matrix6<f64>,
    detection_noise: f64, // Measurement noise variance (scalar for simplicity)
) -> f64 {
    // Innovation (residual)
    let innovation = nalgebra::Vector3::new(
        detection_pos[0] - fused_pos[0],
        detection_pos[1] - fused_pos[1],
        detection_pos[2] - fused_pos[2],
    );

    // Innovation covariance: S = P_position + R
    let p_pos = fused_covariance.fixed_view::<3, 3>(0, 0);
    let r = nalgebra::Matrix3::identity() * detection_noise;
    let s = p_pos + r;

    // NIS = innovation' * S^-1 * innovation
    match s.try_inverse() {
        Some(s_inv) => {
            let nis = innovation.transpose() * s_inv * innovation;
            nis[(0, 0)]
        }
        None => {
            // Singular, return Euclidean squared
            innovation.norm_squared()
        }
    }
}

/// Check if tension exceeds statistical threshold (chi-squared).
///
/// For 3 DOF:
/// - 95% confidence: threshold = 7.81
/// - 99% confidence: threshold = 11.34
pub fn is_tension_significant(tension: f64, confidence_level: f64) -> bool {
    // Chi-squared thresholds for 3 DOF
    let threshold = match confidence_level {
        c if c >= 0.99 => 11.34,
        c if c >= 0.95 => 7.81,
        c if c >= 0.90 => 6.25,
        _ => 5.0,
    };
    tension > threshold
}

// =============================================================================
// TESTS
// =============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_entropy_positive_definite() {
        let cov = Matrix6::identity() * 1.0;
        let entropy = calculate_entropy(&cov);
        assert!(entropy.is_finite());
        assert!(entropy > 0.0);
    }

    #[test]
    fn test_entropy_reduction() {
        let prior = Matrix6::identity() * 10.0; // Large uncertainty
        let posterior = Matrix6::identity() * 1.0; // Small uncertainty

        let reduction = calculate_entropy_reduction(&prior, &posterior);
        assert!(reduction > 0.0, "Entropy should reduce when uncertainty decreases");
    }

    #[test]
    fn test_mahalanobis_identical() {
        let pos = [0.0, 0.0, 0.0];
        let cov = Matrix6::identity();

        let dist = mahalanobis_distance_3d(&pos, &pos, &cov, &cov);
        assert!(dist.abs() < 1e-10, "Distance to self should be zero");
    }

    #[test]
    fn test_ghost_score_range() {
        let pos = [0.0, 0.0, 0.0];
        let vel = [0.0, 0.0, 0.0];
        let cov = Matrix6::identity();
        let config = GhostScoreConfig::default();

        // No neighbors = low ghost score
        let (score, _, _) = calculate_ghost_score(&pos, &vel, &cov, 4, 4, &[], &config);
        assert!(score >= 0.0 && score <= 1.0);
    }

    #[test]
    fn test_tension_zero_for_identical() {
        let pos = [10.0, 20.0, 1.0];
        let cov = Matrix6::identity();

        let tension = calculate_tension(&pos, &pos, &cov, 1.0);
        assert!(tension.abs() < 1e-10, "Tension should be zero for identical positions");
    }
}
