//! A seeded synthetic world and sensors, reproducible in every process.
//!
//! Each node process rebuilds the same ground truth from the seed, so nodes
//! that never share memory still observe one physical scene. Each node's
//! sensor noise and localization error come from its own seeded stream.
//!
//! Model: objects move at constant velocity in a local ENU frame at `site`;
//! agents sit on a ring (plus an optional offset) and are stationary; each
//! agent tracks in its own odometry frame (rotated and offset from ENU); its
//! global pose estimate has a localization error drawn once from the
//! covariance it reports.

use godview_core::frames::{rotate_state_cov, EnuFrame, Geodetic, Pose3, PoseWithCov};
use godview_core::local_tracker::Detection;
use godview_core::localization::{LocalizationStatus, PoseEstimate};
use godview_env::Timestamp;
use nalgebra::{Matrix3, Matrix6, Vector3, Vector6};
use rand::{Rng, SeedableRng};
use rand_chacha::ChaCha8Rng;
use serde::{Deserialize, Serialize};

/// Parameters of the synthetic scene. Every node of one run uses the same values.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SyntheticSpec {
    /// Scene seed.
    pub seed: u64,
    /// Number of agents on the ring.
    pub ring_agents: usize,
    /// Ring radius (m).
    pub ring_radius_m: f64,
    /// Number of objects.
    pub objects: usize,
    /// Objects start within `±half_extent_m` of the site origin (m).
    pub half_extent_m: f64,
    /// Maximum object speed per axis (m/s).
    pub max_speed_mps: f64,
    /// Minimum separation between objects over the run (m).
    pub min_separation_m: f64,
    /// Run length used for the separation check (s).
    pub duration_s: f64,
    /// Sensor range (m).
    pub sensor_range_m: f64,
    /// Isotropic sensor noise (m).
    pub sensor_sigma_m: f64,
    /// Reported and realized horizontal localization error (m).
    pub loc_sigma_h_m: f64,
    /// Reported and realized yaw error (degrees).
    pub loc_sigma_yaw_deg: f64,
}

impl Default for SyntheticSpec {
    fn default() -> Self {
        Self {
            seed: 1,
            ring_agents: 4,
            ring_radius_m: 45.0,
            objects: 4,
            half_extent_m: 25.0,
            max_speed_mps: 1.5,
            min_separation_m: 10.0,
            duration_s: 10.0,
            sensor_range_m: 150.0,
            sensor_sigma_m: 0.3,
            loc_sigma_h_m: 0.1,
            loc_sigma_yaw_deg: 0.2,
        }
    }
}

/// The site every synthetic run is placed at (San Francisco, 10 m).
pub fn site() -> EnuFrame {
    EnuFrame::new(Geodetic::new(37.7749, -122.4194, 10.0).expect("constant site is valid"))
}

/// Ground truth shared by all processes of one run.
#[derive(Debug, Clone)]
pub struct Truth {
    /// `(start position, velocity)` in ENU (m, m/s).
    pub objects: Vec<(Vector3<f64>, Vector3<f64>)>,
}

impl Truth {
    /// Builds the objects for `spec` by seeded rejection sampling.
    pub fn new(spec: &SyntheticSpec) -> Self {
        let mut rng = ChaCha8Rng::seed_from_u64(spec.seed ^ 0x6f62_6a65_6374);
        let mut objects: Vec<(Vector3<f64>, Vector3<f64>)> = Vec::new();
        let mut attempts = 0;
        while objects.len() < spec.objects && attempts < 100_000 {
            attempts += 1;
            let h = spec.half_extent_m;
            let s = spec.max_speed_mps;
            let p = Vector3::new(rng.gen_range(-h..h), rng.gen_range(-h..h), 1.0);
            let v = Vector3::new(rng.gen_range(-s..s), rng.gen_range(-s..s), 0.0);
            let steps = (spec.duration_s * 4.0).ceil() as usize;
            let separated = objects.iter().all(|(q, w)| {
                (0..=steps).all(|k| {
                    let t = k as f64 / 4.0;
                    ((p + v * t) - (q + w * t)).norm() >= spec.min_separation_m
                })
            });
            if separated {
                objects.push((p, v));
            }
        }
        Self { objects }
    }

    /// Object positions in ENU at `t_s` seconds after the start.
    pub fn positions_enu(&self, t_s: f64) -> Vec<Vector3<f64>> {
        self.objects.iter().map(|(p, v)| p + v * t_s).collect()
    }
}

/// One agent's sensors and localization in the synthetic world.
#[derive(Debug, Clone)]
pub struct SyntheticAgent {
    /// Agent position in ENU (m).
    pub position_enu: Vector3<f64>,
    enu_from_odom: Pose3,
    estimate: PoseWithCov,
    sensor_rng: ChaCha8Rng,
    next_obs: u64,
    range_m: f64,
    sigma_m: f64,
}

impl SyntheticAgent {
    /// Agent `index` of the ring, displaced by `offset_enu` (m).
    pub fn new(spec: &SyntheticSpec, index: usize, offset_enu: Vector3<f64>) -> Self {
        let n = spec.ring_agents.max(1) as f64;
        let th = std::f64::consts::TAU * index as f64 / n;
        let position_enu = Vector3::new(
            spec.ring_radius_m * th.cos(),
            spec.ring_radius_m * th.sin(),
            2.0,
        ) + offset_enu;
        let enu_from_odom = Pose3::from_yaw(
            0.3 + 0.7 * index as f64,
            position_enu + Vector3::new(3.0 * index as f64, -2.0 * index as f64, 0.5),
        );
        let site = site();
        let r = site.ecef_from_enu();
        let true_link = site.ecef_from_enu_pose().compose(&enu_from_odom);
        // Tangent ordering is [translation, rotation], as in `Pose3::perturbed`.
        let mut cov_enu = Matrix6::zeros();
        let sh = spec.loc_sigma_h_m;
        let syaw = spec.loc_sigma_yaw_deg.to_radians();
        cov_enu[(0, 0)] = sh * sh;
        cov_enu[(1, 1)] = sh * sh;
        cov_enu[(2, 2)] = (2.0 * sh).powi(2);
        cov_enu[(3, 3)] = (0.5 * syaw).powi(2);
        cov_enu[(4, 4)] = (0.5 * syaw).powi(2);
        cov_enu[(5, 5)] = syaw * syaw;
        let cov = rotate_state_cov(&r, &cov_enu);
        let mut loc_rng = stream(spec.seed, 3, index as u64);
        let delta_enu = Vector6::from_fn(|k, _| randn(&mut loc_rng) * cov_enu[(k, k)].sqrt());
        let mut delta = Vector6::zeros();
        delta
            .fixed_rows_mut::<3>(0)
            .copy_from(&(r * delta_enu.fixed_rows::<3>(0)));
        delta
            .fixed_rows_mut::<3>(3)
            .copy_from(&(r * delta_enu.fixed_rows::<3>(3)));
        // The estimate is the truth moved by -δ, so truth = estimate ⊕ δ with δ ~ N(0, cov).
        let estimate = PoseWithCov {
            pose: true_link.perturbed(&(-delta)),
            cov,
        };
        Self {
            position_enu,
            enu_from_odom,
            estimate,
            sensor_rng: stream(spec.seed, 1, index as u64),
            next_obs: 0,
            range_m: spec.sensor_range_m,
            sigma_m: spec.sensor_sigma_m,
        }
    }

    /// Noisy detections, in this agent's odometry frame, of every object in range.
    pub fn detect(&mut self, t: Timestamp, truth_enu: &[Vector3<f64>]) -> Vec<Detection> {
        let odom_from_enu = self.enu_from_odom.inverse();
        let mut out = Vec::new();
        for p in truth_enu {
            let n = Vector3::new(
                randn(&mut self.sensor_rng),
                randn(&mut self.sensor_rng),
                randn(&mut self.sensor_rng),
            ) * self.sigma_m;
            if (p - self.position_enu).norm() > self.range_m {
                continue;
            }
            self.next_obs += 1;
            out.push(Detection {
                obs_id: self.next_obs,
                t,
                // Noise drawn in ENU and rotated with the point.
                z: odom_from_enu.transform_point(&(p + n)),
                r: Matrix3::identity() * self.sigma_m * self.sigma_m,
                class_id: 1,
                class_prob: 0.9,
            });
        }
        out
    }

    /// The agent's global pose estimate (with its reported uncertainty).
    pub fn pose(&self, t: Timestamp) -> PoseEstimate {
        PoseEstimate {
            t,
            ecef_from_odom: self.estimate,
            status: LocalizationStatus::Global,
        }
    }
}

fn stream(seed: u64, stream: u64, index: u64) -> ChaCha8Rng {
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
    (-2.0 * u1.ln()).sqrt() * (std::f64::consts::TAU * u2).cos()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn truth_is_reproducible_and_separated() {
        let spec = SyntheticSpec::default();
        let a = Truth::new(&spec);
        let b = Truth::new(&spec);
        assert_eq!(a.objects, b.objects);
        assert_eq!(a.objects.len(), spec.objects);
        for t in [0.0, 5.0, 10.0] {
            let p = a.positions_enu(t);
            for i in 0..p.len() {
                for j in i + 1..p.len() {
                    assert!((p[i] - p[j]).norm() >= spec.min_separation_m);
                }
            }
        }
    }

    #[test]
    fn detections_map_back_to_truth_through_the_true_pose() {
        let spec = SyntheticSpec {
            sensor_sigma_m: 0.0,
            ..SyntheticSpec::default()
        };
        let mut agent = SyntheticAgent::new(&spec, 2, Vector3::zeros());
        let truth = Truth::new(&spec).positions_enu(1.0);
        let dets = agent.detect(Timestamp::ZERO, &truth);
        assert_eq!(dets.len(), truth.len());
        for (d, p) in dets.iter().zip(&truth) {
            assert!((agent.enu_from_odom.transform_point(&d.z) - p).norm() < 1e-9);
        }
    }

    #[test]
    fn distant_agent_sees_nothing() {
        let spec = SyntheticSpec::default();
        let mut agent = SyntheticAgent::new(&spec, 0, Vector3::new(5000.0, 0.0, 0.0));
        let truth = Truth::new(&spec).positions_enu(0.0);
        assert!(agent.detect(Timestamp::ZERO, &truth).is_empty());
    }
}
