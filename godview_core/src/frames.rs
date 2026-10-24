//! Coordinate systems, frames and uncertainty propagation.
//!
//! This module is the single place where GodView converts between ways of
//! writing a position. The contract (see `docs/PERCEPTION_SHARING.md`):
//!
//! * **Shared frame:** WGS84 ECEF, meters, `f64`. Every global estimate on
//!   the wire and in the shared world model is expressed here. ECEF is
//!   Cartesian (Mahalanobis gating and Covariance Intersection are valid in
//!   it) and needs no agreement on an anchor point.
//! * **Local frames:** each agent tracks objects in its own smooth odometry
//!   frame (any right-handed Cartesian frame, meters). The link from that
//!   frame to ECEF is a [`PoseWithCov`] supplied by localization.
//! * **Geodetic** latitude/longitude/ellipsoidal height is used only for
//!   human-facing output and for H3 indexing. It is never fused.
//!
//! Positions are typed ([`Ecef`], [`Geodetic`]) so that degrees and meters
//! can no longer be mixed silently, which was the root cause of findings
//! F-01 and F-02 in the architecture audit.
//!
//! # Pose uncertainty convention
//!
//! A [`PoseWithCov`] maps child coordinates to parent coordinates,
//! `p_parent = R p_child + t`. Its 6x6 covariance is over the error vector
//! `[δt, δθ]` expressed in the **parent** frame, with the true pose given by
//! `R_true = Exp(δθ) R`, `t_true = t + δt` (a left perturbation). Under this
//! convention a point's first-order Jacobian with respect to the pose error
//! is `[I, -[R p]×]`.

use nalgebra::{Matrix3, Matrix6, Rotation3, Vector3, Vector6};
use serde::{Deserialize, Serialize};

/// WGS84 semi-major axis (m).
pub const WGS84_A: f64 = 6_378_137.0;
/// WGS84 flattening.
pub const WGS84_F: f64 = 1.0 / 298.257_223_563;
/// WGS84 first eccentricity squared.
pub const WGS84_E2: f64 = WGS84_F * (2.0 - WGS84_F);

// ============================================================================
// TYPED POSITIONS
// ============================================================================

/// Geodetic coordinates on the WGS84 ellipsoid.
///
/// Height is ellipsoidal (not mean sea level).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Geodetic {
    /// Latitude in degrees, positive north.
    pub lat_deg: f64,
    /// Longitude in degrees, positive east.
    pub lon_deg: f64,
    /// Height above the WGS84 ellipsoid in meters.
    pub height_m: f64,
}

impl Geodetic {
    /// Creates a geodetic coordinate, rejecting out-of-range or non-finite values.
    pub fn new(lat_deg: f64, lon_deg: f64, height_m: f64) -> Result<Self, FrameError> {
        if !(lat_deg.is_finite() && lon_deg.is_finite() && height_m.is_finite()) {
            return Err(FrameError::NonFinite);
        }
        if !(-90.0..=90.0).contains(&lat_deg) || !(-180.0..=180.0).contains(&lon_deg) {
            return Err(FrameError::OutOfRange { lat_deg, lon_deg });
        }
        Ok(Self {
            lat_deg,
            lon_deg,
            height_m,
        })
    }

    /// Converts to Earth-centered, Earth-fixed coordinates.
    pub fn to_ecef(&self) -> Ecef {
        let lat = self.lat_deg.to_radians();
        let lon = self.lon_deg.to_radians();
        let (sl, cl) = lat.sin_cos();
        let n = WGS84_A / (1.0 - WGS84_E2 * sl * sl).sqrt();
        Ecef(Vector3::new(
            (n + self.height_m) * cl * lon.cos(),
            (n + self.height_m) * cl * lon.sin(),
            (n * (1.0 - WGS84_E2) + self.height_m) * sl,
        ))
    }

    /// Returns the H3 coordinate for this point (height is ignored by H3).
    pub fn to_latlng(&self) -> Result<h3o::LatLng, FrameError> {
        h3o::LatLng::new(self.lat_deg, self.lon_deg).map_err(|_| FrameError::NonFinite)
    }
}

/// A position in WGS84 Earth-centered, Earth-fixed coordinates (meters).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Ecef(pub Vector3<f64>);

impl Ecef {
    /// Creates an ECEF position from components in meters.
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self(Vector3::new(x, y, z))
    }

    /// Converts to geodetic coordinates.
    ///
    /// Iterates on latitude with the height formula
    /// `h = p cos φ + z sin φ - a sqrt(1 - e² sin² φ)`, which stays well
    /// conditioned at the poles. Converges to well below a micrometer.
    pub fn to_geodetic(&self) -> Geodetic {
        let (x, y, z) = (self.0.x, self.0.y, self.0.z);
        let lon = y.atan2(x);
        let p = (x * x + y * y).sqrt();
        let mut lat = z.atan2(p * (1.0 - WGS84_E2));
        let mut h = 0.0;
        for _ in 0..16 {
            let (sl, cl) = lat.sin_cos();
            let n = WGS84_A / (1.0 - WGS84_E2 * sl * sl).sqrt();
            h = p * cl + z * sl - WGS84_A * (1.0 - WGS84_E2 * sl * sl).sqrt();
            let next = z.atan2(p * (1.0 - WGS84_E2 * n / (n + h)));
            if (next - lat).abs() < 1e-15 {
                lat = next;
                break;
            }
            lat = next;
        }
        Geodetic {
            lat_deg: lat.to_degrees(),
            lon_deg: lon.to_degrees(),
            height_m: h,
        }
    }

    /// Euclidean distance in meters.
    pub fn distance(&self, other: &Ecef) -> f64 {
        (self.0 - other.0).norm()
    }
}

/// Errors from coordinate handling.
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum FrameError {
    /// A coordinate component was NaN or infinite.
    #[error("non-finite coordinate")]
    NonFinite,
    /// Latitude or longitude outside the valid range (usually meters passed as degrees).
    #[error("latitude/longitude out of range: ({lat_deg}, {lon_deg})")]
    OutOfRange {
        /// Offending latitude.
        lat_deg: f64,
        /// Offending longitude.
        lon_deg: f64,
    },
}

// ============================================================================
// LOCAL TANGENT (ENU) FRAMES
// ============================================================================

/// Rotation taking East-North-Up vectors at a point into ECEF.
pub fn ecef_from_enu_rotation(lat_deg: f64, lon_deg: f64) -> Matrix3<f64> {
    let (sl, cl) = lat_deg.to_radians().sin_cos();
    let (so, co) = lon_deg.to_radians().sin_cos();
    // Columns are the east, north and up unit vectors expressed in ECEF.
    Matrix3::new(-so, -sl * co, cl * co, co, -sl * so, cl * so, 0.0, cl, sl)
}

/// A local East-North-Up frame anchored at a geodetic point.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct EnuFrame {
    /// Anchor point.
    pub origin: Geodetic,
    origin_ecef: Ecef,
    ecef_from_enu: Matrix3<f64>,
}

impl EnuFrame {
    /// Creates an ENU frame at the given anchor.
    pub fn new(origin: Geodetic) -> Self {
        Self {
            origin,
            origin_ecef: origin.to_ecef(),
            ecef_from_enu: ecef_from_enu_rotation(origin.lat_deg, origin.lon_deg),
        }
    }

    /// Rotation taking ENU vectors into ECEF.
    pub fn ecef_from_enu(&self) -> Matrix3<f64> {
        self.ecef_from_enu
    }

    /// Converts an ENU position (meters) into ECEF.
    pub fn to_ecef(&self, enu: &Vector3<f64>) -> Ecef {
        Ecef(self.origin_ecef.0 + self.ecef_from_enu * enu)
    }

    /// Converts an ECEF position into this frame's ENU coordinates.
    pub fn from_ecef(&self, p: &Ecef) -> Vector3<f64> {
        self.ecef_from_enu.transpose() * (p.0 - self.origin_ecef.0)
    }

    /// The pose of this ENU frame in ECEF (exact, zero covariance).
    pub fn ecef_from_enu_pose(&self) -> Pose3 {
        Pose3 {
            rotation: Rotation3::from_matrix_unchecked(self.ecef_from_enu),
            translation: self.origin_ecef.0,
        }
    }
}

// ============================================================================
// POSES AND UNCERTAINTY PROPAGATION
// ============================================================================

/// A rigid transform `p_parent = rotation * p_child + translation`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Pose3 {
    /// Rotation from child to parent axes.
    pub rotation: Rotation3<f64>,
    /// Child origin expressed in the parent frame (meters).
    pub translation: Vector3<f64>,
}

impl Pose3 {
    /// The identity transform.
    pub fn identity() -> Self {
        Self {
            rotation: Rotation3::identity(),
            translation: Vector3::zeros(),
        }
    }

    /// Creates a pose from a yaw angle (about +z) and a translation.
    pub fn from_yaw(yaw_rad: f64, translation: Vector3<f64>) -> Self {
        Self {
            rotation: Rotation3::from_axis_angle(&Vector3::z_axis(), yaw_rad),
            translation,
        }
    }

    /// Composes `self ∘ other`: first `other` (child → middle), then `self`.
    pub fn compose(&self, other: &Pose3) -> Pose3 {
        Pose3 {
            rotation: self.rotation * other.rotation,
            translation: self.rotation * other.translation + self.translation,
        }
    }

    /// The inverse transform (parent → child).
    pub fn inverse(&self) -> Pose3 {
        let r_inv = self.rotation.inverse();
        Pose3 {
            rotation: r_inv,
            translation: -(r_inv * self.translation),
        }
    }

    /// Maps a child-frame point into the parent frame.
    pub fn transform_point(&self, p: &Vector3<f64>) -> Vector3<f64> {
        self.rotation * p + self.translation
    }

    /// Maps a child-frame vector (e.g. a velocity) into the parent frame.
    pub fn transform_vector(&self, v: &Vector3<f64>) -> Vector3<f64> {
        self.rotation * v
    }

    /// Applies a left perturbation `[δt, δθ]` (parent frame), returning the perturbed pose.
    pub fn perturbed(&self, delta: &Vector6<f64>) -> Pose3 {
        let dt = Vector3::new(delta[0], delta[1], delta[2]);
        let dth = Vector3::new(delta[3], delta[4], delta[5]);
        Pose3 {
            rotation: Rotation3::new(dth) * self.rotation,
            translation: self.translation + dt,
        }
    }
}

/// A pose with a 6x6 covariance over `[δt, δθ]` in the parent frame.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PoseWithCov {
    /// Mean transform.
    pub pose: Pose3,
    /// Error covariance (m², rad²); see module docs for the convention.
    pub cov: Matrix6<f64>,
}

impl PoseWithCov {
    /// A pose known exactly.
    pub fn exact(pose: Pose3) -> Self {
        Self {
            pose,
            cov: Matrix6::zeros(),
        }
    }

    /// Transforms a child-frame position/velocity state and its covariance into the parent frame.
    ///
    /// First-order propagation:
    ///
    /// ```text
    /// x_p = [R p + t, R v]
    /// P_p = J_x P J_xᵀ + J_pose Σ J_poseᵀ
    /// J_x    = blockdiag(R, R)
    /// J_pose = [[I, -[R p]×], [0, -[R v]×]]
    /// ```
    ///
    /// Assumes the state error and the pose error are independent, which holds
    /// when the state was estimated in the child frame from sensors that do not
    /// depend on the child-to-parent link (the source-purity rule).
    pub fn transform_state(
        &self,
        x_child: &Vector6<f64>,
        p_child: &Matrix6<f64>,
    ) -> (Vector6<f64>, Matrix6<f64>) {
        let r = *self.pose.rotation.matrix();
        let p = Vector3::new(x_child[0], x_child[1], x_child[2]);
        let v = Vector3::new(x_child[3], x_child[4], x_child[5]);
        let rp = r * p;
        let rv = r * v;
        let pos = rp + self.pose.translation;

        let mut jx = Matrix6::zeros();
        jx.fixed_view_mut::<3, 3>(0, 0).copy_from(&r);
        jx.fixed_view_mut::<3, 3>(3, 3).copy_from(&r);

        let mut jp = Matrix6::zeros();
        jp.fixed_view_mut::<3, 3>(0, 0)
            .copy_from(&Matrix3::identity());
        jp.fixed_view_mut::<3, 3>(0, 3).copy_from(&(-skew(&rp)));
        jp.fixed_view_mut::<3, 3>(3, 3).copy_from(&(-skew(&rv)));

        let cov = symmetrize(&(jx * p_child * jx.transpose() + jp * self.cov * jp.transpose()));
        (Vector6::new(pos.x, pos.y, pos.z, rv.x, rv.y, rv.z), cov)
    }

    /// Composes `self ∘ child` with first-order covariance propagation.
    ///
    /// `self` maps middle → parent, `child` maps child → middle; the result maps
    /// child → parent. Errors of the two links are assumed independent.
    pub fn compose(&self, child: &PoseWithCov) -> PoseWithCov {
        let r = *self.pose.rotation.matrix();
        let rt = r * child.pose.translation;
        // Child error [δt_c, δθ_c] is in the middle frame; rotate it into the parent.
        let mut jc = Matrix6::zeros();
        jc.fixed_view_mut::<3, 3>(0, 0).copy_from(&r);
        jc.fixed_view_mut::<3, 3>(3, 3).copy_from(&r);
        // Parent link error moves the child's translation by -[R t_c]× δθ_p.
        let mut jparent = Matrix6::identity();
        jparent
            .fixed_view_mut::<3, 3>(0, 3)
            .copy_from(&(-skew(&rt)));
        PoseWithCov {
            pose: self.pose.compose(&child.pose),
            cov: symmetrize(
                &(jparent * self.cov * jparent.transpose() + jc * child.cov * jc.transpose()),
            ),
        }
    }
}

/// The skew-symmetric (cross-product) matrix of `v`.
pub fn skew(v: &Vector3<f64>) -> Matrix3<f64> {
    Matrix3::new(0.0, -v.z, v.y, v.z, 0.0, -v.x, -v.y, v.x, 0.0)
}

/// Returns `(m + mᵀ) / 2`.
pub fn symmetrize<const N: usize>(
    m: &nalgebra::SMatrix<f64, N, N>,
) -> nalgebra::SMatrix<f64, N, N> {
    (m + m.transpose()) * 0.5
}

/// True if `m` is symmetric positive definite (Cholesky succeeds) with finite entries.
pub fn is_spd<const N: usize>(m: &nalgebra::SMatrix<f64, N, N>) -> bool {
    m.iter().all(|v| v.is_finite()) && m.cholesky().is_some()
}

/// Rotates the position and velocity blocks of a 6x6 covariance by `r`.
pub fn rotate_state_cov(r: &Matrix3<f64>, p: &Matrix6<f64>) -> Matrix6<f64> {
    let mut b = Matrix6::zeros();
    b.fixed_view_mut::<3, 3>(0, 0).copy_from(r);
    b.fixed_view_mut::<3, 3>(3, 3).copy_from(r);
    symmetrize(&(b * p * b.transpose()))
}

/// Adds position uncertainty caused by a timestamp error of standard deviation `sigma_t_s`.
///
/// An estimate stamped `δ` seconds off is displaced by `v δ`, so the position
/// block grows by `v vᵀ σ_t²`.
pub fn add_clock_uncertainty(p: &mut Matrix6<f64>, vel: &Vector3<f64>, sigma_t_s: f64) {
    let add = vel * vel.transpose() * (sigma_t_s * sigma_t_s);
    let mut block = p.fixed_view_mut::<3, 3>(0, 0);
    block += add;
}

/// Process noise for a constant-velocity model driven by white acceleration.
///
/// `accel_psd` holds the acceleration power spectral density (m²/s³) along
/// each axis of `axes`, whose columns are those axes expressed in the
/// target frame. With `axes = I` the noise is in the target frame's own axes.
pub fn cv_process_noise(dt: f64, accel_psd: &Vector3<f64>, axes: &Matrix3<f64>) -> Matrix6<f64> {
    let dt = dt.abs();
    let (dt2, dt3) = (dt * dt, dt * dt * dt);
    let q = Matrix3::from_diagonal(accel_psd);
    let mut qa = Matrix6::zeros();
    qa.fixed_view_mut::<3, 3>(0, 0)
        .copy_from(&(q * (dt3 / 3.0)));
    qa.fixed_view_mut::<3, 3>(0, 3)
        .copy_from(&(q * (dt2 / 2.0)));
    qa.fixed_view_mut::<3, 3>(3, 0)
        .copy_from(&(q * (dt2 / 2.0)));
    qa.fixed_view_mut::<3, 3>(3, 3).copy_from(&(q * dt));
    rotate_state_cov(axes, &qa)
}

/// Constant-velocity transition matrix over `dt` seconds.
pub fn cv_transition(dt: f64) -> Matrix6<f64> {
    let mut f = Matrix6::identity();
    f[(0, 3)] = dt;
    f[(1, 4)] = dt;
    f[(2, 5)] = dt;
    f
}

/// Packs the upper triangle of a symmetric 6x6 matrix (row-major, 21 values).
pub fn pack_upper6(m: &Matrix6<f64>) -> [f64; 21] {
    let mut out = [0.0; 21];
    let mut k = 0;
    for i in 0..6 {
        for j in i..6 {
            out[k] = m[(i, j)];
            k += 1;
        }
    }
    out
}

/// Unpacks a symmetric 6x6 matrix from its packed upper triangle.
pub fn unpack_upper6(v: &[f64; 21]) -> Matrix6<f64> {
    let mut m = Matrix6::zeros();
    let mut k = 0;
    for i in 0..6 {
        for j in i..6 {
            m[(i, j)] = v[k];
            m[(j, i)] = v[k];
            k += 1;
        }
    }
    m
}

#[cfg(test)]
mod tests {
    use super::*;
    use rand::{Rng, SeedableRng};

    fn sf() -> Geodetic {
        Geodetic::new(37.7749, -122.4194, 10.0).unwrap()
    }

    /// Standard normal sample via Box-Muller (keeps tests free of extra crates).
    fn randn(rng: &mut impl Rng) -> f64 {
        let u1: f64 = rng.gen_range(1e-12..1.0);
        let u2: f64 = rng.gen();
        (-2.0 * u1.ln()).sqrt() * (2.0 * std::f64::consts::PI * u2).cos()
    }

    fn sample_gaussian6(
        rng: &mut impl Rng,
        mean: &Vector6<f64>,
        cov: &Matrix6<f64>,
    ) -> Vector6<f64> {
        let l = cov.cholesky().map(|c| c.l()).unwrap_or_else(Matrix6::zeros);
        let n = Vector6::from_fn(|_, _| randn(rng));
        mean + l * n
    }

    #[test]
    fn geodetic_ecef_round_trip_is_submillimeter() {
        for &(lat, lon, h) in &[
            (37.7749, -122.4194, 10.0),
            (0.0, 0.0, 0.0),
            (-33.86, 151.2, 50.0),
            (89.9999, 45.0, 300.0),
            (-89.5, -170.0, -20.0),
            (64.1, -21.9, 1200.0),
        ] {
            let g = Geodetic::new(lat, lon, h).unwrap();
            let back = g.to_ecef().to_geodetic();
            let again = back.to_ecef();
            assert!(
                g.to_ecef().distance(&again) < 1e-6,
                "round trip at ({lat},{lon})"
            );
            assert!((back.height_m - h).abs() < 1e-6);
        }
    }

    #[test]
    fn rejects_meters_passed_as_degrees() {
        // Regression for audit finding F-01: simulator meters were accepted as degrees by H3.
        assert!(matches!(
            Geodetic::new(1300.0, 0.0, 100.0),
            Err(FrameError::OutOfRange { .. })
        ));
        assert_eq!(
            Geodetic::new(f64::NAN, 0.0, 0.0),
            Err(FrameError::NonFinite)
        );
    }

    #[test]
    fn enu_axes_point_the_right_way() {
        let frame = EnuFrame::new(sf());
        let north = frame.to_ecef(&Vector3::new(0.0, 100.0, 0.0)).to_geodetic();
        let east = frame.to_ecef(&Vector3::new(100.0, 0.0, 0.0)).to_geodetic();
        let up = frame.to_ecef(&Vector3::new(0.0, 0.0, 100.0)).to_geodetic();
        assert!(north.lat_deg > sf().lat_deg);
        assert!(east.lon_deg > sf().lon_deg);
        assert!((up.height_m - 110.0).abs() < 1e-3);
        let enu = Vector3::new(12.5, -40.0, 3.0);
        let back = frame.from_ecef(&frame.to_ecef(&enu));
        assert!((back - enu).norm() < 1e-9);
    }

    #[test]
    fn same_point_from_two_local_frames_agrees_in_ecef() {
        // Two agents with different odometry frames observe one physical point.
        let site = EnuFrame::new(sf());
        let point_enu = Vector3::new(40.0, 25.0, 2.0);
        let a = site
            .ecef_from_enu_pose()
            .compose(&Pose3::from_yaw(0.3, Vector3::new(-10.0, 5.0, 0.0)));
        let b = site
            .ecef_from_enu_pose()
            .compose(&Pose3::from_yaw(-2.1, Vector3::new(200.0, -80.0, 1.5)));
        let in_a = a.inverse().transform_point(&site.to_ecef(&point_enu).0);
        let in_b = b.inverse().transform_point(&site.to_ecef(&point_enu).0);
        assert!((in_a - in_b).norm() > 10.0, "local coordinates differ");
        let ea = a.transform_point(&in_a);
        let eb = b.transform_point(&in_b);
        assert!((ea - eb).norm() < 1e-6);
    }

    #[test]
    fn exact_pose_transform_rotates_covariance_only() {
        let pose = PoseWithCov::exact(Pose3::from_yaw(0.7, Vector3::new(1.0, 2.0, 3.0)));
        let p = Matrix6::from_diagonal(&Vector6::new(4.0, 1.0, 0.25, 2.0, 2.0, 0.5));
        let (_, out) = pose.transform_state(&Vector6::new(10.0, 0.0, 0.0, 1.0, 0.0, 0.0), &p);
        assert!(
            (out.trace() - p.trace()).abs() < 1e-9,
            "rotation preserves trace"
        );
    }

    #[test]
    fn heading_error_scales_with_range() {
        let mut cov = Matrix6::zeros();
        cov[(5, 5)] = 1f64.to_radians().powi(2); // 1 degree yaw
        let pose = PoseWithCov {
            pose: Pose3::identity(),
            cov,
        };
        let near = pose
            .transform_state(
                &Vector6::new(10.0, 0.0, 0.0, 0.0, 0.0, 0.0),
                &Matrix6::zeros(),
            )
            .1;
        let far = pose
            .transform_state(
                &Vector6::new(50.0, 0.0, 0.0, 0.0, 0.0, 0.0),
                &Matrix6::zeros(),
            )
            .1;
        // Lateral (y) std at 50 m with 1 degree is 50 * 0.01745 = 0.873 m.
        assert!((far[(1, 1)].sqrt() - 0.8727).abs() < 1e-3);
        assert!((far[(1, 1)] / near[(1, 1)] - 25.0).abs() < 1e-9);
    }

    #[test]
    fn covariance_propagation_matches_monte_carlo() {
        // I3: analytic first-order covariance vs sample covariance of the exact transform.
        let mut rng = rand::rngs::StdRng::seed_from_u64(11);
        let site = EnuFrame::new(sf());
        let link = site
            .ecef_from_enu_pose()
            .compose(&Pose3::from_yaw(0.9, Vector3::new(30.0, -12.0, 1.0)));
        // Pose error: 0.5 m horizontal, 0.3 m vertical (in ENU), 1 deg yaw, 0.2 deg tilt.
        let r = site.ecef_from_enu();
        let mut cov_enu = Matrix6::zeros();
        cov_enu[(0, 0)] = 0.25;
        cov_enu[(1, 1)] = 0.25;
        cov_enu[(2, 2)] = 0.09;
        cov_enu[(3, 3)] = 0.2f64.to_radians().powi(2);
        cov_enu[(4, 4)] = 0.2f64.to_radians().powi(2);
        cov_enu[(5, 5)] = 1f64.to_radians().powi(2);
        let pose = PoseWithCov {
            pose: link,
            cov: rotate_state_cov(&r, &cov_enu),
        };
        let x = Vector6::new(45.0, 20.0, 1.5, 3.0, -1.0, 0.0);
        let p = Matrix6::from_diagonal(&Vector6::new(0.3, 0.2, 0.1, 0.5, 0.5, 0.1));
        let (mean, analytic) = pose.transform_state(&x, &p);

        let n = 60_000;
        let mut samples = Vec::with_capacity(n);
        for _ in 0..n {
            let d = sample_gaussian6(&mut rng, &Vector6::zeros(), &pose.cov);
            let xs = sample_gaussian6(&mut rng, &x, &p);
            let tp = pose.pose.perturbed(&d);
            let pp = tp.transform_point(&Vector3::new(xs[0], xs[1], xs[2]));
            let vv = tp.transform_vector(&Vector3::new(xs[3], xs[4], xs[5]));
            samples.push(Vector6::new(pp.x, pp.y, pp.z, vv.x, vv.y, vv.z));
        }
        let m: Vector6<f64> = samples.iter().fold(Vector6::zeros(), |a, s| a + s) / n as f64;
        let sample_cov = samples
            .iter()
            .fold(Matrix6::zeros(), |a, s| a + (s - m) * (s - m).transpose())
            / (n - 1) as f64;
        assert!((m - mean).norm() < 0.02, "mean bias {}", (m - mean).norm());
        let rel = (sample_cov - analytic).norm() / analytic.norm();
        assert!(rel < 0.05, "relative Frobenius error {rel}");
    }

    #[test]
    fn pose_composition_propagates_both_links() {
        let parent = PoseWithCov {
            pose: Pose3::from_yaw(0.2, Vector3::new(5.0, 0.0, 0.0)),
            cov: Matrix6::from_diagonal(&Vector6::new(0.01, 0.01, 0.01, 0.0, 0.0, 1e-4)),
        };
        let child = PoseWithCov {
            pose: Pose3::from_yaw(-0.4, Vector3::new(100.0, 0.0, 0.0)),
            cov: Matrix6::from_diagonal(&Vector6::new(0.04, 0.04, 0.0, 0.0, 0.0, 0.0)),
        };
        let both = parent.compose(&child);
        // A point at the child's origin: its covariance must equal both translation terms
        // plus the parent's yaw error levered over 100 m.
        let (_, p) = both.transform_state(&Vector6::zeros(), &Matrix6::zeros());
        let (_, p_two_step) = {
            let (x_mid, p_mid) = child.transform_state(&Vector6::zeros(), &Matrix6::zeros());
            parent.transform_state(&x_mid, &p_mid)
        };
        assert!((p - p_two_step).norm() < 1e-9);
        assert!(p.fixed_view::<3, 3>(0, 0).trace() > 0.01 * 3.0 + 0.04 * 2.0);
    }

    #[test]
    fn packing_round_trips() {
        let mut m = Matrix6::from_fn(|i, j| (i * 7 + j) as f64);
        m = symmetrize(&m);
        assert_eq!(unpack_upper6(&pack_upper6(&m)), m);
    }

    #[test]
    fn process_noise_is_rotation_covariant() {
        let r = *Rotation3::from_axis_angle(&Vector3::z_axis(), 1.1).matrix();
        let q_iso_h = Vector3::new(2.0, 2.0, 0.5);
        let a = cv_process_noise(0.1, &q_iso_h, &Matrix3::identity());
        let b = cv_process_noise(0.1, &q_iso_h, &r);
        // Horizontally isotropic noise is unchanged by a rotation about the vertical axis.
        assert!((a - b).norm() < 1e-12);
        assert!(is_spd(&(a + Matrix6::identity() * 1e-12)));
    }
}
