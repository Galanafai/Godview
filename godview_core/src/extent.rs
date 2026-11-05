//! Object extent: oriented 3D boxes with uncertainty.
//!
//! A box is optional everywhere. A point with a covariance remains the base
//! case (radar tracks, aircraft, anything a sensor reports without a size);
//! a box is added when the sensor provides one, as 3D detectors for road
//! users do. The box is centered on the estimated position, so the position
//! filters are unchanged; the box adds size and heading.
//!
//! * **Size** is length (along the heading), width and height, each with a
//!   variance. Sizes are nearly constant, so a track refines its size from
//!   every measurement with a small process noise that keeps it adaptive.
//! * **Heading** (`yaw`) is the direction of the length axis,
//!   counterclockwise from the frame's x axis: the odometry frame for
//!   detections and local tracks, local East (ENU at the object) for shared
//!   reports and world objects. Many detectors cannot tell front from back,
//!   so a heading can be marked ambiguous: known only modulo π. Motion
//!   resolves the ambiguity once the velocity is well determined.
//!
//! Boxes are used to refuse associations whose sizes or headings disagree
//! beyond their uncertainty (a car beside a truck must not merge even when
//! position alone cannot tell them apart), and they are fused across
//! sources with covariance intersection like positions.

use crate::frames::{ecef_from_enu_rotation, Ecef, PoseWithCov};
use nalgebra::{Matrix3, Vector3};
use std::f64::consts::{FRAC_PI_2, PI};

/// An oriented box with uncertainty.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ObjectBox {
    /// Length (along the heading), width, height (m).
    pub size: [f64; 3],
    /// Variances of `size` (m²).
    pub size_var: [f64; 3],
    /// Heading of the length axis (rad, counterclockwise from the frame's x axis), in `(-π, π]`.
    pub yaw: f64,
    /// Heading variance (rad²).
    pub yaw_var: f64,
    /// Heading known only modulo π (front and back indistinguishable).
    pub yaw_ambiguous: bool,
}

/// Smallest accepted size or heading variance; tighter claims are implausible.
pub const MIN_BOX_VARIANCE: f64 = 1e-6;
/// Largest accepted size (m).
pub const MAX_BOX_SIZE: f64 = 500.0;
/// Largest accepted size variance (m²) or heading variance (rad²).
pub const MAX_BOX_VARIANCE: f64 = 1e6;

impl ObjectBox {
    /// True if every value is finite and plausible.
    pub fn is_valid(&self) -> bool {
        self.size
            .iter()
            .all(|s| s.is_finite() && *s > 0.0 && *s <= MAX_BOX_SIZE)
            && self
                .size_var
                .iter()
                .chain(std::iter::once(&self.yaw_var))
                .all(|v| v.is_finite() && *v >= MIN_BOX_VARIANCE && *v <= MAX_BOX_VARIANCE)
            && self.yaw.is_finite()
            && self.yaw > -PI - 1e-6
            && self.yaw <= PI + 1e-6
    }

    /// The same box with every value rounded to `f32` precision, variances
    /// rounded up, so what a source keeps equals what receivers decode.
    pub fn quantized(&self) -> ObjectBox {
        let up = |v: f64| {
            let f = v.max(MIN_BOX_VARIANCE * 1.01) as f32;
            if (f as f64) < v {
                f32::from_bits(f.to_bits() + 1) as f64
            } else {
                f as f64
            }
        };
        ObjectBox {
            size: self.size.map(|s| s as f32 as f64),
            size_var: self.size_var.map(up),
            yaw: wrap_pi(self.yaw) as f32 as f64,
            yaw_var: up(self.yaw_var),
            yaw_ambiguous: self.yaw_ambiguous,
        }
    }

    /// The box with its heading rotated by `delta` (rad), variance increased by `extra_var`.
    pub fn rotated(&self, delta: f64, extra_var: f64) -> ObjectBox {
        ObjectBox {
            yaw: wrap_pi(self.yaw + delta),
            yaw_var: self.yaw_var + extra_var.max(0.0),
            ..*self
        }
    }
}

/// Wraps an angle into `(-π, π]`.
pub fn wrap_pi(a: f64) -> f64 {
    let mut r = a.rem_euclid(2.0 * PI);
    if r > PI {
        r -= 2.0 * PI;
    }
    r
}

/// Wraps an angle into `(-π/2, π/2]`.
pub fn wrap_half_pi(a: f64) -> f64 {
    let mut r = a.rem_euclid(PI);
    if r > FRAC_PI_2 {
        r -= PI;
    }
    r
}

/// Heading residual `a - b`, modulo π if either heading is ambiguous.
pub fn yaw_residual(a: f64, b: f64, ambiguous: bool) -> f64 {
    if ambiguous {
        wrap_half_pi(a - b)
    } else {
        wrap_pi(a - b)
    }
}

/// Gates for box consistency (chi-square thresholds).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BoxGate {
    /// Size difference gate, 3 DOF (21.11 = 99.99%). `f64::INFINITY` disables it.
    pub size_chi2: f64,
    /// Heading difference gate, 1 DOF (15.13 = 99.99%). `f64::INFINITY` disables it.
    pub yaw_chi2: f64,
}

impl Default for BoxGate {
    fn default() -> Self {
        Self {
            size_chi2: 21.11,
            yaw_chi2: 15.13,
        }
    }
}

impl BoxGate {
    /// A gate that never refuses (for ablations).
    pub fn disabled() -> Self {
        Self {
            size_chi2: f64::INFINITY,
            yaw_chi2: f64::INFINITY,
        }
    }

    /// True if two boxes may describe the same object. Missing boxes never refuse.
    pub fn passes(&self, a: Option<&ObjectBox>, b: Option<&ObjectBox>) -> bool {
        let (Some(a), Some(b)) = (a, b) else {
            return true;
        };
        let size: f64 = (0..3)
            .map(|i| (a.size[i] - b.size[i]).powi(2) / (a.size_var[i] + b.size_var[i]))
            .sum();
        if size.is_nan() || size >= self.size_chi2 {
            return false;
        }
        let dy = yaw_residual(a.yaw, b.yaw, a.yaw_ambiguous || b.yaw_ambiguous);
        let yaw = dy * dy / (a.yaw_var + b.yaw_var);
        !(yaw.is_nan() || yaw >= self.yaw_chi2)
    }
}

/// Process noise for a box estimate.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BoxModel {
    /// Size random-walk spectral density (m²/s); keeps sizes adaptive.
    pub size_q: f64,
    /// Heading random-walk spectral density (rad²/s); covers turning.
    pub yaw_q: f64,
    /// Speed above which, if well determined, motion fixes the heading (m/s).
    pub resolve_speed: f64,
    /// Minimum track age before motion may fix the heading (s). A young
    /// track's velocity is far less certain than its covariance claims (a
    /// 0.15 s old track reported 10.3 ± 2.6 m/s for an object moving at
    /// 1.6 m/s, DST `large_objects` seed 195335910), and a resolution is kept.
    pub resolve_after_s: f64,
}

impl Default for BoxModel {
    fn default() -> Self {
        Self {
            size_q: 0.01,
            yaw_q: 0.05,
            resolve_speed: 1.0,
            resolve_after_s: 1.0,
        }
    }
}

impl BoxModel {
    /// The box predicted `dt` seconds ahead (sizes and heading constant, variances grow).
    pub fn predict(&self, b: &ObjectBox, dt: f64) -> ObjectBox {
        let dt = dt.abs();
        ObjectBox {
            size_var: b.size_var.map(|v| v + self.size_q * dt),
            yaw_var: b.yaw_var + self.yaw_q * dt,
            ..*b
        }
    }

    /// Kalman update of an estimate with a measurement (both in the same frame).
    ///
    /// Sizes update per axis. The heading updates with the residual wrapped
    /// modulo π when either side is ambiguous; the result stays ambiguous
    /// only if both were.
    pub fn update(&self, est: &ObjectBox, meas: &ObjectBox) -> ObjectBox {
        let mut out = *est;
        for i in 0..3 {
            let k = est.size_var[i] / (est.size_var[i] + meas.size_var[i]);
            out.size[i] = est.size[i] + k * (meas.size[i] - est.size[i]);
            out.size_var[i] = (1.0 - k) * est.size_var[i];
        }
        let ambiguous = est.yaw_ambiguous || meas.yaw_ambiguous;
        let r = yaw_residual(meas.yaw, est.yaw, ambiguous);
        let k = est.yaw_var / (est.yaw_var + meas.yaw_var);
        out.yaw = wrap_pi(est.yaw + k * r);
        out.yaw_var = (1.0 - k) * est.yaw_var;
        out.yaw_ambiguous = est.yaw_ambiguous && meas.yaw_ambiguous;
        out
    }

    /// Uses the direction of travel to pick front from back.
    ///
    /// If the object moves faster than `resolve_speed` and the speed exceeds
    /// three standard deviations of its uncertainty, the heading is flipped
    /// by π if it points against the motion, and marked unambiguous. The
    /// velocity is not fused as a heading measurement (objects can move
    /// sideways), so the estimators stay independent. The caller applies
    /// `resolve_after_s`: young tracks must not decide.
    pub fn resolve_with_velocity(
        &self,
        b: &ObjectBox,
        vx: f64,
        vy: f64,
        speed_var: f64,
    ) -> ObjectBox {
        let speed = (vx * vx + vy * vy).sqrt();
        if speed < self.resolve_speed || speed * speed < 9.0 * speed_var {
            return *b;
        }
        let heading = vy.atan2(vx);
        let mut out = *b;
        if wrap_pi(b.yaw - heading).abs() > FRAC_PI_2 {
            out.yaw = wrap_pi(b.yaw + PI);
        }
        out.yaw_ambiguous = false;
        out
    }
}

/// Adds a detector's systematic size and heading errors to a box's variances.
///
/// The size counterpart of [`center_sys_cov`]: a detector (or an annotator)
/// that gets an object's size or heading wrong by the same amount on every
/// scan produces an error that the track's filter cannot average out. Two
/// sources' converged tracks of one object then disagree by more than their
/// shrinking variances allow, and the box gate splits the object. `size_m`
/// is the standard deviation of the size error (m, every dimension);
/// `yaw_sys_rad` is that of the heading error. Meant for published boxes,
/// not for the per-scan measurement noise.
///
/// The size error is absolute on purpose. A variance proportional to each
/// source's own size estimate makes a source that underestimates an object
/// also claim to be more certain, and covariance intersection, which favors
/// the more certain input, then pulls the fused size toward the smallest
/// estimate with an understated variance (seen in `large_objects`: pooled
/// size NEES up to 8.8 against 3 expected).
pub fn with_systematic(b: &ObjectBox, size_m: f64, yaw_sys_rad: f64) -> ObjectBox {
    let mut out = *b;
    for v in out.size_var.iter_mut() {
        *v += size_m * size_m;
    }
    out.yaw_var += yaw_sys_rad * yaw_sys_rad;
    out
}

/// Covariance (in the box's frame) of a systematic center error that does
/// not average out over scans.
///
/// A detector that sees only part of an object places the center toward the
/// visible side, scan after scan. A filter that treats that error as
/// independent noise shrinks its covariance below the real error, and two
/// agents viewing an object from opposite ends then report confident
/// centers that disagree. `frac` is the standard deviation of that error as
/// a fraction of each half-dimension (length and width; height is left to
/// the detector), and the result is meant to be added to a published
/// position covariance, not to the per-scan measurement noise.
pub fn center_sys_cov(b: &ObjectBox, frac: f64) -> Matrix3<f64> {
    if frac <= 0.0 {
        return Matrix3::zeros();
    }
    let (c, s) = (b.yaw.cos(), b.yaw.sin());
    let along = Vector3::new(c, s, 0.0);
    let across = Vector3::new(-s, c, 0.0);
    let sl = frac * b.size[0] / 2.0;
    let sw = frac * b.size[1] / 2.0;
    along * along.transpose() * (sl * sl) + across * across.transpose() * (sw * sw)
}

/// Re-expresses a box from an agent's odometry frame in local East-North-Up
/// at the object, the frame shared boxes use.
///
/// The heading's direction vector is rotated into ECEF with the agent's
/// estimated pose and its angle is read in ENU at `at` (the object's ECEF
/// position). The pose's rotation uncertainty about the local up axis is
/// added to the heading variance. Sizes are unchanged by a rigid transform.
pub fn box_to_enu(b: &ObjectBox, ecef_from_odom: &PoseWithCov, at: &Ecef) -> ObjectBox {
    let g = at.to_geodetic();
    let r_enu = ecef_from_enu_rotation(g.lat_deg, g.lon_deg);
    let h_odom = Vector3::new(b.yaw.cos(), b.yaw.sin(), 0.0);
    let h_enu = r_enu.transpose() * (ecef_from_odom.pose.rotation * h_odom);
    let horizontal = (h_enu.x * h_enu.x + h_enu.y * h_enu.y).sqrt();
    let up: Vector3<f64> = r_enu.column(2).into_owned();
    let rot_cov: Matrix3<f64> = ecef_from_odom.cov.fixed_view::<3, 3>(3, 3).into_owned();
    let pose_var = up.dot(&(rot_cov * up)).max(0.0);
    let mut out = *b;
    if horizontal < 0.1 {
        // The odometry frame is tipped so far that the heading is nearly vertical.
        out.yaw = 0.0;
        out.yaw_var = PI * PI;
        return out;
    }
    out.yaw = h_enu.y.atan2(h_enu.x);
    out.yaw_var = b.yaw_var + pose_var;
    out
}

/// Covariance intersection of two boxes in the same frame.
///
/// Sizes: diagonal CI with the weight that minimizes the summed fused
/// variance. Heading: for a scalar, CI keeps the more certain input, after
/// aligning the other modulo π if either is ambiguous. CI is conservative
/// under unknown correlation (both sources may run the same detector).
pub fn fuse_boxes(a: &ObjectBox, b: &ObjectBox) -> ObjectBox {
    let fused_var = |w: f64, i: usize| 1.0 / (w / a.size_var[i] + (1.0 - w) / b.size_var[i]);
    let cost = |w: f64| (0..3).map(|i| fused_var(w, i)).sum::<f64>();
    // Golden section on [0, 1]; ties go to 0.5 so equal inputs fuse symmetrically.
    let (mut lo, mut hi) = (0.0f64, 1.0f64);
    let g = 0.618_033_988_749_894_9;
    for _ in 0..60 {
        let m1 = hi - g * (hi - lo);
        let m2 = lo + g * (hi - lo);
        if cost(m1) <= cost(m2) {
            hi = m2;
        } else {
            lo = m1;
        }
    }
    let mut w = 0.5 * (lo + hi);
    if (cost(w) - cost(0.5)).abs() <= 1e-12 * cost(0.5) {
        w = 0.5;
    }
    let mut out = *a;
    for i in 0..3 {
        let v = fused_var(w, i);
        out.size[i] = v * (w * a.size[i] / a.size_var[i] + (1.0 - w) * b.size[i] / b.size_var[i]);
        out.size_var[i] = v;
    }
    let ambiguous = a.yaw_ambiguous || b.yaw_ambiguous;
    // Two sources that each claim to know front from back, and disagree, leave
    // the fused heading ambiguous: one wrongly resolved source must not turn
    // the shared heading around for everyone.
    let conflict = !a.yaw_ambiguous && !b.yaw_ambiguous && wrap_pi(a.yaw - b.yaw).abs() > FRAC_PI_2;
    let (keep, other) = if a.yaw_var <= b.yaw_var {
        (a, b)
    } else {
        (b, a)
    };
    // The kept heading may still be flipped by the other if only the kept one is ambiguous.
    let mut yaw = keep.yaw;
    if keep.yaw_ambiguous && !other.yaw_ambiguous && wrap_pi(other.yaw - yaw).abs() > FRAC_PI_2 {
        yaw = wrap_pi(yaw + PI);
    }
    out.yaw = yaw;
    out.yaw_var = keep.yaw_var;
    out.yaw_ambiguous = conflict || (ambiguous && keep.yaw_ambiguous && other.yaw_ambiguous);
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn bx(l: f64, yaw: f64, amb: bool) -> ObjectBox {
        ObjectBox {
            size: [l, 2.0, 1.6],
            size_var: [0.04; 3],
            yaw,
            yaw_var: 0.01,
            yaw_ambiguous: amb,
        }
    }

    #[test]
    fn systematic_errors_widen_variances_and_let_biased_sources_pass_the_gate() {
        // Two sources whose converged tracks of one car disagree by 0.5 m in
        // length (an annotator or detector bias) and 0.1 rad in heading.
        let mut a = bx(4.5, 0.0, false);
        let mut b = bx(5.0, 0.1, false);
        for x in [&mut a, &mut b] {
            x.size_var = [1e-3; 3];
            x.yaw_var = 1e-4;
        }
        let g = BoxGate::default();
        assert!(!g.passes(Some(&a), Some(&b)), "confident tracks are split");
        let (sa, sb) = (
            with_systematic(&a, 0.3, 0.05),
            with_systematic(&b, 0.3, 0.05),
        );
        assert!((sa.size_var[0] - (1e-3 + 0.09)).abs() < 1e-12);
        assert!((sa.yaw_var - (1e-4 + 0.0025)).abs() < 1e-12);
        assert_eq!((sa.size, sa.yaw), (a.size, a.yaw), "only variances change");
        assert!(
            g.passes(Some(&sa), Some(&sb)),
            "declared bias lets them fuse"
        );
        // A real truck is still not a car.
        let truck = with_systematic(&bx(12.0, 0.0, false), 0.3, 0.05);
        assert!(!g.passes(Some(&sa), Some(&truck)));
    }

    #[test]
    fn sources_that_disagree_on_front_and_back_leave_the_heading_ambiguous() {
        let forward = bx(4.5, 0.1, false);
        let backward = bx(4.5, 0.1 + PI, false);
        let f = fuse_boxes(&forward, &backward);
        assert!(
            f.yaw_ambiguous,
            "conflicting resolutions must not pick a side"
        );
        assert!(wrap_half_pi(f.yaw - 0.1).abs() < 1e-9, "axis still known");
        assert!(!fuse_boxes(&forward, &bx(4.5, 0.15, false)).yaw_ambiguous);
    }

    #[test]
    fn wrapping() {
        assert!((wrap_pi(3.0 * PI) - PI).abs() < 1e-12);
        assert!((wrap_pi(-PI) - PI).abs() < 1e-12);
        assert!((wrap_half_pi(PI - 0.1) + 0.1).abs() < 1e-12);
        assert!(
            (yaw_residual(0.1, PI, true) - 0.1).abs() < 1e-12,
            "front/back flip is no residual when ambiguous"
        );
        assert!((yaw_residual(0.1, PI, false).abs() - (PI - 0.1)).abs() < 1e-12);
    }

    #[test]
    fn gate_refuses_a_truck_for_a_car_and_a_crossing_heading() {
        let g = BoxGate::default();
        let car = bx(4.5, 0.0, false);
        assert!(g.passes(Some(&car), Some(&bx(4.7, 0.05, false))));
        assert!(!g.passes(Some(&car), Some(&bx(12.0, 0.0, false))), "size");
        assert!(
            !g.passes(Some(&car), Some(&bx(4.5, FRAC_PI_2, false))),
            "heading"
        );
        assert!(
            g.passes(Some(&car), Some(&bx(4.5, PI, true))),
            "flipped but ambiguous"
        );
        assert!(g.passes(Some(&car), None), "a missing box never refuses");
        assert!(BoxGate::disabled().passes(Some(&car), Some(&bx(12.0, 1.0, false))));
    }

    #[test]
    fn update_converges_and_handles_ambiguity() {
        let m = BoxModel::default();
        let mut est = bx(5.0, 0.2, true);
        est.size_var = [1.0; 3];
        for k in 0..50 {
            // Ambiguous measurements, half of them flipped by π.
            let flip = if k % 2 == 0 { 0.0 } else { PI };
            est = m.update(&m.predict(&est, 0.1), &bx(4.5, wrap_pi(0.25 + flip), true));
        }
        assert!((est.size[0] - 4.5).abs() < 0.05, "{:?}", est.size);
        assert!(wrap_half_pi(est.yaw - 0.25).abs() < 0.02, "yaw {}", est.yaw);
        assert!(est.yaw_ambiguous);
        // Motion pointing the other way resolves front and back.
        let r = m.resolve_with_velocity(&est, -3.0 * 0.25f64.cos(), -3.0 * 0.25f64.sin(), 0.01);
        assert!(!r.yaw_ambiguous);
        assert!(wrap_pi(r.yaw - (0.25 - PI)).abs() < 0.02, "yaw {}", r.yaw);
        // Uncertain or slow motion does not.
        assert!(m.resolve_with_velocity(&est, 0.5, 0.0, 0.0).yaw_ambiguous);
        assert!(m.resolve_with_velocity(&est, 3.0, 0.0, 4.0).yaw_ambiguous);
    }

    #[test]
    fn fusion_is_conservative_and_symmetric() {
        let a = bx(4.4, 0.1, false);
        let b = bx(4.6, 0.12, false);
        let f = fuse_boxes(&a, &b);
        assert!(
            (f.size[0] - 4.5).abs() < 1e-9,
            "equal weights for equal variances"
        );
        assert!(
            f.size_var[0] >= 0.04 * 0.999,
            "CI never claims more than the best input"
        );
        let same = fuse_boxes(&a, &a);
        assert_eq!(same.size, a.size);
        assert!(
            same.size_var[0] >= a.size_var[0] * 0.999,
            "fusing a box with itself gains nothing"
        );
        // The more certain heading wins; an ambiguous one is flipped by an unambiguous one.
        let mut sharp = bx(4.5, wrap_pi(0.1 + PI), true);
        sharp.yaw_var = 0.001;
        let f = fuse_boxes(&sharp, &a);
        assert!(
            wrap_pi(f.yaw - 0.1).abs() < 1e-9 && !f.yaw_ambiguous,
            "yaw {}",
            f.yaw
        );
    }

    #[test]
    fn odometry_heading_maps_to_east_north_up() {
        use crate::frames::{EnuFrame, Geodetic, Pose3};
        let site = EnuFrame::new(Geodetic::new(37.7749, -122.4194, 10.0).unwrap());
        // An odometry frame rotated 0.7 rad from ENU: a box at odometry yaw 0.2
        // points 0.9 rad from East.
        let odom = Pose3::from_yaw(0.7, Vector3::new(30.0, -10.0, 2.0));
        let mut pose = PoseWithCov::exact(site.ecef_from_enu_pose().compose(&odom));
        let at = site.to_ecef(&Vector3::new(40.0, 0.0, 1.0));
        let b = bx(4.5, 0.2, false);
        let e = box_to_enu(&b, &pose, &at);
        assert!((e.yaw - 0.9).abs() < 1e-4, "yaw {}", e.yaw);
        assert_eq!(e.yaw_var, b.yaw_var);
        // Pose heading uncertainty adds to the box's heading variance.
        let r = site.ecef_from_enu();
        let mut rot = Matrix3::zeros();
        rot[(2, 2)] = 0.0004;
        pose.cov
            .fixed_view_mut::<3, 3>(3, 3)
            .copy_from(&(r * rot * r.transpose()));
        let e = box_to_enu(&b, &pose, &at);
        assert!(
            (e.yaw_var - (b.yaw_var + 0.0004)).abs() < 1e-6,
            "var {}",
            e.yaw_var
        );
    }

    #[test]
    fn systematic_center_covariance_follows_the_box_axes() {
        let b = bx(12.0, FRAC_PI_2, false);
        let c = center_sys_cov(&b, 0.25);
        // Length along y (north), width along x.
        assert!((c[(1, 1)] - 1.5f64.powi(2)).abs() < 1e-9);
        assert!((c[(0, 0)] - 0.25f64.powi(2)).abs() < 1e-9);
        assert!(c[(2, 2)].abs() < 1e-12 && c[(0, 1)].abs() < 1e-9);
        assert_eq!(center_sys_cov(&b, 0.0), Matrix3::zeros());
    }

    #[test]
    fn validation_and_quantization() {
        let a = bx(4.5, 0.3, false);
        assert!(a.is_valid());
        let q = a.quantized();
        assert!(q.is_valid() && q.size_var[0] >= a.size_var[0]);
        let mut bad = a;
        bad.size[0] = -1.0;
        assert!(!bad.is_valid());
        bad = a;
        bad.yaw_var = f64::NAN;
        assert!(!bad.is_valid());
    }
}
