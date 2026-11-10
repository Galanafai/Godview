//! External detection input as JSON lines (one frame per line).
//!
//! This is how an outside perception stack (a detector on a GPU box, a
//! recorded log, a simulator bridge) feeds a node:
//!
//! ```json
//! {"t_unix_ns": 1761200000000000000,
//!  "pose": {"lat_deg": 37.7749, "lon_deg": -122.4194, "height_m": 10.0,
//!           "yaw_deg": 30.0, "sigma_h_m": 0.1, "sigma_v_m": 0.2, "sigma_yaw_deg": 0.3},
//!  "detections": [{"x": 12.0, "y": -3.0, "z": 1.0, "sigma_m": 0.3,
//!                  "class_id": 1, "class_prob": 0.9,
//!                  "box": {"length_m": 4.5, "width_m": 1.8, "height_m": 1.5,
//!                          "yaw_deg": 10.0, "sigma_size_m": 0.2,
//!                          "sigma_yaw_deg": 3.0, "yaw_ambiguous": true}}]}
//! ```
//!
//! Detections are positions (m) in the agent's odometry frame: a smooth
//! frame fixed to the world locally, like ROS `odom`. The tracker runs in
//! that frame, so it must not jump between lines. `pose` places that frame
//! on the Earth, like ROS `map -> odom`: its origin at the geodetic point,
//! its x axis `yaw_deg` counterclockwise from East, z up, with the
//! uncertainty of that placement. `agent_xyz` is the agent's own position in
//! the odometry frame (default: the origin); it decides which topic cells
//! the node subscribes to. `pose` may be `null` when the agent has no global
//! localization; its tracks are then kept local and not shared. `box` is
//! optional: an oriented box with its heading in the odometry frame.

use godview_core::extent::{wrap_pi, ObjectBox};
use godview_core::frames::{rotate_state_cov, EnuFrame, FrameError, Geodetic, Pose3, PoseWithCov};
use godview_core::local_tracker::Detection;
use godview_core::localization::{LocalizationStatus, PoseEstimate};
use godview_env::Timestamp;
use nalgebra::{Matrix3, Matrix6, Vector3};
use serde::{Deserialize, Serialize};

/// One input line.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct InputFrame {
    /// Measurement time (ns since the Unix epoch).
    pub t_unix_ns: i64,
    /// Agent pose for this frame, if globally localized.
    pub pose: Option<InputPose>,
    /// Detections in the odometry frame.
    #[serde(default)]
    pub detections: Vec<InputDetection>,
    /// The agent's position in the odometry frame (m).
    #[serde(default)]
    pub agent_xyz: [f64; 3],
}

/// Geodetic placement of the odometry frame, with its uncertainty.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct InputPose {
    /// Latitude (degrees).
    pub lat_deg: f64,
    /// Longitude (degrees).
    pub lon_deg: f64,
    /// Height above the ellipsoid (m).
    pub height_m: f64,
    /// Heading of the frame's x axis, counterclockwise from East (degrees).
    pub yaw_deg: f64,
    /// Horizontal position standard deviation (m).
    pub sigma_h_m: f64,
    /// Vertical position standard deviation (m).
    pub sigma_v_m: f64,
    /// Yaw standard deviation (degrees).
    pub sigma_yaw_deg: f64,
}

/// One detection in the odometry frame.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct InputDetection {
    /// Odometry-frame x (m).
    pub x: f64,
    /// Odometry-frame y (m).
    pub y: f64,
    /// Odometry-frame z, up (m).
    pub z: f64,
    /// Isotropic position standard deviation (m). With `sigma_along_m` and
    /// `sigma_across_m` it gives only the vertical part (unless `sigma_z_m`).
    pub sigma_m: f64,
    /// Horizontal standard deviation along the box heading (m). Needs `box`
    /// and `sigma_across_m`; the covariance is rotated by the box heading
    /// (unchanged by a 180 degree flip, so `yaw_ambiguous` does not matter).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub sigma_along_m: Option<f64>,
    /// Horizontal standard deviation across the box heading (m).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub sigma_across_m: Option<f64>,
    /// Vertical standard deviation (m), with the along / across pair.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub sigma_z_m: Option<f64>,
    /// Class id.
    pub class_id: u8,
    /// Class confidence in `[0, 1]`.
    pub class_prob: f32,
    /// Oriented box, if the detector reports extent (JSON key `box`).
    #[serde(rename = "box", default, skip_serializing_if = "Option::is_none")]
    pub object_box: Option<InputBox>,
}

/// An oriented box in the odometry frame.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct InputBox {
    /// Length along the heading, width, height (m).
    pub length_m: f64,
    /// Width (m).
    pub width_m: f64,
    /// Height (m).
    pub height_m: f64,
    /// Heading of the length axis, counterclockwise from the odometry x axis (degrees).
    pub yaw_deg: f64,
    /// Size standard deviation, each dimension (m).
    pub sigma_size_m: f64,
    /// Heading standard deviation (degrees).
    pub sigma_yaw_deg: f64,
    /// The detector cannot tell front from back (heading known modulo 180 degrees).
    #[serde(default)]
    pub yaw_ambiguous: bool,
}

/// Errors in an input line.
#[derive(Debug, thiserror::Error)]
pub enum InputError {
    /// Not valid JSON for [`InputFrame`].
    #[error("bad input line: {0}")]
    Json(#[from] serde_json::Error),
    /// Invalid pose coordinates.
    #[error("bad pose: {0}")]
    Pose(#[from] FrameError),
    /// A standard deviation was negative or not finite.
    #[error("bad uncertainty: {0}")]
    Uncertainty(&'static str),
}

/// A parsed frame ready for the tracker.
#[derive(Debug, Clone)]
pub struct ParsedFrame {
    /// Measurement time.
    pub t: Timestamp,
    /// Pose of the frame the detections are in, if global.
    pub pose: Option<PoseEstimate>,
    /// Detections, in the odometry frame.
    pub detections: Vec<Detection>,
    /// The agent's position in the odometry frame (m).
    pub agent_odom: Vector3<f64>,
}

/// Parses one line. `next_obs` supplies unique observation ids.
pub fn parse_line(line: &str, next_obs: &mut u64) -> Result<ParsedFrame, InputError> {
    let frame: InputFrame = serde_json::from_str(line)?;
    let t = Timestamp(frame.t_unix_ns);
    let pose = frame.pose.map(|p| pose_estimate(&p, t)).transpose()?;
    let mut detections = Vec::with_capacity(frame.detections.len());
    for d in &frame.detections {
        if !(d.sigma_m.is_finite() && d.sigma_m > 0.0) {
            return Err(InputError::Uncertainty("detection sigma_m"));
        }
        let object_box = d.object_box.map(|b| b.to_box()).transpose()?;
        let r = measurement_cov(d, object_box.as_ref())?;
        *next_obs += 1;
        detections.push(Detection {
            obs_id: *next_obs,
            t,
            z: Vector3::new(d.x, d.y, d.z),
            r,
            class_id: d.class_id,
            class_prob: d.class_prob.clamp(0.0, 1.0),
            object_box,
        });
    }
    Ok(ParsedFrame {
        t,
        pose,
        detections,
        agent_odom: Vector3::from(frame.agent_xyz),
    })
}

/// Position covariance of a detection: isotropic, or along / across its box.
fn measurement_cov(d: &InputDetection, b: Option<&ObjectBox>) -> Result<Matrix3<f64>, InputError> {
    let ok = |v: f64| v.is_finite() && v > 0.0;
    match (d.sigma_along_m, d.sigma_across_m) {
        (None, None) => {
            if d.sigma_z_m.is_some() {
                return Err(InputError::Uncertainty("sigma_z_m needs sigma_along_m and sigma_across_m"));
            }
            Ok(Matrix3::identity() * d.sigma_m * d.sigma_m)
        }
        (Some(sl), Some(sc)) => {
            let sz = d.sigma_z_m.unwrap_or(d.sigma_m);
            if !(ok(sl) && ok(sc) && ok(sz)) {
                return Err(InputError::Uncertainty("detection sigma_along_m / sigma_across_m / sigma_z_m"));
            }
            let b = b.ok_or(InputError::Uncertainty("sigma_along_m needs a box"))?;
            let (c, s) = (b.yaw.cos(), b.yaw.sin());
            let rot = Matrix3::new(c, -s, 0.0, s, c, 0.0, 0.0, 0.0, 1.0);
            let diag = Matrix3::from_diagonal(&Vector3::new(sl * sl, sc * sc, sz * sz));
            Ok(rot * diag * rot.transpose())
        }
        _ => Err(InputError::Uncertainty("sigma_along_m and sigma_across_m go together")),
    }
}

impl InputBox {
    fn to_box(self) -> Result<ObjectBox, InputError> {
        let b = ObjectBox {
            size: [self.length_m, self.width_m, self.height_m],
            size_var: [self.sigma_size_m.powi(2); 3],
            yaw: wrap_pi(self.yaw_deg.to_radians()),
            yaw_var: self.sigma_yaw_deg.to_radians().powi(2),
            yaw_ambiguous: self.yaw_ambiguous,
        };
        if b.is_valid() {
            Ok(b)
        } else {
            Err(InputError::Uncertainty(
                "box (sizes must be positive, sigmas positive)",
            ))
        }
    }
}

fn pose_estimate(p: &InputPose, t: Timestamp) -> Result<PoseEstimate, InputError> {
    for (v, name) in [
        (p.sigma_h_m, "pose sigma_h_m"),
        (p.sigma_v_m, "pose sigma_v_m"),
        (p.sigma_yaw_deg, "pose sigma_yaw_deg"),
    ] {
        if !(v.is_finite() && v >= 0.0) {
            return Err(InputError::Uncertainty(name));
        }
    }
    let frame = EnuFrame::new(Geodetic::new(p.lat_deg, p.lon_deg, p.height_m)?);
    let pose = frame
        .ecef_from_enu_pose()
        .compose(&Pose3::from_yaw(p.yaw_deg.to_radians(), Vector3::zeros()));
    // Tangent ordering is [translation, rotation]; tilt is taken as half the yaw error.
    let syaw = p.sigma_yaw_deg.to_radians();
    let mut cov_enu = Matrix6::zeros();
    cov_enu[(0, 0)] = p.sigma_h_m.powi(2);
    cov_enu[(1, 1)] = p.sigma_h_m.powi(2);
    cov_enu[(2, 2)] = p.sigma_v_m.powi(2);
    cov_enu[(3, 3)] = (0.5 * syaw).powi(2);
    cov_enu[(4, 4)] = (0.5 * syaw).powi(2);
    cov_enu[(5, 5)] = syaw * syaw;
    Ok(PoseEstimate {
        t,
        ecef_from_odom: PoseWithCov {
            pose,
            cov: rotate_state_cov(&frame.ecef_from_enu(), &cov_enu),
        },
        status: LocalizationStatus::Global,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    const LINE: &str = r#"{"t_unix_ns": 1761200000000000000,
        "pose": {"lat_deg": 37.7749, "lon_deg": -122.4194, "height_m": 10.0, "yaw_deg": 90.0,
                 "sigma_h_m": 0.1, "sigma_v_m": 0.2, "sigma_yaw_deg": 0.3},
        "detections": [{"x": 10.0, "y": 0.0, "z": 0.0, "sigma_m": 0.3, "class_id": 1, "class_prob": 0.9}]}"#;

    #[test]
    fn parses_and_places_detections_in_the_world() {
        let mut next = 0;
        let f = parse_line(&LINE.replace('\n', " "), &mut next).unwrap();
        assert_eq!(f.detections.len(), 1);
        assert_eq!(next, 1);
        let pose = f.pose.unwrap();
        // Yaw 90 degrees: the frame's x axis points North, so x = 10 m is 10 m north.
        let site = EnuFrame::new(Geodetic::new(37.7749, -122.4194, 10.0).unwrap());
        let world = pose.ecef_from_odom.pose.transform_point(&f.detections[0].z);
        let enu = site.from_ecef(&godview_core::frames::Ecef(world));
        assert!(
            (enu - Vector3::new(0.0, 10.0, 0.0)).norm() < 1e-6,
            "{enu:?}"
        );
        assert!(pose.ecef_from_odom.cov[(0, 0)] > 0.0);
    }

    #[test]
    fn boxes_parse_and_bad_boxes_are_rejected() {
        let mut next = 0;
        let line = LINE.replace(
            "\"class_prob\": 0.9}",
            "\"class_prob\": 0.9, \"box\": {\"length_m\": 12.0, \"width_m\": 2.5, \"height_m\": 3.5, \"yaw_deg\": 90.0, \"sigma_size_m\": 0.2, \"sigma_yaw_deg\": 3.0, \"yaw_ambiguous\": true}}",
        )
        .replace('\n', " ");
        let f = parse_line(&line, &mut next).unwrap();
        let b = f.detections[0].object_box.unwrap();
        assert_eq!(b.size, [12.0, 2.5, 3.5]);
        assert!((b.yaw - std::f64::consts::FRAC_PI_2).abs() < 1e-12 && b.yaw_ambiguous);
        let bad = line.replace("\"length_m\": 12.0", "\"length_m\": -1.0");
        assert!(matches!(
            parse_line(&bad, &mut next),
            Err(InputError::Uncertainty(_))
        ));
    }

    #[test]
    fn anisotropic_covariance_follows_the_box_heading() {
        let mut next = 0;
        let line = LINE
            .replace(
                "\"class_prob\": 0.9}",
                "\"class_prob\": 0.9, \"sigma_along_m\": 0.4, \"sigma_across_m\": 0.1, \"sigma_z_m\": 0.05, \"box\": {\"length_m\": 4.5, \"width_m\": 1.8, \"height_m\": 1.5, \"yaw_deg\": 90.0, \"sigma_size_m\": 0.2, \"sigma_yaw_deg\": 3.0, \"yaw_ambiguous\": true}}",
            )
            .replace('\n', " ");
        let f = parse_line(&line, &mut next).unwrap();
        let r = f.detections[0].r;
        // Heading along odometry y: the large sigma is on y.
        assert!((r[(1, 1)] - 0.16).abs() < 1e-12 && (r[(0, 0)] - 0.01).abs() < 1e-12);
        assert!((r[(2, 2)] - 0.0025).abs() < 1e-12 && r[(0, 1)].abs() < 1e-12);
        // A flipped heading gives the same covariance.
        let flipped = line.replace("\"yaw_deg\": 90.0, \"sigma_size_m\"", "\"yaw_deg\": -90.0, \"sigma_size_m\"");
        let g = parse_line(&flipped, &mut next).unwrap();
        assert!((g.detections[0].r - r).norm() < 1e-12);
        // Without a box, or with only one of the pair, the line is rejected.
        let no_box = LINE
            .replace("\"class_prob\": 0.9}", "\"class_prob\": 0.9, \"sigma_along_m\": 0.4, \"sigma_across_m\": 0.1}")
            .replace('\n', " ");
        assert!(matches!(parse_line(&no_box, &mut next), Err(InputError::Uncertainty(_))));
        let half = line.replace("\"sigma_across_m\": 0.1, ", "");
        assert!(matches!(parse_line(&half, &mut next), Err(InputError::Uncertainty(_))));
    }

    #[test]
    fn null_pose_is_local_only() {
        let mut next = 0;
        let f = parse_line(
            r#"{"t_unix_ns": 5, "pose": null, "detections": []}"#,
            &mut next,
        )
        .unwrap();
        assert!(f.pose.is_none());
    }

    #[test]
    fn rejects_bad_lines() {
        let mut next = 0;
        assert!(parse_line("{not json", &mut next).is_err());
        let bad_sigma = LINE
            .replace("\"sigma_m\": 0.3", "\"sigma_m\": -1.0")
            .replace('\n', " ");
        assert!(matches!(
            parse_line(&bad_sigma, &mut next),
            Err(InputError::Uncertainty(_))
        ));
        let bad_lat = LINE.replace("37.7749", "137.7749").replace('\n', " ");
        assert!(matches!(
            parse_line(&bad_lat, &mut next),
            Err(InputError::Pose(_))
        ));
    }
}
