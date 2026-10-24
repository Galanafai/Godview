//! Localization: where an agent's odometry frame sits in ECEF, and how sure it is.
//!
//! Local tracks live in the agent's odometry frame, which is smooth but not
//! globally referenced. Localization supplies the link `ECEF ← odometry` with
//! a covariance. The publisher applies the link valid at each report's
//! `t_valid`, so a localization correction changes only future reports and
//! never rewrites track history or buffered observations.

use crate::frames::PoseWithCov;
use godview_env::Timestamp;
use std::collections::VecDeque;

/// Whether an agent currently has a globally referenced pose.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum LocalizationStatus {
    /// The link to ECEF is valid; reports are published in ECEF.
    Global,
    /// No valid global fix; reports are published in the local frame and
    /// excluded from other agents' global fusion.
    LocalOnly,
}

/// A localization estimate at one time.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PoseEstimate {
    /// Time of the estimate.
    pub t: Timestamp,
    /// Transform from the agent's odometry frame into ECEF, with covariance.
    pub ecef_from_odom: PoseWithCov,
    /// Global or local-only.
    pub status: LocalizationStatus,
}

/// Source of pose estimates for publishing.
pub trait LocalizationProvider {
    /// The estimate applicable at time `t`, if any.
    fn pose_at(&self, t: Timestamp) -> Option<PoseEstimate>;
}

/// A short history of pose estimates, queried by time.
///
/// Returns the most recent estimate at or before the requested time, as long
/// as it is not older than `max_age`. The odometry-to-ECEF link changes only
/// when localization corrects itself, so holding the last estimate is exact
/// between corrections.
#[derive(Debug, Clone)]
pub struct PoseHistory {
    buf: VecDeque<PoseEstimate>,
    capacity: usize,
    max_age_ns: i64,
}

impl PoseHistory {
    /// Creates an empty history.
    pub fn new(capacity: usize, max_age_s: f64) -> Self {
        Self {
            buf: VecDeque::with_capacity(capacity),
            capacity: capacity.max(1),
            max_age_ns: (max_age_s * 1e9) as i64,
        }
    }

    /// Records an estimate. Estimates must arrive in time order; an estimate
    /// with the same time as the last one replaces it, an older one is ignored.
    pub fn push(&mut self, est: PoseEstimate) {
        match self.buf.back() {
            Some(last) if est.t < last.t => return,
            Some(last) if est.t == last.t => {
                self.buf.pop_back();
            }
            _ => {}
        }
        if self.buf.len() == self.capacity {
            self.buf.pop_front();
        }
        self.buf.push_back(est);
    }

    /// The latest recorded estimate.
    pub fn latest(&self) -> Option<&PoseEstimate> {
        self.buf.back()
    }
}

impl LocalizationProvider for PoseHistory {
    fn pose_at(&self, t: Timestamp) -> Option<PoseEstimate> {
        let idx = self.buf.partition_point(|e| e.t <= t);
        let est = *self.buf.get(idx.checked_sub(1)?)?;
        (t.as_nanos() - est.t.as_nanos() <= self.max_age_ns).then_some(est)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::frames::Pose3;
    use nalgebra::Vector3;

    fn est(ms: i64, x: f64, status: LocalizationStatus) -> PoseEstimate {
        PoseEstimate {
            t: Timestamp::from_millis(ms),
            ecef_from_odom: PoseWithCov::exact(Pose3::from_yaw(0.0, Vector3::new(x, 0.0, 0.0))),
            status,
        }
    }

    #[test]
    fn returns_latest_estimate_not_after_query() {
        let mut h = PoseHistory::new(8, 1.0);
        h.push(est(0, 1.0, LocalizationStatus::Global));
        h.push(est(100, 2.0, LocalizationStatus::Global));
        h.push(est(200, 3.0, LocalizationStatus::LocalOnly));
        let at = |ms| {
            h.pose_at(Timestamp::from_millis(ms))
                .map(|e| e.ecef_from_odom.pose.translation.x)
        };
        assert_eq!(at(-1), None);
        assert_eq!(at(0), Some(1.0));
        assert_eq!(at(150), Some(2.0));
        assert_eq!(at(250), Some(3.0));
        assert_eq!(at(1300), None, "stale pose must not be used");
        assert_eq!(
            h.pose_at(Timestamp::from_millis(201)).unwrap().status,
            LocalizationStatus::LocalOnly
        );
    }

    #[test]
    fn ignores_out_of_order_and_bounds_capacity() {
        let mut h = PoseHistory::new(2, 10.0);
        h.push(est(100, 1.0, LocalizationStatus::Global));
        h.push(est(50, 9.0, LocalizationStatus::Global));
        h.push(est(200, 2.0, LocalizationStatus::Global));
        h.push(est(300, 3.0, LocalizationStatus::Global));
        assert_eq!(h.pose_at(Timestamp::from_millis(150)), None, "evicted");
        assert_eq!(h.latest().unwrap().t, Timestamp::from_millis(300));
    }
}
