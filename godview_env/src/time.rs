//! The shared timebase used by every GodView message and estimate.
//!
//! GodView previously mixed four time conventions (f64 seconds, i64 ms,
//! sender-clock u64 ms and a monotonic duration since process start). Every
//! timestamp that crosses a module or network boundary is now a
//! [`Timestamp`]: signed nanoseconds since the UNIX epoch in a disciplined
//! clock (GPS or PTP in production, the virtual clock in simulation).
//!
//! Agents are assumed to share this timebase to within a stated skew bound;
//! residual skew is modeled as position uncertainty by the publisher rather
//! than ignored.

use serde::{Deserialize, Serialize};
use std::time::{Duration, SystemTime, UNIX_EPOCH};

const NANOS_PER_SEC: i64 = 1_000_000_000;

/// Nanoseconds since the UNIX epoch in the shared timebase.
#[derive(
    Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default, Serialize, Deserialize,
)]
pub struct Timestamp(pub i64);

impl Timestamp {
    /// The UNIX epoch.
    pub const ZERO: Timestamp = Timestamp(0);

    /// Creates a timestamp from nanoseconds since the epoch.
    pub const fn from_nanos(ns: i64) -> Self {
        Self(ns)
    }

    /// Creates a timestamp from milliseconds since the epoch.
    pub const fn from_millis(ms: i64) -> Self {
        Self(ms * 1_000_000)
    }

    /// Creates a timestamp from (possibly fractional) seconds since the epoch.
    ///
    /// Rounds to the nearest nanosecond.
    pub fn from_secs_f64(secs: f64) -> Self {
        Self((secs * NANOS_PER_SEC as f64).round() as i64)
    }

    /// Converts a wall-clock time into the shared timebase.
    ///
    /// Times before the epoch map to negative values.
    pub fn from_system_time(t: SystemTime) -> Self {
        match t.duration_since(UNIX_EPOCH) {
            Ok(d) => Self(d.as_nanos() as i64),
            Err(e) => Self(-(e.duration().as_nanos() as i64)),
        }
    }

    /// Nanoseconds since the epoch.
    pub const fn as_nanos(self) -> i64 {
        self.0
    }

    /// Milliseconds since the epoch (truncated toward negative infinity).
    pub const fn as_millis(self) -> i64 {
        self.0.div_euclid(1_000_000)
    }

    /// Seconds since the epoch as `f64`.
    pub fn as_secs_f64(self) -> f64 {
        self.0 as f64 / NANOS_PER_SEC as f64
    }

    /// Signed difference `self - earlier` in seconds.
    pub fn secs_since(self, earlier: Timestamp) -> f64 {
        (self.0 - earlier.0) as f64 / NANOS_PER_SEC as f64
    }

    /// Returns this timestamp shifted by `secs` (may be negative).
    pub fn add_secs_f64(self, secs: f64) -> Self {
        Self(self.0 + (secs * NANOS_PER_SEC as f64).round() as i64)
    }

    /// Returns this timestamp shifted by `ms` milliseconds (may be negative).
    pub const fn add_millis(self, ms: i64) -> Self {
        Self(self.0 + ms * 1_000_000)
    }

    /// Returns this timestamp shifted forward by a duration.
    pub fn add_duration(self, d: Duration) -> Self {
        Self(self.0 + d.as_nanos() as i64)
    }
}

impl std::fmt::Display for Timestamp {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{:.6}s", self.as_secs_f64())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn round_trips_through_units() {
        let t = Timestamp::from_millis(1_704_067_200_123);
        assert_eq!(t.as_millis(), 1_704_067_200_123);
        assert_eq!(
            Timestamp::from_secs_f64(t.as_secs_f64()).as_millis(),
            t.as_millis()
        );
    }

    #[test]
    fn differences_are_signed_seconds() {
        let a = Timestamp::from_millis(1_000);
        let b = a.add_millis(250);
        assert!((b.secs_since(a) - 0.25).abs() < 1e-12);
        assert!((a.secs_since(b) + 0.25).abs() < 1e-12);
    }

    #[test]
    fn system_time_conversion_matches_epoch_offset() {
        let st = UNIX_EPOCH + Duration::from_millis(5_500);
        assert_eq!(
            Timestamp::from_system_time(st),
            Timestamp::from_millis(5_500)
        );
        let before = UNIX_EPOCH - Duration::from_millis(10);
        assert_eq!(
            Timestamp::from_system_time(before),
            Timestamp::from_millis(-10)
        );
    }

    #[test]
    fn negative_millis_floor() {
        assert_eq!(Timestamp::from_nanos(-1).as_millis(), -1);
    }
}
