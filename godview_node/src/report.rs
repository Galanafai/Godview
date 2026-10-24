//! What a node reports when it finishes (written as JSON).

use godview_core::assignment::assign;
use godview_core::frames::Ecef;
use godview_core::world_model::{WorldObject, WorldSnapshot};
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;

/// Matching cutoff when scoring against truth (m).
pub const CUTOFF_M: f64 = 5.0;

/// One node's end-of-run report.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct NodeReport {
    /// The node's agent id (hex of its verifying key).
    pub agent_id: String,
    /// Topic resolution.
    pub topic_resolution: u8,
    /// Topic cells subscribed at the end.
    pub subscribed_cells: Vec<String>,
    /// Batch copies handed to Zenoh (after faults, including duplicates).
    pub batches_sent: u64,
    /// Local-frame batches not sent (no global pose, nobody could fuse them).
    pub batches_skipped_local: u64,
    /// What the fault injector did.
    pub faults: crate::faults::FaultStats,
    /// Batches received from Zenoh.
    pub batches_received: u64,
    /// Batches verified and accepted.
    pub batches_accepted: u64,
    /// Rejected batches by reason.
    pub rejected: BTreeMap<String, u64>,
    /// Accepted batches by source agent id.
    pub accepted_from: BTreeMap<String, u64>,
    /// Reports this node published (including tombstones).
    pub reports_published: u64,
    /// Tracking quality against ground truth (synthetic source only).
    pub tracking: TrackingStats,
    /// The trust-neutral world at the end.
    pub final_world: WorldOut,
    /// Whether the node ever had a global pose.
    pub pose_known: bool,
    /// How late the loop ran against its schedule.
    pub timing: Timing,
}

/// Real-time behavior of the tick loop.
#[derive(Debug, Clone, Copy, PartialEq, Default, Serialize, Deserialize)]
pub struct Timing {
    /// Ticks run.
    pub ticks: u64,
    /// Ticks that started more than half a period late.
    pub late_ticks: u64,
    /// Largest start lag of a tick (ms).
    pub max_lag_ms: f64,
    /// Lag of the final publish behind its scheduled time (ms).
    pub final_publish_lag_ms: f64,
    /// Ticks skipped because the loop fell more than a period behind.
    pub skipped_ticks: u64,
    /// Mean work per tick: receive, ingest, publish and fuse (ms).
    pub mean_work_ms: f64,
    /// Largest work per tick (ms).
    pub max_work_ms: f64,
    #[serde(skip)]
    period_ms: f64,
}

impl Timing {
    /// Records a tick scheduled at `t` that is starting now.
    pub fn record_lag(&mut self, t: godview_env::Timestamp) {
        let lag = godview_env::Timestamp::from_system_time(std::time::SystemTime::now())
            .secs_since(t)
            * 1e3;
        self.ticks += 1;
        self.max_lag_ms = self.max_lag_ms.max(lag);
        if self.period_ms > 0.0 && lag > self.period_ms / 2.0 {
            self.late_ticks += 1;
        }
    }

    /// Records the work time of one tick.
    pub fn record_work(&mut self, d: std::time::Duration) {
        let ms = d.as_secs_f64() * 1e3;
        let n = self.ticks.max(1) as f64;
        self.mean_work_ms += (ms - self.mean_work_ms) / n;
        self.max_work_ms = self.max_work_ms.max(ms);
    }

    /// Sets the tick period used to classify late ticks.
    pub fn with_period_ms(period_ms: f64) -> Self {
        Self {
            period_ms,
            ..Self::default()
        }
    }
}

/// A world snapshot in geodetic form.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct WorldOut {
    /// Query time (ns since the Unix epoch).
    pub t_unix_ns: i64,
    /// Deterministic digest of the fused state.
    pub digest: String,
    /// Fused objects.
    pub objects: Vec<ObjectOut>,
}

/// One fused object.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ObjectOut {
    /// Latitude (degrees).
    pub lat_deg: f64,
    /// Longitude (degrees).
    pub lon_deg: f64,
    /// Height (m).
    pub height_m: f64,
    /// ECEF position (m), for exact comparison.
    pub ecef: [f64; 3],
    /// Horizontal 1-sigma uncertainty, sqrt of the mean of the east and north variances (m).
    pub sigma_h_m: f64,
    /// Number of contributing reports (sources).
    pub sources: usize,
    /// Contributing reports as `source:track@epoch.seq` (provenance).
    pub members: Vec<String>,
    /// Class id.
    pub class_id: u8,
}

impl ObjectOut {
    /// Converts a fused world object.
    pub fn from_object(o: &WorldObject) -> Self {
        let e = o.to_entity();
        let sigma_h_m = e
            .position_cov_enu
            .map_or(f64::NAN, |c| ((c[0] as f64 + c[3] as f64) / 2.0).sqrt());
        let p = o.position().0;
        Self {
            lat_deg: e.position[0],
            lon_deg: e.position[1],
            height_m: e.position[2],
            ecef: [p.x, p.y, p.z],
            sigma_h_m,
            sources: o.members.len(),
            members: o
                .members
                .iter()
                .map(|((src, tr), (e, q))| format!("{}:{}@{e}.{q}", src.short(), tr.0))
                .collect(),
            class_id: o.class_id,
        }
    }
}

/// Estimates against truth at one instant.
#[derive(Debug, Clone, Copy, PartialEq, Default, Serialize, Deserialize)]
pub struct ScoreOut {
    /// Truths matched within the cutoff.
    pub matched: usize,
    /// Truths without an estimate.
    pub missed: usize,
    /// Estimates without a truth (ghosts and duplicates).
    pub false_estimates: usize,
    /// RMS position error of matched pairs (m).
    pub rms_m: f64,
}

impl ScoreOut {
    /// True if every truth is matched and there is no false estimate.
    pub fn exact(&self) -> bool {
        self.missed == 0 && self.false_estimates == 0
    }
}

/// Optimal (Hungarian) matching of a world against truth positions.
pub fn score(world: &WorldSnapshot, truths: &[Ecef]) -> ScoreOut {
    let est: Vec<Ecef> = world.objects.iter().map(|o| o.position()).collect();
    let cost: Vec<Vec<f64>> = est
        .iter()
        .map(|e| truths.iter().map(|t| (e.0 - t.0).norm_squared()).collect())
        .collect();
    let pairs = if est.is_empty() || truths.is_empty() {
        Vec::new()
    } else {
        assign(&cost, CUTOFF_M * CUTOFF_M)
    };
    let sq: f64 = pairs.iter().map(|(e, t)| cost[*e][*t]).sum();
    ScoreOut {
        matched: pairs.len(),
        missed: truths.len() - pairs.len(),
        false_estimates: est.len() - pairs.len(),
        rms_m: if pairs.is_empty() {
            0.0
        } else {
            (sq / pairs.len() as f64).sqrt()
        },
    }
}

/// Tracking quality over the run.
#[derive(Debug, Clone, PartialEq, Default, Serialize, Deserialize)]
pub struct TrackingStats {
    /// Ticks scored (after warm-up).
    pub ticks_scored: u64,
    /// Ticks with a miss or a false estimate.
    pub error_ticks: u64,
    /// Longest run of consecutive error ticks.
    pub max_consecutive_error_ticks: u64,
    /// Largest matched RMS error seen at any tick (m).
    pub max_rms_m: f64,
    /// Score of the final trust-neutral world.
    pub final_score: Option<ScoreOut>,
    /// Input lines that failed to parse (stdin source).
    pub input_errors: u64,
    #[serde(skip)]
    run: u64,
}

impl TrackingStats {
    /// Adds one tick's score.
    pub fn record(&mut self, s: &ScoreOut) {
        self.ticks_scored += 1;
        self.max_rms_m = self.max_rms_m.max(s.rms_m);
        if s.exact() {
            self.run = 0;
        } else {
            self.error_ticks += 1;
            self.run += 1;
            self.max_consecutive_error_ticks = self.max_consecutive_error_ticks.max(self.run);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tracking_counts_runs_of_errors() {
        let ok = ScoreOut {
            matched: 2,
            ..Default::default()
        };
        let bad = ScoreOut {
            matched: 1,
            missed: 1,
            ..Default::default()
        };
        let mut t = TrackingStats::default();
        for s in [ok, bad, bad, ok, bad, ok] {
            t.record(&s);
        }
        assert_eq!(
            (t.ticks_scored, t.error_ticks, t.max_consecutive_error_ticks),
            (6, 3, 2)
        );
    }
}
