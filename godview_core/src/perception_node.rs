//! One agent's end-to-end perception-sharing pipeline.
//!
//! ```text
//! own sensors ─▶ LocalTracker (L2, odometry frame)
//!                     │ publish(t_valid, pose)
//!                     ▼
//!          signed ReportBatch bytes ───────────────▶ peers
//!                     │                                  │
//!                     ▼                                  ▼
//!              ReportStore (LWW CRDT) ◀── receive(bytes): verify, check, insert
//!                     │
//!                     ▼ world(t)
//!          ConsistencyMonitor ─▶ TrustView ─▶ WorldModel (L3 snapshot)
//! ```
//!
//! Invariants enforced here:
//! * Only the local tracker's own tracks are published (source purity).
//! * Nothing derived from peers or from L3 is ever published or fed back.
//! * Every received byte string is signature-verified and validated before
//!   it can touch state; rejections are counted by reason, never silent.

use crate::consistency::{ConsistencyConfig, ConsistencyMonitor};
use crate::extent::{box_to_enu, center_sys_cov, with_systematic, wrap_pi, ObjectBox};
use crate::filter::CvState;
use crate::frames::{add_clock_uncertainty, Ecef};
use crate::local_tracker::{Detection, LocalTrack, LocalTracker, LocalTrackerConfig, ScanOutcome};
use crate::localization::{LocalizationStatus, PoseEstimate};
use crate::report::{
    quantize_covariance, AgentId, FrameRef, LocalTrackId, ReportBatch, ReportError, ReportKey,
    TrackLifecycle, TrackReport, SELF_TRACK,
};
use crate::report_store::{InsertOutcome, ReportStore};
use crate::topics::topic_cell;
use crate::world_model::{TrustView, WorldModel, WorldModelConfig, WorldSnapshot};
use ed25519_dalek::SigningKey;
use godview_env::Timestamp;
use h3o::{CellIndex, Resolution};
use nalgebra::{Matrix6, Vector3, Vector6};
use std::collections::{BTreeMap, BTreeSet};

/// Publishing parameters.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PublishConfig {
    /// Standard deviation of this agent's clock offset from the shared timebase (s).
    pub clock_sigma_s: f64,
    /// Id of this agent's odometry frame, used when publishing without a global pose.
    pub local_frame_id: u64,
    /// Publish only tracks updated by a detection at the publish time; a track
    /// that was published and no longer qualifies gets one tombstone-form report,
    /// so it leaves the world at once instead of coasting there.
    pub fresh_only: bool,
    /// Publish only tracks whose score is at least this (0 publishes all);
    /// handled like `fresh_only` for a track that was published and falls below.
    pub min_score: f32,
}

impl Default for PublishConfig {
    fn default() -> Self {
        Self {
            clock_sigma_s: 0.002,
            local_frame_id: 1,
            fresh_only: false,
            min_score: 0.0,
        }
    }
}

/// An agent's own state in its odometry frame, for the self report.
///
/// The position is the footprint's centre; its uncertainty here is only the
/// agent's own estimate inside the odometry frame (the placement of that
/// frame on the Earth carries the pose uncertainty, as for detections).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SelfState {
    /// Footprint centre in the odometry frame (m).
    pub position: Vector3<f64>,
    /// Velocity in the odometry frame (m/s).
    pub velocity: Vector3<f64>,
    /// Heading of the footprint's length axis in the odometry frame (rad).
    pub yaw: f64,
    /// Footprint length, width and height (m).
    pub size: [f64; 3],
    /// Class id of the agent as other agents' detections report it.
    pub class_id: u8,
    /// Position standard deviation within the odometry frame (m).
    pub sigma_pos_m: f64,
    /// Velocity standard deviation (m/s).
    pub sigma_vel_mps: f64,
    /// Heading standard deviation (rad).
    pub sigma_yaw_rad: f64,
    /// Footprint size standard deviation, each dimension (m).
    pub sigma_size_m: f64,
}

/// Receiving parameters.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ReceiveConfig {
    /// Batches stamped further than this into the receiver's future are rejected (s).
    pub max_future_skew_s: f64,
    /// Expired reports are garbage collected this long after expiry (s).
    pub gc_grace_s: f64,
}

impl Default for ReceiveConfig {
    fn default() -> Self {
        Self {
            max_future_skew_s: 1.0,
            gc_grace_s: 2.0,
        }
    }
}

/// Full node configuration.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct NodeConfig {
    /// Local tracker.
    pub tracker: LocalTrackerConfig,
    /// Shared world model.
    pub world: WorldModelConfig,
    /// Peer consistency monitoring.
    pub consistency: ConsistencyConfig,
    /// Publishing.
    pub publish: PublishConfig,
    /// Receiving.
    pub receive: ReceiveConfig,
    /// Publish a self report (the agent's own pose and footprint, see
    /// [`SelfState`]) and let the world model assign reports that fall on a
    /// known agent's footprint to that agent. Off by default; sets
    /// `world.self_reports`.
    pub self_reports: bool,
    /// Apply the consistency monitor's trust view in fusion.
    pub disable_trust: bool,
}

/// Why a received byte string was rejected as a whole.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, thiserror::Error)]
pub enum RejectReason {
    /// Failed decoding, signature or validation.
    #[error("invalid batch: {0}")]
    Invalid(ReportError),
    /// The source's key is revoked.
    #[error("source revoked")]
    Revoked,
    /// Stamped too far in the future.
    #[error("batch from the future")]
    FromFuture,
}

/// What a successfully received batch did.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct ReceiveSummary {
    /// Reports that changed the store.
    pub applied: usize,
    /// Reports already held (duplicates, relay echoes).
    pub duplicate: usize,
    /// Reports older than what is held (out-of-order delivery).
    pub stale: usize,
    /// Reports at or below the source's garbage-collection floor (replays).
    pub below_floor: usize,
    /// The batch was this node's own (echoed back by a relay) and was ignored.
    pub own_echo: bool,
    /// The verified source of the batch.
    pub source: Option<AgentId>,
}

/// Counters for diagnosis.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct NodeStats {
    /// Batches this node published.
    pub batches_published: u64,
    /// Reports this node published (including tombstones).
    pub reports_published: u64,
    /// Reports published in the local frame (no global pose).
    pub reports_published_local: u64,
    /// Batches accepted.
    pub batches_accepted: u64,
    /// Totals over accepted batches.
    pub applied: u64,
    /// Duplicate reports received.
    pub duplicate: u64,
    /// Stale reports received.
    pub stale: u64,
    /// Replayed reports below a floor.
    pub below_floor: u64,
    /// Own batches echoed back.
    pub own_echoes: u64,
    /// Rejected batches by reason.
    pub rejected: BTreeMap<RejectReason, u64>,
}

/// One signed batch for one topic cell (see [`PerceptionNode::publish_by_cell`]).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CellBatch {
    /// Topic cell of every report in the batch; `None` for local-frame reports.
    pub cell: Option<CellIndex>,
    /// Signed batch bytes.
    pub bytes: Vec<u8>,
}

/// An agent's perception-sharing node.
pub struct PerceptionNode {
    key: SigningKey,
    id: AgentId,
    epoch: u32,
    next_seq: u64,
    config: NodeConfig,
    tracker: LocalTracker,
    store: ReportStore,
    world: WorldModel,
    monitor: ConsistencyMonitor,
    revoked: BTreeSet<AgentId>,
    /// Terminated tracks and the time until which their tombstone is re-published.
    tombstones: Vec<(LocalTrack, Timestamp)>,
    /// The agent's own state for the next self report, and when it first set one.
    self_state: Option<SelfState>,
    self_born: Option<Timestamp>,
    /// Tracks whose latest report was live (for `publish` suppression).
    live_sent: BTreeSet<LocalTrackId>,
    /// For each own track, the world-object ID it belonged to in this node's
    /// latest world when that ID is another track's key; published as the
    /// report's `name` so the ID can outlive the track that started it.
    names: BTreeMap<LocalTrackId, ReportKey>,
    stats: NodeStats,
}

impl PerceptionNode {
    /// Creates a node. `epoch` must increase across restarts of the same key.
    pub fn new(key: SigningKey, epoch: u32, config: NodeConfig) -> Self {
        let mut world_config = config.world;
        world_config.self_reports = config.self_reports;
        let world = WorldModel::new(world_config);
        Self {
            id: AgentId::from_signing_key(&key),
            key,
            epoch,
            next_seq: 0,
            tracker: LocalTracker::new(config.tracker),
            store: ReportStore::new(),
            monitor: ConsistencyMonitor::new(config.consistency, world.clone()),
            world,
            config,
            revoked: BTreeSet::new(),
            tombstones: Vec::new(),
            self_state: None,
            self_born: None,
            live_sent: BTreeSet::new(),
            names: BTreeMap::new(),
            stats: NodeStats::default(),
        }
    }

    /// This agent's id (its public key).
    pub fn id(&self) -> AgentId {
        self.id
    }

    /// Current epoch.
    pub fn epoch(&self) -> u32 {
        self.epoch
    }

    /// Configuration.
    pub fn config(&self) -> &NodeConfig {
        &self.config
    }

    /// The local tracker (L2).
    pub fn tracker(&self) -> &LocalTracker {
        &self.tracker
    }

    /// The replicated report store.
    pub fn store(&self) -> &ReportStore {
        &self.store
    }

    /// The consistency monitor.
    pub fn monitor(&self) -> &ConsistencyMonitor {
        &self.monitor
    }

    /// Diagnostic counters.
    pub fn stats(&self) -> &NodeStats {
        &self.stats
    }

    /// Sets the agent's own state for the self report published with the next
    /// batch (only with `NodeConfig::self_reports`, and a global pose).
    pub fn set_self_state(&mut self, state: Option<SelfState>) {
        self.self_state = state;
    }

    /// Feeds one scan of the agent's own detections into the local tracker.
    pub fn ingest(&mut self, detections: &[Detection]) -> ScanOutcome {
        self.tracker.process_scan(detections)
    }

    /// Advances track lifecycles and collects expired reports.
    pub fn maintain(&mut self, now: Timestamp) {
        self.tracker.maintain(now);
        self.store
            .collect_garbage(now, self.config.receive.gc_grace_s);
    }

    /// Simulates a restart: local state is lost and the epoch increases.
    ///
    /// Replicated reports from peers are kept (they are soft state anyway).
    pub fn restart(&mut self, new_epoch: u32) {
        assert!(
            new_epoch > self.epoch,
            "epochs must increase across restarts"
        );
        self.epoch = new_epoch;
        self.next_seq = 0;
        self.tracker = LocalTracker::new(self.config.tracker);
        self.tombstones.clear();
        self.names.clear();
    }

    /// Builds, signs and returns a batch reporting every published track at `t_valid`.
    ///
    /// With a global pose the reports are in ECEF, with the pose covariance
    /// propagated into each report; otherwise they are in this agent's local
    /// frame. A terminated track's tombstone is re-published in every batch
    /// for one validity period (soft state, like live tracks), so a single
    /// lost packet cannot leave a ghost at a receiver. The node's own reports
    /// go into its own store exactly as peers will decode them.
    /// Returns `None` when there is nothing to report.
    pub fn publish(&mut self, t_valid: Timestamp, pose: Option<&PoseEstimate>) -> Option<Vec<u8>> {
        let reports = self.build_reports(t_valid, pose);
        self.sign_and_store(t_valid, reports)
    }

    /// Like [`PerceptionNode::publish`], but signs one batch per topic cell.
    ///
    /// Each global report is filed under the H3 cell (at `resolution`) that
    /// contains its position, so a transport can carry it on that cell's
    /// topic and only nodes near it receive it. Local-frame reports have no
    /// global position; they are returned under `cell: None`. Every batch is
    /// independently signed and verifiable.
    pub fn publish_by_cell(
        &mut self,
        t_valid: Timestamp,
        pose: Option<&PoseEstimate>,
        resolution: Resolution,
    ) -> Vec<CellBatch> {
        let mut groups: BTreeMap<Option<CellIndex>, Vec<TrackReport>> = BTreeMap::new();
        for r in self.build_reports(t_valid, pose) {
            let cell = match r.frame {
                FrameRef::Ecef => {
                    topic_cell(&Ecef::new(r.pos[0], r.pos[1], r.pos[2]), resolution).ok()
                }
                FrameRef::Local { .. } => None,
            };
            groups.entry(cell).or_default().push(r);
        }
        groups
            .into_iter()
            .filter_map(|(cell, reports)| {
                self.sign_and_store(t_valid, reports)
                    .map(|bytes| CellBatch { cell, bytes })
            })
            .collect()
    }

    /// The reports for one publish cycle (live tracks and tombstones), each
    /// with a fresh sequence number.
    fn build_reports(
        &mut self,
        t_valid: Timestamp,
        pose: Option<&PoseEstimate>,
    ) -> Vec<TrackReport> {
        let global = pose.filter(|p| p.status == LocalizationStatus::Global);
        let valid_for_ms = (self.config.tracker.terminate_after_s * 1000.0) as u32;
        let pc = self.config.publish;
        let mut entries: Vec<(LocalTrack, CvState)> = Vec::new();
        let mut live_now = BTreeSet::new();
        for t in self.tracker.tracks().filter(|t| t.is_published()) {
            let qualifies = (!pc.fresh_only || t.last_update >= t_valid) && t.class_prob >= pc.min_score;
            if qualifies {
                live_now.insert(t.id);
                entries.push((t.clone(), t.predicted(t_valid)));
            } else if self.live_sent.contains(&t.id) {
                let mut dead = t.clone();
                dead.lifecycle = TrackLifecycle::Terminated;
                entries.push((dead, t.predicted(t_valid)));
            }
        }
        self.live_sent = live_now;
        for t in self.tracker.drain_terminated() {
            // A receiver's last live report of this track expires by
            // termination time + valid_for; keep the tombstone flowing until then.
            let until = t
                .last_update
                .add_secs_f64(self.config.tracker.terminate_after_s)
                .add_millis(valid_for_ms as i64);
            self.tombstones.push((t, until));
        }
        self.tombstones.retain(|(_, until)| *until >= t_valid);
        for (t, _) in &self.tombstones {
            entries.push((t.clone(), t.predicted(t_valid)));
        }
        let want_self = self.config.self_reports && self.self_state.is_some() && global.is_some();
        if entries.is_empty() && !want_self {
            return Vec::new();
        }
        entries.sort_by_key(|(t, _)| t.id);

        let mut reports = Vec::with_capacity(entries.len());
        for (track, state) in entries {
            let tc = &self.config.tracker;
            let odom_box = track
                .published_box(t_valid, tc.publish_raw_box)
                .map(|b| with_systematic(&b, tc.box_size_sys_m, tc.box_yaw_sys_rad));
            let mut state = state;
            if let Some(b) = odom_box {
                let sys = center_sys_cov(&b, self.config.tracker.box_center_sys_frac);
                let mut pp = state.p.fixed_view_mut::<3, 3>(0, 0);
                pp += sys;
            }
            state.p[(2, 2)] += tc.box_center_sys_z_m * tc.box_center_sys_z_m;
            let (frame, x, mut p) = match global {
                Some(g) => {
                    let (x, p) = g.ecef_from_odom.transform_state(&state.x, &state.p);
                    (FrameRef::Ecef, x, p)
                }
                None => (
                    FrameRef::Local {
                        frame_id: self.config.publish.local_frame_id,
                    },
                    state.x,
                    state.p,
                ),
            };
            let vel = Vector3::new(x[3], x[4], x[5]);
            add_clock_uncertainty(&mut p, &vel, self.config.publish.clock_sigma_s);
            if !finite(&x, &p) {
                continue;
            }
            let p_exist = if track.lifecycle == TrackLifecycle::Terminated {
                0.0
            } else {
                track.hits as f32 / (track.hits as f32 + 1.0)
            };
            let report = TrackReport {
                source: self.id,
                epoch: self.epoch,
                seq: self.next_seq,
                track: track.id,
                t_valid,
                frame,
                pos: [x[0], x[1], x[2]],
                vel: [x[3] as f32, x[4] as f32, x[5] as f32],
                cov: quantize_covariance(&p),
                class_id: track.class_id,
                class_prob: track.class_prob.clamp(0.0, 1.0),
                p_exist,
                lifecycle: track.lifecycle,
                valid_for_ms,
                born: track.created.min(t_valid),
                name: self.names.get(&track.id).copied(),
                object_box: odom_box
                    .map(|b| match global {
                        Some(g) => box_to_enu(&b, &g.ecef_from_odom, &Ecef::new(x[0], x[1], x[2])),
                        None => b,
                    })
                    .map(|b| b.quantized())
                    .filter(|b| b.is_valid()),
            };
            if report.validate().is_err() {
                continue;
            }
            self.next_seq += 1;
            if frame != FrameRef::Ecef {
                self.stats.reports_published_local += 1;
            }
            reports.push(report);
        }
        if let (true, Some(s), Some(g)) = (want_self, self.self_state, global) {
            if let Some(r) = self.self_report(t_valid, &s, g, valid_for_ms) {
                reports.push(r);
            }
        }
        reports
    }

    /// The self report: the agent's own pose and footprint as a report with
    /// the reserved track id, placed on the Earth like any other state.
    fn self_report(
        &mut self,
        t_valid: Timestamp,
        s: &SelfState,
        pose: &PoseEstimate,
        valid_for_ms: u32,
    ) -> Option<TrackReport> {
        let x_odom = Vector6::new(
            s.position.x,
            s.position.y,
            s.position.z,
            s.velocity.x,
            s.velocity.y,
            s.velocity.z,
        );
        let mut p_odom = Matrix6::zeros();
        for k in 0..3 {
            p_odom[(k, k)] = s.sigma_pos_m.powi(2);
            p_odom[(k + 3, k + 3)] = s.sigma_vel_mps.powi(2);
        }
        let (x, mut p) = pose.ecef_from_odom.transform_state(&x_odom, &p_odom);
        let vel = Vector3::new(x[3], x[4], x[5]);
        add_clock_uncertainty(&mut p, &vel, self.config.publish.clock_sigma_s);
        if !finite(&x, &p) {
            return None;
        }
        let footprint = ObjectBox {
            size: s.size,
            size_var: [s.sigma_size_m.powi(2); 3],
            yaw: wrap_pi(s.yaw),
            yaw_var: s.sigma_yaw_rad.powi(2),
            yaw_ambiguous: false,
        };
        let object_box = Some(box_to_enu(&footprint, &pose.ecef_from_odom, &Ecef::new(x[0], x[1], x[2])).quantized())
            .filter(|b| b.is_valid());
        let born = *self.self_born.get_or_insert(t_valid);
        let report = TrackReport {
            source: self.id,
            epoch: self.epoch,
            seq: self.next_seq,
            track: SELF_TRACK,
            t_valid,
            frame: FrameRef::Ecef,
            pos: [x[0], x[1], x[2]],
            vel: [x[3] as f32, x[4] as f32, x[5] as f32],
            cov: quantize_covariance(&p),
            class_id: s.class_id,
            class_prob: 1.0,
            p_exist: 1.0,
            lifecycle: TrackLifecycle::Confirmed,
            valid_for_ms,
            born,
            name: None,
            object_box,
        };
        if report.validate().is_err() {
            return None;
        }
        self.next_seq += 1;
        Some(report)
    }

    /// Signs `reports` as one batch, inserts them into the own store exactly
    /// as peers will decode them, and returns the bytes.
    fn sign_and_store(&mut self, t_valid: Timestamp, reports: Vec<TrackReport>) -> Option<Vec<u8>> {
        if reports.is_empty() {
            return None;
        }
        let batch = ReportBatch {
            source: self.id,
            epoch: self.epoch,
            sent_at: t_valid,
            reports,
        };
        let bytes = batch.sign(&self.key);
        for r in &batch.reports {
            self.store.insert(*r);
        }
        self.stats.batches_published += 1;
        self.stats.reports_published += batch.reports.len() as u64;
        Some(bytes)
    }

    /// Verifies, validates and stores a received batch.
    pub fn receive(
        &mut self,
        bytes: &[u8],
        now: Timestamp,
    ) -> Result<ReceiveSummary, RejectReason> {
        let result = self.receive_inner(bytes, now);
        if let Err(reason) = result {
            *self.stats.rejected.entry(reason).or_default() += 1;
        }
        result
    }

    fn receive_inner(
        &mut self,
        bytes: &[u8],
        now: Timestamp,
    ) -> Result<ReceiveSummary, RejectReason> {
        let batch = ReportBatch::verify_and_decode(bytes).map_err(RejectReason::Invalid)?;
        let mut summary = ReceiveSummary {
            source: Some(batch.source),
            ..ReceiveSummary::default()
        };
        if batch.source == self.id {
            summary.own_echo = true;
            self.stats.own_echoes += 1;
            return Ok(summary);
        }
        if self.revoked.contains(&batch.source) {
            return Err(RejectReason::Revoked);
        }
        if batch.sent_at.secs_since(now) > self.config.receive.max_future_skew_s {
            return Err(RejectReason::FromFuture);
        }
        for r in batch.reports {
            match self.store.insert(r) {
                InsertOutcome::Applied => summary.applied += 1,
                InsertOutcome::Duplicate => summary.duplicate += 1,
                InsertOutcome::Stale => summary.stale += 1,
                InsertOutcome::BelowFloor => summary.below_floor += 1,
            }
        }
        self.stats.batches_accepted += 1;
        self.stats.applied += summary.applied as u64;
        self.stats.duplicate += summary.duplicate as u64;
        self.stats.stale += summary.stale as u64;
        self.stats.below_floor += summary.below_floor as u64;
        Ok(summary)
    }

    /// Revokes a source: its stored reports are purged and future batches rejected.
    pub fn revoke(&mut self, source: AgentId) {
        self.revoked.insert(source);
        self.store.purge_source(&source);
    }

    /// The shared world state at `t`, with this node's trust judgment applied.
    ///
    /// Updates the consistency monitor from the reports live at `t` first.
    ///
    /// Also records, for each of this node's own tracks, the ID of the world
    /// object it belongs to, which the next publish carries as its name.
    pub fn world(&mut self, t: Timestamp) -> WorldSnapshot {
        let snap = if self.config.disable_trust {
            self.world.snapshot(&self.store, t, &TrustView::default())
        } else {
            self.monitor.observe(&self.store, t);
            let view = self.monitor.trust_view();
            self.world.snapshot(&self.store, t, &view)
        };
        self.names = snap
            .objects
            .iter()
            .flat_map(|o| {
                o.members
                    .iter()
                    .filter(|((src, tr), _)| {
                        *src == self.id && (*src, *tr) != o.id && o.self_agent.is_none()
                    })
                    .map(|((_, tr), _)| (*tr, o.id))
            })
            .collect();
        snap
    }

    /// The shared world state at `t` with no trust adjustments.
    ///
    /// A pure function of the store: two nodes holding the same reports return
    /// bit-identical snapshots, which is what convergence tests compare.
    pub fn world_neutral(&self, t: Timestamp) -> WorldSnapshot {
        self.world.snapshot(&self.store, t, &TrustView::default())
    }
}

fn finite(x: &Vector6<f64>, p: &Matrix6<f64>) -> bool {
    x.iter().all(|v| v.is_finite()) && p.iter().all(|v| v.is_finite())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::frames::{EnuFrame, Geodetic, Pose3, PoseWithCov};
    use nalgebra::Matrix3;

    fn site() -> EnuFrame {
        EnuFrame::new(Geodetic::new(37.7749, -122.4194, 0.0).unwrap())
    }

    /// An agent at `origin_enu` with odometry yaw `yaw`, observing ENU points.
    struct Rig {
        node: PerceptionNode,
        odom: Pose3, // ENU <- odom
        next_obs: u64,
    }

    impl Rig {
        fn new(seed: u8, origin: Vector3<f64>, yaw: f64) -> Self {
            Self {
                node: PerceptionNode::new(
                    SigningKey::from_bytes(&[seed; 32]),
                    1,
                    NodeConfig::default(),
                ),
                odom: Pose3::from_yaw(yaw, origin),
                next_obs: 0,
            }
        }
        fn pose(&self, t: Timestamp) -> PoseEstimate {
            let link = site().ecef_from_enu_pose().compose(&self.odom);
            PoseEstimate {
                t,
                ecef_from_odom: PoseWithCov::exact(link),
                status: LocalizationStatus::Global,
            }
        }
        fn see(&mut self, t: Timestamp, points_enu: &[Vector3<f64>]) {
            let inv = self.odom.inverse();
            let dets: Vec<Detection> = points_enu
                .iter()
                .map(|p| {
                    self.next_obs += 1;
                    Detection {
                        obs_id: self.next_obs,
                        t,
                        z: inv.transform_point(p),
                        r: Matrix3::identity() * 0.04,
                        class_id: 1,
                        class_prob: 0.9,
                        object_box: None,
                    }
                })
                .collect();
            self.node.ingest(&dets);
            self.node.maintain(t);
        }
    }

    #[test]
    fn two_agents_exchange_and_agree_on_one_object() {
        let mut a = Rig::new(1, Vector3::new(0.0, 0.0, 0.0), 0.4);
        let mut b = Rig::new(2, Vector3::new(80.0, 30.0, 0.0), -2.0);
        let truth = Vector3::new(40.0, 10.0, 1.0);
        let mut last = None;
        for k in 0..10 {
            let t = Timestamp::from_millis(k * 100);
            a.see(t, &[truth]);
            b.see(t, &[truth]);
            let ba = a.node.publish(t, Some(&a.pose(t)));
            let bb = b.node.publish(t, Some(&b.pose(t)));
            if let (Some(ba), Some(bb)) = (ba, bb) {
                a.node.receive(&bb, t).unwrap();
                b.node.receive(&ba, t).unwrap();
                last = Some(t);
            }
        }
        let t = last.expect("tracks confirmed and published");
        let wa = a.node.world_neutral(t);
        let wb = b.node.world_neutral(t);
        assert_eq!(wa.objects.len(), 1);
        assert_eq!(wa.objects[0].members.len(), 2);
        assert_eq!(wa.digest(), wb.digest(), "same reports, same world");
        let err = wa.objects[0].position().distance(&site().to_ecef(&truth));
        assert!(err < 0.5, "fused error {err} m");
    }

    #[test]
    fn per_cell_batches_carry_the_same_reports_as_one_batch() {
        // Two objects 3 km apart fall in different topic cells. Receiving every
        // per-cell batch must give exactly the store (and world) that the single
        // batch of an identical twin gives.
        let res = crate::topics::DEFAULT_TOPIC_RESOLUTION;
        let objects = [
            Vector3::new(20.0, 5.0, 1.0),
            Vector3::new(3000.0, -40.0, 1.0),
        ];
        let mut split = Rig::new(5, Vector3::zeros(), 0.7);
        let mut whole = Rig::new(5, Vector3::zeros(), 0.7);
        let (mut rx_split, mut rx_whole) = (
            Rig::new(6, Vector3::zeros(), 0.0),
            Rig::new(6, Vector3::zeros(), 0.0),
        );
        let mut cells_seen = BTreeSet::new();
        let mut t = Timestamp::ZERO;
        for k in 0..6 {
            t = Timestamp::from_millis(k * 100);
            split.see(t, &objects);
            whole.see(t, &objects);
            for b in split.node.publish_by_cell(t, Some(&split.pose(t)), res) {
                let cell = b.cell.expect("global reports have a cell");
                let decoded = ReportBatch::verify_and_decode(&b.bytes).unwrap();
                for r in &decoded.reports {
                    let at = topic_cell(&Ecef::new(r.pos[0], r.pos[1], r.pos[2]), res).unwrap();
                    assert_eq!(at, cell, "report filed under its own cell");
                }
                cells_seen.insert(cell);
                rx_split.node.receive(&b.bytes, t).unwrap();
            }
            if let Some(bytes) = whole.node.publish(t, Some(&whole.pose(t))) {
                rx_whole.node.receive(&bytes, t).unwrap();
            }
        }
        assert_eq!(cells_seen.len(), 2, "one topic per object here");
        assert_eq!(
            rx_split.node.store().digest(),
            rx_whole.node.store().digest()
        );
        assert_eq!(
            rx_split.node.world_neutral(t).digest(),
            rx_whole.node.world_neutral(t).digest()
        );
        assert_eq!(rx_split.node.world_neutral(t).objects.len(), 2);
    }

    #[test]
    fn local_frame_reports_have_no_cell() {
        let res = crate::topics::DEFAULT_TOPIC_RESOLUTION;
        let mut a = Rig::new(7, Vector3::zeros(), 0.0);
        let mut out = Vec::new();
        for k in 0..5 {
            let t = Timestamp::from_millis(k * 100);
            a.see(t, &[Vector3::new(5.0, 5.0, 0.0)]);
            out = a.node.publish_by_cell(t, None, res);
        }
        assert_eq!(out.len(), 1);
        assert_eq!(out[0].cell, None);
    }

    #[test]
    fn published_boxes_carry_heading_in_east_north_up_and_systematic_errors() {
        // An agent whose odometry frame is rotated 0.7 rad sees a stationary 12 m
        // truck whose length points 0.2 rad from its odometry x axis: the shared
        // heading is 0.9 rad from East (modulo π, the detector is ambiguous), and
        // the declared systematic center error widens the report along the truck;
        // declared systematic size and heading errors widen the box variances.
        let mut cfg = NodeConfig::default();
        cfg.tracker.box_center_sys_frac = 0.25;
        cfg.tracker.box_size_sys_m = 0.4;
        cfg.tracker.box_yaw_sys_rad = 0.05;
        cfg.tracker.box_center_sys_z_m = 0.3;
        let mut plain = Rig::new(8, Vector3::zeros(), 0.7);
        let mut rig = Rig::new(8, Vector3::zeros(), 0.7);
        rig.node = PerceptionNode::new(SigningKey::from_bytes(&[8; 32]), 1, cfg);
        let truth = Vector3::new(20.0, 10.0, 1.0);
        let mut last = (None, None);
        for k in 0..6 {
            let t = Timestamp::from_millis(k * 100);
            for (r, out) in [(&mut plain, &mut last.0), (&mut rig, &mut last.1)] {
                let z = r.odom.inverse().transform_point(&truth);
                r.next_obs += 1;
                r.node.ingest(&[Detection {
                    obs_id: r.next_obs,
                    t,
                    z,
                    r: Matrix3::identity() * 0.04,
                    class_id: 1,
                    class_prob: 0.9,
                    object_box: Some(crate::extent::ObjectBox {
                        size: [12.0, 2.5, 3.5],
                        size_var: [0.04; 3],
                        yaw: 0.2,
                        yaw_var: 0.0025,
                        yaw_ambiguous: true,
                    }),
                }]);
                r.node.maintain(t);
                *out = r.node.publish(t, Some(&r.pose(t))).or(out.take());
            }
        }
        let decode =
            |b: Option<Vec<u8>>| ReportBatch::verify_and_decode(&b.unwrap()).unwrap().reports[0];
        let (p, s) = (decode(last.0), decode(last.1));
        let b = s.object_box.expect("box published");
        assert!(
            crate::extent::wrap_half_pi(b.yaw - 0.9).abs() < 1e-3,
            "yaw {}",
            b.yaw
        );
        assert!((b.size[0] - 12.0).abs() < 0.05);
        let pb = p.object_box.expect("box published");
        let added = b.size_var[0] - pb.size_var[0];
        assert!((added - 0.16).abs() < 1e-4, "size variance added {added}");
        let added = b.yaw_var - pb.yaw_var;
        assert!(
            (added - 0.0025).abs() < 1e-5,
            "heading variance added {added}"
        );
        let along = site().ecef_from_enu() * Vector3::new(0.9f64.cos(), 0.9f64.sin(), 0.0);
        let var = |r: &TrackReport| {
            let c = r.covariance();
            let pp: Matrix3<f64> = c.fixed_view::<3, 3>(0, 0).into_owned();
            along.dot(&(pp * along))
        };
        assert!(
            var(&s) - var(&p) > 1.5f64.powi(2) * 0.99,
            "systematic error added along the length: {} vs {}",
            var(&s),
            var(&p)
        );
        let up = site().ecef_from_enu() * Vector3::z();
        let var_up = |r: &TrackReport| {
            let c = r.covariance();
            let pp: Matrix3<f64> = c.fixed_view::<3, 3>(0, 0).into_owned();
            up.dot(&(pp * up))
        };
        let added = var_up(&s) - var_up(&p);
        assert!(
            (added - 0.09).abs() < 1e-6,
            "vertical systematic variance added {added}"
        );
    }

    fn footprint() -> SelfState {
        SelfState {
            position: Vector3::zeros(),
            velocity: Vector3::zeros(),
            yaw: 0.0,
            size: [4.5, 2.0, 1.6],
            class_id: 1,
            sigma_pos_m: 0.1,
            sigma_vel_mps: 0.5,
            sigma_yaw_rad: 0.03,
            sigma_size_m: 0.2,
        }
    }

    /// A pose with a horizontal position uncertainty of `sigma_h` (m) per axis.
    fn uncertain_pose(rig: &Rig, t: Timestamp, sigma_h: f64) -> PoseEstimate {
        let link = site().ecef_from_enu_pose().compose(&rig.odom);
        let mut cov_enu = nalgebra::Matrix6::zeros();
        cov_enu[(0, 0)] = sigma_h * sigma_h;
        cov_enu[(1, 1)] = sigma_h * sigma_h;
        cov_enu[(2, 2)] = 0.01;
        cov_enu[(3, 3)] = 1e-6;
        cov_enu[(4, 4)] = 1e-6;
        cov_enu[(5, 5)] = 1e-6;
        PoseEstimate {
            t,
            ecef_from_odom: PoseWithCov {
                pose: link,
                cov: crate::frames::rotate_state_cov(&site().ecef_from_enu(), &cov_enu),
            },
            status: LocalizationStatus::Global,
        }
    }

    #[test]
    fn self_report_is_published_only_when_on_and_decodes_as_the_footprint() {
        let decode = |b: Option<Vec<u8>>| -> Vec<TrackReport> {
            b.map(|b| ReportBatch::verify_and_decode(&b).unwrap().reports.to_vec()).unwrap_or_default()
        };
        let t = Timestamp::from_millis(100);
        // Off by default: nothing is published even with a self state.
        let mut rig = Rig::new(31, Vector3::new(5.0, 7.0, 0.0), 0.4);
        rig.node.set_self_state(Some(footprint()));
        assert!(decode(rig.node.publish(t, Some(&rig.pose(t)))).is_empty());
        // On: one self report with the reserved id, at the agent, with the footprint.
        let mut cfg = NodeConfig::default();
        cfg.self_reports = true;
        let mut rig = Rig::new(32, Vector3::new(5.0, 7.0, 0.0), 0.4);
        rig.node = PerceptionNode::new(SigningKey::from_bytes(&[32; 32]), 1, cfg);
        let pose = rig.pose(t);
        // No self state, or no global pose: nothing.
        assert!(decode(rig.node.publish(t, Some(&pose))).is_empty());
        rig.node.set_self_state(Some(footprint()));
        let local = PoseEstimate { status: LocalizationStatus::LocalOnly, ..pose };
        assert!(decode(rig.node.publish(t, Some(&local))).is_empty());
        let reports = decode(rig.node.publish(t, Some(&pose)));
        assert_eq!(reports.len(), 1);
        let r = &reports[0];
        assert_eq!(r.track, SELF_TRACK);
        let enu = site().from_ecef(&crate::frames::Ecef::new(r.pos[0], r.pos[1], r.pos[2]));
        assert!((enu - Vector3::new(5.0, 7.0, 0.0)).norm() < 1e-6, "{enu:?}");
        let b = r.object_box.expect("footprint box");
        assert!(b.size.iter().zip([4.5, 2.0, 1.6]).all(|(a, e)| (a - e).abs() < 1e-6), "{:?}", b.size);
        // The heading (0 in the odometry frame, which is rotated 0.4 rad) is 0.4 from East.
        assert!((b.yaw - 0.4).abs() < 1e-3, "{}", b.yaw);
        // The world model of the same node holds it as a self object.
        let w = rig.node.world(t);
        assert_eq!(w.objects.len(), 1);
        assert_eq!(w.objects[0].self_agent, Some(rig.node.id()));
    }

    #[test]
    fn two_agents_seeing_each_other_never_merge_with_self_reports_and_do_without() {
        // A at the origin, B 6 m East, both with a 1.5 m horizontal pose
        // uncertainty. Each detects the other with an error (as a pose offset
        // would cause) and both detect a car C 15 m aside.
        let run = |self_reports: bool| -> (usize, usize, Vec<Vec<u8>>) {
            let mut cfg = NodeConfig::default();
            cfg.self_reports = self_reports;
            let mut a = Rig::new(41, Vector3::new(0.0, 0.0, 0.0), 0.0);
            let mut b = Rig::new(42, Vector3::new(6.0, 0.0, 0.0), 0.0);
            a.node = PerceptionNode::new(SigningKey::from_bytes(&[41; 32]), 1, cfg);
            b.node = PerceptionNode::new(SigningKey::from_bytes(&[42; 32]), 1, cfg);
            let det = |r: &mut Rig, t: Timestamp, enu: Vector3<f64>| {
                r.next_obs += 1;
                Detection {
                    obs_id: r.next_obs,
                    t,
                    z: r.odom.inverse().transform_point(&enu),
                    r: Matrix3::identity() * 0.04,
                    class_id: 1,
                    class_prob: 0.9,
                    object_box: Some(crate::extent::ObjectBox {
                        size: [4.5, 2.0, 1.6],
                        size_var: [0.04; 3],
                        yaw: 0.0,
                        yaw_var: 0.0025,
                        yaw_ambiguous: true,
                    }),
                }
            };
            let mut last = Timestamp::from_millis(0);
            for k in 0..6 {
                let t = Timestamp::from_millis(k * 100);
                last = t;
                let (ca, cb) = (Vector3::new(7.5, 0.6, 0.0), Vector3::new(-1.1, -0.4, 0.0));
                let c = Vector3::new(3.0, 15.0, 0.0);
                let da = vec![det(&mut a, t, ca), det(&mut a, t, c)];
                let db = vec![det(&mut b, t, cb), det(&mut b, t, c)];
                a.node.ingest(&da);
                b.node.ingest(&db);
                a.node.maintain(t);
                b.node.maintain(t);
                a.node.set_self_state(Some(footprint()));
                let mut sb = footprint();
                sb.position = b.odom.inverse().transform_point(&Vector3::new(6.0, 0.0, 0.0));
                b.node.set_self_state(Some(sb));
                let (pa, pb) = (uncertain_pose(&a, t, 1.5), uncertain_pose(&b, t, 1.5));
                let ba = a.node.publish(t, Some(&pa));
                let bb = b.node.publish(t, Some(&pb));
                if let Some(x) = ba {
                    b.node.receive(&x, t).ok();
                }
                if let Some(x) = bb {
                    a.node.receive(&x, t).ok();
                }
            }
            let w = a.node.world(last);
            let selfs = w.objects.iter().filter(|o| o.self_agent.is_some()).count();
            let kinds = w
                .objects
                .iter()
                .map(|o| o.members.iter().map(|((s, _), _)| s.0[0]).collect::<Vec<u8>>())
                .collect();
            (w.objects.len(), selfs, kinds)
        };
        let (off_objects, off_selfs, off_kinds) = run(false);
        assert_eq!(off_selfs, 0);
        assert_eq!(off_objects, 2, "without self reports the two detections of the agents merge: {off_kinds:?}");
        let (on_objects, on_selfs, on_kinds) = run(true);
        assert_eq!(on_selfs, 2, "{on_kinds:?}");
        assert_eq!(on_objects, 3, "two agents and the common car: {on_kinds:?}");
    }

    #[test]
    fn publish_suppression_sends_one_tombstone_and_nothing_below_the_score() {
        let reports = |b: Option<Vec<u8>>| -> Vec<TrackReport> {
            b.map(|b| ReportBatch::verify_and_decode(&b).unwrap().reports.to_vec()).unwrap_or_default()
        };
        // fresh_only: a track without a detection this frame leaves the world once.
        let mut cfg = NodeConfig::default();
        cfg.publish.fresh_only = true;
        let mut rig = Rig::new(21, Vector3::zeros(), 0.0);
        rig.node = PerceptionNode::new(SigningKey::from_bytes(&[21; 32]), 1, cfg);
        let p = Vector3::new(5.0, 5.0, 0.0);
        let mut live = 0;
        for k in 0..5 {
            let t = Timestamp::from_millis(k * 100);
            rig.see(t, &[p]);
            live = reports(rig.node.publish(t, Some(&rig.pose(t)))).iter().filter(|r| r.lifecycle == TrackLifecycle::Confirmed).count();
        }
        assert_eq!(live, 1);
        let t = Timestamp::from_millis(500);
        rig.node.maintain(t);
        let r = reports(rig.node.publish(t, Some(&rig.pose(t))));
        assert_eq!(r.len(), 1);
        assert_eq!(r[0].lifecycle, TrackLifecycle::Terminated);
        let t = Timestamp::from_millis(600);
        rig.node.maintain(t);
        assert!(reports(rig.node.publish(t, Some(&rig.pose(t)))).is_empty());
        // min_score above the detections' 0.9: nothing is ever published.
        let mut cfg = NodeConfig::default();
        cfg.publish.min_score = 0.95;
        let mut rig = Rig::new(22, Vector3::zeros(), 0.0);
        rig.node = PerceptionNode::new(SigningKey::from_bytes(&[22; 32]), 1, cfg);
        for k in 0..5 {
            let t = Timestamp::from_millis(k * 100);
            rig.see(t, &[p]);
            assert!(rig.node.publish(t, Some(&rig.pose(t))).is_none());
        }
    }

    #[test]
    fn rejects_tampered_revoked_and_future_batches() {
        let mut a = Rig::new(3, Vector3::zeros(), 0.0);
        let mut b = Rig::new(4, Vector3::new(10.0, 0.0, 0.0), 0.0);
        let mut bytes = None;
        for k in 0..5 {
            let t = Timestamp::from_millis(k * 100);
            b.see(t, &[Vector3::new(5.0, 5.0, 0.0)]);
            bytes = b.node.publish(t, Some(&b.pose(t))).or(bytes);
        }
        let bytes = bytes.unwrap();
        let mut tampered = bytes.clone();
        tampered[60] ^= 1;
        let now = Timestamp::from_millis(400);
        assert!(matches!(
            a.node.receive(&tampered, now),
            Err(RejectReason::Invalid(_))
        ));
        assert!(
            a.node.store().is_empty(),
            "nothing from a rejected batch is stored"
        );
        assert_eq!(
            a.node.receive(&bytes, Timestamp::from_millis(-5000)),
            Err(RejectReason::FromFuture)
        );
        assert!(a.node.receive(&bytes, now).unwrap().applied > 0);
        assert_eq!(a.node.receive(&bytes, now).unwrap().duplicate, 1);
        a.node.revoke(b.node.id());
        assert!(
            a.node.store().is_empty(),
            "revocation purges the source's reports"
        );
        assert_eq!(a.node.receive(&bytes, now), Err(RejectReason::Revoked));
        assert_eq!(a.node.stats().rejected.values().sum::<u64>(), 3);
    }

    #[test]
    fn without_global_pose_reports_are_local_and_not_fused_by_peers() {
        let mut a = Rig::new(5, Vector3::zeros(), 0.0);
        let mut b = Rig::new(6, Vector3::new(10.0, 0.0, 0.0), 0.0);
        let mut bytes = None;
        for k in 0..5 {
            let t = Timestamp::from_millis(k * 100);
            b.see(t, &[Vector3::new(5.0, 5.0, 0.0)]);
            let mut p = b.pose(t);
            p.status = LocalizationStatus::LocalOnly;
            bytes = b.node.publish(t, Some(&p)).or(bytes);
        }
        let t = Timestamp::from_millis(400);
        a.node.receive(&bytes.unwrap(), t).unwrap();
        let w = a.node.world_neutral(t);
        assert!(w.objects.is_empty());
        assert_eq!(w.skipped_local_frame, 1);
        assert!(b.node.stats().reports_published_local > 0);
    }

    #[test]
    fn own_echo_is_ignored_and_tombstone_published() {
        let mut a = Rig::new(7, Vector3::zeros(), 0.0);
        let mut bytes = None;
        for k in 0..5 {
            let t = Timestamp::from_millis(k * 100);
            a.see(t, &[Vector3::new(5.0, 5.0, 0.0)]);
            bytes = a.node.publish(t, Some(&a.pose(t))).or(bytes);
        }
        assert!(
            a.node
                .receive(&bytes.unwrap(), Timestamp::from_millis(400))
                .unwrap()
                .own_echo
        );
        // Object disappears: track coasts, terminates, and a tombstone is published.
        let t = Timestamp::from_millis(2500);
        a.node.maintain(t);
        let tomb = a
            .node
            .publish(t, Some(&a.pose(t)))
            .expect("tombstone batch");
        let batch = ReportBatch::verify_and_decode(&tomb).unwrap();
        assert_eq!(batch.reports.len(), 1);
        assert_eq!(batch.reports[0].lifecycle, TrackLifecycle::Terminated);
        assert!(a.node.world_neutral(t).objects.is_empty());
    }
}
