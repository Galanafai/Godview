//! The node runtime: Zenoh I/O around a synchronous [`PerceptionNode`].
//!
//! ```text
//!  detections ─▶ PerceptionNode (sync, owned by the main loop) ─▶ per-cell batches
//!                      ▲                                              │ faults
//!                      │ receive(bytes)                               ▼
//!  Zenoh subscriber callbacks ──▶ channel ──────────┐        Zenoh put on
//!  (one per topic cell around the agent)            │  <prefix>/reports/<res>/<cell>
//!                                                   ▼
//!                                        drained every tick
//! ```
//!
//! All perception math stays synchronous and pure; only this module is
//! async. The node runs at a fixed rate. With a synthetic source it senses
//! for `duration_s`, then performs one final fault-free publish at the stop
//! time, waits `settle_s` for peers' final batches, and records the world at
//! the stop time. Nodes that receive the same final batches therefore
//! compute bit-identical worlds, which the multi-process tests check.

use crate::faults::{FaultInjector, FaultSpec, FaultStats};
use crate::input::{parse_line, ParsedFrame};
use crate::keys::report_key;
use crate::report::{score, NodeReport, ObjectOut, Timing, TrackingStats, WorldOut};
use crate::synthetic::{site, SyntheticAgent, SyntheticSpec, Truth};
use ed25519_dalek::SigningKey;
use godview_core::frames::Ecef;
use godview_core::localization::PoseEstimate;
use godview_core::perception_node::{NodeConfig, PerceptionNode};
use godview_core::report::AgentId;
use godview_core::topics::cover_cells;
use godview_env::Timestamp;
use h3o::{CellIndex, Resolution};
use nalgebra::Vector3;
use rand::{RngCore, SeedableRng};
use std::collections::{BTreeMap, BTreeSet};
use std::time::{Duration, SystemTime};
use tokio::io::AsyncBufReadExt;
use tokio::sync::mpsc;
use zenoh::qos::CongestionControl;

/// Where detections come from.
#[derive(Debug, Clone)]
pub enum Source {
    /// The seeded synthetic world; this node is ring agent `index`, displaced by `offset_enu`.
    Synthetic {
        /// Scene parameters (identical for every node of a run).
        spec: SyntheticSpec,
        /// Ring index.
        index: usize,
        /// Displacement from the ring position (m, ENU).
        offset_enu: Vector3<f64>,
    },
    /// JSON lines on standard input (see [`crate::input`]); runs until end of input.
    Stdin,
}

/// Everything a node needs to run.
#[derive(Debug, Clone)]
pub struct RunConfig {
    /// Seed for the signing key; random if `None`.
    pub key_seed: Option<u64>,
    /// Zenoh endpoints to listen on (for example `tcp/127.0.0.1:7447`).
    pub listen: Vec<String>,
    /// Zenoh endpoints to connect to.
    pub connect: Vec<String>,
    /// Key expression prefix.
    pub prefix: String,
    /// Topic resolution.
    pub topic_resolution: Resolution,
    /// Radius around the agent whose reports this node subscribes to (m).
    pub interest_radius_m: f64,
    /// Publish and fuse rate (Hz).
    pub rate_hz: f64,
    /// Start time (Unix ms); now if `None`. Lets several processes start in step.
    pub start_unix_ms: Option<i64>,
    /// Sensing time for the synthetic source (s).
    pub duration_s: f64,
    /// Wait for peers' final batches after the final publish (s).
    pub settle_s: f64,
    /// Ignore the first seconds when scoring tracking (s).
    pub warmup_s: f64,
    /// Faults on outgoing traffic.
    pub faults: FaultSpec,
    /// Seed for the fault stream.
    pub fault_seed: u64,
    /// Detection source.
    pub source: Source,
}

impl Default for RunConfig {
    fn default() -> Self {
        Self {
            key_seed: None,
            listen: Vec::new(),
            connect: Vec::new(),
            prefix: crate::keys::DEFAULT_PREFIX.to_string(),
            topic_resolution: godview_core::topics::DEFAULT_TOPIC_RESOLUTION,
            interest_radius_m: 200.0,
            rate_hz: 10.0,
            start_unix_ms: None,
            duration_s: 10.0,
            settle_s: 1.5,
            warmup_s: 2.5,
            faults: FaultSpec::default(),
            fault_seed: 0,
            source: Source::Stdin,
        }
    }
}

/// Errors that stop a node.
#[derive(Debug, thiserror::Error)]
pub enum RunError {
    /// Zenoh failed.
    #[error("zenoh: {0}")]
    Zenoh(String),
    /// Reading input failed.
    #[error("io: {0}")]
    Io(#[from] std::io::Error),
    /// The configuration is unusable.
    #[error("config: {0}")]
    Config(String),
}

fn zerr(e: impl std::fmt::Display) -> RunError {
    RunError::Zenoh(e.to_string())
}

/// The signing key for `seed` (deterministic), or a fresh random key.
pub fn signing_key(seed: Option<u64>) -> SigningKey {
    let mut bytes = [0u8; 32];
    match seed {
        Some(s) => rand_chacha::ChaCha20Rng::seed_from_u64(s ^ 0x6b65_7973).fill_bytes(&mut bytes),
        None => rand::thread_rng().fill_bytes(&mut bytes),
    }
    SigningKey::from_bytes(&bytes)
}

fn zenoh_config(cfg: &RunConfig) -> Result<zenoh::Config, RunError> {
    let mut z = zenoh::Config::default();
    let json =
        |v: &Vec<String>| serde_json::to_string(v).map_err(|e| RunError::Config(e.to_string()));
    z.insert_json5("mode", "\"peer\"").map_err(zerr)?;
    z.insert_json5("listen/endpoints", &json(&cfg.listen)?)
        .map_err(zerr)?;
    z.insert_json5("connect/endpoints", &json(&cfg.connect)?)
        .map_err(zerr)?;
    // Explicit endpoints only: no multicast discovery of unrelated peers.
    z.insert_json5("scouting/multicast/enabled", "false")
        .map_err(zerr)?;
    Ok(z)
}

fn now() -> Timestamp {
    Timestamp::from_system_time(SystemTime::now())
}

async fn sleep_until(t: Timestamp) {
    let wait = t.as_nanos() - now().as_nanos();
    if wait > 0 {
        tokio::time::sleep(Duration::from_nanos(wait as u64)).await;
    }
}

/// Topic subscriptions that follow the agent.
struct Subscriptions {
    prefix: String,
    resolution: Resolution,
    radius_m: f64,
    active: BTreeMap<CellIndex, zenoh::pubsub::Subscriber<()>>,
    tx: mpsc::UnboundedSender<Vec<u8>>,
}

impl Subscriptions {
    /// Subscribes to every cell around `center` and drops the rest.
    async fn follow(&mut self, session: &zenoh::Session, center: &Ecef) -> Result<(), RunError> {
        let Ok(want) = cover_cells(center, self.radius_m, self.resolution) else {
            return Ok(());
        };
        let have: BTreeSet<CellIndex> = self.active.keys().copied().collect();
        if want == have {
            return Ok(());
        }
        self.active.retain(|c, _| want.contains(c));
        for cell in want.difference(&have) {
            let tx = self.tx.clone();
            let sub = session
                .declare_subscriber(report_key(&self.prefix, *cell))
                // Our own batches are already in our store.
                .allowed_origin(zenoh::sample::Locality::Remote)
                .callback(move |sample| {
                    let _ = tx.send(sample.payload().to_bytes().into_owned());
                })
                .await
                .map_err(zerr)?;
            self.active.insert(*cell, sub);
        }
        Ok(())
    }
}

/// Outgoing side: faults, then Zenoh.
struct Outbox {
    prefix: String,
    faults: FaultInjector,
    sent: u64,
    skipped_local: u64,
}

impl Outbox {
    async fn send(
        &mut self,
        session: &zenoh::Session,
        batches: Vec<godview_core::perception_node::CellBatch>,
        reliable: bool,
    ) -> Result<(), RunError> {
        for b in batches {
            // Local-frame reports have no global position, so no peer could fuse them.
            let Some(cell) = b.cell else {
                self.skipped_local += 1;
                continue;
            };
            let key = report_key(&self.prefix, cell);
            let delays = if reliable {
                vec![Duration::ZERO]
            } else {
                self.faults.plan()
            };
            for d in delays {
                self.sent += 1;
                if d.is_zero() {
                    session
                        .put(&key, b.bytes.clone())
                        .congestion_control(CongestionControl::Block)
                        .await
                        .map_err(zerr)?;
                } else {
                    let (s, k, bytes) = (session.clone(), key.clone(), b.bytes.clone());
                    tokio::spawn(async move {
                        tokio::time::sleep(d).await;
                        let _ = s
                            .put(&k, bytes)
                            .congestion_control(CongestionControl::Block)
                            .await;
                    });
                }
            }
        }
        Ok(())
    }
}

/// Drains received batches into the node.
fn drain(
    node: &mut PerceptionNode,
    rx: &mut mpsc::UnboundedReceiver<Vec<u8>>,
    received: &mut u64,
    from: &mut BTreeMap<AgentId, u64>,
) {
    while let Ok(bytes) = rx.try_recv() {
        *received += 1;
        if let Ok(summary) = node.receive(&bytes, now()) {
            if let (Some(src), false) = (summary.source, summary.own_echo) {
                *from.entry(src).or_default() += 1;
            }
        }
    }
}

/// Runs a node to completion and returns its report.
pub async fn run(cfg: RunConfig) -> Result<NodeReport, RunError> {
    if !(cfg.rate_hz > 0.0 && cfg.rate_hz <= 1000.0) {
        return Err(RunError::Config(format!(
            "rate_hz {} out of range",
            cfg.rate_hz
        )));
    }
    let key = signing_key(cfg.key_seed);
    let id = AgentId::from_signing_key(&key);
    let epoch = (now().as_nanos() / 1_000_000_000).clamp(1, u32::MAX as i64) as u32;
    let mut node = PerceptionNode::new(key, epoch, NodeConfig::default());

    let session = zenoh::open(zenoh_config(&cfg)?).await.map_err(zerr)?;
    let (tx, mut rx) = mpsc::unbounded_channel::<Vec<u8>>();
    let mut subs = Subscriptions {
        prefix: cfg.prefix.clone(),
        resolution: cfg.topic_resolution,
        radius_m: cfg.interest_radius_m,
        active: BTreeMap::new(),
        tx,
    };
    let mut out = Outbox {
        prefix: cfg.prefix.clone(),
        faults: FaultInjector::new(cfg.faults, cfg.fault_seed),
        sent: 0,
        skipped_local: 0,
    };
    let mut received = 0u64;
    let mut from: BTreeMap<AgentId, u64> = BTreeMap::new();

    let start = cfg.start_unix_ms.map_or_else(now, Timestamp::from_millis);
    let period_ns = (1e9 / cfg.rate_hz) as i64;
    let mut tracking = TrackingStats::default();
    let mut timing = Timing::with_period_ms(1e3 / cfg.rate_hz);
    let mut pose: Option<PoseEstimate> = None;
    let mut had_pose = false;

    let final_t = match &cfg.source {
        Source::Synthetic {
            spec,
            index,
            offset_enu,
        } => {
            let truth = Truth::new(spec);
            let mut agent = SyntheticAgent::new(spec, *index, *offset_enu);
            let here = site().to_ecef(&agent.position_enu);
            subs.follow(&session, &here).await?;
            sleep_until(start).await;
            let ticks = (cfg.duration_s * cfg.rate_hz).floor() as i64;
            let mut k = 0i64;
            while k < ticks {
                sleep_until(Timestamp(start.as_nanos() + k * period_ns)).await;
                // A tick that starts more than a period late is skipped rather
                // than run late: lag never accumulates, and data is always
                // stamped with the schedule it was produced on.
                let behind = (now().as_nanos() - start.as_nanos()) / period_ns;
                if behind > k {
                    timing.skipped_ticks += (behind - k) as u64;
                    k = behind;
                    if k >= ticks {
                        break;
                    }
                }
                let t = Timestamp(start.as_nanos() + k * period_ns);
                k += 1;
                timing.record_lag(t);
                let work = std::time::Instant::now();
                drain(&mut node, &mut rx, &mut received, &mut from);
                let t_s = t.secs_since(start);
                node.ingest(&agent.detect(t, &truth.positions_enu(t_s)));
                node.maintain(t);
                let p = agent.pose(t);
                let batches = node.publish_by_cell(t, Some(&p), cfg.topic_resolution);
                out.send(&session, batches, false).await?;
                let world = node.world(t);
                timing.record_work(work.elapsed());
                if t_s >= cfg.warmup_s {
                    tracking.record(&score(
                        &world,
                        &expected_truth(spec, &truth, &agent, t_s, cfg.interest_radius_m),
                    ));
                }
            }
            let t_stop = Timestamp(start.as_nanos() + ticks * period_ns);
            sleep_until(t_stop).await;
            timing.final_publish_lag_ms = now().secs_since(t_stop) * 1e3;
            drain(&mut node, &mut rx, &mut received, &mut from);
            // Final reliable round: everyone republishes at the same instant without faults.
            let p = agent.pose(t_stop);
            let batches = node.publish_by_cell(t_stop, Some(&p), cfg.topic_resolution);
            out.send(&session, batches, true).await?;
            pose = Some(p);
            let settle_until = t_stop.add_secs_f64(cfg.settle_s);
            while now() < settle_until {
                tokio::time::sleep(Duration::from_millis(20)).await;
                drain(&mut node, &mut rx, &mut received, &mut from);
            }
            drain(&mut node, &mut rx, &mut received, &mut from);
            let world = node.world_neutral(t_stop);
            let truth_now = expected_truth(
                spec,
                &truth,
                &agent,
                t_stop.secs_since(start),
                cfg.interest_radius_m,
            );
            tracking.final_score = Some(score(&world, &truth_now));
            t_stop
        }
        Source::Stdin => {
            let (line_tx, mut line_rx) =
                mpsc::unbounded_channel::<std::io::Result<Option<String>>>();
            tokio::spawn(async move {
                let mut lines = tokio::io::BufReader::new(tokio::io::stdin()).lines();
                loop {
                    let next = lines.next_line().await;
                    let done = !matches!(next, Ok(Some(_)));
                    if line_tx.send(next).is_err() || done {
                        break;
                    }
                }
            });
            sleep_until(start).await;
            let mut next_obs = 0u64;
            let mut k = 0i64;
            let mut last_t = Timestamp(i64::MIN);
            let mut eof = false;
            let mut seen_any = false;
            while !eof {
                sleep_until(Timestamp(start.as_nanos() + k * period_ns)).await;
                // Skip ticks the loop fell behind on, as in the synthetic loop.
                let behind = (now().as_nanos() - start.as_nanos()) / period_ns;
                if behind > k {
                    timing.skipped_ticks += (behind - k) as u64;
                    k = behind;
                }
                k += 1;
                drain(&mut node, &mut rx, &mut received, &mut from);
                let mut frames: Vec<ParsedFrame> = Vec::new();
                while let Ok(item) = line_rx.try_recv() {
                    match item? {
                        Some(line) if line.trim().is_empty() => {}
                        Some(line) => match parse_line(&line, &mut next_obs) {
                            Ok(f) => frames.push(f),
                            Err(e) => {
                                tracking.input_errors += 1;
                                eprintln!("godview-node: skipping input line: {e}");
                            }
                        },
                        None => eof = true,
                    }
                }
                for f in &frames {
                    node.ingest(&f.detections);
                    // The newest frame decides the pose: a frame without one
                    // means global localization is lost, and tracks are then
                    // published in the local frame (not shared) until it returns.
                    pose = f.pose;
                    if let Some(p) = f.pose {
                        had_pose = true;
                        let here = p.ecef_from_odom.pose.transform_point(&f.agent_odom);
                        subs.follow(&session, &Ecef(here)).await?;
                    }
                    last_t = last_t.max(f.t);
                }
                // Tracks are stamped with the input clock, which should be live
                // (Unix time). Nothing is published before the first frame.
                if seen_any || !frames.is_empty() {
                    seen_any = true;
                    node.maintain(last_t);
                    let batches = node.publish_by_cell(last_t, pose.as_ref(), cfg.topic_resolution);
                    out.send(&session, batches, false).await?;
                }
            }
            if seen_any {
                last_t
            } else {
                now()
            }
        }
    };

    let world = node.world_neutral(final_t);
    let stats = node.stats();
    let report = NodeReport {
        agent_id: hex(&id.0),
        topic_resolution: u8::from(cfg.topic_resolution),
        subscribed_cells: subs.active.keys().map(|c| c.to_string()).collect(),
        batches_sent: out.sent,
        batches_skipped_local: out.skipped_local,
        faults: fault_stats(&out.faults),
        batches_received: received,
        batches_accepted: stats.batches_accepted,
        rejected: stats
            .rejected
            .iter()
            .map(|(r, n)| (r.to_string(), *n))
            .collect(),
        accepted_from: from.iter().map(|(a, n)| (hex(&a.0), *n)).collect(),
        reports_published: stats.reports_published,
        tracking,
        final_world: WorldOut {
            t_unix_ns: final_t.as_nanos(),
            digest: format!("{:016x}", world.digest()),
            objects: world.objects.iter().map(ObjectOut::from_object).collect(),
        },
        pose_known: had_pose || pose.is_some(),
        timing,
    };
    session.close().await.map_err(zerr)?;
    Ok(report)
}

fn fault_stats(f: &FaultInjector) -> FaultStats {
    f.stats()
}

/// The objects this agent is expected to know: those any ring agent can see
/// and that lie within the agent's interest radius.
fn expected_truth(
    spec: &SyntheticSpec,
    truth: &Truth,
    agent: &SyntheticAgent,
    t_s: f64,
    interest_m: f64,
) -> Vec<Ecef> {
    let ring: Vec<Vector3<f64>> = (0..spec.ring_agents)
        .map(|i| SyntheticAgent::new(spec, i, Vector3::zeros()).position_enu)
        .collect();
    truth
        .positions_enu(t_s)
        .into_iter()
        .filter(|p| ring.iter().any(|a| (p - a).norm() <= spec.sensor_range_m))
        .filter(|p| (p - agent.position_enu).norm() <= interest_m)
        .map(|p| site().to_ecef(&p))
        .collect()
}

/// Lowercase hex of a byte string.
pub fn hex(bytes: &[u8]) -> String {
    bytes.iter().map(|b| format!("{b:02x}")).collect()
}
