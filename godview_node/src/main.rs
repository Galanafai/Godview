//! `godview-node`: run one GodView perception-sharing node over Zenoh.
//!
//! ```text
//! # Synthetic agent 0 of a 4-agent ring, listening on 7447:
//! godview-node --listen tcp/127.0.0.1:7447 --synthetic-index 0 --seed 1
//! # A node fed by an external perception stack (JSON lines on stdin):
//! my_detector | godview-node --connect tcp/10.0.0.5:7447
//! ```

use clap::Parser;
use godview_node::faults::FaultSpec;
use godview_node::runner::{run, RunConfig, Source};
use godview_node::synthetic::SyntheticSpec;
use h3o::Resolution;
use nalgebra::Vector3;
use std::path::PathBuf;

#[derive(Parser, Debug)]
#[command(
    name = "godview-node",
    about = "GodView perception-sharing node over Zenoh"
)]
struct Args {
    /// Zenoh endpoint to listen on (repeatable), e.g. tcp/127.0.0.1:7447
    #[arg(long)]
    listen: Vec<String>,
    /// Zenoh endpoint to connect to (repeatable)
    #[arg(long)]
    connect: Vec<String>,
    /// Key expression prefix
    #[arg(long, default_value = godview_node::keys::DEFAULT_PREFIX)]
    prefix: String,
    /// H3 topic resolution (0-15)
    #[arg(long, default_value_t = 8)]
    topic_res: u8,
    /// Subscribe to reports within this radius of the agent (m)
    #[arg(long, default_value_t = 200.0)]
    interest_m: f64,
    /// Publish and fuse rate (Hz)
    #[arg(long, default_value_t = 10.0)]
    rate_hz: f64,
    /// Seed for the signing key (random if omitted)
    #[arg(long)]
    key_seed: Option<u64>,
    /// Start time, Unix milliseconds (default: now)
    #[arg(long)]
    start_unix_ms: Option<i64>,
    /// Use the synthetic world as agent N of the ring (default: read JSON lines on stdin)
    #[arg(long)]
    synthetic_index: Option<usize>,
    /// Synthetic scene seed (same for every node of a run)
    #[arg(long, default_value_t = 1)]
    seed: u64,
    /// Synthetic ring size
    #[arg(long, default_value_t = 4)]
    ring_agents: usize,
    /// Synthetic object count
    #[arg(long, default_value_t = 4)]
    objects: usize,
    /// Displace the synthetic agent east by this distance (m)
    #[arg(long, default_value_t = 0.0)]
    offset_east_m: f64,
    /// Synthetic sensing time (s)
    #[arg(long, default_value_t = 10.0)]
    duration_s: f64,
    /// Wait for peers' final batches (s)
    #[arg(long, default_value_t = 1.5)]
    settle_s: f64,
    /// Outgoing loss probability
    #[arg(long, default_value_t = 0.0)]
    loss: f64,
    /// Outgoing duplication probability
    #[arg(long, default_value_t = 0.0)]
    duplicate: f64,
    /// Outgoing base delay (ms)
    #[arg(long, default_value_t = 0)]
    delay_ms: u64,
    /// Outgoing uniform jitter (ms)
    #[arg(long, default_value_t = 0)]
    jitter_ms: u64,
    /// Seed for the fault stream
    #[arg(long, default_value_t = 0)]
    fault_seed: u64,
    /// Write the final report (JSON) here; stdout if omitted
    #[arg(long)]
    report: Option<PathBuf>,
}

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let a = Args::parse();
    let topic_resolution = Resolution::try_from(a.topic_res)?;
    let source = match a.synthetic_index {
        Some(index) => Source::Synthetic {
            spec: SyntheticSpec {
                seed: a.seed,
                ring_agents: a.ring_agents,
                objects: a.objects,
                duration_s: a.duration_s + a.settle_s,
                ..SyntheticSpec::default()
            },
            index,
            offset_enu: Vector3::new(a.offset_east_m, 0.0, 0.0),
        },
        None => Source::Stdin,
    };
    let cfg = RunConfig {
        key_seed: a.key_seed,
        listen: a.listen,
        connect: a.connect,
        prefix: a.prefix,
        topic_resolution,
        interest_radius_m: a.interest_m,
        rate_hz: a.rate_hz,
        start_unix_ms: a.start_unix_ms,
        duration_s: a.duration_s,
        settle_s: a.settle_s,
        faults: FaultSpec {
            loss: a.loss,
            duplicate: a.duplicate,
            delay_ms: a.delay_ms,
            jitter_ms: a.jitter_ms,
        },
        fault_seed: a.fault_seed,
        source,
        ..RunConfig::default()
    };
    let report = run(cfg).await?;
    let json = serde_json::to_string_pretty(&report)?;
    match a.report {
        Some(path) => std::fs::write(path, json)?,
        None => println!("{json}"),
    }
    Ok(())
}
