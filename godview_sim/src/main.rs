//! GodView DST Simulator CLI
//!
//! Run deterministic simulation tests with chaos engineering scenarios.

use clap::Parser;
use godview_sim::{ScenarioRunner, ScenarioResult};
use godview_sim::scenarios::ScenarioId;
use godview_sim::{SimExport, SimFrame, EntityPosition, AgentFrame, TrackPosition};
use godview_sim::{SimContext, SimNetwork, SimulatedAgent, Oracle, DeterministicKeyProvider, SensorReading};
use godview_core::AgentConfig;
use godview_env::NodeId;
use nalgebra::Vector3;
use std::sync::Arc;
use std::time::Duration;
use tracing::{info, error, debug, Level};
use tracing_subscriber::FmtSubscriber;

/// Run a scenario with frame-by-frame export for visualization.
fn run_with_export(
    seed: u64,
    _num_agents: usize,
    scenario: ScenarioId,
    duration: f64,
    export_path: &str,
) -> ScenarioResult {
    let context_seed = seed;
    let physics_seed = seed.wrapping_mul(0x9e3779b97f4a7c15);
    
    let context = Arc::new(SimContext::new(context_seed));
    let network = Arc::new(SimNetwork::new_stub(NodeId::from_seed(0)));
    let key_provider = DeterministicKeyProvider::new(seed);
    let root_key = key_provider.biscuit_root_key().public();
    
    let mut oracle = Oracle::new(physics_seed);
    let mut agent = SimulatedAgent::new(
        context.clone(),
        network,
        root_key,
        0,
        AgentConfig::default(),
    );
    
    let mut export = SimExport::new(scenario.name(), seed);
    
    // Spawn entities based on scenario
    let num_entities = match scenario {
        ScenarioId::FlashMob => 100, // Reduced for visualization
        _ => 10,
    };
    
    for i in 0..num_entities {
        let pos = Vector3::new(
            (i as f64) * 50.0,
            (i as f64 % 10.0) * 20.0,
            100.0 + (i as f64) * 5.0,
        );
        let vel = Vector3::new(20.0, 5.0 * (i as f64 - 5.0), 0.0);
        oracle.spawn_entity(pos, vel, "drone");
    }
    
    let tick_rate_hz = 30;
    let dt = 1.0 / tick_rate_hz as f64;
    let target_ticks = (duration * tick_rate_hz as f64) as u64;
    
    // Export every 10 ticks (3 FPS in Rerun)
    let export_interval = 10;
    
    for tick in 0..target_ticks {
        oracle.step(dt);
        context.advance_time(Duration::from_secs_f64(dt));
        agent.tick();
        
        let readings = oracle.generate_sensor_readings();
        agent.ingest_readings(&readings);
        
        // Export frame periodically
        if tick % export_interval == 0 {
            let ground_truth: Vec<EntityPosition> = oracle.ground_truth_positions()
                .into_iter()
                .map(|(id, pos)| EntityPosition::new(id, pos))
                .collect();
            
            let tracks: Vec<TrackPosition> = agent.track_positions()
                .into_iter()
                .map(|(uuid, pos)| TrackPosition {
                    track_id: uuid.to_string(),
                    x: pos.x,
                    y: pos.y,
                    z: pos.z,
                })
                .collect();
            
            let gt_for_error = oracle.ground_truth_positions();
            let rms_error = agent.compute_position_error(&gt_for_error);
            
            let frame = SimFrame {
                time_sec: oracle.time(),
                ground_truth,
                agents: vec![AgentFrame {
                    agent_id: 0,
                    tracks,
                    rms_error: Some(rms_error),
                }],
                events: vec![],
            };
            
            export.add_frame(frame);
        }
        
        if tick % 30 == 0 {
            debug!("  t={:.1}s | entities={} | tracks={}", 
                oracle.time(), 
                oracle.active_entities().len(),
                agent.track_count()
            );
        }
    }
    
    let ground_truth = oracle.ground_truth_positions();
    let rms_error = agent.compute_position_error(&ground_truth);
    let passed = rms_error < 5.0;
    
    export.finalize(passed, Some(rms_error));
    
    if let Err(e) = export.write_to_file(export_path) {
        error!("Failed to write export: {:?}", e);
    } else {
        info!("Exported {} frames to {}", export.frames.len(), export_path);
    }
    
    ScenarioResult {
        scenario,
        seed,
        passed,
        total_ticks: target_ticks,
        final_time_secs: oracle.time(),
        final_entity_count: oracle.active_entities().len(),
        failure_reason: if !passed {
            Some(format!("RMS error {:.2}m exceeds threshold", rms_error))
        } else {
            None
        },
        metrics: godview_sim::ScenarioMetrics::default(),
    }
}



/// GodView Deterministic Simulation Testing CLI
#[derive(Parser, Debug)]
#[command(name = "godview-sim")]
#[command(about = "Run deterministic simulation tests for GodView", long_about = None)]
struct Args {
    /// Master seed for determinism (0 = random from time)
    #[arg(short, long, default_value = "42")]
    seed: u64,
    
    /// Number of agent nodes
    #[arg(short, long, default_value = "6")]
    agents: usize,
    
    /// Scenario to run (time_warp, split_brain, byzantine, flash_mob, slow_loris, all)
    #[arg(short = 'S', long, default_value = "all")]
    scenario: String,
    
    /// Number of random seeds to test (for CI mode)
    #[arg(long, default_value = "1")]
    seeds: usize,
    
    /// Maximum simulation duration in seconds
    #[arg(short, long, default_value = "10")]
    duration: f64,
    
    /// Verbose output
    #[arg(short, long)]
    verbose: bool,
    
    /// JSON output for CI parsing
    #[arg(long)]
    json: bool,
    
    /// Export simulation data to JSON file for Rerun visualization
    #[arg(long)]
    export: Option<String>,
}

fn main() {
    let args = Args::parse();
    
    // Initialize logging
    let level = if args.verbose { Level::DEBUG } else { Level::INFO };
    let subscriber = FmtSubscriber::builder()
        .with_max_level(level)
        .finish();
    tracing::subscriber::set_global_default(subscriber)
        .expect("Failed to set tracing subscriber");
    
    if !args.json {
        info!("GodView DST Simulator v0.1.0");
        info!("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    }
    
    // Parse scenarios
    let scenarios: Vec<ScenarioId> = if args.scenario == "all" {
        ScenarioId::all()
    } else {
        vec![args.scenario.parse().unwrap_or_else(|e| {
            eprintln!("Error: {}", e);
            eprintln!("Available scenarios: time_warp, split_brain, byzantine, flash_mob, slow_loris, all");
            std::process::exit(1);
        })]
    };
    
    // Determine base seed
    let base_seed = if args.seed == 0 {
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos() as u64
    } else {
        args.seed
    };
    
    // Track results
    let mut all_results: Vec<ScenarioResult> = Vec::new();
    let mut failed_count = 0;
    
    // Handle --export mode for visualization
    if let Some(export_path) = &args.export {
        if scenarios.len() > 1 {
            eprintln!("Error: --export only supports a single scenario, not 'all'");
            std::process::exit(1);
        }
        
        info!("Running with export to: {}", export_path);
        
        // Run specialized export simulation
        let result = run_with_export(
            base_seed, 
            args.agents, 
            scenarios[0], 
            args.duration,
            export_path,
        );
        
        if result.passed {
            info!("✓ {} (seed={}) PASSED - exported to {}", 
                scenarios[0].name(), base_seed, export_path);
            info!("Visualize with: python godview_sim/visualize.py {}", export_path);
        } else {
            error!("✗ {} FAILED: {}", 
                scenarios[0].name(),
                result.failure_reason.as_deref().unwrap_or("unknown")
            );
        }
        
        if !result.passed {
            std::process::exit(1);
        }
        return;
    }
    
    // Run simulations
    for seed_offset in 0..args.seeds {
        let seed = base_seed.wrapping_add(seed_offset as u64);
        
        let runner = ScenarioRunner::new(seed, args.agents)
            .with_duration(args.duration);
        
        for scenario in &scenarios {
            let result = runner.run(*scenario);
            
            if !args.json {
                if result.passed {
                    info!("✓ {} (seed={}) PASSED", scenario.name(), seed);
                } else {
                    error!("✗ {} (seed={}) FAILED: {}", 
                        scenario.name(), 
                        seed, 
                        result.failure_reason.as_deref().unwrap_or("unknown")
                    );
                }
            }
            
            if !result.passed {
                failed_count += 1;
            }
            
            all_results.push(result);
        }
    }
    
    // Summary
    let total = all_results.len();
    let passed = total - failed_count;
    
    if args.json {
        // JSON output for CI parsing
        let summary = serde_json::json!({
            "total": total,
            "passed": passed,
            "failed": failed_count,
            "results": all_results.iter().map(|r| {
                serde_json::json!({
                    "scenario": r.scenario.name(),
                    "seed": r.seed,
                    "passed": r.passed,
                    "ticks": r.total_ticks,
                    "time_secs": r.final_time_secs,
                    "failure_reason": r.failure_reason,
                })
            }).collect::<Vec<_>>(),
        });
        println!("{}", serde_json::to_string_pretty(&summary).unwrap());
    } else {
        info!("");
        info!("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
        
        if failed_count == 0 {
            info!("✅ All {} scenario runs passed!", total);
        } else {
            error!("❌ {}/{} scenario runs failed!", failed_count, total);
            
            // List failed seeds
            for result in &all_results {
                if !result.passed {
                    error!("  - {} seed={}: {}", 
                        result.scenario.name(),
                        result.seed,
                        result.failure_reason.as_deref().unwrap_or("unknown")
                    );
                }
            }
        }
    }
    
    // Exit with proper code for CI
    if failed_count > 0 {
        std::process::exit(1);
    }
}
