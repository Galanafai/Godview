//! GodView DST Simulator CLI
//!
//! Run deterministic simulation tests with chaos engineering scenarios.

use clap::Parser;
use godview_sim::{SimWorld, SimConfig};
use godview_sim::scenarios::ScenarioId;
use tracing::{info, Level};
use tracing_subscriber::FmtSubscriber;

/// GodView Deterministic Simulation Testing CLI
#[derive(Parser, Debug)]
#[command(name = "godview-sim")]
#[command(about = "Run deterministic simulation tests for GodView", long_about = None)]
struct Args {
    /// Master seed for determinism (0 = random)
    #[arg(short, long, default_value = "42")]
    seed: u64,
    
    /// Number of agent nodes
    #[arg(short, long, default_value = "6")]
    agents: usize,
    
    /// Scenario to run (time_warp, split_brain, byzantine, flash_mob, slow_loris, all)
    #[arg(short = 'S', long, default_value = "time_warp")]
    scenario: String,
    
    /// Number of random seeds to test (for CI mode)
    #[arg(long, default_value = "1")]
    seeds: usize,
    
    /// Maximum simulation duration in seconds
    #[arg(short, long, default_value = "60")]
    duration: f64,
    
    /// Verbose output
    #[arg(short, long)]
    verbose: bool,
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
    
    info!("GodView DST Simulator v0.1.0");
    info!("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    
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
    
    // Run simulations
    let base_seed = if args.seed == 0 {
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos() as u64
    } else {
        args.seed
    };
    
    for seed_offset in 0..args.seeds {
        let seed = base_seed.wrapping_add(seed_offset as u64);
        
        for scenario in &scenarios {
            info!("");
            info!("┌─────────────────────────────────────────────────────");
            info!("│ Scenario: {} ({})", scenario.name(), scenario.description());
            info!("│ Seed: {}", seed);
            info!("│ Agents: {}", args.agents);
            info!("└─────────────────────────────────────────────────────");
            
            let config = SimConfig {
                seed,
                num_agents: args.agents,
                max_duration_secs: args.duration,
                ..Default::default()
            };
            
            // Create world
            let mut world = SimWorld::new(config);
            let agent_ids = world.spawn_agents();
            
            info!("Spawned {} agents: {:?}", agent_ids.len(), 
                agent_ids.iter().map(|id| id.to_string()).collect::<Vec<_>>());
            
            // Run simulation ticks
            let target_ticks = (args.duration * world.config.tick_rate_hz as f64) as u64;
            
            for tick in 0..target_ticks {
                world.tick();
                
                // Progress logging every second of simulated time
                if tick % world.config.tick_rate_hz as u64 == 0 {
                    info!("  t={:.1}s | tick={} | entities={}", 
                        world.time(), 
                        world.tick_count(),
                        world.oracle.active_entities().len()
                    );
                }
            }
            
            info!("✓ Scenario complete: {} ticks in {:.2}s virtual time", 
                world.tick_count(), world.time());
        }
    }
    
    info!("");
    info!("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    info!("All simulations complete!");
}
