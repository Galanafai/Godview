//! GodView DST Simulator CLI
//!
//! Run deterministic simulation tests with chaos engineering scenarios.

use clap::Parser;
use godview_sim::{ScenarioRunner, ScenarioResult};
use godview_sim::scenarios::ScenarioId;
use tracing::{info, error, Level};
use tracing_subscriber::FmtSubscriber;

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
