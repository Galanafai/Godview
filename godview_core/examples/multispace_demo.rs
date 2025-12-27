//! Multi-Space GodView Demo - Problem | Data | Solution
//! ======================================================
//!
//! Shows THREE concurrent viewports in one file:
//!
//! 1. PROBLEM SPACE - Multiple agents creating ghost tracks
//! 2. DATA SPACE    - Raw detections from each agent
//! 3. SOLUTION SPACE - Highlander merging duplicates
//!
//! Each space is a separate entity root, so Rerun creates separate viewports.
//!
//! Run:
//! ```bash
//! cargo run --example multispace_demo --features visualization -- --save multispace_demo.rrd
//! ```

use godview_core::{
    GlobalHazardPacket, TrackManager,
    visualization::RerunVisualizer,
    metrics::{calculate_ghost_score, GhostScoreConfig},
};
use nalgebra::Vector3;
use rand::Rng;
use uuid::Uuid;

const FRAME_RATE: f64 = 30.0;
const DURATION_SECONDS: f64 = 20.0; // Shorter for clarity
const TOTAL_FRAMES: usize = (FRAME_RATE * DURATION_SECONDS) as usize;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("🎭 Multi-Space GodView Demo");
    println!("============================\n");
    println!("Creates 3 viewports: PROBLEM | DATA | SOLUTION\n");

    // Create visualizer
    let args: Vec<String> = std::env::args().collect();
    let viz = if args.len() > 1 && args[1] == "--save" {
        let path = args.get(2).map(|s| s.as_str()).unwrap_or("multispace_demo.rrd");
        println!("📹 Saving to: {}\n", path);
        RerunVisualizer::new_to_file("GodView Multi-Space", path)?
    } else {
        RerunVisualizer::new("GodView Multi-Space")?
    };

    // =========================================================================
    // SETUP: 3 Agents, 2 Moving Objects (simple for clarity)
    // =========================================================================
    
    let agents = vec![
        ("CAM_A", Vector3::new(-30.0, 0.0, 8.0), [255, 100, 100, 255u8]),
        ("CAM_B", Vector3::new(30.0, 0.0, 8.0), [100, 200, 255, 255u8]),
        ("DRONE", Vector3::new(0.0, 0.0, 25.0), [255, 215, 0, 255u8]),
    ];
    
    let objects = vec![
        ("CAR_1", Vector3::new(-15.0, -20.0, 1.0), Vector3::new(0.0, 3.0, 0.0)),
        ("CAR_2", Vector3::new(10.0, 20.0, 1.0), Vector3::new(0.0, -2.0, 0.0)),
    ];

    let mut track_manager = TrackManager::with_defaults();
    let ghost_config = GhostScoreConfig::default();
    let mut rng = rand::thread_rng();
    
    println!("▶️  Running simulation...\n");
    println!("   SPACE 1: 'problem/' - Shows ghost ellipsoids forming");
    println!("   SPACE 2: 'data/'    - Shows raw agent detections");
    println!("   SPACE 3: 'solution/'- Shows merged tracks (future)\n");

    // =========================================================================
    // SIMULATION
    // =========================================================================
    
    for frame in 0..TOTAL_FRAMES {
        let t = frame as f64 / FRAME_RATE;
        viz.set_time("frame", frame as u64);

        // -----------------------------------------------------------------
        // SPACE 1: PROBLEM - Ground plane + agents + ghosts
        // -----------------------------------------------------------------
        
        // Ground (only first frame)
        if frame == 0 {
            viz.rec.log_static(
                "problem/ground",
                &rerun::Boxes3D::from_centers_and_sizes(
                    [[0.0, 0.0, 0.0]],
                    [[100.0, 100.0, 0.1]],
                ).with_colors([[80, 80, 80, 100]])
            )?;
            
            // Title label
            viz.rec.log_static(
                "problem/title",
                &rerun::TextLog::new("🔴 THE PROBLEM: Multiple IDs for same object")
            )?;
        }
        
        // Agents in problem space
        for (name, pos, color) in &agents {
            viz.rec.log(
                format!("problem/agents/{}", name),
                &rerun::Points3D::new([[pos.x as f32, pos.y as f32, pos.z as f32]])
                    .with_colors([*color])
                    .with_radii([2.0])
                    .with_labels([*name])
            )?;
        }
        
        // Ground truth objects
        for (name, init_pos, vel) in &objects {
            let pos = init_pos + vel * t;
            viz.rec.log(
                format!("problem/objects/{}", name),
                &rerun::Boxes3D::from_centers_and_sizes(
                    [[pos.x as f32, pos.y as f32, pos.z as f32]],
                    [[4.0, 2.0, 1.5]],
                ).with_colors([[200, 200, 200, 150]])
                 .with_labels([format!("{} (true)", name)])
            )?;
        }

        // -----------------------------------------------------------------
        // SPACE 2: DATA - Raw detections from each agent
        // -----------------------------------------------------------------
        
        if frame == 0 {
            viz.rec.log_static(
                "data/ground",
                &rerun::Boxes3D::from_centers_and_sizes(
                    [[0.0, 0.0, 0.0]],
                    [[100.0, 100.0, 0.1]],
                ).with_colors([[60, 100, 80, 100]])
            )?;
            
            viz.rec.log_static(
                "data/title",
                &rerun::TextLog::new("📡 RAW DATA: Each agent reports with local IDs")
            )?;
        }
        
        // Each agent detects each object (with noise) → feeds to TrackManager
        for (agent_name, agent_pos, agent_color) in &agents {
            for (obj_name, init_pos, vel) in &objects {
                let obj_pos = init_pos + vel * t;
                
                // Add noise
                let noise = Vector3::new(
                    rng.gen_range(-1.5..1.5),
                    rng.gen_range(-1.5..1.5),
                    rng.gen_range(-0.3..0.3),
                );
                let detected_pos = obj_pos + noise;
                
                // Log detection in DATA space (colored by agent)
                viz.rec.log(
                    format!("data/{}/{}", agent_name, obj_name),
                    &rerun::Points3D::new([[detected_pos.x as f32, detected_pos.y as f32, detected_pos.z as f32]])
                        .with_colors([*agent_color])
                        .with_radii([0.5])
                        .with_labels([format!("{} → {}", agent_name, obj_name)])
                )?;
                
                // Create unique ID per agent-object pair (causes ghosts!)
                let agent_hash = agent_name.as_bytes().iter().fold(0u128, |a, &b| a.wrapping_mul(31).wrapping_add(b as u128));
                let obj_hash = obj_name.as_bytes().iter().fold(0u128, |a, &b| a.wrapping_mul(31).wrapping_add(b as u128));
                let local_id = Uuid::from_u128(agent_hash ^ obj_hash);
                
                let packet = GlobalHazardPacket {
                    entity_id: local_id,
                    position: [detected_pos.x, detected_pos.y, detected_pos.z],
                    velocity: [vel.x, vel.y, vel.z],
                    class_id: 1,
                    timestamp: t,
                    confidence_score: 0.9,
                };
                
                let _ = track_manager.process_packet(&packet);
            }
        }

        // -----------------------------------------------------------------
        // SPACE 3: SOLUTION - Fused tracks with ghost scoring
        // -----------------------------------------------------------------
        
        if frame == 0 {
            viz.rec.log_static(
                "solution/ground",
                &rerun::Boxes3D::from_centers_and_sizes(
                    [[0.0, 0.0, 0.0]],
                    [[100.0, 100.0, 0.1]],
                ).with_colors([[60, 80, 100, 100]])
            )?;
            
            viz.rec.log_static(
                "solution/title",
                &rerun::TextLog::new("✅ SOLUTION: Highlander merges duplicate IDs")
            )?;
        }
        
        let tracks: Vec<_> = track_manager.tracks().collect();
        let track_data: Vec<_> = tracks.iter()
            .map(|t| (
                [t.state[0], t.state[1], t.state[2]],
                [t.state[3], t.state[4], t.state[5]],
                t.covariance.clone()
            ))
            .collect();
        
        let mut ghost_count = 0;
        
        for (i, track) in tracks.iter().enumerate() {
            let pos = [track.state[0], track.state[1], track.state[2]];
            let vel = [track.state[3], track.state[4], track.state[5]];
            
            let neighbors: Vec<_> = track_data.iter()
                .enumerate()
                .filter(|(j, _)| *j != i)
                .map(|(_, d)| d.clone())
                .collect();
            
            let (ghost_score, _, _) = calculate_ghost_score(
                &pos, &vel, &track.covariance,
                1, 3, &neighbors, &ghost_config,
            );
            
            if ghost_score > 0.7 { ghost_count += 1; }
            
            // Color by ghost score
            let color: [u8; 4] = if ghost_score > 0.7 {
                [255, 50, 50, 200] // Red = ghost
            } else if ghost_score > 0.3 {
                [255, 200, 50, 200] // Yellow = uncertain
            } else {
                [50, 255, 50, 200] // Green = solid
            };
            
            viz.rec.log(
                format!("solution/tracks/{}", track.canonical_id),
                &rerun::Ellipsoids3D::from_centers_and_half_sizes(
                    [[pos[0] as f32, pos[1] as f32, pos[2] as f32]],
                    [[1.0, 1.0, 1.0]],
                )
                .with_colors([color])
                .with_labels([format!("GS:{:.0}%", ghost_score * 100.0)])
            )?;
        }

        // Progress
        if frame % 60 == 0 {
            println!("  t={:.0}s | Tracks: {} | Ghosts: {}", t, tracks.len(), ghost_count);
        }
    }

    println!("\n✅ Multi-space demo complete!");
    println!("\n📺 In Rerun:");
    println!("   1. You should see 3 separate viewports");
    println!("   2. Drag/arrange them side by side");
    println!("   3. Each shows: problem / data / solution");

    Ok(())
}
