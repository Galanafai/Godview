//! GodView Showcase Demo - See the Fusion Engine in Action
//! =========================================================
//!
//! This demo clearly visualizes HOW godview_core works:
//! 
//! 1. **Multiple Agents** see the same objects
//! 2. **TrackManager** creates tracks for each detection
//! 3. **Ghost Detection** identifies duplicate tracks
//! 4. **Highlander Merge** collapses ghosts into canonical IDs
//! 5. **Covariance Intersection** fuses state estimates
//!
//! Visual Elements:
//! - Raw detections (small colored dots) from each agent
//! - Fused tracks (ellipsoids) from TrackManager
//! - Ghost score coloring (green=solid, red=ghost)
//! - Detection lines showing which agent saw what
//! - Info panel with live statistics
//!
//! Run:
//! ```bash
//! cargo run --example godview_showcase --features visualization -- --save godview_showcase.rrd
//! ```

use godview_core::{
    GlobalHazardPacket, TrackManager, TrackingConfig,
    visualization::RerunVisualizer,
    metrics::{calculate_ghost_score, calculate_entropy, GhostScoreConfig},
};
use nalgebra::{Matrix6, Vector3};
use rand::Rng;
use uuid::Uuid;

const FRAME_RATE: f64 = 30.0;
const DURATION_SECONDS: f64 = 30.0; // 30 second demo
const TOTAL_FRAMES: usize = (FRAME_RATE * DURATION_SECONDS) as usize;

// Agent definition
struct Agent {
    name: String,
    id: Uuid,
    position: Vector3<f64>,
    color: [u8; 4],
    range: f64,
    is_drone: bool,
    noise_scale: f64,
}

// Ground truth object
struct GroundTruthObject {
    id: Uuid,
    name: String,
    initial_pos: Vector3<f64>,
    velocity: Vector3<f64>,
    class: &'static str,
    size: [f32; 3],
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("🎯 GodView Showcase - See the Fusion Engine in Action!");
    println!("========================================================\n");

    // Create visualizer
    let args: Vec<String> = std::env::args().collect();
    let viz = if args.len() > 1 && args[1] == "--save" {
        let path = args.get(2).map(|s| s.as_str()).unwrap_or("godview_showcase.rrd");
        println!("📹 Saving to: {}\n", path);
        RerunVisualizer::new_to_file("GodView Showcase", path)?
    } else {
        println!("📹 Opening Rerun viewer...\n");
        RerunVisualizer::new("GodView Showcase")?
    };

    // Ground plane
    viz.log_ground_plane(150.0, 15)?;

    // =========================================================================
    // DEFINE AGENTS - 4 fixed cameras + 1 drone
    // =========================================================================
    
    let mut agents = vec![
        Agent {
            name: "CAM_A".to_string(),
            id: Uuid::new_v4(),
            position: Vector3::new(-40.0, -40.0, 10.0),
            color: [255, 100, 100, 255], // Red
            range: 50.0,
            is_drone: false,
            noise_scale: 2.0,
        },
        Agent {
            name: "CAM_B".to_string(),
            id: Uuid::new_v4(),
            position: Vector3::new(40.0, -40.0, 10.0),
            color: [100, 150, 255, 255], // Blue
            range: 50.0,
            is_drone: false,
            noise_scale: 1.5,
        },
        Agent {
            name: "CAM_C".to_string(),
            id: Uuid::new_v4(),
            position: Vector3::new(40.0, 40.0, 10.0),
            color: [100, 255, 150, 255], // Green
            range: 50.0,
            is_drone: false,
            noise_scale: 1.8,
        },
        Agent {
            name: "CAM_D".to_string(),
            id: Uuid::new_v4(),
            position: Vector3::new(-40.0, 40.0, 10.0),
            color: [255, 200, 100, 255], // Orange
            range: 50.0,
            is_drone: false,
            noise_scale: 2.2,
        },
        Agent {
            name: "DRONE".to_string(),
            id: Uuid::new_v4(),
            position: Vector3::new(0.0, 0.0, 30.0), // Will move
            color: [255, 215, 0, 255], // Gold
            range: 80.0,
            is_drone: true,
            noise_scale: 0.5, // High accuracy
        },
    ];

    // Log fixed agents
    for agent in agents.iter().filter(|a| !a.is_drone) {
        viz.log_agent(
            &agent.name,
            [agent.position.x, agent.position.y, agent.position.z],
            [2.0, 2.0, 3.0],
            agent.color,
            false,
        )?;
        viz.log_sensor_range(
            &agent.name,
            [agent.position.x, agent.position.y, 0.0],
            agent.range as f32,
            agent.color,
        )?;
    }

    // =========================================================================
    // DEFINE GROUND TRUTH OBJECTS - 5 objects for clear visualization
    // =========================================================================
    
    let objects = vec![
        GroundTruthObject {
            id: Uuid::new_v4(),
            name: "CAR_1".to_string(),
            initial_pos: Vector3::new(0.0, 0.0, 1.0),
            velocity: Vector3::new(3.0, 0.0, 0.0), // Moving east
            class: "car",
            size: [4.5, 2.0, 1.6],
        },
        GroundTruthObject {
            id: Uuid::new_v4(),
            name: "CAR_2".to_string(),
            initial_pos: Vector3::new(-20.0, 10.0, 1.0),
            velocity: Vector3::new(0.0, 4.0, 0.0), // Moving north
            class: "car",
            size: [4.5, 2.0, 1.6],
        },
        GroundTruthObject {
            id: Uuid::new_v4(),
            name: "TRUCK".to_string(),
            initial_pos: Vector3::new(15.0, -15.0, 1.5),
            velocity: Vector3::new(-2.0, 2.0, 0.0), // Moving NW
            class: "truck",
            size: [8.0, 2.5, 3.0],
        },
        GroundTruthObject {
            id: Uuid::new_v4(),
            name: "PED_1".to_string(),
            initial_pos: Vector3::new(-10.0, -5.0, 0.9),
            velocity: Vector3::new(0.0, 1.2, 0.0), // Walking north
            class: "pedestrian",
            size: [0.6, 0.6, 1.8],
        },
        GroundTruthObject {
            id: Uuid::new_v4(),
            name: "CYCLIST".to_string(),
            initial_pos: Vector3::new(5.0, 20.0, 1.0),
            velocity: Vector3::new(5.0, -3.0, 0.0), // Moving SE fast
            class: "cyclist",
            size: [1.8, 0.6, 1.5],
        },
    ];

    println!("📍 Scene Setup:");
    println!("   5 Agents: CAM_A, CAM_B, CAM_C, CAM_D, DRONE");
    println!("   5 Objects: CAR_1, CAR_2, TRUCK, PED_1, CYCLIST\n");

    // =========================================================================
    // INITIALIZE GODVIEW TRACKMANAGER
    // =========================================================================
    
    let mut track_manager = TrackManager::with_defaults();
    let ghost_config = GhostScoreConfig::default();
    let mut rng = rand::thread_rng();
    
    // Statistics
    let mut total_detections = 0usize;
    let mut total_ghosts = 0usize;
    
    println!("▶️  Running simulation...\n");
    println!("   Watch: Colored dots = Agent detections");
    println!("          Ellipsoids = Fused tracks (green=solid, red=ghost)");
    println!("          Lines = Detection links\n");

    // =========================================================================
    // SIMULATION LOOP
    // =========================================================================
    
    for frame in 0..TOTAL_FRAMES {
        let t = frame as f64 / FRAME_RATE;
        viz.set_time("frame", frame as u64);
        
        // Update drone position (figure-8)
        agents[4].position = Vector3::new(
            (t * 0.3).sin() * 50.0,
            (t * 0.6).sin() * 30.0,
            30.0 + (t * 0.5).cos() * 5.0,
        );
        
        viz.log_agent(
            "DRONE",
            [agents[4].position.x, agents[4].position.y, agents[4].position.z],
            [3.0, 3.0, 1.0],
            [255, 215, 0, 255],
            true,
        )?;

        // =====================================================================
        // STEP 1: Log Ground Truth Positions
        // =====================================================================
        for obj in &objects {
            let gt_pos = obj.initial_pos + obj.velocity * t;
            
            // Log as white transparent box (ground truth)
            viz.log_class_bbox(
                &format!("GT_{}", obj.class),
                &obj.name,
                [gt_pos.x, gt_pos.y, gt_pos.z],
                obj.size,
                0.0,
                1.0,
            )?;
        }

        // =====================================================================
        // STEP 2: Each Agent Detects Objects (with noise)
        // =====================================================================
        for agent in &agents {
            let agent_pos = agent.position;
            
            for obj in &objects {
                let obj_pos = obj.initial_pos + obj.velocity * t;
                let dist = (obj_pos - agent_pos).norm();
                
                // Only detect if in range
                if dist > agent.range { continue; }
                
                total_detections += 1;
                
                // Add noise
                let noise = Vector3::new(
                    rng.gen_range(-1.0..1.0) * agent.noise_scale,
                    rng.gen_range(-1.0..1.0) * agent.noise_scale,
                    rng.gen_range(-0.2..0.2) * agent.noise_scale,
                );
                let detected_pos = obj_pos + noise;
                
                // Create detection packet
                // IMPORTANT: Each agent creates its OWN ID for the object
                // This is the source of "ghost" tracks!
                let agent_hash = agent.id.as_u128();
                let obj_hash = obj.name.as_bytes().iter().fold(0u128, |acc, &b| acc.wrapping_mul(31).wrapping_add(b as u128));
                let local_id = Uuid::from_u128(agent_hash ^ obj_hash);
                
                let packet = GlobalHazardPacket {
                    entity_id: local_id,
                    position: [detected_pos.x, detected_pos.y, detected_pos.z],
                    velocity: [obj.velocity.x, obj.velocity.y, obj.velocity.z],
                    class_id: match obj.class {
                        "car" => 1,
                        "truck" => 1,
                        "pedestrian" => 2,
                        "cyclist" => 3,
                        _ => 0,
                    },
                    timestamp: t,
                    confidence_score: 1.0 - (dist / agent.range) * 0.3,
                };
                
                // Log raw detection as small colored dot
                viz.log_packet_detection(&packet, agent.noise_scale as f32 * 0.3)?;
                
                // Log detection line (agent -> detection)
                viz.log_detection_line(
                    &format!("{}_{}", agent.name, obj.name),
                    [agent_pos.x, agent_pos.y, agent_pos.z],
                    [detected_pos.x, detected_pos.y, detected_pos.z],
                    [agent.color[0], agent.color[1], agent.color[2], 50],
                )?;
                
                // =========================================================
                // STEP 3: Feed to TrackManager (Covariance Intersection)
                // =========================================================
                let _ = track_manager.process_packet(&packet);
            }
        }

        // =====================================================================
        // STEP 4: Visualize Fused Tracks with Ghost Scores
        // =====================================================================
        let tracks: Vec<_> = track_manager.tracks().collect();
        
        // Prepare neighbor data
        let track_data: Vec<_> = tracks.iter()
            .map(|t| (
                [t.state[0], t.state[1], t.state[2]],
                [t.state[3], t.state[4], t.state[5]],
                t.covariance.clone()
            ))
            .collect();
        
        let mut frame_ghosts = 0;
        
        for (i, track) in tracks.iter().enumerate() {
            let track_pos = [track.state[0], track.state[1], track.state[2]];
            let track_vel = [track.state[3], track.state[4], track.state[5]];
            
            // Get neighbors
            let neighbors: Vec<_> = track_data.iter()
                .enumerate()
                .filter(|(j, _)| *j != i)
                .map(|(_, d)| d.clone())
                .collect();
            
            // Calculate ghost score
            let (ghost_score, _, _) = calculate_ghost_score(
                &track_pos,
                &track_vel,
                &track.covariance,
                track.observed_ids.len(),
                agents.len(),
                &neighbors,
                &ghost_config,
            );
            
            if ghost_score > 0.7 {
                frame_ghosts += 1;
            }
            
            // Log with Ghost Hunter coloring
            viz.log_track_with_ghost_score(
                track.canonical_id,
                track_pos,
                track_vel,
                &track.covariance,
                ghost_score,
                frame,
            )?;
        }
        
        total_ghosts = frame_ghosts;
        
        // =====================================================================
        // STEP 5: Log Statistics
        // =====================================================================
        if frame % 30 == 0 {
            let entropy_avg = if tracks.is_empty() { 0.0 } else {
                tracks.iter()
                    .map(|t| calculate_entropy(&t.covariance))
                    .sum::<f64>() / tracks.len() as f64
            };
            
            viz.log_stats(tracks.len(), entropy_avg, total_detections as f64)?;
            
            println!("  t={:.1}s | Detections: {} | Tracks: {} | Ghosts: {} | Avg Entropy: {:.1}",
                t, total_detections, tracks.len(), total_ghosts, entropy_avg);
        }
    }

    println!("\n✅ Showcase complete!");
    println!("\n📊 Summary:");
    println!("   Total detections processed: {}", total_detections);
    println!("   Final track count: {}", track_manager.track_count());
    println!("   Active ghosts at end: {}", total_ghosts);
    
    println!("\n📖 What to observe:");
    println!("   • White boxes = Ground truth objects");
    println!("   • Colored dots = Raw agent detections (noisy)");
    println!("   • Lines = Detection links from agent to object");
    println!("   • Green ellipsoids = Solid fused tracks");
    println!("   • Red pulsing ellipsoids = Ghost tracks (duplicates!)");
    println!("   • Multiple ellipsoids per object = Ghost problem!");

    Ok(())
}
