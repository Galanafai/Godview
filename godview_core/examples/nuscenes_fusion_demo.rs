//! nuScenes Multi-Agent Fusion Demo
//! ==================================
//! 
//! Demonstrates GodView distributed sensor fusion using real nuScenes tracks.
//! 
//! - Uses ground truth 3D annotations from nuScenes mini dataset
//! - Simulates 4 virtual agents observing the scene:
//!   - Alpha, Beta, Gamma (ground-level sensors)
//!   - StarLink (drone at 30m altitude)
//! - Each agent adds realistic noise based on distance
//! - TrackManager fuses all observations via Covariance Intersection
//! - Rerun visualizes: Ground Truth → Raw Detections → Fused Tracks
//!
//! Run:
//! ```bash
//! # First parse nuScenes data
//! python3 scripts/parse_nuscenes.py
//! 
//! # Then run the demo
//! cargo run --example nuscenes_fusion_demo --features visualization
//! ```

use godview_core::{
    GlobalHazardPacket, TrackManager, TrackingConfig,
    visualization::RerunVisualizer,
    metrics::{calculate_ghost_score, calculate_entropy, calculate_entropy_reduction, calculate_tension, GhostScoreConfig},
};
use nalgebra::{Matrix6, Vector3};
use rand::Rng;
use serde::Deserialize;
use std::collections::HashMap;
use std::fs::File;
use std::io::BufReader;
use uuid::Uuid;

// ============================================================================
// JSON Data Structures (Match parse_nuscenes.py output)
// ============================================================================

#[derive(Debug, Deserialize)]
struct NuScenesData {
    scenes: Vec<Scene>,
}

#[derive(Debug, Deserialize)]
struct Scene {
    name: String,
    description: String,
    frames: Vec<Frame>,
}

#[derive(Debug, Deserialize)]
struct Frame {
    timestamp: f64,
    ego_pose: Option<EgoPose>,
    objects: Vec<Object>,
}

#[derive(Debug, Deserialize)]
struct EgoPose {
    translation: [f64; 3],
    rotation: [f64; 4],
}

#[derive(Debug, Deserialize)]
struct Object {
    instance_id: String,
    category: String,
    position: [f64; 3],
    velocity: [f64; 2], // nuScenes only has 2D velocity
    size: [f64; 3],
}

// ============================================================================
// Virtual Agent Definition
// ============================================================================

struct VirtualAgent {
    name: String,
    id: Uuid,
    base_position: Vector3<f64>, // Relative to scene center
    color: [u8; 4],
    is_drone: bool,
    noise_scale: f64, // Base noise level
    max_range: f64,   // Sensor range in meters
}

// ============================================================================
// MAIN
// ============================================================================

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("🚀 nuScenes Multi-Agent Fusion Demo");
    println!("====================================\n");
    
    // 1. Load parsed nuScenes data
    println!("📂 Loading nuScenes tracks...");
    let file = File::open("../data/nuscenes_tracks.json")?;
    let reader = BufReader::new(file);
    let data: NuScenesData = serde_json::from_reader(reader)?;
    
    println!("   Loaded {} scenes", data.scenes.len());
    
    // Use scene-0061 (has 77 objects, described as: "Parked truck, construction, intersection")
    let scene = &data.scenes[0];
    println!("   Using scene: {} - {}", scene.name, scene.description);
    println!("   Frames: {}", scene.frames.len());
    
    // Calculate scene center from first frame
    let first_frame = &scene.frames[0];
    let scene_center = if !first_frame.objects.is_empty() {
        let sum: Vector3<f64> = first_frame.objects.iter()
            .map(|o| Vector3::new(o.position[0], o.position[1], o.position[2]))
            .fold(Vector3::zeros(), |a, b| a + b);
        sum / first_frame.objects.len() as f64
    } else {
        Vector3::new(380.0, 1140.0, 0.0) // Fallback
    };
    println!("   Scene center: ({:.1}, {:.1}, {:.1})", scene_center.x, scene_center.y, scene_center.z);
    
    // 2. Initialize Rerun Visualizer
    let args: Vec<String> = std::env::args().collect();
    let viz = if args.len() > 1 && args[1] == "--save" {
        let path = args.get(2).map(|s| s.as_str()).unwrap_or("godview_nuscenes_showcase.rrd");
        println!("\n📹 Saving to: {}", path);
        RerunVisualizer::new_to_file("GodView nuScenes Fusion", path)?
    } else {
        println!("\n📹 Opening Rerun viewer...");
        RerunVisualizer::new("GodView nuScenes Fusion")?
    };
    
    // Log ground plane
    viz.log_ground_plane(200.0, 20)?;
    
    // 3. Define Virtual Agents
    let agents = vec![
        VirtualAgent {
            name: "Alpha (Traffic Cam NE)".to_string(),
            id: Uuid::new_v4(),
            base_position: scene_center + Vector3::new(40.0, 30.0, 8.0),
            color: [255, 100, 100, 255], // Red
            is_drone: false,
            noise_scale: 1.5,
            max_range: 80.0,
        },
        VirtualAgent {
            name: "Beta (Connected Car SW)".to_string(),
            id: Uuid::new_v4(),
            base_position: scene_center + Vector3::new(-25.0, -20.0, 1.5),
            color: [100, 150, 255, 255], // Blue
            is_drone: false,
            noise_scale: 1.0,
            max_range: 60.0,
        },
        VirtualAgent {
            name: "Gamma (RSU West)".to_string(),
            id: Uuid::new_v4(),
            base_position: scene_center + Vector3::new(-40.0, 10.0, 6.0),
            color: [100, 255, 150, 255], // Green
            is_drone: false,
            noise_scale: 1.2,
            max_range: 70.0,
        },
        VirtualAgent {
            name: "StarLink (Drone)".to_string(),
            id: Uuid::new_v4(),
            base_position: scene_center + Vector3::new(0.0, 0.0, 30.0),
            color: [255, 215, 0, 255], // Gold
            is_drone: true,
            noise_scale: 0.4, // Excellent view from above
            max_range: 150.0,
        },
    ];
    
    // Log agent positions AND sensor ranges
    for agent in &agents {
        let agent_rel_pos = [
            agent.base_position.x - scene_center.x,
            agent.base_position.y - scene_center.y,
            agent.base_position.z
        ];
        
        viz.log_agent(
            &agent.name,
            agent_rel_pos,
            if agent.is_drone { [2.0, 2.0, 0.5] } else { [2.5, 2.5, 2.0] },
            agent.color,
            agent.is_drone
        )?;
        
        // Draw sensor range circle on ground
        viz.log_sensor_range(
            &agent.name,
            agent_rel_pos,
            agent.max_range as f32,
            agent.color,
        )?;
    }
    
    // 4. Initialize GodView TrackManager
    let mut godview = TrackManager::with_defaults();
    let ghost_config = GhostScoreConfig::default();
    
    // Map instance_id -> stable Uuid (for visualization continuity)
    let mut instance_uuid_map: HashMap<String, Uuid> = HashMap::new();
    
    // 5. Simulation Loop
    println!("\n▶️  Starting simulation...\n");
    
    let mut rng = rand::thread_rng();
    
    for (frame_idx, frame) in scene.frames.iter().enumerate() {
        viz.set_time("frame", frame_idx as u64);
        viz.set_time("timestamp", (frame.timestamp * 1000.0) as u64);
        
        // Move drone in figure-8 pattern
        let t = frame_idx as f64 * 0.5;
        let drone_offset = Vector3::new(
            (t * 0.15).sin() * 30.0,
            (t * 0.3).cos() * 15.0,
            0.0
        );
        let drone_pos = agents[3].base_position + drone_offset;
        
        viz.log_agent(
            "StarLink (Drone)",
            [drone_pos.x - scene_center.x, drone_pos.y - scene_center.y, drone_pos.z],
            [2.0, 2.0, 0.5],
            [255, 215, 0, 255],
            true
        )?;
        
        // A. Log Ground Truth (White ghostly boxes)
        for obj in &frame.objects {
            // Get or create stable UUID for this instance
            let gt_id = instance_uuid_map.entry(obj.instance_id.clone())
                .or_insert_with(Uuid::new_v4);
            
            let pos = Vector3::new(obj.position[0], obj.position[1], obj.position[2]);
            let rel_pos = pos - scene_center;
            
            // Log as white ghostly box (ground truth)
            let gt_cov = Matrix6::identity() * 0.01; // Tiny uncertainty
            viz.log_track_colored(
                *gt_id,
                [rel_pos.x, rel_pos.y, rel_pos.z],
                [obj.velocity[0], obj.velocity[1], 0.0],
                &gt_cov,
                &format!("GT/{}", obj.category.split('.').last().unwrap_or("unknown")),
                [255, 255, 255, 100], // White, semi-transparent
            )?;
        }
        
        // B. Each Agent Detects Objects
        for (agent_idx, agent) in agents.iter().enumerate() {
            let agent_pos = if agent.is_drone { drone_pos } else { agent.base_position };
            
            for obj in &frame.objects {
                let obj_pos = Vector3::new(obj.position[0], obj.position[1], obj.position[2]);
                let dist = (obj_pos - agent_pos).norm();
                
                // Skip if out of range
                if dist > agent.max_range { continue; }
                
                // Add noise (increases with distance)
                let noise_factor = agent.noise_scale * (1.0 + dist / 50.0);
                let noise = Vector3::new(
                    rng.gen_range(-1.0..1.0) * noise_factor,
                    rng.gen_range(-1.0..1.0) * noise_factor,
                    rng.gen_range(-0.3..0.3) * noise_factor,
                );
                
                let measured_pos = obj_pos + noise;
                let rel_meas_pos = measured_pos - scene_center;
                
                // Create local detection ID (simulate each agent having its own ID)
                // Use XOR-based deterministic ID (no need for uuid v5 feature)
                let agent_hash = agent.id.as_u128();
                let obj_hash = obj.instance_id.as_bytes().iter().fold(0u128, |acc, &b| acc.wrapping_mul(31).wrapping_add(b as u128));
                let local_id = Uuid::from_u128(agent_hash ^ obj_hash);
                
                // Create GlobalHazardPacket
                let class_id = match obj.category.as_str() {
                    c if c.contains("car") => 1,
                    c if c.contains("truck") => 1,
                    c if c.contains("bus") => 1,
                    c if c.contains("pedestrian") => 2,
                    c if c.contains("bicycle") || c.contains("motorcycle") => 3,
                    _ => 0,
                };
                
                let packet = GlobalHazardPacket {
                    entity_id: local_id,
                    position: [measured_pos.x, measured_pos.y, measured_pos.z],
                    velocity: [obj.velocity[0], obj.velocity[1], 0.0],
                    class_id,
                    timestamp: frame.timestamp,
                    confidence_score: 1.0 / (1.0 + noise_factor * 0.5),
                };
                
                // Log raw detection (small colored dot)
                viz.log_packet_detection(&packet, noise_factor as f32 * 0.5)?;
                
                // Feed to GodView TrackManager
                match godview.process_packet(&packet) {
                    Ok(_canonical_id) => {},
                    Err(e) => {
                        eprintln!("Warning: Track processing error: {:?}", e);
                    }
                }
            }
        }
        
        // C. Log Fused Tracks with Ghost Hunter Mode (color by ghost score)
        let tracks: Vec<_> = godview.tracks().collect();
        
        // Prepare neighbor data for ghost score calculation
        let track_data: Vec<([f64; 3], [f64; 3], Matrix6<f64>)> = tracks.iter()
            .map(|t| (
                [t.state[0], t.state[1], t.state[2]],
                [t.state[3], t.state[4], t.state[5]],
                t.covariance.clone()
            ))
            .collect();
        
        let total_agents = agents.len();
        let mut ghost_count = 0;
        let mut total_entropy = 0.0;
        
        for (i, track) in tracks.iter().enumerate() {
            let rel_pos = [
                track.state[0] - scene_center.x,
                track.state[1] - scene_center.y,
                track.state[2] - scene_center.z,
            ];
            let track_pos = [track.state[0], track.state[1], track.state[2]];
            let track_vel = [track.state[3], track.state[4], track.state[5]];
            
            // Get neighbors (all other tracks)
            let neighbors: Vec<_> = track_data.iter()
                .enumerate()
                .filter(|(j, _)| *j != i)
                .map(|(_, d)| d.clone())
                .collect();
            
            // Calculate Ghost Score
            let supporting_agents = track.observed_ids.len().min(total_agents);
            let (ghost_score, _nearest_idx, _nearest_dist) = calculate_ghost_score(
                &track_pos,
                &track_vel,
                &track.covariance,
                supporting_agents,
                total_agents,
                &neighbors,
                &ghost_config,
            );
            
            if ghost_score > 0.7 {
                ghost_count += 1;
            }
            
            // Calculate Entropy
            let entropy = calculate_entropy(&track.covariance);
            total_entropy += entropy;
            
            // Log with Ghost Hunter coloring
            viz.log_track_with_ghost_score(
                track.canonical_id,
                rel_pos,
                track_vel,
                &track.covariance,
                ghost_score,
                frame_idx,
            )?;
            
            // Log entropy metrics
            viz.log_entropy(track.canonical_id, entropy, 0.0)?;
        }
        
        // Log stats every 10 frames
        if frame_idx % 10 == 0 {
            let avg_uncertainty = if tracks.is_empty() { 0.0 } else {
                tracks.iter().map(|t| t.covariance.trace()).sum::<f64>() / tracks.len() as f64
            };
            let avg_entropy = if tracks.is_empty() { 0.0 } else { total_entropy / tracks.len() as f64 };
            viz.log_stats(tracks.len(), avg_uncertainty, avg_entropy)?;
            
            println!("  Frame {}/{}: {} objects → {} fused tracks (ghosts: {}, avg uncertainty: {:.2})",
                frame_idx + 1, scene.frames.len(),
                frame.objects.len(), tracks.len(), ghost_count, avg_uncertainty
            );
        }
    }
    
    println!("\n✅ Simulation complete!");
    println!("   Final track count: {}", godview.track_count());
    
    Ok(())
}
