//! Autonomous Vehicle Demo - Self-Driving Cars with 3D Models
//! ============================================================
//!
//! Demonstrates:
//! - 3 Self-driving cars (connected vehicles with sensors)
//! - 4 Fixed infrastructure cameras
//! - 1 Patrol drone
//! - 3D model loading (if available)
//! - Each AV shares detections via V2X
//! - Ghost detection across all agents
//!
//! Run:
//! ```bash
//! cargo run --example autonomous_demo --features visualization -- --save autonomous_demo.rrd
//! ```

use godview_core::{
    GlobalHazardPacket, TrackManager,
    visualization::RerunVisualizer,
    metrics::{calculate_ghost_score, calculate_entropy, GhostScoreConfig},
};
use nalgebra::Vector3;
use rand::Rng;
use uuid::Uuid;
use std::path::Path;

const FRAME_RATE: f64 = 30.0;
const DURATION_SECONDS: f64 = 45.0; // 45 second demo
const TOTAL_FRAMES: usize = (FRAME_RATE * DURATION_SECONDS) as usize;

// Autonomous Vehicle
struct AutonomousVehicle {
    id: Uuid,
    name: String,
    color: [u8; 4],
    sensor_range: f64,
    noise_scale: f64,
    // Path parameters (circular path)
    center: Vector3<f64>,
    radius: f64,
    speed: f64,      // Radians per second
    start_angle: f64,
}

impl AutonomousVehicle {
    fn position_at(&self, t: f64) -> Vector3<f64> {
        let angle = self.start_angle + self.speed * t;
        Vector3::new(
            self.center.x + self.radius * angle.cos(),
            self.center.y + self.radius * angle.sin(),
            1.0, // Ground level
        )
    }
    
    fn heading_at(&self, t: f64) -> f64 {
        let angle = self.start_angle + self.speed * t;
        angle + std::f64::consts::FRAC_PI_2 // Perpendicular to radius
    }
}

// Static infrastructure camera
struct InfraCamera {
    id: Uuid,
    name: String,
    position: Vector3<f64>,
    color: [u8; 4],
    range: f64,
    noise_scale: f64,
}

// Target object
struct TargetObject {
    id: Uuid,
    name: String,
    class: &'static str,
    size: [f32; 3],
    initial_pos: Vector3<f64>,
    velocity: Vector3<f64>,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("🚗 Autonomous Vehicle Demo - Self-Driving Cars");
    println!("================================================\n");

    // Create visualizer
    let args: Vec<String> = std::env::args().collect();
    let viz = if args.len() > 1 && args[1] == "--save" {
        let path = args.get(2).map(|s| s.as_str()).unwrap_or("autonomous_demo.rrd");
        println!("📹 Saving to: {}\n", path);
        RerunVisualizer::new_to_file("Autonomous Vehicles", path)?
    } else {
        println!("📹 Opening Rerun viewer...\n");
        RerunVisualizer::new("Autonomous Vehicles")?
    };

    // Ground and roads
    viz.log_ground_plane(200.0, 20)?;
    viz.log_road([-100.0, 0.0], [100.0, 0.0], 15.0)?;
    viz.log_road([0.0, -100.0], [0.0, 100.0], 15.0)?;

    // =========================================================================
    // AUTONOMOUS VEHICLES - 3 self-driving cars
    // =========================================================================
    
    let avs = vec![
        AutonomousVehicle {
            id: Uuid::new_v4(),
            name: "AV_ALPHA".to_string(),
            color: [0, 200, 255, 255], // Cyan (Tesla blue)
            sensor_range: 50.0,
            noise_scale: 0.5, // High accuracy
            center: Vector3::new(0.0, 0.0, 0.0),
            radius: 30.0,
            speed: 0.35, // Faster for visible movement
            start_angle: 0.0,
        },
        AutonomousVehicle {
            id: Uuid::new_v4(),
            name: "AV_BETA".to_string(),
            color: [255, 100, 200, 255], // Pink (Waymo style)
            sensor_range: 45.0,
            noise_scale: 0.6,
            center: Vector3::new(0.0, 0.0, 0.0),
            radius: 30.0,
            speed: 0.35,
            start_angle: std::f64::consts::TAU / 3.0, // 120 degrees offset
        },
        AutonomousVehicle {
            id: Uuid::new_v4(),
            name: "AV_GAMMA".to_string(),
            color: [100, 255, 100, 255], // Green (Cruise style)
            sensor_range: 55.0,
            noise_scale: 0.4,
            center: Vector3::new(0.0, 0.0, 0.0),
            radius: 30.0,
            speed: 0.35,
            start_angle: 2.0 * std::f64::consts::TAU / 3.0, // 240 degrees offset
        },
    ];

    // =========================================================================
    // INFRASTRUCTURE CAMERAS - 4 fixed cameras
    // =========================================================================
    
    let infra_cameras = vec![
        InfraCamera {
            id: Uuid::new_v4(),
            name: "INFRA_NE".to_string(),
            position: Vector3::new(50.0, 50.0, 12.0),
            color: [255, 150, 100, 255], // Orange
            range: 60.0,
            noise_scale: 1.5,
        },
        InfraCamera {
            id: Uuid::new_v4(),
            name: "INFRA_NW".to_string(),
            position: Vector3::new(-50.0, 50.0, 12.0),
            color: [100, 150, 255, 255], // Blue
            range: 60.0,
            noise_scale: 1.8,
        },
        InfraCamera {
            id: Uuid::new_v4(),
            name: "INFRA_SE".to_string(),
            position: Vector3::new(50.0, -50.0, 12.0),
            color: [255, 255, 100, 255], // Yellow
            range: 60.0,
            noise_scale: 1.6,
        },
        InfraCamera {
            id: Uuid::new_v4(),
            name: "INFRA_SW".to_string(),
            position: Vector3::new(-50.0, -50.0, 12.0),
            color: [200, 100, 255, 255], // Purple
            range: 60.0,
            noise_scale: 2.0,
        },
    ];

    // Log infrastructure cameras (static)
    for cam in &infra_cameras {
        viz.log_agent(
            &cam.name,
            [cam.position.x, cam.position.y, cam.position.z],
            [1.5, 1.5, 2.5],
            cam.color,
            false,
        )?;
        viz.log_sensor_range(
            &cam.name,
            [cam.position.x, cam.position.y, 0.0],
            cam.range as f32,
            cam.color,
        )?;
    }

    // =========================================================================
    // TARGET OBJECTS - Pedestrians and other vehicles
    // =========================================================================
    
    let targets = vec![
        TargetObject {
            id: Uuid::new_v4(),
            name: "PED_1".to_string(),
            class: "pedestrian",
            size: [0.6, 0.6, 1.8],
            initial_pos: Vector3::new(10.0, -10.0, 0.9),
            velocity: Vector3::new(0.0, 1.0, 0.0),
        },
        TargetObject {
            id: Uuid::new_v4(),
            name: "PED_2".to_string(),
            class: "pedestrian",
            size: [0.6, 0.6, 1.8],
            initial_pos: Vector3::new(-15.0, 5.0, 0.9),
            velocity: Vector3::new(1.2, 0.0, 0.0),
        },
        TargetObject {
            id: Uuid::new_v4(),
            name: "CYCLIST".to_string(),
            class: "cyclist",
            size: [1.8, 0.6, 1.5],
            initial_pos: Vector3::new(0.0, 40.0, 1.0),
            velocity: Vector3::new(4.0, -3.0, 0.0),
        },
        TargetObject {
            id: Uuid::new_v4(),
            name: "PARKED_CAR".to_string(),
            class: "car",
            size: [4.5, 2.0, 1.6],
            initial_pos: Vector3::new(-20.0, -20.0, 1.0),
            velocity: Vector3::new(0.0, 0.0, 0.0), // Stationary
        },
    ];

    println!("📍 Scene Setup:");
    println!("   3 Self-driving AVs: ALPHA, BETA, GAMMA");
    println!("   4 Infrastructure cameras: NE, NW, SE, SW");
    println!("   4 Target objects: 2 pedestrians, 1 cyclist, 1 parked car");
    println!("   1 Patrol drone\n");

    // 3D model disabled for now (causes rendering issues)
    let has_3d_model = false;
    if has_3d_model {
        println!("✅ 3D model found: toycar.glb\n");
    } else {
        println!("ℹ️  No 3D model found, using boxes\n");
    }

    // TrackManager and config
    let mut track_manager = TrackManager::with_defaults();
    let ghost_config = GhostScoreConfig::default();
    let mut rng = rand::thread_rng();
    let mut merge_count = 0usize;
    
    // Log section titles
    viz.log_section_titles()?;

    println!("▶️  Running simulation...\n");

    // Simulation loop
    for frame in 0..TOTAL_FRAMES {
        let t = frame as f64 / FRAME_RATE;
        viz.set_time("frame", frame as u64);

        // =====================================================================
        // RENDER AUTONOMOUS VEHICLES
        // =====================================================================
        for av in &avs {
            let pos = av.position_at(t);
            let heading = av.heading_at(t);
            
            // Render AV as 3D car mesh
            viz.log_car_mesh(
                &format!("world/avs/{}", av.name),
                [pos.x, pos.y, pos.z],
                1.0, // Scale
                heading as f32,
                av.color,
            )?;
            
            // Render sensor dome on top
            viz.log_agent(
                &format!("{}_sensor", av.name),
                [pos.x, pos.y, pos.z + 1.5],
                [1.5, 1.5, 0.8],
                [255, 255, 255, 180],
                false,
            )?;
            
            // Render sensor range (update every second)
            if frame % 30 == 0 {
                viz.log_sensor_range(
                    &format!("{}_range", av.name),
                    [pos.x, pos.y, 0.0],
                    av.sensor_range as f32,
                    [av.color[0], av.color[1], av.color[2], 50],
                )?;
            }
        }

        // =====================================================================
        // DRONE - Patrol pattern
        // =====================================================================
        let drone_pos = [
            (t * 0.2).sin() * 60.0,
            (t * 0.4).sin() * 40.0,
            35.0,
        ];
        let drone_yaw = (t * 0.5) as f32; // Slow rotation
        viz.log_drone_mesh("PATROL_DRONE", drone_pos, 2.0, drone_yaw, [255, 215, 0, 255])?;

        // =====================================================================
        // RENDER TARGETS (Ground Truth)
        // =====================================================================
        for target in &targets {
            let pos = target.initial_pos + target.velocity * t;
            viz.log_class_bbox(
                target.class,
                &target.name,
                [pos.x, pos.y, pos.z],
                target.size,
                0.0,
                1.0,
            )?;
        }

        // =====================================================================
        // AV DETECTIONS - Each AV senses targets
        // =====================================================================
        for av in &avs {
            let av_pos = av.position_at(t);
            
            // AV also sees other AVs!
            for other_av in &avs {
                if other_av.id == av.id { continue; }
                let other_pos = other_av.position_at(t);
                let dist = (other_pos - av_pos).norm();
                
                if dist > av.sensor_range { continue; }
                
                // Detect with noise
                let noise = Vector3::new(
                    rng.gen_range(-1.0..1.0) * av.noise_scale,
                    rng.gen_range(-1.0..1.0) * av.noise_scale,
                    rng.gen_range(-0.1..0.1),
                );
                let detected_pos = other_pos + noise;
                
                let local_id = Uuid::from_u128(av.id.as_u128() ^ other_av.id.as_u128());
                let packet = GlobalHazardPacket {
                    entity_id: local_id,
                    position: [detected_pos.x, detected_pos.y, detected_pos.z],
                    velocity: [0.0, 0.0, 0.0],
                    class_id: 1,
                    timestamp: t,
                    confidence_score: 0.95,
                };
                
                viz.log_detection_line(
                    &format!("{}_sees_{}", av.name, other_av.name),
                    [av_pos.x, av_pos.y, av_pos.z + 1.5],
                    [detected_pos.x, detected_pos.y, detected_pos.z],
                    [av.color[0], av.color[1], av.color[2], 60],
                )?;
                
                let _ = track_manager.process_packet(&packet);
            }
            
            // Each target
            for target in &targets {
                let target_pos = target.initial_pos + target.velocity * t;
                let dist = (target_pos - av_pos).norm();
                
                if dist > av.sensor_range { continue; }
                
                let noise = Vector3::new(
                    rng.gen_range(-1.0..1.0) * av.noise_scale,
                    rng.gen_range(-1.0..1.0) * av.noise_scale,
                    rng.gen_range(-0.1..0.1),
                );
                let detected_pos = target_pos + noise;
                
                let local_id = Uuid::from_u128(av.id.as_u128() ^ target.id.as_u128());
                let packet = GlobalHazardPacket {
                    entity_id: local_id,
                    position: [detected_pos.x, detected_pos.y, detected_pos.z],
                    velocity: [target.velocity.x, target.velocity.y, target.velocity.z],
                    class_id: match target.class { "pedestrian" => 2, "cyclist" => 3, _ => 1 },
                    timestamp: t,
                    confidence_score: 1.0 - dist / av.sensor_range * 0.2,
                };
                
                viz.log_packet_detection(&packet, av.noise_scale as f32 * 0.3)?;
                viz.log_detection_line(
                    &format!("{}_det_{}", av.name, target.name),
                    [av_pos.x, av_pos.y, av_pos.z + 1.5],
                    [detected_pos.x, detected_pos.y, detected_pos.z],
                    [av.color[0], av.color[1], av.color[2], 40],
                )?;
                
                let _ = track_manager.process_packet(&packet);
            }
        }

        // =====================================================================
        // INFRA CAMERA DETECTIONS
        // =====================================================================
        for cam in &infra_cameras {
            // Detect targets
            for target in &targets {
                let target_pos = target.initial_pos + target.velocity * t;
                let dist = (target_pos - cam.position).norm();
                
                if dist > cam.range { continue; }
                
                let noise = Vector3::new(
                    rng.gen_range(-1.0..1.0) * cam.noise_scale,
                    rng.gen_range(-1.0..1.0) * cam.noise_scale,
                    rng.gen_range(-0.2..0.2),
                );
                let detected_pos = target_pos + noise;
                
                let local_id = Uuid::from_u128(cam.id.as_u128() ^ target.id.as_u128());
                let packet = GlobalHazardPacket {
                    entity_id: local_id,
                    position: [detected_pos.x, detected_pos.y, detected_pos.z],
                    velocity: [target.velocity.x, target.velocity.y, target.velocity.z],
                    class_id: match target.class { "pedestrian" => 2, "cyclist" => 3, _ => 1 },
                    timestamp: t,
                    confidence_score: 0.8,
                };
                
                viz.log_packet_detection(&packet, cam.noise_scale as f32 * 0.3)?;
                let _ = track_manager.process_packet(&packet);
            }
            
            // Detect AVs
            for av in &avs {
                let av_pos = av.position_at(t);
                let dist = (av_pos - cam.position).norm();
                
                if dist > cam.range { continue; }
                
                let noise = Vector3::new(
                    rng.gen_range(-1.0..1.0) * cam.noise_scale,
                    rng.gen_range(-1.0..1.0) * cam.noise_scale,
                    rng.gen_range(-0.2..0.2),
                );
                let detected_pos = av_pos + noise;
                
                let local_id = Uuid::from_u128(cam.id.as_u128() ^ av.id.as_u128());
                let packet = GlobalHazardPacket {
                    entity_id: local_id,
                    position: [detected_pos.x, detected_pos.y, detected_pos.z],
                    velocity: [0.0, 0.0, 0.0],
                    class_id: 1,
                    timestamp: t,
                    confidence_score: 0.85,
                };
                
                viz.log_packet_detection(&packet, cam.noise_scale as f32 * 0.3)?;
                let _ = track_manager.process_packet(&packet);
            }
        }

        // =====================================================================
        // FUSED TRACKS WITH GHOST SCORES
        // =====================================================================
        let tracks: Vec<_> = track_manager.tracks().collect();
        let track_data: Vec<_> = tracks.iter()
            .map(|t| (
                [t.state[0], t.state[1], t.state[2]],
                [t.state[3], t.state[4], t.state[5]],
                t.covariance.clone()
            ))
            .collect();
        
        let mut ghost_count = 0;
        let total_agents = avs.len() + infra_cameras.len() + 1; // +1 for drone potential
        
        for (i, track) in tracks.iter().enumerate() {
            let track_pos = [track.state[0], track.state[1], track.state[2]];
            let track_vel = [track.state[3], track.state[4], track.state[5]];
            
            let neighbors: Vec<_> = track_data.iter()
                .enumerate()
                .filter(|(j, _)| *j != i)
                .map(|(_, d)| d.clone())
                .collect();
            
            let (ghost_score, _, _) = calculate_ghost_score(
                &track_pos, &track_vel, &track.covariance,
                track.observed_ids.len(), total_agents,
                &neighbors, &ghost_config,
            );
            
            if ghost_score > 0.7 { ghost_count += 1; }
            
            viz.log_track_with_ghost_score(
                track.canonical_id, track_pos, track_vel,
                &track.covariance, ghost_score, frame,
            )?;
        }

        // Progress
        if frame % (FRAME_RATE as usize * 5) == 0 {
            let entropy_avg: f64 = if tracks.is_empty() { 0.0 } else {
                tracks.iter().map(|t| calculate_entropy(&t.covariance)).sum::<f64>() / tracks.len() as f64
            };
            
            // Log stats panel
            viz.log_stats_panel(7, tracks.len(), ghost_count, merge_count)?;
            
            println!("  t={:.0}s | Tracks: {} | Ghosts: {} | Entropy: {:.1}",
                t, tracks.len(), ghost_count, entropy_avg);
        }
    }

    println!("\n✅ Autonomous Demo complete!");
    println!("   Final tracks: {}", track_manager.track_count());
    
    println!("\n📖 What to observe:");
    println!("   • 3 Colored AVs driving in circle (Cyan, Pink, Green)");
    println!("   • White sensor domes on top of AVs");
    println!("   • Detection lines from AVs to objects they see");
    println!("   • Ghost ellipsoids (red) where AVs + cameras see same object");
    println!("   • Patrol drone flying overhead");

    Ok(())
}
