//! Extended Visual Demo - Realistic Intersection Scene
//! =====================================================
//!
//! A 1-minute simulation showing:
//! - Realistic intersection with lanes
//! - 4 fixed sensor agents (traffic cameras)
//! - 1 moving vehicle with sensor (connected car)
//! - 1 patrolling drone
//! - 10+ vehicles flowing through intersection
//! - Pedestrians crossing
//! - Detection lines and ghost visualization
//!
//! Run:
//! ```bash
//! cargo run --example extended_demo --features visualization -- --save extended_demo.rrd
//! ```

use godview_core::visualization::RerunVisualizer;
use godview_core::GlobalHazardPacket;
use std::collections::HashMap;
use uuid::Uuid;

const FRAME_RATE: f64 = 30.0;
const DURATION_SECONDS: f64 = 60.0;
const TOTAL_FRAMES: usize = (FRAME_RATE * DURATION_SECONDS) as usize;

// Agent types for better visual distinction
#[derive(Clone)]
struct Agent {
    name: String,
    position: [f64; 3],
    color: [u8; 4],
    range: f64,
    is_mobile: bool,
    model_type: ModelType,
}

#[derive(Clone, Copy)]
enum ModelType {
    TrafficCamera,  // Tall pole with camera
    ConnectedCar,   // Car shape
    Drone,          // Quadcopter shape
}

// Traffic participant
struct TrafficParticipant {
    id: Uuid,
    name: String,
    start_pos: [f64; 2],
    end_pos: [f64; 2],
    start_frame: usize,
    speed: f64,  // meters per second
    participant_type: ParticipantType,
}

#[derive(Clone, Copy)]
enum ParticipantType {
    Car,
    Truck,
    Pedestrian,
    Cyclist,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("🎬 Extended Visual Demo - Realistic Intersection");
    println!("=================================================\n");
    println!("Duration: {} seconds ({} frames at {}fps)\n", 
        DURATION_SECONDS, TOTAL_FRAMES, FRAME_RATE);

    // Create visualizer
    let args: Vec<String> = std::env::args().collect();
    let viz = if args.len() > 1 && args[1] == "--save" {
        let path = args.get(2).map(|s| s.as_str()).unwrap_or("extended_demo.rrd");
        println!("📹 Saving to: {}\n", path);
        RerunVisualizer::new_to_file("GodView Extended Demo", path)?
    } else {
        println!("📹 Opening Rerun viewer...\n");
        RerunVisualizer::new("GodView Extended Demo")?
    };

    // ========================================================================
    // SCENE SETUP - Intersection
    // ========================================================================
    
    // Ground plane (larger for intersection)
    viz.log_ground_plane(200.0, 20)?;
    
    // Draw roads (North-South and East-West)
    draw_intersection(&viz)?;
    
    // ========================================================================
    // AGENTS - 4 fixed cameras + 1 connected car + 1 drone
    // ========================================================================
    
    let fixed_agents = vec![
        Agent {
            name: "CAM_NORTH".to_string(),
            position: [0.0, 50.0, 12.0],
            color: [255, 100, 100, 255], // Red
            range: 60.0,
            is_mobile: false,
            model_type: ModelType::TrafficCamera,
        },
        Agent {
            name: "CAM_SOUTH".to_string(),
            position: [0.0, -50.0, 12.0],
            color: [100, 100, 255, 255], // Blue
            range: 60.0,
            is_mobile: false,
            model_type: ModelType::TrafficCamera,
        },
        Agent {
            name: "CAM_EAST".to_string(),
            position: [50.0, 0.0, 12.0],
            color: [100, 255, 100, 255], // Green
            range: 60.0,
            is_mobile: false,
            model_type: ModelType::TrafficCamera,
        },
        Agent {
            name: "CAM_WEST".to_string(),
            position: [-50.0, 0.0, 12.0],
            color: [255, 200, 100, 255], // Orange
            range: 60.0,
            is_mobile: false,
            model_type: ModelType::TrafficCamera,
        },
    ];
    
    // Log fixed agents with sensor ranges (static)
    for agent in &fixed_agents {
        log_agent_model(&viz, agent)?;
        viz.log_sensor_range(&agent.name, agent.position, agent.range as f32, agent.color)?;
    }
    
    // ========================================================================
    // TRAFFIC PARTICIPANTS - Vehicles and pedestrians with timed spawns
    // ========================================================================
    
    let participants = generate_traffic_schedule();
    println!("📍 Scene: {} traffic participants scheduled\n", participants.len());
    
    // ========================================================================
    // SIMULATION LOOP - 60 seconds at 30fps
    // ========================================================================
    
    println!("▶️  Running simulation...\n");
    
    // Track which agents see which participants (for ghost tracking)
    let mut detection_history: HashMap<Uuid, Vec<String>> = HashMap::new();
    
    for frame in 0..TOTAL_FRAMES {
        let t = frame as f64 / FRAME_RATE; // Time in seconds
        viz.set_time("frame", frame as u64);
        viz.set_time("timestamp", (t * 1_000_000_000.0) as u64);
        
        // === DRONE - Figure-8 patrol pattern ===
        let drone_x = (t * 0.2).sin() * 60.0;
        let drone_y = (t * 0.4).sin() * 40.0;
        let drone_pos = [drone_x, drone_y, 35.0];
        
        log_drone(&viz, "DRONE_PATROL", drone_pos, [255, 215, 0, 255])?;
        
        // === CONNECTED CAR - Drives through intersection ===
        let car_cycle = t % 30.0; // 30 second cycle
        let connected_car_pos = if car_cycle < 15.0 {
            // North to South
            [5.0, 70.0 - car_cycle * 10.0, 1.0]
        } else {
            // East to West
            [70.0 - (car_cycle - 15.0) * 10.0, -5.0, 1.0]
        };
        
        log_connected_car(&viz, "CONNECTED_CAR", connected_car_pos, [0, 200, 255, 255])?;
        
        // === TRAFFIC PARTICIPANTS - Update active ones ===
        for p in &participants {
            if frame >= p.start_frame {
                let elapsed = (frame - p.start_frame) as f64 / FRAME_RATE;
                let total_distance = ((p.end_pos[0] - p.start_pos[0]).powi(2) 
                    + (p.end_pos[1] - p.start_pos[1]).powi(2)).sqrt();
                let travel_time = total_distance / p.speed;
                
                if elapsed <= travel_time {
                    // Calculate current position (linear interpolation)
                    let progress = elapsed / travel_time;
                    let pos = [
                        p.start_pos[0] + (p.end_pos[0] - p.start_pos[0]) * progress,
                        p.start_pos[1] + (p.end_pos[1] - p.start_pos[1]) * progress,
                        match p.participant_type {
                            ParticipantType::Car | ParticipantType::Truck => 1.0,
                            ParticipantType::Pedestrian | ParticipantType::Cyclist => 0.9,
                        },
                    ];
                    
                    // Log participant
                    log_participant(&viz, &p.name, pos, p.participant_type)?;
                    
                    // Check detection by each agent
                    let mut detected_by = Vec::new();
                    
                    // Check fixed cameras
                    for agent in &fixed_agents {
                        if in_range(pos, agent.position, agent.range) {
                            detected_by.push(agent.name.clone());
                            viz.log_detection_line(
                                &format!("{}_{}", agent.name, p.name),
                                agent.position,
                                pos,
                                [agent.color[0], agent.color[1], agent.color[2], 40],
                            )?;
                        }
                    }
                    
                    // Check drone
                    if in_range(pos, drone_pos, 80.0) {
                        detected_by.push("DRONE".to_string());
                        viz.log_detection_line(
                            &format!("DRONE_{}", p.name),
                            drone_pos,
                            pos,
                            [255, 215, 0, 40],
                        )?;
                    }
                    
                    // Check connected car
                    if in_range(pos, connected_car_pos, 40.0) {
                        detected_by.push("CONNECTED_CAR".to_string());
                        viz.log_detection_line(
                            &format!("CCAR_{}", p.name),
                            connected_car_pos,
                            pos,
                            [0, 200, 255, 40],
                        )?;
                    }
                    
                    // Track detections for ghost counting
                    detection_history.insert(p.id, detected_by);
                }
            }
        }
        
        // Progress indicator every 5 seconds
        if frame % (FRAME_RATE as usize * 5) == 0 {
            let active = participants.iter()
                .filter(|p| {
                    if frame < p.start_frame { return false; }
                    let elapsed = (frame - p.start_frame) as f64 / FRAME_RATE;
                    let total_distance = ((p.end_pos[0] - p.start_pos[0]).powi(2) 
                        + (p.end_pos[1] - p.start_pos[1]).powi(2)).sqrt();
                    elapsed <= total_distance / p.speed
                })
                .count();
            
            let ghosts: usize = detection_history.values()
                .filter(|agents| agents.len() >= 2)
                .map(|agents| agents.len() - 1) // Each extra detection is a "ghost"
                .sum();
            
            println!("  t={:.0}s  Active: {}  Potential ghosts: {}", t, active, ghosts);
        }
    }
    
    println!("\n✅ Simulation complete!");
    println!("   Total frames: {}", TOTAL_FRAMES);
    println!("   Duration: {} seconds", DURATION_SECONDS);
    
    Ok(())
}

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================

fn draw_intersection(viz: &RerunVisualizer) -> Result<(), Box<dyn std::error::Error>> {
    // North-South road
    viz.log_road([-10.0, -100.0], [-10.0, 100.0], 20.0)?;
    viz.log_road([10.0, -100.0], [10.0, 100.0], 20.0)?;
    
    // East-West road
    viz.log_road([-100.0, -10.0], [100.0, -10.0], 20.0)?;
    viz.log_road([-100.0, 10.0], [100.0, 10.0], 20.0)?;
    
    Ok(())
}

fn log_agent_model(viz: &RerunVisualizer, agent: &Agent) -> Result<(), Box<dyn std::error::Error>> {
    match agent.model_type {
        ModelType::TrafficCamera => {
            // Pole
            viz.log_agent(
                &format!("{}_pole", agent.name),
                [agent.position[0], agent.position[1], agent.position[2] / 2.0],
                [0.5, 0.5, agent.position[2] as f32],
                [100, 100, 100, 255], // Gray
                false,
            )?;
            // Camera head
            viz.log_agent(
                &agent.name,
                agent.position,
                [2.0, 2.0, 1.5],
                agent.color,
                false,
            )?;
        }
        _ => {
            viz.log_agent(
                &agent.name,
                agent.position,
                [4.0, 2.0, 1.5],
                agent.color,
                false,
            )?;
        }
    }
    Ok(())
}

fn log_drone(viz: &RerunVisualizer, name: &str, pos: [f64; 3], color: [u8; 4]) -> Result<(), Box<dyn std::error::Error>> {
    // Central body
    viz.log_agent(name, pos, [3.0, 3.0, 1.0], color, true)?;
    Ok(())
}

fn log_connected_car(viz: &RerunVisualizer, name: &str, pos: [f64; 3], color: [u8; 4]) -> Result<(), Box<dyn std::error::Error>> {
    viz.log_agent(name, pos, [5.0, 2.5, 1.8], color, false)?;
    // Sensor dome on top
    viz.log_agent(&format!("{}_sensor", name), [pos[0], pos[1], pos[2] + 1.5], [1.0, 1.0, 0.5], [255, 255, 255, 200], false)?;
    Ok(())
}

fn log_participant(viz: &RerunVisualizer, name: &str, pos: [f64; 3], p_type: ParticipantType) -> Result<(), Box<dyn std::error::Error>> {
    let (size, color): ([f32; 3], [u8; 4]) = match p_type {
        ParticipantType::Car => ([4.5, 2.0, 1.5], [200, 200, 200, 220]),
        ParticipantType::Truck => ([8.0, 2.5, 3.0], [180, 180, 180, 220]),
        ParticipantType::Pedestrian => ([0.5, 0.5, 1.8], [255, 200, 150, 220]),
        ParticipantType::Cyclist => ([1.8, 0.6, 1.5], [100, 200, 100, 220]),
    };
    
    viz.log_agent(name, pos, size, color, false)?;
    Ok(())
}

fn in_range(pos: [f64; 3], agent_pos: [f64; 3], range: f64) -> bool {
    let dist = ((pos[0] - agent_pos[0]).powi(2) + (pos[1] - agent_pos[1]).powi(2)).sqrt();
    dist <= range
}

fn generate_traffic_schedule() -> Vec<TrafficParticipant> {
    let mut participants = Vec::new();
    let mut id = 0;
    
    // Northbound traffic (every 3 seconds)
    for i in 0..20 {
        participants.push(TrafficParticipant {
            id: Uuid::new_v4(),
            name: format!("CAR_NB_{}", id),
            start_pos: [5.0, -80.0],
            end_pos: [5.0, 80.0],
            start_frame: (i as f64 * 3.0 * FRAME_RATE) as usize,
            speed: 12.0, // 12 m/s = ~43 km/h
            participant_type: if i % 5 == 0 { ParticipantType::Truck } else { ParticipantType::Car },
        });
        id += 1;
    }
    
    // Southbound traffic (every 4 seconds, offset)
    for i in 0..15 {
        participants.push(TrafficParticipant {
            id: Uuid::new_v4(),
            name: format!("CAR_SB_{}", id),
            start_pos: [-5.0, 80.0],
            end_pos: [-5.0, -80.0],
            start_frame: ((i as f64 * 4.0 + 1.5) * FRAME_RATE) as usize,
            speed: 10.0,
            participant_type: ParticipantType::Car,
        });
        id += 1;
    }
    
    // Eastbound traffic (every 5 seconds)
    for i in 0..12 {
        participants.push(TrafficParticipant {
            id: Uuid::new_v4(),
            name: format!("CAR_EB_{}", id),
            start_pos: [-80.0, -5.0],
            end_pos: [80.0, -5.0],
            start_frame: ((i as f64 * 5.0 + 2.0) * FRAME_RATE) as usize,
            speed: 11.0,
            participant_type: ParticipantType::Car,
        });
        id += 1;
    }
    
    // Westbound traffic
    for i in 0..12 {
        participants.push(TrafficParticipant {
            id: Uuid::new_v4(),
            name: format!("CAR_WB_{}", id),
            start_pos: [80.0, 5.0],
            end_pos: [-80.0, 5.0],
            start_frame: ((i as f64 * 5.0 + 3.5) * FRAME_RATE) as usize,
            speed: 10.5,
            participant_type: ParticipantType::Car,
        });
        id += 1;
    }
    
    // Pedestrians crossing (every 10 seconds)
    for i in 0..6 {
        // North-South crossing
        participants.push(TrafficParticipant {
            id: Uuid::new_v4(),
            name: format!("PED_{}", id),
            start_pos: [15.0, -15.0],
            end_pos: [15.0, 15.0],
            start_frame: ((i as f64 * 10.0 + 5.0) * FRAME_RATE) as usize,
            speed: 1.4, // Walking speed
            participant_type: ParticipantType::Pedestrian,
        });
        id += 1;
    }
    
    // Cyclists
    for i in 0..4 {
        participants.push(TrafficParticipant {
            id: Uuid::new_v4(),
            name: format!("BIKE_{}", id),
            start_pos: [-80.0, -8.0],
            end_pos: [80.0, -8.0],
            start_frame: ((i as f64 * 15.0 + 7.0) * FRAME_RATE) as usize,
            speed: 6.0, // Cycling speed
            participant_type: ParticipantType::Cyclist,
        });
        id += 1;
    }
    
    participants
}
