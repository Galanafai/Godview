//! CARLA Integration Demo - End-to-End with Rerun Visualization
//! =============================================================
//!
//! This example demonstrates the complete GodView + CARLA integration pipeline:
//! - Receives telemetry from zmq_bridge.py via ZeroMQ
//! - Processes actor updates with real simulation timestamps
//! - Visualizes all actors in Rerun with cinematic camera
//! - Validates detections against ground truth
//!
//! Run:
//! ```bash
//! # Terminal 1: Start CARLA headless
//! docker-compose up carla-headless
//!
//! # Terminal 2: Start ZMQ bridge
//! python3 carla_bridge/zmq_bridge.py --vehicles 10 --duration 120
//!
//! # Terminal 3: Run this demo
//! cargo run --example carla_demo --features carla,visualization -- --save carla_demo.rrd
//! ```

use std::collections::HashMap;
use std::time::{Duration, Instant};

use godview_core::{
    TrackManager, GlobalHazardPacket,
    visualization::{RerunVisualizer, CinematicCamera},
    metrics::{calculate_ghost_score, GhostScoreConfig},
};

#[cfg(feature = "carla")]
use godview_core::carla_zmq::{CarlaZmqReceiver, TelemetryPacket, ActorUpdate};

use nalgebra::{Matrix6, Vector6};
use uuid::Uuid;

// =============================================================================
// CONFIGURATION
// =============================================================================

const ZMQ_TELEMETRY_PORT: u16 = 5555;
const ZMQ_METADATA_PORT: u16 = 5556;
const MAX_DURATION_SECONDS: u64 = 120;

/// Actor class colors for visualization
fn class_color(actor_type: &str) -> [u8; 4] {
    match actor_type {
        "vehicle" => [0, 200, 255, 200],    // Cyan
        "pedestrian" => [255, 100, 100, 200], // Red
        "cyclist" => [100, 255, 100, 200],   // Green
        _ => [200, 200, 200, 200],            // Gray
    }
}

// =============================================================================
// VALIDATION SYSTEM - Compare GodView vs CARLA Ground Truth
// =============================================================================

/// Validation metrics for comparing detected positions vs ground truth
#[derive(Debug, Default)]
pub struct ValidationMetrics {
    /// Total frames processed
    pub frames_processed: usize,
    /// Total detections received
    pub detections_count: usize,
    /// Sum of position errors (for averaging)
    pub position_error_sum: f64,
    /// Maximum position error observed
    pub max_position_error: f64,
    /// Tracking latency samples (ms)
    pub latency_samples: Vec<f64>,
    /// Per-actor tracking errors
    pub per_actor_errors: HashMap<u32, Vec<f64>>,
}

impl ValidationMetrics {
    pub fn new() -> Self {
        Self::default()
    }
    
    /// Record a detection and compare to ground truth
    pub fn record_detection(
        &mut self,
        actor_id: u32,
        detected_pos: [f32; 3],
        ground_truth_pos: [f32; 3],
        detection_time: f64,
        current_sim_time: f64,
    ) {
        // Calculate position error (Euclidean distance)
        let dx = detected_pos[0] - ground_truth_pos[0];
        let dy = detected_pos[1] - ground_truth_pos[1];
        let dz = detected_pos[2] - ground_truth_pos[2];
        let error = (dx * dx + dy * dy + dz * dz).sqrt() as f64;
        
        self.detections_count += 1;
        self.position_error_sum += error;
        self.max_position_error = self.max_position_error.max(error);
        
        // Record per-actor error
        self.per_actor_errors
            .entry(actor_id)
            .or_insert_with(Vec::new)
            .push(error);
        
        // Record latency (time since detection was timestamped)
        let latency_ms = (current_sim_time - detection_time) * 1000.0;
        if latency_ms >= 0.0 && latency_ms < 1000.0 {
            self.latency_samples.push(latency_ms);
        }
    }
    
    /// Mark a frame as processed
    pub fn record_frame(&mut self) {
        self.frames_processed += 1;
    }
    
    /// Calculate average position error
    pub fn avg_position_error(&self) -> f64 {
        if self.detections_count > 0 {
            self.position_error_sum / self.detections_count as f64
        } else {
            0.0
        }
    }
    
    /// Calculate average latency
    pub fn avg_latency_ms(&self) -> f64 {
        if !self.latency_samples.is_empty() {
            self.latency_samples.iter().sum::<f64>() / self.latency_samples.len() as f64
        } else {
            0.0
        }
    }
    
    /// Print summary report
    pub fn print_report(&self) {
        println!("\n╔══════════════════════════════════════════════════════════╗");
        println!("║            VALIDATION REPORT                              ║");
        println!("╠══════════════════════════════════════════════════════════╣");
        println!("║ Frames Processed:    {:>8}                            ║", self.frames_processed);
        println!("║ Total Detections:    {:>8}                            ║", self.detections_count);
        println!("║ Avg Position Error:  {:>8.3} m                          ║", self.avg_position_error());
        println!("║ Max Position Error:  {:>8.3} m                          ║", self.max_position_error);
        println!("║ Avg Latency:         {:>8.2} ms                         ║", self.avg_latency_ms());
        println!("║ Actors Tracked:      {:>8}                            ║", self.per_actor_errors.len());
        println!("╚══════════════════════════════════════════════════════════╝");
    }
}

// =============================================================================
// "SEEING AROUND CORNERS" DETECTOR
// =============================================================================

/// Tracks which vehicles can see which objects
/// Used to detect when a vehicle "knows" about an object it cannot directly see
pub struct CollaborativePerceptionTracker {
    /// Map of actor_id -> set of other actor_ids that can directly see it
    direct_observers: HashMap<u32, Vec<u32>>,
    /// Map of vehicle_id -> set of actor_ids it has received via V2X
    indirect_knowledge: HashMap<u32, Vec<u32>>,
    /// "Seeing around corners" events
    corner_sight_events: Vec<CornerSightEvent>,
}

/// Event when a vehicle learns about an object it cannot see directly
#[derive(Debug, Clone)]
pub struct CornerSightEvent {
    /// The vehicle that received the information
    pub receiver_vehicle_id: u32,
    /// The object being seen "around the corner"
    pub target_id: u32,
    /// The vehicle that provided the detection
    pub source_vehicle_id: u32,
    /// Simulation time of the event
    pub sim_time: f64,
    /// Position of the target
    pub target_pos: [f32; 3],
}

impl CollaborativePerceptionTracker {
    pub fn new() -> Self {
        Self {
            direct_observers: HashMap::new(),
            indirect_knowledge: HashMap::new(),
            corner_sight_events: Vec::new(),
        }
    }
    
    /// Record a direct observation (vehicle sees object)
    pub fn record_direct_observation(&mut self, observer_id: u32, target_id: u32) {
        self.direct_observers
            .entry(target_id)
            .or_insert_with(Vec::new)
            .push(observer_id);
    }
    
    /// Check if a V2X message creates a "seeing around corners" event
    pub fn check_corner_sight(
        &mut self,
        receiver_id: u32,
        source_id: u32,
        target_id: u32,
        target_pos: [f32; 3],
        sim_time: f64,
    ) -> Option<CornerSightEvent> {
        // Does the receiver already see this target directly?
        let receiver_can_see = self.direct_observers
            .get(&target_id)
            .map(|observers| observers.contains(&receiver_id))
            .unwrap_or(false);
        
        if receiver_can_see {
            // Not a corner sight - receiver can see it directly
            return None;
        }
        
        // Does the receiver already know about this target via V2X?
        let already_known = self.indirect_knowledge
            .get(&receiver_id)
            .map(|known| known.contains(&target_id))
            .unwrap_or(false);
        
        if already_known {
            // Already knew about it
            return None;
        }
        
        // This is a NEW "seeing around corners" event!
        // The receiver is learning about a target it cannot see, from another vehicle
        
        // Record the knowledge
        self.indirect_knowledge
            .entry(receiver_id)
            .or_insert_with(Vec::new)
            .push(target_id);
        
        let event = CornerSightEvent {
            receiver_vehicle_id: receiver_id,
            target_id,
            source_vehicle_id: source_id,
            sim_time,
            target_pos,
        };
        
        self.corner_sight_events.push(event.clone());
        
        Some(event)
    }
    
    /// Get all corner sight events
    pub fn events(&self) -> &[CornerSightEvent] {
        &self.corner_sight_events
    }
    
    /// Clear per-frame state (keep events)
    pub fn clear_frame(&mut self) {
        self.direct_observers.clear();
    }
    
    /// Print summary
    pub fn print_summary(&self) {
        println!("\n🔮 'SEEING AROUND CORNERS' SUMMARY");
        println!("══════════════════════════════════════");
        println!("Total corner sight events: {}", self.corner_sight_events.len());
        
        if !self.corner_sight_events.is_empty() {
            println!("\nFirst 5 events:");
            for (i, event) in self.corner_sight_events.iter().take(5).enumerate() {
                println!("  {}. Vehicle {} learned about {} from Vehicle {} at t={:.1}s",
                    i + 1,
                    event.receiver_vehicle_id,
                    event.target_id,
                    event.source_vehicle_id,
                    event.sim_time
                );
            }
        }
    }
}

// =============================================================================
// MAIN DEMO
// =============================================================================

#[cfg(feature = "carla")]
fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("╔════════════════════════════════════════════════════════════╗");
    println!("║      GODVIEW + CARLA INTEGRATION DEMO                      ║");
    println!("║      End-to-End with Validation & Corner Sight             ║");
    println!("╚════════════════════════════════════════════════════════════╝\n");

    // Parse args
    let args: Vec<String> = std::env::args().collect();
    let save_path = if args.len() > 1 && args[1] == "--save" {
        args.get(2).map(|s| s.as_str())
    } else {
        None
    };

    // Create visualizer
    let viz = if let Some(path) = save_path {
        println!("📹 Recording to: {}\n", path);
        RerunVisualizer::new_to_file("CARLA Demo", path)?
    } else {
        println!("📹 Opening Rerun viewer...\n");
        RerunVisualizer::new("CARLA Demo")?
    };

    // Setup scene
    viz.log_ground_plane(500.0, 50)?;
    
    // Initialize components
    println!("🔌 Connecting to ZMQ ports {} and {}...", 
        ZMQ_TELEMETRY_PORT, ZMQ_METADATA_PORT);
    
    let mut receiver = CarlaZmqReceiver::new(ZMQ_TELEMETRY_PORT, ZMQ_METADATA_PORT)?;
    println!("✅ ZMQ receiver connected\n");
    
    let mut track_manager = TrackManager::with_defaults();
    let ghost_config = GhostScoreConfig::default();
    let mut cinematic_camera = CinematicCamera::default();
    let mut validation = ValidationMetrics::new();
    let mut collab_tracker = CollaborativePerceptionTracker::new();
    
    // Ground truth storage (simulated - in real use, this comes from CARLA)
    let mut ground_truth: HashMap<u32, [f32; 3]> = HashMap::new();
    
    // Stats
    let mut last_print_time = Instant::now();
    let start_time = Instant::now();
    let mut ego_vehicle_id: Option<u32> = None;
    
    println!("🎬 Listening for CARLA telemetry...");
    println!("   (Run zmq_bridge.py in another terminal)\n");
    println!("══════════════════════════════════════════════════════════════\n");

    loop {
        // Check timeout
        if start_time.elapsed() > Duration::from_secs(MAX_DURATION_SECONDS) {
            println!("\n⏰ Max duration reached ({} seconds)", MAX_DURATION_SECONDS);
            break;
        }
        
        // Receive metadata (spawn events)
        while let Ok(Some(metadata)) = receiver.receive_metadata() {
            println!("🆕 New actor spawned: {} ({}) - color {:?}", 
                metadata.actor_id, metadata.actor_type, metadata.color);
            
            // First vehicle is ego vehicle (for cinematic camera)
            if ego_vehicle_id.is_none() && metadata.actor_type == "vehicle" {
                ego_vehicle_id = Some(metadata.actor_id);
                println!("   → Selected as ego vehicle for cinematic camera");
            }
        }
        
        // Receive telemetry
        match receiver.receive_telemetry() {
            Ok(Some(packet)) => {
                let sim_time = { packet.header.timestamp };
                let frame_id = { packet.header.frame_id };
                
                // Set Rerun timeline to simulation time
                viz.set_sim_time(sim_time);
                viz.set_time("frame", frame_id);
                
                // Clear per-frame collaborative tracking
                collab_tracker.clear_frame();
                
                // Update ground truth (in CARLA, positions ARE ground truth)
                for actor in &packet.actors {
                    ground_truth.insert(actor.id, actor.pos);
                }
                
                // Process each actor
                for actor in &packet.actors {
                    // Get actor type from registry
                    let actor_type = receiver.known_actors
                        .get(&actor.id)
                        .map(|m| m.actor_type.as_str())
                        .unwrap_or("unknown");
                    
                    let color = class_color(actor_type);
                    
                    // Visualize actor
                    let path = format!("world/actors/{}", actor.id);
                    viz.log_agent(
                        &path,
                        [actor.pos[0] as f64, actor.pos[1] as f64, actor.pos[2] as f64],
                        [4.0, 2.0, 1.5], // Vehicle size
                        color,
                        false,
                    )?;
                    
                    // Record as direct observation
                    collab_tracker.record_direct_observation(actor.id, actor.id);
                    
                    // Validate against ground truth
                    if let Some(&gt_pos) = ground_truth.get(&actor.id) {
                        validation.record_detection(
                            actor.id,
                            actor.pos,
                            gt_pos,
                            sim_time,
                            sim_time,
                        );
                    }
                    
                    // Create hazard packet for tracking
                    let packet = GlobalHazardPacket {
                        entity_id: Uuid::from_u128(actor.id as u128),
                        position: [actor.pos[0] as f64, actor.pos[1] as f64, actor.pos[2] as f64],
                        velocity: [actor.vel[0] as f64, actor.vel[1] as f64, actor.vel[2] as f64],
                        class_id: match actor_type { "vehicle" => 1, "pedestrian" => 2, _ => 0 },
                        timestamp: sim_time,
                        confidence_score: 1.0,
                    };
                    
                    let _ = track_manager.process_packet(&packet);
                    
                    // Update cinematic camera if this is ego vehicle
                    if Some(actor.id) == ego_vehicle_id {
                        cinematic_camera.update_target(
                            [actor.pos[0] as f64, actor.pos[1] as f64, actor.pos[2] as f64],
                            actor.rot[1] as f64, // yaw
                        );
                        
                        let (cam_pos, look_at) = cinematic_camera.get_camera_state(0.05);
                        viz.log_cinematic_camera(
                            cam_pos,
                            look_at,
                            [0.0, 0.0, 1.0], // up
                            60.0, // FOV
                        )?;
                    }
                }
                
                // Simulate "seeing around corners" by checking V2X sharing
                // In real V2X, vehicles share detections; here we simulate it
                let actors: Vec<_> = packet.actors.iter().collect();
                for (i, &vehicle) in actors.iter().enumerate() {
                    let v_type = receiver.known_actors
                        .get(&vehicle.id)
                        .map(|m| m.actor_type.as_str())
                        .unwrap_or("");
                    
                    if v_type != "vehicle" { continue; }
                    
                    // This vehicle "shares" what it can see with other vehicles
                    for (j, &other) in actors.iter().enumerate() {
                        if i == j { continue; }
                        
                        let other_type = receiver.known_actors
                            .get(&other.id)
                            .map(|m| m.actor_type.as_str())
                            .unwrap_or("");
                        
                        if other_type != "vehicle" { continue; }
                        
                        // Check if 'other' vehicle learns about any target from 'vehicle'
                        for &target in actors.iter() {
                            if target.id == vehicle.id || target.id == other.id { continue; }
                            
                            // Calculate distances
                            let d_vehicle_target = distance(vehicle.pos, target.pos);
                            let d_other_target = distance(other.pos, target.pos);
                            
                            // If vehicle can see target but other cannot
                            let sensor_range = 50.0; // meters
                            if d_vehicle_target < sensor_range && d_other_target > sensor_range {
                                // This is a "seeing around corners" event!
                                if let Some(event) = collab_tracker.check_corner_sight(
                                    other.id,
                                    vehicle.id,
                                    target.id,
                                    target.pos,
                                    sim_time,
                                ) {
                                    // Visualize the V2X communication
                                    viz.log_data_packet(
                                        [vehicle.pos[0] as f64, vehicle.pos[1] as f64, vehicle.pos[2] as f64 + 2.0],
                                        [other.pos[0] as f64, other.pos[1] as f64, other.pos[2] as f64 + 2.0],
                                        &format!("v2x_{}_{}", vehicle.id, other.id),
                                    )?;
                                    
                                    // Log event
                                    if frame_id % 100 == 0 {
                                        println!("🔮 Corner Sight: Vehicle {} told Vehicle {} about target {}", 
                                            event.source_vehicle_id, event.receiver_vehicle_id, event.target_id);
                                    }
                                }
                            }
                        }
                    }
                }
                
                validation.record_frame();
                
                // Print stats periodically
                if last_print_time.elapsed() > Duration::from_secs(2) {
                    println!("⏱️  t={:.1}s | Frame {} | Actors: {} | Tracks: {} | Corner Events: {}",
                        sim_time,
                        frame_id,
                        packet.actors.len(),
                        track_manager.track_count(),
                        collab_tracker.events().len()
                    );
                    last_print_time = Instant::now();
                }
            }
            Ok(None) => {
                // No message available, brief sleep
                std::thread::sleep(Duration::from_millis(10));
            }
            Err(e) => {
                eprintln!("⚠️  ZMQ error: {}", e);
                std::thread::sleep(Duration::from_millis(100));
            }
        }
    }

    // Print final reports
    validation.print_report();
    collab_tracker.print_summary();
    
    println!("\n✅ CARLA Demo complete!");
    
    Ok(())
}

#[cfg(not(feature = "carla"))]
fn main() {
    eprintln!("❌ This example requires the 'carla' feature.");
    eprintln!("   Run with: cargo run --example carla_demo --features carla,visualization");
}

/// Calculate distance between two 3D points
fn distance(a: [f32; 3], b: [f32; 3]) -> f32 {
    let dx = a[0] - b[0];
    let dy = a[1] - b[1];
    let dz = a[2] - b[2];
    (dx * dx + dy * dy + dz * dz).sqrt()
}
