//! CARLA Mode for GodView Agent
//!
//! This module handles input from the CARLA simulator via stdin instead of webcam.
//! The Python bridge sends JSON-formatted detection data which we process and
//! feed into the GodView Core v3 engines.

use anyhow::Result;
use serde::Deserialize;
use std::io::{self, BufRead};
use std::time::{SystemTime, UNIX_EPOCH};
// use tokio::time::{sleep, Duration};  // Not needed for CARLA mode
// use zenoh::prelude::*;  // Removed in Zenoh 1.0

// GodView Core v3 imports
use godview_core::{
    Entity, AugmentedStateFilter, SpatialEngine, SignedPacket,
    TrackManager, TrackingConfig, GlobalHazardPacket as CoreHazardPacket,
};
use ed25519_dalek::SigningKey;
use h3o::Resolution;
use nalgebra::{DVector, DMatrix};
use uuid::Uuid;
use rand::rngs::OsRng;

use crate::GlobalHazardPacket;

/// Detection data from CARLA Python bridge
#[derive(Debug, Deserialize)]
pub struct CARLADetection {
    /// Bounding box [x1, y1, x2, y2] in pixels
    pub bbox: [f32; 4],
    /// Detection confidence (0.0-1.0)
    pub confidence: f32,
    /// Object class name (e.g., "car", "person")
    pub class_name: String,
    /// GPS latitude
    pub gps_lat: f64,
    /// GPS longitude
    pub gps_lon: f64,
    /// GPS altitude (meters)
    pub gps_alt: f32,
    /// Vehicle heading (degrees, 0=North)
    pub heading: f32,
    /// Timestamp from CARLA
    pub timestamp: f64,
}

/// Run GodView agent in CARLA mode (reads from stdin)
pub async fn run_carla_mode() -> Result<()> {
    println!("╔════════════════════════════════════════════╗");
    println!("║   GODVIEW AGENT V3 (CARLA MODE)           ║");
    println!("╚════════════════════════════════════════════╝");
    println!();

    // Get agent ID from environment
    let agent_id = std::env::var("AGENT_ID")
        .unwrap_or_else(|_| "carla_agent_unknown".to_string());
    
    println!("📍 Agent Configuration:");
    println!("   ID: {}", agent_id);
    println!("   Mode: CARLA (stdin input)");
    println!();

    // ========== INITIALIZE V3 ENGINES ==========
    
    println!("🔧 Initializing GodView Core v3 engines...");
    
    // Initialize AS-EKF with 6D state [x, y, z, vx, vy, vz] to match godview_core
    let initial_state = DVector::from_vec(vec![
        0.0, 0.0, 0.0,  // Position (will be updated with first GPS)
        0.0, 0.0, 0.0,  // Velocity
    ]);
    let initial_cov = DMatrix::identity(6, 6) * 10.0;
    let Q = DMatrix::identity(6, 6) * 0.01;  // Process noise
    let R = DMatrix::identity(3, 3) * 0.1;   // Measurement noise (position only)
    
    let mut ekf = AugmentedStateFilter::new(initial_state, initial_cov, Q, R, 20);
    println!("   ✅ AS-EKF initialized (lag depth: 20 states)");
    
    // Initialize Spatial Engine
    let mut spatial_engine = SpatialEngine::new(Resolution::Ten);
    println!("   ✅ Spatial Engine initialized (H3 Resolution 10)");
    
    // Initialize Track Manager for distributed data association
    let mut track_manager = TrackManager::new(TrackingConfig::default());
    println!("   ✅ Track Manager initialized (GNN + CI + Highlander)");
    
    // Initialize Security
    let signing_key = SigningKey::generate(&mut OsRng);
    println!("   ✅ Security initialized (Ed25519)");
    println!();

    // ========== INITIALIZE ZENOH ==========
    
    let config = zenoh::Config::default();
    let session = zenoh::open(config).await
        .map_err(|e| anyhow::anyhow!("Zenoh error: {}", e))?;
    println!("🌐 Zenoh session established");

    let key = "godview/carla/hazards";  // CARLA-specific topic
    println!("📡 Publishing to: {}", key);
    println!();

    // ========== READ FROM STDIN ==========
    
    println!("🎬 Waiting for detections from CARLA bridge...");
    println!("════════════════════════════════════════════");
    println!();
    
    let stdin = io::stdin();
    let mut detection_count = 0u64;
    let mut last_gps: Option<(f64, f64, f32)> = None;
    
    for line in stdin.lock().lines() {
        let line = match line {
            Ok(l) => l,
            Err(e) => {
                eprintln!("❌ Error reading stdin: {}", e);
                break;
            }
        };
        
        // Skip empty lines
        if line.trim().is_empty() {
            continue;
        }
        
        // Parse JSON detection
        let detection: CARLADetection = match serde_json::from_str(&line) {
            Ok(d) => d,
            Err(e) => {
                eprintln!("⚠️  Failed to parse detection: {}", e);
                continue;
            }
        };
        
        detection_count += 1;
        
        // Update last known GPS
        last_gps = Some((detection.gps_lat, detection.gps_lon, detection.gps_alt));
        
        // Calculate 3D position from bounding box
        // For CARLA, we use the detection directly at vehicle's GPS
        // In a real system, you'd project the bbox to 3D space
        let global_pos = [
            detection.gps_lat,
            detection.gps_lon,
            detection.gps_alt as f64
        ];
        
        // Get current time
        let current_time = SystemTime::now()
            .duration_since(UNIX_EPOCH)?
            .as_secs_f64();
        
        // Create Entity
        let timestamp = SystemTime::now()
            .duration_since(UNIX_EPOCH)?
            .as_millis() as i64;
        
        let entity = Entity {
            id: Uuid::new_v4(),
            position: global_pos,
            velocity: [0.0, 0.0, 0.0],  // TODO: Calculate from tracking
            entity_type: detection.class_name.clone(),
            timestamp,
            confidence: detection.confidence,  // Entity uses f32
        };

        // ========== UPDATE SPATIAL ENGINE ==========
        if let Err(e) = spatial_engine.update_entity(entity.clone()) {
            eprintln!("⚠️  Spatial engine error: {}", e);
        }

        // ========== UPDATE AS-EKF ==========
        let measurement = DVector::from_vec(vec![
            global_pos[0],
            global_pos[1],
            global_pos[2]
        ]);
        ekf.update_oosm(measurement, current_time);
        
        // ========== PROCESS THROUGH TRACK MANAGER ==========
        // Convert detection class to class_id (0=person, 1=car, 2=truck, etc.)
        let class_id = match detection.class_name.as_str() {
            "person" => 0u8,
            "car" => 1,
            "truck" | "bus" => 2,
            "bicycle" | "motorcycle" => 3,
            _ => 255,  // Unknown
        };
        
        let core_packet = CoreHazardPacket {
            entity_id: entity.id,
            position: global_pos,
            velocity: [0.0, 0.0, 0.0],  // TODO: velocity estimation
            class_id,
            timestamp: current_time,
            confidence_score: detection.confidence as f64,
        };
        
        // Run through GNN + Covariance Intersection + Highlander pipeline
        let canonical_id = match track_manager.process_packet(&core_packet) {
            Ok(id) => id,
            Err(e) => {
                eprintln!("⚠️  Track manager error: {}", e);
                entity.id  // Fallback to original ID
            }
        };

        // ========== CREATE SIGNED PACKET ==========
        let packet = GlobalHazardPacket {
            entity: entity.clone(),
            camera_pos: [0.0, 0.0, 0.0],  // Not applicable in CARLA mode
            agent_id: agent_id.clone(),
        };

        let payload = serde_json::to_vec(&packet)?;
        let signed_packet = SignedPacket::new(payload, &signing_key, None);
        
        // ========== PUBLISH VIA ZENOH ==========
        let signed_payload = serde_json::to_vec(&signed_packet)?;
        session.put(key, signed_payload).await
            .map_err(|e| anyhow::anyhow!("Zenoh put error: {}", e))?;

        // Print status every 10 detections
        if detection_count % 10 == 0 {
            println!(
                "📤 [Detection {}] {} detected:",
                detection_count,
                detection.class_name
            );
            println!("   GPS: [{:.6}, {:.6}, {:.2}]", 
                     global_pos[0], global_pos[1], global_pos[2]);
            println!("   Confidence: {:.2}", detection.confidence);
            println!("   Entity ID: {}", entity.id);
            println!();
        }

        // Predict EKF forward
        ekf.predict(0.05, current_time);  // 20 FPS = 0.05s
    }
    
    println!("\n✅ CARLA mode ended");
    println!("Total detections processed: {}", detection_count);
    
    Ok(())
}
