//! GodView Agent v3 - Global Coordinate System with AS-EKF Sensor Fusion
//!
//! This agent integrates the GodView Core v3 library to provide:
//! - Global GPS coordinates (not camera-relative)
//! - AS-EKF sensor fusion for delayed measurements
//! - H3+Octree spatial indexing
//! - Ed25519 cryptographic signatures

use anyhow::Result;
use serde::{Deserialize, Serialize};
use std::time::{SystemTime, UNIX_EPOCH};
use tokio::time::{sleep, Duration};
// use zenoh::prelude::*;  // Removed in Zenoh 1.0

// GodView Core v3 imports
use godview_core::{Entity, AugmentedStateFilter, SpatialEngine, SignedPacket};
use ed25519_dalek::SigningKey;
use h3o::Resolution;
use nalgebra::{DVector, DMatrix};
use uuid::Uuid;
use rand::rngs::OsRng;

// Webcam mode (optional - requires OpenCV)
#[cfg(feature = "webcam")]
mod webcam_mode;

// CARLA mode (always available)
mod carla_mode;

/// Global Hazard Packet (v3 format)
#[derive(Serialize, Deserialize, Debug)]
pub struct GlobalHazardPacket {
    /// The entity with global GPS coordinates
    entity: Entity,
    /// Camera-relative position (for debugging)
    camera_pos: [f32; 3],
    /// Agent identifier
    agent_id: String,
}

#[tokio::main]
async fn main() -> Result<()> {
    // Check if running in CARLA mode
    let carla_mode_enabled = std::env::var("CARLA_MODE").is_ok();
    
    if carla_mode_enabled {
        // Run CARLA mode (reads from stdin)
        return carla_mode::run_carla_mode().await;
    }
    
    // Otherwise, run webcam mode (requires OpenCV feature)
    #[cfg(feature = "webcam")]
    {
        return webcam_mode::run_webcam_mode().await;
    }
    
    #[cfg(not(feature = "webcam"))]
    {
        eprintln!("❌ Webcam mode requires the 'webcam' feature.");
        eprintln!("   Build with: cargo build --features webcam");
        eprintln!("   Or set CARLA_MODE=true for CARLA simulation mode.");
        std::process::exit(1);
    }
}
