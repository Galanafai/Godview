//! GodView Agent v3 - Global Coordinate System with AS-EKF Sensor Fusion
//!
//! This agent integrates the GodView Core v3 library to provide:
//! - Global GPS coordinates (not camera-relative)
//! - AS-EKF sensor fusion for delayed measurements
//! - H3+Octree spatial indexing
//! - Ed25519 cryptographic signatures

use anyhow::Result;
use opencv::{
    core::{Mat, Vector},
    objdetect::CascadeClassifier,
    prelude::*,
    videoio::{self, VideoCapture, CAP_ANY},
    imgproc,
};
use serde::{Deserialize, Serialize};
use std::time::{SystemTime, UNIX_EPOCH};
use tokio::time::{sleep, Duration};
use zenoh::prelude::*;

// GodView Core v3 imports
use godview_core::{Entity, AugmentedStateFilter, SpatialEngine, SignedPacket};
use ed25519_dalek::SigningKey;
use h3o::Resolution;
use nalgebra::{DVector, DMatrix};
use uuid::Uuid;
use rand::rngs::OsRng;

/// Global Hazard Packet (v3 format)
#[derive(Serialize, Deserialize, Debug)]
struct GlobalHazardPacket {
    /// The entity with global GPS coordinates
    entity: Entity,
    /// Camera-relative position (for debugging)
    camera_pos: [f32; 3],
    /// Agent identifier
    agent_id: String,
}

// Constants for 3D projection math
const FOCAL_LENGTH_CONST: f32 = 500.0; // Approximate focal length in pixels
const REAL_FACE_WIDTH_M: f32 = 0.15; // Average human face width in meters (~15cm)

// Earth radius for coordinate conversion
const METERS_PER_DEGREE_LAT: f64 = 111320.0;

#[tokio::main]
async fn main() -> Result<()> {
    println!("╔════════════════════════════════════════════╗");
    println!("║   GODVIEW AGENT V3 (GLOBAL MODE)          ║");
    println!("╚════════════════════════════════════════════╝");
    println!();

    // ========== CONFIGURATION ==========
    
    // Read Virtual GPS from environment (for indoor testing)
    let agent_lat: f64 = std::env::var("AGENT_GPS_LAT")
        .unwrap_or("37.7749".to_string())
        .parse()
        .expect("Invalid AGENT_GPS_LAT");
    
    let agent_lon: f64 = std::env::var("AGENT_GPS_LON")
        .unwrap_or("-122.4194".to_string())
        .parse()
        .expect("Invalid AGENT_GPS_LON");
    
    let agent_alt: f32 = std::env::var("AGENT_GPS_ALT")
        .unwrap_or("10.0".to_string())
        .parse()
        .expect("Invalid AGENT_GPS_ALT");
    
    let agent_heading: f32 = std::env::var("AGENT_HEADING")
        .unwrap_or("0.0".to_string())
        .parse()
        .expect("Invalid AGENT_HEADING");
    
    let agent_id = std::env::var("AGENT_ID")
        .unwrap_or("agent_warehouse_1".to_string());

    println!("📍 Agent Configuration:");
    println!("   GPS: ({:.6}, {:.6}, {:.1}m)", agent_lat, agent_lon, agent_alt);
    println!("   Heading: {:.1}° (0°=North)", agent_heading);
    println!("   ID: {}", agent_id);
    println!();

    // ========== INITIALIZE V3 ENGINES ==========
    
    println!("🔧 Initializing GodView Core v3 engines...");
    
    // 1. Initialize AS-EKF (9D state: position, velocity, acceleration)
    let initial_state = DVector::from_vec(vec![
        agent_lat, agent_lon, agent_alt as f64,  // Position
        0.0, 0.0, 0.0,  // Velocity
        0.0, 0.0, 0.0,  // Acceleration
    ]);
    let initial_cov = DMatrix::identity(9, 9) * 10.0;
    let Q = DMatrix::identity(9, 9) * 0.01;  // Process noise
    let R = DMatrix::identity(3, 3) * 0.1;   // Measurement noise
    
    let mut ekf = AugmentedStateFilter::new(initial_state, initial_cov, Q, R, 20);
    println!("   ✅ AS-EKF initialized (lag depth: 20 states)");
    
    // 2. Initialize Spatial Engine (H3 Resolution 10 = ~66m cells)
    let mut spatial_engine = SpatialEngine::new(Resolution::Ten);
    println!("   ✅ Spatial Engine initialized (H3 Resolution 10)");
    
    // 3. Initialize Security (Ed25519 signing key)
    let signing_key = SigningKey::generate(&mut OsRng);
    println!("   ✅ Security initialized (Ed25519)");
    println!();

    // ========== INITIALIZE ZENOH ==========
    
    let config = zenoh::Config::default();
    let session = zenoh::open(config).await?;
    println!("🌐 Zenoh session established");

    let key = "godview/global/hazards";  // NEW: Global topic
    println!("📡 Publishing to: {}", key);
    println!();

    // ========== INITIALIZE OPENCV ==========
    
    let mut cam = VideoCapture::new(0, CAP_ANY)?;
    if !cam.is_opened()? {
        anyhow::bail!("Failed to open webcam (device 0)");
    }
    println!("📷 Webcam opened successfully");

    let cascade_path = "haarcascade_frontalface_alt.xml";
    let mut face_cascade = CascadeClassifier::new(cascade_path)?;
    if face_cascade.empty() {
        anyhow::bail!("Failed to load Haar Cascade from {}", cascade_path);
    }
    println!("🔍 Haar Cascade loaded: {}", cascade_path);
    println!();

    // ========== MAIN DETECTION LOOP ==========
    
    let mut frame = Mat::default();
    let mut gray = Mat::default();
    let mut frame_counter = 0u64;
    let start_time = SystemTime::now();

    println!("🚀 Starting detection loop (30 Hz)...");
    println!("════════════════════════════════════════════");
    println!();
    
    loop {
        frame_counter += 1;
        let current_time = SystemTime::now()
            .duration_since(UNIX_EPOCH)?
            .as_secs_f64();
        
        // Capture frame
        cam.read(&mut frame)?;
        if frame.empty() {
            sleep(Duration::from_millis(33)).await;
            continue;
        }

        // Convert to grayscale
        imgproc::cvt_color(&frame, &mut gray, imgproc::COLOR_BGR2GRAY, 0)?;

        // Detect faces
        let mut faces = Vector::<opencv::core::Rect>::new();
        face_cascade.detect_multi_scale(
            &gray,
            &mut faces,
            1.1,  // scale factor
            3,    // min neighbors
            0,    // flags
            opencv::core::Size::new(30, 30),
            opencv::core::Size::new(0, 0),
        )?;

        // Process each detected face
        for face in faces.iter() {
            let frame_width = frame.cols() as f32;
            let center_x = frame_width / 2.0;
            
            let face_x = face.x as f32 + (face.width as f32 / 2.0);
            let face_width_px = face.width as f32;

            // ========== 3D PROJECTION (Camera-Relative) ==========
            let z = (FOCAL_LENGTH_CONST * REAL_FACE_WIDTH_M) / face_width_px;
            let x = (face_x - center_x) * z / FOCAL_LENGTH_CONST;
            let y = 0.0;
            let camera_pos = [x, y, z];

            // ========== TRANSFORM TO GLOBAL GPS ==========
            let global_pos = camera_to_global(
                camera_pos,
                [agent_lat, agent_lon, agent_alt],
                agent_heading
            );

            // ========== CREATE ENTITY ==========
            let timestamp = SystemTime::now()
                .duration_since(UNIX_EPOCH)?
                .as_millis() as i64;
            
            let entity = Entity {
                id: Uuid::new_v4(),
                position: global_pos,
                velocity: [0.0, 0.0, 0.0],  // TODO: Derive from EKF
                entity_type: "human_face".to_string(),
                timestamp,
                confidence: 0.95,
            };

            // ========== UPDATE SPATIAL ENGINE ==========
            spatial_engine.update_entity(entity.clone())?;

            // ========== UPDATE AS-EKF ==========
            let measurement = DVector::from_vec(vec![global_pos[0], global_pos[1], global_pos[2]]);
            ekf.update_oosm(measurement, current_time);

            // ========== CREATE SIGNED PACKET ==========
            let packet = GlobalHazardPacket {
                entity: entity.clone(),
                camera_pos,
                agent_id: agent_id.clone(),
            };

            let payload = serde_json::to_vec(&packet)?;
            let signed_packet = SignedPacket::new(payload, &signing_key, None);
            
            // ========== PUBLISH VIA ZENOH ==========
            let signed_payload = serde_json::to_vec(&signed_packet)?;
            session.put(key, signed_payload).await?;

            println!(
                "📤 [Frame {}] Hazard detected:",
                frame_counter
            );
            println!("   Camera: [{:.2}, {:.2}, {:.2}]m", camera_pos[0], camera_pos[1], camera_pos[2]);
            println!("   Global: [{:.6}, {:.6}, {:.2}]", global_pos[0], global_pos[1], global_pos[2]);
            println!("   Entity ID: {}", entity.id);
            println!();
        }

        // Predict EKF forward
        ekf.predict(0.033, current_time);

        // 30 Hz = ~33ms per frame
        sleep(Duration::from_millis(33)).await;
    }
}

/// Transform camera-relative coordinates to global GPS
///
/// # Arguments
/// * `camera_pos` - Position in camera frame [x, y, z] in meters
/// * `agent_gps` - Agent's GPS position [lat, lon, alt]
/// * `heading` - Agent's compass heading in degrees (0° = North)
///
/// # Returns
/// Global GPS coordinates [lat, lon, alt]
fn camera_to_global(
    camera_pos: [f32; 3],
    agent_gps: [f64; 3],
    heading: f32,
) -> [f64; 3] {
    // Convert heading to radians
    let heading_rad = heading.to_radians();
    let cos_h = heading_rad.cos() as f64;
    let sin_h = heading_rad.sin() as f64;
    
    // Rotate camera vector by heading (around Y-axis)
    // Camera Z-axis points forward, X-axis points right
    let x_world = camera_pos[0] as f64 * cos_h - camera_pos[2] as f64 * sin_h;
    let z_world = camera_pos[0] as f64 * sin_h + camera_pos[2] as f64 * cos_h;
    
    // Convert meters to GPS offset
    // Latitude: 1 degree ≈ 111.32 km
    // Longitude: 1 degree ≈ 111.32 km * cos(latitude)
    let meters_per_degree_lon = METERS_PER_DEGREE_LAT * agent_gps[0].to_radians().cos();
    
    // Apply offset (Z-world is North/South, X-world is East/West)
    let lat = agent_gps[0] + (z_world / METERS_PER_DEGREE_LAT);
    let lon = agent_gps[1] + (x_world / meters_per_degree_lon);
    let alt = agent_gps[2] + camera_pos[1] as f64;
    
    [lat, lon, alt]
}
