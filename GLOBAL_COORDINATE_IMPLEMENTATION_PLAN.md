# GodView Global Architecture - Implementation Plan

**Mission:** WORLD_BUILDER_PROTOCOL_V1  
**Objective:** Transform GodView from local demo to planetary-scale hive mind  
**Status:** 🔴 BREAKING CHANGES - Major architectural refactor

---

## Executive Summary

This plan upgrades GodView to use **global GPS coordinates** instead of camera-relative positions, enabling true "seeing around corners" for distributed agents anywhere on Earth.

### The Core Problem We're Solving

**Current (Broken):**
```
Agent A: "Hazard at x=10m, z=5m" (relative to Agent A's camera)
Agent B: Receives this but can't use it (doesn't know where Agent A is)
```

**New (Global):**
```
Agent A at GPS(37.7749, -122.4194): "Hazard at GPS(37.7751, -122.4192)"
Agent B at GPS(37.7750, -122.4195): Receives global coordinates
Agent B: "Hazard is 15m Northeast of me" (can navigate to it)
```

---

## User Review Required

> [!WARNING]
> **Breaking Changes**: This refactor will break compatibility with the current MVP. The old camera-relative system will be completely replaced.

> [!IMPORTANT]
> **New Dependencies**: Requires `geo` and `nalgebra` crates for coordinate transforms. These add ~2MB to binary size.

> [!CAUTION]
> **Virtual GPS for Testing**: We'll use simulated GPS for indoor testing. Real GPS requires hardware modules and doesn't work indoors.

> [!IMPORTANT]
> **Coordinate System Choice**: Using ECEF (Earth-Centered, Earth-Fixed) for math, WGS84 (Lat/Lon) for transmission. This is the same system used by GPS satellites and autonomous vehicles.

---

## Proposed Changes

### Component 1: Rust Agent - Global Coordinate System

#### File: `agent/Cargo.toml`

**Add Dependencies:**
```toml
[dependencies]
zenoh = { version = "1.0.0", features = ["unstable"] }
opencv = "0.92"
serde = { version = "1.0", features = ["derive"] }
serde_json = "1.0"
tokio = { version = "1", features = ["full"] }
anyhow = "1.0"
geo = "0.27"           # NEW: Geographic primitives
nalgebra = "0.32"      # NEW: Linear algebra for rotation matrices
```

---

#### File: `agent/src/main.rs`

**Add New Structs:**

```rust
use geo::{Point, Coord};
use nalgebra::{Matrix3, Vector3};

#[derive(Debug, Clone)]
struct VirtualGPS {
    latitude: f64,   // WGS84 degrees
    longitude: f64,  // WGS84 degrees
    altitude: f32,   // Meters above sea level
    heading: f32,    // Degrees (0° = North, 90° = East, 180° = South, 270° = West)
}

#[derive(Serialize, Deserialize, Debug)]
struct GlobalHazardPacket {
    id: String,
    timestamp: i64,
    global_pos: [f64; 3],  // [lat, lon, alt] in WGS84
    agent_id: String,
    agent_pos: [f64; 3],   // Agent's GPS position
    agent_heading: f32,    // Agent's compass heading
    #[serde(rename = "type")]
    hazard_type: String,
}
```

**Add Coordinate Transform Functions:**

```rust
// WGS84 ellipsoid constants
const WGS84_A: f64 = 6378137.0;              // Semi-major axis (meters)
const WGS84_F: f64 = 1.0 / 298.257223563;    // Flattening
const WGS84_E2: f64 = 2.0 * WGS84_F - WGS84_F * WGS84_F;  // Eccentricity squared

/// Convert WGS84 (lat/lon/alt) to ECEF (x/y/z)
fn wgs84_to_ecef(lat: f64, lon: f64, alt: f32) -> Vector3<f64> {
    let lat_rad = lat.to_radians();
    let lon_rad = lon.to_radians();
    
    let sin_lat = lat_rad.sin();
    let cos_lat = lat_rad.cos();
    let sin_lon = lon_rad.sin();
    let cos_lon = lon_rad.cos();
    
    // Radius of curvature in prime vertical
    let n = WGS84_A / (1.0 - WGS84_E2 * sin_lat * sin_lat).sqrt();
    
    let x = (n + alt as f64) * cos_lat * cos_lon;
    let y = (n + alt as f64) * cos_lat * sin_lon;
    let z = (n * (1.0 - WGS84_E2) + alt as f64) * sin_lat;
    
    Vector3::new(x, y, z)
}

/// Convert ECEF (x/y/z) to WGS84 (lat/lon/alt)
fn ecef_to_wgs84(x: f64, y: f64, z: f64) -> [f64; 3] {
    let lon = y.atan2(x);
    let p = (x * x + y * y).sqrt();
    
    // Iterative solution for latitude
    let mut lat = (z / p).atan();
    for _ in 0..5 {
        let sin_lat = lat.sin();
        let n = WGS84_A / (1.0 - WGS84_E2 * sin_lat * sin_lat).sqrt();
        let alt = p / lat.cos() - n;
        lat = (z / (p * (1.0 - WGS84_E2 * n / (n + alt)))).atan();
    }
    
    let sin_lat = lat.sin();
    let n = WGS84_A / (1.0 - WGS84_E2 * sin_lat * sin_lat).sqrt();
    let alt = p / lat.cos() - n;
    
    [lat.to_degrees(), lon.to_degrees(), alt]
}

/// Create rotation matrix from heading (yaw angle)
fn create_rotation_matrix(heading_deg: f32) -> Matrix3<f64> {
    let heading_rad = (heading_deg as f64).to_radians();
    let cos_h = heading_rad.cos();
    let sin_h = heading_rad.sin();
    
    // Rotation around Y-axis (vertical)
    Matrix3::new(
        cos_h,  0.0,  sin_h,
        0.0,    1.0,  0.0,
        -sin_h, 0.0,  cos_h,
    )
}

/// THE "GOD TRIANGLE" - Convert camera-relative to global GPS
fn camera_to_global(
    camera_relative: [f32; 3],  // [x, y, z] from camera in meters
    agent_gps: &VirtualGPS
) -> [f64; 3] {
    // 1. Convert agent GPS to ECEF
    let agent_ecef = wgs84_to_ecef(
        agent_gps.latitude,
        agent_gps.longitude,
        agent_gps.altitude
    );
    
    // 2. Create rotation matrix from heading
    let rotation = create_rotation_matrix(agent_gps.heading);
    
    // 3. Convert camera vector to Vector3
    let camera_vec = Vector3::new(
        camera_relative[0] as f64,
        camera_relative[1] as f64,
        camera_relative[2] as f64
    );
    
    // 4. Rotate camera-relative vector to world coordinates
    let world_vector = rotation * camera_vec;
    
    // 5. Add to agent position in ECEF
    let target_ecef = agent_ecef + world_vector;
    
    // 6. Convert back to WGS84
    ecef_to_wgs84(target_ecef.x, target_ecef.y, target_ecef.z)
}
```

**Update Main Function:**

```rust
#[tokio::main]
async fn main() -> Result<()> {
    println!("[X-RAY EMITTER] Initializing GodView Agent (GLOBAL MODE)...");

    // Initialize Zenoh Session
    let config = zenoh::Config::default();
    let session = zenoh::open(config).await?;
    println!("[X-RAY EMITTER] Zenoh session established");

    let key = "godview/global/hazards";  // NEW: global topic
    println!("[X-RAY EMITTER] Publishing to key: {}", key);

    // Initialize Virtual GPS (read from environment or hardcode)
    let virtual_gps = VirtualGPS {
        latitude: std::env::var("AGENT_GPS_LAT")
            .unwrap_or("37.7749".to_string())
            .parse()?,
        longitude: std::env::var("AGENT_GPS_LON")
            .unwrap_or("-122.4194".to_string())
            .parse()?,
        altitude: std::env::var("AGENT_GPS_ALT")
            .unwrap_or("10.0".to_string())
            .parse()?,
        heading: std::env::var("AGENT_HEADING")
            .unwrap_or("0.0".to_string())
            .parse()?,
    };
    
    let agent_id = std::env::var("AGENT_ID")
        .unwrap_or("agent_warehouse_1".to_string());

    println!("[X-RAY EMITTER] Agent GPS: lat={}, lon={}, alt={}, heading={}°",
        virtual_gps.latitude, virtual_gps.longitude, virtual_gps.altitude, virtual_gps.heading);

    // Open Webcam
    let mut cam = VideoCapture::new(0, CAP_ANY)?;
    if !cam.is_opened()? {
        anyhow::bail!("Failed to open webcam (device 0)");
    }

    // Load Haar Cascade
    let cascade_path = "haarcascade_frontalface_alt.xml";
    let mut face_cascade = CascadeClassifier::new(cascade_path)?;

    let mut frame = Mat::default();
    let mut gray = Mat::default();
    let mut hazard_counter = 0u64;

    println!("[X-RAY EMITTER] Starting detection loop (30 Hz)...");
    
    loop {
        cam.read(&mut frame)?;
        if frame.empty() {
            sleep(Duration::from_millis(33)).await;
            continue;
        }

        imgproc::cvt_color(&frame, &mut gray, imgproc::COLOR_BGR2GRAY, 0)?;

        let mut faces = Vector::<opencv::core::Rect>::new();
        face_cascade.detect_multi_scale(
            &gray,
            &mut faces,
            1.1, 3, 0,
            opencv::core::Size::new(30, 30),
            opencv::core::Size::new(0, 0),
        )?;

        for face in faces.iter() {
            let frame_width = frame.cols() as f32;
            let center_x = frame_width / 2.0;
            
            let face_x = face.x as f32 + (face.width as f32 / 2.0);
            let face_width_px = face.width as f32;

            // 3D PROJECTION MATH (same as before)
            let z = (FOCAL_LENGTH_CONST * REAL_FACE_WIDTH_M) / face_width_px;
            let x = (face_x - center_x) * z / FOCAL_LENGTH_CONST;
            let y = 0.0;

            let camera_pos = [x, y, z];
            
            // NEW: Convert to global GPS coordinates
            let global_pos = camera_to_global(camera_pos, &virtual_gps);

            hazard_counter += 1;
            let timestamp = SystemTime::now()
                .duration_since(UNIX_EPOCH)?
                .as_millis() as i64;

            let packet = GlobalHazardPacket {
                id: format!("{}_{}", agent_id, hazard_counter),
                timestamp,
                global_pos,  // NOW IN GPS COORDINATES!
                agent_id: agent_id.clone(),
                agent_pos: [virtual_gps.latitude, virtual_gps.longitude, virtual_gps.altitude as f64],
                agent_heading: virtual_gps.heading,
                hazard_type: "human_face".to_string(),
            };

            let json_payload = serde_json::to_string(&packet)?;
            session.put(key, json_payload.clone()).await?;

            println!(
                "[X-RAY EMITTER] Sent Hazard at GPS: [{:.6}, {:.6}, {:.2}] | Camera: [{:.2}, {:.2}, {:.2}]",
                global_pos[0], global_pos[1], global_pos[2],
                x, y, z
            );
        }

        sleep(Duration::from_millis(33)).await;
    }
}
```

---

### Component 2: Web Viewer - Geospatial Projection

#### File: `viewer/src/main.js`

**Add World Origin Configuration:**

```javascript
// The "World Anchor" - All coordinates relative to this point
const WORLD_ORIGIN = {
    lat: parseFloat(import.meta.env.VITE_WORLD_ORIGIN_LAT || '37.7749'),
    lon: parseFloat(import.meta.env.VITE_WORLD_ORIGIN_LON || '-122.4194'),
    alt: parseFloat(import.meta.env.VITE_WORLD_ORIGIN_ALT || '0.0')
};

console.log('[GODVIEW] World Origin:', WORLD_ORIGIN);
```

**Add GPS to Scene Coordinate Converter:**

```javascript
/**
 * Convert GPS coordinates to Three.js scene coordinates
 * Uses simple equirectangular projection for local areas (<10km)
 * 
 * @param {number} lat - Latitude in degrees
 * @param {number} lon - Longitude in degrees
 * @param {number} alt - Altitude in meters
 * @returns {THREE.Vector3} - Scene coordinates in meters from origin
 */
function gpsToSceneCoords(lat, lon, alt) {
    // Constants for WGS84 ellipsoid
    const EARTH_RADIUS = 6378137.0;  // meters at equator
    
    // Calculate offset from world origin
    const lat_diff = (lat - WORLD_ORIGIN.lat) * (Math.PI / 180);
    const lon_diff = (lon - WORLD_ORIGIN.lon) * (Math.PI / 180);
    
    // Convert to meters using equirectangular approximation
    // Good for distances < 10km
    const x = lon_diff * EARTH_RADIUS * Math.cos(WORLD_ORIGIN.lat * Math.PI / 180);
    const z = -lat_diff * EARTH_RADIUS;  // Negative because Three.js Z is forward
    const y = alt - WORLD_ORIGIN.alt;
    
    return new THREE.Vector3(x, y, z);
}
```

**Update Zenoh Callback:**

```javascript
const subscriber = await session.declareSubscriber('godview/global/hazards', {
    callback: (sample) => {
        try {
            const payload = sample.payload.deserialize();
            const data = JSON.parse(payload);
            
            console.log('[GODVIEW] Received Global Hazard:', data);
            
            // Extract global GPS coordinates
            const [lat, lon, alt] = data.global_pos;
            
            // Convert to scene coordinates
            const scenePos = gpsToSceneCoords(lat, lon, alt);
            
            console.log(`[GODVIEW] GPS(${lat.toFixed(6)}, ${lon.toFixed(6)}) → Scene(${scenePos.x.toFixed(2)}, ${scenePos.y.toFixed(2)}, ${scenePos.z.toFixed(2)})`);
            
            // Extract agent ID
            const agentId = data.id;
            
            // Update or create ghost
            if (!ghosts.has(agentId)) {
                console.log(`[GODVIEW] Spawning new ghost for: ${agentId}`);
                const newGhost = createGhost();
                scene.add(newGhost);
                ghosts.set(agentId, newGhost);
            }
            
            const ghost = ghosts.get(agentId);
            ghost.userData.targetPos.copy(scenePos);  // NOW IN GLOBAL COORDINATES!
            ghost.userData.lastUpdate = Date.now();
            
            // Calculate latency
            const latency = Date.now() - data.timestamp;
            latencyElement.textContent = latency;
            
            // Update status
            statusElement.textContent = `TRACKING ${ghosts.size} HAZARD(S) (GLOBAL MODE)`;
            statusElement.style.color = '#ff0000';
            
        } catch (error) {
            console.error('[GODVIEW] Error processing message:', error);
        }
    }
});
```

**Add Visual Debug Helpers:**

```javascript
// Show world origin as green sphere
const originGeometry = new THREE.SphereGeometry(0.5, 16, 16);
const originMaterial = new THREE.MeshBasicMaterial({ color: 0x00ff00 });
const originMarker = new THREE.Mesh(originGeometry, originMaterial);
originMarker.position.set(0, 0, 0);
scene.add(originMarker);

// Add text label for origin
const canvas = document.createElement('canvas');
const context = canvas.getContext('2d');
canvas.width = 256;
canvas.height = 64;
context.fillStyle = '#00ff00';
context.font = '24px monospace';
context.fillText('WORLD ORIGIN', 10, 30);
context.fillText(`${WORLD_ORIGIN.lat.toFixed(4)}, ${WORLD_ORIGIN.lon.toFixed(4)}`, 10, 55);

const texture = new THREE.CanvasTexture(canvas);
const spriteMaterial = new THREE.SpriteMaterial({ map: texture });
const sprite = new THREE.Sprite(spriteMaterial);
sprite.position.set(0, 1, 0);
sprite.scale.set(2, 0.5, 1);
scene.add(sprite);
```

---

### Component 3: Multi-Agent Simulation Script

#### File: `sim_movement.sh`

```bash
#!/bin/bash

# ============================================
# GODVIEW GLOBAL COORDINATE TEST
# Two Agents, One Hazard - Proof of Concept
# ============================================

set -e

echo "╔════════════════════════════════════════════╗"
echo "║   GODVIEW GLOBAL COORDINATE TEST           ║"
echo "║   Two Agents, One Hazard                   ║"
echo "╚════════════════════════════════════════════╝"
echo ""

# Test Scenario:
# - World Origin: 37.7749, -122.4194 (San Francisco)
# - Hazard Location: 37.7750, -122.4193 (10m North, 10m East of origin)
# - Agent A: 37.7749, -122.4194, facing North (0°)
#   - Sees hazard at camera position: x=10m, z=10m
# - Agent B: 37.7749, -122.4192, facing West (270°)
#   - Sees hazard at camera position: x=10m, z=20m
# - BOTH should publish SAME global GPS coordinates

# Cleanup function
cleanup() {
    echo ""
    echo "[TEST] Shutting down..."
    kill $PID_ZENOH $PID_AGENT_A $PID_AGENT_B $PID_VIEWER 2>/dev/null || true
    exit 0
}

trap cleanup SIGINT SIGTERM

# Start Zenoh router
echo "--- [TEST] STARTING ZENOH ROUTER ---"
zenohd --listen tcp/0.0.0.0:7447 --rest-http-port 8000 &
PID_ZENOH=$!
sleep 2

# Start Agent A (North-facing)
echo ""
echo "--- [TEST] STARTING AGENT A (NORTH-FACING) ---"
echo "GPS: 37.7749, -122.4194"
echo "Heading: 0° (North)"
cd agent
AGENT_GPS_LAT=37.7749 \
AGENT_GPS_LON=-122.4194 \
AGENT_GPS_ALT=10.0 \
AGENT_HEADING=0.0 \
AGENT_ID="agent_a_north" \
cargo run --release &
PID_AGENT_A=$!
cd ..
sleep 3

# Start Agent B (West-facing)
echo ""
echo "--- [TEST] STARTING AGENT B (WEST-FACING) ---"
echo "GPS: 37.7749, -122.4192"
echo "Heading: 270° (West)"
cd agent
AGENT_GPS_LAT=37.7749 \
AGENT_GPS_LON=-122.4192 \
AGENT_GPS_ALT=10.0 \
AGENT_HEADING=270.0 \
AGENT_ID="agent_b_west" \
cargo run --release &
PID_AGENT_B=$!
cd ..
sleep 3

# Start viewer
echo ""
echo "--- [TEST] STARTING VIEWER ---"
cd viewer
VITE_WORLD_ORIGIN_LAT=37.7749 \
VITE_WORLD_ORIGIN_LON=-122.4194 \
VITE_WORLD_ORIGIN_ALT=0.0 \
npm run dev &
PID_VIEWER=$!
cd ..

echo ""
echo "╔════════════════════════════════════════════╗"
echo "║          TEST RUNNING                      ║"
echo "╚════════════════════════════════════════════╝"
echo ""
echo "Expected Behavior:"
echo "1. Both agents detect the same face"
echo "2. Both publish DIFFERENT camera-relative coordinates"
echo "3. Both publish SAME global GPS coordinates"
echo "4. Viewer shows ONE ghost (or two very close ghosts)"
echo ""
echo "Open browser to Vite URL (typically http://localhost:5173)"
echo ""
echo "Press Ctrl+C to stop"
echo ""

wait $PID_ZENOH $PID_AGENT_A $PID_AGENT_B $PID_VIEWER
```

---

## Verification Plan

### Unit Tests (add to `agent/src/main.rs`)

```rust
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_wgs84_ecef_roundtrip() {
        let lat = 37.7749;
        let lon = -122.4194;
        let alt = 10.0;
        
        let ecef = wgs84_to_ecef(lat, lon, alt);
        let wgs84 = ecef_to_wgs84(ecef.x, ecef.y, ecef.z);
        
        assert!((wgs84[0] - lat).abs() < 1e-6, "Latitude mismatch");
        assert!((wgs84[1] - lon).abs() < 1e-6, "Longitude mismatch");
        assert!((wgs84[2] - alt as f64).abs() < 1e-3, "Altitude mismatch");
    }

    #[test]
    fn test_god_triangle_north_facing() {
        let agent = VirtualGPS {
            latitude: 37.7749,
            longitude: -122.4194,
            altitude: 10.0,
            heading: 0.0,  // North
        };
        
        // Object 10m directly ahead (North)
        let camera_pos = [0.0, 0.0, 10.0];
        let global = camera_to_global(camera_pos, &agent);
        
        // Should be ~10m North of agent
        assert!(global[0] > agent.latitude, "Should be more North");
        assert!((global[1] - agent.longitude).abs() < 1e-6, "Should be same longitude");
        
        println!("Agent: {}, {}", agent.latitude, agent.longitude);
        println!("Hazard: {}, {}", global[0], global[1]);
    }

    #[test]
    fn test_two_agents_same_target() {
        // Agent A: North-facing, 10m South of target
        let agent_a = VirtualGPS {
            latitude: 37.7749,
            longitude: -122.4194,
            altitude: 10.0,
            heading: 0.0,
        };
        let camera_a = [0.0, 0.0, 10.0];  // 10m ahead
        
        // Agent B: East-facing, 10m West of target
        let agent_b = VirtualGPS {
            latitude: 37.7750,
            longitude: -122.4195,
            altitude: 10.0,
            heading: 90.0,
        };
        let camera_b = [0.0, 0.0, 10.0];  // 10m ahead
        
        let global_a = camera_to_global(camera_a, &agent_a);
        let global_b = camera_to_global(camera_b, &agent_b);
        
        println!("Agent A sees hazard at GPS: {}, {}", global_a[0], global_a[1]);
        println!("Agent B sees hazard at GPS: {}, {}", global_b[0], global_b[1]);
        
        // Should be same global coordinates (within 1cm = 1e-7 degrees)
        let lat_diff = (global_a[0] - global_b[0]).abs();
        let lon_diff = (global_a[1] - global_b[1]).abs();
        
        assert!(lat_diff < 1e-6, "Latitude difference too large: {}", lat_diff);
        assert!(lon_diff < 1e-6, "Longitude difference too large: {}", lon_diff);
    }
}
```

---

## Success Criteria

1. ✅ Unit tests pass (coordinate transforms work)
2. ✅ Two agents at different positions detect same object
3. ✅ Both agents publish same global GPS (within 1cm)
4. ✅ Viewer renders ghost at correct position
5. ✅ Distance calculations are accurate

---

## Estimated Timeline

- **Week 1:** Implement coordinate transforms, update Rust agent
- **Week 2:** Update viewer, create simulation script, testing
- **Total:** 1-2 weeks

---

**This plan solves the critical "Horizon Constraint" identified in the system audit.**

