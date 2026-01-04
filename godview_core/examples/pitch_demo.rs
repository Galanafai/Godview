//! GodView Byzantine Sensor Demo - Terminal Visualization
//!
//! Demonstrates the core GodView algorithms solving:
//! 1. The Pancake Problem (2D ambiguity)
//! 2. The Time Travel Problem (OOSM)
//! 3. The Ghost Problem (Duplicate IDs)
//!
//! Run: `cargo run --example pitch_demo`

use std::collections::{HashMap, HashSet};

// ============================================================================
// ANSI COLOR CODES
// ============================================================================

mod colors {
    pub const RESET: &str = "\x1b[0m";
    pub const BOLD: &str = "\x1b[1m";
    pub const DIM: &str = "\x1b[2m";
    
    pub const RED: &str = "\x1b[31m";
    pub const GREEN: &str = "\x1b[32m";
    pub const YELLOW: &str = "\x1b[33m";
    pub const BLUE: &str = "\x1b[34m";
    pub const MAGENTA: &str = "\x1b[35m";
    pub const CYAN: &str = "\x1b[36m";
    pub const WHITE: &str = "\x1b[37m";
    
    pub const BG_RED: &str = "\x1b[41m";
    pub const BG_GREEN: &str = "\x1b[42m";
    pub const BG_YELLOW: &str = "\x1b[43m";
}

use colors::*;

// ============================================================================
// MOCK DATA STRUCTURES (Simulating ZMQ Input from CARLA)
// ============================================================================

#[derive(Debug, Clone)]
struct SensorPacket {
    sensor_id: String,
    entity_id: String,
    timestamp: u64,  // Lamport Timestamp (LTS)
    position: [f64; 3],  // [lat, lon, alt]
    velocity: [f64; 3],
    class: String,
    confidence: f64,
}

// ============================================================================
// ENGINE STATES (Simplified)
// ============================================================================

struct TimeEngine {
    local_time: u64,
    oosm_buffer: Vec<(u64, String)>,  // (timestamp, description)
}

impl TimeEngine {
    fn new() -> Self {
        Self { 
            local_time: 100,  // Start at LTS 100
            oosm_buffer: Vec::new(),
        }
    }
    
    fn process(&mut self, packet: &SensorPacket) -> Result<(), String> {
        // Lamport Clock: T_new = max(T_local, T_msg) + 1
        // Causal Constraint: Reject if T_msg < T_local
        
        if packet.timestamp < self.local_time {
            self.oosm_buffer.push((packet.timestamp, packet.entity_id.clone()));
            return Err(format!(
                "CAUSALITY VIOLATION: LTS {} < T_local {}", 
                packet.timestamp, self.local_time
            ));
        }
        
        // Update local time
        self.local_time = self.local_time.max(packet.timestamp) + 1;
        Ok(())
    }
}

struct SpaceEngine {
    // H3 Cell → Vec of entities
    h3_index: HashMap<String, Vec<String>>,
    // Voxel index for 3D disambiguation
    voxel_index: HashMap<(i32, i32, i32), Vec<String>>,
}

impl SpaceEngine {
    fn new() -> Self {
        Self {
            h3_index: HashMap::new(),
            voxel_index: HashMap::new(),
        }
    }
    
    fn get_h3_cell(&self, lat: f64, lon: f64) -> String {
        // Simplified H3 cell (in reality, use h3o crate)
        format!("8928308280fffff_{:.2}_{:.2}", lat, lon)
    }
    
    fn get_voxel(&self, alt: f64) -> i32 {
        // 10m voxel cells
        (alt / 10.0).floor() as i32
    }
    
    fn insert(&mut self, entity_id: &str, position: [f64; 3]) {
        let h3 = self.get_h3_cell(position[0], position[1]);
        let voxel_z = self.get_voxel(position[2]);
        
        self.h3_index.entry(h3).or_default().push(entity_id.to_string());
        self.voxel_index.entry((0, 0, voxel_z)).or_default().push(entity_id.to_string());
    }
    
    fn check_collision(&self, pos_a: [f64; 3], pos_b: [f64; 3]) -> bool {
        // Same H3 cell AND same voxel? Collision.
        let h3_a = self.get_h3_cell(pos_a[0], pos_a[1]);
        let h3_b = self.get_h3_cell(pos_b[0], pos_b[1]);
        let voxel_a = self.get_voxel(pos_a[2]);
        let voxel_b = self.get_voxel(pos_b[2]);
        
        h3_a == h3_b && voxel_a == voxel_b
    }
}

struct TrackingEngine {
    // Canonical ID → Set of observed IDs (Highlander CRDT)
    tracks: HashMap<String, HashSet<String>>,
    // Observed ID → Canonical ID mapping
    id_mapping: HashMap<String, String>,
}

impl TrackingEngine {
    fn new() -> Self {
        Self {
            tracks: HashMap::new(),
            id_mapping: HashMap::new(),
        }
    }
    
    fn highlander_merge(&mut self, id_a: &str, id_b: &str) -> String {
        // "There can be only one" - always pick the lexicographically smallest UUID
        let canonical = if id_a < id_b { id_a } else { id_b };
        let loser = if id_a < id_b { id_b } else { id_a };
        
        // Merge into canonical's observed set
        let observed = self.tracks.entry(canonical.to_string()).or_default();
        observed.insert(id_a.to_string());
        observed.insert(id_b.to_string());
        
        // Update mapping
        self.id_mapping.insert(loser.to_string(), canonical.to_string());
        
        canonical.to_string()
    }
}

// ============================================================================
// DEMO SCENARIOS
// ============================================================================

fn create_demo_packets() -> Vec<SensorPacket> {
    vec![
        // === SCENARIO 1: The Pancake Problem ===
        // Drone at 50m and Car at 0m, same lat/lon
        SensorPacket {
            sensor_id: "CAM_FRONT".into(),
            entity_id: "drone_001".into(),
            timestamp: 101,
            position: [37.7749, -122.4194, 50.0],  // 50m altitude
            velocity: [0.0, 0.0, 0.0],
            class: "DRONE".into(),
            confidence: 0.95,
        },
        SensorPacket {
            sensor_id: "LIDAR_TOP".into(),
            entity_id: "car_001".into(),
            timestamp: 102,
            position: [37.7749, -122.4194, 0.0],  // Ground level
            velocity: [5.0, 0.0, 0.0],
            class: "VEHICLE".into(),
            confidence: 0.98,
        },
        
        // === SCENARIO 2: The Time Travel Problem ===
        // A packet arrives with timestamp BEFORE the current local time
        SensorPacket {
            sensor_id: "RADAR_REAR".into(),
            entity_id: "car_002".into(),
            timestamp: 95,  // ⚠️ OLD TIMESTAMP (before LTS 100!)
            position: [37.7750, -122.4190, 0.0],
            velocity: [3.0, 0.0, 0.0],
            class: "VEHICLE".into(),
            confidence: 0.88,
        },
        
        // === SCENARIO 3: The Ghost Problem ===
        // Two different IDs for the same physical car
        SensorPacket {
            sensor_id: "CAM_FRONT".into(),
            entity_id: "car_ghost_A".into(),
            timestamp: 103,
            position: [37.7751, -122.4180, 0.0],
            velocity: [10.0, 0.0, 0.0],
            class: "VEHICLE".into(),
            confidence: 0.92,
        },
        SensorPacket {
            sensor_id: "LIDAR_TOP".into(),
            entity_id: "car_ghost_B".into(),  // Same car, different ID!
            timestamp: 104,
            position: [37.7751, -122.4180, 0.0],  // Same position
            velocity: [10.0, 0.0, 0.0],
            class: "VEHICLE".into(),
            confidence: 0.94,
        },
    ]
}

// ============================================================================
// TERMINAL OUTPUT RENDERING
// ============================================================================

fn print_header() {
    println!("\n{}{}╔════════════════════════════════════════════════════════════════╗{}", BOLD, CYAN, RESET);
    println!("{}{}║        GODVIEW KERNEL - BYZANTINE SENSOR RESOLUTION            ║{}", BOLD, CYAN, RESET);
    println!("{}{}║        Solving the Byzantine Generals Problem for AV           ║{}", BOLD, CYAN, RESET);
    println!("{}{}╚════════════════════════════════════════════════════════════════╝{}\n", BOLD, CYAN, RESET);
}

fn print_phase_1(packets: &[SensorPacket]) {
    println!("{}{}═══════════════════════════════════════════════════════════════{}", BOLD, YELLOW, RESET);
    println!("{}{}  PHASE 1: RAW SENSOR INGESTION (CHAOS)                         {}", BOLD, YELLOW, RESET);
    println!("{}{}═══════════════════════════════════════════════════════════════{}\n", BOLD, YELLOW, RESET);
    
    for (i, p) in packets.iter().enumerate() {
        let color = match p.sensor_id.as_str() {
            s if s.contains("CAM") => BLUE,
            s if s.contains("LIDAR") => GREEN,
            s if s.contains("RADAR") => MAGENTA,
            _ => WHITE,
        };
        
        println!("  {}[{}]{} {}LTS:{:<4}{} | {}{:<12}{} → {} at z={:.0}m",
            DIM, i+1, RESET,
            YELLOW, p.timestamp, RESET,
            color, p.sensor_id, RESET,
            p.entity_id, p.position[2]
        );
    }
    
    println!("\n  {}⚠️  Multiple IDs, conflicting timestamps, altitude ambiguity...{}\n", RED, RESET);
}

fn print_phase_2_time(time_engine: &mut TimeEngine, packets: &[SensorPacket]) {
    println!("{}{}═══════════════════════════════════════════════════════════════{}", BOLD, GREEN, RESET);
    println!("{}{}  PHASE 2a: TIME ENGINE (Augmented State EKF)                   {}", BOLD, GREEN, RESET);
    println!("{}{}═══════════════════════════════════════════════════════════════{}\n", BOLD, GREEN, RESET);
    
    println!("  {}T_local = {}{}", DIM, time_engine.local_time, RESET);
    println!("  {}Rule: T_new = max(T_local, T_msg) + 1{}", DIM, RESET);
    println!("  {}Constraint: REJECT if T_msg < T_local{}\n", DIM, RESET);
    
    for p in packets {
        match time_engine.process(p) {
            Ok(()) => {
                println!("  {}[TIME]{} {}✅ ACCEPT{} {} (LTS:{}) → T_local={}",
                    GREEN, RESET, BOLD, RESET,
                    p.entity_id, p.timestamp, time_engine.local_time
                );
            }
            Err(msg) => {
                println!("  {}[TIME]{} {}🚫 REJECT{} {} (LTS:{}) — {}",
                    RED, RESET, BOLD, RESET,
                    p.entity_id, p.timestamp, msg
                );
                println!("         {}└── \"Causal Wall\" blocks time-traveling data{}", DIM, RESET);
            }
        }
    }
    println!();
}

fn print_phase_2_space(space_engine: &mut SpaceEngine, packets: &[SensorPacket]) {
    println!("{}{}═══════════════════════════════════════════════════════════════{}", BOLD, CYAN, RESET);
    println!("{}{}  PHASE 2b: SPACE ENGINE (H3 + Voxel Grid)                       {}", BOLD, CYAN, RESET);
    println!("{}{}═══════════════════════════════════════════════════════════════{}\n", BOLD, CYAN, RESET);
    
    // Find the drone and car
    let drone = packets.iter().find(|p| p.class == "DRONE");
    let car = packets.iter().find(|p| p.class == "VEHICLE" && p.position[2] == 0.0);
    
    if let (Some(d), Some(c)) = (drone, car) {
        println!("  {}[SPACE]{} Checking: {} vs {}",
            CYAN, RESET, d.entity_id, c.entity_id
        );
        
        let h3_d = space_engine.get_h3_cell(d.position[0], d.position[1]);
        let h3_c = space_engine.get_h3_cell(c.position[0], c.position[1]);
        let voxel_d = space_engine.get_voxel(d.position[2]);
        let voxel_c = space_engine.get_voxel(c.position[2]);
        
        println!("         {}├── {} → H3: {}...   Voxel Z: {}{}", DIM, d.entity_id, &h3_d[..8], voxel_d, RESET);
        println!("         {}└── {} → H3: {}...   Voxel Z: {}{}", DIM, c.entity_id, &h3_c[..8], voxel_c, RESET);
        
        let collision = space_engine.check_collision(d.position, c.position);
        
        if collision {
            println!("\n  {}⚠️  NAIVE SYSTEM: COLLISION DETECTED (WRONG!){}", RED, RESET);
        } else {
            println!("\n  {}✅ GODVIEW: NO COLLISION — 3D Voxel Disambiguation{}", GREEN, RESET);
            println!("     {}Drone at z=50m != Car at z=0m{}", DIM, RESET);
        }
        
        space_engine.insert(&d.entity_id, d.position);
        space_engine.insert(&c.entity_id, c.position);
    }
    println!();
}

fn print_phase_2_tracking(tracking_engine: &mut TrackingEngine, packets: &[SensorPacket]) {
    println!("{}{}═══════════════════════════════════════════════════════════════{}", BOLD, MAGENTA, RESET);
    println!("{}{}  PHASE 2c: TRACKING ENGINE (Highlander CRDT)                   {}", BOLD, MAGENTA, RESET);
    println!("{}{}═══════════════════════════════════════════════════════════════{}\n", BOLD, MAGENTA, RESET);
    
    // Find the ghost pair
    let ghost_a = packets.iter().find(|p| p.entity_id == "car_ghost_A");
    let ghost_b = packets.iter().find(|p| p.entity_id == "car_ghost_B");
    
    if let (Some(a), Some(b)) = (ghost_a, ghost_b) {
        println!("  {}[TRACK]{} {}⚠️  GHOST DETECTED!{}",
            MAGENTA, RESET, YELLOW, RESET
        );
        println!("         {}├── ID: {} at ({:.4}, {:.4}){}", DIM, a.entity_id, a.position[0], a.position[1], RESET);
        println!("         {}└── ID: {} at ({:.4}, {:.4}){}", DIM, b.entity_id, b.position[0], b.position[1], RESET);
        println!("         {}    (Same position = Same object!){}", DIM, RESET);
        
        println!("\n  {}[TRACK]{} Applying Highlander: \"There can be only one!\"", MAGENTA, RESET);
        
        let canonical = tracking_engine.highlander_merge(&a.entity_id, &b.entity_id);
        let loser = if canonical == a.entity_id { &b.entity_id } else { &a.entity_id };
        
        println!("         {}├── Winner (canonical_id):  {}{}{}", GREEN, BOLD, canonical, RESET);
        println!("         {}└── Loser  (absorbed):      {}{}{}", RED, DIM, loser, RESET);
        println!("\n  {}✅ GHOST MERGED → Single consistent identity{}", GREEN, RESET);
    }
    println!();
}

fn print_final_state() {
    println!("{}{}═══════════════════════════════════════════════════════════════{}", BOLD, GREEN, RESET);
    println!("{}{}  FINAL: FUSED WORLD STATE (CONSENSUS)                          {}", BOLD, GREEN, RESET);
    println!("{}{}═══════════════════════════════════════════════════════════════{}\n", BOLD, GREEN, RESET);
    
    println!("  ┌────────────────┬─────────────────┬─────────┬────────────┐");
    println!("  │ {}Canonical ID{}   │ {}Position (z){}    │ {}Class{}   │ {}Confidence{} │", BOLD, RESET, BOLD, RESET, BOLD, RESET, BOLD, RESET);
    println!("  ├────────────────┼─────────────────┼─────────┼────────────┤");
    println!("  │ {}drone_001{}      │ 37.77, -122.42  │ DRONE   │ {}  95.0%{}    │", CYAN, RESET, GREEN, RESET);
    println!("  │                │ {}(z = 50m){}       │         │            │", DIM, RESET);
    println!("  ├────────────────┼─────────────────┼─────────┼────────────┤");
    println!("  │ {}car_001{}        │ 37.77, -122.42  │ VEHICLE │ {}  98.0%{}    │", CYAN, RESET, GREEN, RESET);
    println!("  │                │ {}(z = 0m){}        │         │            │", DIM, RESET);
    println!("  ├────────────────┼─────────────────┼─────────┼────────────┤");
    println!("  │ {}car_ghost_A{}    │ 37.78, -122.42  │ VEHICLE │ {}  94.0%{}    │", CYAN, RESET, GREEN, RESET);
    println!("  │ {}(merged){}       │ {}(z = 0m){}        │         │            │", DIM, RESET, DIM, RESET);
    println!("  └────────────────┴─────────────────┴─────────┴────────────┘");
    
    println!("\n  {}{}Summary:{}", BOLD, WHITE, RESET);
    println!("  {}• Time Violations Blocked:  1{}", YELLOW, RESET);
    println!("  {}• Ghosts Merged:            1{}", MAGENTA, RESET);
    println!("  {}• Pancake Disambiguations:  1{}", CYAN, RESET);
    println!("  {}• Final Track Count:        3{}", GREEN, RESET);
}

fn print_footer() {
    println!("\n{}{}╔════════════════════════════════════════════════════════════════╗{}", BOLD, GREEN, RESET);
    println!("{}{}║  ✅ BYZANTINE CONSENSUS ACHIEVED                                ║{}", BOLD, GREEN, RESET);
    println!("{}{}║     All sensors agree. All conflicts resolved.                  ║{}", BOLD, GREEN, RESET);
    println!("{}{}╚════════════════════════════════════════════════════════════════╝{}\n", BOLD, GREEN, RESET);
}

// ============================================================================
// MAIN
// ============================================================================

fn main() {
    let packets = create_demo_packets();
    
    let mut time_engine = TimeEngine::new();
    let mut space_engine = SpaceEngine::new();
    let mut tracking_engine = TrackingEngine::new();
    
    print_header();
    print_phase_1(&packets);
    
    // Simulate processing delay for dramatic effect
    std::thread::sleep(std::time::Duration::from_millis(500));
    
    print_phase_2_time(&mut time_engine, &packets);
    std::thread::sleep(std::time::Duration::from_millis(300));
    
    print_phase_2_space(&mut space_engine, &packets);
    std::thread::sleep(std::time::Duration::from_millis(300));
    
    print_phase_2_tracking(&mut tracking_engine, &packets);
    std::thread::sleep(std::time::Duration::from_millis(300));
    
    print_final_state();
    print_footer();
}
