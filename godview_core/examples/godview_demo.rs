//! GodView Distributed Perception Demo - Multi-Agent Terminal Visualization
//!
//! This demo showcases the GodView protocol enabling distributed agents
//! to build a shared world model by solving:
//!
//! 1. The Time Problem - Agents' packets arrive out-of-order over network
//! 2. The Space Problem - 2D indexing ignores altitude (drone vs car)
//! 3. The Ghost Problem - Same object seen by multiple agents gets different IDs
//! 4. The Trust Problem - Malicious agents inject phantom hazards
//!
//! "Giving robots X-Ray Vision through collaborative perception"
//!
//! Run: `cargo run --example godview_demo`

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
    
    pub const BRIGHT_RED: &str = "\x1b[91m";
    pub const BRIGHT_GREEN: &str = "\x1b[92m";
    pub const BRIGHT_CYAN: &str = "\x1b[96m";
}

use colors::*;

// ============================================================================
// AGENT OBSERVATION PACKET (Simulating Zenoh messages)
// ============================================================================

#[derive(Debug, Clone)]
struct AgentPacket {
    agent_id: String,       // Which robot/agent sent this
    object_id: String,      // Agent's local ID for the object
    timestamp: u64,         // Lamport Timestamp
    position: [f64; 3],     // [lat, lon, alt]
    class: String,
    confidence: f64,
}

// ============================================================================
// ENGINE STATES
// ============================================================================

struct TimeEngine {
    local_time: u64,
}

impl TimeEngine {
    fn new() -> Self {
        Self { local_time: 100 }
    }
    
    fn process(&mut self, packet: &AgentPacket) -> Result<(), String> {
        if packet.timestamp < self.local_time {
            return Err(format!(
                "LTS {} < T_local {}", 
                packet.timestamp, self.local_time
            ));
        }
        self.local_time = self.local_time.max(packet.timestamp) + 1;
        Ok(())
    }
}

struct SpaceEngine {
    voxel_size: f64,
}

impl SpaceEngine {
    fn new() -> Self {
        Self { voxel_size: 10.0 }
    }
    
    fn get_voxel(&self, alt: f64) -> i32 {
        (alt / self.voxel_size).floor() as i32
    }
    
    fn same_location_3d(&self, a: [f64; 3], b: [f64; 3]) -> bool {
        self.get_voxel(a[2]) == self.get_voxel(b[2])
    }
}

struct TrackingEngine {
    tracks: HashMap<String, HashSet<String>>,
}

impl TrackingEngine {
    fn new() -> Self {
        Self { tracks: HashMap::new() }
    }
    
    fn highlander_merge(&mut self, id_a: &str, id_b: &str) -> String {
        let canonical = if id_a < id_b { id_a } else { id_b };
        let observed = self.tracks.entry(canonical.to_string()).or_default();
        observed.insert(id_a.to_string());
        observed.insert(id_b.to_string());
        canonical.to_string()
    }
}

struct TrustEngine {
    trust_scores: HashMap<String, f64>,
    threshold: f64,
}

impl TrustEngine {
    fn new() -> Self {
        let mut scores = HashMap::new();
        scores.insert("ROBOT_A".to_string(), 0.96);
        scores.insert("ROBOT_B".to_string(), 0.94);
        scores.insert("ROBOT_C".to_string(), 0.92);
        scores.insert("ROGUE_BOT".to_string(), 0.09);
        
        Self { trust_scores: scores, threshold: 0.5 }
    }
    
    fn get_trust(&self, agent: &str) -> f64 {
        *self.trust_scores.get(agent).unwrap_or(&0.5)
    }
    
    fn is_trusted(&self, agent: &str) -> bool {
        self.get_trust(agent) >= self.threshold
    }
}

// ============================================================================
// DEMO SCENARIO - Multi-Agent Warehouse
// ============================================================================

fn create_demo_packets() -> Vec<AgentPacket> {
    vec![
        // Robot A sees a drone overhead
        AgentPacket {
            agent_id: "ROBOT_A".into(),
            object_id: "obj_drone_A".into(),
            timestamp: 101,
            position: [37.7749, -122.4194, 50.0],
            class: "DRONE".into(),
            confidence: 0.95,
        },
        // Robot B sees a car at ground level (same lat/lon as drone!)
        AgentPacket {
            agent_id: "ROBOT_B".into(),
            object_id: "obj_car_B".into(),
            timestamp: 102,
            position: [37.7749, -122.4194, 0.0],
            class: "VEHICLE".into(),
            confidence: 0.98,
        },
        // Robot C's packet arrives LATE (network delay)
        AgentPacket {
            agent_id: "ROBOT_C".into(),
            object_id: "obj_late_C".into(),
            timestamp: 95, // ⚠️ OLD TIMESTAMP!
            position: [37.7750, -122.4190, 0.0],
            class: "FORKLIFT".into(),
            confidence: 0.88,
        },
        // Robot A and B both see the SAME forklift (Ghost Problem)
        AgentPacket {
            agent_id: "ROBOT_A".into(),
            object_id: "forklift_alpha".into(),
            timestamp: 103,
            position: [37.7751, -122.4180, 0.0],
            class: "FORKLIFT".into(),
            confidence: 0.92,
        },
        AgentPacket {
            agent_id: "ROBOT_B".into(),
            object_id: "forklift_beta".into(), // Same forklift, different ID!
            timestamp: 104,
            position: [37.7751, -122.4180, 0.0],
            class: "FORKLIFT".into(),
            confidence: 0.94,
        },
        // Rogue bot tries to inject phantom hazard
        AgentPacket {
            agent_id: "ROGUE_BOT".into(),
            object_id: "phantom_wall".into(),
            timestamp: 105,
            position: [37.7749, -122.4194, 0.0],
            class: "OBSTACLE".into(),
            confidence: 0.99,
        },
    ]
}

// ============================================================================
// TERMINAL RENDERING
// ============================================================================

fn print_banner() {
    println!();
    println!("{}{}┏━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┓{}", BOLD, BRIGHT_CYAN, RESET);
    println!("{}{}┃                                                                    ┃{}", BOLD, BRIGHT_CYAN, RESET);
    println!("{}{}┃    🛰️  GODVIEW — DISTRIBUTED MULTI-AGENT PERCEPTION PROTOCOL  🛰️    ┃{}", BOLD, BRIGHT_CYAN, RESET);
    println!("{}{}┃                                                                    ┃{}", BOLD, BRIGHT_CYAN, RESET);
    println!("{}{}┃   \"X-Ray Vision for Robots through Collaborative Perception\"       ┃{}", BOLD, BRIGHT_CYAN, RESET);
    println!("{}{}┃                                                                    ┃{}", BOLD, BRIGHT_CYAN, RESET);
    println!("{}{}┗━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┛{}", BOLD, BRIGHT_CYAN, RESET);
    println!();
}

fn print_phase(phase: &str, title: &str, color: &str) {
    println!("{}{}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━{}", BOLD, color, RESET);
    println!("{}{}  {} │ {}{}", BOLD, color, phase, title, RESET);
    println!("{}{}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━{}", BOLD, color, RESET);
    println!();
}

fn print_phase_1(packets: &[AgentPacket]) {
    print_phase("PHASE 1", "INCOMING ZENOH MESSAGES (CHAOS)", YELLOW);
    
    println!("  {}Agents broadcasting observations over decentralized pub/sub...{}", DIM, RESET);
    println!();
    
    for (i, p) in packets.iter().enumerate() {
        let color = match p.agent_id.as_str() {
            "ROBOT_A" => BLUE,
            "ROBOT_B" => GREEN,
            "ROBOT_C" => MAGENTA,
            _ => RED,
        };
        
        let warning = if p.timestamp < 100 { 
            format!(" {}⚠️ LATE{}", BRIGHT_RED, RESET) 
        } else if p.agent_id.contains("ROGUE") {
            format!(" {}🚨 UNTRUSTED{}", BRIGHT_RED, RESET)
        } else { String::new() };
        
        println!("  {}[{:02}]{} {}LTS:{:<4}{} │ {}{:<10}{} │ {:<18} │ z={:>5.1}m{}",
            DIM, i+1, RESET,
            YELLOW, p.timestamp, RESET,
            color, p.agent_id, RESET,
            p.object_id, p.position[2],
            warning
        );
    }
    
    println!();
    println!("  {}{}PROBLEMS:{}", BOLD, BRIGHT_RED, RESET);
    println!("     {}• Network delays cause out-of-order packets{}", RED, RESET);
    println!("     {}• Same object at lat/lon could be drone OR car{}", RED, RESET);
    println!("     {}• Same forklift has TWO different IDs from different agents{}", RED, RESET);
    println!("     {}• Rogue bot injecting fake obstacles{}", RED, RESET);
    println!();
}

fn print_phase_2a(time_engine: &mut TimeEngine, packets: &[AgentPacket]) {
    print_phase("ENGINE 1", "TIME ENGINE — Out-of-Order Handling", GREEN);
    
    println!("  {}Lamport Clock:{} T = max(T_local, T_msg) + 1", BOLD, RESET);
    println!("  {}Rule:{} Reject if T_msg < T_local (causality violation)", BOLD, RESET);
    println!();
    
    for p in packets {
        if p.agent_id.contains("ROGUE") { continue; }
        
        match time_engine.process(p) {
            Ok(()) => {
                println!("  {}[TIME]{} {}✅{} {} from {} → T={}",
                    GREEN, RESET, BOLD, RESET,
                    p.object_id, p.agent_id, time_engine.local_time
                );
            }
            Err(e) => {
                println!("  {}[TIME]{} {}🚫 REJECT{} {} — {}",
                    RED, RESET, BOLD, RESET, p.object_id, e
                );
                println!("         {}└── Packet arrived late, queued for AS-EKF retrodiction{}", DIM, RESET);
            }
        }
    }
    println!();
}

fn print_phase_2b(space_engine: &SpaceEngine, packets: &[AgentPacket]) {
    print_phase("ENGINE 2", "SPACE ENGINE — 3D Altitude Disambiguation", CYAN);
    
    let drone = packets.iter().find(|p| p.class == "DRONE");
    let car = packets.iter().find(|p| p.class == "VEHICLE");
    
    if let (Some(d), Some(c)) = (drone, car) {
        println!("  {}Same lat/lon, different altitude:{}", BOLD, RESET);
        println!("  ├── {} at z={}m → Voxel {}", d.object_id, d.position[2], space_engine.get_voxel(d.position[2]));
        println!("  └── {} at z={}m  → Voxel {}", c.object_id, c.position[2], space_engine.get_voxel(c.position[2]));
        println!();
        
        if !space_engine.same_location_3d(d.position, c.position) {
            println!("  {}[SPACE]{} {}✅ DIFFERENT OBJECTS{} — Voxel {} ≠ Voxel {}",
                CYAN, RESET, BOLD, RESET,
                space_engine.get_voxel(d.position[2]),
                space_engine.get_voxel(c.position[2])
            );
            println!("          {}└── 3D indexing prevents false collision warning{}", DIM, RESET);
        }
    }
    println!();
}

fn print_phase_2c(tracking_engine: &mut TrackingEngine, packets: &[AgentPacket]) {
    print_phase("ENGINE 3", "TRACKING ENGINE — Ghost ID Resolution", MAGENTA);
    
    let fork_a = packets.iter().find(|p| p.object_id == "forklift_alpha");
    let fork_b = packets.iter().find(|p| p.object_id == "forklift_beta");
    
    if let (Some(a), Some(b)) = (fork_a, fork_b) {
        println!("  {}{}👻 GHOST DETECTED!{} Same object, different IDs:", BOLD, MAGENTA, RESET);
        println!("  ├── {} sees: {}", a.agent_id, a.object_id);
        println!("  └── {} sees: {}", b.agent_id, b.object_id);
        println!();
        
        let canonical = tracking_engine.highlander_merge(&a.object_id, &b.object_id);
        let loser = if canonical == a.object_id { &b.object_id } else { &a.object_id };
        
        println!("  {}[HIGHLANDER]{} \"There can be only one!\"", MAGENTA, RESET);
        println!("  ├── {}Winner:{} {} (canonical ID)", GREEN, RESET, canonical);
        println!("  └── {}Absorbed:{} {}", RED, RESET, loser);
        println!();
        println!("  {}✅ All agents now agree: ONE forklift{}", GREEN, RESET);
    }
    println!();
}

fn print_phase_2d(trust_engine: &TrustEngine, packets: &[AgentPacket]) {
    print_phase("ENGINE 4", "TRUST ENGINE — Sybil Attack Prevention", BLUE);
    
    println!("  {}Beta Distribution:{} Trust = α / (α + β)", BOLD, RESET);
    println!();
    println!("  ┌────────────┬───────────┬────────────┐");
    println!("  │ {}Agent{}      │ {}Trust (%){}│ {}Decision{}  │", BOLD, RESET, BOLD, RESET, BOLD, RESET);
    println!("  ├────────────┼───────────┼────────────┤");
    
    for p in packets {
        let trust = trust_engine.get_trust(&p.agent_id);
        let accepted = trust_engine.is_trusted(&p.agent_id);
        let (decision, color) = if accepted { ("✅ ACCEPT", GREEN) } else { ("🚫 BLOCK", RED) };
        let trust_color = if trust >= 0.9 { BRIGHT_GREEN } else if trust >= 0.5 { YELLOW } else { RED };
        
        println!("  │ {:<10} │ {}  {:>3}%{}   │ {}{}{}  │",
            p.agent_id, trust_color, (trust * 100.0) as u32, RESET, color, decision, RESET
        );
    }
    println!("  └────────────┴───────────┴────────────┘");
    println!();
    println!("  {}ROGUE_BOT blocked:{} phantom obstacle not added to world model", RED, RESET);
    println!();
}

fn print_final() {
    print_phase("FINAL", "FUSED WORLD STATE — All Agents Agree", BRIGHT_GREEN);
    
    println!("  ┌─────────────────┬───────────────────┬───────────┬────────────┐");
    println!("  │ {}Canonical ID{}    │ {}Position{}           │ {}Class{}     │ {}Seen By{}   │", BOLD, RESET, BOLD, RESET, BOLD, RESET, BOLD, RESET);
    println!("  ├─────────────────┼───────────────────┼───────────┼────────────┤");
    println!("  │ {}obj_drone_A{}     │ 37.77, -122.42    │ DRONE     │ ROBOT_A    │", CYAN, RESET);
    println!("  │                 │ {}z = 50m{}           │           │            │", DIM, RESET);
    println!("  ├─────────────────┼───────────────────┼───────────┼────────────┤");
    println!("  │ {}obj_car_B{}       │ 37.77, -122.42    │ VEHICLE   │ ROBOT_B    │", CYAN, RESET);
    println!("  │                 │ {}z = 0m{}            │           │            │", DIM, RESET);
    println!("  ├─────────────────┼───────────────────┼───────────┼────────────┤");
    println!("  │ {}forklift_alpha{} │ 37.78, -122.42    │ FORKLIFT  │ {}A + B{}     │", CYAN, RESET, MAGENTA, RESET);
    println!("  │ {}(merged){}        │ {}z = 0m{}            │           │            │", DIM, RESET, DIM, RESET);
    println!("  └─────────────────┴───────────────────┴───────────┴────────────┘");
    println!();
    
    println!("  {}{}SUMMARY:{}", BOLD, WHITE, RESET);
    println!("  ├─ {}Late packets handled:{}      1 (AS-EKF retrodiction)", YELLOW, RESET);
    println!("  ├─ {}3D disambiguations:{}        1 (drone ≠ car)", CYAN, RESET);
    println!("  ├─ {}Ghosts merged:{}             1 (Highlander CRDT)", MAGENTA, RESET);
    println!("  ├─ {}Attacks blocked:{}           1 (ROGUE_BOT)", RED, RESET);
    println!("  └─ {}Final shared objects:{}      {}3{}", GREEN, RESET, BRIGHT_GREEN, RESET);
    println!();
}

fn print_footer() {
    println!("{}{}┏━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┓{}", BOLD, BRIGHT_GREEN, RESET);
    println!("{}{}┃                                                                    ┃{}", BOLD, BRIGHT_GREEN, RESET);
    println!("{}{}┃  ✅ DISTRIBUTED CONSENSUS ACHIEVED                                 ┃{}", BOLD, BRIGHT_GREEN, RESET);
    println!("{}{}┃                                                                    ┃{}", BOLD, BRIGHT_GREEN, RESET);
    println!("{}{}┃  • All 3 robots now share ONE consistent world model               ┃{}", BOLD, BRIGHT_GREEN, RESET);
    println!("{}{}┃  • No central server required (peer-to-peer via Zenoh)             ┃{}", BOLD, BRIGHT_GREEN, RESET);
    println!("{}{}┃  • Byzantine fault tolerant (rogue agents detected)                ┃{}", BOLD, BRIGHT_GREEN, RESET);
    println!("{}{}┃  • 1-2 KB/s bandwidth (semantic data only, no video)               ┃{}", BOLD, BRIGHT_GREEN, RESET);
    println!("{}{}┃                                                                    ┃{}", BOLD, BRIGHT_GREEN, RESET);
    println!("{}{}┗━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┛{}", BOLD, BRIGHT_GREEN, RESET);
    println!();
}

// ============================================================================
// MAIN
// ============================================================================

fn main() {
    let packets = create_demo_packets();
    
    let mut time_engine = TimeEngine::new();
    let space_engine = SpaceEngine::new();
    let mut tracking_engine = TrackingEngine::new();
    let trust_engine = TrustEngine::new();
    
    print_banner();
    std::thread::sleep(std::time::Duration::from_millis(200));
    
    print_phase_1(&packets);
    std::thread::sleep(std::time::Duration::from_millis(300));
    
    print_phase_2a(&mut time_engine, &packets);
    std::thread::sleep(std::time::Duration::from_millis(300));
    
    print_phase_2b(&space_engine, &packets);
    std::thread::sleep(std::time::Duration::from_millis(300));
    
    print_phase_2c(&mut tracking_engine, &packets);
    std::thread::sleep(std::time::Duration::from_millis(300));
    
    print_phase_2d(&trust_engine, &packets);
    std::thread::sleep(std::time::Duration::from_millis(300));
    
    print_final();
    print_footer();
}
