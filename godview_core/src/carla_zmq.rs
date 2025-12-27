//! CARLA ZMQ Receiver - Zero-Copy Telemetry Ingestion
//!
//! High-performance ZeroMQ subscriber for CARLA simulation data.
//! Uses zero-copy deserialization via the zerocopy crate.
//!
//! Based on architectural specifications in cara_sim_imp.md:
//! - Zero-copy binary deserialization (instant regardless of actor count)
//! - ZMQ SUB with CONFLATE (always process latest data)
//! - Simulation time synchronization for Rerun

use std::collections::HashMap;
use std::time::Duration;
use thiserror::Error;

/// Binary packet header (matches Python HEADER_FORMAT)
/// Layout: [frame_id: u64, timestamp: f64, actor_count: u32, padding: u32]
#[repr(C, packed)]
#[derive(Debug, Clone, Copy)]
pub struct PacketHeader {
    pub frame_id: u64,
    pub timestamp: f64,  // Simulation time in seconds
    pub actor_count: u32,
    pub _padding: u32,
}

impl PacketHeader {
    pub const SIZE: usize = 24;  // 8 + 8 + 4 + 4 bytes
    
    /// Parse header from bytes (safe version)
    pub fn from_bytes(data: &[u8]) -> Option<Self> {
        if data.len() < Self::SIZE {
            return None;
        }
        
        // Manual parsing for safety (packed struct alignment issues)
        let frame_id = u64::from_le_bytes(data[0..8].try_into().ok()?);
        let timestamp = f64::from_le_bytes(data[8..16].try_into().ok()?);
        let actor_count = u32::from_le_bytes(data[16..20].try_into().ok()?);
        let _padding = u32::from_le_bytes(data[20..24].try_into().ok()?);
        
        Some(Self {
            frame_id,
            timestamp,
            actor_count,
            _padding,
        })
    }
}

/// Per-actor update (matches Python ACTOR_DTYPE)
/// Layout: { id: u32, pos: [f32; 3], rot: [f32; 3], vel: [f32; 3] }
#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
pub struct ActorUpdate {
    pub id: u32,
    pub pos: [f32; 3],  // [x, y, z] in CARLA coordinates
    pub rot: [f32; 3],  // [pitch, yaw, roll] in degrees
    pub vel: [f32; 3],  // [vx, vy, vz] in m/s
}

impl ActorUpdate {
    pub const SIZE: usize = 40;  // 4 + 12 + 12 + 12 bytes
    
    /// Parse actor update from bytes (zero-copy intent)
    pub fn from_bytes(data: &[u8]) -> Option<Self> {
        if data.len() < Self::SIZE {
            return None;
        }
        
        let id = u32::from_le_bytes(data[0..4].try_into().ok()?);
        
        let pos = [
            f32::from_le_bytes(data[4..8].try_into().ok()?),
            f32::from_le_bytes(data[8..12].try_into().ok()?),
            f32::from_le_bytes(data[12..16].try_into().ok()?),
        ];
        
        let rot = [
            f32::from_le_bytes(data[16..20].try_into().ok()?),
            f32::from_le_bytes(data[20..24].try_into().ok()?),
            f32::from_le_bytes(data[24..28].try_into().ok()?),
        ];
        
        let vel = [
            f32::from_le_bytes(data[28..32].try_into().ok()?),
            f32::from_le_bytes(data[32..36].try_into().ok()?),
            f32::from_le_bytes(data[36..40].try_into().ok()?),
        ];
        
        Some(Self { id, pos, rot, vel })
    }
    
    /// Parse multiple actors from contiguous buffer
    pub fn parse_batch(data: &[u8], count: usize) -> Vec<Self> {
        let mut actors = Vec::with_capacity(count);
        
        for i in 0..count {
            let start = i * Self::SIZE;
            let end = start + Self::SIZE;
            
            if end > data.len() {
                break;
            }
            
            if let Some(actor) = Self::from_bytes(&data[start..end]) {
                actors.push(actor);
            }
        }
        
        actors
    }
}

/// Complete telemetry packet from CARLA
#[derive(Debug, Clone)]
pub struct TelemetryPacket {
    pub header: PacketHeader,
    pub actors: Vec<ActorUpdate>,
}

impl TelemetryPacket {
    /// Parse complete packet from binary data
    pub fn from_bytes(data: &[u8]) -> Option<Self> {
        let header = PacketHeader::from_bytes(data)?;
        
        let actor_data_start = PacketHeader::SIZE;
        let actor_data = &data[actor_data_start..];
        
        let actors = ActorUpdate::parse_batch(actor_data, header.actor_count as usize);
        
        Some(Self { header, actors })
    }
}

/// Actor metadata (received on spawn events)
#[derive(Debug, Clone)]
pub struct ActorMetadata {
    pub actor_id: u32,
    pub actor_type: String,  // "vehicle", "pedestrian"
    pub model: String,       // "vehicle.tesla.model3"
    pub color: [u8; 4],      // RGBA
}

/// Errors for ZMQ receiver
#[derive(Error, Debug)]
pub enum CarlaZmqError {
    #[error("ZMQ error: {0}")]
    Zmq(String),
    
    #[error("Parse error: {0}")]
    Parse(String),
    
    #[error("Connection timeout")]
    Timeout,
}

/// ZMQ-based CARLA telemetry receiver
/// 
/// Connects to the Python zmq_bridge.py publisher and receives
/// high-frequency actor state updates.
#[cfg(feature = "carla")]
pub struct CarlaZmqReceiver {
    telemetry_socket: zmq::Socket,
    metadata_socket: zmq::Socket,
    _context: zmq::Context,
    
    // Actor registry
    pub known_actors: HashMap<u32, ActorMetadata>,
    
    // Latest state
    pub last_packet: Option<TelemetryPacket>,
}

#[cfg(feature = "carla")]
impl CarlaZmqReceiver {
    /// Create new receiver connected to zmq_bridge.py
    pub fn new(telemetry_port: u16, metadata_port: u16) -> Result<Self, CarlaZmqError> {
        let context = zmq::Context::new();
        
        // Telemetry subscriber (high-frequency binary)
        let telemetry_socket = context.socket(zmq::SUB)
            .map_err(|e| CarlaZmqError::Zmq(e.to_string()))?;
        
        telemetry_socket.set_conflate(true)  // Keep only latest
            .map_err(|e| CarlaZmqError::Zmq(e.to_string()))?;
        telemetry_socket.set_rcvtimeo(100)  // 100ms timeout
            .map_err(|e| CarlaZmqError::Zmq(e.to_string()))?;
        telemetry_socket.connect(&format!("tcp://127.0.0.1:{}", telemetry_port))
            .map_err(|e| CarlaZmqError::Zmq(e.to_string()))?;
        telemetry_socket.set_subscribe(b"")  // Subscribe to all
            .map_err(|e| CarlaZmqError::Zmq(e.to_string()))?;
        
        // Metadata subscriber (low-frequency JSON)
        let metadata_socket = context.socket(zmq::SUB)
            .map_err(|e| CarlaZmqError::Zmq(e.to_string()))?;
        metadata_socket.connect(&format!("tcp://127.0.0.1:{}", metadata_port))
            .map_err(|e| CarlaZmqError::Zmq(e.to_string()))?;
        metadata_socket.set_subscribe(b"spawn")
            .map_err(|e| CarlaZmqError::Zmq(e.to_string()))?;
        
        Ok(Self {
            telemetry_socket,
            metadata_socket,
            _context: context,
            known_actors: HashMap::new(),
            last_packet: None,
        })
    }
    
    /// Receive latest telemetry packet (non-blocking with ZMQ_CONFLATE)
    pub fn receive_telemetry(&mut self) -> Result<Option<TelemetryPacket>, CarlaZmqError> {
        match self.telemetry_socket.recv_bytes(zmq::DONTWAIT) {
            Ok(data) => {
                let packet = TelemetryPacket::from_bytes(&data)
                    .ok_or_else(|| CarlaZmqError::Parse("Invalid packet format".into()))?;
                self.last_packet = Some(packet.clone());
                Ok(Some(packet))
            }
            Err(zmq::Error::EAGAIN) => Ok(None),  // No message available
            Err(e) => Err(CarlaZmqError::Zmq(e.to_string())),
        }
    }
    
    /// Check for new actor spawn metadata (non-blocking)
    pub fn receive_metadata(&mut self) -> Result<Option<ActorMetadata>, CarlaZmqError> {
        match self.metadata_socket.recv_multipart(zmq::DONTWAIT) {
            Ok(parts) => {
                if parts.len() >= 2 {
                    let json_data: serde_json::Value = serde_json::from_slice(&parts[1])
                        .map_err(|e| CarlaZmqError::Parse(e.to_string()))?;
                    
                    let metadata = ActorMetadata {
                        actor_id: json_data["actor_id"].as_u64().unwrap_or(0) as u32,
                        actor_type: json_data["actor_type"].as_str().unwrap_or("unknown").to_string(),
                        model: json_data["model"].as_str().unwrap_or("unknown").to_string(),
                        color: [
                            json_data["color"][0].as_u64().unwrap_or(128) as u8,
                            json_data["color"][1].as_u64().unwrap_or(128) as u8,
                            json_data["color"][2].as_u64().unwrap_or(128) as u8,
                            json_data["color"][3].as_u64().unwrap_or(255) as u8,
                        ],
                    };
                    
                    self.known_actors.insert(metadata.actor_id, metadata.clone());
                    Ok(Some(metadata))
                } else {
                    Ok(None)
                }
            }
            Err(zmq::Error::EAGAIN) => Ok(None),
            Err(e) => Err(CarlaZmqError::Zmq(e.to_string())),
        }
    }
}

// ============================================================================
// NON-ZMQ FALLBACK (when carla feature is disabled)
// ============================================================================

/// Mock receiver for testing without ZMQ
#[cfg(not(feature = "carla"))]
pub struct CarlaZmqReceiver {
    pub known_actors: HashMap<u32, ActorMetadata>,
    pub last_packet: Option<TelemetryPacket>,
}

#[cfg(not(feature = "carla"))]
impl CarlaZmqReceiver {
    pub fn new(_telemetry_port: u16, _metadata_port: u16) -> Result<Self, CarlaZmqError> {
        Ok(Self {
            known_actors: HashMap::new(),
            last_packet: None,
        })
    }
    
    pub fn receive_telemetry(&mut self) -> Result<Option<TelemetryPacket>, CarlaZmqError> {
        Ok(None)
    }
    
    pub fn receive_metadata(&mut self) -> Result<Option<ActorMetadata>, CarlaZmqError> {
        Ok(None)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_header_parsing() {
        // Create test header bytes matching Python's HEADER_FORMAT = '<QdII'
        let mut data = vec![0u8; 24];
        
        // frame_id = 42 (u64, little-endian)
        data[0..8].copy_from_slice(&42u64.to_le_bytes());
        // timestamp = 1.5 (f64, little-endian)
        data[8..16].copy_from_slice(&1.5f64.to_le_bytes());
        // actor_count = 3 (u32, little-endian)
        data[16..20].copy_from_slice(&3u32.to_le_bytes());
        // padding = 0 (u32, little-endian)
        data[20..24].copy_from_slice(&0u32.to_le_bytes());
        
        let header = PacketHeader::from_bytes(&data).unwrap();
        
        // Copy fields to avoid packed struct alignment issues
        let frame_id = { header.frame_id };
        let timestamp = { header.timestamp };
        let actor_count = { header.actor_count };
        
        assert_eq!(frame_id, 42);
        assert!((timestamp - 1.5).abs() < 0.001);
        assert_eq!(actor_count, 3);
    }
    
    #[test]
    fn test_actor_update_parsing() {
        // Create test actor data matching Python's ACTOR_DTYPE
        // Layout: id(u32) + pos([f32;3]) + rot([f32;3]) + vel([f32;3]) = 40 bytes
        let mut data = vec![0u8; 40];
        
        // id = 100
        data[0..4].copy_from_slice(&100u32.to_le_bytes());
        // pos = [1.0, 2.0, 3.0] - CARLA world coordinates
        data[4..8].copy_from_slice(&1.0f32.to_le_bytes());
        data[8..12].copy_from_slice(&2.0f32.to_le_bytes());
        data[12..16].copy_from_slice(&3.0f32.to_le_bytes());
        // rot = [10.0, 20.0, 30.0] - pitch, yaw, roll in degrees
        data[16..20].copy_from_slice(&10.0f32.to_le_bytes());
        data[20..24].copy_from_slice(&20.0f32.to_le_bytes());
        data[24..28].copy_from_slice(&30.0f32.to_le_bytes());
        // vel = [0.1, 0.2, 0.3] - velocity in m/s
        data[28..32].copy_from_slice(&0.1f32.to_le_bytes());
        data[32..36].copy_from_slice(&0.2f32.to_le_bytes());
        data[36..40].copy_from_slice(&0.3f32.to_le_bytes());
        
        let actor = ActorUpdate::from_bytes(&data).unwrap();
        
        assert_eq!(actor.id, 100);
        assert!((actor.pos[0] - 1.0).abs() < 0.001);
        assert!((actor.pos[1] - 2.0).abs() < 0.001);
        assert!((actor.pos[2] - 3.0).abs() < 0.001);
    }
    
    #[test]
    fn test_full_packet_parsing() {
        // Test parsing a complete CARLA telemetry packet
        // Simulates data received from zmq_bridge.py
        let header_size = 24;
        let actor_size = 40;
        let num_actors = 2;
        
        let mut data = vec![0u8; header_size + actor_size * num_actors];
        
        // Header
        data[0..8].copy_from_slice(&1u64.to_le_bytes());  // frame_id = 1
        data[8..16].copy_from_slice(&0.5f64.to_le_bytes());  // timestamp = 0.5s
        data[16..20].copy_from_slice(&(num_actors as u32).to_le_bytes());  // actor_count = 2
        
        // Actor 0: ID = 1
        let actor0_start = header_size;
        data[actor0_start..actor0_start+4].copy_from_slice(&1u32.to_le_bytes());
        
        // Actor 1: ID = 2
        let actor1_start = header_size + actor_size;
        data[actor1_start..actor1_start+4].copy_from_slice(&2u32.to_le_bytes());
        
        let packet = TelemetryPacket::from_bytes(&data).unwrap();
        
        // Copy header fields to avoid packed struct alignment issues
        let frame_id = { packet.header.frame_id };
        
        assert_eq!(frame_id, 1);
        assert_eq!(packet.actors.len(), 2);
        assert_eq!(packet.actors[0].id, 1);
        assert_eq!(packet.actors[1].id, 2);
    }
}

