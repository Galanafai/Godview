//! Common types for the GodView environment abstraction.

use serde::{Deserialize, Serialize};
use uuid::Uuid;

/// Unique identifier for a GodView node/agent.
///
/// Uses UUID v4 for global uniqueness without coordination.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct NodeId(pub Uuid);

impl NodeId {
    /// Creates a new random NodeId.
    pub fn new() -> Self {
        Self(Uuid::new_v4())
    }
    
    /// Creates a NodeId from a UUID.
    pub fn from_uuid(uuid: Uuid) -> Self {
        Self(uuid)
    }
    
    /// Creates a deterministic NodeId from a seed (for simulation).
    pub fn from_seed(seed: u64) -> Self {
        // Use seed bytes to create a deterministic UUID
        let mut bytes = [0u8; 16];
        bytes[0..8].copy_from_slice(&seed.to_le_bytes());
        bytes[8..16].copy_from_slice(&seed.wrapping_mul(0x517cc1b727220a95).to_le_bytes());
        Self(Uuid::from_bytes(bytes))
    }
    
    /// Returns the inner UUID.
    pub fn as_uuid(&self) -> Uuid {
        self.0
    }
}

impl Default for NodeId {
    fn default() -> Self {
        Self::new()
    }
}

impl std::fmt::Display for NodeId {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        // Show first 8 chars for readability
        write!(f, "{}", &self.0.to_string()[..8])
    }
}

/// Envelope for signed packets transmitted between nodes.
///
/// This is a transport-layer wrapper - the actual packet content
/// is opaque bytes that will be deserialized by the receiving engine.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SignedPacketEnvelope {
    /// The raw signed packet bytes
    pub payload: Vec<u8>,
    
    /// Timestamp when the packet was created (sender's clock)
    pub timestamp_ms: u64,
    
    /// Optional routing hint for multi-hop delivery
    pub hop_count: u8,
}

impl SignedPacketEnvelope {
    /// Creates a new envelope from payload bytes.
    pub fn new(payload: Vec<u8>, timestamp_ms: u64) -> Self {
        Self {
            payload,
            timestamp_ms,
            hop_count: 0,
        }
    }
    
    /// Returns the payload size in bytes.
    pub fn size(&self) -> usize {
        self.payload.len()
    }
}
