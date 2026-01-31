//! SimulatedAgent - A wrapper that runs GodViewAgent in simulation.
//!
//! This module bridges the `godview_core::GodViewAgent` with the simulation
//! infrastructure, providing:
//! - Automatic sensor reading ingestion from Oracle
//! - Packet processing loop
//! - Metric collection

use crate::context::SimContext;
use crate::network::SimNetwork;
use crate::oracle::SensorReading;

use godview_core::{GodViewAgent, AgentConfig};
use godview_core::godview_tracking::GlobalHazardPacket;
use godview_env::NodeId;
use nalgebra::Vector3;
use std::sync::Arc;
use uuid::Uuid;

/// A simulated agent running in the deterministic environment.
pub struct SimulatedAgent {
    /// The underlying GodView agent
    inner: GodViewAgent<SimContext, SimNetwork>,
    
    /// Agent index (for key derivation)
    agent_index: u64,
    
    /// Tracks created by this agent (entity_id -> track_id)
    entity_track_map: std::collections::HashMap<u64, Uuid>,
    
    /// Metrics: total readings processed
    readings_processed: u64,
    
    /// Metrics: total tracks created
    tracks_created: u64,
    
    /// Recent packets for gossip (cleared each round)
    recent_packets: Vec<GlobalHazardPacket>,
    
    /// Total gossip packets received
    gossip_received: u64,
}

impl SimulatedAgent {
    /// Creates a new simulated agent.
    ///
    /// # Arguments
    /// * `context` - Simulation context
    /// * `network` - Network interface
    /// * `root_public_key` - Root authority public key (from biscuit_auth)
    /// * `agent_index` - Index for identification
    /// * `config` - Agent configuration
    pub fn new(
        context: Arc<SimContext>,
        network: Arc<SimNetwork>,
        root_public_key: biscuit_auth::PublicKey,
        agent_index: u64,
        config: AgentConfig,
    ) -> Self {
        let inner = GodViewAgent::new(context, network, config, root_public_key);
        
        Self {
            inner,
            agent_index,
            entity_track_map: std::collections::HashMap::new(),
            readings_processed: 0,
            tracks_created: 0,
            recent_packets: Vec::new(),
            gossip_received: 0,
        }
    }
    
    /// Returns the agent's node ID.
    pub fn node_id(&self) -> NodeId {
        self.inner.node_id
    }
    
    /// Processes a single tick - updates filters and ages tracks.
    pub fn tick(&mut self) {
        self.inner.tick();
    }
    
    /// Ingests sensor readings from the Oracle and processes through TrackManager.
    ///
    /// Converts each reading into a GlobalHazardPacket and processes it
    /// through the full tracking pipeline (association, fusion, Highlander).
    pub fn ingest_readings(&mut self, readings: &[SensorReading]) {
        let current_time = self.inner.now_secs();
        
        for reading in readings {
            // Convert sensor reading to GlobalHazardPacket
            let packet = GlobalHazardPacket {
                entity_id: self.get_or_create_entity_uuid(reading.entity_id),
                position: [reading.position.x, reading.position.y, reading.position.z],
                velocity: [reading.velocity.x, reading.velocity.y, reading.velocity.z],
                class_id: 4, // Drone class
                timestamp: current_time,
                confidence_score: 0.95,
            };
            
            // Save for gossip
            self.recent_packets.push(packet.clone());
            
            // Process through TrackManager
            match self.inner.track_manager.process_packet(&packet) {
                Ok(_track_id) => {
                    self.readings_processed += 1;
                }
                Err(e) => {
                    tracing::debug!("Track processing error: {:?}", e);
                }
            }
        }
    }
    
    /// Receives gossip packets from neighbors and processes them.
    pub fn receive_gossip(&mut self, packets: &[GlobalHazardPacket]) {
        for packet in packets {
            self.gossip_received += 1;
            
            // Process through TrackManager (from neighbor's observation)
            match self.inner.track_manager.process_packet(packet) {
                Ok(_) => {}
                Err(e) => {
                    tracing::trace!("Gossip packet error: {:?}", e);
                }
            }
        }
    }
    
    /// Returns recent packets for sharing (since last clear).
    pub fn recent_packets(&self) -> &[GlobalHazardPacket] {
        &self.recent_packets
    }
    
    /// Clears recent packets after gossip round.
    pub fn clear_recent_packets(&mut self) {
        self.recent_packets.clear();
    }
    
    /// Returns total gossip packets received.
    pub fn gossip_received(&self) -> u64 {
        self.gossip_received
    }
    
    /// Gets or creates a deterministic UUID for an entity.
    fn get_or_create_entity_uuid(&mut self, entity_id: u64) -> Uuid {
        if let Some(&uuid) = self.entity_track_map.get(&entity_id) {
            return uuid;
        }
        
        // Create deterministic UUID from agent index and entity ID
        let bytes: [u8; 16] = {
            let mut b = [0u8; 16];
            b[0..8].copy_from_slice(&self.agent_index.to_le_bytes());
            b[8..16].copy_from_slice(&entity_id.to_le_bytes());
            b
        };
        let uuid = Uuid::from_bytes(bytes);
        self.entity_track_map.insert(entity_id, uuid);
        self.tracks_created += 1;
        uuid
    }
    
    /// Returns the number of tracks currently maintained.
    pub fn track_count(&self) -> usize {
        self.inner.track_manager.tracks().count()
    }
    
    /// Returns all current track positions.
    pub fn track_positions(&self) -> Vec<(Uuid, Vector3<f64>)> {
        self.inner.track_manager.tracks()
            .map(|t| (t.canonical_id, t.position()))
            .collect()
    }
    
    /// Returns the current tick count.
    pub fn tick_count(&self) -> u64 {
        self.inner.tick_count()
    }
    
    /// Returns the current simulation time in seconds.
    pub fn time_secs(&self) -> f64 {
        self.inner.now_secs()
    }
    
    /// Returns total readings processed.
    pub fn readings_processed(&self) -> u64 {
        self.readings_processed
    }
    
    /// Returns total unique entities seen.
    pub fn unique_entities(&self) -> usize {
        self.entity_track_map.len()
    }
    
    /// Returns a reference to the inner agent.
    pub fn inner(&self) -> &GodViewAgent<SimContext, SimNetwork> {
        &self.inner
    }
    
    /// Returns a mutable reference to the inner agent.
    pub fn inner_mut(&mut self) -> &mut GodViewAgent<SimContext, SimNetwork> {
        &mut self.inner
    }
    
    /// Returns the agent index.
    pub fn agent_index(&self) -> u64 {
        self.agent_index
    }
    
    /// Computes position error against ground truth.
    pub fn compute_position_error(&self, ground_truth: &[(u64, Vector3<f64>)]) -> f64 {
        let mut total_error = 0.0;
        let mut count = 0;
        
        for (entity_id, true_pos) in ground_truth {
            if let Some(&track_uuid) = self.entity_track_map.get(entity_id) {
                // Find the track with this UUID
                for track in self.inner.track_manager.tracks() {
                    if track.canonical_id == track_uuid {
                        let estimated_pos = track.position();
                        let error = (estimated_pos - true_pos).norm();
                        total_error += error * error;
                        count += 1;
                        break;
                    }
                }
            }
        }
        
        if count > 0 {
            (total_error / count as f64).sqrt() // RMS error
        } else {
            0.0
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::keys::DeterministicKeyProvider;
    
    #[test]
    fn test_agent_uuid_generation() {
        let context = Arc::new(SimContext::new(42));
        let network = Arc::new(SimNetwork::new_stub(NodeId::from_seed(0)));
        let key_provider = DeterministicKeyProvider::new(42);
        let root_key = key_provider.biscuit_root_key().public();
        
        let mut agent = SimulatedAgent::new(
            context,
            network,
            root_key,
            0,
            AgentConfig::default(),
        );
        
        // Same entity ID should give same UUID
        let uuid1 = agent.get_or_create_entity_uuid(100);
        let uuid2 = agent.get_or_create_entity_uuid(100);
        assert_eq!(uuid1, uuid2);
        
        // Different entity IDs should give different UUIDs
        let uuid3 = agent.get_or_create_entity_uuid(200);
        assert_ne!(uuid1, uuid3);
    }
    
    #[test]
    fn test_agent_reading_ingestion() {
        let context = Arc::new(SimContext::new(42));
        let network = Arc::new(SimNetwork::new_stub(NodeId::from_seed(0)));
        let key_provider = DeterministicKeyProvider::new(42);
        let root_key = key_provider.biscuit_root_key().public();
        
        let mut agent = SimulatedAgent::new(
            context,
            network,
            root_key,
            0,
            AgentConfig::default(),
        );
        
        // Create some test readings
        let readings = vec![
            SensorReading {
                entity_id: 1,
                position: Vector3::new(37.7749, -122.4194, 100.0),
                velocity: Vector3::new(1.0, 0.0, 0.0),
            },
            SensorReading {
                entity_id: 2,
                position: Vector3::new(37.7750, -122.4195, 105.0),
                velocity: Vector3::new(0.0, 1.0, 0.0),
            },
        ];
        
        agent.ingest_readings(&readings);
        
        assert_eq!(agent.readings_processed(), 2);
        assert_eq!(agent.unique_entities(), 2);
        assert!(agent.track_count() >= 1); // At least some tracks created
    }
}
