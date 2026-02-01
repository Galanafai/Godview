//! SimulatedAgent - A wrapper that runs GodViewAgent in simulation.
//!
//! This module bridges the `godview_core::GodViewAgent` with the simulation
//! infrastructure, providing:
//! - Automatic sensor reading ingestion from Oracle
//! - Packet processing loop
//! - Metric collection
//! - Adaptive learning (neighbor reputation, track confidence)

use crate::adaptive::AdaptiveState;
use crate::evolution::{EvolutionaryState, FitnessProvider, OracleFitness};
use crate::context::SimContext;
use crate::network::SimNetwork;
use crate::oracle::SensorReading;

use godview_core::{GodViewAgent, AgentConfig};
use godview_core::godview_tracking::GlobalHazardPacket;
use godview_env::NodeId;
use nalgebra::Vector3;
use std::sync::Arc;
use uuid::Uuid;
use rand::SeedableRng;
use rand_chacha::ChaCha8Rng;

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
    
    /// Adaptive intelligence state (learning)
    adaptive: AdaptiveState,
    
    /// Evolutionary state (parameter adaptation)
    evolution: EvolutionaryState,
    
    /// Strategy for calculating fitness
    fitness_provider: Box<dyn FitnessProvider>,

    /// RNG for evolutionary decisions
    rng: ChaCha8Rng,
    
    /// Current energy level (Joules)
    energy: f64,
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
        let rng = ChaCha8Rng::seed_from_u64(agent_index.wrapping_mul(0xeb0123));
        
        Self {
            inner,
            agent_index,
            entity_track_map: std::collections::HashMap::new(),
            readings_processed: 0,
            tracks_created: 0,
            recent_packets: Vec::new(),
            gossip_received: 0,
            adaptive: AdaptiveState::new(),
            evolution: EvolutionaryState::new(),
            fitness_provider: Box::new(OracleFitness::new()), // Default to Oracle
            rng,
            energy: 1000.0, // 1000 Joules capacity
        }
    }
    
    /// Creates a new simulated agent configured as a bad actor (for testing).
    pub fn new_bad_actor(
        context: Arc<SimContext>,
        network: Arc<SimNetwork>,
        root_public_key: biscuit_auth::PublicKey,
        agent_index: u64,
        config: AgentConfig,
    ) -> Self {
        let mut agent = Self::new(context, network, root_public_key, agent_index, config);
        agent.adaptive = AdaptiveState::new_bad_actor();
        agent
    }
    
    /// Sets the fitness provider for this agent (e.g. to switch to BlindFitness).
    pub fn set_fitness_provider(&mut self, provider: Box<dyn FitnessProvider>) {
        self.fitness_provider = provider;
    }
    
    /// Returns the agent's node ID.
    pub fn node_id(&self) -> NodeId {
        self.inner.node_id
    }
    
    /// Consumes energy if available. Returns true if agent is alive (energy > 0).
    pub fn consume_energy(&mut self, amount: f64) -> bool {
        if self.energy > 0.0 {
            self.energy -= amount;
            if self.energy < 0.0 {
                self.energy = 0.0;
            }
        }
        self.energy > 0.0
    }
    
    /// Returns true if the agent has energy remaining.
    pub fn is_alive(&self) -> bool {
        self.energy > 0.0
    }
    
    /// Processes a single tick - updates filters, ages tracks, and decays confidence.
    pub fn tick(&mut self) -> bool {
        // Idle cost
        if !self.consume_energy(0.01) {
            return false; // Dead
        }

        self.inner.tick();
        
        // Update adaptive state with current time
        let current_time = self.inner.now_secs();
        self.adaptive.tick(current_time);
        
        true
    }
    
    /// Runs a tick of the evolutionary process.
    ///
    /// # Arguments
    /// * `epoch_length_ticks` - How long each evolutionary epoch lasts
    /// * `ground_truth` - (Optional) Ground truth for calculating error signal
    ///
    /// Returns true if parameters were updated (epoch ended).
    pub fn tick_evolution(&mut self, epoch_length_ticks: u64, ground_truth: Option<&[(u64, Vector3<f64>)]>) -> bool {
        // Collect metrics for this tick
        
        // 1. Accuracy (if GT available)
        let error = if let Some(gt) = ground_truth {
            self.compute_position_error(gt)
        } else {
            0.0 // No error data if no GT
        };
        
        // 2. Consistency (NIS) via Time Engine
        // Assumption: avg_nis is available from the filter
        let nis = self.inner.time_engine.get_average_nis();
        
        // 3. Consensus (Peer Agreement) via Tracking Engine
        let pa_cost = self.inner.track_manager.get_peer_agreement_cost();
        
        // Record all raw metrics to evolutionary state
        self.evolution.record_metrics(error, nis, pa_cost, self.energy);
        
        // Check if epoch should end
        if self.inner.tick_count() % epoch_length_ticks == 0 {
            // Calculate fitness using the configured provider
            // The state handles aggregating average metrics from the recorded sums
            self.evolution.evolve(&mut self.rng, self.fitness_provider.as_ref());
            return true;
        }
        false
    }
    
    /// Ingests sensor readings from the Oracle and processes through TrackManager.
    ///
    /// Converts each reading into a GlobalHazardPacket and processes it
    /// through the full tracking pipeline (association, fusion, Highlander).
    pub fn ingest_readings(&mut self, readings: &[SensorReading]) {
        // Sensor/CPU cost
        self.consume_energy(0.05 * readings.len() as f64);

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
            
            // Save for gossip (subject to evolution params?)
            // For now, always save, but gossip logic determines sending frequency
            self.recent_packets.push(packet.clone());
            
            // Process through TrackManager
            // Local readings: No adaptive state or neighbor ID needed
            match self.inner.track_manager.process_packet(&packet, None, None) {
                Ok(_track_id) => {
                    self.readings_processed += 1;
                }
                Err(e) => {
                    tracing::debug!("Track processing error: {:?}", e);
                }
            }
        }
    }
    
    /// Receives gossip packets from neighbors and processes them with learning.
    ///
    /// Tracks which neighbors provide useful vs redundant/wrong data.
    pub fn receive_gossip_from(&mut self, neighbor_id: usize, packets: &[GlobalHazardPacket]) {
        // Check if we should accept gossip from this neighbor
        if !self.adaptive.should_accept_gossip(neighbor_id) {
            self.adaptive.gossip_filtered += packets.len() as u64;
            return;
        }
        
        for packet in packets {
            // Apply evolutionary confidence threshold
            if packet.confidence_score < self.evolution.current_params.confidence_threshold {
                continue;
            }

            self.gossip_received += 1;
            
            // Check if we already have this track with high confidence
            let existing_confidence = self.adaptive.track_confidences
                .get(&packet.entity_id)
                .map(|tc| tc.confidence)
                .unwrap_or(0.0);
            
            // Process through TrackManager
            // Gossip: Pass adaptive state and neighbor ID for peer agreement tracking
            let was_useful = match self.inner.track_manager.process_packet(
                packet, 
                Some(&self.adaptive), 
                Some(neighbor_id)
            ) {
                Ok(_) => existing_confidence < 0.5, // Useful if we didn't have it
                Err(_) => false,
            };
            
            // Update neighbor reputation
            self.adaptive.process_gossip(
                neighbor_id,
                packet,
                was_useful,
                false, // TODO: detect contradictions
            );
        }
    }
    
    /// Legacy receive_gossip without neighbor tracking (for backward compat).
    pub fn receive_gossip(&mut self, packets: &[GlobalHazardPacket]) {
        // Use a dummy neighbor ID for non-tracked gossip
        self.receive_gossip_from(usize::MAX, packets);
    }
    
    /// Returns recent packets for sharing (since last clear).
    pub fn recent_packets(&self) -> &[GlobalHazardPacket] {
        &self.recent_packets
    }
    
    /// Clears recent packets after gossip round.
    pub fn clear_recent_packets(&mut self) {
        self.recent_packets.clear();
    }
    
    /// Records a message sent metric for evolution.
    pub fn record_message_sent_metric(&mut self, bytes_sent: u64) {
        self.evolution.record_message_sent(bytes_sent);
    }
    
    /// Returns current gossip interval in ticks (evolved).
    pub fn gossip_interval(&self) -> u64 {
        self.evolution.current_params.gossip_interval_ticks
    }
    
    /// Emergency Protocol (v0.6.0): Returns whether the agent should broadcast.
    /// Returns false if energy is critically low (< 50J) to prevent messaging death.
    pub fn should_broadcast(&self, current_tick: u64) -> bool {
        // Emergency Protocol: Enter conservation mode at 50J
        if self.energy < 50.0 {
            return false;
        }
        // Normal gossip interval check
        current_tick % self.gossip_interval() == 0
    }
    
    /// Returns max gossip neighbors (evolved).
    pub fn max_gossip_neighbors(&self) -> usize {
        self.evolution.current_params.max_neighbors_gossip
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
    
    /// Returns whether this agent is configured as a bad actor.
    pub fn is_bad_actor(&self) -> bool {
        self.adaptive.is_bad_actor
    }
    
    /// Returns adaptive learning metrics.
    pub fn adaptive_metrics(&self) -> crate::adaptive::AdaptiveMetrics {
        self.adaptive.metrics()
    }
    
    /// Returns a reference to the adaptive state.
    pub fn adaptive_state(&self) -> &AdaptiveState {
        &self.adaptive
    }
    
    /// Returns a reference to the evolutionary state.
    pub fn evolutionary_state(&self) -> &EvolutionaryState {
        &self.evolution
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
