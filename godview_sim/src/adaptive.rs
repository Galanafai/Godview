//! Adaptive intelligence for learning agents.
//!
//! This module provides adaptive behaviors that allow agents to:
//! - Learn which neighbors provide reliable gossip
//! - Evolve confidence in tracks based on corroboration
//! - Exhibit emergent swarm intelligence behaviors

use godview_core::godview_tracking::GlobalHazardPacket;
use std::collections::HashMap;
use uuid::Uuid;

/// Tracks the reliability of a neighbor agent.
#[derive(Debug, Clone)]
pub struct NeighborReputation {
    /// Neighbor agent ID
    pub neighbor_id: usize,
    
    /// Total packets received from this neighbor
    pub packets_received: u64,
    
    /// Packets that led to track updates (useful)
    pub packets_useful: u64,
    
    /// Packets with info we already had (redundant)
    pub packets_redundant: u64,
    
    /// Packets that contradicted local high-confidence data (wrong)
    pub packets_wrong: u64,
    
    /// Computed reliability score (0.0 to 1.0)
    pub reliability_score: f64,
}

impl NeighborReputation {
    /// Creates a new neighbor reputation starting at neutral.
    pub fn new(neighbor_id: usize) -> Self {
        Self {
            neighbor_id,
            packets_received: 0,
            packets_useful: 0,
            packets_redundant: 0,
            packets_wrong: 0,
            reliability_score: 0.5, // Start neutral
        }
    }
    
    /// Records a useful packet and boosts reliability.
    pub fn record_useful(&mut self) {
        self.packets_received += 1;
        self.packets_useful += 1;
        self.reliability_score = (self.reliability_score + 0.01).min(1.0);
    }
    
    /// Records a redundant packet (slight penalty).
    pub fn record_redundant(&mut self) {
        self.packets_received += 1;
        self.packets_redundant += 1;
        self.reliability_score = (self.reliability_score - 0.001).max(0.0);
    }
    
    /// Records a wrong/contradictory packet (major penalty).
    pub fn record_wrong(&mut self) {
        self.packets_received += 1;
        self.packets_wrong += 1;
        self.reliability_score = (self.reliability_score - 0.05).max(0.0);
    }
    
    /// Returns true if this neighbor is considered reliable.
    pub fn is_reliable(&self) -> bool {
        self.reliability_score >= 0.3
    }
    
    /// Returns true if this neighbor is considered a bad actor.
    pub fn is_bad_actor(&self) -> bool {
        self.reliability_score < 0.2 && self.packets_received > 10
    }
}

/// Tracks confidence in a specific track.
#[derive(Debug, Clone)]
pub struct TrackConfidence {
    /// Track ID
    pub track_id: Uuid,
    
    /// Total observations from self
    pub observations: u64,
    
    /// Corroborations from neighbors
    pub corroborations: u64,
    
    /// Contradictions from neighbors
    pub contradictions: u64,
    
    /// Last update time (seconds)
    pub last_update_time: f64,
    
    /// Current confidence (0.0 to 1.0)
    pub confidence: f64,
}

impl TrackConfidence {
    /// Creates a new track confidence starting high (fresh observation).
    pub fn new(track_id: Uuid, current_time: f64) -> Self {
        Self {
            track_id,
            observations: 1,
            corroborations: 0,
            contradictions: 0,
            last_update_time: current_time,
            confidence: 0.8, // Start high for fresh observation
        }
    }
    
    /// Records a new observation and boosts confidence.
    pub fn observe(&mut self, current_time: f64) {
        self.observations += 1;
        self.last_update_time = current_time;
        self.confidence = (self.confidence + 0.05).min(1.0);
    }
    
    /// Records corroboration from a neighbor and boosts confidence.
    pub fn corroborate(&mut self, current_time: f64) {
        self.corroborations += 1;
        self.last_update_time = current_time;
        self.confidence = (self.confidence * 1.10).min(1.0); // +10%
    }
    
    /// Records contradiction from a neighbor and drops confidence.
    pub fn contradict(&mut self) {
        self.contradictions += 1;
        self.confidence *= 0.80; // -20%
    }
    
    /// Applies time decay to confidence.
    pub fn decay(&mut self, current_time: f64, decay_rate: f64) {
        let elapsed = current_time - self.last_update_time;
        if elapsed > 0.0 {
            // Decay by decay_rate per second
            self.confidence *= decay_rate.powf(elapsed);
        }
    }
    
    /// Returns true if confidence is below drop threshold.
    pub fn should_drop(&self) -> bool {
        self.confidence < 0.1
    }
    
    /// Returns true if this is a high-confidence track.
    pub fn is_high_confidence(&self) -> bool {
        self.confidence >= 0.7
    }
}

/// Adaptive state for a learning agent.
#[derive(Debug, Clone)]
pub struct AdaptiveState {
    /// Reputation scores for each neighbor
    pub neighbor_reputations: HashMap<usize, NeighborReputation>,
    
    /// Confidence in each track
    pub track_confidences: HashMap<Uuid, TrackConfidence>,
    
    /// Decay rate per second (0.99 = slow, 0.95 = fast)
    pub confidence_decay_rate: f64,
    
    /// Current simulation time
    pub current_time: f64,
    
    /// Total gossip filtered (didn't process due to low reliability)
    pub gossip_filtered: u64,
    
    /// Total tracks dropped due to low confidence
    pub tracks_dropped: u64,
    
    /// Whether this agent is a "bad actor" (for testing)
    pub is_bad_actor: bool,
}

impl AdaptiveState {
    /// Creates a new adaptive state.
    pub fn new() -> Self {
        Self {
            neighbor_reputations: HashMap::new(),
            track_confidences: HashMap::new(),
            confidence_decay_rate: 0.99, // 1% decay per second
            current_time: 0.0,
            gossip_filtered: 0,
            tracks_dropped: 0,
            is_bad_actor: false,
        }
    }
    
    /// Creates a bad actor state (for testing).
    pub fn new_bad_actor() -> Self {
        let mut state = Self::new();
        state.is_bad_actor = true;
        state
    }
    
    /// Updates the current time and applies decay to all tracks.
    pub fn tick(&mut self, current_time: f64) {
        self.current_time = current_time;
        
        // Decay all track confidences
        for tc in self.track_confidences.values_mut() {
            tc.decay(current_time, self.confidence_decay_rate);
        }
        
        // Drop low-confidence tracks
        let to_drop: Vec<Uuid> = self.track_confidences.iter()
            .filter(|(_, tc)| tc.should_drop())
            .map(|(id, _)| *id)
            .collect();
        
        for id in to_drop {
            self.track_confidences.remove(&id);
            self.tracks_dropped += 1;
        }
    }
    
    /// Gets or creates reputation for a neighbor.
    pub fn get_neighbor(&mut self, neighbor_id: usize) -> &mut NeighborReputation {
        self.neighbor_reputations
            .entry(neighbor_id)
            .or_insert_with(|| NeighborReputation::new(neighbor_id))
    }
    
    /// Gets or creates confidence for a track.
    pub fn get_track(&mut self, track_id: Uuid) -> &mut TrackConfidence {
        let time = self.current_time;
        self.track_confidences
            .entry(track_id)
            .or_insert_with(|| TrackConfidence::new(track_id, time))
    }
    
    /// Decides whether to accept gossip from a neighbor.
    pub fn should_accept_gossip(&self, neighbor_id: usize) -> bool {
        match self.neighbor_reputations.get(&neighbor_id) {
            Some(rep) => rep.is_reliable(),
            None => true, // Accept from unknown neighbors initially
        }
    }
    
    /// Processes incoming gossip and updates reputations.
    ///
    /// Returns true if the gossip was useful, false if redundant/filtered.
    pub fn process_gossip(
        &mut self,
        neighbor_id: usize,
        packet: &GlobalHazardPacket,
        was_useful: bool,
        was_contradictory: bool,
    ) -> bool {
        let rep = self.get_neighbor(neighbor_id);
        
        if was_contradictory {
            rep.record_wrong();
            return false;
        }
        
        if was_useful {
            rep.record_useful();
            
            // Also boost track confidence
            let time = self.current_time;
            let tc = self.get_track(packet.entity_id);
            tc.corroborate(time);
            return true;
        }
        
        rep.record_redundant();
        false
    }
    
    /// Records a direct observation (not from gossip).
    pub fn observe_directly(&mut self, track_id: Uuid) {
        let time = self.current_time;
        let tc = self.get_track(track_id);
        tc.observe(time);
    }
    
    /// Returns metrics for reporting.
    pub fn metrics(&self) -> AdaptiveMetrics {
        let reputations: Vec<f64> = self.neighbor_reputations.values()
            .map(|r| r.reliability_score)
            .collect();
        
        let avg_reliability = if reputations.is_empty() {
            0.0
        } else {
            reputations.iter().sum::<f64>() / reputations.len() as f64
        };
        
        let bad_actors_identified = self.neighbor_reputations.values()
            .filter(|r| r.is_bad_actor())
            .count();
        
        let high_confidence_tracks = self.track_confidences.values()
            .filter(|tc| tc.is_high_confidence())
            .count();
        
        let total_useful: u64 = self.neighbor_reputations.values()
            .map(|r| r.packets_useful)
            .sum();
        let total_received: u64 = self.neighbor_reputations.values()
            .map(|r| r.packets_received)
            .sum();
        let gossip_efficiency = if total_received > 0 {
            total_useful as f64 / total_received as f64
        } else {
            0.0
        };
        
        AdaptiveMetrics {
            avg_neighbor_reliability: avg_reliability,
            bad_actors_identified,
            high_confidence_tracks,
            tracks_dropped: self.tracks_dropped,
            gossip_filtered: self.gossip_filtered,
            gossip_efficiency,
        }
    }
}

impl Default for AdaptiveState {
    fn default() -> Self {
        Self::new()
    }
}

/// Metrics from adaptive intelligence.
#[derive(Debug, Clone, Default)]
pub struct AdaptiveMetrics {
    /// Average reliability score across all neighbors
    pub avg_neighbor_reliability: f64,
    
    /// Number of neighbors identified as bad actors
    pub bad_actors_identified: usize,
    
    /// Number of high-confidence tracks
    pub high_confidence_tracks: usize,
    
    /// Total tracks dropped due to low confidence
    pub tracks_dropped: u64,
    
    /// Total gossip filtered due to low reliability
    pub gossip_filtered: u64,
    
    /// Ratio of useful gossip to total gossip
    pub gossip_efficiency: f64,
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_neighbor_reputation_learning() {
        let mut rep = NeighborReputation::new(0);
        
        // Start neutral
        assert!((rep.reliability_score - 0.5).abs() < 0.001);
        
        // Useful packets boost reliability
        for _ in 0..20 {
            rep.record_useful();
        }
        assert!(rep.reliability_score > 0.6);
        assert!(rep.is_reliable());
        
        // Wrong packets drop it fast
        for _ in 0..20 {
            rep.record_wrong();
        }
        assert!(rep.reliability_score < 0.2);
        assert!(rep.is_bad_actor());
    }
    
    #[test]
    fn test_track_confidence_decay() {
        let mut tc = TrackConfidence::new(Uuid::nil(), 0.0);
        
        // Start high
        assert!(tc.confidence > 0.7);
        
        // Decay over time
        tc.decay(10.0, 0.9); // 10% decay per second
        assert!(tc.confidence < 0.3);
        
        // Should be dropped
        tc.decay(20.0, 0.9);
        assert!(tc.should_drop());
    }
    
    #[test]
    fn test_adaptive_state_gossip_filtering() {
        let mut state = AdaptiveState::new();
        state.current_time = 1.0;
        
        // Unknown neighbor → accept
        assert!(state.should_accept_gossip(0));
        
        // Mark neighbor as bad
        for _ in 0..20 {
            state.get_neighbor(0).record_wrong();
        }
        
        // Bad neighbor → reject
        assert!(!state.should_accept_gossip(0));
    }
}
