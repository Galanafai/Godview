//! P2P Swarm Network for multi-agent simulation.
//!
//! Simulates gossip-based communication between neighboring agents
//! in an H3 spatial grid.

use godview_core::godview_tracking::GlobalHazardPacket;
use std::collections::HashMap;

/// Represents the P2P network topology for a swarm of agents.
pub struct SwarmNetwork {
    /// Adjacency list: agent_index -> list of neighbor indices
    adjacency: HashMap<usize, Vec<usize>>,
    
    /// Gossip buffer: pending packets per agent
    gossip_buffers: HashMap<usize, Vec<GlobalHazardPacket>>,
    
    /// Total messages sent (for metrics)
    messages_sent: u64,
}

impl SwarmNetwork {
    /// Creates a new swarm network with a grid topology.
    ///
    /// Agents are arranged in a `rows x cols` grid where each agent
    /// can communicate with its 4-8 neighbors (depending on position).
    pub fn new_grid(rows: usize, cols: usize) -> Self {
        let mut adjacency = HashMap::new();
        
        for row in 0..rows {
            for col in 0..cols {
                let idx = row * cols + col;
                let mut neighbors = Vec::new();
                
                // Add neighbors (up, down, left, right, diagonals)
                for dr in -1i32..=1 {
                    for dc in -1i32..=1 {
                        if dr == 0 && dc == 0 {
                            continue;
                        }
                        
                        let nr = row as i32 + dr;
                        let nc = col as i32 + dc;
                        
                        if nr >= 0 && nr < rows as i32 && nc >= 0 && nc < cols as i32 {
                            let neighbor_idx = (nr as usize) * cols + (nc as usize);
                            neighbors.push(neighbor_idx);
                        }
                    }
                }
                
                adjacency.insert(idx, neighbors);
            }
        }
        
        // Initialize empty gossip buffers for all agents
        let gossip_buffers = (0..rows * cols)
            .map(|i| (i, Vec::new()))
            .collect();
        
        Self {
            adjacency,
            gossip_buffers,
            messages_sent: 0,
        }
    }
    
    /// Returns the neighbors of an agent.
    pub fn neighbors(&self, agent_idx: usize) -> &[usize] {
        self.adjacency.get(&agent_idx).map(|v| v.as_slice()).unwrap_or(&[])
    }
    
    /// Queues a packet for gossip to neighbors.
    pub fn queue_gossip(&mut self, from_agent: usize, packet: GlobalHazardPacket) {
        if let Some(neighbors) = self.adjacency.get(&from_agent) {
            for &neighbor in neighbors {
                if let Some(buffer) = self.gossip_buffers.get_mut(&neighbor) {
                    buffer.push(packet.clone());
                    self.messages_sent += 1;
                }
            }
        }
    }
    
    /// Takes all pending gossip for an agent (drains the buffer).
    pub fn take_gossip(&mut self, agent_idx: usize) -> Vec<GlobalHazardPacket> {
        self.gossip_buffers
            .get_mut(&agent_idx)
            .map(|b| std::mem::take(b))
            .unwrap_or_default()
    }
    
    /// Returns the total number of messages sent.
    pub fn messages_sent(&self) -> u64 {
        self.messages_sent
    }
    
    /// Returns the total number of agents in the network.
    pub fn agent_count(&self) -> usize {
        self.adjacency.len()
    }
}

/// Configuration for the swarm scenario.
#[derive(Debug, Clone)]
pub struct SwarmConfig {
    /// Number of agent rows in grid
    pub rows: usize,
    
    /// Number of agent columns in grid
    pub cols: usize,
    
    /// Number of entities to simulate
    pub num_entities: usize,
    
    /// Simulation duration in seconds
    pub duration_secs: f64,
    
    /// Tick rate (Hz)
    pub tick_rate_hz: usize,
    
    /// Gossip interval (every N ticks)
    pub gossip_interval: usize,
    
    /// Maximum acceptable entity count variance
    pub max_variance: f64,
    
    /// Maximum acceptable position error (meters)
    pub max_position_error: f64,
}

impl Default for SwarmConfig {
    fn default() -> Self {
        Self {
            rows: 5,
            cols: 10,
            num_entities: 200,
            duration_secs: 30.0,
            tick_rate_hz: 30,
            gossip_interval: 3, // Gossip every 3 ticks (~10 Hz)
            max_variance: 0.15,  // 15% CV allowed (partial visibility causes variance)
            max_position_error: 3.0,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_grid_topology() {
        let network = SwarmNetwork::new_grid(3, 3);
        
        // Center agent (1,1) = index 4 should have 8 neighbors
        assert_eq!(network.neighbors(4).len(), 8);
        
        // Corner agent (0,0) = index 0 should have 3 neighbors
        assert_eq!(network.neighbors(0).len(), 3);
        
        // Edge agent (0,1) = index 1 should have 5 neighbors
        assert_eq!(network.neighbors(1).len(), 5);
    }
    
    #[test]
    fn test_gossip_delivery() {
        let mut network = SwarmNetwork::new_grid(2, 2);
        
        let packet = GlobalHazardPacket {
            entity_id: uuid::Uuid::nil(),
            position: [0.0, 0.0, 0.0],
            velocity: [0.0, 0.0, 0.0],
            class_id: 1,
            timestamp: 0.0,
            confidence_score: 0.9,
        };
        
        // Agent 0 gossips
        network.queue_gossip(0, packet.clone());
        
        // Neighbors should receive it (agents 1, 2, 3 in a 2x2 grid)
        assert_eq!(network.take_gossip(1).len(), 1);
        assert_eq!(network.take_gossip(2).len(), 1);
        assert_eq!(network.take_gossip(3).len(), 1);
        
        // Agent 0 shouldn't receive its own gossip
        assert_eq!(network.take_gossip(0).len(), 0);
        
        assert_eq!(network.messages_sent(), 3);
    }
}
