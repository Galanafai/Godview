//! SimWorld - The simulation harness container.

use crate::context::SimContext;
use crate::keys::DeterministicKeyProvider;
use crate::network::{SimNetwork, SimNetworkController, NetworkMessage};
use crate::oracle::Oracle;

use godview_env::{GodViewContext, NodeId};
use std::collections::HashMap;
use std::sync::Arc;
use tokio::sync::mpsc;

/// Configuration for a simulation run.
#[derive(Debug, Clone)]
pub struct SimConfig {
    /// Master seed for determinism
    pub seed: u64,
    
    /// Number of agents to spawn
    pub num_agents: usize,
    
    /// Tick rate in Hz
    pub tick_rate_hz: u32,
    
    /// Maximum simulation duration in seconds (0 = unlimited)
    pub max_duration_secs: f64,
    
    /// Position noise standard deviation for sensor readings
    pub sensor_noise_std: f64,
}

impl Default for SimConfig {
    fn default() -> Self {
        Self {
            seed: 42,
            num_agents: 6,
            tick_rate_hz: 30,
            max_duration_secs: 60.0,
            sensor_noise_std: 0.5,
        }
    }
}

/// Handle to a simulated agent.
pub struct AgentHandle {
    /// Agent's node ID
    pub id: NodeId,
    
    /// Network interface for this agent
    pub network: Arc<SimNetwork>,
    
    /// Sender to deliver packets to this agent
    pub inbox_tx: mpsc::Sender<(NodeId, godview_env::SignedPacketEnvelope)>,
}

/// The SimWorld - container for the entire simulation.
pub struct SimWorld {
    /// Configuration
    pub config: SimConfig,
    
    /// Shared simulation context (virtual clock)
    pub context: Arc<SimContext>,
    
    /// Key provider (deterministic crypto)
    pub keys: DeterministicKeyProvider,
    
    /// Ground truth oracle
    pub oracle: Oracle,
    
    /// Network controller for fault injection
    pub network_controller: SimNetworkController,
    
    /// Agent handles
    pub agents: HashMap<NodeId, AgentHandle>,
    
    /// Central router sender (receives all outgoing packets)
    router_tx: mpsc::Sender<NetworkMessage>,
    
    /// Central router receiver
    router_rx: mpsc::Receiver<NetworkMessage>,
    
    /// Current tick count
    tick_count: u64,
}

impl SimWorld {
    /// Creates a new SimWorld with the given configuration.
    pub fn new(config: SimConfig) -> Self {
        // Derive separate seeds for different subsystems
        let context_seed = config.seed;
        let physics_seed = config.seed.wrapping_mul(0x9e3779b97f4a7c15);
        let key_seed = config.seed.wrapping_mul(0x517cc1b727220a95);
        
        let context = SimContext::shared(context_seed);
        let keys = DeterministicKeyProvider::new(key_seed);
        
        let mut oracle = Oracle::new(physics_seed);
        oracle.set_position_noise(config.sensor_noise_std);
        
        let network_controller = SimNetworkController::new();
        
        // Create central router channel
        let (router_tx, router_rx) = mpsc::channel::<NetworkMessage>(10000);
        
        Self {
            config,
            context,
            keys,
            oracle,
            network_controller,
            agents: HashMap::new(),
            router_tx,
            router_rx,
            tick_count: 0,
        }
    }
    
    /// Spawns agents and returns their IDs.
    pub fn spawn_agents(&mut self) -> Vec<NodeId> {
        let mut ids = Vec::new();
        
        for i in 0..self.config.num_agents {
            let node_id = NodeId::from_seed(i as u64);
            let (inbox_tx, inbox_rx) = mpsc::channel(1000);
            
            let network = Arc::new(SimNetwork::new(
                node_id,
                self.router_tx.clone(),
                inbox_rx,
            ));
            
            let handle = AgentHandle {
                id: node_id,
                network,
                inbox_tx,
            };
            
            self.agents.insert(node_id, handle);
            ids.push(node_id);
        }
        
        ids
    }
    
    /// Advances simulation by one tick.
    pub fn tick(&mut self) {
        let dt = 1.0 / self.config.tick_rate_hz as f64;
        
        // Advance virtual time
        self.context.advance_time(std::time::Duration::from_secs_f64(dt));
        
        // Advance physics
        self.oracle.step(dt);
        
        self.tick_count += 1;
    }
    
    /// Processes pending network messages (routes packets).
    pub async fn process_network(&mut self) {
        // Drain all pending messages
        while let Ok(msg) = self.router_rx.try_recv() {
            // Check partition
            if !self.network_controller.can_communicate(msg.from, msg.to) {
                continue; // Packet dropped due to partition
            }
            
            // Deliver to recipient
            if let Some(agent) = self.agents.get(&msg.to) {
                let _ = agent.inbox_tx.send((msg.from, msg.packet)).await;
            }
        }
    }
    
    /// Returns the current simulation time in seconds.
    pub fn time(&self) -> f64 {
        self.context.now().as_secs_f64()
    }
    
    /// Returns the current tick count.
    pub fn tick_count(&self) -> u64 {
        self.tick_count
    }
    
    /// Returns the number of active agents.
    pub fn agent_count(&self) -> usize {
        self.agents.len()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_sim_world_creation() {
        let config = SimConfig {
            seed: 42,
            num_agents: 3,
            ..Default::default()
        };
        
        let mut world = SimWorld::new(config);
        let ids = world.spawn_agents();
        
        assert_eq!(ids.len(), 3);
        assert_eq!(world.agent_count(), 3);
    }
    
    #[test]
    fn test_sim_world_tick() {
        let config = SimConfig {
            seed: 42,
            tick_rate_hz: 30,
            ..Default::default()
        };
        
        let mut world = SimWorld::new(config);
        
        assert_eq!(world.tick_count(), 0);
        assert_eq!(world.time(), 0.0);
        
        world.tick();
        
        assert_eq!(world.tick_count(), 1);
        assert!((world.time() - 1.0 / 30.0).abs() < 0.0001);
    }
    
    #[test]
    fn test_sim_world_determinism() {
        let config = SimConfig {
            seed: 42,
            ..Default::default()
        };
        
        let world1 = SimWorld::new(config.clone());
        let world2 = SimWorld::new(config);
        
        // Same seed = same root key
        assert_eq!(
            world1.keys.root_public_key().to_bytes(),
            world2.keys.root_public_key().to_bytes()
        );
    }
}
