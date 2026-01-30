//! SimulatedAgent - A wrapper that runs GodViewAgent in simulation.
//!
//! This module bridges the `godview_core::GodViewAgent` with the simulation
//! infrastructure, providing:
//! - Automatic sensor reading ingestion from Oracle
//! - Packet processing loop
//! - Metric collection

use crate::context::SimContext;
use crate::network::SimNetwork;

use godview_core::{GodViewAgent, AgentConfig};
use godview_env::NodeId;
use nalgebra::Vector3;
use std::sync::Arc;

/// A simulated agent running in the deterministic environment.
pub struct SimulatedAgent {
    /// The underlying GodView agent
    inner: GodViewAgent<SimContext, SimNetwork>,
    
    /// Agent index (for key derivation)
    agent_index: u64,
    
    /// Last known positions from sensor readings
    last_readings: Vec<(u64, Vector3<f64>)>,
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
            last_readings: Vec::new(),
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
    
    /// Ingests sensor readings from the Oracle.
    ///
    /// In simulation, all agents have perfect visibility of all entities.
    /// This can be made more realistic by adding range/occlusion limits.
    pub fn ingest_readings(&mut self, readings: Vec<(u64, Vector3<f64>)>) {
        self.last_readings = readings;
        
        // TODO: Feed readings into the tracking engine
        // This would call track_manager.process_packet() with synthetic detections
    }
    
    /// Returns the number of tracks currently maintained.
    pub fn track_count(&self) -> usize {
        self.inner.track_manager.tracks().count()
    }
    
    /// Returns the current tick count.
    pub fn tick_count(&self) -> u64 {
        self.inner.tick_count()
    }
    
    /// Returns the current simulation time in seconds.
    pub fn time_secs(&self) -> f64 {
        self.inner.now_secs()
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
}

#[cfg(test)]
mod tests {
    // Tests require full simulation setup, covered in world tests
}
