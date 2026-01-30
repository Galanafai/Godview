//! Agent Runtime - Orchestrates GodView engines with environment context.
//!
//! This module provides the integration layer between the pure-math engines
//! (time, space, trust, tracking) and the environment abstraction (GodViewContext).
//!
//! # Architecture
//!
//! ```text
//! ┌─────────────────────────────────────────────────────────────┐
//! │                      GodViewAgent                           │
//! │  ┌──────────────────────────────────────────────────────┐   │
//! │  │              Context: GodViewContext                  │   │
//! │  │  • now() → timestamp for engines                     │   │
//! │  │  • sleep() → tick rate control                       │   │
//! │  │  • derive_signing_key() → Trust Engine keys          │   │
//! │  └──────────────────────────────────────────────────────┘   │
//! │                              │                               │
//! │  ┌─────────┐ ┌─────────┐ ┌─────────┐ ┌─────────────────┐   │
//! │  │  TIME   │ │  SPACE  │ │  TRUST  │ │    TRACKING     │   │
//! │  │ Engine  │ │ Engine  │ │ Engine  │ │     Engine      │   │
//! │  └─────────┘ └─────────┘ └─────────┘ └─────────────────┘   │
//! └─────────────────────────────────────────────────────────────┘
//! ```
//!
//! # Usage
//!
//! ```ignore
//! use godview_core::agent_runtime::{GodViewAgent, AgentConfig};
//! use godview_env::TokioContext;
//!
//! let ctx = TokioContext::shared();
//! let config = AgentConfig::default();
//! let agent = GodViewAgent::new(ctx, config);
//!
//! // Run agent loop
//! agent.run().await;
//! ```

use godview_env::{GodViewContext, NodeId, NetworkTransport, SignedPacketEnvelope, EnvError};
use crate::godview_time::AugmentedStateFilter;
use crate::godview_space::SpatialEngine;
use crate::godview_trust::SecurityContext;
use crate::godview_tracking::TrackManager;

use std::sync::Arc;

/// Configuration for a GodView agent.
#[derive(Debug, Clone)]
pub struct AgentConfig {
    /// Agent's logical name (for logging)
    pub name: String,
    
    /// Tick rate in Hz (default: 30)
    pub tick_rate_hz: u32,
    
    /// H3 resolution for spatial indexing (default: 11)
    pub h3_resolution: u8,
    
    /// Maximum OOSM lag depth in ticks (default: 20)
    pub max_lag_depth: usize,
}

impl Default for AgentConfig {
    fn default() -> Self {
        Self {
            name: "godview-agent".to_string(),
            tick_rate_hz: 30,
            h3_resolution: 11,
            max_lag_depth: 20,
        }
    }
}

/// A GodView agent that orchestrates all engines.
///
/// Generic over the context and network implementations,
/// allowing the same agent code to run in production (tokio)
/// or simulation (madsim).
pub struct GodViewAgent<Ctx, Net>
where
    Ctx: GodViewContext,
    Net: NetworkTransport,
{
    /// Node identifier
    pub node_id: NodeId,
    
    /// Environment context
    pub context: Arc<Ctx>,
    
    /// Network interface
    pub network: Arc<Net>,
    
    /// Configuration
    pub config: AgentConfig,
    
    /// Time Engine - OOSM filter
    pub time_engine: AugmentedStateFilter,
    
    /// Space Engine - H3 + 3D indexing
    pub space_engine: SpatialEngine,
    
    /// Trust Engine - CapBAC security
    pub trust_engine: SecurityContext,
    
    /// Tracking Engine - Multi-object tracking
    pub track_manager: TrackManager,
    
    /// Current tick number
    tick_count: u64,
}

impl<Ctx, Net> GodViewAgent<Ctx, Net>
where
    Ctx: GodViewContext,
    Net: NetworkTransport,
{
    /// Creates a new GodView agent with the given context and network.
    pub fn new(
        context: Arc<Ctx>,
        network: Arc<Net>,
        config: AgentConfig,
        root_public_key: biscuit_auth::PublicKey,
    ) -> Self {
        use h3o::Resolution;
        use crate::godview_tracking::TrackingConfig;
        
        // Initialize Time Engine with default state
        let initial_state = nalgebra::DVector::zeros(6); // [pos_x, pos_y, pos_z, vel_x, vel_y, vel_z]
        let initial_cov = nalgebra::DMatrix::identity(6, 6) * 100.0;
        let process_noise = nalgebra::DMatrix::identity(6, 6) * 0.01;
        let measurement_noise = nalgebra::DMatrix::identity(3, 3) * 0.1;
        
        let time_engine = AugmentedStateFilter::new(
            initial_state,
            initial_cov,
            process_noise,
            measurement_noise,
            config.max_lag_depth,
        );
        
        // Initialize Space Engine with H3 resolution
        let resolution = Resolution::try_from(config.h3_resolution)
            .unwrap_or(Resolution::Eleven);
        let space_engine = SpatialEngine::new(resolution);
        
        // Initialize Trust Engine with root key
        let trust_engine = SecurityContext::new(root_public_key);
        
        // Initialize Track Manager with config
        let tracking_config = TrackingConfig {
            h3_resolution: resolution,
            ..TrackingConfig::default()
        };
        let track_manager = TrackManager::new(tracking_config);
        
        Self {
            node_id: network.local_id(),
            context,
            network,
            config,
            time_engine,
            space_engine,
            trust_engine,
            track_manager,
            tick_count: 0,
        }
    }
    
    /// Returns the current simulation time from the context.
    pub fn now_secs(&self) -> f64 {
        self.context.now().as_secs_f64()
    }
    
    /// Returns the current tick count.
    pub fn tick_count(&self) -> u64 {
        self.tick_count
    }
    
    /// Increments the tick counter and returns the new value.
    pub fn tick(&mut self) -> u64 {
        self.tick_count += 1;
        
        // Run Time Engine prediction step
        let dt = 1.0 / self.config.tick_rate_hz as f64;
        let current_time = self.now_secs();
        self.time_engine.predict(dt, current_time);
        
        // Age tracks
        self.track_manager.age_tracks();
        
        self.tick_count
    }
    
    /// Processes an incoming signed packet.
    pub fn process_packet(&mut self, sender: NodeId, envelope: SignedPacketEnvelope) -> Result<(), EnvError> {
        // Validate packet timestamp isn't too old
        let now_ms = self.context.now().as_millis() as u64;
        let packet_age_ms = now_ms.saturating_sub(envelope.timestamp_ms);
        
        if packet_age_ms > 10_000 {
            // Packet older than 10 seconds - discard
            return Ok(());
        }
        
        // TODO: Deserialize packet, verify trust, update tracking
        // This would be expanded in Phase 4.3
        
        let _ = (sender, envelope); // Suppress unused warning for now
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    // Tests will be added when we have a mock network implementation
    #[test]
    fn test_agent_config_default() {
        let config = AgentConfig::default();
        assert_eq!(config.tick_rate_hz, 30);
        assert_eq!(config.h3_resolution, 11);
        assert_eq!(config.max_lag_depth, 20);
    }
}
