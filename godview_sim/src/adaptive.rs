//! Adaptive intelligence for learning agents.
//!
//! This module provides adaptive behaviors that allow agents to:
//! - Learn which neighbors provide reliable gossip
//! - Evolve confidence in tracks based on corroboration
//! - Exhibit emergent swarm intelligence behaviors

// Re-export types now defined in godview_core::godview_trust
pub use godview_core::godview_trust::{
    AdaptiveState,
    NeighborReputation,
    TrackConfidence,
    AdaptiveMetrics,
};
