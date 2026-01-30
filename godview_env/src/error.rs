//! Error types for the GodView environment abstraction.

use thiserror::Error;

/// Errors that can occur in the environment abstraction layer.
#[derive(Debug, Error)]
pub enum EnvError {
    /// Network send failed (buffer full, connection closed, etc.)
    #[error("Network error: {0}")]
    NetworkError(String),
    
    /// Target node is unreachable (simulated partition)
    #[error("Node unreachable: {0}")]
    NodeUnreachable(String),
    
    /// Packet serialization/deserialization failed
    #[error("Serialization error: {0}")]
    SerializationError(String),
    
    /// Context operation failed
    #[error("Context error: {0}")]
    ContextError(String),
    
    /// Operation timed out
    #[error("Timeout after {0}ms")]
    Timeout(u64),
}

impl EnvError {
    /// Creates a network error.
    pub fn network(msg: impl Into<String>) -> Self {
        Self::NetworkError(msg.into())
    }
    
    /// Creates an unreachable error.
    pub fn unreachable(node: impl std::fmt::Display) -> Self {
        Self::NodeUnreachable(node.to_string())
    }
}
