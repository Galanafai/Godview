//! Network transport abstraction for GodView agents.

use async_trait::async_trait;
use crate::error::EnvError;
use crate::types::{NodeId, SignedPacketEnvelope};

/// Abstraction for network I/O between GodView agents.
///
/// # Implementations
///
/// - **Production**: Wraps UDP/TCP sockets or ZMQ
/// - **Simulation**: Channel-based with configurable latency/loss
///
/// # Packet Flow
///
/// ```text
/// Agent A                    Network                    Agent B
///   |                           |                          |
///   |-- send(B, packet) ------->|                          |
///   |                           |-- [latency/jitter] ----->|
///   |                           |                          |-- recv() -> packet
/// ```
#[async_trait]
pub trait NetworkTransport: Send + Sync + 'static {
    /// Sends a signed packet to a target node.
    ///
    /// # Arguments
    /// * `target` - The destination node ID
    /// * `packet` - The signed packet envelope to send
    ///
    /// # Returns
    /// * `Ok(())` - Packet queued for delivery
    /// * `Err(EnvError::NetworkError)` - Immediate send failure (e.g., buffer full)
    ///
    /// # Note
    /// Success does not guarantee delivery - packets may be lost/delayed in simulation.
    async fn send(&self, target: NodeId, packet: SignedPacketEnvelope) -> Result<(), EnvError>;
    
    /// Receives the next packet addressed to this node.
    ///
    /// # Returns
    /// * `Some((sender, packet))` - A packet was received
    /// * `None` - The channel was closed (shutdown)
    ///
    /// # Blocking
    /// This method blocks until a packet arrives or the channel closes.
    async fn recv(&self) -> Option<(NodeId, SignedPacketEnvelope)>;
    
    /// Broadcasts a packet to all connected nodes.
    ///
    /// # Arguments
    /// * `packet` - The packet to broadcast
    ///
    /// # Returns
    /// Number of nodes the packet was sent to.
    async fn broadcast(&self, packet: SignedPacketEnvelope) -> usize;
    
    /// Returns this node's ID.
    fn local_id(&self) -> NodeId;
}

/// Marker trait for network controllers in simulation.
///
/// Allows injecting faults like partitions and latency.
pub trait NetworkController: Send + Sync {
    /// Creates a network partition between two node sets.
    fn partition(&self, group_a: &[NodeId], group_b: &[NodeId]);
    
    /// Heals all partitions.
    fn heal_all(&self);
    
    /// Sets latency for a specific link.
    fn set_link_latency(&self, from: NodeId, to: NodeId, latency_ms: u64);
    
    /// Sets packet loss probability for a link (0.0 - 1.0).
    fn set_link_loss(&self, from: NodeId, to: NodeId, loss_rate: f64);
}
