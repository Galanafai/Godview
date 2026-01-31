//! Simulated network transport with fault injection.

use async_trait::async_trait;
use godview_env::{EnvError, NetworkTransport, NodeId, SignedPacketEnvelope};
use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use tokio::sync::mpsc;

/// Simulated network interface for an agent.
pub struct SimNetwork {
    /// This node's ID
    local_id: NodeId,
    
    /// Sender to central router
    tx: mpsc::Sender<NetworkMessage>,
    
    /// Receiver for incoming packets (behind tokio mutex for async)
    rx: Arc<tokio::sync::Mutex<mpsc::Receiver<(NodeId, SignedPacketEnvelope)>>>,
}

/// Internal message to the network router.
#[derive(Debug)]
pub struct NetworkMessage {
    pub from: NodeId,
    pub to: NodeId,
    pub packet: SignedPacketEnvelope,
}

impl SimNetwork {
    /// Creates a new simulated network interface.
    pub fn new(
        local_id: NodeId,
        tx: mpsc::Sender<NetworkMessage>,
        rx: mpsc::Receiver<(NodeId, SignedPacketEnvelope)>,
    ) -> Self {
        Self {
            local_id,
            tx,
            rx: Arc::new(tokio::sync::Mutex::new(rx)),
        }
    }
    
    /// Creates a stub network for testing (doesn't actually send/receive).
    pub fn new_stub(local_id: NodeId) -> Self {
        let (tx, _) = mpsc::channel(1);
        let (_, rx) = mpsc::channel(1);
        Self {
            local_id,
            tx,
            rx: Arc::new(tokio::sync::Mutex::new(rx)),
        }
    }
}

#[async_trait]
impl NetworkTransport for SimNetwork {
    async fn send(&self, target: NodeId, packet: SignedPacketEnvelope) -> Result<(), EnvError> {
        let msg = NetworkMessage {
            from: self.local_id,
            to: target,
            packet,
        };
        
        self.tx.send(msg).await.map_err(|_| {
            EnvError::network("Channel closed")
        })
    }
    
    async fn recv(&self) -> Option<(NodeId, SignedPacketEnvelope)> {
        let mut rx = self.rx.lock().await;
        rx.recv().await
    }
    
    async fn broadcast(&self, _packet: SignedPacketEnvelope) -> usize {
        // Broadcasts would be sent to all known peers
        // For now, return 0 as we don't have peer discovery yet
        0
    }
    
    fn local_id(&self) -> NodeId {
        self.local_id
    }
}

/// Network controller for fault injection.
pub struct SimNetworkController {
    /// Per-link latency in milliseconds
    link_latency: Arc<Mutex<HashMap<(NodeId, NodeId), u64>>>,
    
    /// Per-link packet loss rate (0.0 - 1.0)
    link_loss: Arc<Mutex<HashMap<(NodeId, NodeId), f64>>>,
    
    /// Active partitions (nodes that cannot communicate)
    partitions: Arc<Mutex<Vec<(Vec<NodeId>, Vec<NodeId>)>>>,
}

impl SimNetworkController {
    /// Creates a new network controller.
    pub fn new() -> Self {
        Self {
            link_latency: Arc::new(Mutex::new(HashMap::new())),
            link_loss: Arc::new(Mutex::new(HashMap::new())),
            partitions: Arc::new(Mutex::new(Vec::new())),
        }
    }
    
    /// Sets latency for a specific link.
    pub fn set_latency(&self, from: NodeId, to: NodeId, latency_ms: u64) {
        let mut latencies = self.link_latency.lock().unwrap();
        latencies.insert((from, to), latency_ms);
    }
    
    /// Sets packet loss rate for a link.
    pub fn set_loss(&self, from: NodeId, to: NodeId, loss_rate: f64) {
        let mut losses = self.link_loss.lock().unwrap();
        losses.insert((from, to), loss_rate.clamp(0.0, 1.0));
    }
    
    /// Creates a network partition between two groups.
    pub fn partition(&self, group_a: Vec<NodeId>, group_b: Vec<NodeId>) {
        let mut partitions = self.partitions.lock().unwrap();
        partitions.push((group_a, group_b));
    }
    
    /// Heals all active partitions.
    pub fn heal_all(&self) {
        let mut partitions = self.partitions.lock().unwrap();
        partitions.clear();
    }
    
    /// Checks if two nodes can communicate (not partitioned).
    pub fn can_communicate(&self, from: NodeId, to: NodeId) -> bool {
        let partitions = self.partitions.lock().unwrap();
        
        for (group_a, group_b) in partitions.iter() {
            let from_in_a = group_a.contains(&from);
            let from_in_b = group_b.contains(&from);
            let to_in_a = group_a.contains(&to);
            let to_in_b = group_b.contains(&to);
            
            // Partitioned if one is in A and other in B (or vice versa)
            if (from_in_a && to_in_b) || (from_in_b && to_in_a) {
                return false;
            }
        }
        
        true
    }
    
    /// Gets the latency for a link (default 0).
    pub fn get_latency(&self, from: NodeId, to: NodeId) -> u64 {
        let latencies = self.link_latency.lock().unwrap();
        *latencies.get(&(from, to)).unwrap_or(&0)
    }
    
    /// Gets the loss rate for a link (default 0.0).
    pub fn get_loss(&self, from: NodeId, to: NodeId) -> f64 {
        let losses = self.link_loss.lock().unwrap();
        *losses.get(&(from, to)).unwrap_or(&0.0)
    }
}

impl Default for SimNetworkController {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_network_controller_partition() {
        let controller = SimNetworkController::new();
        
        let a = NodeId::from_seed(1);
        let b = NodeId::from_seed(2);
        let c = NodeId::from_seed(3);
        
        // Initially all can communicate
        assert!(controller.can_communicate(a, b));
        assert!(controller.can_communicate(a, c));
        assert!(controller.can_communicate(b, c));
        
        // Partition: {a} vs {b, c}
        controller.partition(vec![a], vec![b, c]);
        
        // Now a cannot talk to b or c
        assert!(!controller.can_communicate(a, b));
        assert!(!controller.can_communicate(a, c));
        
        // But b and c can still talk
        assert!(controller.can_communicate(b, c));
        
        // Heal
        controller.heal_all();
        assert!(controller.can_communicate(a, b));
    }
    
    #[test]
    fn test_network_controller_latency() {
        let controller = SimNetworkController::new();
        
        let a = NodeId::from_seed(1);
        let b = NodeId::from_seed(2);
        
        assert_eq!(controller.get_latency(a, b), 0);
        
        controller.set_latency(a, b, 100);
        assert_eq!(controller.get_latency(a, b), 100);
        
        // Reverse direction is separate
        assert_eq!(controller.get_latency(b, a), 0);
    }
}
