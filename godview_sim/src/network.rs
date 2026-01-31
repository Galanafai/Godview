//! Simulated network transport with fault injection.

use async_trait::async_trait;
use godview_env::{EnvError, NetworkTransport, NodeId, SignedPacketEnvelope};
use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use std::sync::atomic::{AtomicU64, Ordering};
use tokio::sync::mpsc;

/// Simulated network interface for an agent.
pub struct SimNetwork {
    /// This node's ID
    local_id: NodeId,
    
    /// Sender to central router
    tx: mpsc::Sender<NetworkMessage>,
    
    /// Receiver for incoming packets (behind tokio mutex for async)
    rx: Arc<tokio::sync::Mutex<mpsc::Receiver<(NodeId, SignedPacketEnvelope)>>>,

    /// Metric: Total bytes sent
    pub bytes_sent: Arc<AtomicU64>,

    /// Metric: Total packets sent
    pub packets_sent: Arc<AtomicU64>,
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
            bytes_sent: Arc::new(AtomicU64::new(0)),
            packets_sent: Arc::new(AtomicU64::new(0)),
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
            bytes_sent: Arc::new(AtomicU64::new(0)),
            packets_sent: Arc::new(AtomicU64::new(0)),
        }
    }
    /// Returns current bandwidth usage (bytes_sent, packets_sent).
    pub fn get_bandwidth_usage(&self) -> (u64, u64) {
        (
            self.bytes_sent.load(Ordering::Relaxed),
            self.packets_sent.load(Ordering::Relaxed),
        )
    }
}

#[async_trait]
impl NetworkTransport for SimNetwork {
    async fn send(&self, target: NodeId, packet: SignedPacketEnvelope) -> Result<(), EnvError> {
        let msg = NetworkMessage {
            from: self.local_id,
            to: target,
            packet: packet.clone(),
        };
        
        // Track metrics
        // Estimate wire size: payload + overhead (timestamp=8 + hop=1 + envelope_struct~=16)
        let wire_size = packet.payload.len() as u64 + 25;
        self.bytes_sent.fetch_add(wire_size, Ordering::Relaxed);
        self.packets_sent.fetch_add(1, Ordering::Relaxed);

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
    
    #[tokio::test]
    async fn test_bandwidth_tracking() {
        let (tx, _rx) = mpsc::channel(10);
        let (_, rx_dummy) = mpsc::channel(10);
        
        let network = SimNetwork::new(
            NodeId::new(),
            tx,
            rx_dummy,
        );
        
        // Initial state
        let (bytes, packets) = network.get_bandwidth_usage();
        assert_eq!(bytes, 0);
        assert_eq!(packets, 0);
        
        // Send a packet
        let payload = vec![1, 2, 3, 4, 5]; // 5 bytes
        let packet = SignedPacketEnvelope::new(payload, 100);
        
        // Wire size estimate: 5 (payload) + 25 (overhead) = 30
        network.send(NodeId::new(), packet).await.unwrap();
        
        let (bytes, packets) = network.get_bandwidth_usage();
        assert_eq!(bytes, 30);
        assert_eq!(packets, 1);
        
        // Send another
        let payload2 = vec![0; 100]; // 100 bytes
        let packet2 = SignedPacketEnvelope::new(payload2, 200);
        
        // Wire size: 100 + 25 = 125
        network.send(NodeId::new(), packet2).await.unwrap();
        
        let (bytes, packets) = network.get_bandwidth_usage();
        assert_eq!(bytes, 30 + 125);
        assert_eq!(packets, 2);
    }
}
