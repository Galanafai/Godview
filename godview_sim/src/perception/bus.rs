//! A deterministic, fault-injecting message bus for perception DST.
//!
//! Messages are opaque byte strings (signed report batches), exactly what a
//! real transport carries. Faults are applied to bytes in flight:
//!
//! * per-link latency plus uniform jitter (jitter reorders messages),
//! * loss, duplication and single-byte corruption with given probabilities,
//! * partitions and per-link overrides via [`SimNetworkController`].
//!
//! All randomness comes from one seeded ChaCha stream and delivery is ordered
//! by `(delivery time, send sequence)`, so a run is reproducible from its seed.

use crate::network::SimNetworkController;
use godview_env::{NodeId, Timestamp};
use rand::{Rng, SeedableRng};
use rand_chacha::ChaCha8Rng;
use std::collections::{BTreeMap, BTreeSet};

/// Default link behavior.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LinkFaults {
    /// Base one-way latency (ms).
    pub latency_ms: u64,
    /// Uniform extra delay in `[0, jitter_ms]` (ms).
    pub jitter_ms: u64,
    /// Probability a message is dropped.
    pub loss: f64,
    /// Probability a message is delivered twice (second copy with its own jitter).
    pub duplicate: f64,
    /// Probability one byte of a message is flipped in flight.
    pub corrupt: f64,
}

impl Default for LinkFaults {
    fn default() -> Self {
        Self {
            latency_ms: 20,
            jitter_ms: 0,
            loss: 0.0,
            duplicate: 0.0,
            corrupt: 0.0,
        }
    }
}

/// Who can talk to whom (before partitions).
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Topology {
    /// Every agent reaches every other agent.
    FullMesh,
    /// Agent `i` reaches `i - 1` and `i + 1` only.
    Line,
}

/// A message that has arrived.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Delivery {
    /// Sending agent index.
    pub from: usize,
    /// Receiving agent index.
    pub to: usize,
    /// Payload bytes (possibly corrupted).
    pub bytes: Vec<u8>,
}

/// Counters of what the bus did.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct BusStats {
    /// Messages handed to the bus (per destination).
    pub sent: u64,
    /// Dropped by loss.
    pub lost: u64,
    /// Dropped by a partition or missing link.
    pub blocked: u64,
    /// Extra copies created.
    pub duplicated: u64,
    /// Messages corrupted.
    pub corrupted: u64,
    /// Messages delivered.
    pub delivered: u64,
    /// Deliveries that arrived after a later-sent message on the same link.
    pub reordered: u64,
}

/// A queued message: sender, receiver, send sequence, payload.
type Queued = (usize, usize, u64, Vec<u8>);

/// The deterministic bus.
pub struct SimBus {
    nodes: Vec<NodeId>,
    topology: Topology,
    faults: LinkFaults,
    controller: SimNetworkController,
    rng: ChaCha8Rng,
    queue: BTreeMap<(i64, u64), Queued>,
    next_seq: u64,
    last_delivered: BTreeMap<(usize, usize), u64>,
    stats: BusStats,
    lossless: bool,
}

impl SimBus {
    /// Creates a bus for `n` agents.
    pub fn new(n: usize, topology: Topology, faults: LinkFaults, seed: u64) -> Self {
        Self {
            nodes: (0..n as u64).map(NodeId::from_seed).collect(),
            topology,
            faults,
            controller: SimNetworkController::new(),
            rng: ChaCha8Rng::seed_from_u64(seed),
            queue: BTreeMap::new(),
            next_seq: 0,
            last_delivered: BTreeMap::new(),
            stats: BusStats::default(),
            lossless: false,
        }
    }

    /// Partition and per-link fault controls.
    pub fn controller(&self) -> &SimNetworkController {
        &self.controller
    }

    /// Node id of agent `i` (for the controller).
    pub fn node(&self, i: usize) -> NodeId {
        self.nodes[i]
    }

    /// Counters.
    pub fn stats(&self) -> BusStats {
        self.stats
    }

    /// Disables loss, duplication and corruption (used for the final
    /// synchronization round, where convergence assumes reliable delivery).
    pub fn set_lossless(&mut self, lossless: bool) {
        self.lossless = lossless;
    }

    /// Neighbors of agent `i` under the topology.
    pub fn neighbors(&self, i: usize) -> Vec<usize> {
        match self.topology {
            Topology::FullMesh => (0..self.nodes.len()).filter(|j| *j != i).collect(),
            Topology::Line => [i.checked_sub(1), Some(i + 1)]
                .into_iter()
                .flatten()
                .filter(|j| *j < self.nodes.len())
                .collect(),
        }
    }

    /// Sends `bytes` from `from` to each neighbor except those in `skip`.
    pub fn broadcast(&mut self, from: usize, bytes: &[u8], now: Timestamp, skip: &BTreeSet<usize>) {
        for to in self.neighbors(from) {
            if !skip.contains(&to) {
                self.send(from, to, bytes.to_vec(), now);
            }
        }
    }

    /// Sends one message, applying faults.
    pub fn send(&mut self, from: usize, to: usize, bytes: Vec<u8>, now: Timestamp) {
        self.stats.sent += 1;
        let (a, b) = (self.nodes[from], self.nodes[to]);
        if !self.controller.can_communicate(a, b) {
            self.stats.blocked += 1;
            return;
        }
        let loss = self.controller.get_loss(a, b).max(self.faults.loss);
        // Draw every random number unconditionally so that toggling one fault
        // does not shift the stream for the others.
        let r_loss: f64 = self.rng.gen();
        let r_dup: f64 = self.rng.gen();
        let r_cor: f64 = self.rng.gen();
        let r_pos: usize = self.rng.gen();
        let r_bit: u8 = self.rng.gen_range(0..8);
        if !self.lossless && r_loss < loss {
            self.stats.lost += 1;
            return;
        }
        let mut payload = bytes;
        if !self.lossless && r_cor < self.faults.corrupt && !payload.is_empty() {
            let i = r_pos % payload.len();
            payload[i] ^= 1 << r_bit;
            self.stats.corrupted += 1;
        }
        let copies = if !self.lossless && r_dup < self.faults.duplicate {
            self.stats.duplicated += 1;
            2
        } else {
            1
        };
        let link_latency = self
            .controller
            .get_latency(a, b)
            .max(self.faults.latency_ms);
        for _ in 0..copies {
            let jitter = if self.faults.jitter_ms > 0 {
                self.rng.gen_range(0..=self.faults.jitter_ms)
            } else {
                0
            };
            let at = now.add_millis((link_latency + jitter) as i64).as_nanos();
            let seq = self.next_seq;
            self.next_seq += 1;
            self.queue
                .insert((at, seq), (from, to, seq, payload.clone()));
        }
    }

    /// Queues a message for delivery at `at` with no faults and without
    /// touching the bus RNG.
    ///
    /// Used for adversarial injections, so that a run with an adversary and
    /// its control run draw exactly the same network randomness.
    pub fn inject(&mut self, from: usize, to: usize, bytes: Vec<u8>, at: Timestamp) {
        let seq = self.next_seq;
        self.next_seq += 1;
        self.queue
            .insert((at.as_nanos(), seq), (from, to, seq, bytes));
    }

    /// Removes and returns every message due at or before `now`, in delivery order.
    pub fn deliver_until(&mut self, now: Timestamp) -> Vec<Delivery> {
        let mut out = Vec::new();
        while let Some((&(at, seq), _)) = self.queue.first_key_value() {
            if at > now.as_nanos() {
                break;
            }
            let (from, to, sent_seq, bytes) = self.queue.remove(&(at, seq)).expect("present");
            let last = self.last_delivered.entry((from, to)).or_insert(0);
            if sent_seq < *last {
                self.stats.reordered += 1;
            }
            *last = (*last).max(sent_seq);
            self.stats.delivered += 1;
            out.push(Delivery { from, to, bytes });
        }
        out
    }

    /// True if nothing is in flight.
    pub fn is_idle(&self) -> bool {
        self.queue.is_empty()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn same_seed_same_deliveries() {
        let run = || {
            let mut bus = SimBus::new(
                3,
                Topology::FullMesh,
                LinkFaults {
                    latency_ms: 10,
                    jitter_ms: 50,
                    loss: 0.2,
                    duplicate: 0.2,
                    corrupt: 0.1,
                },
                9,
            );
            for k in 0..50 {
                bus.broadcast(
                    k % 3,
                    &[k as u8; 8],
                    Timestamp::from_millis(k as i64 * 5),
                    &BTreeSet::new(),
                );
            }
            bus.deliver_until(Timestamp::from_millis(10_000))
        };
        let a = run();
        assert_eq!(a, run());
        assert!(!a.is_empty());
    }

    #[test]
    fn jitter_reorders_and_faults_are_counted() {
        let mut bus = SimBus::new(
            2,
            Topology::FullMesh,
            LinkFaults {
                latency_ms: 10,
                jitter_ms: 80,
                loss: 0.1,
                duplicate: 0.1,
                corrupt: 0.0,
            },
            3,
        );
        for k in 0..200 {
            bus.send(0, 1, vec![k as u8], Timestamp::from_millis(k as i64 * 5));
        }
        let d = bus.deliver_until(Timestamp::from_millis(10_000));
        let s = bus.stats();
        assert!(s.reordered > 10, "jitter must reorder: {s:?}");
        assert!(s.lost > 5 && s.duplicated > 5);
        assert_eq!(d.len() as u64, s.delivered);
        assert_eq!(s.delivered, s.sent - s.lost + s.duplicated);
    }

    #[test]
    fn partitions_block_and_line_topology_limits_reach() {
        let mut bus = SimBus::new(3, Topology::Line, LinkFaults::default(), 1);
        assert_eq!(bus.neighbors(0), vec![1]);
        assert_eq!(bus.neighbors(1), vec![0, 2]);
        bus.controller()
            .partition(vec![bus.node(0)], vec![bus.node(1)]);
        bus.send(0, 1, vec![1], Timestamp::ZERO);
        assert_eq!(bus.stats().blocked, 1);
        bus.controller().heal_all();
        bus.send(0, 1, vec![1], Timestamp::ZERO);
        assert_eq!(bus.deliver_until(Timestamp::from_millis(100)).len(), 1);
    }

    #[test]
    fn lossless_mode_disables_faults() {
        let mut bus = SimBus::new(
            2,
            Topology::FullMesh,
            LinkFaults {
                loss: 1.0,
                duplicate: 1.0,
                corrupt: 1.0,
                ..Default::default()
            },
            1,
        );
        bus.set_lossless(true);
        bus.send(0, 1, vec![7, 7], Timestamp::ZERO);
        assert_eq!(
            bus.deliver_until(Timestamp::from_millis(100))[0].bytes,
            vec![7, 7]
        );
    }
}
