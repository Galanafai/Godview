//! Seeded fault injection on outgoing traffic.
//!
//! A real network on localhost almost never loses or reorders packets, so a
//! node can make its own outgoing traffic hostile on purpose: drop it, delay
//! it with jitter (which reorders it), or send it twice. Decisions come from
//! a seeded stream, so a node's fault pattern is reproducible even though
//! the real-time interleaving of processes is not.

use rand::{Rng, SeedableRng};
use rand_chacha::ChaCha8Rng;
use serde::{Deserialize, Serialize};
use std::time::Duration;

/// Fault probabilities and delays for outgoing batches.
#[derive(Debug, Clone, Copy, PartialEq, Default, Serialize, Deserialize)]
pub struct FaultSpec {
    /// Probability a batch is dropped.
    pub loss: f64,
    /// Probability a batch is sent twice (each copy with its own delay).
    pub duplicate: f64,
    /// Base delay before sending (ms).
    pub delay_ms: u64,
    /// Uniform extra delay in `[0, jitter_ms]` (ms).
    pub jitter_ms: u64,
}

impl FaultSpec {
    /// True if no fault is configured.
    pub fn is_clean(&self) -> bool {
        self.loss <= 0.0 && self.duplicate <= 0.0 && self.delay_ms == 0 && self.jitter_ms == 0
    }
}

/// Counters of what the injector did.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
pub struct FaultStats {
    /// Batches offered.
    pub offered: u64,
    /// Batches dropped.
    pub dropped: u64,
    /// Extra copies sent.
    pub duplicated: u64,
    /// Copies sent with a nonzero delay.
    pub delayed: u64,
}

/// The injector.
#[derive(Debug, Clone)]
pub struct FaultInjector {
    spec: FaultSpec,
    rng: ChaCha8Rng,
    stats: FaultStats,
}

impl FaultInjector {
    /// Creates an injector with its own seeded stream.
    pub fn new(spec: FaultSpec, seed: u64) -> Self {
        Self {
            spec,
            rng: ChaCha8Rng::seed_from_u64(seed ^ 0x6661_756c_7473),
            stats: FaultStats::default(),
        }
    }

    /// The send delays for one batch: empty if dropped, two entries if duplicated.
    pub fn plan(&mut self) -> Vec<Duration> {
        self.stats.offered += 1;
        // Draw every number unconditionally so one fault does not shift the others.
        let r_loss: f64 = self.rng.gen();
        let r_dup: f64 = self.rng.gen();
        let jitters: [u64; 2] = [self.jitter(), self.jitter()];
        if r_loss < self.spec.loss {
            self.stats.dropped += 1;
            return Vec::new();
        }
        let copies = if r_dup < self.spec.duplicate {
            self.stats.duplicated += 1;
            2
        } else {
            1
        };
        (0..copies)
            .map(|c| {
                let ms = self.spec.delay_ms + jitters[c];
                if ms > 0 {
                    self.stats.delayed += 1;
                }
                Duration::from_millis(ms)
            })
            .collect()
    }

    /// Counters so far.
    pub fn stats(&self) -> FaultStats {
        self.stats
    }

    fn jitter(&mut self) -> u64 {
        if self.spec.jitter_ms == 0 {
            0
        } else {
            self.rng.gen_range(0..=self.spec.jitter_ms)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn clean_spec_sends_once_immediately() {
        let mut f = FaultInjector::new(FaultSpec::default(), 1);
        for _ in 0..100 {
            assert_eq!(f.plan(), vec![Duration::ZERO]);
        }
        assert_eq!(f.stats().dropped, 0);
    }

    #[test]
    fn rates_are_realized_and_reproducible() {
        let spec = FaultSpec {
            loss: 0.2,
            duplicate: 0.1,
            delay_ms: 10,
            jitter_ms: 40,
        };
        let run = || {
            let mut f = FaultInjector::new(spec, 9);
            let plans: Vec<_> = (0..5000).map(|_| f.plan()).collect();
            (plans, f.stats())
        };
        let (a, s) = run();
        assert_eq!(a, run().0, "same seed, same faults");
        let loss = s.dropped as f64 / s.offered as f64;
        let dup = s.duplicated as f64 / (s.offered - s.dropped) as f64;
        assert!((loss - 0.2).abs() < 0.03, "loss {loss}");
        assert!((dup - 0.1).abs() < 0.03, "dup {dup}");
        assert!(a
            .iter()
            .flatten()
            .all(|d| (10..=50).contains(&(d.as_millis() as u64))));
    }
}
