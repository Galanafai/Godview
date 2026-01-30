//! Simulation context implementing GodViewContext for deterministic testing.

use async_trait::async_trait;
use ed25519_dalek::SigningKey;
use godview_env::GodViewContext;
use rand::SeedableRng;
use rand_chacha::ChaCha8Rng;
use std::sync::{Arc, Mutex};
use std::time::{Duration, SystemTime, UNIX_EPOCH};

/// Simulation context backed by deterministic time and RNG.
///
/// This implements `GodViewContext` using:
/// - A virtual clock that can be advanced manually
/// - A seeded ChaCha8 RNG for deterministic key generation
/// - Simulated sleep that advances virtual time
pub struct SimContext {
    /// Master seed for this simulation
    seed: u64,
    
    /// Current virtual time (nanoseconds since simulation start)
    virtual_time_ns: Arc<Mutex<u64>>,
    
    /// Deterministic RNG for crypto operations
    rng: Arc<Mutex<ChaCha8Rng>>,
    
    /// Epoch offset (virtual time 0 maps to this wall-clock time)
    epoch: SystemTime,
}

impl SimContext {
    /// Creates a new SimContext with the given seed.
    pub fn new(seed: u64) -> Self {
        Self {
            seed,
            virtual_time_ns: Arc::new(Mutex::new(0)),
            rng: Arc::new(Mutex::new(ChaCha8Rng::seed_from_u64(seed))),
            epoch: UNIX_EPOCH + Duration::from_secs(1704067200), // 2024-01-01 00:00:00 UTC
        }
    }
    
    /// Creates an Arc-wrapped context for sharing.
    pub fn shared(seed: u64) -> Arc<Self> {
        Arc::new(Self::new(seed))
    }
    
    /// Advances virtual time by the given duration.
    pub fn advance_time(&self, duration: Duration) {
        let mut time = self.virtual_time_ns.lock().unwrap();
        *time += duration.as_nanos() as u64;
    }
    
    /// Sets the virtual time to a specific value.
    pub fn set_time(&self, time_ns: u64) {
        let mut time = self.virtual_time_ns.lock().unwrap();
        *time = time_ns;
    }
    
    /// Returns the current virtual time in nanoseconds.
    pub fn time_ns(&self) -> u64 {
        *self.virtual_time_ns.lock().unwrap()
    }
}

impl Clone for SimContext {
    fn clone(&self) -> Self {
        Self {
            seed: self.seed,
            virtual_time_ns: Arc::clone(&self.virtual_time_ns),
            rng: Arc::clone(&self.rng),
            epoch: self.epoch,
        }
    }
}

#[async_trait]
impl GodViewContext for SimContext {
    fn now(&self) -> Duration {
        Duration::from_nanos(*self.virtual_time_ns.lock().unwrap())
    }
    
    fn system_time(&self) -> SystemTime {
        self.epoch + self.now()
    }
    
    async fn sleep(&self, duration: Duration) {
        // In simulation, sleep advances virtual time
        // In a full madsim integration, this would yield to the scheduler
        self.advance_time(duration);
    }
    
    fn spawn<F>(&self, name: &str, future: F)
    where
        F: std::future::Future<Output = ()> + Send + 'static,
    {
        let _name = name.to_string();
        // In full madsim, this would use madsim::spawn
        // For now, use tokio::spawn as a fallback
        tokio::spawn(async move {
            future.await;
        });
    }
    
    fn derive_signing_key(&self, seed_extension: u64) -> SigningKey {
        // Combine master seed with extension for deterministic key
        let combined_seed = self.seed.wrapping_mul(0x517cc1b727220a95) ^ seed_extension;
        let mut key_rng = ChaCha8Rng::seed_from_u64(combined_seed);
        SigningKey::generate(&mut key_rng)
    }
    
    fn seed(&self) -> u64 {
        self.seed
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_sim_context_time() {
        let ctx = SimContext::new(42);
        assert_eq!(ctx.now(), Duration::ZERO);
        
        ctx.advance_time(Duration::from_secs(1));
        assert_eq!(ctx.now(), Duration::from_secs(1));
        
        ctx.advance_time(Duration::from_millis(500));
        assert_eq!(ctx.now(), Duration::from_millis(1500));
    }
    
    #[test]
    fn test_sim_context_deterministic_keys() {
        let ctx1 = SimContext::new(42);
        let ctx2 = SimContext::new(42);
        
        let key1 = ctx1.derive_signing_key(1);
        let key2 = ctx2.derive_signing_key(1);
        
        // Same seed + extension = same key
        assert_eq!(key1.to_bytes(), key2.to_bytes());
        
        // Different extension = different key
        let key3 = ctx1.derive_signing_key(2);
        assert_ne!(key1.to_bytes(), key3.to_bytes());
    }
    
    #[test]
    fn test_sim_context_seed() {
        let ctx = SimContext::new(12345);
        assert_eq!(ctx.seed(), 12345);
    }
    
    #[test]
    fn test_sim_context_clone_shares_time() {
        let ctx1 = SimContext::new(42);
        let ctx2 = ctx1.clone();
        
        ctx1.advance_time(Duration::from_secs(5));
        
        // Both should see the same time
        assert_eq!(ctx1.now(), ctx2.now());
    }
}
