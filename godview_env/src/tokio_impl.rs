//! Production implementation of GodViewContext using Tokio.

use crate::GodViewContext;
use async_trait::async_trait;
use ed25519_dalek::SigningKey;
use std::sync::Arc;
use std::time::{Duration, Instant, SystemTime};

/// Production context backed by Tokio and OS entropy.
///
/// This is the "real" implementation used in production deployments.
/// Time comes from the system clock, randomness from OsRng.
pub struct TokioContext {
    /// Start time for monotonic duration calculations
    start: Instant,
}

impl TokioContext {
    /// Creates a new TokioContext.
    pub fn new() -> Self {
        Self {
            start: Instant::now(),
        }
    }
    
    /// Creates an Arc-wrapped context for sharing across tasks.
    pub fn shared() -> Arc<Self> {
        Arc::new(Self::new())
    }
}

impl Default for TokioContext {
    fn default() -> Self {
        Self::new()
    }
}

#[async_trait]
impl GodViewContext for TokioContext {
    fn now(&self) -> Duration {
        self.start.elapsed()
    }
    
    fn system_time(&self) -> SystemTime {
        SystemTime::now()
    }
    
    async fn sleep(&self, duration: Duration) {
        tokio::time::sleep(duration).await;
    }
    
    fn spawn<F>(&self, name: &str, future: F)
    where
        F: std::future::Future<Output = ()> + Send + 'static,
    {
        let _name = name.to_string(); // Would be used for tracing
        tokio::spawn(async move {
            future.await;
        });
    }
    
    fn derive_signing_key(&self, _seed_extension: u64) -> SigningKey {
        // In production, generate a truly random key
        use rand::rngs::OsRng;
        SigningKey::generate(&mut OsRng)
    }
    
    fn seed(&self) -> u64 {
        // Production is not seeded
        0
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[tokio::test]
    async fn test_tokio_context_time() {
        let ctx = TokioContext::new();
        let t1 = ctx.now();
        ctx.sleep(Duration::from_millis(10)).await;
        let t2 = ctx.now();
        
        assert!(t2 > t1);
        assert!(t2 - t1 >= Duration::from_millis(10));
    }
    
    #[tokio::test]
    async fn test_tokio_context_keypair() {
        let ctx = TokioContext::new();
        let key1 = ctx.derive_signing_key(1);
        let key2 = ctx.derive_signing_key(1);
        
        // In production, keys should be different (random)
        assert_ne!(key1.to_bytes(), key2.to_bytes());
    }
    
    #[test]
    fn test_tokio_context_seed() {
        let ctx = TokioContext::new();
        assert_eq!(ctx.seed(), 0);
    }
}
