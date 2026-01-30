//! Core environment context trait for GodView agents.

use async_trait::async_trait;
use ed25519_dalek::SigningKey;
use std::future::Future;
use std::time::{Duration, SystemTime};

/// The central interface for Environment Interaction.
///
/// This trait abstracts the "real world" so that GodView engines can run
/// in both production (tokio) and simulation (madsim) environments.
///
/// # Implementations
///
/// - **Production**: `TokioContext` - wraps `tokio::time`, `OsRng`
/// - **Simulation**: `SimContext` - wraps `madsim::time`, `StdRng(seed)`
///
/// # Determinism
///
/// For DST, all methods that would normally introduce non-determinism
/// (time, randomness) are controlled by the implementation.
#[async_trait]
pub trait GodViewContext: Send + Sync + 'static {
    /// Returns the current monotonic time since context creation.
    ///
    /// Used for internal timers and duration measurements.
    /// In simulation, this is the virtual clock time.
    fn now(&self) -> Duration;
    
    /// Returns the wall-clock time for packet timestamps.
    ///
    /// Critical for validating OOSM logic in `godview_time`.
    /// In simulation, this is derived from virtual clock + epoch offset.
    fn system_time(&self) -> SystemTime;
    
    /// Suspends execution for the given duration.
    ///
    /// In production: wraps `tokio::time::sleep`
    /// In simulation: advances virtual clock
    async fn sleep(&self, duration: Duration);
    
    /// Spawns a background task.
    ///
    /// In production: `tokio::spawn`
    /// In simulation: `madsim::spawn`
    fn spawn<F>(&self, name: &str, future: F)
    where
        F: Future<Output = ()> + Send + 'static;
    
    /// Generates a deterministic keypair from a seed extension.
    ///
    /// Essential for testing `godview_trust` without depleting OS entropy.
    /// The implementation combines the global seed with `seed_extension`
    /// to derive unique but reproducible keys.
    ///
    /// # Arguments
    /// * `seed_extension` - A value to combine with the global seed
    fn derive_signing_key(&self, seed_extension: u64) -> SigningKey;
    
    /// Returns the context's seed (for logging/debugging).
    ///
    /// In production, returns 0 (not seeded).
    /// In simulation, returns the master seed.
    fn seed(&self) -> u64;
}
