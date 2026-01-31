//! Deterministic key provider for simulation.

use ed25519_dalek::{SigningKey, VerifyingKey};
use rand::SeedableRng;
use rand_chacha::ChaCha8Rng;
use std::collections::HashMap;

/// Provides deterministic Ed25519 keys derived from seeds.
///
/// In simulation, we need reproducible keys for each agent.
/// This provider generates keys that are:
/// - Deterministic: Same seed always produces same keys
/// - Unique: Each agent gets a different key
/// - Isolated: Changing agent count doesn't affect other agents' keys
pub struct DeterministicKeyProvider {
    /// Master seed
    master_seed: u64,
    
    /// Cache of generated keys by agent ID
    key_cache: HashMap<u64, SigningKey>,
    
    /// Root key for the simulation (for Trust Engine)
    root_key: SigningKey,
}

impl DeterministicKeyProvider {
    /// Creates a new key provider with the given master seed.
    pub fn new(master_seed: u64) -> Self {
        // Generate root key from master seed
        let mut rng = ChaCha8Rng::seed_from_u64(master_seed);
        let root_key = SigningKey::generate(&mut rng);
        
        Self {
            master_seed,
            key_cache: HashMap::new(),
            root_key,
        }
    }
    
    /// Returns the root signing key (for Trust Engine root authority).
    pub fn root_signing_key(&self) -> &SigningKey {
        &self.root_key
    }
    
    /// Returns the root public key.
    pub fn root_public_key(&self) -> VerifyingKey {
        self.root_key.verifying_key()
    }
    
    /// Returns the root key as a biscuit-auth KeyPair.
    ///
    /// This is needed because biscuit-auth has its own key type that's incompatible
    /// with ed25519-dalek keys. We derive a separate biscuit key from the seed.
    pub fn biscuit_root_key(&self) -> biscuit_auth::KeyPair {
        use rand::SeedableRng;
        use rand_chacha::ChaCha8Rng;
        // Use a different salt for biscuit keys to avoid collision
        let biscuit_seed = self.master_seed.wrapping_mul(0x3c6ef372fe94f82b);
        let mut rng = ChaCha8Rng::seed_from_u64(biscuit_seed);
        biscuit_auth::KeyPair::new_with_rng(&mut rng)
    }
    
    /// Generates or retrieves the signing key for an agent.
    ///
    /// The key is derived deterministically from:
    /// `master_seed XOR (agent_id * prime)`
    pub fn agent_key(&mut self, agent_id: u64) -> SigningKey {
        if let Some(key) = self.key_cache.get(&agent_id) {
            return key.clone();
        }
        
        // Derive unique seed for this agent
        let agent_seed = self.master_seed
            .wrapping_mul(0x9e3779b97f4a7c15)  // Golden ratio prime
            .wrapping_add(agent_id.wrapping_mul(0x517cc1b727220a95));
        
        let mut rng = ChaCha8Rng::seed_from_u64(agent_seed);
        let key = SigningKey::generate(&mut rng);
        
        self.key_cache.insert(agent_id, key.clone());
        key
    }
    
    /// Generates a batch of agent keys.
    pub fn generate_agent_keys(&mut self, num_agents: usize) -> Vec<SigningKey> {
        (0..num_agents as u64)
            .map(|id| self.agent_key(id))
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_deterministic_root_key() {
        let provider1 = DeterministicKeyProvider::new(42);
        let provider2 = DeterministicKeyProvider::new(42);
        
        assert_eq!(
            provider1.root_signing_key().to_bytes(),
            provider2.root_signing_key().to_bytes()
        );
    }
    
    #[test]
    fn test_deterministic_agent_keys() {
        let mut provider1 = DeterministicKeyProvider::new(42);
        let mut provider2 = DeterministicKeyProvider::new(42);
        
        let key1 = provider1.agent_key(5);
        let key2 = provider2.agent_key(5);
        
        assert_eq!(key1.to_bytes(), key2.to_bytes());
    }
    
    #[test]
    fn test_different_agents_different_keys() {
        let mut provider = DeterministicKeyProvider::new(42);
        
        let key0 = provider.agent_key(0);
        let key1 = provider.agent_key(1);
        let key2 = provider.agent_key(2);
        
        assert_ne!(key0.to_bytes(), key1.to_bytes());
        assert_ne!(key1.to_bytes(), key2.to_bytes());
        assert_ne!(key0.to_bytes(), key2.to_bytes());
    }
    
    #[test]
    fn test_key_isolation() {
        // Adding more agents shouldn't change existing keys
        let mut provider1 = DeterministicKeyProvider::new(42);
        let mut provider2 = DeterministicKeyProvider::new(42);
        
        // Provider 1: generate keys 0-2
        let keys1: Vec<_> = (0..3).map(|i| provider1.agent_key(i)).collect();
        
        // Provider 2: generate keys 0-9 (more agents)
        let _extra: Vec<_> = (0..10).map(|i| provider2.agent_key(i)).collect();
        
        // Keys 0-2 should be identical
        for i in 0..3 {
            assert_eq!(
                keys1[i as usize].to_bytes(),
                provider2.agent_key(i).to_bytes()
            );
        }
    }
}
