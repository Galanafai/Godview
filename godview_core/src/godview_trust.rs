//! The "TRUST" Engine - Capability-Based Access Control (CapBAC)
//!
//! Solves the "Phantom Hazards" problem by:
//! - Using Biscuit tokens for decentralized authorization
//! - Using Ed25519 signatures for cryptographic provenance
//! - Preventing Sybil attacks and data spoofing
//! - **V4**: Persisting revocation list to survive restarts

use biscuit_auth::{Biscuit, KeyPair, PublicKey, macros::*};
use ed25519_dalek::{Signature, Signer, SigningKey, Verifier, VerifyingKey};
use serde::{Deserialize, Serialize};
use std::collections::HashSet;
use std::path::Path;
use std::sync::Arc;
use thiserror::Error;

/// Authentication and authorization errors
#[derive(Debug, Error)]
pub enum AuthError {
    #[error("Invalid token: {0}")]
    InvalidToken(String),
    
    #[error("Invalid signature")]
    InvalidSignature,
    
    #[error("Unauthorized: {0}")]
    Unauthorized(String),
    
    #[error("Token expired")]
    TokenExpired,
    
    #[error("Biscuit error: {0}")]
    BiscuitError(String),
    
    #[error("Storage error: {0}")]
    StorageError(String),
}

/// A cryptographically signed packet
///
/// Ensures:
/// 1. Integrity: Payload hasn't been tampered with
/// 2. Provenance: We know who created this data
/// 3. Non-repudiation: Sender cannot deny creating it
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SignedPacket {
    /// The actual data (e.g., serialized HazardPacket)
    pub payload: Vec<u8>,
    
    /// Ed25519 signature of the payload
    #[serde(with = "signature_serde")]
    pub signature: Signature,
    
    /// Public key of the signer
    #[serde(with = "verifying_key_serde")]
    pub public_key: VerifyingKey,
    
    /// Optional metadata (agent ID, timestamp, etc.)
    pub metadata: Option<PacketMetadata>,
}

/// Metadata attached to signed packets
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PacketMetadata {
    pub agent_id: String,
    pub timestamp: i64,
    pub packet_type: String,
}

impl SignedPacket {
    /// Create a new signed packet
    ///
    /// # Arguments
    /// * `payload` - The data to sign
    /// * `signing_key` - The sender's private key
    /// * `metadata` - Optional metadata
    pub fn new(
        payload: Vec<u8>,
        signing_key: &SigningKey,
        metadata: Option<PacketMetadata>,
    ) -> Self {
        let signature = signing_key.sign(&payload);
        let public_key = signing_key.verifying_key();
        
        Self {
            payload,
            signature,
            public_key,
            metadata,
        }
    }
    
    /// Verify the cryptographic integrity of this packet
    ///
    /// Returns Ok(()) if signature is valid, Err otherwise
    pub fn verify_integrity(&self) -> Result<(), AuthError> {
        self.public_key
            .verify(&self.payload, &self.signature)
            .map_err(|_| AuthError::InvalidSignature)
    }
    
    /// Get the payload if signature is valid
    pub fn get_verified_payload(&self) -> Result<&[u8], AuthError> {
        self.verify_integrity()?;
        Ok(&self.payload)
    }
}

// ============================================================================
// REVOCATION STORE (V4: Persistent Storage)
// ============================================================================

/// Trait for persistent revocation storage
/// 
/// Implementations must be thread-safe and persist data across restarts.
pub trait RevocationStore: Send + Sync {
    /// Insert a revoked key into the store
    fn insert(&self, key_bytes: [u8; 32]) -> Result<(), AuthError>;
    
    /// Check if a key is in the revocation store
    fn contains(&self, key_bytes: &[u8; 32]) -> bool;
    
    /// Load all revoked keys from persistent storage
    fn load_all(&self) -> Result<HashSet<[u8; 32]>, AuthError>;
}

/// Sled-based persistent revocation store
/// 
/// Uses an embedded key-value database for durability.
pub struct SledRevocationStore {
    db: sled::Db,
}

impl SledRevocationStore {
    /// Open a persistent store at the given path
    pub fn open<P: AsRef<Path>>(path: P) -> Result<Self, AuthError> {
        let db = sled::open(path)
            .map_err(|e| AuthError::StorageError(format!("Failed to open sled DB: {}", e)))?;
        Ok(Self { db })
    }
    
    /// Create a temporary store (for testing)
    #[cfg(test)]
    pub fn open_temp() -> Result<Self, AuthError> {
        let config = sled::Config::new().temporary(true);
        let db = config.open()
            .map_err(|e| AuthError::StorageError(format!("Failed to open temp DB: {}", e)))?;
        Ok(Self { db })
    }
}

impl RevocationStore for SledRevocationStore {
    fn insert(&self, key_bytes: [u8; 32]) -> Result<(), AuthError> {
        self.db.insert(&key_bytes, &[1u8])
            .map_err(|e| AuthError::StorageError(format!("Insert failed: {}", e)))?;
        self.db.flush()
            .map_err(|e| AuthError::StorageError(format!("Flush failed: {}", e)))?;
        Ok(())
    }
    
    fn contains(&self, key_bytes: &[u8; 32]) -> bool {
        self.db.contains_key(key_bytes).unwrap_or(false)
    }
    
    fn load_all(&self) -> Result<HashSet<[u8; 32]>, AuthError> {
        let mut keys = HashSet::new();
        for result in self.db.iter() {
            let (key, _) = result
                .map_err(|e| AuthError::StorageError(format!("Iteration failed: {}", e)))?;
            if key.len() == 32 {
                let mut arr = [0u8; 32];
                arr.copy_from_slice(&key);
                keys.insert(arr);
            }
        }
        Ok(keys)
    }
}

// ============================================================================
// SECURITY CONTEXT
// ============================================================================

/// Security context for the GodView system
///
/// Handles:
/// - Biscuit token verification
/// - Access control policy enforcement
/// - Public key management
/// - **V4**: Optional persistent revocation storage
pub struct SecurityContext {
    /// Root public key for verifying Biscuit tokens
    pub root_public_key: PublicKey,
    
    /// Cache of revoked public keys (stored as bytes for O(1) lookup)
    /// VerifyingKey doesn't implement Hash, so we store [u8; 32] instead
    revoked_keys: HashSet<[u8; 32]>,
    
    /// Optional persistent store for revocations
    store: Option<Arc<dyn RevocationStore>>,
}

impl SecurityContext {
    /// Create a new SecurityContext (in-memory only, for backward compatibility)
    ///
    /// # Arguments
    /// * `root_public_key` - The root authority's public key
    pub fn new(root_public_key: PublicKey) -> Self {
        Self {
            root_public_key,
            revoked_keys: HashSet::new(),
            store: None,
        }
    }
    
    /// Create a SecurityContext with persistent revocation storage (V4)
    ///
    /// Hydrates the in-memory cache from the persistent store on creation.
    ///
    /// # Arguments  
    /// * `root_public_key` - The root authority's public key
    /// * `store` - Persistent revocation store implementation
    pub fn with_store(
        root_public_key: PublicKey,
        store: Arc<dyn RevocationStore>,
    ) -> Result<Self, AuthError> {
        // Hydrate in-memory cache from persistent store
        let revoked_keys = store.load_all()?;
        
        Ok(Self {
            root_public_key,
            revoked_keys,
            store: Some(store),
        })
    }
    
    /// Verify access to a resource
    ///
    /// This checks:
    /// 1. Token signature is valid (signed by root authority)
    /// 2. Token hasn't expired
    /// 3. Datalog policies allow the operation
    ///
    /// # Arguments
    /// * `token_bytes` - Serialized Biscuit token
    /// * `resource` - Resource being accessed (e.g., "godview/nyc/sector_7")
    /// * `operation` - Operation being performed (e.g., "publish_hazard")
    pub fn verify_access(
        &self,
        token_bytes: &[u8],
        resource: &str,
        operation: &str,
    ) -> Result<(), AuthError> {
        // Step 1: Deserialize and verify token signature
        let biscuit = Biscuit::from(token_bytes, self.root_public_key)
            .map_err(|e| AuthError::InvalidToken(format!("{:?}", e)))?;
        
        // Step 2: Create authorizer context
        let mut authorizer = biscuit
            .authorizer()
            .map_err(|e| AuthError::BiscuitError(format!("{:?}", e)))?;
        
        // Step 3: Add facts about the current request using macros
        authorizer
            .add_fact(fact!("resource({resource})"))
            .map_err(|e| AuthError::BiscuitError(format!("{:?}", e)))?;
        
        authorizer
            .add_fact(fact!("operation({operation})"))
            .map_err(|e| AuthError::BiscuitError(format!("{:?}", e)))?;
        
        // Step 4: Add authorization policies
        // Policy 1: Allow if token has admin right
        authorizer
            .add_policy("allow if right(\"admin\")")
            .map_err(|e| AuthError::BiscuitError(format!("{:?}", e)))?;
        
        // Policy 2: Allow if token has write right and resource matches
        authorizer
            .add_policy("allow if right(\"write\"), resource($res), $res.starts_with(\"godview/\")")
            .map_err(|e| AuthError::BiscuitError(format!("{:?}", e)))?;
        
        // Policy 3: Allow if token has specific sector access
        authorizer
            .add_policy("allow if right(\"publish\"), operation(\"publish_hazard\")")
            .map_err(|e| AuthError::BiscuitError(format!("{:?}", e)))?;
        
        // Step 5: Evaluate policies
        authorizer
            .authorize()
            .map_err(|e| AuthError::Unauthorized(format!("{:?}", e)))?;
        
        Ok(())
    }
    
    /// Verify a signed packet and check authorization
    ///
    /// This is the complete security check:
    /// 1. Verify packet signature (integrity + provenance)
    /// 2. Check if sender's key is revoked
    /// 3. Verify Biscuit token authorizes this operation
    pub fn verify_packet(
        &self,
        packet: &SignedPacket,
        token_bytes: &[u8],
        resource: &str,
        operation: &str,
    ) -> Result<(), AuthError> {
        // Step 1: Verify cryptographic signature
        packet.verify_integrity()?;
        
        // Step 2: Check if key is revoked (O(1) lookup with HashSet)
        if self.revoked_keys.contains(&packet.public_key.to_bytes()) {
            return Err(AuthError::Unauthorized("Public key revoked".to_string()));
        }
        
        // Step 3: Verify Biscuit token
        self.verify_access(token_bytes, resource, operation)?;
        
        Ok(())
    }
    
    /// Revoke a public key (for compromised agents)
    /// 
    /// If a persistent store is configured, the revocation is also persisted.
    /// In-memory revocation always succeeds; storage errors are logged but not propagated.
    /// 
    /// Complexity: O(1) insertion
    pub fn revoke_key(&mut self, key: VerifyingKey) {
        let key_bytes = key.to_bytes();
        self.revoked_keys.insert(key_bytes);
        
        // Persist to store if configured (V4)
        if let Some(ref store) = self.store {
            if let Err(e) = store.insert(key_bytes) {
                // Log but don't fail - in-memory revocation still valid
                eprintln!("[WARN] Failed to persist revocation: {}", e);
            }
        }
    }
    
    /// Check if a key is revoked
    /// 
    /// Complexity: O(1) lookup
    pub fn is_revoked(&self, key: &VerifyingKey) -> bool {
        self.revoked_keys.contains(&key.to_bytes())
    }
    
    /// Get the number of revoked keys
    pub fn revoked_count(&self) -> usize {
        self.revoked_keys.len()
    }
}

/// Helper for creating Biscuit tokens (for testing and admin tools)
pub struct TokenFactory {
    root_keypair: KeyPair,
}

impl TokenFactory {
    /// Create a new TokenFactory
    pub fn new(root_keypair: KeyPair) -> Self {
        Self { root_keypair }
    }
    
    /// Create a token with admin rights
    pub fn create_admin_token(&self) -> Result<Vec<u8>, AuthError> {
        let biscuit = biscuit!(r#"
            right("admin");
        "#)
            .build(&self.root_keypair)
            .map_err(|e| AuthError::BiscuitError(format!("{:?}", e)))?;
        
        Ok(biscuit.to_vec()
            .map_err(|e| AuthError::BiscuitError(format!("{:?}", e)))?)
    }
    
    /// Create a token with write access to a specific resource prefix
    pub fn create_write_token(&self, resource_prefix: &str) -> Result<Vec<u8>, AuthError> {
        let biscuit = biscuit!(r#"
            right("write");
            check if resource($res), $res.starts_with({resource_prefix});
        "#)
            .build(&self.root_keypair)
            .map_err(|e| AuthError::BiscuitError(format!("{:?}", e)))?;
        
        Ok(biscuit.to_vec()
            .map_err(|e| AuthError::BiscuitError(format!("{:?}", e)))?)
    }
    
    /// Create a token with publish rights
    pub fn create_publish_token(&self, sector: &str) -> Result<Vec<u8>, AuthError> {
        let prefix = format!("godview/{}", sector);
        let biscuit = biscuit!(r#"
            right("publish");
            check if resource($res), $res.starts_with({prefix});
        "#)
            .build(&self.root_keypair)
            .map_err(|e| AuthError::BiscuitError(format!("{:?}", e)))?;
        
        Ok(biscuit.to_vec()
            .map_err(|e| AuthError::BiscuitError(format!("{:?}", e)))?)
    }
}

mod signature_serde {
    use ed25519_dalek::Signature;
    use serde::{Deserialize, Deserializer, Serialize, Serializer};
    
    pub fn serialize<S>(sig: &Signature, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        sig.to_bytes().serialize(serializer)
    }
    
    pub fn deserialize<'de, D>(deserializer: D) -> Result<Signature, D::Error>
    where
        D: Deserializer<'de>,
    {
        // Deserialize as a Vec<u8> first, then convert to array
        let bytes: Vec<u8> = Deserialize::deserialize(deserializer)?;
        let bytes_array: [u8; 64] = bytes.try_into()
            .map_err(|_| serde::de::Error::custom("Expected 64 bytes for signature"))?;
        Ok(Signature::from_bytes(&bytes_array))
    }
}

mod verifying_key_serde {
    use ed25519_dalek::VerifyingKey;
    use serde::{Deserialize, Deserializer, Serialize, Serializer};
    
    pub fn serialize<S>(key: &VerifyingKey, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        key.to_bytes().serialize(serializer)
    }
    
    pub fn deserialize<'de, D>(deserializer: D) -> Result<VerifyingKey, D::Error>
    where
        D: Deserializer<'de>,
    {
        let bytes: [u8; 32] = Deserialize::deserialize(deserializer)?;
        VerifyingKey::from_bytes(&bytes).map_err(serde::de::Error::custom)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rand::rngs::OsRng;
    
    #[test]
    fn test_signed_packet_creation() {
        let signing_key = SigningKey::generate(&mut OsRng);
        let payload = b"test payload".to_vec();
        
        let packet = SignedPacket::new(payload.clone(), &signing_key, None);
        
        assert_eq!(packet.payload, payload);
        assert!(packet.verify_integrity().is_ok());
    }
    
    #[test]
    fn test_signature_verification_fails_on_tampering() {
        let signing_key = SigningKey::generate(&mut OsRng);
        let payload = b"original payload".to_vec();
        
        let mut packet = SignedPacket::new(payload, &signing_key, None);
        
        // Tamper with payload
        packet.payload = b"tampered payload".to_vec();
        
        assert!(packet.verify_integrity().is_err());
    }
    
    #[test]
    fn test_biscuit_authorization() {
        let root_keypair = KeyPair::new();
        let public_key = root_keypair.public();  // Get public key before moving
        let factory = TokenFactory::new(root_keypair);
        
        let admin_token = factory.create_admin_token().unwrap();
        
        let context = SecurityContext::new(public_key);
        
        let result = context.verify_access(
            &admin_token,
            "godview/nyc/sector_7",
            "publish_hazard",
        );
        
        assert!(result.is_ok());
    }
    
    #[test]
    fn test_unauthorized_access_denied() {
        let root_keypair = KeyPair::new();
        let public_key = root_keypair.public();  // Get public key before moving
        let factory = TokenFactory::new(root_keypair);
        
        // Create token for NYC only
        let nyc_token = factory.create_write_token("godview/nyc").unwrap();
        
        let context = SecurityContext::new(public_key);
        
        // Try to access SF resource with NYC token
        let result = context.verify_access(
            &nyc_token,
            "godview/sf/sector_1",
            "publish_hazard",
        );
        
        // Should fail (token restricted to NYC)
        assert!(result.is_err());
    }
}
