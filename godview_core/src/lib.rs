//! GodView Core v3 - High-Precision Distributed Spatial Computing Protocol
//!
//! This library solves four critical flaws in distributed sensor fusion:
//! 1. **Time Travel Problem**: Out-of-Sequence Measurements via Augmented State EKF
//! 2. **Pancake World Problem**: 3D spatial indexing via H3 + Sparse Voxel Octrees
//! 3. **Phantom Hazards Problem**: Cryptographic provenance via CapBAC + Ed25519
//! 4. **Duplicate Ghost Problem**: Distributed data association via GNN + CI + Highlander

pub mod godview_time;
pub mod godview_space;
pub mod godview_trust;
pub mod godview_tracking;

// Re-export key types for convenience
pub use godview_time::AugmentedStateFilter;
pub use godview_space::{Entity, SpatialEngine, WorldShard};
pub use godview_trust::{AuthError, SecurityContext, SignedPacket};
pub use godview_tracking::{GlobalHazardPacket, TrackManager, TrackingConfig, TrackingError, UniqueTrack};
