# GodView Core - Code Logic Deep Dive

This document provides detailed explanations of the key algorithms and code blocks in `godview_core`.

---

## Table of Contents

1. [Time Engine: Augmented State EKF](#time-engine-augmented-state-ekf)
2. [Space Engine: H3 + 3D Grid](#space-engine-h3--3d-grid)
3. [Trust Engine: CapBAC + Ed25519](#trust-engine-capbac--ed25519)
4. [Tracking Engine: GNN + CI + Highlander](#tracking-engine-gnn--ci--highlander)

---

## Time Engine: Augmented State EKF

**File:** `godview_core/src/godview_time.rs`

### Purpose

The Time Engine handles **Out-of-Sequence Measurements (OOSM)** - a common problem in distributed systems where measurements arrive with variable latency (100ms - 500ms). Instead of rewinding time, the AS-EKF maintains a history of past states.

### Key Data Structure

```rust
pub struct AugmentedStateFilter {
    /// Augmented state: [x_k, x_{k-1}, ..., x_{k-N}]^T
    /// Size: state_dim × (max_lag_depth + 1)
    pub state_vector: DVector<f64>,
    
    /// Augmented covariance matrix
    /// Tracks correlations between current and past states
    pub covariance: DMatrix<f64>,
    
    pub state_dim: usize,      // e.g., 6 for [x, y, z, vx, vy, vz]
    pub max_lag_depth: usize,  // e.g., 20 for 600ms at 30Hz
}
```

### Critical Algorithm: State Augmentation

When time advances, we shift states into history **and** shift the covariance matrix:

```rust
fn augment_state(&mut self, current_time: f64) {
    let s = self.state_dim;
    
    // 1. Shift state blocks (index N → N+1, etc.)
    for i in (1..=self.max_lag_depth).rev() {
        let src = (i - 1) * s;
        let dst = i * s;
        let block = self.state_vector.rows(src, s).clone_owned();
        self.state_vector.rows_mut(dst, s).copy_from(&block);
    }
    
    // 2. CRITICAL: Also shift covariance matrix blocks
    // Block P_{i,j} moves to P_{i+1, j+1}
    for i in (1..=self.max_lag_depth).rev() {
        for j in (1..=self.max_lag_depth).rev() {
            let block = self.covariance
                .view(((i-1)*s, (j-1)*s), (s, s))
                .clone_owned();
            self.covariance
                .view_mut((i*s, j*s), (s, s))
                .copy_from(&block);
        }
    }
}
```

**Why This Matters:** Without shifting the covariance, the correlations between past states become invalid, causing incorrect Kalman gains and potential filter divergence.

---

## Space Engine: H3 + 3D Grid

**File:** `godview_core/src/godview_space.rs`

### Purpose

The Space Engine solves the **"Pancake World"** problem - standard 2D geospatial indexes (like geohash) ignore altitude, causing drones and ground vehicles to appear co-located.

### Two-Level Hierarchy

```
                    ┌─────────────────────────────────────┐
                    │         Global (Spherical)           │
                    │    H3 Hexagonal Grid (Res 10)       │
                    │         ~66m per hexagon            │
                    └─────────────┬───────────────────────┘
                                  │
                    ┌─────────────▼───────────────────────┐
                    │        Local (3D Cartesian)         │
                    │     3D Hash Grid (10m cells)        │
                    │      O(k³) spherical queries        │
                    └─────────────────────────────────────┘
```

### 3D Grid Spatial Index

```rust
#[derive(Hash, Eq, PartialEq)]
struct GridCell {
    x: i32,
    y: i32,
    z: i32,
}

impl GridCell {
    fn from_local_coords(coords: [f32; 3], cell_size: f32) -> Self {
        Self {
            x: (coords[0] / cell_size).floor() as i32,
            y: (coords[1] / cell_size).floor() as i32,
            z: (coords[2] / cell_size).floor() as i32,
        }
    }
}
```

### Sphere Query Algorithm

Instead of O(N) linear scan, we query only neighboring grid cells:

```rust
pub fn query_sphere(&self, center: [f32; 3], radius: f32) -> Vec<&Entity> {
    let cell_radius = (radius / self.grid_cell_size).ceil() as i32;
    let center_cell = GridCell::from_local_coords(center, self.grid_cell_size);
    
    let mut results = Vec::new();
    
    // Only check cells within radius (O(k³) vs O(N))
    for grid_cell in center_cell.neighbors(cell_radius) {
        if let Some(entity_ids) = self.spatial_grid.get(&grid_cell) {
            for &id in entity_ids {
                // Fine-grained distance check
                if distance(&entity, center) <= radius {
                    results.push(entity);
                }
            }
        }
    }
    
    results
}
```

### H3 Resolution Lookup Table

```rust
const H3_EDGE_LENGTH_M: [f64; 16] = [
    1107712.591,  // Res 0
    // ...
    65.907,       // Res 10 - ~66m (default)
    24.910,       // Res 11 - ~25m
    // ...
];

fn edge_length_meters(resolution: Resolution) -> f64 {
    H3_EDGE_LENGTH_M[resolution as usize]
}
```

---

## Trust Engine: CapBAC + Ed25519

**File:** `godview_core/src/godview_trust.rs`

### Purpose

The Trust Engine prevents **Sybil attacks** and data spoofing using:
1. **Ed25519 signatures** for cryptographic provenance
2. **Biscuit tokens** for capability-based access control

### Signed Packet Structure

```rust
pub struct SignedPacket {
    /// Serialized payload (msgpack or similar)
    payload: Vec<u8>,
    
    /// Ed25519 signature over payload
    #[serde(with = "signature_serde")]
    signature: Signature,
    
    /// Sender's public key (for verification)
    #[serde(with = "verifying_key_serde")]
    public_key: VerifyingKey,
}
```

### Signature Verification

```rust
impl SignedPacket {
    pub fn verify_integrity(&self) -> Result<(), AuthError> {
        self.public_key
            .verify(&self.payload, &self.signature)
            .map_err(|_| AuthError::InvalidSignature)
    }
}
```

### O(1) Revocation Check

```rust
pub struct SecurityContext {
    root_public_key: PublicKey,
    
    /// HashSet for O(1) lookup (previously Vec with O(N))
    revoked_keys: HashSet<[u8; 32]>,
}

impl SecurityContext {
    pub fn revoke_key(&mut self, key: VerifyingKey) {
        self.revoked_keys.insert(key.to_bytes());  // O(1)
    }
    
    pub fn is_revoked(&self, key: &VerifyingKey) -> bool {
        self.revoked_keys.contains(&key.to_bytes())  // O(1)
    }
}
```

### Biscuit Token Creation (v4.0 API)

```rust
pub fn create_admin_token(&self) -> Result<Vec<u8>, AuthError> {
    let biscuit = biscuit!(r#"
        right("admin");
    "#)
        .build(&self.root_keypair)?;
    
    Ok(biscuit.to_vec()?)
}
```

---

## Tracking Engine: GNN + CI + Highlander

**File:** `godview_core/src/godview_tracking.rs`

### Purpose

The Tracking Engine eliminates **duplicate ghosts** when multiple agents observe the same entity and report it independently. It uses three key algorithms:

1. **GNN (Global Nearest Neighbor)** for data association
2. **Covariance Intersection** for loop-safe fusion
3. **Highlander CRDT** for distributed ID resolution

### The 4-Stage Pipeline

```rust
pub fn process_packet(&mut self, packet: &GlobalHazardPacket) -> Result<Uuid, TrackingError> {
    // Stage 1 & 2: Find association
    match self.find_association(packet)? {
        Some(track_id) => {
            // Stage 3 & 4: Fuse with existing track
            self.fuse_track(track_id, packet)?;
            let track = self.tracks.get(&track_id).unwrap();
            Ok(track.canonical_id)
        }
        None => {
            // No match: Create new track
            self.create_track(packet)
        }
    }
}
```

### Stage 1: Spatial Pruning (H3 k-ring)

```rust
pub fn spatial_query_kring(&self, cell: CellIndex, k: u32) -> HashSet<Uuid> {
    let mut result = HashSet::new();
    
    // Query center cell + k rings of neighbors
    for neighbor_cell in cell.grid_disk_safe(k) {
        if let Some(track_ids) = self.spatial_index.get(&neighbor_cell) {
            result.extend(track_ids.iter().copied());
        }
    }
    
    result
}
```

**Result:** For k=1, returns ~7 hexagons covering ~300m radius.

### Stage 2: Mahalanobis Gating

```rust
pub fn mahalanobis_distance_squared(
    &self,
    track: &UniqueTrack,
    packet: &GlobalHazardPacket,
) -> f64 {
    // Observation matrix H: extracts [x, y, z, vx, vy, vz] from state
    let h = Matrix6::identity();
    
    // Measurement vector
    let z = Vector6::new(
        packet.position[0], packet.position[1], packet.position[2],
        packet.velocity[0], packet.velocity[1], packet.velocity[2],
    );
    
    // Residual
    let residual = z - &h * &track.state;
    
    // Innovation covariance: S = H*P*H' + R
    let r = self.confidence_to_covariance(packet.confidence_score);
    let s = &h * &track.covariance * h.transpose() + r;
    
    // Mahalanobis distance²
    let s_inv = s.try_inverse().unwrap_or(Matrix6::identity() * 1e6);
    (residual.transpose() * s_inv * residual)[(0, 0)]
}
```

**Threshold:** D² < 12.59 (Chi² distribution, 6 DOF, 95% confidence)

### Stage 3: Highlander ID Resolution (Min-UUID CRDT)

```rust
impl UniqueTrack {
    pub fn merge_id(&mut self, remote_id: Uuid) {
        // G-Set: Only grows, never shrinks
        self.observed_ids.insert(remote_id);
        
        // Highlander: "There can be only one" - the smallest UUID wins
        if remote_id < self.canonical_id {
            self.canonical_id = remote_id;
        }
    }
}
```

**Why This Works:** All agents will eventually converge to the same canonical_id (the smallest) without any coordination protocol.

### Stage 4: Covariance Intersection (Loop-Safe Fusion)

```rust
pub fn covariance_intersection(
    x_a: &Vector6<f64>,
    p_a: &Matrix6<f64>,
    x_b: &Vector6<f64>,
    p_b: &Matrix6<f64>,
) -> Option<(Vector6<f64>, Matrix6<f64>)> {
    // Weight using trace minimization
    let tr_a = p_a.trace();
    let tr_b = p_b.trace();
    let omega = tr_b / (tr_a + tr_b);  // More weight to smaller trace
    
    // Information matrices
    let p_a_inv = p_a.try_inverse()?;
    let p_b_inv = p_b.try_inverse()?;
    
    // Fused information matrix
    let p_ci_inv = p_a_inv * omega + p_b_inv * (1.0 - omega);
    let p_ci = p_ci_inv.try_inverse()?;
    
    // Fused state
    let x_ci = p_ci * (p_a_inv * x_a * omega + p_b_inv * x_b * (1.0 - omega));
    
    Some((x_ci, p_ci))
}
```

**Why Covariance Intersection?**  
Standard Kalman fusion assumes independent measurements. In a gossip network, the same data can recirculate (rumors). CI is **loop-safe** - it never reduces uncertainty below the most confident input.

**Proof (Rumor Safety Test):**
```rust
#[test]
fn test_covariance_intersection_rumor_safety() {
    let x = Vector6::new(10.0, 20.0, 30.0, 1.0, 2.0, 0.0);
    let p = Matrix6::from_diagonal(&Vector6::new(4.0, 4.0, 4.0, 1.0, 1.0, 1.0));
    let original_trace = p.trace();
    
    // Fuse with EXACT SAME DATA (simulating looped rumor)
    let (_, p_fused) = covariance_intersection(&x, &p, &x, &p).unwrap();
    
    // INVARIANT: Covariance should NOT shrink
    assert!(p_fused.trace() >= original_trace * 0.99);
}
```

---

## Complexity Summary

| Operation | Complexity per Object | Notes |
|:--|:--|:--|
| `predict()` | **O(1)** | Constant time (fixed 9×9 matrix ops) |
| `update_oosm()` | **O(1)** | Constant time (fixed 9×9 matrix ops) |
| `query_sphere()` | **O(1)** | Constant time (queries fixed number of neighboring cells) |
| `verify_integrity()` | O(P) | Ed25519 verifies linear with payload size P |
| `is_revoked()` | O(1) | HashSet lookup |
| `process_packet()` | O(log M) | M candidates in local neighborhood |
| `covariance_intersection()` | **O(1)** | Constant time (fixed 6×6 matrix ops) |

> **Note on "O(N³)" notation:** In filtering literature, you often see O(N³) where N is the state dimension (e.g., 9 variables). Since N is fixed and small (9), these operations are **O(1) Constant Time** relative to the number of agents or tracks in the system. The system scales linearly with the number of tracked objects.

---

## Further Reading

- [TRACKING_STRATEGY.md](./TRACKING_STRATEGY.md) - Original design specification
- [ARCHITECTURE.md](./ARCHITECTURE.md) - High-level architecture overview
- [H3 Documentation](https://h3geo.org/) - Hexagonal spatial indexing
- [Biscuit Auth](https://www.biscuitsec.org/) - Capability-based tokens
