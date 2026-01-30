# GodView Glossary

> Domain-specific terminology. Reference this to avoid semantic confusion during code generation.

## Core Concepts

### Shard (WorldShard)
A **columnar volume of space** defined by an H3 cell. The H3 hexagon defines the 2D footprint; the internal 3D grid provides altitude-aware queries.

**File**: `godview_space.rs`

---

### Entity
A tracked object in the GodView world (drone, vehicle, hazard, etc.). Contains:
- Position `[f64; 3]` (lat, lon, alt)
- Velocity `[f64; 3]`
- Type classification
- Source agent ID

**File**: `godview_space.rs`

---

## Ghost (DISAMBIGUATION)

> [!CAUTION]
> The term "Ghost" has **two distinct meanings** in GodView. Context matters!

### Ghost (Spatial) - Edge Cache
A **cached entity reference** stored in a neighboring shard's "ghost buffer" for boundary optimization.

**Purpose**: When an entity is near a shard boundary, its reference is duplicated to neighbors. This allows radius queries to complete within a single shard lock.

**Location**: `WorldShard.ghost_buffer` (planned)

### Ghost (Tracking) - Duplicate Track
A **spurious duplicate track** created when multiple sensors detect the same object and generate independent track IDs.

**Purpose**: The Highlander algorithm resolves these by merging tracks with the same physical object.

**Metric**: `ghost_score` (probability that tracked ID is a duplicate)

**Location**: `godview_tracking.rs`, `metrics.rs`

---

## Highlander
"There Can Be Only One" - The **CRDT merge heuristic** for track ID convergence.

**Mechanism**: When two tracks are associated (Mahalanobis distance below threshold), their IDs merge using Min-UUID selection:
```rust
canonical_id = min(track_a.id, track_b.id)
```

**Properties**:
- Idempotent: `merge(A, A) = A`
- Commutative: `merge(A, B) = merge(B, A)`
- Associative: Distributed nodes converge to same canonical ID

**File**: `godview_tracking.rs`

---

## OOSM (Out-of-Sequence Measurement)
A sensor measurement that arrives **after** the system has already advanced past its capture timestamp.

**Example**: Drone captures position at t=100ms, packet arrives at t=350ms due to network latency. The filter has already predicted to t=350ms.

**Solution**: Augmented State EKF maintains history, enabling retrodiction.

**File**: `godview_time.rs`

---

## ASEKF (Augmented State Extended Kalman Filter)
The **Time Engine** implementation that handles OOSM by maintaining a rolling window of past states in the state vector.

**State structure**:
```
x_aug = [x_current | x_{t-1} | x_{t-2} | ... | x_{t-lag}]
```

**File**: `godview_time.rs`

---

## CapBAC (Capability-Based Access Control)
The decentralized authorization model. Capabilities are embedded in **Biscuit tokens** and verified locally without central authority.

**File**: `godview_trust.rs`

---

## Biscuit
A cryptographic token format (like JWT but with embedded logic). Contains:
- Facts and rules (Datalog)
- Signature chain (Ed25519)
- Attenuation (tokens can be restricted, never expanded)

**Crate**: `biscuit-auth`

---

## GNN (Global Nearest Neighbor)
Data association algorithm that assigns measurements to tracks by minimizing total assignment cost (Mahalanobis distances).

**File**: `godview_tracking.rs`

---

## CI (Covariance Intersection)
Fusion algorithm that safely combines estimates when correlations are unknown. More conservative than Kalman fusion but guaranteed consistent.

**File**: `godview_tracking.rs`
