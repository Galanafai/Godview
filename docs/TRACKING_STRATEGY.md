# GodView Core: Distributed Data Association Strategy

**Module:** `godview_core::tracking`  
**Version:** 0.4.0  
**Date:** 2025-12-22  
**Status:** Design Complete — Ready for Implementation

---

## Executive Summary

This document defines the strategy for the **Distributed Data Association Layer** in Project GodView v3. The goal is to eliminate the "Duplicate Ghost" phenomenon—where the same physical object appears with multiple track IDs across a decentralized mesh of ~50 autonomous agents—while preventing "Rumor Propagation" that causes filter divergence.

After evaluating alternatives, we have selected:

| Component | Algorithm | Rationale |
|:--|:--|:--|
| **Association** | Global Nearest Neighbor (GNN) | $O(N \log N)$ with spatial indexing; deterministic |
| **Fusion** | Covariance Intersection (CI) | Loop-safe; handles unknown correlations |
| **Identity** | Highlander Heuristic (Min-UUID CRDT) | Eventual consistency without negotiation |

This document serves as the **architectural blueprint** for implementation. No Rust code is included—only the design that guides the coding.

---

## Part 1: The Strategic Decision

### 1.1 The Problem Statement

Project GodView v3 operates as a decentralized, peer-to-peer perception network. Without a central server assigning "ground truth" IDs, the system faces two fundamental pathologies:

#### 1.1.1 The "Duplicate Ghost" Problem

When multiple agents observe the same physical object, each generates an independent track with a unique UUID. Until these tracks are associated and merged, the global world model contains **N duplicate entries** for a single hazard.

```
┌─────────────────────────────────────────────────────────────┐
│                    PHYSICAL WORLD                           │
│                          🧍                                  │
│                     (One Pedestrian)                        │
└─────────────────────────────────────────────────────────────┘
                            │
          ┌─────────────────┼─────────────────┐
          ▼                 ▼                 ▼
     Agent A            Agent B            Agent C
     UUID: abc...       UUID: def...       UUID: 123...
          │                 │                 │
          └─────────────────┼─────────────────┘
                            ▼
┌─────────────────────────────────────────────────────────────┐
│                    WORLD MODEL                              │
│          🧍 (abc)      🧍 (def)      🧍 (123)                │
│              THREE GHOSTS FOR ONE PERSON                    │
└─────────────────────────────────────────────────────────────┘
```

**Consequences:**
- Visual flicker as IDs compete for the same screen position
- Double/triple counting in path planning ("avoid three pedestrians" vs. one)
- Downstream planning errors and resource waste

#### 1.1.2 The "Rumor Propagation" Problem

In a mesh network, fused data recirculates. If Agent A sends a track to Agent B, and Agent B fuses it with its own observation, Agent B's "improved" estimate returns to Agent A. If Agent A treats this as new independent information and fuses again, the covariance shrinks artificially.

```
Initial:  Agent A observes object with P = σ²

Loop 1:   A → B → A    →  P' = σ²/2   (halved uncertainty)
Loop 2:   A → B → A    →  P'' = σ²/4  (quartered)
Loop N:   ...          →  P → 0       (filter thinks it's infinitely precise)
```

This is known as **data incest** or **mathematical incest**. The filter becomes overconfident, ignores new sensor data, and eventually diverges from reality.

> [!CAUTION]
> Standard Kalman fusion (Information Filter) assumes independent errors.  
> In a mesh network, this assumption is **always violated**.  
> Using $P_{fused}^{-1} = P_A^{-1} + P_B^{-1}$ causes catastrophic overconfidence.

---

### 1.2 The Solution: GNN + CI + Highlander

We reject the following alternatives:

| Algorithm | Rejection Reason |
|:--|:--|
| **MHT (Multiple Hypothesis Tracking)** | Exponential complexity $O(m^n)$; infeasible on GTX 1050 Ti |
| **JPDA (Joint Probabilistic Data Association)** | Risk of **track coalescence**—distinct close objects merge into one "average" track. Also, synchronizing probabilities across a mesh is bandwidth-prohibitive. |
| **Consensus Kalman** | Requires multiple communication rounds per cycle; violates 100 ms budget |

We select:

#### 1.2.1 Global Nearest Neighbor (GNN)

GNN is a **hard assignment** algorithm: each incoming observation is assigned to exactly one existing track (or spawns a new track).

**Advantages:**
- Deterministic: No probabilistic bookkeeping
- Fast: $O(N \log N)$ with spatial indexing (vs. $O(N^2)$ naive)
- Simple: Easy to debug, test, and verify

**Disadvantages (mitigated):**
- Can mis-associate in high clutter → Mitigated by Mahalanobis gating
- No uncertainty in assignment → Acceptable for 30 Hz update rate (errors correct quickly)

#### 1.2.2 Covariance Intersection (CI)

CI provides a **conservative but consistent** fusion when correlation between estimates is unknown.

$$P_{CI}^{-1} = \omega P_A^{-1} + (1 - \omega) P_B^{-1}$$

$$\hat{x}_{CI} = P_{CI} \left[ \omega P_A^{-1} \hat{x}_A + (1 - \omega) P_B^{-1} \hat{x}_B \right]$$

**Key Property:** Even if $\hat{x}_A$ and $\hat{x}_B$ are 100% correlated (same underlying data), CI produces a covariance that is **no smaller** than the best single source. This guarantees loop-safety.

**Weight Selection:** We use **Fast-CI with Trace Minimization**:

$$\omega = \frac{\text{tr}(P_B)}{\text{tr}(P_A) + \text{tr}(P_B)}$$

This closed-form heuristic avoids iterative optimization while giving more weight to the more precise estimate.

#### 1.2.3 The Highlander Heuristic (Min-UUID CRDT)

> *"There can be only one."*

When two tracks are associated, they carry different UUIDs. We need a deterministic rule to pick the **canonical ID** that all agents will independently converge to.

**The Rule:**
```
canonical_id = min(local_id, remote_id, ...all_observed_ids)
```

**Properties:**
- **Deterministic:** All agents perform the same comparison, reach the same result
- **Monotonic:** IDs only ever decrease (no oscillation)
- **Conflict-Free:** This is a CRDT (Conflict-free Replicated Data Type) merge function
- **No Negotiation:** Zero additional messages required

**Convergence Proof:**  
Let $ID_1 < ID_2 < ID_3$ be the UUIDs for the same object across three agents.  
- Agent 1 sees $ID_1$ → keeps $ID_1$  
- Agent 2 sees $ID_2$, receives $ID_1$ → switches to $ID_1$  
- Agent 3 sees $ID_3$, receives $ID_1$ or $ID_2$ → switches to min → $ID_1$  

After one complete gossip round, all agents display $ID_1$. The flicker lasts at most one network RTT (~100-200 ms).

---

### 1.3 Complexity Constraints

**Target Platform:** GTX 1050 Ti (4 GB VRAM, mid-range CPU)  
**Cycle Budget:** 100 ms (10 Hz minimum, 30 Hz target)  
**Bandwidth Budget:** ~1.5 MB/s for 50 agents

| Operation | Required Complexity | Target Latency |
|:--|:--|:--|
| Spatial Pruning (H3 lookup) | $O(1)$ | < 1 µs |
| Candidate Filtering (k-ring) | $O(k)$ where $k \approx 7$ | < 10 µs |
| Mahalanobis Gating | $O(M)$ where $M$ = candidates | < 50 µs |
| GNN Assignment | $O(M \log M)$ | < 100 µs |
| Covariance Intersection | $O(1)$ per fusion (fixed 6×6 matrix) | < 20 µs |
| Highlander ID Resolution | $O(1)$ (UUID comparison) | < 1 µs |

**Total per packet:** < 200 µs  
**Throughput:** 5000+ packets/second (well above the ~1500/s from 50 agents at 30 Hz)

> [!IMPORTANT]
> The GNN algorithm **must** use spatial indexing to achieve $O(N \log N)$.  
> Without H3+Octree pre-filtering, complexity degrades to $O(N^2)$, violating the cycle budget.

---

## Part 2: The Implementation Architecture

### 2.1 Data Structures

#### 2.1.1 `GlobalHazardPacket` (Network Message)

This is the wire format received from Zenoh. Already defined in `godview_core`.

```
GlobalHazardPacket {
    entity_id:        Uuid        // Publisher's local UUID for this object
    position:         [f64; 3]    // ECEF or WGS84 [lat, lon, alt]
    velocity:         [f64; 3]    // [vx, vy, vz] in m/s
    class_id:         u8          // 0=Unknown, 1=Vehicle, 2=Pedestrian, 3=Cyclist, ...
    timestamp:        f64         // Unix timestamp (seconds)
    confidence_score: f64         // 0.0 - 1.0
    // Optional: feature_embedding: [f32; 16]  (future: appearance features)
}
```

#### 2.1.2 `UniqueTrack` (Internal Representation)

```
UniqueTrack {
    // === Identity ===
    canonical_id:   Uuid              // The "winning" ID (smallest UUID seen)
    observed_ids:   HashSet<Uuid>     // All aliases (for debugging/analysis)
    
    // === State (6-DOF) ===
    state:          Vector6<f64>      // [x, y, z, vx, vy, vz]
    covariance:     Matrix6<f64>      // 6×6 uncertainty matrix
    
    // === Metadata ===
    class_id:       u8                // Object class
    last_update:    f64               // Timestamp of last fusion
    age:            u32               // Number of cycles since last update
    
    // === Spatial Index Key ===
    h3_cell:        CellIndex         // Current H3 cell (Resolution 10)
}
```

**Key Design Decisions:**
- `observed_ids` is a **growing set** that accumulates all UUIDs ever associated with this track. This is a CRDT (G-Set).
- `canonical_id` is always the **minimum** element of `observed_ids`.
- `covariance` is a full 6×6 matrix (not diagonal) to capture velocity-position correlations from the EKF.

#### 2.1.3 `TrackManager` (The Engine)

```
TrackManager {
    // === Track Store ===
    tracks:             HashMap<Uuid, UniqueTrack>   // Keyed by canonical_id
    
    // === Spatial Index ===
    spatial_index:      HashMap<CellIndex, HashSet<Uuid>>  // H3 cell → track IDs
    h3_resolution:      Resolution                   // Default: Resolution::Ten
    
    // === Configuration ===
    gating_threshold:   f64        // Chi-squared threshold (e.g., 12.59 for 6 DOF, 95%)
    max_age:            u32        // Cycles before track deletion (e.g., 60 = 2 seconds at 30 Hz)
    base_pos_variance:  f64        // For confidence → covariance conversion
    base_vel_variance:  f64
}
```

**Spatial Index Invariants:**
1. Every track in `tracks` has exactly one entry in `spatial_index` under its current `h3_cell`.
2. When a track moves to a new cell, the old entry is removed and a new one is added.
3. The spatial index is **not** the source of truth—it's a secondary index for fast lookup.

---

### 2.2 The Processing Pipeline

Every incoming `GlobalHazardPacket` flows through a 4-stage pipeline:

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                         PACKET PROCESSING PIPELINE                          │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│   ┌──────────────┐    ┌──────────────┐    ┌──────────────┐    ┌──────────┐ │
│   │   STAGE 1    │───▶│   STAGE 2    │───▶│   STAGE 3    │───▶│  STAGE 4 │ │
│   │   Spatial    │    │   Geometric  │    │   Identity   │    │   State  │ │
│   │   Pruning    │    │    Gating    │    │  Resolution  │    │  Fusion  │ │
│   └──────────────┘    └──────────────┘    └──────────────┘    └──────────┘ │
│         │                   │                   │                   │       │
│         ▼                   ▼                   ▼                   ▼       │
│   H3 Cell Lookup      Mahalanobis         Highlander           Covariance  │
│   + k-ring(1)         Distance            Min-UUID             Intersection│
│   → ~7-19 cells       + Chi² test         → canonical_id       → fused P   │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

#### Stage 1: Spatial Pruning

**Goal:** Reduce the search space from all tracks to only those in the local vicinity.

**Algorithm:**
1. Convert packet position to H3 cell at Resolution 10
2. Query spatial_index for the packet's cell
3. Query spatial_index for the k-ring(1) neighbors (~6 additional cells)
4. Collect all track IDs from these cells → `candidates`

**Complexity:** $O(k)$ where $k \approx 7$ hexagons

**Output:** `candidates: Vec<Uuid>` (typically 0-20 tracks)

#### Stage 2: Geometric Gating (Mahalanobis Distance)

**Goal:** Filter candidates to only those that are statistically consistent with the observation.

**Algorithm:**
For each candidate track, compute:

$$D_M^2 = (\mathbf{z} - H\mathbf{x})^T S^{-1} (\mathbf{z} - H\mathbf{x})$$

Where:
- $\mathbf{z}$ = measurement vector (position from packet)
- $\mathbf{x}$ = track state
- $H$ = observation matrix (extracts position from state)
- $S = HPH^T + R$ = innovation covariance
- $R$ = measurement noise (derived from `confidence_score`)

**Gating Rule:**
```
if D_M² < γ   →  Accept as candidate
else          →  Reject
```

**Chi-Squared Threshold ($\gamma$):**
| DOF | 90% | 95% | 99% |
|:--|:--|:--|:--|
| 3 (position only) | 6.25 | 7.81 | 11.34 |
| 6 (position + velocity) | 10.64 | 12.59 | 16.81 |

We recommend $\gamma = 12.59$ (6 DOF, 95% confidence) for the full state, or $\gamma = 7.81$ (3 DOF) if gating on position only.

**Additional Hard Gating:**
- **Class Mismatch:** If `track.class_id != packet.class_id`, reject immediately. A pedestrian observation should not associate with a vehicle track.

**Output:** `gated_candidates: Vec<(Uuid, f64)>` (track ID, Mahalanobis distance²)

#### Stage 3: Identity Resolution (Highlander)

**Goal:** If a match is found, determine which UUID becomes canonical.

**Algorithm (GNN + Highlander):**

```
if gated_candidates.is_empty() {
    // No match → Create new track
    create_track(packet)
} else {
    // Match found → Select best (GNN: minimum distance)
    best_match = gated_candidates.min_by_key(|(_, d)| d)
    
    let track = tracks.get_mut(best_match.id)
    
    // Highlander: Swap to smaller UUID
    if packet.entity_id < track.canonical_id {
        track.canonical_id = packet.entity_id
    }
    
    // Accumulate alias
    track.observed_ids.insert(packet.entity_id)
    
    // Proceed to fusion
    fuse(track, packet)
}
```

**Key Insight:** The GNN assignment is implicit—by taking the minimum Mahalanobis distance, we implement greedy nearest-neighbor without explicitly solving an assignment matrix.

#### Stage 4: State Fusion (Covariance Intersection)

**Goal:** Combine the track state with the incoming observation while maintaining consistency despite unknown correlations.

**Algorithm (Fast-CI with Trace Minimization):**

```
Input:
    x_A, P_A  = track state and covariance
    x_B, P_B  = measurement state and covariance (from packet)

Step 1: Compute weight
    ω = tr(P_B) / (tr(P_A) + tr(P_B))

Step 2: Compute information matrices
    P_A_inv = P_A.inverse()
    P_B_inv = P_B.inverse()

Step 3: Fuse information
    P_CI_inv = ω * P_A_inv + (1 - ω) * P_B_inv
    P_CI = P_CI_inv.inverse()

Step 4: Fuse state
    x_CI = P_CI * (ω * P_A_inv * x_A + (1 - ω) * P_B_inv * x_B)

Output:
    track.state = x_CI
    track.covariance = P_CI
```

**Fallback on Singularity:** If any matrix inverse fails (determinant ≈ 0), skip fusion and retain the current track state. Log a warning.

---

### 2.3 Track Lifecycle Management

#### Creation
A new track is created when:
1. No candidates pass spatial pruning, OR
2. No candidates pass geometric gating

Initial state:
```
canonical_id = packet.entity_id
observed_ids = { packet.entity_id }
state = [packet.position..., packet.velocity...]
covariance = confidence_to_covariance(packet.confidence_score)
age = 0
```

#### Update
On successful association:
```
age = 0
last_update = packet.timestamp
// State and covariance updated by CI fusion
// canonical_id potentially swapped by Highlander
```

#### Aging
Every cycle (even without update):
```
age += 1
// Optionally: inflate covariance slightly (process noise for tracks not observed)
```

#### Deletion
When `age > max_age`:
```
remove track from tracks
remove track from spatial_index
```

This handles objects leaving the scene or going out of range.

---

## Part 3: The Execution Plan

### Implementation Phases

We break the build into four sequential phases, each delivering a testable increment.

---

### Phase 1: Core Structs & Spatial Indexing (The Skeleton)

**Goal:** Establish the data structures and the spatial index infrastructure.

**Deliverables:**
1. Define `UniqueTrack` struct with all fields
2. Define `TrackManager` struct with `tracks` and `spatial_index`
3. Implement `TrackManager::new(config)` constructor
4. Implement H3 cell computation: `position_to_cell(lat, lon, resolution) -> CellIndex`
5. Implement `spatial_index.insert(cell, track_id)`
6. Implement `spatial_index.remove(cell, track_id)`
7. Implement `spatial_index.query_kring(cell, k) -> HashSet<Uuid>`

**Tests:**
- Insert track, verify it appears in spatial index
- Move track (change cell), verify old cell is empty and new cell contains it
- Query k-ring, verify correct neighbor cells are returned

**Dependencies:** `h3o`, `uuid`, `nalgebra`

---

### Phase 2: The Math Engine (Mahalanobis & GNN)

**Goal:** Implement the geometric gating logic.

**Deliverables:**
1. Implement `confidence_to_covariance(score: f64) -> Matrix6<f64>`
2. Implement `mahalanobis_distance_squared(track, packet) -> f64`
3. Implement `gate_candidates(candidates, packet, threshold) -> Vec<(Uuid, f64)>`
4. Implement `select_best_match(gated) -> Option<Uuid>` (GNN: min distance)

**Key Math:**
```
S = H * P * H^T + R
residual = z - H * x
d² = residual^T * S^-1 * residual
```

Where $H$ is the 3×6 observation matrix (or 6×6 if observing velocity too).

**Tests:**
- Two tracks at same position with different covariances → correct Mahalanobis values
- Track and packet with matching class → not rejected by class gate
- Track and packet with different class → rejected before Mahalanobis

**Edge Cases:**
- Singular covariance matrix → return `f64::MAX`
- Zero velocity → still computes correctly

---

### Phase 3: The Logic Engine (Highlander & Aliases)

**Goal:** Implement identity resolution and alias management.

**Deliverables:**
1. Implement `UniqueTrack::merge_id(remote_id: Uuid)` → updates `canonical_id` if remote is smaller
2. Implement `UniqueTrack::add_alias(id: Uuid)` → adds to `observed_ids`
3. Implement `TrackManager::resolve_identity(track, packet)` → orchestrates the swap
4. Implement `TrackManager::reindex_track(track)` → updates spatial index if cell changed

**The Highlander Invariant:**
```
assert!(track.canonical_id == track.observed_ids.iter().min())
```

**Tests:**
- Track with ID `zzz...` receives packet with ID `aaa...` → canonical becomes `aaa...`
- Track already at `aaa...` receives `bbb...` → canonical stays `aaa...`
- `observed_ids` grows with each new alias

---

### Phase 4: The Fusion Engine (Covariance Intersection)

**Goal:** Implement the Fast-CI algorithm.

**Deliverables:**
1. Implement `covariance_intersection(x_A, P_A, x_B, P_B) -> (x_CI, P_CI)`
2. Implement `TrackManager::fuse_track(track, packet)`
3. Implement `TrackManager::process_packet(packet)` → the full pipeline

**Algorithm (Trace-Minimization CI):**
```rust
fn covariance_intersection(
    x_a: &Vector6, p_a: &Matrix6,
    x_b: &Vector6, p_b: &Matrix6
) -> (Vector6, Matrix6) {
    let tr_a = p_a.trace();
    let tr_b = p_b.trace();
    let omega = tr_b / (tr_a + tr_b);
    
    let p_a_inv = p_a.try_inverse()?;
    let p_b_inv = p_b.try_inverse()?;
    
    let p_ci_inv = p_a_inv * omega + p_b_inv * (1.0 - omega);
    let p_ci = p_ci_inv.try_inverse()?;
    
    let x_ci = p_ci * (p_a_inv * x_a * omega + p_b_inv * x_b * (1.0 - omega));
    
    (x_ci, p_ci)
}
```

**Tests:**
- Fuse two identical estimates → result equals the input (idempotent)
- Fuse precise and imprecise → result biased toward precise
- Fuse correlated data (same source looped back) → covariance does NOT shrink below original

**Validation (Rumor Propagation Test):**
```
Initial: P = diag(1.0, 1.0, 1.0, 0.1, 0.1, 0.1)
Fuse with copy of self (simulating loop)
Assert: P_fused.trace() >= P.trace()
```

---

## Appendix A: Known Issues from Code Audit

The following issues were identified in the existing `godview_core` implementation and must be addressed during this implementation:

| Module | Issue | Severity | Fix Required |
|:--|:--|:--|:--|
| `godview_time.rs` | Covariance matrix not shifted in `augment_state` | Critical | Implement block-shift for covariance rows/cols |
| `godview_space.rs` | `query_sphere` uses linear scan instead of octree | High | Use `Oktree::query_range()` for retrieval |
| `godview_space.rs` | Hardcoded `66.0` divisor for k-ring calculation | Medium | Compute from `resolution.edge_length()` |
| `godview_trust.rs` | Revocation check is $O(N)$ on Vec | Low | Change to `HashSet<VerifyingKey>` |

---

## Appendix B: Performance Budget

| Component | Budget | Justification |
|:--|:--|:--|
| **Network Receive** | 50 µs | Zenoh is optimized for low-latency |
| **Spatial Pruning** | 10 µs | HashMap lookups + k-ring iteration |
| **Mahalanobis Gating** | 50 µs | Per-candidate: 1 µs × 50 candidates (generous) |
| **GNN Selection** | 10 µs | Sorting ~10 candidates |
| **Covariance Intersection** | 30 µs | 3 matrix inversions (6×6) |
| **Highlander + Alias** | 5 µs | UUID comparison, HashSet insert |
| **Spatial Re-index** | 10 µs | If cell changed |
| **Total per Packet** | **~165 µs** | |

At 30 Hz × 50 agents = 1500 packets/second, the CPU will spend:
```
1500 × 165 µs = 247,500 µs = 0.247 seconds per second = 24.7% CPU utilization
```

This leaves ample headroom (~75%) for the EKF prediction, network I/O, and other processing.

---

## Appendix C: References

1. **Fusion Algorithm Design** — Internal design docs (`Fusion1.md`, `Fusion2.md`)
2. **Sanity Check** — Architecture review (`sanitycheck.md`)
3. **Julier & Uhlmann** — Covariance Intersection for unknown correlations
4. **ETSI TR 103 562** — Collective Perception Service (CPM) standards
5. **Uber H3** — Hierarchical hexagonal geospatial indexing

---

**Document Status:** ✅ Ready for Implementation  
**Next Step:** Begin Phase 1 (Core Structs & Spatial Indexing)

---

*Author: Principal Sensor Fusion Engineer*  
*Reviewed: GodView Architecture Team*
