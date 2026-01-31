# GodView DST: Deterministic Simulation Testing

## Overview

DST ensures GodView is **correct, reproducible, and resilient** by running chaos engineering scenarios with deterministic seeds.

---

## What DST Proves About Core Code

| Scenario | Core System Tested | What It Proves |
|----------|-------------------|----------------|
| **DST-001: TimeWarp** | Time Engine (OOSM) | Out-of-sequence measurements with 0-500ms jitter handled correctly |
| **DST-002: SplitBrain** | Spatial Engine (CRDTs) | Network partition → Min-UUID convergence works |
| **DST-003: Byzantine** | Trust Engine | Malicious agent revocation propagates correctly |
| **DST-004: FlashMob** | Spatial Engine (H3) | 1000 entities crossing cell boundaries tracked correctly |
| **DST-005: SlowLoris** | Network Layer | 50% packet loss → protocol recovery works |
| **DST-006: Swarm** | All Engines | 50 agents, 200 entities, P2P gossip converges to <1m error |
| **DST-007: AdaptiveSwarm** | Learning System | Agents learn to identify and filter bad actors |

---

## Scenario Details

### DST-001: TimeWarp
**Tests**: OOSM (Out-of-Sequence Measurement) handling

```
Jitter: 0-500ms random delay
Reorder rate: 20% of packets arrive out of order
```

**Pass Criteria**: All OOSM updates processed without exception

**Core Code Validated**:
- `godview_core/src/godview_time.rs` - Lag state augmentation
- Retroactive Kalman fusion

---

### DST-002: SplitBrain
**Tests**: CRDT convergence after network partition

```
Partition duration: 10 seconds
Topology: 6 agents split into 2 groups of 3
```

**Pass Criteria**: Min-UUID track IDs converge across all agents

**Core Code Validated**:
- `godview_core/src/godview_tracking.rs` - Highlander heuristic
- Entity ID resolution via Min-UUID

---

### DST-003: Byzantine
**Tests**: Malicious agent handling

```
Byzantine agent: Sends conflicting track data
Revocation delay: 5 seconds
```

**Pass Criteria**: Revocation propagates, bad agent isolated

**Core Code Validated**:
- `godview_core/src/godview_trust.rs` - Biscuit token verification
- Revocation list propagation

---

### DST-004: FlashMob
**Tests**: H3 cell boundary crossing

```
Entities: 1000 drones
Movement: Rapid boundary crossing
```

**Pass Criteria**: All entities tracked, no ghost tracks

**Core Code Validated**:
- `godview_core/src/godview_spatial.rs` - H3 indexing
- Cell handoff without track loss

---

### DST-005: SlowLoris
**Tests**: High packet loss resilience

```
Packet loss: 50%
Duration: 30 seconds
```

**Pass Criteria**: Tracks maintain accuracy, protocol recovers

**Core Code Validated**:
- UDP reliability layer
- Track prediction during gaps

---

### DST-006: Swarm
**Tests**: Multi-agent scale

```
Agents: 50 in 5x10 grid
Entities: 200
P2P Messages: ~14M per run
```

**Results**:
- Track count CV: 9% (within 15% limit)
- RMS position error: 0.88m

**Core Code Validated**:
- All four engines working together at scale
- P2P gossip correctness

---

### DST-007: AdaptiveSwarm
**Tests**: Learning agent intelligence

```
Agents: 50 (5 become bad actors at t=10s)
Bad actor behavior: Inject garbage packets
```

**Results**:
- Detection rate: 100% of bad actors identified
- Accuracy: 0.88m RMS maintained
- Gossip filtered: 27.8M messages blocked

**Core Code Validated**:
- `godview_sim/src/adaptive.rs` - Neighbor reputation learning
- Automatic bad actor isolation

---

## CLI Usage

```bash
# Run single scenario
godview-sim --seed 42 --scenario swarm

# Run all scenarios
godview-sim --seed 42 --scenario all

# Run with JSON output for CI
godview-sim --seed 42 --scenario all --json

# Run 100 seeds for comprehensive testing
godview-sim --seeds 100 --scenario all
```

---

## Key Metrics

| Metric | Threshold | Purpose |
|--------|-----------|---------|
| RMS Position Error | <3m | Track accuracy |
| Track Count CV | <15% | Agent agreement |
| Detection Rate | >30% | Bad actor identification |
| Convergence Time | <5s | CRDT performance |

---

## Determinism Guarantee

Same seed → Same result. Always.

```bash
# These will produce identical output:
godview-sim --seed 42 --scenario swarm
godview-sim --seed 42 --scenario swarm
```

This enables:
- Reproducible bug investigation
- CI regression testing
- Performance benchmarking
