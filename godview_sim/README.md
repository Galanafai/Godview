# GodView Deterministic Simulation Testing (DST) Framework

## Overview

The **Deterministic Simulation Testing (DST)** framework is a chaos engineering infrastructure for GodView that enables reproducible testing of distributed system behavior under adversarial conditions. By eliminating all sources of non-determinism, we can:

1. **Reproduce any bug** with a single 64-bit seed
2. **Prove correctness** across thousands of randomized scenarios
3. **Catch Heisenbugs** that only manifest under specific timing/ordering conditions
4. **Test years of operation** in minutes of wall-clock time

## Why DST?

### The Problem: Distributed Systems Are Hard

GodView is a multi-agent distributed tracking system where:
- Multiple nodes process sensor data independently
- Out-of-sequence measurements (OOSM) arrive constantly
- Network partitions can split the swarm temporarily
- Malicious actors may attempt to inject false data
- H3 spatial cells must hand off entities smoothly

Traditional testing approaches fail because:

| Testing Approach | Limitation |
|-----------------|------------|
| Unit Tests | Don't catch timing-dependent bugs |
| Integration Tests | Non-deterministic, can't reproduce failures |
| Stress Tests | Find bugs but can't debug them |
| Formal Verification | Doesn't scale to real implementations |

### The Solution: Reactor Pattern + Deterministic Simulation

Inspired by [FoundationDB's simulation testing](https://www.youtube.com/watch?v=4fFDFbi3toc), we implement the **Reactor Pattern**:

```
┌─────────────────────────────────────────────────────────┐
│                    GodViewAgent                          │
│  ┌─────────┐ ┌─────────┐ ┌─────────┐ ┌──────────────┐   │
│  │  Time   │ │  Space  │ │  Trust  │ │   Tracking   │   │
│  │ Engine  │ │ Engine  │ │ Engine  │ │    Engine    │   │
│  └────┬────┘ └────┬────┘ └────┬────┘ └──────┬───────┘   │
│       │           │           │              │           │
│       ▼           ▼           ▼              ▼           │
│  ┌─────────────────────────────────────────────────┐    │
│  │              GodViewContext Trait                │    │
│  │   now() | sleep() | spawn() | derive_key()      │    │
│  └─────────────────────────────────────────────────┘    │
└─────────────────────────────────────────────────────────┘
                           │
          ┌────────────────┴────────────────┐
          ▼                                 ▼
   ┌──────────────┐                 ┌──────────────┐
   │ TokioContext │                 │  SimContext  │
   │ (Production) │                 │ (Simulation) │
   └──────────────┘                 └──────────────┘
```

**Key insight**: By abstracting time, randomness, and I/O behind traits, the same production code runs in both environments—but in simulation, we control every nanosecond.

## What DST Proves

### 1. Time Engine Correctness (DST-001: TimeWarp)

**Scenario**: OOSM stress test with 0-500ms jitter and 20% packet reordering.

**What it proves**:
- Kalman filter correctly handles out-of-sequence measurements
- State augmentation maintains covariance positive-definiteness
- MaxLag gating prevents unbounded state growth
- Cholesky recovery handles numerical instability

### 2. Spatial Consistency (DST-004: FlashMob)

**Scenario**: 1000 drones crossing H3 cell boundaries rapidly.

**What it proves**:
- Gnomonic projection is numerically stable
- H3 indexing handles boundary transitions
- No entity "teleportation" or duplicates
- Spatial queries remain consistent

### 3. Network Resilience (DST-002: SplitBrain)

**Scenario**: Network partition for 10 seconds, then heal.

**What it proves**:
- CRDT-based state converges after partition heals
- No data loss during partition
- Min-UUID conflict resolution is deterministic
- Highlander rule prevents ghost conflicts

### 4. Security Under Attack (DST-003: Byzantine)

**Scenario**: Malicious agent with delayed revocation propagation.

**What it proves**:
- Revocation list replicates correctly
- Invalid packets are rejected after revocation
- Biscuit token validation is enforced
- No privilege escalation possible

### 5. Fault Tolerance (DST-005: SlowLoris)

**Scenario**: 50% packet loss for extended period.

**What it proves**:
- Protocol recovers from high packet loss
- State eventually converges
- No deadlocks or livelocks under pressure
- Bandwidth-limited graceful degradation

## Architecture

### Crate Structure

```
godview/
├── godview_env/          # Abstraction traits
│   ├── GodViewContext    # Time, sleep, spawn, keys
│   ├── NetworkTransport  # Send/recv packets
│   └── TokioContext      # Production implementation
│
├── godview_core/         # Engine implementations
│   ├── agent_runtime     # GodViewAgent<Ctx, Net>
│   ├── godview_time      # Kalman + OOSM
│   ├── godview_space     # H3 + Gnomonic
│   ├── godview_trust     # Biscuit + Revocation
│   └── godview_tracking  # Track fusion
│
└── godview_sim/          # Simulation harness
    ├── SimContext        # Virtual clock + seeded RNG
    ├── DeterministicKeyProvider
    ├── Oracle            # Ground truth physics
    ├── SimNetwork        # Fault injection
    ├── SimWorld          # Container
    └── ScenarioRunner    # Chaos scenarios
```

### Determinism Sources Eliminated

| Non-determinism Source | Solution |
|----------------------|----------|
| System clock | Virtual clock via `SimContext::now()` |
| Random numbers | Seeded ChaCha8Rng |
| Thread scheduling | Single-threaded reactor |
| Network ordering | Controlled `SimNetwork` |
| Crypto keys | `DeterministicKeyProvider` |
| Physics noise | Seeded Gaussian noise |

## Usage

### CLI

```bash
# Run all scenarios with seed 42
godview-sim --seed 42 --scenario all

# Run specific scenario
godview-sim --seed 42 --scenario split_brain --duration 60

# CI mode: 100 random seeds with JSON output
godview-sim --seeds 100 --scenario all --json

# Reproduce a failing seed
godview-sim --seed 8675309 --scenario time_warp -v
```

### GitHub Actions

The DST workflow runs automatically on every push:

```yaml
# .github/workflows/dst.yml
- 100 seeds split across 10 parallel jobs
- Commit hash-based seeds for reproducibility
- Failed seed artifacts uploaded for debugging
```

### Reproducing Failures

When CI fails:
1. Download `failed_seeds.txt` artifact
2. Run locally: `godview-sim --seed <failed_seed> --scenario <scenario> -v`
3. Debug with full visibility into deterministic execution

## Test Coverage

| Component | Tests | Coverage |
|-----------|-------|----------|
| godview_env | 3 | TokioContext |
| godview_core | 38 | All 4 engines |
| godview_sim | 20 | Simulation + scenarios |
| **Total** | **61** | |

## Key Design Decisions

### 1. Trait-Based Abstraction

Instead of mocking, we use **real implementations** behind trait bounds:
```rust
pub struct GodViewAgent<Ctx: GodViewContext, Net: NetworkTransport> {
    context: Arc<Ctx>,
    network: Arc<Net>,
    // ... engines
}
```

This means production bugs are reproducible in simulation.

### 2. Seeded Everything

A single master seed derives all randomness:
```rust
// Different subsystems get isolated RNG streams
let context_seed = master_seed;
let physics_seed = master_seed.wrapping_mul(0x9e3779b97f4a7c15);
let key_seed = master_seed.wrapping_mul(0x517cc1b727220a95);
```

### 3. Oracle-Based Ground Truth

The `Oracle` maintains perfect knowledge of entity states:
```rust
// Generate noisy sensor reading from ground truth
let reading = oracle.generate_reading(entity_id);

// Compare against agent's estimate
let error = (agent_estimate - oracle_truth).norm();
assert!(error < threshold);
```

### 4. Chaos Injection Points

`SimNetworkController` enables targeted failures:
```rust
// Create partition between groups
controller.partition(group_a, group_b);

// Add latency to specific link
controller.set_latency(node_a, node_b, 100);

// 50% packet loss
controller.set_loss(node_a, node_b, 0.5);
```

## Future Work

- [ ] Rerun visualization integration for debugging
- [ ] Property-based testing with Proptest
- [ ] Fuzzing integration with AFL/libFuzzer
- [ ] Clock drift simulation for NTP edge cases
- [ ] Byzantine fault injection (not just revocation)

## References

- [FoundationDB: Testing Distributed Systems w/ Deterministic Simulation](https://www.youtube.com/watch?v=4fFDFbi3toc)
- [TigerBeetle: Simulation Testing](https://tigerbeetle.com/blog/2023-03-28-tigerbeetles-deterministic-simulation-testing/)
- [Jepsen: Distributed Systems Safety Testing](https://jepsen.io/)
- [Antithesis: Autonomous Testing](https://antithesis.com/)

## License

MIT
