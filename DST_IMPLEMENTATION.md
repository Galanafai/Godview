# DST Implementation Plan

> **Deterministic Simulation Testing for GodView Multi-Agent System**

## The Reactor Pattern (Madsim)

Eliminates non-determinism by:
- **Single-threaded execution** - all actors serialized via global event queue
- **Virtual clock** - time advances only when all agents block on I/O
- **Seed-based entropy** - all randomness from 64-bit seed
- **I/O interception** - mocks `std::time`, `std::net`, `rand`, `fs`

**Result**: Any bug reproducible via `Seed #847291` instead of "it failed last Tuesday."

---

## Implementation Phases

### Phase 4.1: Abstraction Layer (Weeks 1-3)

**Deliverable**: `godview_env` crate

```rust
// Core environment abstraction
#[async_trait]
pub trait GodViewContext: Send + Sync + 'static {
    fn now(&self) -> Duration;
    fn system_time(&self) -> SystemTime;
    async fn sleep(&self, duration: Duration);
    fn spawn<F>(&self, future: F) where F: Future<Output = ()> + Send + 'static;
    fn derive_keypair(&self, seed_extension: u64) -> Keypair;
}

// Network abstraction
#[async_trait]
pub trait NetworkTransport: Send + Sync {
    async fn send(&self, target: NodeId, packet: SignedPacket) -> Result<(), NetworkError>;
    async fn recv(&self) -> Option<SignedPacket>;
}
```

**Files to Modify**:
| File | Change |
|------|--------|
| [godview_time.rs](file:///home/lap/Godview/godview_core/src/godview_time.rs) | Add `Ctx: GodViewContext` generic |
| [godview_space.rs](file:///home/lap/Godview/godview_core/src/godview_space.rs) | Add `Ctx: GodViewContext` generic |
| [godview_trust.rs](file:///home/lap/Godview/godview_core/src/godview_trust.rs) | Add `Ctx: GodViewContext` generic |
| [godview_tracking.rs](file:///home/lap/Godview/godview_core/src/godview_tracking.rs) | Add `Ctx: GodViewContext` generic |

**Exit Criteria**: Existing tests pass with `TokioContext` production impl.

---

### Phase 4.2: Simulator Core (Weeks 4-6)

**Deliverable**: `godview-sim` binary

**New Crate Structure**:
```
godview_sim/
├── Cargo.toml        # depends on madsim, godview_core
├── src/
│   ├── lib.rs
│   ├── context.rs    # SimContext impl
│   ├── world.rs      # SimWorld container
│   ├── oracle.rs     # Ground truth physics
│   └── keys.rs       # DeterministicKeyProvider
```

**Key Components**:
```rust
pub struct SimWorld {
    rt: madsim::runtime::Runtime,
    seed: u64,
    agents: HashMap<NodeId, AgentHandle>,
    oracle: OracleHandle,
    net_controller: NetworkController,
}
```

**Exit Criteria**: Single agent tracks synthetic target in simulation.

---

### Phase 4.3: Network & Multi-Agent (Weeks 7-9)

**Deliverable**: Multi-agent chaos scenarios

**SimNetworkController Capabilities**:
- Partition creation/healing
- Per-link latency configuration
- Jitter injection (uniform/exponential)
- Packet reordering/loss

**Chaos Scenarios**:

| ID | Name | Engine | Chaos | Invariant |
|----|------|--------|-------|-----------|
| DST-001 | Time Warp | `godview_time` | 0-500ms jitter, 20% reorder | Covariance convergence |
| DST-002 | Split Brain | `godview_tracking` | 10s network partition | Min-UUID convergence |
| DST-003 | Byzantine | `godview_trust` | Malicious agent, delayed revoke | Revocation rejection |
| DST-004 | Flash Mob | `godview_space` | 1000 drones, H3 boundary crossing | No memory leaks |
| DST-005 | Slow Loris | `godview_net` | 50% packet loss | Protocol recovery |

**Exit Criteria**: CRDT convergence verified across partition heal.

---

### Phase 4.4: CI/CD Integration (Weeks 10-12)

**Deliverable**: Automated chaos testing

**GitHub Actions Workflow**:
```yaml
name: DST Chaos Testing
on: [push, pull_request]
jobs:
  dst:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - run: cargo test --package godview-sim -- --test-threads=1
      - run: cargo run --package godview-sim -- --seeds 100 --scenario all
      - uses: actions/upload-artifact@v4
        if: failure()
        with:
          name: replay-artifacts
          path: target/dst-failures/
```

**Replay Artifacts** (on failure):
- Seed value
- `.rerun` visualization file
- Event log trace

---

## Dependencies

```toml
[dependencies]
madsim = "0.2"
async-trait = "0.1"

[dev-dependencies]
proptest = "1.4"
```

---

## Verification Metrics

| Metric | Target | Engine |
|--------|--------|--------|
| Ghost Score | < 1.05 (5% overhead) | godview_tracking |
| Covariance Trace | Bounded convergence | godview_time |
| NIS (Normalized Innovation) | Chi-square aligned | godview_time |
| Memory Growth | 0 per 100k ticks | godview_space |

---

## Current Status

**Phase**: Not Started  
**Next**: Create `godview_env` crate with `GodViewContext` trait
