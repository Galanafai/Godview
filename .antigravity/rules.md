# Antigravity Agent Rules

> Behavioral guardrails for code generation in GodView. These rules are enforced by agent review.

## Code Generation Standards

### 1. Metric Instrumentation
Every public function in engine modules MUST emit telemetry:

```rust
// ✅ CORRECT
pub fn update_entity(&mut self, entity: Entity) -> Result<CellIndex, SpatialError> {
    metrics::counter!("godview.space.entity_updates").increment(1);
    // ... implementation
}

// ❌ WRONG - missing metrics
pub fn update_entity(&mut self, entity: Entity) -> Result<CellIndex, SpatialError> {
    // ... implementation without instrumentation
}
```

### 2. Error Handling
Use `thiserror` for library errors. **Never** use `anyhow::Result` in public API.

```rust
// ✅ CORRECT
#[derive(Debug, thiserror::Error)]
pub enum SpatialError {
    #[error("Invalid H3 cell: {0}")]
    InvalidCell(String),
    #[error("Entity not found: {0}")]
    NotFound(Uuid),
}

pub fn query_radius(&self, ...) -> Result<Vec<&Entity>, SpatialError>

// ❌ WRONG - generic error type
pub fn query_radius(&self, ...) -> anyhow::Result<Vec<&Entity>>
```

### 3. Panic-Freedom
No `unwrap()` or `expect()` in production code paths.

```rust
// ✅ CORRECT
let chol = S.cholesky().ok_or(TimeError::CholeskyFailed)?;

// ❌ WRONG - will panic
let chol = S.cholesky().expect("Covariance not positive definite");
```

### 4. Async/Sync Boundary
Core calculation logic MUST be synchronous and pure. Only I/O is async.

```rust
// ✅ CORRECT - sync math, async I/O
impl TrackManager {
    pub fn fuse_track(&mut self, ...) -> Result<...>  // sync - pure computation
}

impl NetworkBridge {
    pub async fn send_packet(&self, ...) -> Result<...>  // async - network I/O
}

// ❌ WRONG - async computation
impl TrackManager {
    pub async fn fuse_track(&mut self, ...) -> Result<...>  // async is unnecessary overhead
}
```

---

## Testing Requirements

### Property Testing for Space Engine
Any modification to `godview_space.rs` coordinate functions MUST include `proptest` cases:

```rust
#[cfg(test)]
mod tests {
    use proptest::prelude::*;
    
    proptest! {
        #[test]
        fn test_global_local_roundtrip(
            lat in -85.0f64..85.0,
            lon in -180.0f64..180.0
        ) {
            let shard = create_test_shard(lat, lon);
            let local = shard.global_to_local([lat, lon, 100.0]);
            let global = shard.local_to_global(local);
            
            prop_assert!((global[0] - lat).abs() < 1e-5);
            prop_assert!((global[1] - lon).abs() < 1e-5);
        }
    }
}
```

### Unit Test Coverage
New public functions require at least:
- 1 happy path test
- 1 edge case test (boundary conditions)
- 1 error case test (if Result-returning)

---

## Artifact Requirements

### Pre-Code: PLAN.md
Before making structural changes, output a plan documenting:
- Files to modify
- Breaking changes
- Verification steps

### Post-Code: Visual Proof
For spatial changes, generate Rerun visualization showing:
- H3 cell boundaries
- Entity positions
- Query results

---

## Prohibited Patterns

| Pattern | Reason | Alternative |
|---------|--------|-------------|
| `unwrap()` | Panics in production | `?` operator with Result |
| `expect()` | Panics in production | `.ok_or(Error)?` |
| `panic!()` | Crashes system | Return `Err(...)` |
| `anyhow` in lib | Generic errors | `thiserror` with typed errors |
| `async` in math | Unnecessary overhead | Keep computation sync |
| `HashMap` for hot paths | Poor cache locality | Use `Slab` or `Vec` |
