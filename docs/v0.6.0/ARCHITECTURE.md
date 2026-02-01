# GodView v0.6.0 Architecture Changes

## Engine Modifications

### evolution.rs — The Brain

**New Fields in `EvoParams`:**
```rust
pub sensor_bias_estimate: f64  // GPS calibration offset
```

**New Fields in `EvolutionaryState`:**
```rust
consecutive_failures: u32      // Stagnation tracking
step_multiplier: f64           // Adaptive mutation magnitude
was_multi_param: bool          // Multi-gene mutation flag
covariance_accumulator: f64    // For sharpness penalty
covariance_samples: u64        // Sample counter
```

**New in `BlindFitness`:**
```rust
// Sharpness penalty for groupthink resistance
if ctx.avg_nis < 0.5 && ctx.avg_covariance_trace > 100.0 {
    base_fitness *= 0.8;  // 20% penalty
}
```

**New Mutation Types:**
```rust
IncreaseBias,  // +0.5m per step
DecreaseBias,  // -0.5m per step
```

---

### agent.rs — The Body

**New Methods:**
```rust
/// Emergency Protocol: Stop messaging at low energy
pub fn should_broadcast(&self, current_tick: u64) -> bool {
    if self.energy < 50.0 { return false; }
    current_tick % self.gossip_interval() == 0
}

/// Getter for evolved bias estimate
pub fn sensor_bias_estimate(&self) -> f64
```

**Enhanced `tick_evolution`:**
- Now collects and passes covariance trace to evolution system
- Enables sharpness penalty calculation

---

### oracle.rs — The Simulator

**New Enum:**
```rust
pub enum NoiseModel {
    Gaussian,  // Standard, light-tailed
    Cauchy,    // Heavy-tailed, frequent outliers
    Levy,      // Extreme tails
}
```

**New Fields in `Oracle`:**
```rust
noise_model: NoiseModel
```

**Enhanced `generate_sensor_reading`:**
- Samples from configured noise distribution
- Cauchy uses rand_distr::Cauchy
- Lévy uses inverse CDF sampling

---

### scenarios.rs — The Gauntlet

**New Scenarios:**
```rust
CommonBias,   // DST-020: +5m GPS offset all agents
HeavyTail,    // DST-021: Cauchy noise stress test
SensorDrift,  // DST-022: 5x noise degradation
```

---

### runner.rs — The Arena

**New Scenario Implementations:**

| Function | Lines | Purpose |
|----------|-------|---------|
| `run_common_bias()` | ~120 | Tests evolvable bias compensation |
| `run_heavy_tail()` | ~90 | Tests Cauchy noise robustness |
| `run_sensor_drift()` | ~90 | Tests adaptive noise handling |

---

## Data Flow Changes

### Before v0.6.0
```
Sensor → Filter → Evolution (NIS + PA)
                      ↓
              Mutate single param
```

### After v0.6.0
```
Sensor → Filter → Evolution (NIS + PA + Covariance)
                      ↓
    ┌────────────────┴────────────────┐
    ↓                                 ↓
Stagnation? → Increase step      Multi-param?
    ↓                                 ↓
Mutate 1 param              Mutate ALL params
    ↓                                 ↓
    └────────────────┬────────────────┘
                     ↓
           Check Sharpness Penalty
                     ↓
           Apply Emergency Protocol
```

---

## Key Constants

| Constant | Value | Purpose |
|----------|-------|---------|
| EMERGENCY_ENERGY_THRESHOLD | 50.0 J | Stop messaging below this |
| STAGNATION_THRESHOLD | 5 failures | Trigger step escalation |
| STEP_MULTIPLIER_CAP | 10.0x | Maximum mutation magnitude |
| MULTI_PARAM_CHANCE | 10% | Probability of all-gene mutation |
| SHARPNESS_PENALTY | 0.8x | Fitness reduction for groupthink |
| NIS_THRESHOLD (sharpness) | 0.5 | Low NIS trigger |
| COV_THRESHOLD (sharpness) | 100.0 | High cov trigger |
