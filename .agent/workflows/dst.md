---
description: Run DST (Deterministic Simulation Testing) scenarios
---

# /dst - Run DST Scenarios

Run deterministic chaos engineering tests to validate GodView core systems.

// turbo-all

## Quick Run (default)
```bash
cargo run --release -p godview_sim -- --seed 42 --scenario all
```

## Available Scenarios

| Scenario | Command |
|----------|---------|
| TimeWarp (OOSM) | `cargo run --release -p godview_sim -- --seed 42 --scenario time_warp` |
| SplitBrain (CRDT) | `cargo run --release -p godview_sim -- --seed 42 --scenario split_brain` |
| Byzantine (Trust) | `cargo run --release -p godview_sim -- --seed 42 --scenario byzantine` |
| FlashMob (H3) | `cargo run --release -p godview_sim -- --seed 42 --scenario flash_mob` |
| SlowLoris (Loss) | `cargo run --release -p godview_sim -- --seed 42 --scenario slow_loris` |
| Swarm (50 agents) | `cargo run --release -p godview_sim -- --seed 42 --scenario swarm` |
| AdaptiveSwarm (Learning) | `cargo run --release -p godview_sim -- --seed 42 --scenario adaptive_swarm` |

## Multi-Seed Stress Test

Run 100 different seeds to catch Heisenbugs:
```bash
cargo run --release -p godview_sim -- --seeds 100 --scenario all
```

## Makefile Shortcuts

- `make dst` - Quick single-seed run
- `make dst-quick` - 10 seeds
- `make dst-stress` - 100 seeds
- `make dst-all` - Run each scenario individually

## Expected Results

All scenarios should pass with:
- RMS position error < 3m
- Track count CV < 15%
- Bad actor detection rate > 30%
