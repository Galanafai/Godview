# GodView Demo Story Package

Deterministic log generation for the LinkedIn "Before vs After" demo.

---

## Quick Start

```bash
cd /data/godview/demo/godview_demo_story
python3 generate_demo_logs.py --out ./out
```

---

## Generated Files

| File | Description |
|------|-------------|
| `out/packets.ndjson` | Per-agent detection packets (one per line) |
| `out/world_state.ndjson` | Fused canonical state per frame |
| `out/events.ndjson` | Discrete events: MERGE, TRUST_REJECT, OOSM_CORRECTED, etc. |

---

## Output Stats

| Metric | Value |
|--------|-------|
| Duration | 85 seconds |
| FPS | 30 |
| Total Frames | 2550 |
| Agents | 20 cars + 1 drone |
| Events | ~5 (MERGE, TRUST_REJECT, OOSM_CORRECTED, HANDOFF_OK, SPACE_SEPARATION) |

---

## CLI Arguments

```bash
python3 generate_demo_logs.py \
  --out ./out \        # Output directory
  --seed 42 \          # Random seed for determinism
  --fps 30 \           # Frames per second
  --duration_s 85 \    # Duration in seconds
  --num_agents 20      # Number of car agents (plus 1 drone)
```

---

## Determinism

**Same seed = identical output.**

To verify determinism:
```bash
python3 generate_demo_logs.py --out ./run1 --seed 42
python3 generate_demo_logs.py --out ./run2 --seed 42
diff run1/packets.ndjson run2/packets.ndjson  # Should be empty
```

To vary the scenario:
```bash
python3 generate_demo_logs.py --out ./run3 --seed 123  # Different seed
```

---

## BEFORE vs AFTER Phases

| Phase | Frames | Time | Behavior |
|-------|--------|------|----------|
| **HOOK** | 0–150 | 0–5s | Intro, gentle packet flow |
| **BEFORE** | 150–750 | 5–25s | Chaos: occlusion, ghosts, spoof accepted |
| **AFTER** | 750–1650 | 25–55s | Solution: fusion, merge, spoof rejected |
| **MONTAGE** | 1650–2250 | 55–75s | Deep dives: OOSM, 3D space, handoff, bandwidth |
| **CLOSE** | 2250–2550 | 75–85s | Stable world, fade out |

During **BEFORE**, spoof packets are accepted (no trust filtering).  
During **AFTER**, spoof packets trigger `TRUST_REJECT` events and are excluded from `world_state.ndjson`.

---

## Documentation

| File | Purpose |
|------|---------|
| [schema.md](./schema.md) | NDJSON field definitions and examples |
| [storyboard.md](./storyboard.md) | Beat-by-beat visual script with exact frame ranges |
| [failure_modes.md](./failure_modes.md) | Visual acceptance checklist |
| [manual_touchdesigner_tasks.md](./manual_touchdesigner_tasks.md) | Step-by-step TouchDesigner wiring guide |

---

## No CARLA Required

This script is standalone Python. No CARLA, ROS, or external dependencies.

The only requirement is Python 3.7+.
