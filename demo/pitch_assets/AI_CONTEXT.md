# Godview Pitch Deck - AI Context

## Project Overview

**Godview** solves the "Byzantine Generals Problem" for autonomous sensor networks. When multiple sensors (Camera, LiDAR, Radar) report conflicting data, Godview achieves consensus through four engines:

1. **Time Engine** - Enforces causality via Lamport clocks, rejects "time-traveling" data
2. **Space Engine** - 3D voxel disambiguation (drone at 50m ≠ car at 0m, even with same lat/lon)
3. **Tracking Engine** - Merges duplicate IDs via "Highlander" CRDT (there can be only one)
4. **Trust Engine** - Bayesian scoring (Beta distribution) blocks untrusted sources

## Files in This Folder

| File | Description |
|------|-------------|
| `slides_content.md` | Detailed slide specifications with visuals/math |
| `godview_demo_clean.svg` | Animated terminal demo (embed in HTML) |
| `godview_demo.cast` | Asciinema recording |
| `phase1_chaos.txt` | Terminal output: raw sensor chaos |
| `phase2a_time.txt` | Terminal output: Time Engine rejecting old timestamp |
| `phase2b_space.txt` | Terminal output: Space Engine resolving altitude clash |
| `phase2c_tracking.txt` | Terminal output: Tracking Engine merging ghost IDs |
| `phase2d_trust.txt` | Terminal output: Trust Engine blocking untrusted source |
| `final_consensus.txt` | Terminal output: Fused world state table |

## Key Equations

### Slide 2 - Time Engine (Lamport Clock)
```
T_new = max(T_local, T_msg) + 1
Constraint: REJECT if T_msg < T_local
```

### Slide 3 - Trust Engine (Beta Distribution)
```
Trust = α / (α + β)
```

## Visual Instructions

- **Color scheme**: Dark tech theme, primary cyan (#06B6D4), success green (#22C55E), reject red (#EF4444)
- **Slide 1**: Show 3 sensors disagreeing, brain overloaded with "CONFLICTING REALITY"
- **Slide 2**: Swimlane diagram with "Causal Wall" blocking old timestamps
- **Slide 3**: Split screen - left shows 3D voxel stack, right shows Beta distribution curves

## Demo Proof

The terminal outputs show the system working in real-time:
- Rejected packet: car_002 (LTS 95 < T_local 103)
- Disambiguated: drone at Voxel 5, car at Voxel 0
- Merged: car_ghost_A absorbed car_ghost_B
- Blocked: UNKNOWN_SRC (9% trust < 50% threshold)

## Use This Prompt for Gamma/Tome

"Create a 3-slide technical pitch deck for Godview, a system that solves the Byzantine Generals Problem for autonomous vehicles. Use the slide content from slides_content.md. Dark futuristic theme with cyan accents. Include the math formulas."
