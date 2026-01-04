# Failure Modes & Acceptance Checklist

This checklist defines **strict visual outcomes** for each failure mode demonstrated in the video.

---

## 1. Occlusion Blind Spot (Frames 150–360)

### Expected Behavior
- **Agent A (`agent_00`)**: MUST show a detection box on the pedestrian.
- **Agent B (`agent_01`)**: MUST NOT show a detection box on the pedestrian during BEFORE phase.
- **Truck occluder**: MUST be visible as a grey box between agent_01 and pedestrian.

### Failure Conditions
| Failure | How to Detect |
|---------|---------------|
| Agent B sees pedestrian in BEFORE | `packets.ndjson` contains `agent_01` with `ped_01` detection in frames 150–360 |
| No truck visible | Truck box missing from world view |
| Agent A doesn't see pedestrian | No detection from `agent_00` in relevant frames |

---

## 2. Duplicate Ghosts (Frames 360–540)

### Expected Behavior
- **Two distinct boxes** for the same target car, offset by ~0.5m.
- Boxes should **flicker** (slight position jitter each frame).
- Labels show different IDs: `agent_02_car_target_01` and `agent_03_car_target_01`.

### Failure Conditions
| Failure | How to Detect |
|---------|---------------|
| Only one box appears | Missing detection from either `agent_02` or `agent_03` |
| Boxes perfectly overlap | Position noise < 0.1m |
| No flicker | Position variance = 0 across frames |

---

## 3. Out-of-Sequence Messages (Frames 1650–1800)

### Expected Behavior
- **Delayed packets**: `agent_07` detection packets have `delivery_frame > frame` during this beat.
- **Ghost trail**: Before correction, track shows position lag ("old" position fading out).
- **Snap correction**: After `OOSM_CORRECTED` event (frame 1740), track snaps to correct position.

### Failure Conditions
| Failure | How to Detect |
|---------|---------------|
| No delay modeled | All `agent_07` packets have `delivery_frame == frame` |
| No correction event | Missing `OOSM_CORRECTED` in `events.ndjson` |
| Track doesn't visually correct | No position jump at frame 1740 |

---

## 4. Trust Rejection (Frames 1350–1650)

### Expected Behavior
- **Spoof agent (`unknown_x`)**: Packets with `signature_valid: false`.
- **BEFORE phase**: Fake bus appears in world view (red box, "SPOOF" label).
- **AFTER phase**: Fake bus DOES NOT appear. `TRUST_REJECT` event fires at frame 1500.

### Failure Conditions
| Failure | How to Detect |
|---------|---------------|
| Spoof accepted in AFTER | `world_state.ndjson` contains `spoof_bus_01` after frame 750 |
| No rejection event | Missing `TRUST_REJECT` in `events.ndjson` |
| Spoof packet not marked invalid | `signature_valid: true` for `unknown_x` |

---

## 5. Drone vs Car "Pancake" Separation (Frames 1800–1950)

### Expected Behavior
- **Drone (`drone_00`)**: Position.z ≈ 15m.
- **Car (`agent_03`)**: Position.z ≈ 0m.
- **Same x,y**: Both share approximate x,y coordinates at some point.
- **Separation event**: `SPACE_SEPARATION` event at frame 1860 with `delta_z: 15.0`.

### Failure Conditions
| Failure | How to Detect |
|---------|---------------|
| Drone at ground level | `drone_00` position.z < 5m |
| No z overlap | Drone and car never share x,y within 2m |
| No separation event | Missing `SPACE_SEPARATION` in `events.ndjson` |
| Pancake visualization | Both rendered at same z in TouchDesigner (TD bug, not log bug) |

---

## 6. Ghost Merge (Frames 1050–1350)

### Expected Behavior
- **Pre-merge**: Two boxes visible for same object.
- **Merge event**: `MERGE` event at frame 1140.
- **Post-merge**: Single stable box with `canonical_object_id`.

### Failure Conditions
| Failure | How to Detect |
|---------|---------------|
| No merge event | Missing `MERGE` in `events.ndjson` |
| Two boxes persist after merge | Rendering bug (TD) or incorrect canonical state generation |
| Merge happens without visual transition | No implosion animation (TD implementation issue) |

---

## Validation Script (Optional)

Run this after generating logs:

```bash
# Check for required events
grep -c "MERGE" out/events.ndjson        # Should be >= 1
grep -c "TRUST_REJECT" out/events.ndjson # Should be >= 1
grep -c "OOSM_CORRECTED" out/events.ndjson # Should be >= 1
grep -c "SPACE_SEPARATION" out/events.ndjson # Should be >= 1

# Check for spoof packets
grep "unknown_x" out/packets.ndjson | grep -c "signature_valid.*false" # Should be >= 1

# Check drone altitude
grep "drone_00" out/packets.ndjson | head -1 | grep -o '"z":[0-9.]*' # Should show z > 10
```
