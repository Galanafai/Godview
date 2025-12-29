# Failure Modes & Acceptance Checklist

---

## 1. Viewer Understands Thesis in 5 Seconds

| Check | Pass Criteria |
|-------|---------------|
| ✅ Main caption visible | "20 agents share object packets" readable on mobile |
| ✅ Pane layout obvious | 3 panes clearly separated |
| ✅ No clutter | Object budget enforced (≤12 visible) |

---

## 2. Occlusion Proof Works

| Check | Pass Criteria |
|-------|---------------|
| ✅ Building occluder visible | Grey rectangle in world view |
| ✅ Truth silhouette shown | Faint "OCCLUDED" label for ped_01 during BEFORE |
| ✅ Agent B blind in BEFORE | No ped_01 detection from agent_01 |
| ✅ Agent B sees in AFTER | ped_01 appears after packet arrival |

---

## 3. Packet Arrival Causes Object Appearance

| Check | Pass Criteria |
|-------|---------------|
| ✅ PACKET_ARRIVAL event exists | In events.ndjson |
| ✅ Arrow/beam drawn | From network node toward world object |
| ✅ Timing correct | Object appears AFTER packet arrives |

---

## 4. Merge Collapse Obvious

| Check | Pass Criteria |
|-------|---------------|
| ✅ Two ghosts visible before merge | Labeled with different IDs |
| ✅ MERGE event fires | In events.ndjson at correct frame |
| ✅ Boxes collapse to one | Single canonical box after merge |
| ✅ Pulse ring visible | Visual feedback at merge moment |

---

## 5. Spoof Reject Obvious

| Check | Pass Criteria |
|-------|---------------|
| ✅ Spoof packet has signature_valid=false | In packets.ndjson |
| ✅ Spoof appears in BEFORE | Bus visible (magenta) |
| ✅ TRUST_REJECT event fires | In events.ndjson |
| ✅ Spoof blocked in AFTER | No spoof_bus_01 in world_state after phase change |
| ✅ Shield visual shown | In network pane during trust_reject beat |

---

## 6. OOSM Tag Visible

| Check | Pass Criteria |
|-------|---------------|
| ✅ delivery_frame > frame for agent_07 | In packets.ndjson during OOSM beat |
| ✅ OOSM_CORRECTED event fires | In events.ndjson |
| ✅ "+N frames" label visible | In network or world pane |

---

## 7. 20 Agents Visible But Not Cluttered

| Check | Pass Criteria |
|-------|---------------|
| ✅ 21 nodes in network | 20 agents + 1 drone |
| ✅ Spotlight agents bright | 3–5 agents highlighted per beat |
| ✅ Non-spotlight agents dim | Grey or low opacity |

---

## 8. Object Budget Enforced

| Check | Pass Criteria |
|-------|---------------|
| ✅ Max 12 objects rendered | Per frame in world view |
| ✅ Ranking by importance | Spotlight zone > near spotlight agents > crossing risk |

---

## Validation Commands

```bash
# Check events exist
grep -c "MERGE" out/events.ndjson          # >= 1
grep -c "TRUST_REJECT" out/events.ndjson   # >= 1
grep -c "OOSM_CORRECTED" out/events.ndjson # >= 1
grep -c "PACKET_ARRIVAL" out/events.ndjson # >= 1

# Check spoof packets invalid
grep "unknown_x" out/packets.ndjson | grep -c "signature_valid.*false"  # >= 1

# Check OOSM delay
grep "agent_07" out/packets.ndjson | head -5  # delivery_frame > frame during beat
```
