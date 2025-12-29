# Failure Modes - 30s Demo

---

## 1. Thesis Understandable in 5s

| Check | Criteria |
|-------|----------|
| ✅ Caption readable | "No video streaming" visible on mobile |
| ✅ Layout stable | 3 panes always visible |

---

## 2. Occlusion Proof

| Check | Criteria |
|-------|----------|
| ✅ Building visible | Grey rectangle labeled "BUILDING" |
| ✅ Truth silhouette | Faint "P (truth)" in both halves |
| ✅ BEFORE: B blind | No pedestrian for B |
| ✅ AFTER: B sees | Pedestrian appears after packet |
| ✅ Packet animation | Dot travels A→B in network |

---

## 3. Merge Obvious

| Check | Criteria |
|-------|----------|
| ✅ BEFORE: 2 IDs | Two dots labeled "ID:0", "ID:1" |
| ✅ AFTER: 1 ID | One box labeled "FUSED" |
| ✅ MERGE event | Visible in event log |

---

## 4. Trust Reject Obvious

| Check | Criteria |
|-------|----------|
| ✅ BEFORE: spoof visible | Magenta object |
| ✅ AFTER: spoof blocked | Shield animation, no object |
| ✅ TRUST_REJECT event | Visible in event log |

---

## 5. Scale Reveal

| Check | Criteria |
|-------|----------|
| ✅ 20 nodes light up | In network pane |
| ✅ World stays calm | No clutter |

---

## Validation

```bash
# Check events
grep -c "PACKET_ARRIVAL" out/events_after.ndjson  # >= 1
grep -c "MERGE" out/events_after.ndjson           # >= 1
grep -c "TRUST_REJECT" out/events_after.ndjson    # >= 1
```
