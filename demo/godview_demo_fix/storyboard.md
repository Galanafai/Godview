# Storyboard - GodView Demo Fix

**Duration:** 85 seconds | **FPS:** 30 | **Total Frames:** 2550

---

## Phase A: HOOK (0–5s, Frames 0–150)

| Frames | Spotlight | World View | Network | Caption |
|--------|-----------|------------|---------|---------|
| 0–50 | A00, A01, D00 | Fade in, grid + 20 agents | All nodes visible | — |
| 50–150 | A00, A01, D00 | Objects appear | Packet dots flow | "20 agents share object packets. No video streaming." |

---

## Phase B: BEFORE (5–28s, Frames 150–840)

### Beat B1: Occlusion (5–14s, Frames 150–420)
**Split-Screen:** Frames 300–420 (10–14s)

| Frames | Spotlight | World View | Caption |
|--------|-----------|------------|---------|
| 150–300 | A00, A01 | A00 sees ped, A01 blocked by building | "PROBLEM: Agent B cannot see occluded pedestrian" |
| 300–420 | A00, A01 | **SPLIT:** BEFORE (ped missing for A01) vs AFTER (ped visible) | Banner: "BEFORE vs AFTER" |

### Beat B2: Ghosts (14–21s, Frames 420–630)
**Split-Screen:** Frames 510–630 (17–21s)

| Frames | Spotlight | World View | Caption |
|--------|-----------|------------|---------|
| 420–510 | A02, A03 | Two boxes for same car, slight offset | "PROBLEM: Same object, different IDs = ghost duplicates" |
| 510–630 | A02, A03 | **SPLIT:** BEFORE (2 ghosts) vs AFTER (1 canonical) | — |

### Beat B3: Spoof (21–28s, Frames 630–840)
**Split-Screen:** Frames 720–840 (24–28s)

| Frames | Spotlight | World View | Caption |
|--------|-----------|------------|---------|
| 630–720 | A04, unknown_x | Spoof bus appears | "PROBLEM: Spoofed packet looks identical" |
| 720–840 | A04 | **SPLIT:** BEFORE (spoof visible) vs AFTER (blocked) | — |

---

## Phase C: AFTER (28–55s, Frames 840–1650)

### Beat C1: Remote Observation (28–38s, Frames 840–1140)

| Frames | Spotlight | World View | Network | Caption |
|--------|-----------|------------|---------|---------|
| 840–900 | A00, A01, D00 | Transition sweep | All green | — |
| 900–960 | A00, A01, D00 | Packet travels D00→A01 | Arrow to world | "SOLUTION: Packet arrives → pedestrian visible to B" |
| 960–1140 | A00, A01, D00 | Ped now visible for A01 | — | — |

**Event:** PACKET_ARRIVAL at frame 960

### Beat C2: Merge (38–48s, Frames 1140–1440)

| Frames | Spotlight | World View | Caption |
|--------|-----------|------------|---------|
| 1140–1200 | A02, A03, D00 | Two ghosts visible | "Merging duplicates..." |
| 1200–1260 | A02, A03, D00 | Boxes slide together | — |
| **1260** | — | **MERGE EVENT** - pulse ring | — |
| 1260–1440 | A02, A03, D00 | One canonical box | "SOLUTION: Duplicates merge into one canonical track" |

**Event:** MERGE at frame 1260

### Beat C3: Trust Reject (48–55s, Frames 1440–1650)

| Frames | Spotlight | Network | Caption |
|--------|-----------|---------|---------|
| 1440–1500 | A04 | Spoof packet approaches | — |
| 1500–1560 | A04 | Shield blocks, packet dissolves | "SOLUTION: Invalid signature → packet rejected" |
| **1560** | — | **TRUST_REJECT EVENT** | — |

**Event:** TRUST_REJECT at frame 1560

---

## Phase D: MONTAGE (55–75s, Frames 1650–2250)

### Beat D1: OOSM (55–62s, Frames 1650–1860)

| Frames | Spotlight | Visual | Caption |
|--------|-----------|--------|---------|
| 1650–1770 | A07 | Delayed packet label "+12 frames" | "ENGINE: Late packet (+12 frames) corrected" |
| **1770** | — | **OOSM_CORRECTED EVENT** | — |

### Beat D2: Space (62–68s, Frames 1860–2040)

| Frames | Spotlight | Visual | Caption |
|--------|-----------|--------|---------|
| 1860–1950 | A03, D00 | Drone z=15m label, car z=0 | "ENGINE: Drone at z=15m, car at z=0 → no pancake" |
| **1950** | — | **SPACE_SEPARATION EVENT** | — |

### Beat D3: Bandwidth (68–75s, Frames 2040–2250)

| Frames | Spotlight | Visual | Caption |
|--------|-----------|--------|---------|
| 2040–2250 | A00 | Bandwidth counter | "1000× less bandwidth than video streaming" |

---

## Phase E: CLOSE (75–85s, Frames 2250–2550)

| Frames | Spotlight | World View | Caption |
|--------|-----------|------------|---------|
| 2250–2400 | A00, A01, A02 | Stable traffic, gentle packet flow | "godview_core" |
| 2400–2550 | A00, A01, A02 | Fade to dark | "decentralized fusion with provenance" |
