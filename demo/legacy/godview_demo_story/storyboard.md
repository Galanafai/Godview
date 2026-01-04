# Storyboard & Shot List

**Duration**: 85 seconds  
**FPS**: 30  
**Total Frames**: 2550

---

## Phase A: THE HOOK
**Time**: 0–5s | **Frames**: 0–150

| Beat | Frames | World View | Network View | Shared vs Not Shared | Caption |
|------|--------|------------|--------------|----------------------|---------|
| **A1** | 0–60 | Black → fade in grid + 20 dim agent icons | Single node pulses | Panel visible, static | *"20 agents."* |
| **A2** | 60–120 | One agent brightens, detection box appears | One packet dot travels L→R | "Video stream" gets ❌ | *"No video streaming."* |
| **A3** | 120–150 | Stable consensus box | Packet arrives at destination node | All shared fields ✅ | *"Shared world model."* |

**Spotlight Agents**: `agent_00`, `agent_01`, `agent_02`

---

## Phase B: BEFORE — Isolated Perception
**Time**: 5–25s | **Frames**: 150–750  
**Color Mode**: Red/Cyan (chaos)

### Beat B1: Occlusion Blind Spot
**Time**: 5–12s | **Frames**: 150–360

| Frames | World View | Network View | Caption |
|--------|------------|--------------|---------|
| 150–200 | Agent A (00) sees pedestrian, green box | Packets from agent_00 | *"Agent A sees pedestrian."* |
| 200–300 | Agent B (01) approaches, NO box (blind) | Packets from agent_01 (no ped) | *"Agent B is blind."* |
| 300–360 | Truck occlusion visualized (grey box between) | — | *"PROBLEM: Blind spots."* |

**Spotlight Agents**: `agent_00`, `agent_01`

---

### Beat B2: Duplicate Ghosts
**Time**: 12–18s | **Frames**: 360–540

| Frames | World View | Network View | Caption |
|--------|------------|--------------|---------|
| 360–400 | Target car appears | — | — |
| 400–480 | TWO boxes appear (slightly offset, flickering) | Packets from agent_02, agent_03 | *"Two detections. Same object."* |
| 480–540 | Boxes labeled "ID:A", "ID:B", unstable | — | *"PROBLEM: Ghost duplicates."* |

**Spotlight Agents**: `agent_02`, `agent_03`

---

### Beat B3: Untrusted Data
**Time**: 18–25s | **Frames**: 540–750

| Frames | World View | Network View | Caption |
|--------|------------|--------------|---------|
| 540–600 | Normal scene | Unknown node appears (red) | — |
| 600–680 | FAKE BUS appears (red box, label "SPOOF") | Red packet travels from unknown_x | *"Spoofed packet accepted."* |
| 680–750 | Fake bus persists, looks identical to real | — | *"PROBLEM: No trust."* |

**Spotlight Agents**: `agent_04`, `unknown_x`

---

## Phase C: AFTER — Decentralized Collaboration
**Time**: 25–55s | **Frames**: 750–1650  
**Color Mode**: Green (stable)

### Beat C1: Remote Observation
**Time**: 25–35s | **Frames**: 750–1050

| Frames | World View | Network View | Caption |
|--------|------------|--------------|---------|
| 750–800 | Transition scanline sweeps down | All nodes turn green | *"ACTIVATING GODVIEW"* |
| 800–900 | Agent B still can't see ped locally | Packet from drone_00 travels to agent_01 | *"Drone shares observation."* |
| 900–1050 | Pedestrian appears for B (remote box, dotted) | — | *"B sees around corners."* |

**Spotlight Agents**: `agent_00`, `agent_01`, `drone_00`

---

### Beat C2: Ghost Merge
**Time**: 35–45s | **Frames**: 1050–1350

| Frames | World View | Network View | Caption |
|--------|------------|--------------|---------|
| 1050–1100 | Two ghost boxes visible | — | — |
| 1100–1140 | Boxes slide towards each other | Merge event marker | *"Merging duplicates..."* |
| **1140** | **MERGE EVENT** (flash) | — | — |
| 1140–1350 | Single stable box, thick outline | — | *"One canonical ID."* |

**Spotlight Agents**: `agent_02`, `agent_03`  
**Event**: `MERGE` at frame 1140

---

### Beat C3: Trust Rejection
**Time**: 45–55s | **Frames**: 1350–1650

| Frames | World View | Network View | Caption |
|--------|------------|--------------|---------|
| 1350–1450 | Normal scene, no fake bus | Unknown_x sends red packet | *"Spoof attempt..."* |
| **1500** | **TRUST_REJECT EVENT** | Red packet hits wall / dissolves | *"REJECTED: Invalid signature."* |
| 1500–1650 | Scene remains clean | — | *"Provenance verified."* |

**Spotlight Agents**: `agent_04`, `unknown_x`  
**Event**: `TRUST_REJECT` at frame 1500

---

## Phase D: ENGINE MONTAGE
**Time**: 55–75s | **Frames**: 1650–2250

### Beat D1: OOSM Correction
**Time**: 55–60s | **Frames**: 1650–1800

| Frames | World View | Network View | Caption |
|--------|------------|--------------|---------|
| 1650–1700 | Agent_07 detection arrives late (ghost trail) | Delayed packet animation | *"Late packet arrives..."* |
| **1740** | **OOSM_CORRECTED EVENT** | — | — |
| 1740–1800 | Track snaps to correct position, trail fades | — | *"State corrected."* |

**Spotlight Agents**: `agent_07`  
**Event**: `OOSM_CORRECTED` at frame 1740

---

### Beat D2: Space Engine (3D)
**Time**: 60–65s | **Frames**: 1800–1950

| Frames | World View | Network View | Caption |
|--------|------------|--------------|---------|
| 1800–1860 | Drone and car share same x,y | — | *"Same location?"* |
| **1860** | **SPACE_SEPARATION EVENT** | — | — |
| 1860–1950 | Side view shows Z separation (drone at altitude) | — | *"No. 15m altitude difference."* |

**Spotlight Agents**: `agent_03`, `drone_00`  
**Event**: `SPACE_SEPARATION` at frame 1860

---

### Beat D3: Tracking Handoff
**Time**: 65–70s | **Frames**: 1950–2100

| Frames | World View | Network View | Caption |
|--------|------------|--------------|---------|
| 1950–2000 | Object tracked by agent_05 | Packets from agent_05 | — |
| 2000–2010 | **HANDOFF_OK EVENT** | Handoff marker | — |
| 2010–2100 | Object now tracked by agent_06, same ID | Packets from agent_06 | *"Stable ID through handoff."* |

**Event**: `HANDOFF_OK` at frame 2010

---

### Beat D4: Bandwidth
**Time**: 70–75s | **Frames**: 2100–2250

| Frames | World View | Caption |
|--------|------------|---------|
| 2100–2250 | Counter: "SHARED: 12 KB/s" | *"1000× less bandwidth."* |

**Shared vs Not Shared**: Highlight "VIDEO STREAM ❌ 50 MB/s" vs "PACKETS ✅ 12 KB/s"

---

## Phase E: CLOSE
**Time**: 75–85s | **Frames**: 2250–2550

| Frames | World View | Network View | Caption |
|--------|------------|--------------|---------|
| 2250–2400 | All 20 agents visible, stable tracks | Gentle packet flow | *"godview_core"* |
| 2400–2550 | Fade to logo / dark | — | *"Decentralized. Verifiable. Efficient."* |

**Spotlight Agents**: `agent_00`, `agent_01`, `agent_02`
