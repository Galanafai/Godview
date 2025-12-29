# Storyboard - 30s Demo

**Duration:** 30 seconds | **FPS:** 30 | **Total Frames:** 900

---

## Beat 1: THESIS (0–3s, Frames 0–90)

| Visual | Description |
|--------|-------------|
| World pane | Full AFTER view, calm grid, agents in place |
| Network pane | 3 main nodes: A (blue), B (orange), Drone (green) |
| Caption | "No video streaming. Just object packets." |

---

## Beat 2: OCCLUSION (3–12s, Frames 90–360)

**Split-screen active**

| Side | Visual |
|------|--------|
| BEFORE (left, red) | Agent B cannot see pedestrian behind building |
| AFTER (right, green) | Packet travels A→B, arrow to pedestrian, "Remote from A" |

**Event:** PACKET_ARRIVAL at frame 200

---

## Beat 3: MERGE (12–20s, Frames 360–600)

**Split-screen active**

| Side | Visual |
|------|--------|
| BEFORE (left, red) | Two dots near same position: "ID:0", "ID:1" |
| AFTER (right, green) | One box: "FUSED" with reduced uncertainty ring |

**Event:** MERGE at frame 450

---

## Beat 4: TRUST (20–26s, Frames 600–780)

**Split-screen active**

| Side | Visual |
|------|--------|
| BEFORE (left, red) | Spoof pedestrian visible (magenta) |
| AFTER (right, green) | Spoof packet hits shield, never appears |

**Event:** TRUST_REJECT at frame 680

---

## Beat 5: SCALE (26–30s, Frames 780–900)

| Visual | Description |
|--------|-------------|
| Network pane | 20 nodes light up (scale agents) |
| World pane | Calm, same intersection |
| Caption | "Same mechanism scales to 20+ agents" |
