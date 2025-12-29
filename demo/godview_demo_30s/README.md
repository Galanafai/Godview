# GodView 30s Demo

**Ubuntu 22.04 | 30 seconds | Split-screen BEFORE vs AFTER**

## Quick Start

```bash
sudo apt install ffmpeg
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
python3 build_demo.py
```

**Output:** `out/final_godview_30s.mp4` (1920×1080, 30 FPS)

---

## Story (30 seconds)

| Beat | Time | Content |
|------|------|---------|
| Thesis | 0–3s | "No video streaming. Just object packets." |
| Occlusion | 3–12s | Split: B blind BEFORE, sees via packet AFTER |
| Merge | 12–20s | Split: 2 IDs BEFORE, 1 canonical AFTER |
| Trust | 20–26s | Split: Spoof accepted BEFORE, rejected AFTER |
| Scale | 26–30s | 20 agents light up, same mechanism |

---

## Visualization Rules

- **BEFORE** (left, red banner): Raw detections, no fusion, spoof accepted
- **AFTER** (right, green banner): Fused, merged, trust-filtered
- **Objects**: Dot + uncertainty ring (raw), Box (fused)
- **Causality**: Packet dot travels in network, arrow points to appearing object
