# GodView Demo Fix

**Ubuntu 22.04 | Python 3.10+ | No GUI Required**

A fixed, story-first demo with clear causality visualization.

## Quick Start

```bash
# Install ffmpeg
sudo apt install ffmpeg

# Create venv and install
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt

# Build demo
python3 build_demo.py
```

**Output:** `out/final_godview_demo_fixed.mp4` (1920×1080, 30 FPS, 85s)

---

## Improvements Over Previous Demo

| Issue | Fix |
|-------|-----|
| No causality visible | Packet arrival → object appears with arrow |
| Cluttered objects | Object budget (max 12 visible) |
| Before/After unclear | Split-screen comparisons for key beats |
| Occlusion not proven | Building occluder + truth silhouette |
| Events subtle | Merge pulse, trust shield, OOSM tag |

---

## Story Structure (85s)

| Phase | Time | Content |
|-------|------|---------|
| Hook | 0–5s | "20 agents share object packets" |
| BEFORE | 5–28s | Occlusion, ghosts, spoof (split-screen) |
| AFTER | 28–55s | Remote observation, merge, trust reject |
| Montage | 55–75s | OOSM, space, bandwidth |
| Close | 75–85s | "Decentralized fusion with provenance" |

---

## Files

| File | Purpose |
|------|---------|
| `build_demo.py` | One-command build |
| `generate_demo_logs.py` | NDJSON generator |
| `render_frames.py` | OpenCV renderer |
| `encode_video.py` | FFmpeg encoder |
