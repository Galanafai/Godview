# GodView Demo Auto-Build

**Ubuntu 22.04 + Python 3.10+ | No GUI Required**

One-command build that generates a LinkedIn-ready MP4 demo.

## Quick Start

```bash
# Install system dependency
sudo apt install ffmpeg

# Create virtual environment
python3 -m venv .venv
source .venv/bin/activate

# Install Python dependencies
pip install -r requirements.txt

# Build demo (generates ~85s video)
python3 build_demo.py --out ./out
```

**Output:** `out/final_godview_demo.mp4` (1920×1080, 30 FPS)

---

## Pipeline Steps

| Step | Script | Output |
|------|--------|--------|
| 1. Generate Logs | `generate_demo_logs.py` | `packets.ndjson`, `world_state.ndjson`, `events.ndjson` |
| 2. Render Frames | `render_frames.py` | `frames/frame_00000.png` ... |
| 3. Encode Video | `encode_video.py` | `final_godview_demo.mp4` |

---

## CLI Arguments

```bash
python3 build_demo.py \
  --out ./out \           # Output directory
  --seed 42 \             # Random seed for determinism
  --duration_s 85 \       # Duration in seconds
  --fps 30 \              # Frames per second
  --num_agents 20         # Number of car agents (+ 1 drone)
```

**Skip steps:**
```bash
python3 build_demo.py --skip_logs    # Use existing logs
python3 build_demo.py --skip_render  # Use existing frames
```

---

## Determinism

Same seed = identical output:
```bash
python3 build_demo.py --seed 42 --out run1
python3 build_demo.py --seed 42 --out run2
diff run1/final_godview_demo.mp4 run2/final_godview_demo.mp4  # Identical
```

---

## Story Phases

| Phase | Time | Content |
|-------|------|---------|
| Hook | 0–5s | "20 agents. No video streaming." |
| BEFORE | 5–25s | Occlusion, ghosts, spoofing |
| AFTER | 25–55s | Remote observation, merge, trust reject |
| Montage | 55–75s | OOSM, 3D space, handoff, bandwidth |
| Close | 75–85s | "Decentralized fusion with provenance" |

---

## Files

| File | Purpose |
|------|---------|
| `build_demo.py` | Orchestrator |
| `generate_demo_logs.py` | NDJSON log generator |
| `render_frames.py` | OpenCV frame renderer |
| `encode_video.py` | FFmpeg MP4 encoder |
| `storyboard.md` | Beat-by-beat script |
| `schema.md` | NDJSON field definitions |
| `failure_modes.md` | Acceptance checklist |
