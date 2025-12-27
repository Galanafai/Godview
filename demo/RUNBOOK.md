# GodView CARLA Demo - RUNBOOK

## Overview
This runbook produces a broadcast-quality "Before vs. After" demo video demonstrating GodView's resolution of:
- **OOSM** (Out-of-Sequence Measurements)
- **Pancake World** (2D flattening of 3D data)
- **Ghost/Sybil Attacks** (Identity conflicts and malicious injection)

---

## Prerequisites

### RunPod Instance
- **GPU**: RTX 4090 (24GB VRAM) - Secure Cloud
- **Template**: PyTorch 2.1.0 / CUDA 11.8
- **Volume**: 50GB mounted at `/workspace`

### Installed Dependencies
```bash
pip install carla==0.9.13 numpy opencv-python
apt-get install -y ffmpeg wget nano
```

---

## Directory Structure
```
/workspace/godview_demo/
├── scripts/
│   ├── scenario_runner.py   # The Director
│   ├── generate_logs.py     # The Noise Maker  
│   ├── render_cameras.py    # The Cinematographer
│   └── compose_video.py     # The Editor
├── logs/
│   ├── godview_demo.log     # CARLA binary recording
│   ├── ground_truth.ndjson  # Raw actor positions
│   ├── raw_broken.ndjson    # Faulty sensor data
│   ├── godview_merged.ndjson# Corrected data
│   └── merge_events.ndjson  # GodView decisions
├── frames/
│   ├── pass1_ego/           # Chase camera frames
│   ├── pass2_drone/         # Top-down frames
│   └── pass3_cinematic/     # Sweeping view frames
└── outputs/
    ├── video_before.mp4     # Raw sensor chaos
    ├── video_after.mp4      # GodView corrected
    └── final_linkedin.mp4   # Split-screen comparison
```

---

## Execution Checklist

### Step 1: Start CARLA Server (Headless)
```bash
# Pull and run CARLA Docker container
docker run -d \
  --name carla-server \
  --gpus all \
  -p 2000-2002:2000-2002 \
  -v /workspace/godview_demo/logs:/home/carla/recordings \
  carlasim/carla:0.9.13 \
  ./CarlaUE4.sh -RenderOffScreen -quality-level=Low -benchmark -fps=20

# Wait for startup (check logs)
sleep 30
docker logs carla-server | tail -20
```

### Step 2: Run Simulation (The Director)
```bash
cd /workspace/godview_demo/scripts
python3 scenario_runner.py
# Expected: ~2-3 minutes for 600 frames
# Output: /workspace/godview_demo/logs/godview_demo.log
```

### Step 3: Generate Log Files (The Noise Maker)
```bash
python3 generate_logs.py
# Expected: ~10 seconds
# Output: raw_broken.ndjson, godview_merged.ndjson, merge_events.ndjson
```

### Step 4: Restart CARLA with Epic Quality
```bash
# Stop low-quality server
docker stop carla-server && docker rm carla-server

# Start high-quality server for rendering
docker run -d \
  --name carla-server \
  --gpus all \
  -p 2000-2002:2000-2002 \
  -v /workspace/godview_demo/logs:/home/carla/recordings \
  carlasim/carla:0.9.13 \
  ./CarlaUE4.sh -RenderOffScreen -quality-level=Epic

sleep 30
```

### Step 5: Render Camera Passes (The Cinematographer)
```bash
# Render all passes (takes ~15-30 min)
python3 render_cameras.py --pass all --frames 600

# Or render individual passes:
# python3 render_cameras.py --pass pass2_drone --frames 600
```

### Step 6: Compose Final Video (The Editor)
```bash
# Create split-screen comparison video
python3 compose_video.py --mode split

# Or create individual videos:
# python3 compose_video.py --mode before
# python3 compose_video.py --mode after
```

### Step 7: Download Output
```bash
# Files are at:
# /workspace/godview_demo/outputs/final_linkedin.mp4

# Download via SCP:
# scp -P 31522 root@38.80.152.248:/workspace/godview_demo/outputs/final_linkedin.mp4 ./
```

---

## Troubleshooting

### Black Frames
- **Cause**: CARLA not rendering properly
- **Fix**: Ensure `-RenderOffScreen` flag is set
- **Check**: `docker logs carla-server | grep -i render`

### Out of Video Memory (OOM)
- **Cause**: Too many cameras or high resolution
- **Fix**: 
  1. Reduce resolution in `render_cameras.py` (1280x720)
  2. Render one pass at a time
  3. Switch to RTX A6000 (48GB)

### FFmpeg libx264 Errors
- **Cause**: Missing codec or wrong pixel format
- **Fix**: Ensure ffmpeg is installed with libx264:
  ```bash
  apt-get install -y ffmpeg libx264-dev
  ```

### CARLA Connection Refused
- **Cause**: Docker container not running
- **Fix**: 
  ```bash
  docker ps  # Check if running
  docker start carla-server  # Restart if needed
  ```

### Simulation Crashes
- **Cause**: Town10HD too heavy for memory
- **Fix**: Use simpler map in scenario_runner.py:
  ```python
  world = client.load_world("Town03")  # Lighter map
  ```

---

## HUD Reference

The final video includes these GodView-specific terms:

| Term | Meaning |
|------|---------|
| **Highlander Consensus** | Deterministic ID merge (min-UUID wins) |
| **H3 Spatial Grid** | Uber H3 hexagonal indexing for fast neighbor lookup |
| **Ed25519 Verified** | Cryptographic signature validation |
| **AugmentedStateFilter** | Kalman filter with history replay for OOSM |
| **SecurityContext** | CapBAC-based access control |

---

## Output Verification

Final video should show:
- [x] Split screen: RED boxes (chaos) vs GREEN boxes (stable)
- [x] Drones at correct altitude on GREEN side
- [x] No ghost duplicates on GREEN side
- [x] "SYBIL ATTACK!" labels on RED side, absent on GREEN
- [x] Scrolling event log showing merges and rejections
- [x] Status badges: CRITICAL (red) vs STABLE (green)

---

## Budget Estimate
- RTX 4090 at $0.50/hr
- Simulation: ~5 min = $0.05
- Rendering: ~30 min = $0.25
- Compositing: ~10 min = $0.10
- **Total: ~$0.50 - $1.00**
