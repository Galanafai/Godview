# 👁️ GodView - The Live Reality Protocol

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Rust](https://img.shields.io/badge/Rust-2021-orange.svg)](https://www.rust-lang.org/)
[![Three.js](https://img.shields.io/badge/Three.js-0.160-blue.svg)](https://threejs.org/)
[![Zenoh](https://img.shields.io/badge/Zenoh-1.0-green.svg)](https://zenoh.io/)

**Distributed X-Ray Vision System for Industrial Safety**

GodView is a real-time hazard detection and visualization system that decouples sight from location. It turns **video streams** into **semantic 3D coordinates**, creating a privacy-preserving safety system with **<50ms latency** on standard WiFi.

![GodView Demo](https://via.placeholder.com/800x400/000000/00ff00?text=GodView+Demo)

---

## ⚡ The Breakthrough

Traditional surveillance is broken:
- ❌ **High Bandwidth:** 50 cameras = 200 Mbps
- ❌ **Privacy Risk:** Stores raw video of people
- ❌ **Dumb Data:** "Pixel 500,500 is red" (Meaningless)

GodView changes the game:
- ✅ **Low Bandwidth:** 50 cameras = 1.5 MB/s (99.25% reduction)
- ✅ **True Privacy:** Transmits coordinates, not faces
- ✅ **Smart Data:** "Human at lat/lon/alt" (Actionable)

---

## 🏗️ System Architecture

```
┌──────────────────┐         ┌──────────────┐         ┌──────────────────┐
│   RUST AGENT     │         │    ZENOH     │         │   WEB VIEWER     │
│  (X-Ray Emit)    │────────▶│   ROUTER     │────────▶│   (God View)     │
│                  │         │   (v1.0)     │         │                  │
│ • OpenCV         │         │ WS:8000      │         │ • Three.js       │
│ • Face Detect    │         │ TCP:7447     │         │ • Zenoh-TS       │
│ • 3D Projection  │         │              │         │ • Red Ghost      │
└──────────────────┘         └──────────────┘         └──────────────────┘
```

---

## 📖 Documentation

We believe in "PhD-level work, made accessible."

| Document | Description |
|----------|-------------|
| **[TECHNICAL_DOCUMENTATION.md](TECHNICAL_DOCUMENTATION.md)** | **Deep Dive.** The math, AS-EKF sensor fusion, and H3 indexing details. |
| **[WHY_REVOLUTIONARY.md](WHY_REVOLUTIONARY.md)** | **The Vision.** Why this beats Waymo/Tesla at this specific task. |
| **[CARLA_INTEGRATION_PLAN.md](CARLA_INTEGRATION_PLAN.md)** | **Simulation.** How we prove it works before deployment. |
| **[PROJECT_STATUS.md](PROJECT_STATUS.md)** | **Roadmap.** What's built, wha's next. |

---

## 🚀 Quick Start

### 1. Installation
```bash
# Clone and install dependencies
git clone https://github.com/Galanafai/Hivemind.git
cd Hivemind
./install_dependencies.sh
source ~/.cargo/env
```

### 2. Run It
```bash
# Start the full stack (Router + Agent + Viewer)
./run_godview.sh

# Open http://localhost:5173
```

Position yourself in front of the webcam. You will see a **red sphere** tracking you in 3D space. That sphere is a "Ghost" - a semantic representation of you, transmitted over the network!

---

## 🧪 Simulation Mode (CARLA)

Don't have 50 webcams? Use our CARLA simulator bridge:

```bash
# Setup and run CARLA bridge
./setup_carla_integration.sh
python3 carla_bridge/godview_carla_bridge.py --duration 60
```

---

## 🤝 Contributing

We are looking for Rustaceans and Three.js wizards.

1.  Read the **[Technical Documentation](TECHNICAL_DOCUMENTATION.md)** to understand the math.
2.  Pick an issue from our roadmap.
3.  Submit a PR.

**License:** MIT
