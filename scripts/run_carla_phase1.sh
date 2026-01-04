#!/bin/bash

# GodView + CARLA Phase 1 - Quick Start Script

set -e

echo "╔════════════════════════════════════════════╗"
echo "║   GODVIEW + CARLA PHASE 1 LAUNCHER         ║"
echo "╚════════════════════════════════════════════╝"
echo ""

# Check if CARLA is running
if ! pgrep -x "CarlaUE4" > /dev/null; then
    echo "❌ CARLA server not running!"
    echo ""
    echo "Start CARLA in another terminal:"
    echo "  cd ~/CARLA_0.9.15"
    echo "  ./CarlaUE4.sh -quality-level=Low -RenderOffScreen"
    echo ""
    exit 1
fi

echo "✅ CARLA server detected"
echo ""

# Check Python dependencies
echo "📦 Checking Python dependencies..."
if ! python3 -c "import carla" 2>/dev/null; then
    echo "❌ CARLA Python API not found!"
    echo ""
    echo "Add to PYTHONPATH:"
    echo "  export PYTHONPATH=\$PYTHONPATH:~/CARLA_0.9.15/PythonAPI/carla/dist/carla-0.9.15-py3.7-linux-x86_64.egg"
    echo ""
    exit 1
fi

if ! python3 -c "import ultralytics" 2>/dev/null; then
    echo "⚠️  YOLOv8 not installed. Installing..."
    pip install -r carla_bridge/requirements.txt
fi

echo "✅ Python dependencies OK"
echo ""

# Build Rust agent
echo "🔨 Building Rust agent..."
cd agent
cargo build --release
cd ..
echo "✅ Rust agent built"
echo ""

# Run the bridge
echo "🚀 Launching GodView CARLA Bridge..."
echo "════════════════════════════════════════════"
echo ""

python3 carla_bridge/godview_carla_bridge.py \
    --vehicles 3 \
    --duration 60

echo ""
echo "✅ Simulation complete!"
