#!/bin/bash

# GodView + CARLA - Quick Setup Script

set -e

echo "╔════════════════════════════════════════════╗"
echo "║   GODVIEW + CARLA INTEGRATION SETUP        ║"
echo "╚════════════════════════════════════════════╝"
echo ""

# Check if CARLA is installed
if [ ! -d "$HOME/CARLA_0.9.15" ]; then
    echo "📥 CARLA not found. Download it from:"
    echo "   https://github.com/carla-simulator/carla/releases"
    echo ""
    echo "Or run:"
    echo "   wget https://carla-releases.s3.us-east-005.backblazeb2.com/Linux/CARLA_0.9.15.tar.gz"
    echo "   tar -xzf CARLA_0.9.15.tar.gz -C $HOME"
    echo ""
    exit 1
fi

echo "✅ CARLA found at $HOME/CARLA_0.9.15"
echo ""

# Create carla_bridge directory
echo "📁 Creating carla_bridge directory..."
mkdir -p carla_bridge/scenarios
mkdir -p carla_results/{metrics,visualizations,logs}

# Create Python requirements
cat > carla_bridge/requirements.txt << 'EOF'
# CARLA Python API (comes with CARLA installation)
# Add CARLA to PYTHONPATH: export PYTHONPATH=$PYTHONPATH:$HOME/CARLA_0.9.15/PythonAPI/carla/dist/carla-0.9.15-py3.7-linux-x86_64.egg

# Object Detection
ultralytics>=8.0.0  # YOLOv8
opencv-python>=4.8.0
numpy>=1.24.0

# Zenoh for pub/sub
eclipse-zenoh>=0.11.0

# Utilities
pyyaml>=6.0
matplotlib>=3.7.0
pandas>=2.0.0
EOF

echo "✅ Created requirements.txt"
echo ""

# Install Python dependencies
echo "📦 Installing Python dependencies..."
pip install -r carla_bridge/requirements.txt

# Add CARLA to PYTHONPATH
export PYTHONPATH=$PYTHONPATH:$HOME/CARLA_0.9.15/PythonAPI/carla/dist/carla-0.9.15-py3.7-linux-x86_64.egg
echo "export PYTHONPATH=\$PYTHONPATH:$HOME/CARLA_0.9.15/PythonAPI/carla/dist/carla-0.9.15-py3.7-linux-x86_64.egg" >> ~/.bashrc

echo "✅ Python dependencies installed"
echo ""

# Create basic bridge skeleton
cat > carla_bridge/godview_carla_bridge.py << 'EOF'
#!/usr/bin/env python3
"""
GodView CARLA Bridge
Connects CARLA simulator to GodView v3 agents
"""

import carla
import cv2
import numpy as np
import time
import json
import subprocess
from pathlib import Path

class GodViewCARLABridge:
    def __init__(self, carla_host='localhost', carla_port=2000):
        print("🔌 Connecting to CARLA...")
        self.client = carla.Client(carla_host, carla_port)
        self.client.set_timeout(10.0)
        self.world = self.client.get_world()
        print(f"✅ Connected to CARLA world: {self.world.get_map().name}")
        
        self.vehicles = []
        self.agents = []
    
    def spawn_vehicles(self, num_vehicles=3):
        print(f"\n🚗 Spawning {num_vehicles} vehicles...")
        blueprint_library = self.world.get_blueprint_library()
        vehicle_bp = blueprint_library.filter('vehicle.tesla.model3')[0]
        
        spawn_points = self.world.get_map().get_spawn_points()
        
        for i in range(min(num_vehicles, len(spawn_points))):
            vehicle = self.world.spawn_actor(vehicle_bp, spawn_points[i])
            vehicle.set_autopilot(True)  # Enable autopilot
            
            # Attach camera
            camera_bp = blueprint_library.find('sensor.camera.rgb')
            camera_bp.set_attribute('image_size_x', '1280')
            camera_bp.set_attribute('image_size_y', '720')
            camera_bp.set_attribute('fov', '90')
            
            camera_transform = carla.Transform(carla.Location(x=2.0, z=1.5))
            camera = self.world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)
            
            # Attach GPS
            gps_bp = blueprint_library.find('sensor.other.gnss')
            gps = self.world.spawn_actor(gps_bp, camera_transform, attach_to=vehicle)
            
            vehicle_data = {
                'actor': vehicle,
                'camera': camera,
                'gps': gps,
                'id': f'carla_vehicle_{i}',
                'gps_data': None
            }
            
            # Set up GPS callback
            gps.listen(lambda data, v=vehicle_data: self.on_gps_update(data, v))
            
            self.vehicles.append(vehicle_data)
            print(f"  ✅ Spawned vehicle {i} at {spawn_points[i].location}")
        
        print(f"✅ Spawned {len(self.vehicles)} vehicles")
    
    def on_gps_update(self, gps_data, vehicle_data):
        vehicle_data['gps_data'] = gps_data
    
    def cleanup(self):
        print("\n🧹 Cleaning up...")
        for vehicle_data in self.vehicles:
            vehicle_data['camera'].destroy()
            vehicle_data['gps'].destroy()
            vehicle_data['actor'].destroy()
        print("✅ Cleanup complete")
    
    def run(self, duration=60):
        print(f"\n🚀 Running simulation for {duration} seconds...")
        print("Press Ctrl+C to stop early")
        
        start_time = time.time()
        frame_count = 0
        
        try:
            while time.time() - start_time < duration:
                self.world.tick()
                frame_count += 1
                
                # Print status every 30 frames (~1 second)
                if frame_count % 30 == 0:
                    elapsed = time.time() - start_time
                    print(f"⏱️  {elapsed:.1f}s - {len(self.vehicles)} vehicles active")
                
                time.sleep(0.033)  # ~30 FPS
        
        except KeyboardInterrupt:
            print("\n⚠️  Interrupted by user")
        
        finally:
            self.cleanup()

if __name__ == '__main__':
    bridge = GodViewCARLABridge()
    bridge.spawn_vehicles(num_vehicles=3)
    bridge.run(duration=60)
EOF

chmod +x carla_bridge/godview_carla_bridge.py

echo "✅ Created basic CARLA bridge"
echo ""

# Create README
cat > carla_bridge/README.md << 'EOF'
# GodView + CARLA Integration

## Quick Start

### 1. Start CARLA Server
```bash
cd ~/CARLA_0.9.15
./CarlaUE4.sh -quality-level=Low -RenderOffScreen
```

### 2. Run GodView Bridge
```bash
cd /home/ubu/godview/carla_bridge
python3 godview_carla_bridge.py
```

### 3. View Results
Check `carla_results/` for metrics and logs

## Next Steps

1. Implement YOLO detection
2. Connect to Rust agents
3. Add validation system
4. Run test scenarios

See `CARLA_INTEGRATION_PLAN.md` for full details.
EOF

echo "✅ Created README"
echo ""

echo "╔════════════════════════════════════════════╗"
echo "║          SETUP COMPLETE!                   ║"
echo "╚════════════════════════════════════════════╝"
echo ""
echo "📋 Next Steps:"
echo ""
echo "1. Start CARLA server:"
echo "   cd ~/CARLA_0.9.15"
echo "   ./CarlaUE4.sh -quality-level=Low -RenderOffScreen"
echo ""
echo "2. In another terminal, run the bridge:"
echo "   cd /home/ubu/godview/carla_bridge"
echo "   python3 godview_carla_bridge.py"
echo ""
echo "3. See CARLA_INTEGRATION_PLAN.md for full implementation"
echo ""
