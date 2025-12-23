#!/usr/bin/env python3
"""
GodView CARLA Bridge - Phase 1
Connects CARLA simulator to GodView v3 Rust agents

Hardware Optimization: GTX 1050 Ti (4GB VRAM)
- Low quality rendering
- Reduced camera resolution (640x480)
- Efficient YOLO model (YOLOv8n)
"""

import carla
import cv2
import numpy as np
import time
import json
import subprocess
import sys
from pathlib import Path
from queue import Queue
import threading

try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    print("⚠️  YOLOv8 not available. Install with: pip install ultralytics")
    YOLO_AVAILABLE = False


class GodViewCARLABridge:
    def __init__(self, carla_host='localhost', carla_port=2000, num_vehicles=3):
        """
        Initialize CARLA bridge
        
        Args:
            carla_host: CARLA server hostname
            carla_port: CARLA server port
            num_vehicles: Number of vehicles to spawn
        """
        print("╔════════════════════════════════════════════╗")
        print("║   GODVIEW + CARLA INTEGRATION (Phase 1)   ║")
        print("╚════════════════════════════════════════════╝")
        print()
        
        # Connect to CARLA
        print(f"🔌 Connecting to CARLA at {carla_host}:{carla_port}...")
        self.client = carla.Client(carla_host, carla_port)
        self.client.set_timeout(10.0)
        
        # Get world and apply low-quality settings for GTX 1050 Ti
        self.world = self.client.get_world()
        self._apply_low_quality_settings()
        
        print(f"✅ Connected to CARLA world: {self.world.get_map().name}")
        print(f"🎮 Graphics: Low Quality (GTX 1050 Ti optimized)")
        print()
        
        # Load YOLO model (nano version for speed)
        if YOLO_AVAILABLE:
            print("🤖 Loading YOLOv8n (nano) model...")
            self.yolo = YOLO('yolov8n.pt')  # Smallest, fastest model
            print("✅ YOLO model loaded")
        else:
            self.yolo = None
            print("⚠️  YOLO disabled - will use dummy detections")
        print()
        
        # Storage
        self.vehicles = []
        self.agents = []
        self.frame_queues = {}
        
        # Spawn vehicles
        self.spawn_vehicles(num_vehicles)
    
    def _apply_low_quality_settings(self):
        """Apply low-quality graphics settings for GTX 1050 Ti"""
        settings = self.world.get_settings()
        settings.no_rendering_mode = False  # We need rendering for cameras
        settings.synchronous_mode = True    # Synchronous for deterministic testing
        settings.fixed_delta_seconds = 0.05  # 20 FPS (easier on GPU)
        self.world.apply_settings(settings)
        
        print("⚙️  Applied settings:")
        print("   - Synchronous mode: ON")
        print("   - Fixed delta: 0.05s (20 FPS)")
        print("   - Camera resolution: 640x480 (VRAM optimized)")
    
    def spawn_vehicles(self, num_vehicles=3):
        """Spawn vehicles with cameras and GPS sensors"""
        print(f"🚗 Spawning {num_vehicles} vehicles...")
        
        blueprint_library = self.world.get_blueprint_library()
        vehicle_bp = blueprint_library.filter('vehicle.tesla.model3')[0]
        
        spawn_points = self.world.get_map().get_spawn_points()
        
        if len(spawn_points) < num_vehicles:
            print(f"⚠️  Only {len(spawn_points)} spawn points available")
            num_vehicles = len(spawn_points)
        
        for i in range(num_vehicles):
            try:
                # Spawn vehicle
                vehicle = self.world.spawn_actor(vehicle_bp, spawn_points[i])
                vehicle.set_autopilot(True)  # Enable autopilot
                
                # Attach RGB camera (LOW RESOLUTION for GTX 1050 Ti)
                camera_bp = blueprint_library.find('sensor.camera.rgb')
                camera_bp.set_attribute('image_size_x', '640')   # Reduced from 1280
                camera_bp.set_attribute('image_size_y', '480')   # Reduced from 720
                camera_bp.set_attribute('fov', '90')
                
                camera_transform = carla.Transform(
                    carla.Location(x=2.0, z=1.5)  # Front bumper, 1.5m high
                )
                camera = self.world.spawn_actor(
                    camera_bp,
                    camera_transform,
                    attach_to=vehicle
                )
                
                # Attach GPS sensor
                gps_bp = blueprint_library.find('sensor.other.gnss')
                gps = self.world.spawn_actor(
                    gps_bp,
                    camera_transform,
                    attach_to=vehicle
                )
                
                # Create vehicle data structure
                vehicle_data = {
                    'actor': vehicle,
                    'camera': camera,
                    'gps': gps,
                    'id': f'carla_vehicle_{i}',
                    'gps_data': None,
                    'frame_count': 0
                }
                
                # Set up sensor callbacks
                self.frame_queues[vehicle_data['id']] = Queue(maxsize=2)
                
                camera.listen(lambda image, v=vehicle_data: self._on_camera_frame(image, v))
                gps.listen(lambda data, v=vehicle_data: self._on_gps_update(data, v))
                
                self.vehicles.append(vehicle_data)
                
                location = spawn_points[i].location
                print(f"  ✅ Vehicle {i}: Spawned at ({location.x:.1f}, {location.y:.1f}, {location.z:.1f})")
                
            except Exception as e:
                print(f"  ❌ Failed to spawn vehicle {i}: {e}")
        
        print(f"✅ Spawned {len(self.vehicles)} vehicles")
        print()
    
    def _on_camera_frame(self, image, vehicle_data):
        """Callback for camera frames"""
        # Convert CARLA image to numpy array
        array = np.frombuffer(image.raw_data, dtype=np.uint8)
        array = array.reshape((image.height, image.width, 4))
        frame = array[:, :, :3]  # RGB (drop alpha)
        
        # Put in queue (non-blocking)
        if not self.frame_queues[vehicle_data['id']].full():
            self.frame_queues[vehicle_data['id']].put(frame)
    
    def _on_gps_update(self, gps_data, vehicle_data):
        """Callback for GPS updates"""
        vehicle_data['gps_data'] = gps_data
    
    def start_godview_agents(self):
        """Start Rust GodView agents for each vehicle"""
        print("🚀 Starting GodView Rust agents...")
        
        godview_agent_path = Path(__file__).parent.parent / 'agent'
        
        for vehicle_data in self.vehicles:
            try:
                # Start Rust agent as subprocess
                agent_process = subprocess.Popen(
                    ['cargo', 'run', '--release'],
                    cwd=str(godview_agent_path),
                    stdin=subprocess.PIPE,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    env={
                        **os.environ,  # Inherit from parent (preserves PATH, PYTHONPATH, etc.)
                        'AGENT_ID': vehicle_data['id'],
                        'CARLA_MODE': 'true',
                        'RUST_BACKTRACE': '1'
                    },
                    bufsize=1,
                    universal_newlines=True
                )
                
                vehicle_data['agent_process'] = agent_process
                self.agents.append(agent_process)
                
                print(f"  ✅ Started agent for {vehicle_data['id']} (PID: {agent_process.pid})")
                
            except Exception as e:
                print(f"  ❌ Failed to start agent for {vehicle_data['id']}: {e}")
        
        print(f"✅ Started {len(self.agents)} agents")
        print()
    
    def process_vehicle(self, vehicle_data):
        """Process one vehicle's sensor data and send to agent"""
        # Check if we have GPS data
        if vehicle_data['gps_data'] is None:
            return
        
        # Get frame from queue (non-blocking)
        if self.frame_queues[vehicle_data['id']].empty():
            return
        
        frame = self.frame_queues[vehicle_data['id']].get()
        
        # Get vehicle transform for heading
        transform = vehicle_data['actor'].get_transform()
        heading = transform.rotation.yaw  # Degrees
        
        # Run YOLO detection
        detections = []
        if self.yolo is not None:
            results = self.yolo(frame, verbose=False)
            
            for detection in results[0].boxes:
                bbox = detection.xyxy[0].cpu().numpy()
                confidence = float(detection.conf[0].cpu().numpy())
                class_id = int(detection.cls[0].cpu().numpy())
                class_name = results[0].names[class_id]
                
                # Filter for relevant classes
                if class_name in ['car', 'truck', 'bus', 'person', 'bicycle', 'motorcycle']:
                    detections.append({
                        'bbox': bbox.tolist(),
                        'confidence': confidence,
                        'class_name': class_name
                    })
        
        # Send detections to Rust agent
        for detection in detections:
            detection_data = {
                'bbox': detection['bbox'],
                'confidence': detection['confidence'],
                'class_name': detection['class_name'],
                'gps_lat': vehicle_data['gps_data'].latitude,
                'gps_lon': vehicle_data['gps_data'].longitude,
                'gps_alt': vehicle_data['gps_data'].altitude,
                'heading': heading,
                'timestamp': time.time()
            }
            
            # Send JSON to Rust agent via stdin
            try:
                json_line = json.dumps(detection_data) + '\n'
                vehicle_data['agent_process'].stdin.write(json_line)
                vehicle_data['agent_process'].stdin.flush()
            except Exception as e:
                print(f"⚠️  Failed to send to agent {vehicle_data['id']}: {e}")
        
        vehicle_data['frame_count'] += 1
    
    def run(self, duration=60):
        """Run the simulation"""
        print(f"🎬 Running simulation for {duration} seconds...")
        print("Press Ctrl+C to stop early")
        print()
        
        start_time = time.time()
        tick_count = 0
        
        try:
            while time.time() - start_time < duration:
                # Tick the world (synchronous mode)
                self.world.tick()
                tick_count += 1
                
                # Process each vehicle
                for vehicle_data in self.vehicles:
                    self.process_vehicle(vehicle_data)
                
                # Print status every 20 ticks (~1 second at 20 FPS)
                if tick_count % 20 == 0:
                    elapsed = time.time() - start_time
                    total_frames = sum(v['frame_count'] for v in self.vehicles)
                    fps = total_frames / elapsed if elapsed > 0 else 0
                    
                    print(f"⏱️  {elapsed:.1f}s | Ticks: {tick_count} | "
                          f"Frames: {total_frames} | FPS: {fps:.1f}")
        
        except KeyboardInterrupt:
            print("\n⚠️  Interrupted by user")
        
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Clean up CARLA actors and agents"""
        print("\n🧹 Cleaning up...")
        
        # Stop agents
        for agent in self.agents:
            try:
                agent.terminate()
                agent.wait(timeout=5)
            except:
                agent.kill()
        
        # Destroy CARLA actors
        for vehicle_data in self.vehicles:
            try:
                vehicle_data['camera'].destroy()
                vehicle_data['gps'].destroy()
                vehicle_data['actor'].destroy()
            except:
                pass
        
        print("✅ Cleanup complete")


def main():
    """Main entry point"""
    import argparse
    
    parser = argparse.ArgumentParser(description='GodView CARLA Bridge')
    parser.add_argument('--host', default='localhost', help='CARLA server host')
    parser.add_argument('--port', type=int, default=2000, help='CARLA server port')
    parser.add_argument('--vehicles', type=int, default=3, help='Number of vehicles')
    parser.add_argument('--duration', type=int, default=60, help='Simulation duration (seconds)')
    
    args = parser.parse_args()
    
    # Create bridge
    bridge = GodViewCARLABridge(
        carla_host=args.host,
        carla_port=args.port,
        num_vehicles=args.vehicles
    )
    
    # Start agents
    bridge.start_godview_agents()
    
    # Run simulation
    bridge.run(duration=args.duration)


if __name__ == '__main__':
    main()
