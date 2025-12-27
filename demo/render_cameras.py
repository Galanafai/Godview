#!/usr/bin/env python3
"""
GodView Demo - Camera Renderer (The Cinematographer)
=====================================================
Captures high-res frames from CARLA during LIVE simulation.

FIX: This version captures frames DURING the scenario instead of replaying,
which avoids the replay file compatibility issues.

Usage:
  1. Start CARLA server
  2. Run this script - it will spawn actors AND capture frames simultaneously
"""

import carla
import numpy as np
import cv2
import os
import time
import queue
import argparse
import random

# Configuration
FRAMES_BASE_PATH = "/workspace/godview_demo/frames"
CAMERA_WIDTH = 1920
CAMERA_HEIGHT = 1080
CAMERA_FOV = 90.0


class LiveCameraRenderer:
    """Renders frames from CARLA during live simulation."""
    
    def __init__(self, client, world, output_dir):
        self.client = client
        self.world = world
        self.output_dir = output_dir
        self.cameras = {}
        self.image_queues = {}
        self.vehicles = []
        self.hero_vehicle = None
        
        os.makedirs(output_dir, exist_ok=True)
    
    def setup_world(self):
        """Configure synchronous mode."""
        settings = self.world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05  # 20 FPS
        self.world.apply_settings(settings)
        print("[SETUP] Synchronous mode enabled")
    
    def spawn_actors(self, num_vehicles=15):
        """Spawn vehicles for the scene."""
        bp_lib = self.world.get_blueprint_library()
        spawn_points = self.world.get_map().get_spawn_points()
        random.shuffle(spawn_points)
        
        for i, sp in enumerate(spawn_points[:num_vehicles]):
            bp = random.choice(bp_lib.filter('vehicle.*'))
            vehicle = self.world.try_spawn_actor(bp, sp)
            if vehicle:
                vehicle.set_autopilot(True)
                self.vehicles.append(vehicle)
                if i == 0:
                    self.hero_vehicle = vehicle
        
        print(f"[SPAWN] {len(self.vehicles)} vehicles spawned")
        return self.vehicles
    
    def create_camera(self, name, transform, parent=None):
        """Create and attach an RGB camera."""
        bp = self.world.get_blueprint_library().find('sensor.camera.rgb')
        bp.set_attribute('image_size_x', str(CAMERA_WIDTH))
        bp.set_attribute('image_size_y', str(CAMERA_HEIGHT))
        bp.set_attribute('fov', str(CAMERA_FOV))
        
        if parent:
            camera = self.world.spawn_actor(bp, transform, attach_to=parent)
        else:
            camera = self.world.spawn_actor(bp, transform)
        
        # Setup image queue
        img_queue = queue.Queue()
        camera.listen(img_queue.put)
        
        self.cameras[name] = camera
        self.image_queues[name] = img_queue
        
        print(f"[CAMERA] Created '{name}' camera")
        return camera
    
    def setup_cameras(self):
        """Setup all camera passes."""
        
        # Pass 1: Ego-Chase (behind hero vehicle)
        if self.hero_vehicle:
            ego_transform = carla.Transform(
                carla.Location(x=-8.0, y=0.0, z=4.0),
                carla.Rotation(pitch=-15.0, yaw=0.0)
            )
            self.create_camera("ego", ego_transform, parent=self.hero_vehicle)
        
        # Pass 2: Overhead/Drone view (fixed in sky)
        spectator = self.world.get_spectator()
        spec_loc = spectator.get_transform().location
        overhead_transform = carla.Transform(
            carla.Location(x=spec_loc.x, y=spec_loc.y, z=60.0),
            carla.Rotation(pitch=-90.0)
        )
        self.create_camera("overhead", overhead_transform)
    
    def save_frame(self, image, output_path):
        """Convert CARLA image to file."""
        array = np.frombuffer(image.raw_data, dtype=np.uint8)
        array = array.reshape((CAMERA_HEIGHT, CAMERA_WIDTH, 4))
        array = array[:, :, :3]  # Remove alpha
        array = array[:, :, ::-1]  # BGRA to RGB for cv2
        cv2.imwrite(output_path, array)
    
    def capture_frames(self, num_frames=300, camera_name="ego"):
        """Capture frames from specified camera during simulation."""
        if camera_name not in self.cameras:
            print(f"[ERROR] Camera '{camera_name}' not found")
            return 0
        
        output_dir = os.path.join(self.output_dir, camera_name)
        os.makedirs(output_dir, exist_ok=True)
        
        img_queue = self.image_queues[camera_name]
        captured = 0
        start_time = time.time()
        
        print(f"[CAPTURE] Starting capture of {num_frames} frames from '{camera_name}'")
        
        for frame in range(num_frames):
            self.world.tick()
            
            try:
                image = img_queue.get(timeout=2.0)
                frame_path = os.path.join(output_dir, f"{camera_name}_{frame:04d}.png")
                self.save_frame(image, frame_path)
                captured += 1
                
                if frame % 50 == 0:
                    elapsed = time.time() - start_time
                    fps = captured / elapsed if elapsed > 0 else 0
                    print(f"    Frame {frame}/{num_frames} ({fps:.1f} FPS)")
            
            except queue.Empty:
                print(f"    Warning: No image at frame {frame}")
        
        print(f"[CAPTURE] Saved {captured} frames to {output_dir}")
        return captured
    
    def cleanup(self):
        """Destroy cameras and vehicles."""
        for name, camera in self.cameras.items():
            camera.stop()
            camera.destroy()
        
        for vehicle in self.vehicles:
            if vehicle.is_alive:
                vehicle.destroy()
        
        settings = self.world.get_settings()
        settings.synchronous_mode = False
        self.world.apply_settings(settings)
        
        print("[CLEANUP] Done")


def main():
    parser = argparse.ArgumentParser(description="GodView Demo - Live Camera Renderer")
    parser.add_argument("--frames", type=int, default=300, help="Number of frames to capture")
    parser.add_argument("--camera", default="ego", choices=["ego", "overhead", "all"],
                        help="Which camera to use")
    parser.add_argument("--output", default=FRAMES_BASE_PATH, help="Output directory")
    args = parser.parse_args()
    
    print("=" * 60)
    print("GodView Demo - Live Camera Renderer")
    print("=" * 60)
    
    # Connect
    print("\n[1/5] Connecting to CARLA...")
    client = carla.Client('localhost', 2000)
    client.set_timeout(30.0)
    world = client.get_world()
    
    # Create renderer
    renderer = LiveCameraRenderer(client, world, args.output)
    
    try:
        print("[2/5] Configuring world...")
        renderer.setup_world()
        
        print("[3/5] Spawning actors...")
        renderer.spawn_actors()
        
        print("[4/5] Setting up cameras...")
        renderer.setup_cameras()
        
        print("[5/5] Capturing frames...")
        if args.camera == "all":
            for cam_name in renderer.cameras.keys():
                renderer.capture_frames(args.frames, cam_name)
        else:
            renderer.capture_frames(args.frames, args.camera)
        
        print("\n" + "=" * 60)
        print("COMPLETE!")
        print(f"Frames saved to: {args.output}/")
        print("=" * 60)
    
    finally:
        renderer.cleanup()


if __name__ == "__main__":
    main()
