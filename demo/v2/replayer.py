#!/usr/bin/env python3
"""
GodView V2 Demo - Replayer
===========================
Replays recorded CARLA scenario and captures frames from multiple cameras.
Saves frame images and camera metadata for proper 3D projection in overlay step.

Two passes:
1. Bird's-eye view: frames 0-450 (SETUP) and 2101-2400 (DEEPDIVE)
2. Chase cam: frames 451-2100 (CHAOS, ACTIVATION, SOLUTION)
"""

import carla
import numpy as np
import json
import time
import math
import argparse
from pathlib import Path
from queue import Queue, Empty
from dataclasses import dataclass
from typing import Optional


# ============================================================================
# CONFIGURATION
# ============================================================================

FPS = 30
FIXED_DELTA = 1.0 / FPS
TOTAL_FRAMES = 2400

# Video settings
WIDTH = 1920
HEIGHT = 1080
FOV = 90

# Output directories
OUTPUT_DIR = Path(__file__).parent / "frames"

# Phase frame ranges
SETUP_FRAMES = (0, 450)      # 0-15s
CHAOS_FRAMES = (451, 1350)   # 15-45s
ACTIVATION_FRAMES = (1351, 1650)  # 45-55s
SOLUTION_FRAMES = (1651, 2100)    # 55-70s
DEEPDIVE_FRAMES = (2101, 2400)    # 70-80s

# Camera configs
BIRDSEYE_HEIGHT = 100.0
CHASE_DISTANCE = 15.0
CHASE_HEIGHT = 8.0
CHASE_PITCH = -15.0


# ============================================================================
# CAMERA METADATA
# ============================================================================

def build_camera_intrinsics(width: int, height: int, fov: float) -> dict:
    """Build camera intrinsics dict for saving."""
    fov_rad = np.radians(fov)
    fx = width / (2.0 * np.tan(fov_rad / 2.0))
    fy = fx
    
    return {
        "width": width,
        "height": height,
        "fov": fov,
        "fx": fx,
        "fy": fy,
        "cx": width / 2.0,
        "cy": height / 2.0
    }


def transform_to_dict(transform: carla.Transform) -> dict:
    """Convert CARLA transform to serializable dict."""
    return {
        "location": {
            "x": transform.location.x,
            "y": transform.location.y,
            "z": transform.location.z
        },
        "rotation": {
            "pitch": transform.rotation.pitch,
            "yaw": transform.rotation.yaw,
            "roll": transform.rotation.roll
        }
    }


def save_frame_metadata(
    filepath: Path,
    transform: carla.Transform,
    intrinsics: dict,
    frame_idx: int,
    phase: str
) -> None:
    """Save camera metadata for a frame."""
    metadata = {
        "frame_idx": frame_idx,
        "phase": phase,
        "camera_transform": transform_to_dict(transform),
        "intrinsics": intrinsics
    }
    
    with open(filepath, 'w') as f:
        json.dump(metadata, f, indent=2)


# ============================================================================
# FRAME CAPTURE
# ============================================================================

class FrameCapture:
    """Captures frames from a CARLA camera sensor."""
    
    def __init__(self, world: carla.World, blueprint_library):
        self.world = world
        self.blueprint_library = blueprint_library
        self.camera = None
        self.frame_queue = Queue()
        self.intrinsics = build_camera_intrinsics(WIDTH, HEIGHT, FOV)
    
    def spawn_camera(self, transform: carla.Transform, attach_to: Optional[carla.Actor] = None) -> carla.Actor:
        """Spawn RGB camera sensor."""
        bp = self.blueprint_library.find('sensor.camera.rgb')
        bp.set_attribute('image_size_x', str(WIDTH))
        bp.set_attribute('image_size_y', str(HEIGHT))
        bp.set_attribute('fov', str(FOV))
        bp.set_attribute('sensor_tick', str(FIXED_DELTA))
        
        if attach_to:
            self.camera = self.world.spawn_actor(bp, transform, attach_to=attach_to)
        else:
            self.camera = self.world.spawn_actor(bp, transform)
        
        self.camera.listen(lambda image: self.frame_queue.put(image))
        
        return self.camera
    
    def get_frame(self, timeout: float = 5.0):
        """Get the next frame from the queue."""
        try:
            return self.frame_queue.get(timeout=timeout)
        except Empty:
            return None
    
    def destroy(self):
        """Destroy the camera."""
        if self.camera:
            self.camera.stop()
            self.camera.destroy()
            self.camera = None
        
        # Clear queue
        while not self.frame_queue.empty():
            try:
                self.frame_queue.get_nowait()
            except Empty:
                break


# ============================================================================
# REPLAYER
# ============================================================================

class Replayer:
    def __init__(self, client: carla.Client, world: carla.World):
        self.client = client
        self.world = world
        self.blueprint_library = world.get_blueprint_library()
        
        # Create output directories
        self.birdseye_dir = OUTPUT_DIR / "birdseye"
        self.birdseye_meta_dir = self.birdseye_dir / "meta"
        self.chase_dir = OUTPUT_DIR / "chase"
        self.chase_meta_dir = self.chase_dir / "meta"
        
        for d in [self.birdseye_dir, self.birdseye_meta_dir, 
                  self.chase_dir, self.chase_meta_dir]:
            d.mkdir(parents=True, exist_ok=True)
        
        # Scene center (computed from spawn points)
        spawn_points = world.get_map().get_spawn_points()
        if spawn_points:
            xs = [sp.location.x for sp in spawn_points[:20]]
            ys = [sp.location.y for sp in spawn_points[:20]]
            self.scene_center = carla.Location(
                x=sum(xs) / len(xs),
                y=sum(ys) / len(ys),
                z=0
            )
        else:
            self.scene_center = carla.Location(0, 0, 0)
        
        self.hero_vehicle = None
        self.intrinsics = build_camera_intrinsics(WIDTH, HEIGHT, FOV)
    
    def setup_world(self):
        """Configure world for replay."""
        settings = self.world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = FIXED_DELTA
        self.world.apply_settings(settings)
    
    def find_hero_vehicle(self):
        """Find the hero vehicle after replay starts."""
        # Wait a few ticks for actors to spawn
        for _ in range(10):
            self.world.tick()
            time.sleep(0.1)
        
        # Find vehicles
        vehicles = self.world.get_actors().filter('vehicle.*')
        if vehicles:
            self.hero_vehicle = vehicles[0]
            print(f"[REPLAYER] Found hero vehicle: {self.hero_vehicle.type_id}")
        else:
            print("[REPLAYER] Warning: No hero vehicle found!")
    
    def get_phase_for_frame(self, frame_idx: int) -> str:
        """Get phase name for a frame index."""
        if SETUP_FRAMES[0] <= frame_idx < SETUP_FRAMES[1]:
            return "SETUP"
        elif CHAOS_FRAMES[0] <= frame_idx < CHAOS_FRAMES[1]:
            return "CHAOS"
        elif ACTIVATION_FRAMES[0] <= frame_idx < ACTIVATION_FRAMES[1]:
            return "ACTIVATION"
        elif SOLUTION_FRAMES[0] <= frame_idx < SOLUTION_FRAMES[1]:
            return "SOLUTION"
        else:
            return "DEEPDIVE"
    
    def compute_birdseye_transform(self, frame_idx: int) -> carla.Transform:
        """Compute bird's-eye camera transform with orbit animation."""
        # During SETUP, orbit from 0° to 90° yaw
        if frame_idx <= SETUP_FRAMES[1]:
            progress = frame_idx / (SETUP_FRAMES[1] - SETUP_FRAMES[0])
            yaw = progress * 90.0
        else:
            # DEEPDIVE: static or slow orbit
            progress = (frame_idx - DEEPDIVE_FRAMES[0]) / (DEEPDIVE_FRAMES[1] - DEEPDIVE_FRAMES[0])
            yaw = 90.0 + progress * 45.0
        
        return carla.Transform(
            carla.Location(
                x=self.scene_center.x,
                y=self.scene_center.y,
                z=BIRDSEYE_HEIGHT
            ),
            carla.Rotation(pitch=-90.0, yaw=yaw, roll=0.0)
        )
    
    def compute_chase_transform(self, hero_transform: carla.Transform) -> carla.Transform:
        """Compute chase camera transform relative to hero vehicle."""
        # Get hero forward direction
        yaw_rad = math.radians(hero_transform.rotation.yaw)
        
        # Position behind and above
        cam_x = hero_transform.location.x - CHASE_DISTANCE * math.cos(yaw_rad)
        cam_y = hero_transform.location.y - CHASE_DISTANCE * math.sin(yaw_rad)
        cam_z = hero_transform.location.z + CHASE_HEIGHT
        
        return carla.Transform(
            carla.Location(x=cam_x, y=cam_y, z=cam_z),
            carla.Rotation(pitch=CHASE_PITCH, yaw=hero_transform.rotation.yaw, roll=0.0)
        )
    
    def run_birdseye_pass(self, recording_file: str):
        """Capture bird's-eye frames for SETUP and DEEPDIVE phases."""
        print("\n" + "=" * 60)
        print("BIRD'S-EYE PASS")
        print("=" * 60)
        
        # Start replay
        print(f"[REPLAY] Starting {recording_file}")
        self.client.replay_file(recording_file, 0.0, 0.0, 0)
        
        # Create capture object
        capture = FrameCapture(self.world, self.blueprint_library)
        
        # Initial camera
        initial_transform = self.compute_birdseye_transform(0)
        camera = capture.spawn_camera(initial_transform)
        spectator = self.world.get_spectator()
        
        frame_count = 0
        
        for frame_idx in range(TOTAL_FRAMES):
            # Tick world
            self.world.tick()
            
            # Only capture for SETUP and DEEPDIVE phases
            is_setup = SETUP_FRAMES[0] <= frame_idx < SETUP_FRAMES[1]
            is_deepdive = DEEPDIVE_FRAMES[0] <= frame_idx < DEEPDIVE_FRAMES[1]
            
            if not (is_setup or is_deepdive):
                continue
            
            # Update camera position
            cam_transform = self.compute_birdseye_transform(frame_idx)
            camera.set_transform(cam_transform)
            spectator.set_transform(cam_transform)
            
            # Capture frame
            image = capture.get_frame(timeout=2.0)
            if image is None:
                print(f"[WARNING] No frame at {frame_idx}")
                continue
            
            # Save image
            frame_path = self.birdseye_dir / f"frame_{frame_idx:05d}.png"
            image.save_to_disk(str(frame_path))
            
            # Save metadata
            meta_path = self.birdseye_meta_dir / f"frame_{frame_idx:05d}.json"
            phase = self.get_phase_for_frame(frame_idx)
            save_frame_metadata(meta_path, cam_transform, self.intrinsics, frame_idx, phase)
            
            frame_count += 1
            
            # Progress
            if frame_count % 30 == 0:
                print(f"[BIRDSEYE] Captured {frame_count} frames (current: {frame_idx})")
        
        capture.destroy()
        print(f"[BIRDSEYE] Complete: {frame_count} frames saved")
    
    def run_chase_pass(self, recording_file: str):
        """Capture chase cam frames for CHAOS, ACTIVATION, SOLUTION phases."""
        print("\n" + "=" * 60)
        print("CHASE CAM PASS")
        print("=" * 60)
        
        # Start fresh replay
        print(f"[REPLAY] Restarting {recording_file}")
        
        # Give world time to settle
        for _ in range(5):
            self.world.tick()
        
        self.client.replay_file(recording_file, 0.0, 0.0, 0)
        
        # Wait for hero vehicle
        self.find_hero_vehicle()
        
        if not self.hero_vehicle:
            print("[ERROR] Cannot run chase pass without hero vehicle!")
            return
        
        # Create camera attached to vehicle
        capture = FrameCapture(self.world, self.blueprint_library)
        
        # Relative transform for attached camera
        attach_transform = carla.Transform(
            carla.Location(x=-CHASE_DISTANCE, z=CHASE_HEIGHT),
            carla.Rotation(pitch=CHASE_PITCH)
        )
        
        camera = capture.spawn_camera(attach_transform, attach_to=self.hero_vehicle)
        spectator = self.world.get_spectator()
        
        frame_count = 0
        
        for frame_idx in range(TOTAL_FRAMES):
            # Tick world
            self.world.tick()
            
            # Only capture for middle phases
            if not (CHAOS_FRAMES[0] <= frame_idx < SOLUTION_FRAMES[1]):
                continue
            
            # Get world transform of camera
            cam_transform = camera.get_transform()
            spectator.set_transform(cam_transform)
            
            # Capture frame
            image = capture.get_frame(timeout=2.0)
            if image is None:
                print(f"[WARNING] No frame at {frame_idx}")
                continue
            
            # Save image
            frame_path = self.chase_dir / f"frame_{frame_idx:05d}.png"
            image.save_to_disk(str(frame_path))
            
            # Save metadata
            meta_path = self.chase_meta_dir / f"frame_{frame_idx:05d}.json"
            phase = self.get_phase_for_frame(frame_idx)
            save_frame_metadata(meta_path, cam_transform, self.intrinsics, frame_idx, phase)
            
            frame_count += 1
            
            # Progress
            if frame_count % 30 == 0:
                print(f"[CHASE] Captured {frame_count} frames (current: {frame_idx})")
        
        capture.destroy()
        print(f"[CHASE] Complete: {frame_count} frames saved")
    
    def run(self, recording_file: str):
        """Run both camera passes."""
        print("=" * 60)
        print("GODVIEW V2 REPLAYER")
        print("=" * 60)
        
        self.setup_world()
        
        # Run passes
        self.run_birdseye_pass(recording_file)
        self.run_chase_pass(recording_file)
        
        print("\n" + "=" * 60)
        print("REPLAYER COMPLETE")
        print("=" * 60)
        print(f"Output: {OUTPUT_DIR}")
    
    def cleanup(self):
        """Reset world settings."""
        settings = self.world.get_settings()
        settings.synchronous_mode = False
        self.world.apply_settings(settings)


def main():
    parser = argparse.ArgumentParser(description="GodView V2 Replayer")
    parser.add_argument("--host", default="localhost", help="CARLA host")
    parser.add_argument("--port", type=int, default=2000, help="CARLA port")
    parser.add_argument("--recording", default="data/godview_demo.log", help="Recording file")
    args = parser.parse_args()
    
    print(f"[INIT] Connecting to CARLA at {args.host}:{args.port}")
    client = carla.Client(args.host, args.port)
    client.set_timeout(30.0)
    
    world = client.get_world()
    print(f"[INIT] Connected to {world.get_map().name}")
    
    replayer = Replayer(client, world)
    
    try:
        recording_path = Path(__file__).parent / args.recording
        replayer.run(str(recording_path))
    finally:
        replayer.cleanup()


if __name__ == "__main__":
    main()
