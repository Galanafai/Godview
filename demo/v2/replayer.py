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
        print("[REPLAYER] Waiting for hero vehicle...")
        
        # Try for up to 5 seconds
        for i in range(50):
            self.world.tick()
            vehicles = self.world.get_actors().filter('vehicle.*')
            if vehicles:
                # Pick the one with the lowest ID (likely the first spawned)
                sorted_vehicles = sorted(list(vehicles), key=lambda v: v.id)
                self.hero_vehicle = sorted_vehicles[0]
                print(f"[REPLAYER] Found hero vehicle: {self.hero_vehicle.type_id} (ID: {self.hero_vehicle.id})")
                return
            time.sleep(0.1)
            
        print("[REPLAYER] Warning: No hero vehicle found after 5 seconds!")
        # Debug: print all actors
        print("All actors:", [a.type_id for a in self.world.get_actors()])
    
    # ... (rest of methods)

    def run(self, recording_file: str, run_mode: str = "all"):
        """Run camera passes based on mode."""
        print("=" * 60)
        print(f"GODVIEW V2 REPLAYER (Mode: {run_mode})")
        print("=" * 60)
        
        self.setup_world()
        
        if run_mode in ["all", "birdseye"]:
            self.run_birdseye_pass(recording_file)
            
        if run_mode in ["all", "chase"]:
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
    parser.add_argument("--pass_type", default="all", choices=["all", "birdseye", "chase"], help="Which pass to run")
    args = parser.parse_args()
    
    print(f"[INIT] Connecting to CARLA at {args.host}:{args.port}")
    client = carla.Client(args.host, args.port)
    client.set_timeout(30.0)
    
    world = client.get_world()
    print(f"[INIT] Connected to {world.get_map().name}")
    
    replayer = Replayer(client, world)
    
    try:
        recording_path = Path(__file__).parent / args.recording
        replayer.run(str(recording_path), args.pass_type)
    finally:
        replayer.cleanup()


if __name__ == "__main__":
    main()
