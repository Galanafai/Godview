#!/usr/bin/env python3
"""
GodView V2 Demo - Scenario Runner
==================================
Records a CARLA simulation and generates NDJSON detection logs.

Updated for V2 requirements:
- 2400 frames @ 30 FPS (80 seconds)
- Generates DETECTION, MERGE_EVENT, and CANONICAL_STATE packets
- Injects faults: OOSM, ghosts, pancake drones, Sybil attacks
"""

import carla
import random
import time
import json
import math
import argparse
from pathlib import Path
from dataclasses import dataclass, asdict
from typing import List, Dict, Optional


# ============================================================================
# CONFIGURATION
# ============================================================================

# Simulation settings
FIXED_DELTA = 1.0 / 30.0  # 30 FPS
SIMULATION_FRAMES = 2400   # 80 seconds
FPS = 30

# Actor counts
NUM_VEHICLES = 15
NUM_DRONES = 3  # Pedestrians teleported to altitude

# Scene center (Town10HD)
SCENE_CENTER = carla.Location(x=0, y=0, z=0)

# Fault injection rates
OOSM_RATE = 0.3       # 30% of vehicles have OOSM jitter
GHOST_RATE = 0.15     # 15% chance of ghost per frame
PANCAKE_RATE = 1.0    # 100% of drones pancaked in raw view
SYBIL_ACTIVE = True   # Include Sybil attack object

# Drone altitude
DRONE_ALTITUDE = 25.0

# Output files
OUTPUT_DIR = Path(__file__).parent / "data"


# ============================================================================
# PACKET BUILDERS
# ============================================================================

def build_detection_packet(
    sensor_id: str,
    timestamp_ns: int,
    sequence_id: int,
    objects: List[dict]
) -> dict:
    """Build a DETECTION packet per spec."""
    return {
        "packet_type": "DETECTION",
        "sensor_id": sensor_id,
        "timestamp_ns": timestamp_ns,
        "sequence_id": sequence_id,
        "objects": objects
    }


def build_object_detection(
    local_id: str,
    obj_class: str,
    pose: dict,
    bbox_extent: dict,
    confidence: float = 0.8,
    note: Optional[str] = None,
    signature: Optional[str] = None
) -> dict:
    """Build a single object detection."""
    obj = {
        "local_id": local_id,
        "class": obj_class,
        "confidence": confidence,
        "pose": pose,
        "bbox_extent": bbox_extent,
        "covariance": [0.5, 0.0, 0.0, 0.5]
    }
    if note:
        obj["note"] = note
    if signature:
        obj["signature"] = signature
    return obj


def build_canonical_state_packet(
    timestamp_ns: int,
    objects: List[dict]
) -> dict:
    """Build a CANONICAL_STATE packet (fused output)."""
    return {
        "packet_type": "CANONICAL_STATE",
        "timestamp_ns": timestamp_ns,
        "objects": objects
    }


def build_canonical_object(
    canonical_id: str,
    obj_class: str,
    pose: dict,
    bbox_extent: dict,
    confidence: float = 0.95,
    velocity: Optional[dict] = None
) -> dict:
    """Build a single canonical object."""
    obj = {
        "canonical_id": canonical_id,
        "class": obj_class,
        "confidence": confidence,
        "pose": pose,
        "bbox_extent": bbox_extent
    }
    if velocity:
        obj["velocity"] = velocity
    return obj


def build_merge_event(
    timestamp_ns: int,
    event_code: str,
    details: dict
) -> dict:
    """Build a MERGE_EVENT packet."""
    return {
        "packet_type": "MERGE_EVENT",
        "timestamp_ns": timestamp_ns,
        "event_code": event_code,
        "details": details
    }


# ============================================================================
# SCENARIO RUNNER
# ============================================================================

class ScenarioRunner:
    def __init__(self, client: carla.Client, world: carla.World):
        self.client = client
        self.world = world
        self.blueprint_library = world.get_blueprint_library()
        
        # Actors
        self.vehicles = []
        self.drones = []  # Actually walkers teleported to altitude
        self.hero_vehicle = None
        
        # Tracking
        self.oosm_actors = set()
        self.ghost_actors = {}  # actor_id -> ghost offset
        self.sybil_position = None
        
        # State history for OOSM simulation
        self.position_history = {}  # actor_id -> list of positions
        
        # Output files
        OUTPUT_DIR.mkdir(exist_ok=True)
        self.raw_file = open(OUTPUT_DIR / "raw_broken.ndjson", 'w')
        self.fused_file = open(OUTPUT_DIR / "godview_merged.ndjson", 'w')
        self.events_file = open(OUTPUT_DIR / "merge_events.ndjson", 'w')
        
        # Stats
        self.stats = {
            "ghosts_generated": 0,
            "oosm_packets": 0,
            "trust_rejects": 0,
            "merges": 0
        }
    
    def setup_world(self):
        """Configure world for synchronous mode."""
        settings = self.world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = FIXED_DELTA
        self.world.apply_settings(settings)
        
        # Setup Traffic Manager
        tm = self.client.get_trafficmanager(8000)
        tm.set_synchronous_mode(True)
        tm.set_global_distance_to_leading_vehicle(2.5)
        tm.global_percentage_speed_difference(-20)  # Slightly slower
        
        print(f"[SETUP] World configured: {FIXED_DELTA}s delta, {FPS} FPS")
        
        return tm
    
    def spawn_actors(self, tm):
        """Spawn vehicles and drones."""
        spawn_points = self.world.get_map().get_spawn_points()
        random.shuffle(spawn_points)
        
        # Spawn vehicles
        vehicle_bps = self.blueprint_library.filter('vehicle.*')
        safe_bps = [bp for bp in vehicle_bps if int(bp.get_attribute('number_of_wheels')) == 4]
        
        for i in range(min(NUM_VEHICLES, len(spawn_points))):
            bp = random.choice(safe_bps)
            if bp.has_attribute('color'):
                bp.set_attribute('color', random.choice(bp.get_attribute('color').recommended_values))
            
            vehicle = self.world.try_spawn_actor(bp, spawn_points[i])
            if vehicle:
                vehicle.set_autopilot(True, tm.get_port())
                self.vehicles.append(vehicle)
                
                # Mark some for OOSM
                if random.random() < OOSM_RATE:
                    self.oosm_actors.add(vehicle.id)
                
                # First vehicle is hero
                if self.hero_vehicle is None:
                    self.hero_vehicle = vehicle
        
        # Spawn drones (walkers teleported to altitude)
        walker_bp = self.blueprint_library.filter('walker.pedestrian.*')[0]
        
        for i in range(NUM_DRONES):
            if i + NUM_VEHICLES < len(spawn_points):
                spawn_point = spawn_points[i + NUM_VEHICLES]
                # Elevate spawn point
                spawn_point.location.z = DRONE_ALTITUDE
                
                # Spawn walker
                drone = self.world.try_spawn_actor(walker_bp, spawn_point)
                if drone:
                    self.drones.append(drone)
        
        # Set Sybil attack position (fake object)
        if SYBIL_ACTIVE and self.hero_vehicle:
            hero_loc = self.hero_vehicle.get_location()
            self.sybil_position = {
                'x': hero_loc.x + 15,
                'y': hero_loc.y + 5,
                'z': 0
            }
        
        print(f"[SPAWN] {len(self.vehicles)} vehicles, {len(self.drones)} drones")
        print(f"[SPAWN] OOSM actors: {len(self.oosm_actors)}")
    
    def get_actor_pose(self, actor, add_jitter=False) -> dict:
        """Get actor pose, optionally with OOSM jitter."""
        transform = actor.get_transform()
        loc = transform.location
        rot = transform.rotation
        
        pose = {
            'x': loc.x,
            'y': loc.y,
            'z': loc.z,
            'yaw': rot.yaw,
            'pitch': rot.pitch,
            'roll': rot.roll
        }
        
        if add_jitter and actor.id in self.oosm_actors:
            # OOSM jitter: random offset simulating stale/out-of-order data
            pose['x'] += random.gauss(0, 1.2)
            pose['y'] += random.gauss(0, 1.2)
            self.stats["oosm_packets"] += 1
        
        return pose
    
    def get_actor_bbox(self, actor) -> dict:
        """Get actor bounding box extent."""
        bbox = actor.bounding_box
        return {
            'x': bbox.extent.x,
            'y': bbox.extent.y,
            'z': bbox.extent.z
        }
    
    def generate_raw_detections(self, frame_idx: int, timestamp_ns: int):
        """Generate raw (broken) detections with faults."""
        objects = []
        
        # Vehicles
        for vehicle in self.vehicles:
            if not vehicle.is_alive:
                continue
            
            # Raw pose with possible OOSM jitter
            pose = self.get_actor_pose(vehicle, add_jitter=True)
            bbox = self.get_actor_bbox(vehicle)
            
            obj = build_object_detection(
                local_id=f"vehicle_{vehicle.id}",
                obj_class="vehicle",
                pose=pose,
                bbox_extent=bbox,
                confidence=0.7 + random.uniform(-0.1, 0.1),
                signature=f"sig_{vehicle.id}"
            )
            objects.append(obj)
            
            # Ghost generation
            if random.random() < GHOST_RATE:
                ghost_pose = pose.copy()
                ghost_pose['x'] += random.uniform(-3, 3)
                ghost_pose['y'] += random.uniform(-3, 3)
                
                ghost_obj = build_object_detection(
                    local_id=f"ghost_{vehicle.id}_{frame_idx}",
                    obj_class="vehicle",
                    pose=ghost_pose,
                    bbox_extent=bbox,
                    confidence=0.4 + random.uniform(0, 0.2),
                    note="GHOST_DUPLICATE"
                )
                objects.append(ghost_obj)
                self.stats["ghosts_generated"] += 1
        
        # Drones (pancaked to ground in raw view)
        for drone in self.drones:
            if not drone.is_alive:
                continue
            
            pose = self.get_actor_pose(drone)
            bbox = self.get_actor_bbox(drone)
            
            # Pancake: force Z to ground level
            pose['z'] = 0.5
            
            obj = build_object_detection(
                local_id=f"drone_{drone.id}",
                obj_class="drone",
                pose=pose,
                bbox_extent=bbox,
                confidence=0.6,
                note="PANCAKE_FAILURE_EXAMPLE",
                signature=f"sig_drone_{drone.id}"
            )
            objects.append(obj)
        
        # Sybil attack object
        if SYBIL_ACTIVE and self.sybil_position and frame_idx > 450:  # After setup phase
            sybil_obj = build_object_detection(
                local_id="sybil_fake_barrier",
                obj_class="vehicle",
                pose={**self.sybil_position, 'yaw': 0, 'pitch': 0, 'roll': 0},
                bbox_extent={'x': 3.0, 'y': 1.0, 'z': 2.0},
                confidence=0.9,
                note="SYBIL_ATTACK",
                signature="INVALID_SIGNATURE"
            )
            objects.append(sybil_obj)
        
        # Write packet
        packet = build_detection_packet(
            sensor_id="multi_sensor_array",
            timestamp_ns=timestamp_ns,
            sequence_id=frame_idx,
            objects=objects
        )
        self.raw_file.write(json.dumps(packet) + '\n')
    
    def generate_fused_state(self, frame_idx: int, timestamp_ns: int):
        """Generate GodView fused (corrected) state."""
        objects = []
        
        # Vehicles (smooth, no ghosts)
        for vehicle in self.vehicles:
            if not vehicle.is_alive:
                continue
            
            # Clean pose (no jitter)
            pose = self.get_actor_pose(vehicle, add_jitter=False)
            bbox = self.get_actor_bbox(vehicle)
            
            obj = build_canonical_object(
                canonical_id=f"vehicle_{vehicle.id}",
                obj_class="vehicle",
                pose=pose,
                bbox_extent=bbox,
                confidence=0.95
            )
            objects.append(obj)
        
        # Drones (correct altitude)
        for drone in self.drones:
            if not drone.is_alive:
                continue
            
            pose = self.get_actor_pose(drone)
            bbox = self.get_actor_bbox(drone)
            
            # Correct altitude (not pancaked)
            pose['z'] = DRONE_ALTITUDE
            
            obj = build_canonical_object(
                canonical_id=f"drone_{drone.id}",
                obj_class="drone",
                pose=pose,
                bbox_extent=bbox,
                confidence=0.92
            )
            objects.append(obj)
        
        # No Sybil - rejected by trust verification
        
        # Write packet
        packet = build_canonical_state_packet(
            timestamp_ns=timestamp_ns,
            objects=objects
        )
        self.fused_file.write(json.dumps(packet) + '\n')
    
    def generate_merge_events(self, frame_idx: int, timestamp_ns: int):
        """Generate MERGE_EVENT packets at appropriate times."""
        # Activation phase: frames 1351-1650 (45s-55s)
        if 1351 <= frame_idx <= 1650:
            # Ghost merges
            if frame_idx % 30 == 0:  # Every second
                event = build_merge_event(
                    timestamp_ns=timestamp_ns,
                    event_code="ID_MERGE",
                    details={
                        "incoming_id": f"ghost_vehicle_{random.randint(1, NUM_VEHICLES)}",
                        "canonical_id": f"vehicle_{random.randint(1, NUM_VEHICLES)}",
                        "method": "highlander",
                        "confidence_boost": 0.1 + random.uniform(0, 0.05)
                    }
                )
                self.events_file.write(json.dumps(event) + '\n')
                self.stats["merges"] += 1
            
            # Trust reject (for Sybil) - once at start of activation
            if frame_idx == 1400 and SYBIL_ACTIVE:
                event = build_merge_event(
                    timestamp_ns=timestamp_ns,
                    event_code="TRUST_REJECT",
                    details={
                        "sensor_id": "attacker_sybil",
                        "reason": "INVALID_SIGNATURE"
                    }
                )
                self.events_file.write(json.dumps(event) + '\n')
                self.stats["trust_rejects"] += 1
            
            # OOSM correction events
            if frame_idx % 15 == 0:
                event = build_merge_event(
                    timestamp_ns=timestamp_ns,
                    event_code="OOSM_CORRECTED",
                    details={
                        "packet_id": f"pkt_{frame_idx}",
                        "delay_ms": random.randint(50, 200),
                        "method": "augmented_state_ekf"
                    }
                )
                self.events_file.write(json.dumps(event) + '\n')
        
        # Drone altitude fixes during solution phase
        if frame_idx == 1700:
            for drone in self.drones:
                event = build_merge_event(
                    timestamp_ns=timestamp_ns,
                    event_code="PANCAKE_FIXED",
                    details={
                        "object_id": f"drone_{drone.id}",
                        "corrected_altitude": DRONE_ALTITUDE,
                        "method": "h3_voxel_grid"
                    }
                )
                self.events_file.write(json.dumps(event) + '\n')
    
    def run(self):
        """Run the scenario and generate all outputs."""
        print("=" * 60)
        print("GODVIEW V2 SCENARIO RUNNER")
        print("=" * 60)
        print(f"Duration: {SIMULATION_FRAMES / FPS}s | Frames: {SIMULATION_FRAMES} @ {FPS} FPS")
        print("=" * 60)
        
        # Setup
        tm = self.setup_world()
        
        # Start recording (MUST be before spawning to capture spawn events)
        recording_file = str(OUTPUT_DIR / "godview_demo.log")
        self.client.start_recorder(recording_file)
        print(f"[RECORD] Started recording to {recording_file}")
        
        # Spawn actors
        self.spawn_actors(tm)
        
        # Main loop
        start_time = time.time()
        
        for frame_idx in range(SIMULATION_FRAMES):
            # Tick world
            self.world.tick()
            
            # Calculate timestamp
            timestamp_ns = int((frame_idx / FPS) * 1e9)
            
            # Generate data
            self.generate_raw_detections(frame_idx, timestamp_ns)
            self.generate_fused_state(frame_idx, timestamp_ns)
            self.generate_merge_events(frame_idx, timestamp_ns)
            
            # Teleport drones to altitude (keep them hovering)
            for drone in self.drones:
                if drone.is_alive:
                    transform = drone.get_transform()
                    transform.location.z = DRONE_ALTITUDE
                    drone.set_transform(transform)
            
            # Progress
            if frame_idx % (FPS * 5) == 0:  # Every 5 seconds
                elapsed = time.time() - start_time
                progress = (frame_idx / SIMULATION_FRAMES) * 100
                print(f"[{int(frame_idx/FPS)}s] Frame {frame_idx}/{SIMULATION_FRAMES} ({progress:.0f}%) | Real: {elapsed:.1f}s")
        
        # Stop recording
        self.client.stop_recorder()
        
        # Cleanup
        self.raw_file.close()
        self.fused_file.close()
        self.events_file.close()
        
        print("=" * 60)
        print("SCENARIO COMPLETE")
        print("=" * 60)
        print(f"Stats: {self.stats}")
        print(f"Output files in: {OUTPUT_DIR}")
        print("=" * 60)
    
    def cleanup(self):
        """Destroy all actors."""
        print("[CLEANUP] Destroying actors...")
        
        for vehicle in self.vehicles:
            if vehicle.is_alive:
                vehicle.destroy()
        
        for drone in self.drones:
            if drone.is_alive:
                drone.destroy()
        
        # Reset world settings
        settings = self.world.get_settings()
        settings.synchronous_mode = False
        self.world.apply_settings(settings)
        
        print("[CLEANUP] Complete!")


def main():
    parser = argparse.ArgumentParser(description="GodView V2 Scenario Runner")
    parser.add_argument("--host", default="localhost", help="CARLA host")
    parser.add_argument("--port", type=int, default=2000, help="CARLA port")
    args = parser.parse_args()
    
    print(f"[INIT] Connecting to CARLA at {args.host}:{args.port}")
    client = carla.Client(args.host, args.port)
    client.set_timeout(30.0)
    
    world = client.get_world()
    print(f"[INIT] Connected to {world.get_map().name}")
    
    runner = ScenarioRunner(client, world)
    
    try:
        runner.run()
    finally:
        runner.cleanup()


if __name__ == "__main__":
    main()
