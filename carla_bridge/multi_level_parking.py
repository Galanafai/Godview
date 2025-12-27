#!/usr/bin/env python3
"""
Multi-Level Parking Scenario - 3D Spatial Indexing Test

This scenario tests GodView's 3D spatial indexing capabilities
(H3 + Sparse Voxel Octree) in a vertically-stacked environment.

The parking structure has multiple levels, each at different heights.
Vehicles drive on ramps between levels, testing:

1. Vertical spatial queries (find vehicles on level 3)
2. H3 cell + altitude filtering
3. Cross-level occlusion (vehicle on level 1 hidden from level 3)
4. Track handoff as vehicles transition between levels

Usage:
    python3 multi_level_parking.py --levels 3 --vehicles-per-level 5
"""

import carla
import numpy as np
import time
import json
import struct
import argparse
from dataclasses import dataclass
from typing import List, Dict, Tuple

try:
    import zmq
except ImportError:
    print("Install pyzmq: pip install pyzmq")
    exit(1)


@dataclass
class ParkingLevel:
    """Definition of a parking structure level."""
    level_id: int
    z_height: float  # meters above ground
    spawn_points: List[Tuple[float, float, float]]  # (x, y, z)
    color: Tuple[int, int, int]  # Level indicator color


class MultiLevelParkingScenario:
    """
    Multi-level parking structure scenario.
    
    Features:
    - Multiple parking levels at different heights
    - Vehicles spawned on each level
    - Ramp connections between levels
    - Tests 3D spatial indexing
    """
    
    def __init__(
        self,
        carla_host: str = 'localhost',
        carla_port: int = 2000,
        num_levels: int = 3,
        level_height: float = 4.0,  # meters between levels
    ):
        self.num_levels = num_levels
        self.level_height = level_height
        
        # Connect to CARLA
        print(f"🔌 Connecting to CARLA at {carla_host}:{carla_port}...")
        self.client = carla.Client(carla_host, carla_port)
        self.client.set_timeout(10.0)
        self.world = self.client.get_world()
        
        # Configure sync mode
        settings = self.world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        settings.no_rendering_mode = True
        self.world.apply_settings(settings)
        
        print(f"✅ Connected to: {self.world.get_map().name}")
        
        # ZMQ
        self.context = zmq.Context()
        self.telemetry_socket = self.context.socket(zmq.PUB)
        self.telemetry_socket.setsockopt(zmq.CONFLATE, 1)
        self.telemetry_socket.bind("tcp://127.0.0.1:5555")
        
        # Level metadata socket
        self.metadata_socket = self.context.socket(zmq.PUB)
        self.metadata_socket.bind("tcp://127.0.0.1:5556")
        
        print(f"📡 ZMQ bound on ports 5555 (telemetry) and 5556 (metadata)")
        
        # Parking levels
        self.levels: List[ParkingLevel] = []
        self.vehicles_by_level: Dict[int, List[carla.Actor]] = {}
        
        # Level colors (for visualization)
        self.level_colors = [
            (255, 100, 100),  # Level 0: Red
            (100, 255, 100),  # Level 1: Green
            (100, 100, 255),  # Level 2: Blue
            (255, 255, 100),  # Level 3: Yellow
            (255, 100, 255),  # Level 4: Magenta
        ]
    
    def create_parking_structure(self, center: Tuple[float, float], size: float = 50.0):
        """
        Generate parking level spawn points.
        
        In a real implementation, this would use actual parking garage map.
        Here we simulate by placing vehicles at different Z heights.
        """
        print(f"\n🏗️  Creating {self.num_levels}-level parking structure:")
        
        base_spawn_points = self.world.get_map().get_spawn_points()
        
        # Group spawn points that are close to our center
        nearby_spawns = []
        for sp in base_spawn_points:
            dx = sp.location.x - center[0]
            dy = sp.location.y - center[1]
            if abs(dx) < size/2 and abs(dy) < size/2:
                nearby_spawns.append(sp)
        
        if not nearby_spawns:
            print("   Using first spawn points (no nearby ones found)")
            nearby_spawns = base_spawn_points[:20]
        
        # Create levels
        spawns_per_level = max(2, len(nearby_spawns) // self.num_levels)
        
        for level_id in range(self.num_levels):
            z_height = level_id * self.level_height
            color = self.level_colors[level_id % len(self.level_colors)]
            
            # Assign spawn points to this level
            start_idx = level_id * spawns_per_level
            end_idx = start_idx + spawns_per_level
            level_spawns = nearby_spawns[start_idx:end_idx]
            
            level = ParkingLevel(
                level_id=level_id,
                z_height=z_height,
                spawn_points=[(sp.location.x, sp.location.y, sp.location.z + z_height) 
                              for sp in level_spawns],
                color=color,
            )
            self.levels.append(level)
            
            print(f"   Level {level_id}: z={z_height:.1f}m, {len(level_spawns)} spots, color={color}")
        
        # Publish structure metadata
        self.metadata_socket.send_multipart([
            b'structure',
            json.dumps({
                'type': 'multi_level_parking',
                'num_levels': self.num_levels,
                'level_height': self.level_height,
                'levels': [
                    {
                        'id': l.level_id,
                        'z_height': l.z_height,
                        'color': list(l.color),
                        'spawn_count': len(l.spawn_points),
                    }
                    for l in self.levels
                ]
            }).encode()
        ])
    
    def spawn_vehicles(self, vehicles_per_level: int = 3):
        """Spawn vehicles on each level."""
        print(f"\n🚗 Spawning vehicles ({vehicles_per_level} per level)...")
        
        # Clean up existing
        for actor in self.world.get_actors().filter('vehicle.*'):
            actor.destroy()
        self.world.tick()
        
        bp_library = self.world.get_blueprint_library()
        vehicle_bps = list(bp_library.filter('vehicle.*'))
        
        for level in self.levels:
            self.vehicles_by_level[level.level_id] = []
            spawned = 0
            
            for i, (x, y, z) in enumerate(level.spawn_points[:vehicles_per_level]):
                bp = vehicle_bps[i % len(vehicle_bps)]
                
                # Set color based on level
                if bp.has_attribute('color'):
                    bp.set_attribute('color', f'{level.color[0]},{level.color[1]},{level.color[2]}')
                
                transform = carla.Transform(
                    carla.Location(x=x, y=y, z=z),
                    carla.Rotation()
                )
                
                try:
                    vehicle = self.world.spawn_actor(bp, transform)
                    vehicle.set_autopilot(True)
                    self.vehicles_by_level[level.level_id].append(vehicle)
                    spawned += 1
                    
                    # Publish spawn event with level info
                    self.metadata_socket.send_multipart([
                        b'spawn',
                        json.dumps({
                            'actor_id': vehicle.id,
                            'level_id': level.level_id,
                            'z_height': z,
                            'color': list(level.color),
                            'type': 'vehicle',
                        }).encode()
                    ])
                except Exception as e:
                    pass
            
            print(f"   Level {level.level_id}: spawned {spawned} vehicles")
        
        self.world.tick()
        
        total = sum(len(v) for v in self.vehicles_by_level.values())
        print(f"✅ Total: {total} vehicles across {self.num_levels} levels")
    
    def run(self, duration: float = 60.0):
        """Run the multi-level parking scenario."""
        print(f"\n🏁 Starting Multi-Level Parking Scenario")
        print(f"   Duration: {duration}s")
        print(f"   Testing: 3D spatial indexing (H3 + Octree)")
        print()
        print("=" * 60)
        
        start_time = time.time()
        frame_id = 0
        last_print = start_time
        
        # Track level transitions
        level_transitions = 0
        actor_levels: Dict[int, int] = {}  # actor_id -> current level
        
        try:
            while time.time() - start_time < duration:
                self.world.tick()
                snapshot = self.world.get_snapshot()
                sim_time = snapshot.timestamp.elapsed_seconds
                
                # Build telemetry with level annotations
                all_actors = []
                for level_id, vehicles in self.vehicles_by_level.items():
                    for v in vehicles:
                        if not v.is_alive:
                            continue
                        
                        snap = snapshot.find(v.id)
                        if not snap:
                            continue
                        
                        t = snap.get_transform()
                        vel = snap.get_velocity()
                        
                        # Detect level transitions (vehicle moving between floors)
                        current_z = t.location.z
                        estimated_level = int(current_z / self.level_height)
                        
                        if v.id in actor_levels:
                            if actor_levels[v.id] != estimated_level:
                                level_transitions += 1
                        actor_levels[v.id] = estimated_level
                        
                        all_actors.append({
                            'id': v.id,
                            'pos': [t.location.x, t.location.y, t.location.z],
                            'rot': [t.rotation.pitch, t.rotation.yaw, t.rotation.roll],
                            'vel': [vel.x, vel.y, vel.z],
                            'level': estimated_level,
                        })
                
                # Publish binary telemetry
                if all_actors:
                    packet = self._build_packet(frame_id, sim_time, all_actors)
                    self.telemetry_socket.send(packet, zmq.NOBLOCK)
                
                frame_id += 1
                
                # Print progress with level distribution
                if time.time() - last_print >= 2.0:
                    level_counts = {}
                    for level in actor_levels.values():
                        level_counts[level] = level_counts.get(level, 0) + 1
                    
                    level_str = " ".join([f"L{k}:{v}" for k, v in sorted(level_counts.items())])
                    
                    print(f"⏱️  t={sim_time:.1f}s | Frame {frame_id} | "
                          f"Transitions: {level_transitions} | {level_str}")
                    last_print = time.time()
        
        except KeyboardInterrupt:
            print("\n⚠️  Interrupted")
        
        finally:
            self._cleanup()
    
    def _build_packet(self, frame_id: int, sim_time: float, actors: List[dict]) -> bytes:
        """Build binary telemetry packet."""
        header = struct.pack('<QdII', frame_id, sim_time, len(actors), 0)
        
        actor_data = bytearray()
        for a in actors:
            actor_data.extend(struct.pack('<I', a['id']))
            actor_data.extend(struct.pack('<fff', *a['pos']))
            actor_data.extend(struct.pack('<fff', *a['rot']))
            actor_data.extend(struct.pack('<fff', *a['vel']))
        
        return header + bytes(actor_data)
    
    def _cleanup(self):
        """Clean up resources."""
        print("\n🧹 Cleaning up...")
        
        settings = self.world.get_settings()
        settings.synchronous_mode = False
        settings.no_rendering_mode = False
        self.world.apply_settings(settings)
        
        self.telemetry_socket.close()
        self.metadata_socket.close()
        self.context.term()
        
        print("✅ Complete")


def main():
    parser = argparse.ArgumentParser(description='Multi-Level Parking 3D Scenario')
    parser.add_argument('--host', default='localhost')
    parser.add_argument('--port', type=int, default=2000)
    parser.add_argument('--duration', type=float, default=60.0)
    parser.add_argument('--levels', type=int, default=3, help='Number of parking levels')
    parser.add_argument('--vehicles-per-level', type=int, default=3)
    parser.add_argument('--level-height', type=float, default=4.0, help='Height between levels (m)')
    parser.add_argument('--center-x', type=float, default=0.0)
    parser.add_argument('--center-y', type=float, default=0.0)
    
    args = parser.parse_args()
    
    scenario = MultiLevelParkingScenario(
        carla_host=args.host,
        carla_port=args.port,
        num_levels=args.levels,
        level_height=args.level_height,
    )
    
    scenario.create_parking_structure((args.center_x, args.center_y))
    scenario.spawn_vehicles(args.vehicles_per_level)
    scenario.run(args.duration)


if __name__ == '__main__':
    main()
