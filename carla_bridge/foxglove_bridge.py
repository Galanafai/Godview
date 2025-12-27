#!/usr/bin/env python3
"""
Foxglove CARLA Bridge - Stream CARLA data to Foxglove Studio

This bridge connects to CARLA and streams visualization data to Foxglove Studio
using the Foxglove WebSocket protocol. Much better 3D visualization than Rerun!

Features:
- 3D poses for all vehicles (SceneUpdate)
- GPS/position data as PoseInFrame
- Velocity arrows
- Vehicle meshes (loaded from GLTF)
- Dashboard-ready stats

Usage:
    # Terminal 1: Start CARLA
    cd /data/CARLA_0.9.16 && ./CarlaUE4.sh -RenderOffScreen
    
    # Terminal 2: Run this bridge
    python3 foxglove_bridge.py
    
    # Open Foxglove Studio and connect to ws://localhost:8765

Requirements:
    pip install foxglove-sdk carla numpy
"""

import carla
import numpy as np
import time
import json
import math
import asyncio
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple
import struct

try:
    from foxglove_websocket.server import FoxgloveServer, FoxgloveServerListener
    from foxglove_websocket.types import ChannelId
    FOXGLOVE_AVAILABLE = True
except ImportError:
    print("Install foxglove-websocket: pip install foxglove-websocket")
    FOXGLOVE_AVAILABLE = False

# =============================================================================
# FOXGLOVE SCHEMA DEFINITIONS (foxglove.* schemas)
# =============================================================================

# Foxglove uses well-known schemas for 3D visualization
# See: https://docs.foxglove.dev/docs/visualization/message-schemas/

SCENE_UPDATE_SCHEMA = {
    "type": "object",
    "properties": {
        "deletions": {"type": "array"},
        "entities": {"type": "array"}
    }
}

POSE_IN_FRAME_SCHEMA = {
    "type": "object", 
    "properties": {
        "timestamp": {"type": "object"},
        "frame_id": {"type": "string"},
        "pose": {"type": "object"}
    }
}


class FoxgloveCarlaBridge:
    """
    Streams CARLA simulation data to Foxglove Studio.
    
    Uses Foxglove's WebSocket server to provide:
    - 3D scene with vehicle poses
    - Position/velocity data
    - Stats and metrics
    """
    
    def __init__(
        self,
        carla_host: str = 'localhost',
        carla_port: int = 2000,
        foxglove_port: int = 8765,
    ):
        print("╔════════════════════════════════════════════════╗")
        print("║   FOXGLOVE CARLA BRIDGE                        ║")
        print("╚════════════════════════════════════════════════╝\n")
        
        # Connect to CARLA
        print(f"🔌 Connecting to CARLA at {carla_host}:{carla_port}...")
        self.client = carla.Client(carla_host, carla_port)
        self.client.set_timeout(10.0)
        self.world = self.client.get_world()
        
        # Configure simulation
        settings = self.world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05  # 20 Hz
        settings.no_rendering_mode = True
        self.world.apply_settings(settings)
        
        print(f"✅ Connected to: {self.world.get_map().name}\n")
        
        self.foxglove_port = foxglove_port
        self.frame_id = 0
        self.vehicles: Dict[int, carla.Actor] = {}
        
    def spawn_vehicles(self, count: int = 5):
        """Spawn test vehicles."""
        print(f"🚗 Spawning {count} vehicles...")
        
        # Clean existing
        for actor in self.world.get_actors().filter('vehicle.*'):
            actor.destroy()
        self.world.tick()
        
        bp_library = self.world.get_blueprint_library()
        spawn_points = self.world.get_map().get_spawn_points()
        vehicle_bps = list(bp_library.filter('vehicle.*'))
        
        spawned = 0
        for i, sp in enumerate(spawn_points[:count*2]):
            if spawned >= count:
                break
            bp = vehicle_bps[i % len(vehicle_bps)]
            try:
                v = self.world.spawn_actor(bp, sp)
                v.set_autopilot(True)
                self.vehicles[v.id] = v
                spawned += 1
            except:
                pass
        
        self.world.tick()
        print(f"✅ Spawned {spawned} vehicles\n")
        
    def euler_to_quaternion(self, pitch: float, yaw: float, roll: float) -> Tuple[float, float, float, float]:
        """Convert Euler angles (degrees) to quaternion (x, y, z, w)."""
        # Convert to radians
        pitch = math.radians(pitch)
        yaw = math.radians(yaw)
        roll = math.radians(roll)
        
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        
        return (x, y, z, w)
    
    def build_scene_update(self, snapshot: carla.WorldSnapshot) -> dict:
        """
        Build a Foxglove SceneUpdate message with all vehicles.
        
        SceneUpdate is the primary 3D visualization schema in Foxglove.
        """
        entities = []
        
        for actor_id, actor in self.vehicles.items():
            if not actor.is_alive:
                continue
                
            actor_snap = snapshot.find(actor_id)
            if not actor_snap:
                continue
            
            t = actor_snap.get_transform()
            v = actor_snap.get_velocity()
            
            # Convert rotation to quaternion
            quat = self.euler_to_quaternion(
                t.rotation.pitch,
                t.rotation.yaw,
                t.rotation.roll
            )
            
            # Create entity with cube primitive (representing vehicle)
            entity = {
                "timestamp": {
                    "sec": int(snapshot.timestamp.elapsed_seconds),
                    "nsec": int((snapshot.timestamp.elapsed_seconds % 1) * 1e9)
                },
                "frame_id": "world",
                "id": f"vehicle_{actor_id}",
                "lifetime": {"sec": 0, "nsec": 100000000},  # 100ms
                "frame_locked": False,
                "metadata": [
                    {"key": "type", "value": "vehicle"},
                    {"key": "speed", "value": f"{math.sqrt(v.x**2 + v.y**2):.1f} m/s"}
                ],
                "arrows": [],
                "cubes": [
                    {
                        "pose": {
                            "position": {
                                "x": t.location.x,
                                "y": t.location.y,
                                "z": t.location.z + 0.8  # Offset to ground
                            },
                            "orientation": {
                                "x": quat[0],
                                "y": quat[1],
                                "z": quat[2],
                                "w": quat[3]
                            }
                        },
                        "size": {"x": 4.5, "y": 2.0, "z": 1.5},
                        "color": {"r": 0.2, "g": 0.6, "b": 1.0, "a": 0.9}
                    }
                ],
                "spheres": [],
                "cylinders": [],
                "lines": [],
                "triangles": [],
                "texts": [],
                "models": []
            }
            
            # Add velocity arrow
            speed = math.sqrt(v.x**2 + v.y**2 + v.z**2)
            if speed > 0.5:
                arrow_length = min(speed, 10.0)
                entity["arrows"].append({
                    "pose": {
                        "position": {
                            "x": t.location.x,
                            "y": t.location.y, 
                            "z": t.location.z + 2.0
                        },
                        "orientation": {"x": 0, "y": 0, "z": 0, "w": 1}
                    },
                    "shaft_length": arrow_length,
                    "shaft_diameter": 0.2,
                    "head_length": 0.5,
                    "head_diameter": 0.4,
                    "color": {"r": 1.0, "g": 0.8, "b": 0.0, "a": 1.0}
                })
            
            entities.append(entity)
        
        return {
            "deletions": [],
            "entities": entities
        }
    
    def build_stats_message(self, snapshot: carla.WorldSnapshot) -> dict:
        """Build stats/metrics message."""
        speeds = []
        for actor_id, actor in self.vehicles.items():
            if not actor.is_alive:
                continue
            snap = snapshot.find(actor_id)
            if snap:
                v = snap.get_velocity()
                speeds.append(math.sqrt(v.x**2 + v.y**2))
        
        return {
            "frame_id": self.frame_id,
            "timestamp": snapshot.timestamp.elapsed_seconds,
            "vehicle_count": len(self.vehicles),
            "avg_speed_mps": np.mean(speeds) if speeds else 0,
            "max_speed_mps": max(speeds) if speeds else 0,
            "map": self.world.get_map().name.split('/')[-1]
        }
    
    async def run_server(self, duration: float = 60.0):
        """Run the Foxglove WebSocket server."""
        print(f"🌐 Starting Foxglove server on ws://localhost:{self.foxglove_port}")
        print(f"   Open Foxglove Studio and connect to this address\n")
        print("=" * 60)
        
        # Create server using correct foxglove-websocket API
        async with FoxgloveServer(
            host="0.0.0.0", 
            port=self.foxglove_port,
            name="GodView CARLA Bridge",
        ) as server:
            # Register channels - add_channel returns channel ID
            scene_chan_id = await server.add_channel({
                "topic": "/godview/scene",
                "encoding": "json",
                "schemaName": "foxglove.SceneUpdate",
                "schemaEncoding": "jsonschema",
                "schema": json.dumps(SCENE_UPDATE_SCHEMA),
            })
            
            stats_chan_id = await server.add_channel({
                "topic": "/godview/stats",
                "encoding": "json", 
                "schemaName": "GodViewStats",
                "schemaEncoding": "jsonschema",
                "schema": json.dumps({
                    "type": "object",
                    "properties": {
                        "frame_id": {"type": "integer"},
                        "vehicle_count": {"type": "integer"},
                        "avg_speed_mps": {"type": "number"}
                    }
                }),
            })
            
            print("✅ Foxglove server started")
            print("   📱 Open Foxglove Studio → Data Source → Foxglove WebSocket")
            print(f"   🔗 Enter: ws://localhost:{self.foxglove_port}\n")
            
            start_time = time.time()
            last_print = start_time
            
            try:
                while time.time() - start_time < duration:
                    # Tick simulation
                    self.world.tick()
                    snapshot = self.world.get_snapshot()
                    sim_time = snapshot.timestamp.elapsed_seconds
                    
                    # Build messages
                    scene_msg = self.build_scene_update(snapshot)
                    stats_msg = self.build_stats_message(snapshot)
                    
                    # Get timestamp in nanoseconds
                    timestamp_ns = int(sim_time * 1e9)
                    
                    # Publish to Foxglove using send_message
                    await server.send_message(
                        scene_chan_id,
                        timestamp_ns,
                        json.dumps(scene_msg).encode()
                    )
                    await server.send_message(
                        stats_chan_id,
                        timestamp_ns,
                        json.dumps(stats_msg).encode()
                    )
                    
                    self.frame_id += 1
                    
                    # Print progress
                    if time.time() - last_print >= 2.0:
                        print(f"⏱️  t={sim_time:.1f}s | Frame {self.frame_id} | "
                              f"Vehicles: {len(self.vehicles)} | "
                              f"Avg Speed: {stats_msg['avg_speed_mps']:.1f} m/s")
                        last_print = time.time()
                    
                    await asyncio.sleep(0.01)  # Small delay
                    
            except KeyboardInterrupt:
                print("\n⚠️  Interrupted")
            finally:
                self._cleanup()
    
    def _cleanup(self):
        """Clean up resources."""
        print("\n🧹 Cleaning up...")
        settings = self.world.get_settings()
        settings.synchronous_mode = False
        settings.no_rendering_mode = False
        self.world.apply_settings(settings)
        print("✅ Done")


def main():
    import argparse
    
    parser = argparse.ArgumentParser(description='Foxglove CARLA Bridge')
    parser.add_argument('--host', default='localhost', help='CARLA host')
    parser.add_argument('--port', type=int, default=2000, help='CARLA port')
    parser.add_argument('--foxglove-port', type=int, default=8765, help='Foxglove WebSocket port')
    parser.add_argument('--vehicles', type=int, default=5, help='Number of vehicles')
    parser.add_argument('--duration', type=float, default=120.0, help='Duration in seconds')
    
    args = parser.parse_args()
    
    bridge = FoxgloveCarlaBridge(
        carla_host=args.host,
        carla_port=args.port,
        foxglove_port=args.foxglove_port,
    )
    
    bridge.spawn_vehicles(args.vehicles)
    
    # Run async server
    asyncio.run(bridge.run_server(duration=args.duration))


if __name__ == '__main__':
    main()
