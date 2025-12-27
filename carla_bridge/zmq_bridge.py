#!/usr/bin/env python3
"""
GodView ZMQ CARLA Bridge - High Performance Data Pipeline

Based on architectural specifications in cara_sim_imp.md:
- ZeroMQ PUB/SUB transport (low latency, brokerless)
- Numpy structured arrays (zero-copy on Rust side)
- WorldSnapshot batch retrieval (single API call)
- Static/Dynamic registry pattern

Hardware Optimization: GTX 1050 Ti (4GB VRAM)
- No-Rendering mode for simulation
- RenderOffScreen for camera sensors only
"""

import carla
import numpy as np
import time
import json
import struct
import sys
from pathlib import Path
from typing import Dict, List, Optional, Tuple
from dataclasses import dataclass
from collections import deque

try:
    import zmq
    ZMQ_AVAILABLE = True
except ImportError:
    print("⚠️  ZeroMQ not available. Install with: pip install pyzmq")
    ZMQ_AVAILABLE = False

try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    print("⚠️  YOLOv8 not available. Install with: pip install ultralytics")
    YOLO_AVAILABLE = False


# =============================================================================
# BINARY PROTOCOL DEFINITIONS
# =============================================================================

# Header format: [frame_id: u64, timestamp: f64, actor_count: u32, padding: u32]
HEADER_FORMAT = '<QdII'  # 24 bytes
HEADER_SIZE = struct.calcsize(HEADER_FORMAT)

# Per-actor update: matches Rust #[repr(C)] struct
# ActorUpdate { id: u32, pos: [f32; 3], rot: [f32; 3], vel: [f32; 3] }
ACTOR_DTYPE = np.dtype([
    ('id', '<u4'),        # 4 bytes
    ('pos', '<3f4'),      # 12 bytes [x, y, z]
    ('rot', '<3f4'),      # 12 bytes [pitch, yaw, roll]
    ('vel', '<3f4'),      # 12 bytes [vx, vy, vz]
], align=True)  # Total: 40 bytes per actor

# Static actor metadata (sent once on spawn)
@dataclass
class ActorMetadata:
    actor_id: int
    actor_type: str  # "vehicle", "pedestrian", "cyclist"
    model: str       # "vehicle.tesla.model3"
    color: Tuple[int, int, int, int]  # RGBA
    

class ZMQPublisher:
    """
    High-performance ZMQ publisher for CARLA telemetry.
    
    Uses ZMQ_CONFLATE to keep only the latest message in queue,
    preventing latency buildup if consumer falls behind.
    """
    
    def __init__(self, telemetry_port: int = 5555, metadata_port: int = 5556):
        if not ZMQ_AVAILABLE:
            raise RuntimeError("pyzmq not installed")
        
        self.context = zmq.Context()
        
        # Telemetry socket (high-frequency, binary)
        self.telemetry_socket = self.context.socket(zmq.PUB)
        self.telemetry_socket.setsockopt(zmq.CONFLATE, 1)  # Keep only latest
        self.telemetry_socket.setsockopt(zmq.SNDHWM, 1)    # Send high water mark
        self.telemetry_socket.bind(f"tcp://127.0.0.1:{telemetry_port}")
        
        # Metadata socket (low-frequency, JSON)
        self.metadata_socket = self.context.socket(zmq.PUB)
        self.metadata_socket.bind(f"tcp://127.0.0.1:{metadata_port}")
        
        self.telemetry_port = telemetry_port
        self.metadata_port = metadata_port
        
        print(f"📡 ZMQ Publisher initialized:")
        print(f"   Telemetry: tcp://127.0.0.1:{telemetry_port}")
        print(f"   Metadata:  tcp://127.0.0.1:{metadata_port}")
    
    def publish_telemetry(self, frame_id: int, sim_time: float, actors: np.ndarray):
        """
        Publish binary telemetry packet.
        
        Packet format:
        [Header: 24 bytes][Actor0: 40 bytes][Actor1: 40 bytes]...
        """
        actor_count = len(actors)
        
        # Build header
        header = struct.pack(HEADER_FORMAT, frame_id, sim_time, actor_count, 0)
        
        # Combine header + actor data
        payload = header + actors.tobytes()
        
        # Send (non-blocking due to CONFLATE)
        self.telemetry_socket.send(payload, zmq.NOBLOCK)
    
    def publish_metadata(self, topic, metadata: dict):
        """Publish JSON metadata (spawn events, static properties)."""
        message = json.dumps(metadata).encode('utf-8')
        # Handle both str and bytes for topic
        topic_bytes = topic.encode() if isinstance(topic, str) else topic
        self.metadata_socket.send_multipart([topic_bytes, message], zmq.NOBLOCK)
    
    def close(self):
        self.telemetry_socket.close()
        self.metadata_socket.close()
        self.context.term()


class GodViewZMQBridge:
    """
    High-performance CARLA to GodView bridge using ZeroMQ.
    
    Key optimizations from cara_sim_imp.md:
    1. WorldSnapshot batch retrieval (single API call)
    2. Numpy structured arrays (zero-copy serialization)
    3. ZMQ PUB/SUB with CONFLATE (prevents latency buildup)
    4. Static/Dynamic registry (metadata sent once)
    """
    
    def __init__(
        self,
        carla_host: str = 'localhost',
        carla_port: int = 2000,
        telemetry_port: int = 5555,
        metadata_port: int = 5556,
        no_rendering: bool = True,
        fixed_delta: float = 0.05,  # 20 Hz
    ):
        print("╔════════════════════════════════════════════════╗")
        print("║   GODVIEW ZMQ BRIDGE (High Performance Mode)  ║")
        print("╚════════════════════════════════════════════════╝")
        print()
        
        # Connect to CARLA
        print(f"🔌 Connecting to CARLA at {carla_host}:{carla_port}...")
        self.client = carla.Client(carla_host, carla_port)
        self.client.set_timeout(10.0)
        self.world = self.client.get_world()
        
        # Apply simulation settings
        self._configure_simulation(no_rendering, fixed_delta)
        
        print(f"✅ Connected to CARLA world: {self.world.get_map().name}")
        print()
        
        # Initialize ZMQ publisher
        self.publisher = ZMQPublisher(telemetry_port, metadata_port)
        print()
        
        # Actor registry (Static/Dynamic pattern)
        self.known_actors: Dict[int, ActorMetadata] = {}
        self.frame_id = 0
        
        # Performance tracking
        self.tick_times: deque = deque(maxlen=100)
        
    def _configure_simulation(self, no_rendering: bool, fixed_delta: float):
        """Configure CARLA for optimal performance."""
        settings = self.world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = fixed_delta
        settings.no_rendering_mode = no_rendering
        self.world.apply_settings(settings)
        
        self.fixed_delta = fixed_delta
        self.no_rendering = no_rendering
        
        print(f"⚙️  Simulation settings:")
        print(f"   Synchronous mode: ON")
        print(f"   Fixed delta: {fixed_delta}s ({1/fixed_delta:.0f} Hz)")
        print(f"   No-Rendering mode: {'ON ✓' if no_rendering else 'OFF'}")
        
        if no_rendering:
            print("   ⚡ GPU freed for compute-only (Lidar, physics)")
    
    def _extract_actor_state(self, snapshot: carla.WorldSnapshot) -> np.ndarray:
        """
        Extract actor states from WorldSnapshot into Numpy array.
        
        This is the key optimization: single API call instead of
        iterating through actors with individual get_transform() calls.
        """
        # Get all vehicle and pedestrian actors
        actors = self.world.get_actors()
        vehicles = actors.filter('vehicle.*')
        pedestrians = actors.filter('walker.pedestrian.*')
        
        all_actors = list(vehicles) + list(pedestrians)
        if not all_actors:
            return np.array([], dtype=ACTOR_DTYPE)
        
        # Pre-allocate array
        actor_data = np.zeros(len(all_actors), dtype=ACTOR_DTYPE)
        
        for i, actor in enumerate(all_actors):
            # Check if we need to register this actor (new spawn)
            if actor.id not in self.known_actors:
                self._register_new_actor(actor)
            
            # Get transforms from snapshot for this actor
            actor_snapshot = snapshot.find(actor.id)
            if actor_snapshot is None:
                continue
            
            transform = actor_snapshot.get_transform()
            velocity = actor_snapshot.get_velocity()
            
            # Fill structured array
            actor_data[i]['id'] = actor.id
            actor_data[i]['pos'] = [
                transform.location.x,
                transform.location.y,
                transform.location.z
            ]
            actor_data[i]['rot'] = [
                transform.rotation.pitch,
                transform.rotation.yaw,
                transform.rotation.roll
            ]
            actor_data[i]['vel'] = [velocity.x, velocity.y, velocity.z]
        
        return actor_data
    
    def _register_new_actor(self, actor: carla.Actor):
        """Register new actor and publish metadata."""
        # Determine actor type
        type_id = actor.type_id
        if 'vehicle' in type_id:
            actor_type = 'vehicle'
        elif 'walker' in type_id:
            actor_type = 'pedestrian'
        else:
            actor_type = 'unknown'
        
        # Get color if available
        color = (128, 128, 128, 255)  # Default gray
        if hasattr(actor, 'attributes'):
            attrs = actor.attributes
            if 'color' in attrs:
                try:
                    r, g, b = map(int, attrs['color'].split(','))
                    color = (r, g, b, 255)
                except:
                    pass
        
        metadata = ActorMetadata(
            actor_id=actor.id,
            actor_type=actor_type,
            model=type_id,
            color=color
        )
        
        self.known_actors[actor.id] = metadata
        
        # Publish spawn event
        self.publisher.publish_metadata('spawn', {
            'actor_id': actor.id,
            'actor_type': actor_type,
            'model': type_id,
            'color': list(color)
        })
    
    def spawn_traffic(self, num_vehicles: int = 10, num_pedestrians: int = 5):
        """Spawn traffic for testing."""
        print(f"🚗 Spawning traffic: {num_vehicles} vehicles, {num_pedestrians} pedestrians...")
        
        # First, clean up any existing vehicles from previous runs
        existing_actors = self.world.get_actors()
        existing_vehicles = existing_actors.filter('vehicle.*')
        if len(existing_vehicles) > 0:
            print(f"   Cleaning up {len(existing_vehicles)} existing vehicles...")
            for v in existing_vehicles:
                v.destroy()
            # Tick to let cleanup complete
            self.world.tick()
        
        blueprint_library = self.world.get_blueprint_library()
        spawn_points = self.world.get_map().get_spawn_points()
        
        if not spawn_points:
            print("⚠️  No spawn points available on this map!")
            return
        
        print(f"   Found {len(spawn_points)} spawn points on map")
        
        # Spawn vehicles with retry on different spawn points
        vehicle_bps = list(blueprint_library.filter('vehicle.*'))
        spawned_vehicles = 0
        spawn_idx = 0
        
        while spawned_vehicles < num_vehicles and spawn_idx < len(spawn_points):
            bp = np.random.choice(vehicle_bps)
            # Try to make it drivable (4 wheels)
            if bp.has_attribute('number_of_wheels'):
                if int(bp.get_attribute('number_of_wheels')) != 4:
                    spawn_idx += 1
                    continue
            
            try:
                vehicle = self.world.spawn_actor(bp, spawn_points[spawn_idx])
                vehicle.set_autopilot(True)
                spawned_vehicles += 1
                print(f"   ✓ Spawned {bp.id} at point {spawn_idx}")
            except Exception as e:
                # Spawn point might be occupied, try next one
                pass
            spawn_idx += 1
        
        # Tick to let vehicles initialize
        self.world.tick()
        
        print(f"✅ Spawned {spawned_vehicles} vehicles")
        print()
    
    def run(self, duration: float = 60.0, stats_interval: float = 1.0):
        """
        Main simulation loop.
        
        Uses WorldSnapshot for batch state retrieval (cara_sim_imp.md Section 5.1.2)
        """
        print(f"🎬 Running simulation for {duration}s...")
        print(f"   Publishing telemetry to ZMQ at {1/self.fixed_delta:.0f} Hz")
        print("   Press Ctrl+C to stop")
        print()
        print("=" * 60)
        
        start_time = time.time()
        last_stats_time = start_time
        
        try:
            while time.time() - start_time < duration:
                tick_start = time.perf_counter()
                
                # Advance simulation (synchronous mode)
                # In CARLA 0.9.16, tick() returns frame number, not snapshot
                frame_number = self.world.tick()
                
                # Get snapshot separately (compatible with both 0.9.15 and 0.9.16)
                snapshot = self.world.get_snapshot()
                
                # Get simulation time from snapshot
                sim_time = snapshot.timestamp.elapsed_seconds
                
                # Extract all actor states via snapshot (batch retrieval)
                actor_data = self._extract_actor_state(snapshot)
                
                # Publish binary telemetry
                if len(actor_data) > 0:
                    self.publisher.publish_telemetry(
                        self.frame_id,
                        sim_time,
                        actor_data
                    )
                
                self.frame_id += 1
                
                # Track tick time
                tick_time = (time.perf_counter() - tick_start) * 1000
                self.tick_times.append(tick_time)
                
                # Print stats periodically
                if time.time() - last_stats_time >= stats_interval:
                    self._print_stats()
                    last_stats_time = time.time()
        
        except KeyboardInterrupt:
            print("\n⚠️  Interrupted by user")
        
        finally:
            self._cleanup()
    
    def _print_stats(self):
        """Print performance statistics."""
        if not self.tick_times:
            return
        
        avg_tick = np.mean(self.tick_times)
        max_tick = np.max(self.tick_times)
        fps = 1000 / avg_tick if avg_tick > 0 else 0
        
        print(f"⏱️  Frame {self.frame_id:5d} | "
              f"Actors: {len(self.known_actors):3d} | "
              f"Tick: {avg_tick:5.1f}ms (max {max_tick:5.1f}ms) | "
              f"FPS: {fps:5.1f}")
    
    def _cleanup(self):
        """Clean up resources."""
        print("\n🧹 Cleaning up...")
        
        # Reset simulation settings
        settings = self.world.get_settings()
        settings.synchronous_mode = False
        settings.no_rendering_mode = False
        self.world.apply_settings(settings)
        
        # Close ZMQ
        self.publisher.close()
        
        print("✅ Cleanup complete")


def main():
    import argparse
    
    parser = argparse.ArgumentParser(
        description='GodView ZMQ CARLA Bridge - High Performance Mode'
    )
    parser.add_argument('--host', default='localhost', help='CARLA server host')
    parser.add_argument('--port', type=int, default=2000, help='CARLA server port')
    parser.add_argument('--telemetry-port', type=int, default=5555, 
                        help='ZMQ telemetry port')
    parser.add_argument('--metadata-port', type=int, default=5556,
                        help='ZMQ metadata port')
    parser.add_argument('--duration', type=float, default=60.0,
                        help='Simulation duration (seconds)')
    parser.add_argument('--vehicles', type=int, default=10,
                        help='Number of vehicles to spawn')
    parser.add_argument('--rendering', action='store_true',
                        help='Enable rendering (slower, for debugging)')
    parser.add_argument('--fps', type=int, default=20,
                        help='Simulation FPS (default: 20 for GTX 1050 Ti)')
    
    args = parser.parse_args()
    
    # Create bridge
    bridge = GodViewZMQBridge(
        carla_host=args.host,
        carla_port=args.port,
        telemetry_port=args.telemetry_port,
        metadata_port=args.metadata_port,
        no_rendering=not args.rendering,
        fixed_delta=1.0 / args.fps,
    )
    
    # Spawn traffic
    bridge.spawn_traffic(num_vehicles=args.vehicles)
    
    # Run simulation
    bridge.run(duration=args.duration)


if __name__ == '__main__':
    main()
