#!/usr/bin/env python3
"""
"Seeing Around Corners" Scenario - Multi-Vehicle Collaborative Perception

This scenario demonstrates GodView's key capability: enabling vehicles to
"see" objects that are out of their direct line of sight by sharing
detections via V2X communication.

Scenario Layout:
    
                    [Building]
                       ██████
                       ██████
    Vehicle A ───────► ██████ ◄─────── Pedestrian (hidden from A)
        │              ██████              │
        │                                  │
        │                                  │
        └─────────────────────────────────-┘
                    (V2X Link)
                       ↑
                   Vehicle B ───────────► (can see pedestrian)

Vehicle A cannot see the pedestrian due to the building.
Vehicle B CAN see the pedestrian and shares this via V2X.
GodView fuses the data so Vehicle A "knows" about the pedestrian.

Usage:
    # Terminal 1: Start CARLA
    docker-compose up carla-headless
    
    # Terminal 2: Run this scenario
    python3 seeing_around_corners.py
    
    # Terminal 3: Run Rust visualization
    cargo run --example carla_demo --features carla,visualization
"""

import carla
import numpy as np
import time
import json
import sys
from dataclasses import dataclass
from typing import List, Tuple, Optional

try:
    import zmq
    ZMQ_AVAILABLE = True
except ImportError:
    print("⚠️  ZeroMQ not available. Install with: pip install pyzmq")
    ZMQ_AVAILABLE = False


# =============================================================================
# SCENARIO CONFIGURATION
# =============================================================================

@dataclass
class ScenarioConfig:
    """Configuration for the seeing around corners scenario."""
    # CARLA connection
    carla_host: str = 'localhost'
    carla_port: int = 2000
    
    # ZMQ ports
    telemetry_port: int = 5555
    metadata_port: int = 5556
    v2x_port: int = 5557  # For V2X messages
    
    # Scenario parameters
    duration_seconds: float = 60.0
    tick_rate: float = 20.0  # Hz
    
    # Vehicle A position (cannot see pedestrian)
    vehicle_a_start: Tuple[float, float, float] = (-30.0, 0.0, 0.5)
    vehicle_a_heading: float = 0.0  # degrees
    
    # Vehicle B position (CAN see pedestrian)
    vehicle_b_start: Tuple[float, float, float] = (30.0, 30.0, 0.5)
    vehicle_b_heading: float = -135.0  # degrees, facing towards pedestrian
    
    # Pedestrian position (hidden from A by building)
    pedestrian_start: Tuple[float, float, float] = (20.0, 5.0, 0.9)
    pedestrian_velocity: Tuple[float, float] = (-0.5, 0.0)  # Walking left
    
    # Sensor range
    vehicle_sensor_range: float = 50.0  # meters
    
    # Building position (blocks line of sight)
    building_center: Tuple[float, float] = (0.0, 5.0)
    building_size: Tuple[float, float] = (20.0, 10.0)  # width, depth


# =============================================================================
# OCCLUSION CHECKING
# =============================================================================

def line_intersects_building(
    start: Tuple[float, float],
    end: Tuple[float, float],
    building_center: Tuple[float, float],
    building_size: Tuple[float, float]
) -> bool:
    """
    Check if a line segment intersects with a building (axis-aligned box).
    
    Uses separating axis theorem for 2D AABB intersection.
    """
    # Building bounds
    half_w = building_size[0] / 2
    half_d = building_size[1] / 2
    min_x = building_center[0] - half_w
    max_x = building_center[0] + half_w
    min_y = building_center[1] - half_d
    max_y = building_center[1] + half_d
    
    # Line segment parameters
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    
    # Check if line endpoints are on different sides of each axis
    t_min = 0.0
    t_max = 1.0
    
    for axis in range(2):
        if axis == 0:
            d = dx
            p = start[0]
            box_min, box_max = min_x, max_x
        else:
            d = dy
            p = start[1]
            box_min, box_max = min_y, max_y
        
        if abs(d) < 1e-8:
            # Line parallel to axis
            if p < box_min or p > box_max:
                return False
        else:
            t1 = (box_min - p) / d
            t2 = (box_max - p) / d
            if t1 > t2:
                t1, t2 = t2, t1
            t_min = max(t_min, t1)
            t_max = min(t_max, t2)
            if t_min > t_max:
                return False
    
    return True


def can_see_target(
    vehicle_pos: Tuple[float, float],
    target_pos: Tuple[float, float],
    sensor_range: float,
    config: ScenarioConfig
) -> bool:
    """
    Check if a vehicle can see a target.
    
    Returns True if:
    1. Target is within sensor range
    2. Line of sight is NOT blocked by building
    """
    # Check distance
    dx = target_pos[0] - vehicle_pos[0]
    dy = target_pos[1] - vehicle_pos[1]
    distance = np.sqrt(dx*dx + dy*dy)
    
    if distance > sensor_range:
        return False
    
    # Check line of sight
    blocked = line_intersects_building(
        vehicle_pos, target_pos,
        config.building_center, config.building_size
    )
    
    return not blocked


# =============================================================================
# V2X MESSAGE
# =============================================================================

@dataclass
class V2XMessage:
    """Vehicle-to-Everything communication message."""
    sender_id: int
    target_id: int
    target_position: Tuple[float, float, float]
    target_velocity: Tuple[float, float, float]
    target_class: str
    confidence: float
    timestamp: float
    
    def to_json(self) -> str:
        return json.dumps({
            'sender_id': self.sender_id,
            'target_id': self.target_id,
            'target_position': list(self.target_position),
            'target_velocity': list(self.target_velocity),
            'target_class': self.target_class,
            'confidence': self.confidence,
            'timestamp': self.timestamp,
            'is_corner_sight': True,  # Key flag for "seeing around corners"
        })


# =============================================================================
# SCENARIO RUNNER
# =============================================================================

class SeeingAroundCornersScenario:
    """
    Runs the "seeing around corners" multi-vehicle collaborative perception demo.
    """
    
    def __init__(self, config: ScenarioConfig):
        self.config = config
        self.zmq_context = None
        self.telemetry_socket = None
        self.v2x_socket = None
        
        # Scenario state
        self.vehicle_a_id = None
        self.vehicle_b_id = None
        self.pedestrian_id = None
        
        # Stats
        self.corner_sight_events = 0
        self.total_v2x_messages = 0
        
    def setup_zmq(self):
        """Initialize ZMQ sockets."""
        if not ZMQ_AVAILABLE:
            raise RuntimeError("pyzmq required")
        
        self.zmq_context = zmq.Context()
        
        # Telemetry publisher (binary)
        self.telemetry_socket = self.zmq_context.socket(zmq.PUB)
        self.telemetry_socket.setsockopt(zmq.CONFLATE, 1)
        self.telemetry_socket.bind(f"tcp://127.0.0.1:{self.config.telemetry_port}")
        
        # V2X message publisher (JSON)
        self.v2x_socket = self.zmq_context.socket(zmq.PUB)
        self.v2x_socket.bind(f"tcp://127.0.0.1:{self.config.v2x_port}")
        
        print(f"📡 ZMQ sockets initialized:")
        print(f"   Telemetry: tcp://127.0.0.1:{self.config.telemetry_port}")
        print(f"   V2X:       tcp://127.0.0.1:{self.config.v2x_port}")
    
    def connect_carla(self):
        """Connect to CARLA and configure world."""
        print(f"\n🔌 Connecting to CARLA at {self.config.carla_host}:{self.config.carla_port}...")
        
        self.client = carla.Client(self.config.carla_host, self.config.carla_port)
        self.client.set_timeout(10.0)
        self.world = self.client.get_world()
        
        # Configure synchronous mode
        settings = self.world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 1.0 / self.config.tick_rate
        settings.no_rendering_mode = True
        self.world.apply_settings(settings)
        
        print(f"✅ Connected to: {self.world.get_map().name}")
    
    def spawn_actors(self):
        """Spawn vehicles and pedestrian for the scenario."""
        bp_library = self.world.get_blueprint_library()
        
        # Spawn Vehicle A
        vehicle_bp = bp_library.filter('vehicle.tesla.model3')[0]
        vehicle_bp.set_attribute('color', '0,200,255')  # Cyan
        
        transform_a = carla.Transform(
            carla.Location(*self.config.vehicle_a_start),
            carla.Rotation(yaw=self.config.vehicle_a_heading)
        )
        self.vehicle_a = self.world.spawn_actor(vehicle_bp, transform_a)
        self.vehicle_a_id = self.vehicle_a.id
        print(f"🚗 Spawned Vehicle A (id={self.vehicle_a_id}): Cannot see pedestrian")
        
        # Spawn Vehicle B
        vehicle_bp.set_attribute('color', '255,100,200')  # Pink
        transform_b = carla.Transform(
            carla.Location(*self.config.vehicle_b_start),
            carla.Rotation(yaw=self.config.vehicle_b_heading)
        )
        self.vehicle_b = self.world.spawn_actor(vehicle_bp, transform_b)
        self.vehicle_b_id = self.vehicle_b.id
        print(f"🚗 Spawned Vehicle B (id={self.vehicle_b_id}): CAN see pedestrian")
        
        # Spawn Pedestrian
        walker_bp = bp_library.filter('walker.pedestrian.0001')[0]
        transform_ped = carla.Transform(
            carla.Location(*self.config.pedestrian_start)
        )
        self.pedestrian = self.world.spawn_actor(walker_bp, transform_ped)
        self.pedestrian_id = self.pedestrian.id
        print(f"🚶 Spawned Pedestrian (id={self.pedestrian_id}): Hidden from Vehicle A")
        
        # Setup pedestrian walking
        walker_control = carla.WalkerControl()
        walker_control.speed = 1.0
        walker_control.direction = carla.Vector3D(
            self.config.pedestrian_velocity[0],
            self.config.pedestrian_velocity[1],
            0
        ).make_unit_vector()
        self.pedestrian.apply_control(walker_control)
    
    def run(self):
        """Main scenario loop."""
        print(f"\n🎬 Starting 'Seeing Around Corners' Scenario")
        print(f"   Duration: {self.config.duration_seconds}s")
        print(f"   Building blocks line-of-sight from Vehicle A to Pedestrian")
        print(f"   Vehicle B sees Pedestrian and shares via V2X")
        print()
        print("=" * 60)
        
        start_time = time.time()
        frame_id = 0
        sim_time = 0.0
        
        last_print_time = time.time()
        
        try:
            while time.time() - start_time < self.config.duration_seconds:
                # Tick simulation
                self.world.tick()
                sim_time = frame_id / self.config.tick_rate
                
                # Get actor positions
                va_pos = self.vehicle_a.get_location()
                vb_pos = self.vehicle_b.get_location()
                ped_pos = self.pedestrian.get_location()
                
                va_2d = (va_pos.x, va_pos.y)
                vb_2d = (vb_pos.x, vb_pos.y)
                ped_2d = (ped_pos.x, ped_pos.y)
                
                # Check visibility
                a_sees_ped = can_see_target(
                    va_2d, ped_2d, 
                    self.config.vehicle_sensor_range, 
                    self.config
                )
                b_sees_ped = can_see_target(
                    vb_2d, ped_2d,
                    self.config.vehicle_sensor_range,
                    self.config
                )
                
                # Core "seeing around corners" logic:
                # If B sees pedestrian but A doesn't, B shares via V2X
                if b_sees_ped and not a_sees_ped:
                    ped_vel = self.pedestrian.get_velocity()
                    
                    v2x_msg = V2XMessage(
                        sender_id=self.vehicle_b_id,
                        target_id=self.pedestrian_id,
                        target_position=(ped_pos.x, ped_pos.y, ped_pos.z),
                        target_velocity=(ped_vel.x, ped_vel.y, ped_vel.z),
                        target_class='pedestrian',
                        confidence=0.85,
                        timestamp=sim_time,
                    )
                    
                    # Publish V2X message
                    self.v2x_socket.send_string(v2x_msg.to_json())
                    self.total_v2x_messages += 1
                    
                    # Count unique corner sight events (once per second)
                    if frame_id % int(self.config.tick_rate) == 0:
                        self.corner_sight_events += 1
                
                frame_id += 1
                
                # Print progress
                if time.time() - last_print_time >= 2.0:
                    print(f"⏱️  t={sim_time:.1f}s | "
                          f"A sees ped: {'✓' if a_sees_ped else '✗'} | "
                          f"B sees ped: {'✓' if b_sees_ped else '✗'} | "
                          f"V2X msgs: {self.total_v2x_messages} | "
                          f"Corner events: {self.corner_sight_events}")
                    last_print_time = time.time()
        
        except KeyboardInterrupt:
            print("\n⚠️  Interrupted by user")
        
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Clean up spawned actors and reset world."""
        print("\n🧹 Cleaning up...")
        
        # Destroy actors
        if hasattr(self, 'vehicle_a') and self.vehicle_a:
            self.vehicle_a.destroy()
        if hasattr(self, 'vehicle_b') and self.vehicle_b:
            self.vehicle_b.destroy()
        if hasattr(self, 'pedestrian') and self.pedestrian:
            self.pedestrian.destroy()
        
        # Reset world settings
        if hasattr(self, 'world') and self.world:
            settings = self.world.get_settings()
            settings.synchronous_mode = False
            settings.no_rendering_mode = False
            self.world.apply_settings(settings)
        
        # Close ZMQ
        if self.telemetry_socket:
            self.telemetry_socket.close()
        if self.v2x_socket:
            self.v2x_socket.close()
        if self.zmq_context:
            self.zmq_context.term()
        
        # Print summary
        print()
        print("╔════════════════════════════════════════════════════════╗")
        print("║     'SEEING AROUND CORNERS' SCENARIO COMPLETE         ║")
        print("╠════════════════════════════════════════════════════════╣")
        print(f"║ Total V2X Messages Sent:        {self.total_v2x_messages:>8}              ║")
        print(f"║ Corner Sight Events:            {self.corner_sight_events:>8}              ║")
        print("╚════════════════════════════════════════════════════════╝")
        print()
        print("📖 What happened:")
        print("   • Vehicle A could NOT see the pedestrian (blocked by building)")
        print("   • Vehicle B COULD see the pedestrian")
        print("   • Vehicle B shared detection with Vehicle A via V2X")
        print("   • Vehicle A now 'knows' about the pedestrian it cannot see!")


def main():
    import argparse
    
    parser = argparse.ArgumentParser(
        description="'Seeing Around Corners' - Multi-Vehicle Collaborative Perception Demo"
    )
    parser.add_argument('--host', default='localhost', help='CARLA server host')
    parser.add_argument('--port', type=int, default=2000, help='CARLA server port')
    parser.add_argument('--duration', type=float, default=60.0, help='Duration in seconds')
    
    args = parser.parse_args()
    
    config = ScenarioConfig(
        carla_host=args.host,
        carla_port=args.port,
        duration_seconds=args.duration,
    )
    
    scenario = SeeingAroundCornersScenario(config)
    
    try:
        scenario.setup_zmq()
        scenario.connect_carla()
        scenario.spawn_actors()
        scenario.run()
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == '__main__':
    main()
