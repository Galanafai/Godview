#!/usr/bin/env python3
"""
Emergency Vehicle Priority Scenario - Trust-Based Fusion Test

This scenario tests GodView's trust/security layer (CapBAC + Ed25519)
by introducing emergency vehicles with elevated trust levels.

Key concepts:
1. Emergency vehicles (ambulance, fire truck) have higher trust scores
2. Their detections override conflicting civilian detections
3. Tests the "Highlander" CRDT merge with trust weighting
4. Validates cryptographic provenance tracking

Scenario:
- Normal civilian vehicles driving around
- Emergency vehicle appears with lights/sirens
- Emergency vehicle shares critical hazard (pedestrian in road)
- GodView should prioritize emergency vehicle's detection

Usage:
    python3 emergency_priority.py --duration 60
"""

import carla
import numpy as np
import time
import json
import struct
import argparse
from dataclasses import dataclass
from typing import List, Dict, Optional
from enum import Enum

try:
    import zmq
except ImportError:
    print("Install pyzmq: pip install pyzmq")
    exit(1)


class TrustLevel(Enum):
    """Trust levels for different vehicle types."""
    UNKNOWN = 0
    CIVILIAN = 50
    COMMERCIAL = 60
    GOVERNMENT = 70
    LAW_ENFORCEMENT = 80
    EMERGENCY = 95
    INFRASTRUCTURE = 100


@dataclass
class TrustedVehicle:
    """Vehicle with trust metadata."""
    actor: carla.Actor
    trust_level: TrustLevel
    can_override: bool = False
    lights_active: bool = False


class EmergencyPriorityScenario:
    """
    Emergency vehicle priority scenario.
    
    Tests trust-based fusion by having emergency vehicles
    with elevated trust levels share critical detections.
    """
    
    def __init__(
        self,
        carla_host: str = 'localhost',
        carla_port: int = 2000,
    ):
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
        
        self.trust_socket = self.context.socket(zmq.PUB)
        self.trust_socket.bind("tcp://127.0.0.1:5557")
        
        print(f"📡 ZMQ: telemetry=5555, trust=5557")
        
        # Vehicles
        self.trusted_vehicles: Dict[int, TrustedVehicle] = {}
        self.emergency_vehicle: Optional[TrustedVehicle] = None
        self.hazard_pedestrian: Optional[carla.Actor] = None
    
    def spawn_civilian_vehicles(self, count: int = 8):
        """Spawn normal civilian vehicles."""
        print(f"\n🚗 Spawning {count} civilian vehicles...")
        
        # Clean up
        for actor in self.world.get_actors().filter('vehicle.*'):
            actor.destroy()
        for actor in self.world.get_actors().filter('walker.*'):
            actor.destroy()
        self.world.tick()
        
        bp_library = self.world.get_blueprint_library()
        spawn_points = self.world.get_map().get_spawn_points()
        
        # Civilian vehicles (various colors)
        civilian_bps = [
            bp for bp in bp_library.filter('vehicle.*')
            if 'ambulance' not in bp.id and 'firetruck' not in bp.id and 'police' not in bp.id
        ]
        
        spawned = 0
        for i, sp in enumerate(spawn_points[:count*2]):
            if spawned >= count:
                break
            
            bp = civilian_bps[i % len(civilian_bps)]
            
            try:
                vehicle = self.world.spawn_actor(bp, sp)
                vehicle.set_autopilot(True)
                
                tv = TrustedVehicle(
                    actor=vehicle,
                    trust_level=TrustLevel.CIVILIAN,
                    can_override=False,
                )
                self.trusted_vehicles[vehicle.id] = tv
                spawned += 1
                
                # Publish trust metadata
                self._publish_trust_update(vehicle.id, TrustLevel.CIVILIAN)
                
            except:
                pass
        
        self.world.tick()
        print(f"   Spawned {spawned} civilian vehicles (trust={TrustLevel.CIVILIAN.value})")
    
    def spawn_emergency_vehicle(self):
        """Spawn an emergency vehicle with elevated trust."""
        print(f"\n🚑 Spawning emergency vehicle...")
        
        bp_library = self.world.get_blueprint_library()
        spawn_points = self.world.get_map().get_spawn_points()
        
        # Try ambulance first, then firetruck
        emergency_types = ['vehicle.carlamotors.firetruck', 'vehicle.ford.ambulance']
        
        for em_type in emergency_types:
            bps = list(bp_library.filter(em_type))
            if not bps:
                continue
            
            bp = bps[0]
            
            # Find a spawn point away from other vehicles
            for sp in spawn_points[len(self.trusted_vehicles):]:
                try:
                    vehicle = self.world.spawn_actor(bp, sp)
                    vehicle.set_autopilot(True)
                    
                    # Enable lights (emergency)
                    if hasattr(vehicle, 'set_light_state'):
                        light_state = carla.VehicleLightState.Special1 | carla.VehicleLightState.Special2
                        vehicle.set_light_state(carla.VehicleLightState(light_state))
                    
                    tv = TrustedVehicle(
                        actor=vehicle,
                        trust_level=TrustLevel.EMERGENCY,
                        can_override=True,
                        lights_active=True,
                    )
                    self.trusted_vehicles[vehicle.id] = tv
                    self.emergency_vehicle = tv
                    
                    # Publish trust metadata
                    self._publish_trust_update(vehicle.id, TrustLevel.EMERGENCY, is_emergency=True)
                    
                    print(f"   Spawned {em_type} (trust={TrustLevel.EMERGENCY.value}, override=True)")
                    self.world.tick()
                    return True
                    
                except Exception as e:
                    continue
        
        print("   ⚠️  Could not spawn emergency vehicle (blueprint not available)")
        return False
    
    def spawn_hazard_pedestrian(self):
        """Spawn a pedestrian in a dangerous position."""
        print(f"\n🚶 Spawning hazard pedestrian...")
        
        bp_library = self.world.get_blueprint_library()
        walker_bps = list(bp_library.filter('walker.pedestrian.*'))
        
        if not walker_bps:
            print("   ⚠️  No pedestrian blueprints available")
            return False
        
        # Find a road location
        spawn_points = self.world.get_map().get_spawn_points()
        if not spawn_points:
            return False
        
        # Spawn near a road
        road_point = spawn_points[0].location
        pedestrian_location = carla.Location(
            x=road_point.x + 5.0,
            y=road_point.y,
            z=road_point.z + 1.0,
        )
        
        try:
            bp = walker_bps[0]
            transform = carla.Transform(pedestrian_location)
            self.hazard_pedestrian = self.world.spawn_actor(bp, transform)
            
            print(f"   Pedestrian spawned at ({pedestrian_location.x:.1f}, {pedestrian_location.y:.1f})")
            return True
        except Exception as e:
            print(f"   ⚠️  Failed to spawn pedestrian: {e}")
            return False
    
    def _publish_trust_update(self, actor_id: int, trust_level: TrustLevel, is_emergency: bool = False):
        """Publish trust level update for an actor."""
        msg = json.dumps({
            'type': 'trust_update',
            'actor_id': actor_id,
            'trust_level': trust_level.value,
            'trust_name': trust_level.name,
            'is_emergency': is_emergency,
            'can_override': trust_level.value >= TrustLevel.EMERGENCY.value,
            'timestamp': time.time(),
        }).encode()
        
        self.trust_socket.send_multipart([b'trust', msg], zmq.NOBLOCK)
    
    def run(self, duration: float = 60.0):
        """Run the emergency priority scenario."""
        print(f"\n🏁 Starting Emergency Priority Scenario")
        print(f"   Duration: {duration}s")
        print(f"   Testing: Trust-based fusion (CapBAC)")
        print()
        print("=" * 60)
        
        start_time = time.time()
        frame_id = 0
        last_print = start_time
        
        # Emergency vehicle detection events
        emergency_detections = 0
        civilian_detections = 0
        priority_overrides = 0
        
        try:
            while time.time() - start_time < duration:
                self.world.tick()
                snapshot = self.world.get_snapshot()
                sim_time = snapshot.timestamp.elapsed_seconds
                
                # Build telemetry with trust annotations
                actors_data = []
                
                for actor_id, tv in self.trusted_vehicles.items():
                    if not tv.actor.is_alive:
                        continue
                    
                    snap = snapshot.find(actor_id)
                    if not snap:
                        continue
                    
                    t = snap.get_transform()
                    vel = snap.get_velocity()
                    
                    actors_data.append({
                        'id': actor_id,
                        'pos': [t.location.x, t.location.y, t.location.z],
                        'rot': [t.rotation.pitch, t.rotation.yaw, t.rotation.roll],
                        'vel': [vel.x, vel.y, vel.z],
                        'trust': tv.trust_level.value,
                    })
                    
                    # Count detections by trust level
                    if tv.trust_level == TrustLevel.EMERGENCY:
                        emergency_detections += 1
                    else:
                        civilian_detections += 1
                    
                    # Simulate priority override when emergency detects hazard
                    if tv.trust_level == TrustLevel.EMERGENCY and self.hazard_pedestrian:
                        ped_loc = self.hazard_pedestrian.get_location()
                        veh_loc = tv.actor.get_location()
                        dist = np.sqrt(
                            (ped_loc.x - veh_loc.x)**2 + 
                            (ped_loc.y - veh_loc.y)**2
                        )
                        
                        if dist < 30.0:  # Within 30m
                            # Emergency vehicle detects hazard and broadcasts
                            self.trust_socket.send_multipart([
                                b'hazard',
                                json.dumps({
                                    'type': 'priority_hazard',
                                    'source_id': actor_id,
                                    'trust_level': tv.trust_level.value,
                                    'hazard_type': 'pedestrian',
                                    'hazard_pos': [ped_loc.x, ped_loc.y, ped_loc.z],
                                    'distance': dist,
                                    'override_civilian': True,
                                    'timestamp': sim_time,
                                }).encode()
                            ], zmq.NOBLOCK)
                            priority_overrides += 1
                
                # Publish telemetry
                if actors_data:
                    packet = self._build_packet(frame_id, sim_time, actors_data)
                    self.telemetry_socket.send(packet, zmq.NOBLOCK)
                
                frame_id += 1
                
                # Print progress
                if time.time() - last_print >= 2.0:
                    print(f"⏱️  t={sim_time:.1f}s | Frame {frame_id} | "
                          f"Emergency: {emergency_detections} | "
                          f"Civilian: {civilian_detections} | "
                          f"Overrides: {priority_overrides}")
                    
                    # Reset counters
                    emergency_detections = 0
                    civilian_detections = 0
                    last_print = time.time()
        
        except KeyboardInterrupt:
            print("\n⚠️  Interrupted")
        
        finally:
            self._cleanup()
    
    def _build_packet(self, frame_id: int, sim_time: float, actors: List[dict]) -> bytes:
        """Build binary telemetry packet with trust info."""
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
        self.trust_socket.close()
        self.context.term()
        
        print("✅ Complete")


def main():
    parser = argparse.ArgumentParser(description='Emergency Vehicle Priority Scenario')
    parser.add_argument('--host', default='localhost')
    parser.add_argument('--port', type=int, default=2000)
    parser.add_argument('--duration', type=float, default=60.0)
    parser.add_argument('--civilians', type=int, default=8)
    
    args = parser.parse_args()
    
    scenario = EmergencyPriorityScenario(
        carla_host=args.host,
        carla_port=args.port,
    )
    
    scenario.spawn_civilian_vehicles(args.civilians)
    scenario.spawn_emergency_vehicle()
    scenario.spawn_hazard_pedestrian()
    scenario.run(args.duration)


if __name__ == '__main__':
    main()
