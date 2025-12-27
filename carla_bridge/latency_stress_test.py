#!/usr/bin/env python3
"""
Network Latency Stress Test - V2X Communication Delays

This scenario tests GodView's ability to handle out-of-sequence
measurements caused by network latency in Vehicle-to-Everything (V2X)
communication.

Key tests:
1. Random latency injection (0-500ms)
2. Packet reordering
3. Packet loss (5-20%)
4. Burst delays (simulating cellular handoffs)

The Augmented State EKF in GodView should handle these gracefully
via its "time travel" capability (interpolating past states).

Usage:
    python3 latency_stress_test.py --latency-min 50 --latency-max 300 --loss 10
"""

import carla
import numpy as np
import time
import json
import random
import argparse
import heapq
from collections import deque
from dataclasses import dataclass, field
from typing import List, Tuple, Optional

try:
    import zmq
except ImportError:
    print("Install pyzmq: pip install pyzmq")
    exit(1)


@dataclass
class LatencyConfig:
    """Configuration for network latency simulation."""
    min_latency_ms: float = 50.0
    max_latency_ms: float = 300.0
    packet_loss_rate: float = 0.10  # 10% loss
    burst_probability: float = 0.05  # 5% chance of burst
    burst_duration_ms: float = 500.0
    burst_latency_ms: float = 800.0


@dataclass(order=True)
class DelayedPacket:
    """Packet scheduled for future delivery."""
    delivery_time: float
    data: bytes = field(compare=False)
    topic: str = field(compare=False)


class LatencySimulator:
    """
    Simulates network latency, jitter, and packet loss.
    
    Uses a priority queue to delay and reorder packets,
    creating realistic V2X network conditions.
    """
    
    def __init__(self, config: LatencyConfig):
        self.config = config
        self.packet_queue: List[DelayedPacket] = []
        self.in_burst = False
        self.burst_end_time = 0.0
        
        # Stats
        self.packets_sent = 0
        self.packets_dropped = 0
        self.packets_delivered = 0
        self.total_delay = 0.0
    
    def maybe_start_burst(self, current_time: float) -> bool:
        """Randomly start a latency burst (cellular handoff simulation)."""
        if self.in_burst:
            if current_time > self.burst_end_time:
                self.in_burst = False
            return self.in_burst
        
        if random.random() < self.config.burst_probability:
            self.in_burst = True
            self.burst_end_time = current_time + self.config.burst_duration_ms / 1000.0
            return True
        
        return False
    
    def enqueue_packet(self, data: bytes, topic: str, current_time: float) -> bool:
        """
        Add packet to delay queue with simulated latency.
        Returns False if packet was dropped.
        """
        self.packets_sent += 1
        
        # Check for packet loss
        if random.random() < self.config.packet_loss_rate:
            self.packets_dropped += 1
            return False
        
        # Calculate latency
        if self.maybe_start_burst(current_time):
            latency_ms = self.config.burst_latency_ms + random.gauss(0, 50)
        else:
            latency_ms = random.uniform(
                self.config.min_latency_ms,
                self.config.max_latency_ms
            )
        
        delivery_time = current_time + latency_ms / 1000.0
        self.total_delay += latency_ms
        
        packet = DelayedPacket(delivery_time, data, topic)
        heapq.heappush(self.packet_queue, packet)
        
        return True
    
    def get_ready_packets(self, current_time: float) -> List[DelayedPacket]:
        """Get all packets ready for delivery (past their delay time)."""
        ready = []
        while self.packet_queue and self.packet_queue[0].delivery_time <= current_time:
            packet = heapq.heappop(self.packet_queue)
            ready.append(packet)
            self.packets_delivered += 1
        return ready
    
    def print_stats(self):
        """Print latency simulation statistics."""
        print("\n📊 Network Simulation Statistics:")
        print(f"   Packets Sent:     {self.packets_sent}")
        print(f"   Packets Dropped:  {self.packets_dropped} ({100*self.packets_dropped/max(1,self.packets_sent):.1f}%)")
        print(f"   Packets Delivered:{self.packets_delivered}")
        avg_delay = self.total_delay / max(1, self.packets_delivered)
        print(f"   Average Delay:    {avg_delay:.1f} ms")


class LatencyStressTest:
    """
    Main stress test scenario runner.
    
    Spawns vehicles and injects latency into telemetry stream
    to test GodView's handling of out-of-sequence measurements.
    """
    
    def __init__(
        self,
        carla_host: str = 'localhost',
        carla_port: int = 2000,
        latency_config: Optional[LatencyConfig] = None
    ):
        self.latency_config = latency_config or LatencyConfig()
        self.latency_sim = LatencySimulator(self.latency_config)
        
        # Connect to CARLA
        print(f"🔌 Connecting to CARLA at {carla_host}:{carla_port}...")
        self.client = carla.Client(carla_host, carla_port)
        self.client.set_timeout(10.0)
        self.world = self.client.get_world()
        
        # Configure sync mode
        settings = self.world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05  # 20 Hz
        settings.no_rendering_mode = True
        self.world.apply_settings(settings)
        
        print(f"✅ Connected to: {self.world.get_map().name}")
        
        # ZMQ
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
        self.socket.bind("tcp://127.0.0.1:5555")
        print("📡 ZMQ Publisher bound to tcp://127.0.0.1:5555")
    
    def spawn_vehicles(self, count: int = 10):
        """Spawn test vehicles."""
        # Clean up existing
        for actor in self.world.get_actors().filter('vehicle.*'):
            actor.destroy()
        self.world.tick()
        
        bp_library = self.world.get_blueprint_library()
        spawn_points = self.world.get_map().get_spawn_points()
        
        spawned = 0
        for i, sp in enumerate(spawn_points[:count*2]):
            bp = random.choice(list(bp_library.filter('vehicle.*')))
            try:
                v = self.world.spawn_actor(bp, sp)
                v.set_autopilot(True)
                spawned += 1
                if spawned >= count:
                    break
            except:
                pass
        
        self.world.tick()
        print(f"🚗 Spawned {spawned} vehicles")
        return spawned
    
    def run(self, duration: float = 60.0):
        """Run the stress test."""
        print(f"\n🏁 Starting Latency Stress Test")
        print(f"   Duration: {duration}s")
        print(f"   Latency: {self.latency_config.min_latency_ms}-{self.latency_config.max_latency_ms}ms")
        print(f"   Packet Loss: {100*self.latency_config.packet_loss_rate:.0f}%")
        print(f"   Burst Probability: {100*self.latency_config.burst_probability:.0f}%")
        print()
        print("=" * 60)
        
        start_time = time.time()
        frame_id = 0
        last_print = start_time
        
        try:
            while time.time() - start_time < duration:
                # Tick simulation
                self.world.tick()
                current_time = time.time() - start_time
                
                # Get snapshot
                snapshot = self.world.get_snapshot()
                sim_time = snapshot.timestamp.elapsed_seconds
                
                # Build telemetry packet (simplified)
                actors = self.world.get_actors().filter('vehicle.*')
                packet = self._build_packet(frame_id, sim_time, actors, snapshot)
                
                # Inject latency
                self.latency_sim.enqueue_packet(packet, "telemetry", current_time)
                
                # Deliver any ready packets
                for delayed in self.latency_sim.get_ready_packets(current_time):
                    self.socket.send(delayed.data, zmq.NOBLOCK)
                
                frame_id += 1
                
                # Print progress
                if time.time() - last_print >= 2.0:
                    print(f"⏱️  t={current_time:.1f}s | Frame {frame_id} | "
                          f"Queue: {len(self.latency_sim.packet_queue)} | "
                          f"Dropped: {self.latency_sim.packets_dropped}")
                    last_print = time.time()
        
        except KeyboardInterrupt:
            print("\n⚠️  Interrupted")
        
        finally:
            self._cleanup()
    
    def _build_packet(self, frame_id, sim_time, actors, snapshot) -> bytes:
        """Build binary telemetry packet."""
        import struct
        
        # Header
        header = struct.pack('<QdII', frame_id, sim_time, len(actors), 0)
        
        # Actor data
        actor_data = bytearray()
        for actor in actors:
            actor_snap = snapshot.find(actor.id)
            if not actor_snap:
                continue
            
            t = actor_snap.get_transform()
            v = actor_snap.get_velocity()
            
            # Pack actor: id(u32) + pos(3xf32) + rot(3xf32) + vel(3xf32)
            actor_data.extend(struct.pack('<I', actor.id))
            actor_data.extend(struct.pack('<fff', t.location.x, t.location.y, t.location.z))
            actor_data.extend(struct.pack('<fff', t.rotation.pitch, t.rotation.yaw, t.rotation.roll))
            actor_data.extend(struct.pack('<fff', v.x, v.y, v.z))
        
        return header + bytes(actor_data)
    
    def _cleanup(self):
        """Clean up resources."""
        print("\n🧹 Cleaning up...")
        
        # Print stats
        self.latency_sim.print_stats()
        
        # Reset CARLA
        settings = self.world.get_settings()
        settings.synchronous_mode = False
        settings.no_rendering_mode = False
        self.world.apply_settings(settings)
        
        self.socket.close()
        self.context.term()
        
        print("✅ Complete")


def main():
    parser = argparse.ArgumentParser(description='V2X Network Latency Stress Test')
    parser.add_argument('--host', default='localhost')
    parser.add_argument('--port', type=int, default=2000)
    parser.add_argument('--duration', type=float, default=60.0)
    parser.add_argument('--vehicles', type=int, default=10)
    parser.add_argument('--latency-min', type=float, default=50.0, help='Min latency (ms)')
    parser.add_argument('--latency-max', type=float, default=300.0, help='Max latency (ms)')
    parser.add_argument('--loss', type=float, default=10.0, help='Packet loss rate (%)')
    parser.add_argument('--burst', type=float, default=5.0, help='Burst probability (%)')
    
    args = parser.parse_args()
    
    config = LatencyConfig(
        min_latency_ms=args.latency_min,
        max_latency_ms=args.latency_max,
        packet_loss_rate=args.loss / 100.0,
        burst_probability=args.burst / 100.0,
    )
    
    test = LatencyStressTest(
        carla_host=args.host,
        carla_port=args.port,
        latency_config=config,
    )
    
    test.spawn_vehicles(args.vehicles)
    test.run(args.duration)


if __name__ == '__main__':
    main()
