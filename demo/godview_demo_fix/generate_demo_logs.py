#!/usr/bin/env python3
"""
GodView Demo Fix - Enhanced Log Generator
==========================================
Generates rich NDJSON logs with:
- 20 agents + 1 drone
- Richer object set (cars, pedestrians, cyclists, cones)
- Occluder (building) blocking visibility
- OOSM delays
- Spoof packets
- Causality events for packet arrival

Output:
  out/packets.ndjson
  out/world_state.ndjson
  out/events.ndjson
"""

import argparse
import json
import math
import os
import random
from dataclasses import dataclass, field
from typing import List, Dict, Optional, Set, Tuple
from pathlib import Path

# =============================================================================
# CONFIGURATION
# =============================================================================

NUM_CARS = 20
NUM_DRONES = 1

# Scene geometry
INTERSECTION_CENTER = (0, 0)
INTERSECTION_RADIUS = 25.0
ROAD_WIDTH = 8.0

# Occluder (building) - blocks visibility
OCCLUDER = {
    "id": "building_01",
    "x": 8, "y": 5,
    "width": 6, "height": 8,
}

# Ground truth objects (cars, pedestrians, cyclists, cones)
GT_OBJECTS = [
    {"id": "ped_01", "class": "pedestrian", "x": 12, "y": 8, "vx": -0.5, "vy": 0},
    {"id": "ped_02", "class": "pedestrian", "x": -5, "y": 12, "vx": 0.3, "vy": -0.2},
    {"id": "cyclist_01", "class": "cyclist", "x": -15, "y": -8, "vx": 2, "vy": 0.5},
    {"id": "car_target_01", "class": "car", "x": -10, "y": 5, "vx": 1.5, "vy": 0},
    {"id": "car_target_02", "class": "car", "x": 5, "y": -12, "vx": 0, "vy": 1.2},
    {"id": "truck_01", "class": "truck", "x": 15, "y": -5, "vx": -0.8, "vy": 0},
    {"id": "cone_01", "class": "cone", "x": 3, "y": 3, "vx": 0, "vy": 0},
    {"id": "cone_02", "class": "cone", "x": -3, "y": -3, "vx": 0, "vy": 0},
    {"id": "barrier_01", "class": "barrier", "x": 0, "y": 8, "vx": 0, "vy": 0},
    {"id": "car_overlap_01", "class": "car", "x": 0, "y": 0, "vx": 0.5, "vy": 0.5},
]

# Phases (in seconds)
PHASE_HOOK = (0, 5)
PHASE_BEFORE = (5, 28)
PHASE_AFTER = (28, 55)
PHASE_MONTAGE = (55, 75)
PHASE_CLOSE = (75, 85)

# Beat definitions with spotlight agents
BEATS = {
    "hook": {"start": 0, "end": 5, "spotlight": ["agent_00", "agent_01", "drone_00"]},
    "occlusion": {"start": 5, "end": 14, "spotlight": ["agent_00", "agent_01"], "split_screen": (10, 14)},
    "ghosts": {"start": 14, "end": 21, "spotlight": ["agent_02", "agent_03"], "split_screen": (17, 21)},
    "spoof": {"start": 21, "end": 28, "spotlight": ["agent_04", "unknown_x"], "split_screen": (24, 28)},
    "remote_obs": {"start": 28, "end": 38, "spotlight": ["agent_00", "agent_01", "drone_00"]},
    "merge": {"start": 38, "end": 48, "spotlight": ["agent_02", "agent_03", "drone_00"]},
    "trust_reject": {"start": 48, "end": 55, "spotlight": ["agent_04"]},
    "oosm": {"start": 55, "end": 62, "spotlight": ["agent_07"]},
    "space": {"start": 62, "end": 68, "spotlight": ["agent_03", "drone_00"]},
    "bandwidth": {"start": 68, "end": 75, "spotlight": ["agent_00"]},
    "close": {"start": 75, "end": 85, "spotlight": ["agent_00", "agent_01", "agent_02"]},
}

# =============================================================================
# DATA CLASSES
# =============================================================================

@dataclass
class Vec3:
    x: float
    y: float
    z: float = 0.0

    def to_dict(self):
        return {"x": self.x, "y": self.y, "z": self.z}


@dataclass
class DetectedObject:
    local_object_id: str
    obj_class: str
    x: float
    y: float
    z: float
    yaw: float
    covariance: List[float]
    vx: float = 0
    vy: float = 0

    def to_dict(self):
        return {
            "local_object_id": self.local_object_id,
            "class": self.obj_class,
            "pose": {"position": {"x": self.x, "y": self.y, "z": self.z}, "yaw": self.yaw},
            "covariance": self.covariance,
            "velocity": {"x": self.vx, "y": self.vy, "z": 0},
        }


@dataclass
class DetectionPacket:
    frame: int
    timestamp_ns: int
    agent_id: str
    delivery_frame: int
    signature_valid: bool
    objects: List[DetectedObject]

    def to_ndjson(self):
        return json.dumps({
            "packet_type": "DETECTION",
            "frame": self.frame,
            "timestamp_ns": self.timestamp_ns,
            "agent_id": self.agent_id,
            "delivery_frame": self.delivery_frame,
            "signature_valid": self.signature_valid,
            "objects": [o.to_dict() for o in self.objects],
        })


@dataclass
class CanonicalObject:
    canonical_object_id: str
    obj_class: str
    x: float
    y: float
    z: float
    yaw: float
    covariance: List[float]
    source_agents: List[str]
    confidence: float

    def to_dict(self):
        return {
            "canonical_object_id": self.canonical_object_id,
            "class": self.obj_class,
            "pose": {"position": {"x": self.x, "y": self.y, "z": self.z}, "yaw": self.yaw},
            "covariance": self.covariance,
            "source_agents": self.source_agents,
            "confidence": self.confidence,
        }


@dataclass
class CanonicalState:
    frame: int
    timestamp_ns: int
    objects: List[CanonicalObject]

    def to_ndjson(self):
        return json.dumps({
            "packet_type": "CANONICAL_STATE",
            "frame": self.frame,
            "timestamp_ns": self.timestamp_ns,
            "objects": [o.to_dict() for o in self.objects],
        })


@dataclass
class Event:
    frame: int
    event_type: str
    payload: Dict

    def to_ndjson(self):
        return json.dumps({
            "packet_type": "EVENT",
            "frame": self.frame,
            "event_type": self.event_type,
            "payload": self.payload,
        })


@dataclass
class Agent:
    agent_id: str
    x: float
    y: float
    z: float
    yaw: float
    is_drone: bool = False
    is_spoof: bool = False


# =============================================================================
# WORLD SIMULATION
# =============================================================================

class WorldSimulator:
    def __init__(self, seed: int, fps: int, duration_s: int, num_agents: int):
        self.seed = seed
        self.fps = fps
        self.duration_s = duration_s
        self.total_frames = fps * duration_s
        self.num_agents = num_agents

        random.seed(seed)

        # Spoof config (needed before _init_agents)
        self.spoof_agent_id = "unknown_x"
        self.spoof_object_id = "spoof_bus_01"

        # Initialize agents
        self.agents: List[Agent] = []
        self._init_agents()

        # Ground truth objects (copy so we can mutate positions)
        self.gt_objects = [dict(o) for o in GT_OBJECTS]

        # OOSM config
        self.oosm_agent = "agent_07"
        self.oosm_delay_frames = 12
        self.oosm_active_range = (int(55 * fps), int(62 * fps))

        # Output buffers
        self.packets: List[DetectionPacket] = []
        self.world_states: List[CanonicalState] = []
        self.events: List[Event] = []

    def _init_agents(self):
        # Car agents in a ring around intersection
        for i in range(self.num_agents):
            angle = (2 * math.pi * i) / self.num_agents
            radius = INTERSECTION_RADIUS * 0.8
            x = radius * math.cos(angle)
            y = radius * math.sin(angle)
            yaw = angle + math.pi

            self.agents.append(Agent(
                agent_id=f"agent_{i:02d}",
                x=x, y=y, z=0,
                yaw=yaw,
                is_drone=False,
            ))

        # Drone at center, elevated
        self.agents.append(Agent(
            agent_id="drone_00",
            x=0, y=0, z=15.0,
            yaw=0,
            is_drone=True,
        ))

        # Spoof agent (not in main list)
        self.spoof_agent = Agent(
            agent_id=self.spoof_agent_id,
            x=-30, y=-30, z=0,
            yaw=0,
            is_spoof=True,
        )

    def get_phase(self, frame: int) -> str:
        t = frame / self.fps
        if PHASE_HOOK[0] <= t < PHASE_HOOK[1]:
            return "hook"
        elif PHASE_BEFORE[0] <= t < PHASE_BEFORE[1]:
            return "before"
        elif PHASE_AFTER[0] <= t < PHASE_AFTER[1]:
            return "after"
        elif PHASE_MONTAGE[0] <= t < PHASE_MONTAGE[1]:
            return "montage"
        else:
            return "close"

    def get_beat(self, frame: int) -> str:
        t = frame / self.fps
        for beat_name, beat_info in BEATS.items():
            if beat_info["start"] <= t < beat_info["end"]:
                return beat_name
        return "general"

    def _update_positions(self, frame: int):
        dt = 1.0 / self.fps

        # Update agents (circular motion)
        for agent in self.agents:
            if not agent.is_drone:
                angle_speed = 0.015
                current_angle = math.atan2(agent.y, agent.x)
                new_angle = current_angle + angle_speed
                radius = math.sqrt(agent.x**2 + agent.y**2)
                agent.x = radius * math.cos(new_angle)
                agent.y = radius * math.sin(new_angle)
                agent.yaw = new_angle + math.pi
            else:
                # Drone gentle drift
                agent.x = 2 * math.sin(frame * 0.02)
                agent.y = 2 * math.cos(frame * 0.02)

        # Update ground truth objects
        for obj in self.gt_objects:
            obj["x"] += obj.get("vx", 0) * dt
            obj["y"] += obj.get("vy", 0) * dt
            # Wrap around if too far
            if abs(obj["x"]) > 40:
                obj["vx"] *= -1
            if abs(obj["y"]) > 40:
                obj["vy"] *= -1

    def _is_occluded(self, agent: Agent, obj_x: float, obj_y: float) -> bool:
        """Check if object is occluded by building from agent's view."""
        if agent.is_drone:
            return False  # Drone sees from above

        # Simple box occlusion check
        occ = OCCLUDER
        bx, by = occ["x"], occ["y"]
        bw, bh = occ["width"], occ["height"]

        # Check if line from agent to object passes through occluder
        ax, ay = agent.x, agent.y

        # Parametric line: P = A + t*(O - A)
        dx, dy = obj_x - ax, obj_y - ay
        dist = math.sqrt(dx*dx + dy*dy)
        if dist < 0.1:
            return False

        # Check intersection with box
        for t in [i * 0.1 for i in range(1, 10)]:
            px = ax + t * dx
            py = ay + t * dy
            if (bx - bw/2 <= px <= bx + bw/2) and (by - bh/2 <= py <= by + bh/2):
                return True
        return False

    def _agent_can_see(self, agent: Agent, obj: Dict, phase: str, beat: str) -> bool:
        """Determine if agent can see the object."""
        obj_x, obj_y = obj["x"], obj["y"]

        # During occlusion beat in BEFORE phase, agent_01 cannot see ped_01
        if beat == "occlusion" and phase == "before":
            if agent.agent_id == "agent_01" and obj["id"] == "ped_01":
                if self._is_occluded(agent, obj_x, obj_y):
                    return False

        # Distance check (agents can see ~30m)
        dist = math.sqrt((agent.x - obj_x)**2 + (agent.y - obj_y)**2)
        max_range = 35 if agent.is_drone else 25
        if dist > max_range:
            return False

        return True

    def _generate_detection(self, agent: Agent, obj: Dict, local_id: str) -> DetectedObject:
        noise_x = random.gauss(0, 0.2)
        noise_y = random.gauss(0, 0.2)
        noise_yaw = random.gauss(0, 0.03)

        return DetectedObject(
            local_object_id=local_id,
            obj_class=obj["class"],
            x=obj["x"] + noise_x,
            y=obj["y"] + noise_y,
            z=obj.get("z", 0),
            yaw=noise_yaw,
            covariance=[0.4, 0.0, 0.0, 0.4],
            vx=obj.get("vx", 0),
            vy=obj.get("vy", 0),
        )

    def _generate_spoof_packet(self, frame: int) -> DetectionPacket:
        timestamp_ns = int((frame / self.fps) * 1e9)
        fake_bus = DetectedObject(
            local_object_id=self.spoof_object_id,
            obj_class="bus",
            x=8, y=-8, z=0,
            yaw=0,
            covariance=[1.0, 0.0, 0.0, 1.0],
        )
        return DetectionPacket(
            frame=frame,
            timestamp_ns=timestamp_ns,
            agent_id=self.spoof_agent_id,
            delivery_frame=frame + 2,
            signature_valid=False,
            objects=[fake_bus],
        )

    def simulate_frame(self, frame: int):
        phase = self.get_phase(frame)
        beat = self.get_beat(frame)
        timestamp_ns = int((frame / self.fps) * 1e9)

        self._update_positions(frame)

        frame_packets: List[DetectionPacket] = []

        # Generate per-agent detections
        for agent in self.agents:
            objects_seen: List[DetectedObject] = []

            for obj in self.gt_objects:
                if self._agent_can_see(agent, obj, phase, beat):
                    if random.random() < 0.85:  # 85% detection rate
                        local_id = f"{agent.agent_id}_{obj['id']}"
                        det = self._generate_detection(agent, obj, local_id)
                        objects_seen.append(det)

            # OOSM delay for agent_07
            delivery_frame = frame
            if agent.agent_id == self.oosm_agent:
                if self.oosm_active_range[0] <= frame < self.oosm_active_range[1]:
                    delivery_frame = frame + self.oosm_delay_frames

            packet = DetectionPacket(
                frame=frame,
                timestamp_ns=timestamp_ns,
                agent_id=agent.agent_id,
                delivery_frame=delivery_frame,
                signature_valid=True,
                objects=objects_seen,
            )
            frame_packets.append(packet)

        # Spoof packets during spoof beat
        if beat in ["spoof", "trust_reject"]:
            spoof_pkt = self._generate_spoof_packet(frame)
            frame_packets.append(spoof_pkt)

        self.packets.extend(frame_packets)

        # Generate canonical state
        canonical_objects = self._fuse_to_canonical(frame_packets, frame, phase)
        self.world_states.append(CanonicalState(
            frame=frame,
            timestamp_ns=timestamp_ns,
            objects=canonical_objects,
        ))

        # Generate events
        self._generate_events(frame, beat, phase)

    def _fuse_to_canonical(self, packets: List[DetectionPacket], frame: int, phase: str) -> List[CanonicalObject]:
        obj_observations: Dict[str, List[Tuple[str, DetectedObject]]] = {}

        for pkt in packets:
            # Skip invalid signatures in AFTER phase
            if phase in ["after", "montage", "close"] and not pkt.signature_valid:
                continue

            for det in pkt.objects:
                parts = det.local_object_id.split("_", 2)
                if len(parts) >= 3:
                    gt_id = "_".join(parts[2:])
                else:
                    gt_id = det.local_object_id

                if gt_id not in obj_observations:
                    obj_observations[gt_id] = []
                obj_observations[gt_id].append((pkt.agent_id, det))

        canonical_objects: List[CanonicalObject] = []
        for gt_id, observations in obj_observations.items():
            if not observations:
                continue

            avg_x = sum(o[1].x for o in observations) / len(observations)
            avg_y = sum(o[1].y for o in observations) / len(observations)
            avg_z = sum(o[1].z for o in observations) / len(observations)
            avg_yaw = sum(o[1].yaw for o in observations) / len(observations)

            fused_cov = 0.4 / math.sqrt(len(observations))
            source_agents = list(set(o[0] for o in observations))

            canonical_objects.append(CanonicalObject(
                canonical_object_id=f"canonical_{gt_id}",
                obj_class=observations[0][1].obj_class,
                x=avg_x, y=avg_y, z=avg_z,
                yaw=avg_yaw,
                covariance=[fused_cov, 0, 0, fused_cov],
                source_agents=source_agents,
                confidence=min(0.99, 0.5 + 0.1 * len(observations)),
            ))

        return canonical_objects

    def _generate_events(self, frame: int, beat: str, phase: str):
        # MERGE event
        if beat == "merge" and frame == int(42 * self.fps):
            self.events.append(Event(
                frame=frame,
                event_type="MERGE",
                payload={
                    "from_ids": ["agent_02_car_target_01", "agent_03_car_target_01"],
                    "to_id": "canonical_car_target_01",
                    "reason": "highlander_min_uuid",
                }
            ))

        # TRUST_REJECT event
        if beat == "trust_reject" and frame == int(52 * self.fps):
            self.events.append(Event(
                frame=frame,
                event_type="TRUST_REJECT",
                payload={
                    "agent_id": self.spoof_agent_id,
                    "object_id": self.spoof_object_id,
                    "reason": "invalid_signature",
                }
            ))

        # OOSM event
        if beat == "oosm" and frame == int(59 * self.fps):
            self.events.append(Event(
                frame=frame,
                event_type="OOSM_CORRECTED",
                payload={
                    "agent_id": self.oosm_agent,
                    "delayed_frames": self.oosm_delay_frames,
                }
            ))

        # SPACE_SEPARATION event
        if beat == "space" and frame == int(65 * self.fps):
            self.events.append(Event(
                frame=frame,
                event_type="SPACE_SEPARATION",
                payload={
                    "drone_id": "drone_00",
                    "car_id": "agent_03",
                    "delta_z": 15.0,
                }
            ))

        # PACKET_ARRIVAL events (causality)
        if beat == "remote_obs" and frame == int(32 * self.fps):
            self.events.append(Event(
                frame=frame,
                event_type="PACKET_ARRIVAL",
                payload={
                    "from_agent": "drone_00",
                    "to_agent": "agent_01",
                    "object_id": "ped_01",
                    "causes": "remote_observation",
                }
            ))

    def run(self):
        print(f"Simulating {self.total_frames} frames ({self.duration_s}s at {self.fps} FPS)...")
        for frame in range(self.total_frames):
            self.simulate_frame(frame)
            if frame % (self.fps * 10) == 0:
                print(f"  Frame {frame}/{self.total_frames} ({frame/self.fps:.0f}s)")
        print("Simulation complete.")

    def write_output(self, out_dir: str):
        os.makedirs(out_dir, exist_ok=True)

        packets_path = os.path.join(out_dir, "packets.ndjson")
        with open(packets_path, "w") as f:
            for pkt in self.packets:
                f.write(pkt.to_ndjson() + "\n")
        print(f"Wrote {len(self.packets)} packets to {packets_path}")

        state_path = os.path.join(out_dir, "world_state.ndjson")
        with open(state_path, "w") as f:
            for state in self.world_states:
                f.write(state.to_ndjson() + "\n")
        print(f"Wrote {len(self.world_states)} states to {state_path}")

        events_path = os.path.join(out_dir, "events.ndjson")
        with open(events_path, "w") as f:
            for evt in self.events:
                f.write(evt.to_ndjson() + "\n")
        print(f"Wrote {len(self.events)} events to {events_path}")

        # Write beat config for renderer
        beats_path = os.path.join(out_dir, "beats.json")
        with open(beats_path, "w") as f:
            json.dump(BEATS, f, indent=2)
        print(f"Wrote beat config to {beats_path}")

        # Write occluder config
        occluder_path = os.path.join(out_dir, "occluder.json")
        with open(occluder_path, "w") as f:
            json.dump(OCCLUDER, f, indent=2)
        print(f"Wrote occluder config to {occluder_path}")


def main():
    parser = argparse.ArgumentParser(description="GodView Demo Fix - Log Generator")
    parser.add_argument("--out", type=str, default="./out", help="Output directory")
    parser.add_argument("--seed", type=int, default=42, help="Random seed")
    parser.add_argument("--fps", type=int, default=30, help="Frames per second")
    parser.add_argument("--duration_s", type=int, default=85, help="Duration in seconds")
    parser.add_argument("--num_agents", type=int, default=20, help="Number of car agents")
    args = parser.parse_args()

    print("=" * 60)
    print("GodView Demo Fix - Log Generator")
    print("=" * 60)
    print(f"Seed: {args.seed}, Duration: {args.duration_s}s, FPS: {args.fps}")
    print("=" * 60)

    sim = WorldSimulator(
        seed=args.seed,
        fps=args.fps,
        duration_s=args.duration_s,
        num_agents=args.num_agents,
    )
    sim.run()
    sim.write_output(args.out)

    print("=" * 60)
    print("DONE")
    print("=" * 60)


if __name__ == "__main__":
    main()
