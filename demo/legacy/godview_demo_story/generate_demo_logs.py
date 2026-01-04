#!/usr/bin/env python3
"""
GodView Demo Log Generator
==========================
Generates deterministic NDJSON logs for the LinkedIn demo.
No CARLA, no ROS. Pure Python simulation.

Outputs:
  - packets.ndjson: Per-agent detection packets
  - world_state.ndjson: Fused canonical state per frame
  - events.ndjson: Discrete events (MERGE, TRUST_REJECT, OOSM_CORRECTED, etc.)

Usage:
  python3 generate_demo_logs.py --out ./out --seed 42 --duration_s 85 --fps 30
"""

import argparse
import json
import math
import os
import random
from dataclasses import dataclass, field, asdict
from typing import List, Dict, Optional, Set, Tuple
from pathlib import Path

# =============================================================================
# CONFIGURATION
# =============================================================================

# Agent counts
NUM_CARS = 20
NUM_DRONES = 1

# Intersection geometry (meters, centered at origin)
INTERSECTION_SIZE = 40.0
ROAD_WIDTH = 8.0

# Phases (in seconds)
PHASE_HOOK = (0, 5)
PHASE_BEFORE = (5, 25)
PHASE_AFTER = (25, 55)
PHASE_MONTAGE = (55, 75)
PHASE_CLOSE = (75, 90)

# Spotlight agents per beat (indices into agent list)
SPOTLIGHT_AGENTS = {
    "hook": ["agent_00", "agent_01", "agent_02"],
    "occlusion": ["agent_00", "agent_01"],  # A=00, B=01
    "ghosts": ["agent_02", "agent_03"],
    "spoof": ["agent_04", "unknown_x"],
    "remote_obs": ["agent_00", "agent_01", "drone_00"],
    "merge": ["agent_02", "agent_03"],
    "trust_reject": ["agent_04", "unknown_x"],
    "oosm": ["agent_07"],
    "space": ["agent_03", "drone_00"],
    "handoff": ["agent_05", "agent_06"],
    "bandwidth": ["agent_00"],
    "close": ["agent_00", "agent_01", "agent_02"],
}

# =============================================================================
# DATA CLASSES
# =============================================================================

@dataclass
class Vec3:
    x: float
    y: float
    z: float

    def to_dict(self):
        return {"x": self.x, "y": self.y, "z": self.z}


@dataclass
class Pose:
    position: Vec3
    yaw: float

    def to_dict(self):
        return {"position": self.position.to_dict(), "yaw": self.yaw}


@dataclass
class DetectedObject:
    local_object_id: str
    obj_class: str
    pose: Pose
    covariance: List[float]  # 2x2 flattened [a, b, c, d]
    velocity: Optional[Vec3] = None

    def to_dict(self):
        d = {
            "local_object_id": self.local_object_id,
            "class": self.obj_class,
            "pose": self.pose.to_dict(),
            "covariance": self.covariance,
        }
        if self.velocity:
            d["velocity"] = self.velocity.to_dict()
        return d


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
    pose: Pose
    covariance: List[float]
    source_agents: List[str]
    confidence: float

    def to_dict(self):
        return {
            "canonical_object_id": self.canonical_object_id,
            "class": self.obj_class,
            "pose": self.pose.to_dict(),
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


# =============================================================================
# AGENT STATE
# =============================================================================

@dataclass
class Agent:
    agent_id: str
    position: Vec3
    yaw: float
    velocity: Vec3
    is_drone: bool = False
    is_spoof: bool = False


# =============================================================================
# WORLD SIMULATION
# =============================================================================

class WorldSimulator:
    """Deterministic world simulation for demo purposes."""

    def __init__(self, seed: int, fps: int, duration_s: int, num_agents: int):
        self.seed = seed
        self.fps = fps
        self.duration_s = duration_s
        self.total_frames = fps * duration_s
        self.num_agents = num_agents

        random.seed(seed)

        # Spoof agent ID (needed before _init_agents)
        self.spoof_agent_id = "unknown_x"
        self.spoof_object_id = "spoof_bus_01"

        # Agents
        self.agents: List[Agent] = []
        self._init_agents()

        # Ground truth objects (pedestrian, obstacles)
        self.gt_objects: Dict[str, Dict] = {}
        self._init_ground_truth_objects()

        # OOSM delay buffer: agent_id -> list of (original_frame, packet)
        self.oosm_buffer: Dict[str, List[Tuple[int, DetectionPacket]]] = {}
        self.oosm_agent = "agent_07"
        self.oosm_delay_frames = 10
        self.oosm_active_range = (int(55 * fps), int(60 * fps))  # Montage beat

        # Phase tracking
        self.current_phase = "hook"

        # Output buffers
        self.packets: List[DetectionPacket] = []
        self.world_states: List[CanonicalState] = []
        self.events: List[Event] = []

    def _init_agents(self):
        """Initialize agent positions around the intersection."""
        # Regular car agents
        for i in range(self.num_agents):
            angle = (2 * math.pi * i) / self.num_agents
            radius = INTERSECTION_SIZE * 0.6
            x = radius * math.cos(angle)
            y = radius * math.sin(angle)
            yaw = angle + math.pi  # facing inward

            self.agents.append(Agent(
                agent_id=f"agent_{i:02d}",
                position=Vec3(x, y, 0.0),
                yaw=yaw,
                velocity=Vec3(random.uniform(1, 3), 0, 0),
                is_drone=False,
            ))

        # Drone agent
        self.agents.append(Agent(
            agent_id="drone_00",
            position=Vec3(0, 0, 15.0),  # altitude 15m
            yaw=0,
            velocity=Vec3(0.5, 0.5, 0),
            is_drone=True,
        ))

        # Spoof agent (not in regular list, but we track it)
        self.spoof_agent = Agent(
            agent_id=self.spoof_agent_id,
            position=Vec3(-20, -20, 0),
            yaw=0,
            velocity=Vec3(0, 0, 0),
            is_spoof=True,
        )

    def _init_ground_truth_objects(self):
        """Initialize ground truth objects (pedestrians, obstacles)."""
        # Pedestrian (for occlusion demo)
        self.gt_objects["ped_01"] = {
            "class": "pedestrian",
            "position": Vec3(5, 2, 0),
            "yaw": 0,
            "velocity": Vec3(0.5, 0, 0),
        }
        # Occluding truck
        self.gt_objects["truck_01"] = {
            "class": "truck",
            "position": Vec3(3, 2, 0),
            "yaw": 0,
            "velocity": Vec3(0, 0, 0),
        }
        # Target car for ghost duplication demo
        self.gt_objects["car_target_01"] = {
            "class": "car",
            "position": Vec3(-8, 5, 0),
            "yaw": math.pi / 4,
            "velocity": Vec3(2, 0, 0),
        }
        # Car that overlaps with drone in x,y (for space demo)
        self.gt_objects["car_overlap_01"] = {
            "class": "car",
            "position": Vec3(0, 0, 0),  # same x,y as drone
            "yaw": 0,
            "velocity": Vec3(1, 0, 0),
        }

    def get_phase(self, frame: int) -> str:
        """Determine current phase based on frame number."""
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
        """Get specific beat within a phase."""
        t = frame / self.fps
        # BEFORE beats
        if 5 <= t < 12:
            return "occlusion"
        elif 12 <= t < 18:
            return "ghosts"
        elif 18 <= t < 25:
            return "spoof"
        # AFTER beats
        elif 25 <= t < 35:
            return "remote_obs"
        elif 35 <= t < 45:
            return "merge"
        elif 45 <= t < 55:
            return "trust_reject"
        # MONTAGE beats
        elif 55 <= t < 60:
            return "oosm"
        elif 60 <= t < 65:
            return "space"
        elif 65 <= t < 70:
            return "handoff"
        elif 70 <= t < 75:
            return "bandwidth"
        else:
            return "general"

    def _update_agent_positions(self, frame: int):
        """Update agent positions with simple motion."""
        dt = 1.0 / self.fps
        for agent in self.agents:
            # Simple circular motion around intersection
            if not agent.is_drone:
                angle_speed = 0.02  # radians per frame
                current_angle = math.atan2(agent.position.y, agent.position.x)
                new_angle = current_angle + angle_speed
                radius = math.sqrt(agent.position.x**2 + agent.position.y**2)
                agent.position.x = radius * math.cos(new_angle)
                agent.position.y = radius * math.sin(new_angle)
                agent.yaw = new_angle + math.pi
            else:
                # Drone hovers with gentle drift
                agent.position.x += 0.1 * math.sin(frame * 0.05)
                agent.position.y += 0.1 * math.cos(frame * 0.05)

        # Update ground truth objects
        for obj_id, obj in self.gt_objects.items():
            if obj["velocity"].x != 0 or obj["velocity"].y != 0:
                obj["position"].x += obj["velocity"].x * dt
                obj["position"].y += obj["velocity"].y * dt

    def _can_agent_see(self, agent: Agent, obj_pos: Vec3, occluders: List[Vec3]) -> bool:
        """Check if agent can see object (simple line-of-sight)."""
        if agent.is_drone:
            return True  # Drone sees everything from above

        for occ in occluders:
            # Simple check: is occluder between agent and object?
            ax, ay = agent.position.x, agent.position.y
            ox, oy = obj_pos.x, obj_pos.y
            bx, by = occ.x, occ.y

            # Vector from agent to object
            dx, dy = ox - ax, oy - ay
            dist_to_obj = math.sqrt(dx*dx + dy*dy)

            # Vector from agent to occluder
            dbx, dby = bx - ax, by - ay
            dist_to_occ = math.sqrt(dbx*dbx + dby*dby)

            if dist_to_occ < dist_to_obj:
                # Check if occluder is roughly in line
                if dist_to_obj > 0.1:
                    dot = (dx * dbx + dy * dby) / (dist_to_obj * dist_to_occ + 0.001)
                    if dot > 0.9:  # Close to line
                        return False
        return True

    def _generate_detection_with_noise(self, obj: Dict, agent: Agent, local_id: str) -> DetectedObject:
        """Generate a detection with sensor noise."""
        noise_x = random.gauss(0, 0.3)
        noise_y = random.gauss(0, 0.3)
        noise_yaw = random.gauss(0, 0.05)

        return DetectedObject(
            local_object_id=local_id,
            obj_class=obj["class"],
            pose=Pose(
                position=Vec3(
                    obj["position"].x + noise_x,
                    obj["position"].y + noise_y,
                    obj["position"].z,
                ),
                yaw=obj["yaw"] + noise_yaw,
            ),
            covariance=[0.5, 0.0, 0.0, 0.5],  # 2x2 diagonal
            velocity=obj.get("velocity"),
        )

    def _generate_spoof_packet(self, frame: int) -> DetectionPacket:
        """Generate a spoofed packet from unknown agent."""
        timestamp_ns = int((frame / self.fps) * 1e9)
        fake_bus = DetectedObject(
            local_object_id=self.spoof_object_id,
            obj_class="bus",
            pose=Pose(position=Vec3(10, 10, 0), yaw=0),
            covariance=[1.0, 0.0, 0.0, 1.0],
        )
        return DetectionPacket(
            frame=frame,
            timestamp_ns=timestamp_ns,
            agent_id=self.spoof_agent_id,
            delivery_frame=frame,
            signature_valid=False,
            objects=[fake_bus],
        )

    def simulate_frame(self, frame: int):
        """Simulate a single frame."""
        phase = self.get_phase(frame)
        beat = self.get_beat(frame)
        timestamp_ns = int((frame / self.fps) * 1e9)

        self._update_agent_positions(frame)

        # Collect packets for this frame
        frame_packets: List[DetectionPacket] = []

        # Generate per-agent detections
        occluders = [self.gt_objects["truck_01"]["position"]]

        for agent in self.agents:
            objects_seen: List[DetectedObject] = []

            for obj_id, obj in self.gt_objects.items():
                # Check visibility (with occlusion during BEFORE phase)
                can_see = True
                if phase == "before" and beat == "occlusion":
                    if obj_id == "ped_01" and agent.agent_id == "agent_01":
                        # agent_01 cannot see pedestrian (occluded)
                        can_see = self._can_agent_see(agent, obj["position"], occluders)

                if can_see and random.random() < 0.8:  # 80% detection rate
                    local_id = f"{agent.agent_id}_{obj_id}"
                    detected = self._generate_detection_with_noise(obj, agent, local_id)
                    objects_seen.append(detected)

            # Handle OOSM for agent_07 during montage
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

        # Generate spoof packet during spoof beats
        if beat == "spoof" or (beat == "trust_reject" and phase == "after"):
            spoof_pkt = self._generate_spoof_packet(frame)
            frame_packets.append(spoof_pkt)

            if phase == "after" and frame == int(50 * self.fps):
                # Emit trust rejection event
                self.events.append(Event(
                    frame=frame,
                    event_type="TRUST_REJECT",
                    payload={
                        "agent_id": self.spoof_agent_id,
                        "object_id": self.spoof_object_id,
                        "reason": "invalid_signature",
                    }
                ))

        self.packets.extend(frame_packets)

        # Generate canonical state
        canonical_objects = self._fuse_to_canonical(frame_packets, frame, phase)
        self.world_states.append(CanonicalState(
            frame=frame,
            timestamp_ns=timestamp_ns,
            objects=canonical_objects,
        ))

        # Generate events based on beat
        self._generate_beat_events(frame, beat, phase)

    def _fuse_to_canonical(self, packets: List[DetectionPacket], frame: int, phase: str) -> List[CanonicalObject]:
        """Simple fusion to create canonical state."""
        # Group detections by ground truth object (simplified)
        obj_observations: Dict[str, List[Tuple[str, DetectedObject]]] = {}

        for pkt in packets:
            # Skip invalid signatures in AFTER phase
            if phase in ["after", "montage", "close"] and not pkt.signature_valid:
                continue

            for det in pkt.objects:
                # Extract GT object ID from local_id (format: agent_XX_obj_id)
                parts = det.local_object_id.split("_", 2)
                if len(parts) >= 3:
                    gt_id = "_".join(parts[2:])
                else:
                    gt_id = det.local_object_id

                if gt_id not in obj_observations:
                    obj_observations[gt_id] = []
                obj_observations[gt_id].append((pkt.agent_id, det))

        # Fuse observations
        canonical_objects: List[CanonicalObject] = []
        for gt_id, observations in obj_observations.items():
            if not observations:
                continue

            # Average position
            avg_x = sum(o[1].pose.position.x for o in observations) / len(observations)
            avg_y = sum(o[1].pose.position.y for o in observations) / len(observations)
            avg_z = sum(o[1].pose.position.z for o in observations) / len(observations)
            avg_yaw = sum(o[1].pose.yaw for o in observations) / len(observations)

            # Reduce covariance based on number of observers
            base_cov = 0.5
            fused_cov = base_cov / math.sqrt(len(observations))

            source_agents = list(set(o[0] for o in observations))

            canonical_objects.append(CanonicalObject(
                canonical_object_id=f"canonical_{gt_id}",
                obj_class=observations[0][1].obj_class,
                pose=Pose(position=Vec3(avg_x, avg_y, avg_z), yaw=avg_yaw),
                covariance=[fused_cov, 0, 0, fused_cov],
                source_agents=source_agents,
                confidence=min(0.99, 0.5 + 0.1 * len(observations)),
            ))

        return canonical_objects

    def _generate_beat_events(self, frame: int, beat: str, phase: str):
        """Generate events for specific beats."""
        # MERGE event at start of merge beat
        if beat == "merge" and frame == int(38 * self.fps):
            self.events.append(Event(
                frame=frame,
                event_type="MERGE",
                payload={
                    "from_ids": ["agent_02_car_target_01", "agent_03_car_target_01"],
                    "to_id": "canonical_car_target_01",
                    "reason": "spatial_gating_highlander",
                }
            ))

        # OOSM_CORRECTED event
        if beat == "oosm" and frame == int(58 * self.fps):
            self.events.append(Event(
                frame=frame,
                event_type="OOSM_CORRECTED",
                payload={
                    "agent_id": self.oosm_agent,
                    "delayed_frames": self.oosm_delay_frames,
                    "correction_magnitude_m": 0.8,
                }
            ))

        # SPACE_SEPARATION event
        if beat == "space" and frame == int(62 * self.fps):
            self.events.append(Event(
                frame=frame,
                event_type="SPACE_SEPARATION",
                payload={
                    "drone_id": "drone_00",
                    "car_id": "agent_03",
                    "delta_z": 15.0,
                }
            ))

        # HANDOFF_OK event
        if beat == "handoff" and frame == int(67 * self.fps):
            self.events.append(Event(
                frame=frame,
                event_type="HANDOFF_OK",
                payload={
                    "object_id": "canonical_car_target_01",
                    "from_agent": "agent_05",
                    "to_agent": "agent_06",
                    "id_stable": True,
                }
            ))

    def run(self):
        """Run full simulation."""
        print(f"Simulating {self.total_frames} frames ({self.duration_s}s at {self.fps} FPS)...")
        for frame in range(self.total_frames):
            self.simulate_frame(frame)
            if frame % (self.fps * 10) == 0:
                print(f"  Frame {frame}/{self.total_frames} ({frame/self.fps:.0f}s)")
        print("Simulation complete.")

    def write_output(self, out_dir: str):
        """Write NDJSON files."""
        os.makedirs(out_dir, exist_ok=True)

        # packets.ndjson
        packets_path = os.path.join(out_dir, "packets.ndjson")
        with open(packets_path, "w") as f:
            for pkt in self.packets:
                f.write(pkt.to_ndjson() + "\n")
        print(f"Wrote {len(self.packets)} packets to {packets_path}")

        # world_state.ndjson
        state_path = os.path.join(out_dir, "world_state.ndjson")
        with open(state_path, "w") as f:
            for state in self.world_states:
                f.write(state.to_ndjson() + "\n")
        print(f"Wrote {len(self.world_states)} states to {state_path}")

        # events.ndjson
        events_path = os.path.join(out_dir, "events.ndjson")
        with open(events_path, "w") as f:
            for evt in self.events:
                f.write(evt.to_ndjson() + "\n")
        print(f"Wrote {len(self.events)} events to {events_path}")


# =============================================================================
# MAIN
# =============================================================================

def main():
    parser = argparse.ArgumentParser(description="GodView Demo Log Generator")
    parser.add_argument("--out", type=str, default="./out", help="Output directory")
    parser.add_argument("--seed", type=int, default=42, help="Random seed for determinism")
    parser.add_argument("--fps", type=int, default=30, help="Frames per second")
    parser.add_argument("--duration_s", type=int, default=85, help="Duration in seconds")
    parser.add_argument("--num_agents", type=int, default=20, help="Number of car agents")
    args = parser.parse_args()

    print("=" * 60)
    print("GodView Demo Log Generator")
    print("=" * 60)
    print(f"Seed: {args.seed}")
    print(f"FPS: {args.fps}")
    print(f"Duration: {args.duration_s}s ({args.fps * args.duration_s} frames)")
    print(f"Agents: {args.num_agents} cars + 1 drone")
    print(f"Output: {args.out}")
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
