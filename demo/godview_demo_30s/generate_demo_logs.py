#!/usr/bin/env python3
"""
GodView 30s Demo - Log Generator
=================================
Generates BEFORE and AFTER datasets for split-screen comparison.
Simple world: 2 agents + 1 drone + 1 hero pedestrian + building occluder.
"""

import argparse
import json
import math
import os
import random
from dataclasses import dataclass
from typing import List, Dict, Tuple
from pathlib import Path

# =============================================================================
# CONFIGURATION
# =============================================================================

FPS = 30
DURATION_S = 30
TOTAL_FRAMES = FPS * DURATION_S  # 900

# Agents
AGENT_A = {"id": "agent_A", "x": -15, "y": -5, "color": "blue"}
AGENT_B = {"id": "agent_B", "x": 10, "y": -10, "color": "orange"}
DRONE = {"id": "drone", "x": 0, "y": 0, "z": 12, "color": "green"}
SPOOF = {"id": "unknown_node", "color": "red"}

# Scale agents (for network pane, minimal world presence)
SCALE_AGENTS = [{"id": f"agent_{i:02d}", "x": 20 + i*2, "y": -20 + i} for i in range(3, 20)]

# Building occluder
BUILDING = {"x": 5, "y": 5, "width": 8, "height": 10}

# Hero object: pedestrian behind building
HERO_PED = {"id": "ped_hero", "class": "pedestrian", "x": 12, "y": 8, "vx": -0.1, "vy": 0}

# Background objects (optional, not focus)
BG_CARS = [
    {"id": "car_bg_1", "class": "car", "x": -20, "y": 5, "vx": 0.3, "vy": 0},
    {"id": "car_bg_2", "class": "car", "x": 15, "y": -15, "vx": 0, "vy": 0.2},
]

# Beats (frame ranges)
BEAT_THESIS = (0, 90)           # 0-3s
BEAT_OCCLUSION = (90, 360)      # 3-12s
BEAT_MERGE = (360, 600)         # 12-20s
BEAT_TRUST = (600, 780)         # 20-26s
BEAT_SCALE = (780, 900)         # 26-30s


# =============================================================================
# GEOMETRY
# =============================================================================

def line_intersects_rect(ax: float, ay: float, bx: float, by: float,
                          rx: float, ry: float, rw: float, rh: float) -> bool:
    """Check if line segment (ax,ay)-(bx,by) intersects rectangle centered at (rx,ry)."""
    # Sample points along line
    for t in [i * 0.05 for i in range(1, 20)]:
        px = ax + t * (bx - ax)
        py = ay + t * (by - ay)
        if (rx - rw/2 <= px <= rx + rw/2) and (ry - rh/2 <= py <= ry + rh/2):
            return True
    return False


def is_occluded(agent_x: float, agent_y: float, obj_x: float, obj_y: float) -> bool:
    """Check if object is occluded from agent by building."""
    return line_intersects_rect(
        agent_x, agent_y, obj_x, obj_y,
        BUILDING["x"], BUILDING["y"], BUILDING["width"], BUILDING["height"]
    )


# =============================================================================
# DATA GENERATION
# =============================================================================

@dataclass
class DetectedObject:
    local_id: str
    obj_class: str
    x: float
    y: float
    z: float
    yaw: float
    cov: List[float]
    confidence: float

    def to_dict(self):
        return {
            "local_object_id": self.local_id,
            "class": self.obj_class,
            "pose": {"position": {"x": self.x, "y": self.y, "z": self.z}, "yaw": self.yaw},
            "covariance": self.cov,
            "confidence": self.confidence,
        }


def generate_detection_packet(frame: int, agent: Dict, objects: List[DetectedObject],
                               delivery_frame: int = None, sig_valid: bool = True) -> Dict:
    if delivery_frame is None:
        delivery_frame = frame
    return {
        "packet_type": "DETECTION",
        "frame": frame,
        "timestamp_ns": int((frame / FPS) * 1e9),
        "agent_id": agent["id"],
        "delivery_frame": delivery_frame,
        "signature_valid": sig_valid,
        "objects": [o.to_dict() for o in objects],
    }


def generate_canonical_state(frame: int, objects: List[Dict]) -> Dict:
    return {
        "packet_type": "CANONICAL_STATE",
        "frame": frame,
        "timestamp_ns": int((frame / FPS) * 1e9),
        "objects": objects,
    }


def generate_logs(out_dir: Path, seed: int = 42):
    random.seed(seed)
    os.makedirs(out_dir, exist_ok=True)

    # Data buffers
    packets_before = []
    packets_after = []
    world_before = []
    world_after = []
    events_before = []
    events_after = []

    # Object positions (will update per frame)
    ped_x, ped_y = HERO_PED["x"], HERO_PED["y"]
    car1_x, car1_y = BG_CARS[0]["x"], BG_CARS[0]["y"]
    car2_x, car2_y = BG_CARS[1]["x"], BG_CARS[1]["y"]

    # Event flags
    merge_emitted = False
    trust_reject_emitted = False
    oosm_emitted = False
    packet_arrival_emitted = False

    for frame in range(TOTAL_FRAMES):
        dt = 1.0 / FPS

        # Update positions (slow movement)
        ped_x += HERO_PED["vx"] * dt
        ped_y += HERO_PED["vy"] * dt
        car1_x += BG_CARS[0]["vx"] * dt
        car2_x += BG_CARS[1]["vx"] * dt

        # Determine beat
        in_occlusion = BEAT_OCCLUSION[0] <= frame < BEAT_OCCLUSION[1]
        in_merge = BEAT_MERGE[0] <= frame < BEAT_MERGE[1]
        in_trust = BEAT_TRUST[0] <= frame < BEAT_TRUST[1]

        # =========== AGENT A DETECTIONS ===========
        # A can see pedestrian (no occlusion from A's position)
        a_sees_ped = not is_occluded(AGENT_A["x"], AGENT_A["y"], ped_x, ped_y)
        if a_sees_ped:
            ped_det_a = DetectedObject(
                local_id=f"A_ped_{frame % 100:02d}",  # A's local ID
                obj_class="pedestrian",
                x=ped_x + random.gauss(0, 0.1),
                y=ped_y + random.gauss(0, 0.1),
                z=0, yaw=0,
                cov=[0.3, 0, 0, 0.3],
                confidence=0.85,
            )
            pkt_a = generate_detection_packet(frame, AGENT_A, [ped_det_a])
            packets_before.append(pkt_a)
            packets_after.append(pkt_a)

        # =========== AGENT B DETECTIONS ===========
        # B is occluded from ped by building
        b_sees_ped = not is_occluded(AGENT_B["x"], AGENT_B["y"], ped_x, ped_y)

        if b_sees_ped:
            # B can see (rare, only if ped moves)
            ped_det_b = DetectedObject(
                local_id=f"B_ped_{frame % 100:02d}",
                obj_class="pedestrian",
                x=ped_x + random.gauss(0, 0.1),
                y=ped_y + random.gauss(0, 0.1),
                z=0, yaw=0,
                cov=[0.3, 0, 0, 0.3],
                confidence=0.80,
            )
            pkt_b = generate_detection_packet(frame, AGENT_B, [ped_det_b])
            packets_before.append(pkt_b)
            packets_after.append(pkt_b)

        # =========== DRONE DETECTIONS ===========
        # Drone sees everything from above
        ped_det_drone = DetectedObject(
            local_id=f"D_ped_{(frame + 50) % 100:02d}",  # Different ID!
            obj_class="pedestrian",
            x=ped_x + random.gauss(0, 0.15),
            y=ped_y + random.gauss(0, 0.15),
            z=0, yaw=0,
            cov=[0.4, 0, 0, 0.4],
            confidence=0.75,
        )

        # OOSM: delay drone packet during merge beat
        delivery = frame
        if in_merge and 400 <= frame < 420:
            delivery = frame + 10  # Delayed by 10 frames

        pkt_drone = generate_detection_packet(frame, DRONE, [ped_det_drone], delivery)
        packets_before.append(pkt_drone)
        packets_after.append(pkt_drone)

        # =========== SPOOF PACKET ===========
        if in_trust and 650 <= frame < 700:
            spoof_det = DetectedObject(
                local_id="SPOOF_ped_99",
                obj_class="pedestrian",
                x=-5, y=10, z=0, yaw=0,
                cov=[1.0, 0, 0, 1.0],
                confidence=0.9,
            )
            pkt_spoof = generate_detection_packet(frame, SPOOF, [spoof_det], sig_valid=False)
            packets_before.append(pkt_spoof)
            packets_after.append(pkt_spoof)  # Still sent, but will be rejected

        # =========== WORLD STATE BEFORE ===========
        # Before: no fusion, show all raw detections as separate objects
        before_objs = []

        # A's detection
        if a_sees_ped:
            before_objs.append({
                "canonical_object_id": f"raw_A_ped",
                "class": "pedestrian",
                "pose": {"position": {"x": ped_x, "y": ped_y, "z": 0}, "yaw": 0},
                "covariance": [0.3, 0, 0, 0.3],
                "source_agents": ["agent_A"],
                "confidence": 0.85,
            })

        # Drone's detection (different ID = ghost duplicate)
        before_objs.append({
            "canonical_object_id": f"raw_D_ped",
            "class": "pedestrian",
            "pose": {"position": {"x": ped_x + 0.3, "y": ped_y - 0.2, "z": 0}, "yaw": 0},
            "covariance": [0.4, 0, 0, 0.4],
            "source_agents": ["drone"],
            "confidence": 0.75,
        })

        # Accept spoof in BEFORE
        if in_trust and 650 <= frame < 700:
            before_objs.append({
                "canonical_object_id": "SPOOF_ped",
                "class": "pedestrian",
                "pose": {"position": {"x": -5, "y": 10, "z": 0}, "yaw": 0},
                "covariance": [1.0, 0, 0, 1.0],
                "source_agents": ["unknown_node"],
                "confidence": 0.9,
            })

        world_before.append(generate_canonical_state(frame, before_objs))

        # =========== WORLD STATE AFTER ===========
        # After: fused, merged, trust-filtered
        after_objs = []

        # Merged canonical pedestrian
        after_objs.append({
            "canonical_object_id": "canonical_ped_1",
            "class": "pedestrian",
            "pose": {"position": {"x": ped_x, "y": ped_y, "z": 0}, "yaw": 0},
            "covariance": [0.15, 0, 0, 0.15],  # Reduced by fusion
            "source_agents": ["agent_A", "drone"] + (["agent_B"] if b_sees_ped else []),
            "confidence": 0.95,
        })

        # Spoof is REJECTED in AFTER (not in world state)

        world_after.append(generate_canonical_state(frame, after_objs))

        # =========== EVENTS ===========
        # PACKET_ARRIVAL (for occlusion beat)
        if in_occlusion and frame == 200 and not packet_arrival_emitted:
            events_after.append({
                "packet_type": "EVENT",
                "frame": frame,
                "event_type": "PACKET_ARRIVAL",
                "payload": {
                    "src": "agent_A",
                    "dst": "agent_B",
                    "object_id": "ped_hero",
                },
            })
            packet_arrival_emitted = True

        # MERGE event
        if in_merge and frame == 450 and not merge_emitted:
            events_after.append({
                "packet_type": "EVENT",
                "frame": frame,
                "event_type": "MERGE",
                "payload": {
                    "from_ids": ["A_ped", "D_ped"],
                    "canonical_id": "canonical_ped_1",
                },
            })
            merge_emitted = True

        # TRUST_REJECT event
        if in_trust and frame == 680 and not trust_reject_emitted:
            events_after.append({
                "packet_type": "EVENT",
                "frame": frame,
                "event_type": "TRUST_REJECT",
                "payload": {
                    "agent_id": "unknown_node",
                    "reason": "invalid_signature",
                },
            })
            trust_reject_emitted = True

        # OOSM event
        if in_merge and frame == 420 and not oosm_emitted:
            events_after.append({
                "packet_type": "EVENT",
                "frame": frame,
                "event_type": "OOSM_CORRECTED",
                "payload": {
                    "agent_id": "drone",
                    "delayed_frames": 10,
                },
            })
            oosm_emitted = True

    # Write files
    def write_ndjson(path, data):
        with open(path, "w") as f:
            for item in data:
                f.write(json.dumps(item) + "\n")
        print(f"Wrote {len(data)} records to {path}")

    write_ndjson(out_dir / "packets_before.ndjson", packets_before)
    write_ndjson(out_dir / "packets_after.ndjson", packets_after)
    write_ndjson(out_dir / "world_before.ndjson", world_before)
    write_ndjson(out_dir / "world_after.ndjson", world_after)
    write_ndjson(out_dir / "events_before.ndjson", events_before)
    write_ndjson(out_dir / "events_after.ndjson", events_after)

    # Write building config
    with open(out_dir / "config.json", "w") as f:
        json.dump({
            "building": BUILDING,
            "agent_a": AGENT_A,
            "agent_b": AGENT_B,
            "drone": DRONE,
            "hero_ped": HERO_PED,
            "beats": {
                "thesis": BEAT_THESIS,
                "occlusion": BEAT_OCCLUSION,
                "merge": BEAT_MERGE,
                "trust": BEAT_TRUST,
                "scale": BEAT_SCALE,
            },
        }, f, indent=2)
    print(f"Wrote config to {out_dir / 'config.json'}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", default="./out")
    parser.add_argument("--seed", type=int, default=42)
    args = parser.parse_args()

    print("=" * 60)
    print("GodView 30s Demo - Log Generator")
    print("=" * 60)

    generate_logs(Path(args.out), args.seed)

    print("=" * 60)
    print("DONE")
    print("=" * 60)


if __name__ == "__main__":
    main()
