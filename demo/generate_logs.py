#!/usr/bin/env python3
"""
GodView Demo - Log Generator (The Noise Maker)
Parses ground truth NDJSON and generates two streams:
  - raw_broken.ndjson: Simulates sensor failures (OOSM, Pancake, Ghosts, Sybil)
  - godview_merged.ndjson: Simulates GodView corrections

Based on: carla.md Section 6.3 and 7.1-7.2
"""

import json
import random
import uuid
import hashlib
import os
from collections import defaultdict
from typing import List, Dict, Any

# Configuration
GROUND_TRUTH_PATH = "/workspace/godview_demo/logs/ground_truth.ndjson"
RAW_BROKEN_PATH = "/workspace/godview_demo/logs/raw_broken.ndjson"
GODVIEW_MERGED_PATH = "/workspace/godview_demo/logs/godview_merged.ndjson"
MERGE_EVENTS_PATH = "/workspace/godview_demo/logs/merge_events.ndjson"

# Fault injection parameters
OOSM_AFFECTED_RATIO = 0.30  # 30% of actors get delayed timestamps
OOSM_DELAY_MIN = 0.2  # seconds
OOSM_DELAY_MAX = 1.5  # seconds

GHOST_ACTOR_COUNT = 2  # Number of actors that get ghost duplicates
GHOST_OFFSET_METERS = 2.5  # Position offset for ghost

SYBIL_INJECTION_FRAMES = [100, 200, 300, 400, 500]  # When to inject fake obstacles
SYBIL_POSITION = {"x": 50.0, "y": -25.0, "z": 0.0}  # Fixed fake obstacle position


def generate_uuid():
    """Generate a deterministic-looking UUID."""
    return str(uuid.uuid4())


def generate_fake_signature():
    """Generate a fake Ed25519 signature (invalid for demo)."""
    return "INVALID_" + hashlib.sha256(str(random.random()).encode()).hexdigest()[:32]


def generate_valid_signature(data: dict) -> str:
    """Generate a mock valid Ed25519 signature."""
    payload = json.dumps(data, sort_keys=True)
    return "Ed25519_" + hashlib.sha256(payload.encode()).hexdigest()[:48]


def load_ground_truth() -> List[Dict]:
    """Load ground truth NDJSON."""
    print(f"Loading ground truth from: {GROUND_TRUTH_PATH}")
    packets = []
    with open(GROUND_TRUTH_PATH, "r") as f:
        for line in f:
            if line.strip():
                packets.append(json.loads(line))
    print(f"  Loaded {len(packets)} packets")
    return packets


def inject_oosm_delays(packets: List[Dict], affected_actors: set) -> List[Dict]:
    """Apply Gaussian timestamp delays to simulate OOSM."""
    result = []
    for p in packets:
        modified = p.copy()
        if p["actor_id"] in affected_actors:
            delay = random.gauss((OOSM_DELAY_MIN + OOSM_DELAY_MAX) / 2, 0.3)
            delay = max(OOSM_DELAY_MIN, min(OOSM_DELAY_MAX, delay))
            modified["timestamp"] = p["timestamp"] + delay
            modified["oosm_injected"] = True
        result.append(modified)
    return result


def inject_pancake_world(packets: List[Dict]) -> List[Dict]:
    """Flatten drone Z-coordinates to 0 (simulates 2D perception failure)."""
    result = []
    for p in packets:
        modified = p.copy()
        if p.get("is_drone", False):
            modified["position"] = p["position"].copy()
            modified["position"]["z"] = 0.0
            modified["pancake_flattened"] = True
        result.append(modified)
    return result


def inject_ghosts(packets: List[Dict], ghost_actors: set) -> List[Dict]:
    """Create duplicate detections with different UUIDs."""
    result = []
    for p in packets:
        result.append(p)
        if p["actor_id"] in ghost_actors:
            ghost = p.copy()
            ghost["entity_id"] = generate_uuid()  # New UUID
            ghost["original_actor_id"] = p["actor_id"]
            ghost["position"] = {
                "x": p["position"]["x"] + random.gauss(GHOST_OFFSET_METERS, 0.5),
                "y": p["position"]["y"] + random.gauss(GHOST_OFFSET_METERS, 0.5),
                "z": p["position"]["z"]
            }
            ghost["ghost_injected"] = True
            result.append(ghost)
    return result


def inject_sybil_attack(packets: List[Dict]) -> List[Dict]:
    """Inject fake obstacle with invalid signature."""
    result = []
    frame_set = set(SYBIL_INJECTION_FRAMES)
    injected_frames = set()
    
    for p in packets:
        result.append(p)
        if p["frame"] in frame_set and p["frame"] not in injected_frames:
            sybil = {
                "frame": p["frame"],
                "timestamp": p["timestamp"],
                "actor_id": -999,
                "entity_id": "MALICIOUS_" + generate_uuid(),
                "actor_type": "obstacle",
                "position": SYBIL_POSITION.copy(),
                "rotation": {"pitch": 0, "yaw": 0, "roll": 0},
                "velocity": {"x": 0, "y": 0, "z": 0},
                "sybil_attack": True,
                "signature": generate_fake_signature()
            }
            result.append(sybil)
            injected_frames.add(p["frame"])
    return result


def generate_raw_broken_stream(packets: List[Dict]) -> List[Dict]:
    """Apply all fault injections to create the broken stream."""
    print("\nGenerating raw_broken stream (The Chaos)...")
    
    # Select affected actors
    actor_ids = list(set(p["actor_id"] for p in packets))
    random.shuffle(actor_ids)
    
    oosm_actors = set(actor_ids[:int(len(actor_ids) * OOSM_AFFECTED_RATIO)])
    ghost_actors = set(actor_ids[:GHOST_ACTOR_COUNT])
    
    print(f"  OOSM affected actors: {len(oosm_actors)}")
    print(f"  Ghost actors: {len(ghost_actors)}")
    
    # Apply faults in sequence
    result = packets.copy()
    result = inject_oosm_delays(result, oosm_actors)
    result = inject_pancake_world(result)
    result = inject_ghosts(result, ghost_actors)
    result = inject_sybil_attack(result)
    
    # Convert to GlobalHazardPacket format
    output = []
    for p in result:
        packet = {
            "packet_type": "DETECTION",
            "entity_id": p.get("entity_id", f"actor_{p['actor_id']}"),
            "position": [p["position"]["x"], p["position"]["y"], p["position"]["z"]],
            "velocity": [p["velocity"]["x"], p["velocity"]["y"], p["velocity"]["z"]],
            "class_id": 4 if p.get("is_drone") else (2 if p["actor_type"] == "pedestrian" else 1),
            "timestamp": p["timestamp"],
            "confidence_score": random.uniform(0.7, 0.95),
            "frame": p["frame"],
            # Fault markers (for visualization)
            "faults": {
                "oosm": p.get("oosm_injected", False),
                "pancake": p.get("pancake_flattened", False),
                "ghost": p.get("ghost_injected", False),
                "sybil": p.get("sybil_attack", False)
            },
            "signature": p.get("signature", generate_valid_signature(p))
        }
        output.append(packet)
    
    print(f"  Total broken packets: {len(output)}")
    return output


def generate_godview_merged_stream(raw_packets: List[Dict], ground_truth: List[Dict]) -> tuple:
    """Apply GodView corrections to fix all faults."""
    print("\nGenerating godview_merged stream (The Solution)...")
    
    merged = []
    merge_events = []
    
    # Group packets by frame for processing
    by_frame = defaultdict(list)
    for p in raw_packets:
        by_frame[p["frame"]].append(p)
    
    gt_by_frame = defaultdict(dict)
    for p in ground_truth:
        gt_by_frame[p["frame"]][p["actor_id"]] = p
    
    ghost_merges = {}  # Track ghost -> canonical mappings
    rejected_sybils = 0
    resequenced_oosm = 0
    z_corrections = 0
    
    for frame in sorted(by_frame.keys()):
        frame_packets = by_frame[frame]
        gt_frame = gt_by_frame.get(frame, {})
        
        for p in frame_packets:
            # 1. TRUST: Reject Sybil attacks (invalid signatures)
            if p.get("faults", {}).get("sybil", False):
                rejected_sybils += 1
                merge_events.append({
                    "packet_type": "MERGE_EVENT",
                    "timestamp": p["timestamp"],
                    "event_code": "TRUST_REJECT",
                    "frame": frame,
                    "details": {
                        "entity_id": p["entity_id"],
                        "reason": "INVALID_ED25519_SIGNATURE",
                        "method": "SecurityContext.verify_packet()"
                    }
                })
                continue
            
            # 2. IDENTITY: Merge ghost duplicates (Highlander principle)
            if p.get("faults", {}).get("ghost", False):
                # Find canonical ID from ground truth
                entity_parts = p["entity_id"].split("_")
                if "actor" in entity_parts:
                    canonical_id = p["entity_id"]
                else:
                    canonical_id = f"actor_{p.get('original_actor_id', p['entity_id'])}"
                
                if p["entity_id"] not in ghost_merges:
                    ghost_merges[p["entity_id"]] = canonical_id
                    merge_events.append({
                        "packet_type": "MERGE_EVENT",
                        "timestamp": p["timestamp"],
                        "event_code": "ID_MERGE",
                        "frame": frame,
                        "details": {
                            "incoming_id": p["entity_id"],
                            "canonical_id": canonical_id,
                            "method": "HIGHLANDER_MIN_UUID",
                            "confidence_boost": 0.12
                        }
                    })
                continue  # Skip ghost, keep only canonical
            
            # 3. TIME: Fix OOSM by sorting (simulates AugmentedStateFilter replay)
            if p.get("faults", {}).get("oosm", False):
                resequenced_oosm += 1
                # Restore correct timestamp from ground truth
                actor_id = int(p["entity_id"].split("_")[-1]) if "actor_" in p["entity_id"] else -1
                if actor_id in gt_frame:
                    p["timestamp"] = gt_frame[actor_id]["timestamp"]
            
            # 4. SPACE: Restore Z-height (simulates SpatialEngine voxel lookup)
            if p.get("faults", {}).get("pancake", False):
                actor_id = int(p["entity_id"].split("_")[-1]) if "actor_" in p["entity_id"] else -1
                if actor_id in gt_frame:
                    corrected_z = gt_frame[actor_id]["position"]["z"]
                    p["position"][2] = corrected_z
                    z_corrections += 1
                    merge_events.append({
                        "packet_type": "MERGE_EVENT",
                        "timestamp": p["timestamp"],
                        "event_code": "SPATIAL_CORRECTION",
                        "frame": frame,
                        "details": {
                            "entity_id": p["entity_id"],
                            "corrected_z": corrected_z,
                            "method": "H3_VOXEL_GRID_LOOKUP"
                        }
                    })
            
            # Add to merged stream (without fault markers)
            clean_packet = {
                "packet_type": "DETECTION",
                "entity_id": p["entity_id"],
                "position": p["position"],
                "velocity": p["velocity"],
                "class_id": p["class_id"],
                "timestamp": p["timestamp"],
                "confidence_score": min(0.98, p["confidence_score"] + 0.1),
                "frame": p["frame"],
                "verified": True,
                "signature": generate_valid_signature(p)
            }
            merged.append(clean_packet)
    
    # Sort by timestamp (final OOSM fix)
    merged.sort(key=lambda x: (x["frame"], x["timestamp"]))
    
    print(f"  Sybil attacks rejected: {rejected_sybils}")
    print(f"  Ghost merges: {len(ghost_merges)}")
    print(f"  OOSM resequenced: {resequenced_oosm}")
    print(f"  Z-height corrections: {z_corrections}")
    print(f"  Final merged packets: {len(merged)}")
    
    return merged, merge_events


def save_ndjson(data: List[Dict], path: str):
    """Save list of dicts as NDJSON."""
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w") as f:
        for item in data:
            f.write(json.dumps(item) + "\n")
    print(f"  Saved: {path} ({len(data)} records)")


def main():
    separator = "=" * 60
    print(separator)
    print("GodView Demo - Log Generator")
    print(separator)
    
    # Load ground truth
    ground_truth = load_ground_truth()
    
    # Generate broken stream (with all faults)
    raw_broken = generate_raw_broken_stream(ground_truth)
    save_ndjson(raw_broken, RAW_BROKEN_PATH)
    
    # Generate corrected stream (GodView applied)
    godview_merged, merge_events = generate_godview_merged_stream(raw_broken, ground_truth)
    save_ndjson(godview_merged, GODVIEW_MERGED_PATH)
    save_ndjson(merge_events, MERGE_EVENTS_PATH)
    
    print(f"\n{separator}")
    print("COMPLETE!")
    print(f"  Raw/Broken:  {RAW_BROKEN_PATH}")
    print(f"  GodView:     {GODVIEW_MERGED_PATH}")
    print(f"  Events:      {MERGE_EVENTS_PATH}")
    print(separator)


if __name__ == "__main__":
    main()
