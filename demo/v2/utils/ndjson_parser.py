#!/usr/bin/env python3
"""
NDJSON Parser for GodView V2 Demo
==================================
Parses DETECTION, MERGE_EVENT, and CANONICAL_STATE packets
per the GodView specification.
"""

import json
from dataclasses import dataclass, field
from typing import List, Dict, Optional, Generator
from pathlib import Path


@dataclass
class ObjectDetection:
    """Single detected object from a sensor."""
    local_id: str
    object_class: str
    confidence: float
    pose: Dict[str, float]  # x, y, z, yaw, pitch, roll
    bbox_extent: Dict[str, float]  # x, y, z (half-extents)
    covariance: Optional[List[float]] = None
    note: Optional[str] = None
    signature: Optional[str] = None


@dataclass
class DetectionPacket:
    """Raw detection packet from a sensor."""
    packet_type: str = "DETECTION"
    sensor_id: str = ""
    timestamp_ns: int = 0
    sequence_id: int = 0
    objects: List[ObjectDetection] = field(default_factory=list)


@dataclass
class MergeEventDetails:
    """Details for merge/trust events."""
    # For ID_MERGE
    incoming_id: Optional[str] = None
    canonical_id: Optional[str] = None
    method: Optional[str] = None
    confidence_boost: Optional[float] = None
    # For TRUST_REJECT
    sensor_id: Optional[str] = None
    reason: Optional[str] = None


@dataclass
class MergeEventPacket:
    """Merge or trust event packet."""
    packet_type: str = "MERGE_EVENT"
    timestamp_ns: int = 0
    event_code: str = ""  # ID_MERGE, TRUST_REJECT, OOSM_CORRECTED, etc.
    details: Optional[MergeEventDetails] = None


@dataclass
class CanonicalObject:
    """Single canonical (fused) object."""
    canonical_id: str
    object_class: str
    confidence: float
    pose: Dict[str, float]  # x, y, z, yaw
    bbox_extent: Dict[str, float]  # x, y, z
    velocity: Optional[Dict[str, float]] = None  # vx, vy, vz


@dataclass
class CanonicalStatePacket:
    """Fused canonical state packet."""
    packet_type: str = "CANONICAL_STATE"
    timestamp_ns: int = 0
    objects: List[CanonicalObject] = field(default_factory=list)


def parse_object_detection(obj_dict: dict) -> ObjectDetection:
    """Parse a single object from detection packet."""
    return ObjectDetection(
        local_id=obj_dict.get('local_id', obj_dict.get('id', 'unknown')),
        object_class=obj_dict.get('class', 'unknown'),
        confidence=obj_dict.get('confidence', 0.5),
        pose=obj_dict.get('pose', {'x': 0, 'y': 0, 'z': 0, 'yaw': 0}),
        bbox_extent=obj_dict.get('bbox_extent', {'x': 2.0, 'y': 2.0, 'z': 1.5}),
        covariance=obj_dict.get('covariance'),
        note=obj_dict.get('note'),
        signature=obj_dict.get('signature')
    )


def parse_canonical_object(obj_dict: dict) -> CanonicalObject:
    """Parse a single canonical object."""
    return CanonicalObject(
        canonical_id=obj_dict.get('canonical_id', obj_dict.get('id', 'unknown')),
        object_class=obj_dict.get('class', 'unknown'),
        confidence=obj_dict.get('confidence', 0.9),
        pose=obj_dict.get('pose', {'x': 0, 'y': 0, 'z': 0, 'yaw': 0}),
        bbox_extent=obj_dict.get('bbox_extent', {'x': 2.0, 'y': 2.0, 'z': 1.5}),
        velocity=obj_dict.get('velocity')
    )


def parse_packet(line: str) -> Optional[DetectionPacket | MergeEventPacket | CanonicalStatePacket]:
    """
    Parse a single NDJSON line into a packet object.
    
    Returns None for invalid lines.
    """
    try:
        data = json.loads(line.strip())
    except json.JSONDecodeError:
        return None
    
    packet_type = data.get('packet_type', '')
    
    if packet_type == 'DETECTION':
        packet = DetectionPacket(
            sensor_id=data.get('sensor_id', ''),
            timestamp_ns=data.get('timestamp_ns', 0),
            sequence_id=data.get('sequence_id', 0)
        )
        for obj in data.get('objects', []):
            packet.objects.append(parse_object_detection(obj))
        return packet
    
    elif packet_type == 'MERGE_EVENT':
        details_dict = data.get('details', {})
        details = MergeEventDetails(
            incoming_id=details_dict.get('incoming_id'),
            canonical_id=details_dict.get('canonical_id'),
            method=details_dict.get('method'),
            confidence_boost=details_dict.get('confidence_boost'),
            sensor_id=details_dict.get('sensor_id'),
            reason=details_dict.get('reason')
        )
        return MergeEventPacket(
            timestamp_ns=data.get('timestamp_ns', 0),
            event_code=data.get('event_code', ''),
            details=details
        )
    
    elif packet_type == 'CANONICAL_STATE':
        packet = CanonicalStatePacket(
            timestamp_ns=data.get('timestamp_ns', 0)
        )
        for obj in data.get('objects', []):
            packet.objects.append(parse_canonical_object(obj))
        return packet
    
    # Legacy format support (V1 compatibility)
    elif 'id' in data and 'x' in data:
        # Convert legacy format to DETECTION
        obj = ObjectDetection(
            local_id=data.get('id'),
            object_class=data.get('class', 'vehicle'),
            confidence=data.get('confidence', 0.7),
            pose={'x': data['x'], 'y': data['y'], 'z': data.get('z', 0), 'yaw': 0},
            bbox_extent={'x': 2.0, 'y': 4.5, 'z': 1.5},
            note='LEGACY_FORMAT'
        )
        return DetectionPacket(
            sensor_id=data.get('source', 'legacy'),
            timestamp_ns=int(data.get('ts', 0) * 1e9),
            objects=[obj]
        )
    
    return None


def load_ndjson_file(filepath: Path) -> Generator:
    """
    Load and parse an NDJSON file.
    
    Yields packet objects for each valid line.
    """
    with open(filepath, 'r') as f:
        for line in f:
            if line.strip():
                packet = parse_packet(line)
                if packet is not None:
                    yield packet


def load_packets_by_timestamp(filepath: Path) -> Dict[int, List]:
    """
    Load NDJSON and group packets by timestamp.
    
    Returns:
        Dict mapping timestamp_ns to list of packets at that time
    """
    packets_by_ts = {}
    
    for packet in load_ndjson_file(filepath):
        ts = packet.timestamp_ns
        if ts not in packets_by_ts:
            packets_by_ts[ts] = []
        packets_by_ts[ts].append(packet)
    
    return packets_by_ts


def get_frame_data(
    raw_packets: Dict[int, List],
    fused_packets: Dict[int, List],
    merge_events: Dict[int, List],
    frame_idx: int,
    fps: int = 30
) -> Dict:
    """
    Get all data for a specific frame.
    
    Args:
        raw_packets: Dict of raw detection packets by timestamp
        fused_packets: Dict of canonical state packets by timestamp
        merge_events: Dict of merge event packets by timestamp
        frame_idx: Frame index (0-based)
        fps: Frames per second
    
    Returns:
        Dict with 'raw', 'fused', 'events' lists for this frame
    """
    # Convert frame index to timestamp
    time_sec = frame_idx / fps
    timestamp_ns = int(time_sec * 1e9)
    
    # Find closest timestamp (within 50ms window)
    window_ns = int(0.05 * 1e9)
    
    def find_nearest(packets_dict, target_ts):
        if target_ts in packets_dict:
            return packets_dict[target_ts]
        
        # Search within window
        for ts in packets_dict:
            if abs(ts - target_ts) <= window_ns:
                return packets_dict[ts]
        return []
    
    return {
        'raw': find_nearest(raw_packets, timestamp_ns),
        'fused': find_nearest(fused_packets, timestamp_ns),
        'events': find_nearest(merge_events, timestamp_ns),
        'frame_idx': frame_idx,
        'timestamp_ns': timestamp_ns
    }


# Event code constants
class EventCode:
    ID_MERGE = "ID_MERGE"
    TRUST_REJECT = "TRUST_REJECT"
    OOSM_CORRECTED = "OOSM_CORRECTED"
    PANCAKE_FIXED = "PANCAKE_FIXED"
    GHOST_ELIMINATED = "GHOST_ELIMINATED"
