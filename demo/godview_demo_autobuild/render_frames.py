#!/usr/bin/env python3
"""
GodView Demo - Frame Renderer
=============================
Renders frames using OpenCV (2D) with optional Open3D for 3D world view.
Falls back to pure OpenCV if Open3D unavailable or headless.

Output: out/frames/frame_XXXXX.png

This renderer creates a 3-pane layout:
- Pane A (left 2/3): World View - top-down 2.5D with boxes and uncertainty ellipses
- Pane B (top-right 1/3): Network View - nodes and packet dots
- Pane C (bottom-right 1/3): Shared vs Not Shared + event log
"""

import argparse
import json
import math
import os
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np

# Try to import Open3D for point cloud rendering
try:
    import open3d as o3d
    HAS_OPEN3D = True
except ImportError:
    HAS_OPEN3D = False
    print("[WARN] Open3D not available, using 2D-only rendering")

# =============================================================================
# CONFIGURATION
# =============================================================================

# Canvas size
WIDTH = 1920
HEIGHT = 1080

# Pane layout
WORLD_PANE_WIDTH = 1280
WORLD_PANE_HEIGHT = 1080
NETWORK_PANE_WIDTH = 640
NETWORK_PANE_HEIGHT = 540
DATA_PANE_WIDTH = 640
DATA_PANE_HEIGHT = 540

# Colors (BGR for OpenCV)
BG_COLOR = (10, 10, 10)
GRID_COLOR = (30, 30, 30)
PANE_BORDER = (50, 50, 50)

# Status colors
COLOR_RAW = (80, 80, 255)         # Red - before/chaos
COLOR_FUSED = (80, 255, 80)       # Green - after/stable
COLOR_SPOOF = (255, 80, 255)      # Magenta - malicious
COLOR_GHOST = (150, 150, 255)     # Light red - ghost
COLOR_WHITE = (255, 255, 255)
COLOR_CYAN = (255, 200, 100)      # Cyan for observations
COLOR_DIM = (80, 80, 80)          # Dim agents

# Fonts
FONT = cv2.FONT_HERSHEY_SIMPLEX
FONT_SCALE_LARGE = 0.9
FONT_SCALE_MED = 0.6
FONT_SCALE_SMALL = 0.45

# World view settings
WORLD_SCALE = 12.0  # pixels per meter
WORLD_CENTER = (WORLD_PANE_WIDTH // 2, WORLD_PANE_HEIGHT // 2)

# Network view settings
NETWORK_CENTER = (NETWORK_PANE_WIDTH // 2, NETWORK_PANE_HEIGHT // 2)
NODE_RADIUS = 12
NODE_RING_RADIUS = 180

# Phases (in seconds)
PHASE_HOOK = (0, 5)
PHASE_BEFORE = (5, 25)
PHASE_AFTER = (25, 55)
PHASE_MONTAGE = (55, 75)
PHASE_CLOSE = (75, 90)

# Spotlight agents per beat
SPOTLIGHT_AGENTS = {
    "hook": ["agent_00", "agent_01", "agent_02"],
    "occlusion": ["agent_00", "agent_01"],
    "ghosts": ["agent_02", "agent_03"],
    "spoof": ["agent_04", "unknown_x"],
    "remote_obs": ["agent_00", "agent_01", "drone_00"],
    "merge": ["agent_02", "agent_03"],
    "trust_reject": ["agent_04"],
    "oosm": ["agent_07"],
    "space": ["agent_03", "drone_00"],
    "handoff": ["agent_05", "agent_06"],
    "bandwidth": ["agent_00"],
    "close": ["agent_00", "agent_01", "agent_02"],
}

# Captions per phase
CAPTIONS = {
    (0, 5): "20 agents. No video streaming. Shared world model from signed object packets.",
    (5, 12): "PROBLEM 1: Blind spots from occlusion",
    (12, 18): "PROBLEM 2: Ghost duplicates from multiple observers",
    (18, 25): "PROBLEM 3: Spoofed data looks identical",
    (25, 35): "GODVIEW: See around corners via packet sharing",
    (35, 45): "GODVIEW: Merge duplicates into one canonical track",
    (45, 55): "GODVIEW: Reject untrusted sources via provenance",
    (55, 60): "ENGINE: Out-of-sequence packets corrected",
    (60, 65): "ENGINE: 3D spatial indexing - drone at altitude",
    (65, 70): "ENGINE: Stable ID through tracking handoff",
    (70, 75): "ENGINE: 1000x less bandwidth than video",
    (75, 90): "godview_core: decentralized fusion with uncertainty + provenance",
}


# =============================================================================
# DATA LOADING
# =============================================================================

@dataclass
class FrameData:
    """All data needed to render a single frame."""
    frame: int
    timestamp_s: float
    phase: str
    beat: str
    
    # Detection packets for this frame
    packets: List[Dict]
    
    # Canonical state for this frame
    canonical_objects: List[Dict]
    
    # Events at this frame
    events: List[Dict]
    
    # Active packets in flight (for network animation)
    packets_in_flight: List[Dict]


class DataLoader:
    """Loads and indexes NDJSON files for fast frame lookup."""
    
    def __init__(self, out_dir: Path, fps: int):
        self.fps = fps
        
        # Load all data
        self.packets_by_frame: Dict[int, List[Dict]] = {}
        self.states_by_frame: Dict[int, Dict] = {}
        self.events_by_frame: Dict[int, List[Dict]] = {}
        self.all_packets: List[Dict] = []
        
        self._load_packets(out_dir / "packets.ndjson")
        self._load_states(out_dir / "world_state.ndjson")
        self._load_events(out_dir / "events.ndjson")
        
        self.max_frame = max(self.states_by_frame.keys()) if self.states_by_frame else 0
    
    def _load_packets(self, path: Path):
        """Load packets.ndjson."""
        print(f"Loading {path}...")
        with open(path) as f:
            for line in f:
                pkt = json.loads(line)
                frame = pkt["frame"]
                if frame not in self.packets_by_frame:
                    self.packets_by_frame[frame] = []
                self.packets_by_frame[frame].append(pkt)
                self.all_packets.append(pkt)
        print(f"  Loaded {len(self.all_packets)} packets")
    
    def _load_states(self, path: Path):
        """Load world_state.ndjson."""
        print(f"Loading {path}...")
        count = 0
        with open(path) as f:
            for line in f:
                state = json.loads(line)
                self.states_by_frame[state["frame"]] = state
                count += 1
        print(f"  Loaded {count} states")
    
    def _load_events(self, path: Path):
        """Load events.ndjson."""
        print(f"Loading {path}...")
        count = 0
        with open(path) as f:
            for line in f:
                evt = json.loads(line)
                frame = evt["frame"]
                if frame not in self.events_by_frame:
                    self.events_by_frame[frame] = []
                self.events_by_frame[frame].append(evt)
                count += 1
        print(f"  Loaded {count} events")
    
    def get_phase(self, frame: int) -> str:
        """Determine current phase."""
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
        if 5 <= t < 12:
            return "occlusion"
        elif 12 <= t < 18:
            return "ghosts"
        elif 18 <= t < 25:
            return "spoof"
        elif 25 <= t < 35:
            return "remote_obs"
        elif 35 <= t < 45:
            return "merge"
        elif 45 <= t < 55:
            return "trust_reject"
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
    
    def get_packets_in_flight(self, frame: int) -> List[Dict]:
        """Get packets currently traveling (frame < current <= delivery_frame)."""
        in_flight = []
        # Only check recent packets for performance
        start_check = max(0, frame - 30)
        for check_frame in range(start_check, frame + 1):
            for pkt in self.packets_by_frame.get(check_frame, []):
                if pkt["frame"] < frame <= pkt["delivery_frame"]:
                    in_flight.append(pkt)
        return in_flight
    
    def get_frame_data(self, frame: int) -> FrameData:
        """Get all data for a specific frame."""
        phase = self.get_phase(frame)
        beat = self.get_beat(frame)
        
        state = self.states_by_frame.get(frame, {"objects": []})
        
        return FrameData(
            frame=frame,
            timestamp_s=frame / self.fps,
            phase=phase,
            beat=beat,
            packets=self.packets_by_frame.get(frame, []),
            canonical_objects=state.get("objects", []),
            events=self.events_by_frame.get(frame, []),
            packets_in_flight=self.get_packets_in_flight(frame),
        )


# =============================================================================
# RENDERING
# =============================================================================

class DemoRenderer:
    """Renders demo frames using OpenCV."""
    
    def __init__(self, data: DataLoader, fps: int):
        self.data = data
        self.fps = fps
        
        # Agent positions for network view (circular layout)
        self.agent_positions: Dict[str, Tuple[int, int]] = {}
        self._init_agent_positions()
        
        # Recent events for event log
        self.recent_events: List[Tuple[int, str]] = []
    
    def _init_agent_positions(self):
        """Initialize agent node positions for network view."""
        agents = [f"agent_{i:02d}" for i in range(20)] + ["drone_00", "unknown_x"]
        for i, agent_id in enumerate(agents):
            angle = (2 * math.pi * i) / len(agents) - math.pi / 2
            x = int(NETWORK_CENTER[0] + NODE_RING_RADIUS * math.cos(angle))
            y = int(NETWORK_CENTER[1] + NODE_RING_RADIUS * math.sin(angle))
            self.agent_positions[agent_id] = (x, y)
    
    def world_to_screen(self, x: float, y: float) -> Tuple[int, int]:
        """Convert world coordinates to screen coordinates in world pane."""
        sx = int(WORLD_CENTER[0] + x * WORLD_SCALE)
        sy = int(WORLD_CENTER[1] - y * WORLD_SCALE)  # Flip Y
        return (sx, sy)
    
    def draw_grid(self, canvas: np.ndarray):
        """Draw background grid in world pane."""
        # Draw grid lines every 5 meters
        grid_spacing = int(5 * WORLD_SCALE)
        for x in range(0, WORLD_PANE_WIDTH, grid_spacing):
            cv2.line(canvas, (x, 0), (x, WORLD_PANE_HEIGHT), GRID_COLOR, 1)
        for y in range(0, WORLD_PANE_HEIGHT, grid_spacing):
            cv2.line(canvas, (0, y), (WORLD_PANE_WIDTH, y), GRID_COLOR, 1)
    
    def draw_box(self, canvas: np.ndarray, pos: Dict, yaw: float, 
                 color: Tuple[int, int, int], label: str = "", 
                 thickness: int = 2, size: float = 2.0):
        """Draw a rotated box representing an object."""
        cx, cy = self.world_to_screen(pos["x"], pos["y"])
        
        # Box dimensions in pixels
        w = int(size * WORLD_SCALE)
        h = int(size * 0.5 * WORLD_SCALE)
        
        # Rotated rectangle corners
        cos_a, sin_a = math.cos(yaw), math.sin(yaw)
        corners = [
            (-w/2, -h/2), (w/2, -h/2), (w/2, h/2), (-w/2, h/2)
        ]
        rotated = []
        for dx, dy in corners:
            rx = dx * cos_a - dy * sin_a
            ry = dx * sin_a + dy * cos_a
            rotated.append((int(cx + rx), int(cy - ry)))
        
        pts = np.array(rotated, np.int32).reshape((-1, 1, 2))
        cv2.polylines(canvas, [pts], True, color, thickness)
        
        # Direction indicator
        front_x = int(cx + (w/2) * cos_a)
        front_y = int(cy - (w/2) * sin_a)
        cv2.circle(canvas, (front_x, front_y), 3, color, -1)
        
        # Label
        if label:
            cv2.putText(canvas, label, (cx - 20, cy - 20), FONT, FONT_SCALE_SMALL, color, 1)
    
    def draw_covariance_ellipse(self, canvas: np.ndarray, pos: Dict, 
                                  cov: List[float], color: Tuple[int, int, int]):
        """Draw uncertainty ellipse from 2x2 covariance."""
        cx, cy = self.world_to_screen(pos["x"], pos["y"])
        
        # Covariance is [σxx, σxy, σyx, σyy]
        # Draw as ellipse with axes = 2*sqrt(eigenvalues)
        sigma_x = max(0.5, math.sqrt(abs(cov[0]))) * WORLD_SCALE * 2
        sigma_y = max(0.5, math.sqrt(abs(cov[3]))) * WORLD_SCALE * 2
        
        # Draw ellipse
        axes = (int(sigma_x), int(sigma_y))
        cv2.ellipse(canvas, (cx, cy), axes, 0, 0, 360, color, 1)
    
    def draw_drone(self, canvas: np.ndarray, pos: Dict, color: Tuple[int, int, int], label: str = ""):
        """Draw a drone as a triangle."""
        cx, cy = self.world_to_screen(pos["x"], pos["y"])
        size = 15
        
        pts = np.array([
            [cx, cy - size],
            [cx - size, cy + size],
            [cx + size, cy + size]
        ], np.int32).reshape((-1, 1, 2))
        
        cv2.polylines(canvas, [pts], True, color, 2)
        cv2.circle(canvas, (cx, cy), 3, color, -1)
        
        # Altitude indicator
        z = pos.get("z", 0)
        if z > 0:
            cv2.putText(canvas, f"Z:{z:.0f}m", (cx + 10, cy), FONT, FONT_SCALE_SMALL, color, 1)
        
        if label:
            cv2.putText(canvas, label, (cx - 20, cy - 25), FONT, FONT_SCALE_SMALL, color, 1)
    
    def render_world_pane(self, frame_data: FrameData) -> np.ndarray:
        """Render the world view pane."""
        canvas = np.full((WORLD_PANE_HEIGHT, WORLD_PANE_WIDTH, 3), BG_COLOR, dtype=np.uint8)
        self.draw_grid(canvas)
        
        # Determine what to draw based on phase
        phase = frame_data.phase
        beat = frame_data.beat
        spotlight = SPOTLIGHT_AGENTS.get(beat, SPOTLIGHT_AGENTS["hook"])
        
        # Draw detections from packets
        for pkt in frame_data.packets:
            agent_id = pkt["agent_id"]
            is_spotlight = agent_id in spotlight
            
            for obj in pkt["objects"]:
                pos = obj["pose"]["position"]
                yaw = obj["pose"]["yaw"]
                cov = obj["covariance"]
                obj_class = obj["class"]
                
                # Color based on phase and validity
                if not pkt["signature_valid"]:
                    color = COLOR_SPOOF if phase == "before" else COLOR_DIM
                    if phase != "before":
                        continue  # Don't show rejected spoofs
                elif phase == "before":
                    color = COLOR_RAW if is_spotlight else COLOR_DIM
                else:
                    color = COLOR_CYAN if is_spotlight else COLOR_DIM
                
                # Draw
                thickness = 2 if is_spotlight else 1
                if obj_class == "drone" or "drone" in agent_id:
                    self.draw_drone(canvas, pos, color)
                else:
                    self.draw_box(canvas, pos, yaw, color, thickness=thickness)
                
                if is_spotlight and phase in ["before", "after"]:
                    self.draw_covariance_ellipse(canvas, pos, cov, color)
        
        # Draw canonical objects (in AFTER phase)
        if phase in ["after", "montage", "close"]:
            for obj in frame_data.canonical_objects:
                pos = obj["pose"]["position"]
                yaw = obj["pose"]["yaw"]
                cov = obj["covariance"]
                
                # Thicker, brighter for fused objects
                self.draw_box(canvas, pos, yaw, COLOR_FUSED, thickness=3, size=2.2)
                self.draw_covariance_ellipse(canvas, pos, cov, COLOR_FUSED)
        
        # Draw phase/beat label
        cv2.putText(canvas, f"[{phase.upper()}] {beat}", (20, 30), 
                    FONT, FONT_SCALE_MED, COLOR_WHITE, 1)
        
        return canvas
    
    def render_network_pane(self, frame_data: FrameData) -> np.ndarray:
        """Render the network view pane."""
        canvas = np.full((NETWORK_PANE_HEIGHT, NETWORK_PANE_WIDTH, 3), BG_COLOR, dtype=np.uint8)
        
        phase = frame_data.phase
        beat = frame_data.beat
        spotlight = SPOTLIGHT_AGENTS.get(beat, [])
        
        # Draw edges (light connections between all nodes)
        for agent_id, pos in self.agent_positions.items():
            if agent_id == "unknown_x":
                continue
            # Connect to center
            cv2.line(canvas, pos, NETWORK_CENTER, GRID_COLOR, 1)
        
        # Draw nodes
        for agent_id, pos in self.agent_positions.items():
            is_spotlight = agent_id in spotlight
            is_spoof = agent_id == "unknown_x"
            
            if is_spoof:
                color = COLOR_SPOOF
            elif is_spotlight:
                color = COLOR_FUSED if phase in ["after", "montage", "close"] else COLOR_RAW
            else:
                color = COLOR_DIM
            
            radius = NODE_RADIUS + 4 if is_spotlight else NODE_RADIUS
            cv2.circle(canvas, pos, radius, color, -1 if is_spotlight else 2)
            
            # Label for spotlight
            if is_spotlight:
                label = agent_id.replace("agent_", "A").replace("drone_", "D").replace("unknown_", "?")
                cv2.putText(canvas, label, (pos[0] - 10, pos[1] + 30), 
                            FONT, FONT_SCALE_SMALL, color, 1)
        
        # Draw packets in flight
        for pkt in frame_data.packets_in_flight:
            src_id = pkt["agent_id"]
            if src_id not in self.agent_positions:
                continue
            
            src_pos = self.agent_positions[src_id]
            dst_pos = NETWORK_CENTER  # Packets travel to center (fusion)
            
            # Calculate position along path
            progress = (frame_data.frame - pkt["frame"]) / max(1, pkt["delivery_frame"] - pkt["frame"])
            progress = min(1.0, max(0.0, progress))
            
            px = int(src_pos[0] + (dst_pos[0] - src_pos[0]) * progress)
            py = int(src_pos[1] + (dst_pos[1] - src_pos[1]) * progress)
            
            # Color based on validity
            if not pkt["signature_valid"]:
                dot_color = COLOR_SPOOF
            else:
                dot_color = COLOR_CYAN
            
            cv2.circle(canvas, (px, py), 5, dot_color, -1)
        
        # Title
        cv2.putText(canvas, "NETWORK", (10, 25), FONT, FONT_SCALE_MED, COLOR_WHITE, 1)
        
        return canvas
    
    def render_data_pane(self, frame_data: FrameData) -> np.ndarray:
        """Render the Shared vs Not Shared pane + event log."""
        canvas = np.full((DATA_PANE_HEIGHT, DATA_PANE_WIDTH, 3), BG_COLOR, dtype=np.uint8)
        
        # Title
        cv2.putText(canvas, "DATA SCHEMA", (10, 25), FONT, FONT_SCALE_MED, COLOR_WHITE, 1)
        
        # Shared section
        y = 60
        cv2.putText(canvas, "SHARED:", (20, y), FONT, FONT_SCALE_MED, COLOR_FUSED, 1)
        y += 30
        shared_fields = ["class", "pose (x,y,z,yaw)", "covariance", "timestamp", "signature"]
        for field in shared_fields:
            cv2.putText(canvas, f"  [check] {field}", (20, y), FONT, FONT_SCALE_SMALL, COLOR_FUSED, 1)
            y += 22
        
        # Not shared section
        y += 20
        cv2.putText(canvas, "NOT SHARED:", (20, y), FONT, FONT_SCALE_MED, COLOR_RAW, 1)
        y += 30
        not_shared = ["camera frames", "LiDAR points", "video stream"]
        for field in not_shared:
            cv2.putText(canvas, f"  [X] {field}", (20, y), FONT, FONT_SCALE_SMALL, COLOR_RAW, 1)
            y += 22
        
        # Event log
        y += 40
        cv2.putText(canvas, "EVENTS:", (20, y), FONT, FONT_SCALE_MED, COLOR_WHITE, 1)
        y += 25
        
        # Update recent events
        for evt in frame_data.events:
            self.recent_events.append((frame_data.frame, evt["event_type"]))
        
        # Keep only last 5 events
        self.recent_events = self.recent_events[-5:]
        
        for evt_frame, evt_type in self.recent_events:
            age = frame_data.frame - evt_frame
            alpha = max(0.3, 1.0 - age / 90.0)  # Fade over 3 seconds
            color = tuple(int(c * alpha) for c in COLOR_WHITE)
            cv2.putText(canvas, f"  {evt_type}", (20, y), FONT, FONT_SCALE_SMALL, color, 1)
            y += 20
        
        return canvas
    
    def render_caption(self, canvas: np.ndarray, frame_data: FrameData):
        """Draw caption at bottom of frame."""
        t = frame_data.timestamp_s
        caption = ""
        for (start, end), text in CAPTIONS.items():
            if start <= t < end:
                caption = text
                break
        
        if caption:
            # Draw background bar
            cv2.rectangle(canvas, (0, HEIGHT - 80), (WIDTH, HEIGHT), (0, 0, 0), -1)
            
            # Draw text centered
            text_size = cv2.getTextSize(caption, FONT, FONT_SCALE_LARGE, 2)[0]
            x = (WIDTH - text_size[0]) // 2
            cv2.putText(canvas, caption, (x, HEIGHT - 30), FONT, FONT_SCALE_LARGE, COLOR_WHITE, 2)
    
    def render_frame(self, frame: int) -> np.ndarray:
        """Render a complete frame."""
        frame_data = self.data.get_frame_data(frame)
        
        # Create main canvas
        canvas = np.full((HEIGHT, WIDTH, 3), BG_COLOR, dtype=np.uint8)
        
        # Render panes
        world_pane = self.render_world_pane(frame_data)
        network_pane = self.render_network_pane(frame_data)
        data_pane = self.render_data_pane(frame_data)
        
        # Compose panes
        canvas[0:WORLD_PANE_HEIGHT, 0:WORLD_PANE_WIDTH] = world_pane
        canvas[0:NETWORK_PANE_HEIGHT, WORLD_PANE_WIDTH:WIDTH] = network_pane
        canvas[NETWORK_PANE_HEIGHT:HEIGHT, WORLD_PANE_WIDTH:WIDTH] = data_pane
        
        # Draw pane borders
        cv2.line(canvas, (WORLD_PANE_WIDTH, 0), (WORLD_PANE_WIDTH, HEIGHT), PANE_BORDER, 2)
        cv2.line(canvas, (WORLD_PANE_WIDTH, NETWORK_PANE_HEIGHT), (WIDTH, NETWORK_PANE_HEIGHT), PANE_BORDER, 2)
        
        # Draw caption
        self.render_caption(canvas, frame_data)
        
        # Draw frame counter
        cv2.putText(canvas, f"Frame: {frame}/{self.data.max_frame}", 
                    (WIDTH - 200, 30), FONT, FONT_SCALE_SMALL, COLOR_DIM, 1)
        
        return canvas


# =============================================================================
# MAIN
# =============================================================================

def main():
    parser = argparse.ArgumentParser(description="GodView Demo Frame Renderer")
    parser.add_argument("--out", type=str, default="./out", help="Output directory")
    parser.add_argument("--fps", type=int, default=30, help="Frames per second")
    args = parser.parse_args()
    
    out_dir = Path(args.out)
    frames_dir = out_dir / "frames"
    frames_dir.mkdir(parents=True, exist_ok=True)
    
    print("=" * 60)
    print("GodView Demo Frame Renderer")
    print("=" * 60)
    print(f"Output: {frames_dir}")
    print(f"FPS: {args.fps}")
    print("=" * 60)
    
    # Load data
    data = DataLoader(out_dir, args.fps)
    renderer = DemoRenderer(data, args.fps)
    
    # Render frames
    total_frames = data.max_frame + 1
    print(f"Rendering {total_frames} frames...")
    
    for frame in range(total_frames):
        img = renderer.render_frame(frame)
        path = frames_dir / f"frame_{frame:05d}.png"
        cv2.imwrite(str(path), img)
        
        if frame % (args.fps * 5) == 0:
            print(f"  Frame {frame}/{total_frames} ({frame/args.fps:.0f}s)")
    
    print(f"Rendered {total_frames} frames to {frames_dir}")


if __name__ == "__main__":
    main()
