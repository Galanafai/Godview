#!/usr/bin/env python3
"""
GodView Demo Fix - Enhanced Frame Renderer
===========================================
Renders frames with:
- 3-pane layout (World, Network, Data)
- Object budget (8-15 visible, ranked by importance)
- Causality arrows (packet arrival -> object appearance)
- Split-screen comparisons for key beats
- Occluder visualization
- Agent spotlight panel
- Event visualizations (merge pulse, trust reject shield, OOSM tag)

Output: out/frames/frame_XXXXX.png
"""

import argparse
import json
import math
import os
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple, Set

import cv2
import numpy as np

# =============================================================================
# CONFIGURATION
# =============================================================================

WIDTH = 1920
HEIGHT = 1080

# Pane layout
WORLD_PANE_W = 1280
WORLD_PANE_H = 1080
NETWORK_PANE_W = 640
NETWORK_PANE_H = 480
DATA_PANE_W = 640
DATA_PANE_H = 600

# Inset panel (inside world pane)
INSET_X = 20
INSET_Y = 700
INSET_W = 300
INSET_H = 200

# Agent panel (below network)
AGENT_PANEL_Y = NETWORK_PANE_H
AGENT_PANEL_H = 140

# Colors (BGR)
BG_COLOR = (12, 12, 12)
GRID_COLOR = (25, 25, 25)
PANE_BORDER = (40, 40, 40)

COLOR_BEFORE = (80, 80, 255)      # Red
COLOR_AFTER = (80, 255, 80)       # Green
COLOR_SPOOF = (255, 80, 255)      # Magenta
COLOR_GHOST = (180, 180, 255)     # Light red
COLOR_WHITE = (255, 255, 255)
COLOR_CYAN = (255, 220, 120)      # Cyan
COLOR_DIM = (60, 60, 60)
COLOR_YELLOW = (80, 255, 255)
COLOR_OCCLUDER = (50, 50, 80)

# Fonts
FONT = cv2.FONT_HERSHEY_SIMPLEX
FONT_BOLD = cv2.FONT_HERSHEY_DUPLEX

# World view
WORLD_SCALE = 14.0  # pixels per meter
WORLD_CENTER = (WORLD_PANE_W // 2, 450)

# Network view
NETWORK_CENTER = (NETWORK_PANE_W // 2, NETWORK_PANE_H // 2)
NODE_RADIUS = 10
NODE_RING_RADIUS = 160

# Object budget
MAX_OBJECTS_VISIBLE = 12
SPOTLIGHT_ZONE_RADIUS = 15.0  # meters

# Captions per beat
CAPTIONS = {
    "hook": "20 agents share object packets. No video streaming.",
    "occlusion": "PROBLEM: Agent B cannot see occluded pedestrian",
    "ghosts": "PROBLEM: Same object, different IDs = ghost duplicates",
    "spoof": "PROBLEM: Spoofed packet looks identical",
    "remote_obs": "SOLUTION: Packet arrives → pedestrian visible to B",
    "merge": "SOLUTION: Duplicates merge into one canonical track",
    "trust_reject": "SOLUTION: Invalid signature → packet rejected",
    "oosm": "ENGINE: Late packet (+12 frames) corrected",
    "space": "ENGINE: Drone at z=15m, car at z=0 → no pancake",
    "bandwidth": "1000× less bandwidth than video streaming",
    "close": "godview_core: decentralized fusion with provenance",
}


# =============================================================================
# DATA LOADING
# =============================================================================

@dataclass
class FrameData:
    frame: int
    timestamp_s: float
    phase: str
    beat: str
    beat_info: Dict
    packets: List[Dict]
    canonical_objects: List[Dict]
    events: List[Dict]
    all_events_up_to_now: List[Dict]
    is_split_screen: bool
    occluder: Dict


class DataLoader:
    def __init__(self, out_dir: Path, fps: int):
        self.fps = fps
        self.out_dir = out_dir

        self.packets_by_frame: Dict[int, List[Dict]] = {}
        self.states_by_frame: Dict[int, Dict] = {}
        self.events_by_frame: Dict[int, List[Dict]] = {}
        self.all_events: List[Dict] = []
        self.beats: Dict = {}
        self.occluder: Dict = {}

        self._load_all()
        self.max_frame = max(self.states_by_frame.keys()) if self.states_by_frame else 0

    def _load_all(self):
        # Load packets
        with open(self.out_dir / "packets.ndjson") as f:
            for line in f:
                pkt = json.loads(line)
                frame = pkt["frame"]
                if frame not in self.packets_by_frame:
                    self.packets_by_frame[frame] = []
                self.packets_by_frame[frame].append(pkt)

        # Load states
        with open(self.out_dir / "world_state.ndjson") as f:
            for line in f:
                state = json.loads(line)
                self.states_by_frame[state["frame"]] = state

        # Load events
        with open(self.out_dir / "events.ndjson") as f:
            for line in f:
                evt = json.loads(line)
                frame = evt["frame"]
                if frame not in self.events_by_frame:
                    self.events_by_frame[frame] = []
                self.events_by_frame[frame].append(evt)
                self.all_events.append(evt)

        # Load beats
        with open(self.out_dir / "beats.json") as f:
            self.beats = json.load(f)

        # Load occluder
        with open(self.out_dir / "occluder.json") as f:
            self.occluder = json.load(f)

        print(f"Loaded: {sum(len(v) for v in self.packets_by_frame.values())} packets, "
              f"{len(self.states_by_frame)} states, {len(self.all_events)} events")

    def get_phase(self, frame: int) -> str:
        t = frame / self.fps
        if 0 <= t < 5:
            return "hook"
        elif 5 <= t < 28:
            return "before"
        elif 28 <= t < 55:
            return "after"
        elif 55 <= t < 75:
            return "montage"
        else:
            return "close"

    def get_beat(self, frame: int) -> Tuple[str, Dict]:
        t = frame / self.fps
        for beat_name, beat_info in self.beats.items():
            if beat_info["start"] <= t < beat_info["end"]:
                return beat_name, beat_info
        return "general", {"start": 0, "end": 85, "spotlight": []}

    def get_frame_data(self, frame: int) -> FrameData:
        phase = self.get_phase(frame)
        beat_name, beat_info = self.get_beat(frame)
        t = frame / self.fps

        # Check if split screen active
        is_split = False
        if "split_screen" in beat_info:
            ss_start, ss_end = beat_info["split_screen"]
            is_split = ss_start <= t < ss_end

        state = self.states_by_frame.get(frame, {"objects": []})
        events_up_to_now = [e for e in self.all_events if e["frame"] <= frame]

        return FrameData(
            frame=frame,
            timestamp_s=t,
            phase=phase,
            beat=beat_name,
            beat_info=beat_info,
            packets=self.packets_by_frame.get(frame, []),
            canonical_objects=state.get("objects", []),
            events=self.events_by_frame.get(frame, []),
            all_events_up_to_now=events_up_to_now,
            is_split_screen=is_split,
            occluder=self.occluder,
        )


# =============================================================================
# RENDERER
# =============================================================================

class DemoRenderer:
    def __init__(self, data: DataLoader, fps: int):
        self.data = data
        self.fps = fps

        # Agent positions for network
        self.agent_positions: Dict[str, Tuple[int, int]] = {}
        self._init_agent_positions()

        # Colors per agent (consistent)
        self.agent_colors: Dict[str, Tuple[int, int, int]] = {}
        self._init_agent_colors()

        # Recent events for log
        self.recent_events: List[Tuple[int, str]] = []

        # Causality arrow state
        self.causality_arrows: List[Dict] = []

    def _init_agent_positions(self):
        agents = [f"agent_{i:02d}" for i in range(20)] + ["drone_00", "unknown_x"]
        for i, agent_id in enumerate(agents):
            angle = (2 * math.pi * i) / len(agents) - math.pi / 2
            x = int(NETWORK_CENTER[0] + NODE_RING_RADIUS * math.cos(angle))
            y = int(NETWORK_CENTER[1] + NODE_RING_RADIUS * math.sin(angle))
            self.agent_positions[agent_id] = (x, y)

    def _init_agent_colors(self):
        base_colors = [
            (255, 100, 100), (100, 255, 100), (100, 100, 255),
            (255, 255, 100), (255, 100, 255), (100, 255, 255),
        ]
        agents = [f"agent_{i:02d}" for i in range(20)] + ["drone_00"]
        for i, agent_id in enumerate(agents):
            self.agent_colors[agent_id] = base_colors[i % len(base_colors)]
        self.agent_colors["unknown_x"] = COLOR_SPOOF

    def world_to_screen(self, x: float, y: float) -> Tuple[int, int]:
        sx = int(WORLD_CENTER[0] + x * WORLD_SCALE)
        sy = int(WORLD_CENTER[1] - y * WORLD_SCALE)
        return (sx, sy)

    def draw_grid(self, canvas: np.ndarray):
        spacing = int(5 * WORLD_SCALE)
        for x in range(0, WORLD_PANE_W, spacing):
            cv2.line(canvas, (x, 0), (x, WORLD_PANE_H), GRID_COLOR, 1)
        for y in range(0, WORLD_PANE_H, spacing):
            cv2.line(canvas, (0, y), (WORLD_PANE_W, y), GRID_COLOR, 1)

    def draw_occluder(self, canvas: np.ndarray, occ: Dict):
        """Draw the building/occluder."""
        cx, cy = self.world_to_screen(occ["x"], occ["y"])
        w = int(occ["width"] * WORLD_SCALE)
        h = int(occ["height"] * WORLD_SCALE)
        cv2.rectangle(canvas, (cx - w//2, cy - h//2), (cx + w//2, cy + h//2), COLOR_OCCLUDER, -1)
        cv2.rectangle(canvas, (cx - w//2, cy - h//2), (cx + w//2, cy + h//2), (80, 80, 100), 2)
        cv2.putText(canvas, "BUILDING", (cx - 30, cy), FONT, 0.4, (100, 100, 120), 1)

    def draw_object_box(self, canvas: np.ndarray, obj: Dict, color: Tuple[int, int, int],
                        thickness: int = 2, label: str = "", size: float = 2.0):
        pos = obj["pose"]["position"]
        cx, cy = self.world_to_screen(pos["x"], pos["y"])
        yaw = obj["pose"].get("yaw", 0)

        w = int(size * WORLD_SCALE)
        h = int(size * 0.5 * WORLD_SCALE)

        cos_a, sin_a = math.cos(yaw), math.sin(yaw)
        corners = [(-w/2, -h/2), (w/2, -h/2), (w/2, h/2), (-w/2, h/2)]
        rotated = []
        for dx, dy in corners:
            rx = dx * cos_a - dy * sin_a
            ry = dx * sin_a + dy * cos_a
            rotated.append((int(cx + rx), int(cy - ry)))

        pts = np.array(rotated, np.int32).reshape((-1, 1, 2))
        cv2.polylines(canvas, [pts], True, color, thickness)

        if label:
            cv2.putText(canvas, label, (cx - 20, cy - 15), FONT, 0.35, color, 1)

    def draw_covariance(self, canvas: np.ndarray, obj: Dict, color: Tuple[int, int, int]):
        pos = obj["pose"]["position"]
        cx, cy = self.world_to_screen(pos["x"], pos["y"])
        cov = obj.get("covariance", [0.5, 0, 0, 0.5])

        sigma_x = max(0.3, math.sqrt(abs(cov[0]))) * WORLD_SCALE * 2
        sigma_y = max(0.3, math.sqrt(abs(cov[3]))) * WORLD_SCALE * 2

        cv2.ellipse(canvas, (cx, cy), (int(sigma_x), int(sigma_y)), 0, 0, 360, color, 1)

    def rank_objects(self, objects: List[Dict], spotlight_agents: List[str]) -> List[Dict]:
        """Rank objects by importance and return top N."""
        def importance(obj):
            pos = obj["pose"]["position"]
            dist_to_center = math.sqrt(pos["x"]**2 + pos["y"]**2)

            # Higher score = more important
            score = 100 - dist_to_center

            # Boost if in spotlight zone
            if dist_to_center < SPOTLIGHT_ZONE_RADIUS:
                score += 50

            # Boost pedestrians/cyclists (more interesting)
            if obj.get("class") in ["pedestrian", "cyclist"]:
                score += 20

            return score

        ranked = sorted(objects, key=importance, reverse=True)
        return ranked[:MAX_OBJECTS_VISIBLE]

    def draw_causality_arrow(self, canvas: np.ndarray, from_pos: Tuple[int, int],
                              to_pos: Tuple[int, int], label: str, progress: float):
        """Draw animated arrow from network to world."""
        if progress <= 0 or progress > 1:
            return

        # Interpolate arrow head position
        fx, fy = from_pos
        tx, ty = to_pos
        hx = int(fx + (tx - fx) * progress)
        hy = int(fy + (ty - fy) * progress)

        # Draw line
        alpha = min(1.0, progress * 2)
        color = tuple(int(c * alpha) for c in COLOR_CYAN)
        cv2.line(canvas, from_pos, (hx, hy), color, 2)

        # Draw arrowhead
        cv2.circle(canvas, (hx, hy), 6, COLOR_CYAN, -1)

        # Label at midpoint
        mx, my = (fx + tx) // 2, (fy + ty) // 2
        cv2.putText(canvas, label, (mx - 80, my - 10), FONT, 0.4, COLOR_WHITE, 1)

    def render_world_pane(self, fd: FrameData) -> np.ndarray:
        canvas = np.full((WORLD_PANE_H, WORLD_PANE_W, 3), BG_COLOR, dtype=np.uint8)
        self.draw_grid(canvas)

        phase = fd.phase
        beat = fd.beat
        spotlight = fd.beat_info.get("spotlight", [])

        # Draw occluder
        self.draw_occluder(canvas, fd.occluder)

        # During occlusion beat, show truth silhouette
        if beat == "occlusion" and phase == "before":
            # Show faint pedestrian position
            silhouette_pos = {"x": 12, "y": 8}  # ped_01 approximate position
            sx, sy = self.world_to_screen(silhouette_pos["x"], silhouette_pos["y"])
            cv2.circle(canvas, (sx, sy), 10, (40, 40, 40), 2)
            cv2.putText(canvas, "OCCLUDED", (sx - 30, sy - 15), FONT, 0.35, (60, 60, 60), 1)

        # Collect and rank objects to display
        if phase in ["after", "montage", "close"]:
            # Show canonical objects
            objects_to_draw = self.rank_objects(fd.canonical_objects, spotlight)
            color = COLOR_AFTER
            thickness = 3
        else:
            # Show raw detections
            all_detections = []
            for pkt in fd.packets:
                for obj in pkt["objects"]:
                    obj["_agent"] = pkt["agent_id"]
                    obj["_valid"] = pkt["signature_valid"]
                    all_detections.append(obj)
            objects_to_draw = self.rank_objects(all_detections, spotlight)[:MAX_OBJECTS_VISIBLE]
            color = COLOR_BEFORE
            thickness = 2

        # Draw objects
        for obj in objects_to_draw:
            obj_id = obj.get("canonical_object_id", obj.get("local_object_id", ""))
            agent = obj.get("_agent", "")
            is_spotlight_agent = agent in spotlight

            # Determine color
            if not obj.get("_valid", True):
                draw_color = COLOR_SPOOF
                label = "SPOOF"
            elif is_spotlight_agent or phase in ["after", "montage", "close"]:
                draw_color = color
                label = obj.get("class", "")[:3].upper()
            else:
                draw_color = COLOR_DIM
                label = ""

            self.draw_object_box(canvas, obj, draw_color, thickness, label)
            if is_spotlight_agent or phase in ["after", "montage", "close"]:
                self.draw_covariance(canvas, obj, draw_color)

        # Draw agents as small icons
        for pkt in fd.packets:
            if pkt["agent_id"] in self.agent_positions:
                # Get agent position from first object or estimate
                if pkt["objects"]:
                    # Estimate agent position
                    pass

        # Phase/beat label
        phase_label = f"[{phase.upper()}] {beat}"
        cv2.putText(canvas, phase_label, (20, 30), FONT, 0.6, COLOR_WHITE, 1)

        # Draw 3D inset placeholder
        cv2.rectangle(canvas, (INSET_X, INSET_Y), (INSET_X + INSET_W, INSET_Y + INSET_H), (30, 30, 30), -1)
        cv2.rectangle(canvas, (INSET_X, INSET_Y), (INSET_X + INSET_W, INSET_Y + INSET_H), (60, 60, 60), 2)
        cv2.putText(canvas, "3D INSET (local sensors)", (INSET_X + 10, INSET_Y + 20), FONT, 0.4, (80, 80, 80), 1)
        cv2.putText(canvas, "LiDAR stays local", (INSET_X + 30, INSET_Y + INSET_H // 2), FONT, 0.5, (100, 100, 100), 1)

        return canvas

    def render_network_pane(self, fd: FrameData) -> np.ndarray:
        canvas = np.full((NETWORK_PANE_H, NETWORK_PANE_W, 3), BG_COLOR, dtype=np.uint8)

        phase = fd.phase
        spotlight = fd.beat_info.get("spotlight", [])

        # Draw connections
        for agent_id, pos in self.agent_positions.items():
            if agent_id != "unknown_x":
                cv2.line(canvas, pos, NETWORK_CENTER, GRID_COLOR, 1)

        # Draw nodes
        for agent_id, pos in self.agent_positions.items():
            is_spotlight = agent_id in spotlight
            is_spoof = agent_id == "unknown_x"

            if is_spoof:
                color = COLOR_SPOOF
            elif is_spotlight:
                color = COLOR_AFTER if phase in ["after", "montage", "close"] else COLOR_BEFORE
            else:
                color = COLOR_DIM

            radius = NODE_RADIUS + 4 if is_spotlight else NODE_RADIUS
            cv2.circle(canvas, pos, radius, color, -1 if is_spotlight else 2)

            if is_spotlight:
                label = agent_id.replace("agent_", "A").replace("drone_", "D")[:3]
                cv2.putText(canvas, label, (pos[0] - 8, pos[1] + 25), FONT, 0.35, color, 1)

        # Draw packets in flight
        for pkt in fd.packets:
            src_id = pkt["agent_id"]
            if src_id not in self.agent_positions:
                continue

            if pkt["frame"] < fd.frame <= pkt["delivery_frame"]:
                src_pos = self.agent_positions[src_id]
                progress = (fd.frame - pkt["frame"]) / max(1, pkt["delivery_frame"] - pkt["frame"])
                progress = min(1.0, max(0.0, progress))

                px = int(src_pos[0] + (NETWORK_CENTER[0] - src_pos[0]) * progress)
                py = int(src_pos[1] + (NETWORK_CENTER[1] - src_pos[1]) * progress)

                dot_color = COLOR_SPOOF if not pkt["signature_valid"] else COLOR_CYAN
                cv2.circle(canvas, (px, py), 5, dot_color, -1)

        # Event visualization: Trust reject shield
        if fd.beat == "trust_reject" and phase in ["after", "montage"]:
            # Draw shield at center
            cv2.circle(canvas, NETWORK_CENTER, 30, COLOR_AFTER, 3)
            cv2.putText(canvas, "SHIELD", (NETWORK_CENTER[0] - 25, NETWORK_CENTER[1] + 50),
                        FONT, 0.4, COLOR_AFTER, 1)

        # Title
        cv2.putText(canvas, "NETWORK", (10, 25), FONT, 0.5, COLOR_WHITE, 1)

        return canvas

    def render_data_pane(self, fd: FrameData) -> np.ndarray:
        canvas = np.full((DATA_PANE_H, DATA_PANE_W, 3), BG_COLOR, dtype=np.uint8)

        # Title
        cv2.putText(canvas, "SHARED vs NOT SHARED", (10, 25), FONT, 0.5, COLOR_WHITE, 1)

        # Shared section
        y = 60
        cv2.putText(canvas, "SHARED:", (20, y), FONT, 0.5, COLOR_AFTER, 1)
        y += 28
        shared = ["class (car, ped...)", "pose (x, y, z, yaw)", "covariance (2x2)",
                  "timestamp", "signature"]
        for field in shared:
            cv2.putText(canvas, f"  [check] {field}", (20, y), FONT, 0.4, COLOR_AFTER, 1)
            y += 22

        # Not shared
        y += 20
        cv2.putText(canvas, "NOT SHARED:", (20, y), FONT, 0.5, COLOR_BEFORE, 1)
        y += 28
        not_shared = ["camera frames", "LiDAR point cloud", "video stream"]
        for field in not_shared:
            cv2.putText(canvas, f"  [X] {field}", (20, y), FONT, 0.4, COLOR_BEFORE, 1)
            y += 22

        # Event log
        y += 30
        cv2.putText(canvas, "EVENTS:", (20, y), FONT, 0.5, COLOR_WHITE, 1)
        y += 25

        for evt in fd.events:
            self.recent_events.append((fd.frame, evt["event_type"]))
        self.recent_events = self.recent_events[-4:]

        for evt_frame, evt_type in self.recent_events:
            age = fd.frame - evt_frame
            alpha = max(0.3, 1.0 - age / 60.0)
            color = tuple(int(c * alpha) for c in COLOR_YELLOW)
            cv2.putText(canvas, f"  {evt_type}", (20, y), FONT, 0.4, color, 1)
            y += 20

        # Agent spotlight panel
        y += 30
        spotlight = fd.beat_info.get("spotlight", [])
        if spotlight:
            cv2.putText(canvas, "SPOTLIGHT:", (20, y), FONT, 0.45, COLOR_CYAN, 1)
            y += 22
            for agent in spotlight[:3]:
                cv2.putText(canvas, f"  {agent}", (20, y), FONT, 0.35, COLOR_CYAN, 1)
                y += 18

        return canvas

    def render_caption(self, canvas: np.ndarray, fd: FrameData):
        caption = CAPTIONS.get(fd.beat, "")
        if not caption:
            return

        # Background bar
        cv2.rectangle(canvas, (0, HEIGHT - 80), (WIDTH, HEIGHT), (0, 0, 0), -1)

        # Text centered
        text_size = cv2.getTextSize(caption, FONT_BOLD, 0.8, 2)[0]
        x = (WIDTH - text_size[0]) // 2
        cv2.putText(canvas, caption, (x, HEIGHT - 30), FONT_BOLD, 0.8, COLOR_WHITE, 2)

    def render_split_screen(self, canvas: np.ndarray, fd: FrameData, before_canvas: np.ndarray):
        """Render split-screen comparison."""
        # Left half: BEFORE
        # Right half: AFTER (current)

        mid = WIDTH // 2

        # Draw BEFORE on left
        before_scaled = cv2.resize(before_canvas, (mid, HEIGHT))
        canvas[:, :mid] = before_scaled

        # Red banner
        cv2.rectangle(canvas, (0, 0), (mid, 50), COLOR_BEFORE, -1)
        cv2.putText(canvas, "BEFORE", (mid // 2 - 50, 35), FONT_BOLD, 1.0, COLOR_WHITE, 2)

        # Green banner on right
        cv2.rectangle(canvas, (mid, 0), (WIDTH, 50), COLOR_AFTER, -1)
        cv2.putText(canvas, "AFTER", (mid + mid // 2 - 40, 35), FONT_BOLD, 1.0, COLOR_WHITE, 2)

        # Divider
        cv2.line(canvas, (mid, 0), (mid, HEIGHT), COLOR_WHITE, 3)

    def render_before_snapshot(self, fd: FrameData) -> np.ndarray:
        """Render a BEFORE version for split-screen."""
        # Create a simple before version
        canvas = np.full((HEIGHT, WIDTH, 3), BG_COLOR, dtype=np.uint8)

        # Simplified world pane for BEFORE
        world = np.full((WORLD_PANE_H, WORLD_PANE_W, 3), BG_COLOR, dtype=np.uint8)
        self.draw_grid(world)
        self.draw_occluder(world, fd.occluder)

        # Show chaos: more boxes, jitter
        for pkt in fd.packets[:5]:
            for obj in pkt["objects"][:3]:
                self.draw_object_box(world, obj, COLOR_BEFORE, 2, "?")

        canvas[:WORLD_PANE_H, :WORLD_PANE_W] = world
        return canvas

    def render_frame(self, frame: int) -> np.ndarray:
        fd = self.data.get_frame_data(frame)

        canvas = np.full((HEIGHT, WIDTH, 3), BG_COLOR, dtype=np.uint8)

        # Render panes
        world_pane = self.render_world_pane(fd)
        network_pane = self.render_network_pane(fd)
        data_pane = self.render_data_pane(fd)

        # Compose
        canvas[:WORLD_PANE_H, :WORLD_PANE_W] = world_pane
        canvas[:NETWORK_PANE_H, WORLD_PANE_W:] = network_pane
        canvas[NETWORK_PANE_H:, WORLD_PANE_W:] = data_pane[:HEIGHT - NETWORK_PANE_H, :]

        # Pane borders
        cv2.line(canvas, (WORLD_PANE_W, 0), (WORLD_PANE_W, HEIGHT), PANE_BORDER, 2)
        cv2.line(canvas, (WORLD_PANE_W, NETWORK_PANE_H), (WIDTH, NETWORK_PANE_H), PANE_BORDER, 2)

        # Caption
        self.render_caption(canvas, fd)

        # Split-screen for key beats
        if fd.is_split_screen:
            before_canvas = self.render_before_snapshot(fd)
            self.render_split_screen(canvas, fd, before_canvas)

        # Frame counter
        cv2.putText(canvas, f"Frame: {frame}", (WIDTH - 120, 25), FONT, 0.4, COLOR_DIM, 1)

        return canvas


# =============================================================================
# MAIN
# =============================================================================

def main():
    parser = argparse.ArgumentParser(description="GodView Demo Fix - Frame Renderer")
    parser.add_argument("--out", type=str, default="./out", help="Output directory")
    parser.add_argument("--fps", type=int, default=30, help="FPS")
    args = parser.parse_args()

    out_dir = Path(args.out)
    frames_dir = out_dir / "frames"
    frames_dir.mkdir(parents=True, exist_ok=True)

    print("=" * 60)
    print("GodView Demo Fix - Frame Renderer")
    print("=" * 60)

    data = DataLoader(out_dir, args.fps)
    renderer = DemoRenderer(data, args.fps)

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
