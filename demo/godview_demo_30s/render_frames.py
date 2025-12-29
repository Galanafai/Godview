#!/usr/bin/env python3
"""
GodView 30s Demo - Frame Renderer
==================================
Renders 900 frames (30s @ 30fps) with:
- 3-pane layout
- Split-screen for BEFORE vs AFTER
- Causality arrows on packet arrival
- Stable dot+ring visualization
"""

import argparse
import json
import math
from pathlib import Path
from typing import Dict, List, Tuple

import cv2
import numpy as np

# =============================================================================
# CONFIGURATION
# =============================================================================

WIDTH = 1920
HEIGHT = 1080
FPS = 30

# Pane layout
WORLD_W = 1280
WORLD_H = 1080
NETWORK_W = 640
NETWORK_H = 450
DATA_W = 640
DATA_H = 630

# World view
WORLD_SCALE = 18.0
WORLD_CENTER = (WORLD_W // 2, WORLD_H // 2)

# Colors (BGR)
BG = (15, 15, 15)
GRID = (30, 30, 30)
BORDER = (50, 50, 50)

COLOR_A = (255, 150, 100)       # Blue for agent_A
COLOR_B = (100, 180, 255)       # Orange for agent_B
COLOR_DRONE = (100, 255, 150)   # Green for drone
COLOR_SPOOF = (255, 100, 255)   # Magenta
COLOR_BEFORE = (100, 100, 255)  # Red
COLOR_AFTER = (100, 255, 100)   # Green
COLOR_WHITE = (255, 255, 255)
COLOR_DIM = (80, 80, 80)
COLOR_BUILDING = (60, 60, 90)
COLOR_PED = (255, 200, 100)     # Cyan for pedestrian

FONT = cv2.FONT_HERSHEY_SIMPLEX
FONT_BOLD = cv2.FONT_HERSHEY_DUPLEX

# Beats
BEAT_THESIS = (0, 90)
BEAT_OCCLUSION = (90, 360)
BEAT_MERGE = (360, 600)
BEAT_TRUST = (600, 780)
BEAT_SCALE = (780, 900)

# Captions
CAPTIONS = {
    BEAT_THESIS: "No video streaming. Just object packets.",
    BEAT_OCCLUSION: "Packet arrival → pedestrian visible to B",
    BEAT_MERGE: "Duplicates merge into one canonical track",
    BEAT_TRUST: "Invalid signature → packet REJECTED",
    BEAT_SCALE: "Same mechanism scales to 20+ agents",
}


# =============================================================================
# DATA LOADING
# =============================================================================

class DataLoader:
    def __init__(self, out_dir: Path):
        self.out_dir = out_dir

        # Load all data
        self.world_before = self._load_ndjson("world_before.ndjson")
        self.world_after = self._load_ndjson("world_after.ndjson")
        self.packets_before = self._load_ndjson("packets_before.ndjson")
        self.packets_after = self._load_ndjson("packets_after.ndjson")
        self.events_after = self._load_ndjson("events_after.ndjson")

        with open(out_dir / "config.json") as f:
            self.config = json.load(f)

        # Index by frame
        self.world_before_by_frame = {w["frame"]: w for w in self.world_before}
        self.world_after_by_frame = {w["frame"]: w for w in self.world_after}
        self.events_by_frame = {}
        for e in self.events_after:
            f = e["frame"]
            if f not in self.events_by_frame:
                self.events_by_frame[f] = []
            self.events_by_frame[f].append(e)

        print(f"Loaded {len(self.world_before)} before states, {len(self.world_after)} after states")

    def _load_ndjson(self, name: str) -> List[Dict]:
        path = self.out_dir / name
        data = []
        with open(path) as f:
            for line in f:
                data.append(json.loads(line))
        return data


# =============================================================================
# RENDERER
# =============================================================================

class Renderer:
    def __init__(self, data: DataLoader):
        self.data = data
        self.config = data.config

        # Agent positions for network view
        self.agent_net_pos = {
            "agent_A": (200, 150),
            "agent_B": (440, 150),
            "drone": (320, 280),
            "unknown_node": (320, 380),
        }
        # Add scale agents
        for i in range(3, 20):
            angle = (i - 3) * 0.4
            x = 320 + int(150 * math.cos(angle))
            y = 220 + int(100 * math.sin(angle))
            self.agent_net_pos[f"agent_{i:02d}"] = (x, y)

        # Smoothed positions for stable rendering
        self.smoothed_pos: Dict[str, Tuple[float, float]] = {}

    def w2s(self, x: float, y: float) -> Tuple[int, int]:
        """World to screen coordinates."""
        sx = int(WORLD_CENTER[0] + x * WORLD_SCALE)
        sy = int(WORLD_CENTER[1] - y * WORLD_SCALE)
        return (sx, sy)

    def get_beat(self, frame: int) -> str:
        if BEAT_THESIS[0] <= frame < BEAT_THESIS[1]:
            return "thesis"
        elif BEAT_OCCLUSION[0] <= frame < BEAT_OCCLUSION[1]:
            return "occlusion"
        elif BEAT_MERGE[0] <= frame < BEAT_MERGE[1]:
            return "merge"
        elif BEAT_TRUST[0] <= frame < BEAT_TRUST[1]:
            return "trust"
        else:
            return "scale"

    def is_split_screen(self, frame: int) -> bool:
        """Split screen for occlusion, merge, trust beats."""
        beat = self.get_beat(frame)
        return beat in ["occlusion", "merge", "trust"]

    def draw_grid(self, canvas: np.ndarray, offset_x: int = 0, scale: float = 1.0):
        """Draw world grid."""
        w = int(WORLD_W * scale)
        h = int(WORLD_H * scale)
        spacing = int(5 * WORLD_SCALE * scale)
        for x in range(0, w, spacing):
            cv2.line(canvas, (offset_x + x, 0), (offset_x + x, h), GRID, 1)
        for y in range(0, h, spacing):
            cv2.line(canvas, (offset_x, y), (offset_x + w, y), GRID, 1)

    def draw_building(self, canvas: np.ndarray, offset_x: int = 0, scale: float = 1.0):
        """Draw building occluder."""
        b = self.config["building"]
        cx = int(WORLD_CENTER[0] * scale + offset_x + b["x"] * WORLD_SCALE * scale)
        cy = int(WORLD_CENTER[1] * scale - b["y"] * WORLD_SCALE * scale)
        w = int(b["width"] * WORLD_SCALE * scale)
        h = int(b["height"] * WORLD_SCALE * scale)

        cv2.rectangle(canvas, (cx - w//2, cy - h//2), (cx + w//2, cy + h//2), COLOR_BUILDING, -1)
        cv2.rectangle(canvas, (cx - w//2, cy - h//2), (cx + w//2, cy + h//2), (100, 100, 130), 2)
        cv2.putText(canvas, "BUILDING", (cx - 35, cy), FONT, 0.4 * scale, (120, 120, 150), 1)

    def draw_agent(self, canvas: np.ndarray, agent: Dict, offset_x: int = 0, scale: float = 1.0):
        """Draw agent icon."""
        x, y = agent["x"], agent["y"]
        sx, sy = int(WORLD_CENTER[0] * scale + offset_x + x * WORLD_SCALE * scale), \
                 int(WORLD_CENTER[1] * scale - y * WORLD_SCALE * scale)

        color = {"agent_A": COLOR_A, "agent_B": COLOR_B, "drone": COLOR_DRONE}.get(agent["id"], COLOR_DIM)

        if agent["id"] == "drone":
            # Triangle for drone
            pts = np.array([[sx, sy - 12], [sx - 10, sy + 8], [sx + 10, sy + 8]], np.int32)
            cv2.polylines(canvas, [pts], True, color, 2)
        else:
            # Circle for car
            cv2.circle(canvas, (sx, sy), int(10 * scale), color, -1)

        cv2.putText(canvas, agent["id"][-1], (sx - 4, sy + 4), FONT, 0.4 * scale, (0, 0, 0), 1)

    def draw_object(self, canvas: np.ndarray, obj: Dict, is_fused: bool,
                    offset_x: int = 0, scale: float = 1.0, label: str = ""):
        """Draw object as dot + uncertainty ring (before) or box (after)."""
        pos = obj["pose"]["position"]
        cov = obj.get("covariance", [0.3, 0, 0, 0.3])

        sx = int(WORLD_CENTER[0] * scale + offset_x + pos["x"] * WORLD_SCALE * scale)
        sy = int(WORLD_CENTER[1] * scale - pos["y"] * WORLD_SCALE * scale)

        # Determine color
        obj_id = obj.get("canonical_object_id", "")
        if "SPOOF" in obj_id:
            color = COLOR_SPOOF
        elif is_fused:
            color = COLOR_AFTER
        else:
            color = COLOR_BEFORE

        if is_fused:
            # Box for fused
            w = int(1.5 * WORLD_SCALE * scale)
            h = int(0.8 * WORLD_SCALE * scale)
            cv2.rectangle(canvas, (sx - w//2, sy - h//2), (sx + w//2, sy + h//2), color, 2)
        else:
            # Dot for raw
            cv2.circle(canvas, (sx, sy), int(6 * scale), color, -1)

        # Uncertainty ring
        sigma = max(0.2, math.sqrt(abs(cov[0]))) * WORLD_SCALE * scale
        cv2.ellipse(canvas, (sx, sy), (int(sigma), int(sigma)), 0, 0, 360, color, 1)

        if label:
            cv2.putText(canvas, label, (sx + 10, sy - 5), FONT, 0.35 * scale, color, 1)

    def draw_truth_silhouette(self, canvas: np.ndarray, offset_x: int = 0, scale: float = 1.0):
        """Draw faint silhouette of pedestrian (truth position)."""
        ped = self.config["hero_ped"]
        sx = int(WORLD_CENTER[0] * scale + offset_x + ped["x"] * WORLD_SCALE * scale)
        sy = int(WORLD_CENTER[1] * scale - ped["y"] * WORLD_SCALE * scale)

        cv2.circle(canvas, (sx, sy), int(8 * scale), (50, 50, 50), 2)  # Faint
        cv2.putText(canvas, "P (truth)", (sx + 10, sy), FONT, 0.3 * scale, (60, 60, 60), 1)

    def draw_causality_arrow(self, canvas: np.ndarray, frame: int,
                              offset_x: int = 0, scale: float = 1.0):
        """Draw packet arrival arrow."""
        events = self.data.events_by_frame.get(frame, [])
        for evt in events:
            if evt["event_type"] == "PACKET_ARRIVAL":
                # Arrow from network area to pedestrian
                ped = self.config["hero_ped"]
                sx = int(WORLD_CENTER[0] * scale + offset_x + ped["x"] * WORLD_SCALE * scale)
                sy = int(WORLD_CENTER[1] * scale - ped["y"] * WORLD_SCALE * scale)

                # Start from top-right (network area)
                start = (int(WORLD_W * scale + offset_x - 50), 100)

                cv2.arrowedLine(canvas, start, (sx, sy), COLOR_AFTER, 3, tipLength=0.1)
                cv2.putText(canvas, "PACKET ARRIVAL", (start[0] - 120, start[1] - 20),
                            FONT, 0.5 * scale, COLOR_AFTER, 1)

    def render_world_half(self, frame: int, is_after: bool,
                           canvas: np.ndarray, offset_x: int, width: int):
        """Render one half of world view."""
        scale = width / WORLD_W

        # Grid
        self.draw_grid(canvas, offset_x, scale)

        # Building
        self.draw_building(canvas, offset_x, scale)

        # Agents
        for agent in [self.config["agent_a"], self.config["agent_b"]]:
            self.draw_agent(canvas, agent, offset_x, scale)
        self.draw_agent(canvas, self.config["drone"], offset_x, scale)

        # Objects
        if is_after:
            state = self.data.world_after_by_frame.get(frame, {"objects": []})
            for obj in state["objects"]:
                label = "FUSED" if len(obj.get("source_agents", [])) > 1 else ""
                self.draw_object(canvas, obj, True, offset_x, scale, label)

            # Causality arrow
            self.draw_causality_arrow(canvas, frame, offset_x, scale)
        else:
            state = self.data.world_before_by_frame.get(frame, {"objects": []})
            for i, obj in enumerate(state["objects"]):
                label = f"ID:{i}" if "SPOOF" not in obj.get("canonical_object_id", "") else "SPOOF"
                self.draw_object(canvas, obj, False, offset_x, scale, label)

        # Truth silhouette
        self.draw_truth_silhouette(canvas, offset_x, scale)

        # Banner
        banner_h = 40
        banner_color = COLOR_AFTER if is_after else COLOR_BEFORE
        cv2.rectangle(canvas, (offset_x, 0), (offset_x + width, banner_h), banner_color, -1)
        text = "AFTER" if is_after else "BEFORE"
        cv2.putText(canvas, text, (offset_x + width//2 - 40, 28), FONT_BOLD, 0.8, COLOR_WHITE, 2)

    def render_network_pane(self, frame: int) -> np.ndarray:
        """Render network view."""
        canvas = np.full((NETWORK_H, NETWORK_W, 3), BG, dtype=np.uint8)

        beat = self.get_beat(frame)

        # Draw connections
        center = (NETWORK_W // 2, NETWORK_H // 2 - 40)
        for agent_id, pos in self.agent_net_pos.items():
            cv2.line(canvas, pos, center, GRID, 1)

        # Draw nodes
        active = {"agent_A", "agent_B", "drone"}
        if beat == "trust":
            active.add("unknown_node")
        if beat == "scale":
            active.update(f"agent_{i:02d}" for i in range(3, 20))

        for agent_id, pos in self.agent_net_pos.items():
            if agent_id == "unknown_node":
                color = COLOR_SPOOF
            elif agent_id == "agent_A":
                color = COLOR_A
            elif agent_id == "agent_B":
                color = COLOR_B
            elif agent_id == "drone":
                color = COLOR_DRONE
            else:
                color = COLOR_DIM if agent_id not in active else COLOR_AFTER

            is_active = agent_id in active
            cv2.circle(canvas, pos, 12 if is_active else 8, color, -1 if is_active else 2)

            if is_active and agent_id in ["agent_A", "agent_B", "drone", "unknown_node"]:
                label = agent_id.split("_")[-1][0].upper()
                cv2.putText(canvas, label, (pos[0] - 4, pos[1] + 4), FONT, 0.4, (0, 0, 0), 1)

        # Packet animation
        if beat == "occlusion" and 150 <= frame < 250:
            progress = (frame - 150) / 100.0
            src = self.agent_net_pos["agent_A"]
            dst = self.agent_net_pos["agent_B"]
            px = int(src[0] + (dst[0] - src[0]) * progress)
            py = int(src[1] + (dst[1] - src[1]) * progress)
            cv2.circle(canvas, (px, py), 8, COLOR_PED, -1)

        if beat == "trust" and 650 <= frame < 700:
            progress = (frame - 650) / 50.0
            src = self.agent_net_pos["unknown_node"]
            dst = center
            px = int(src[0] + (dst[0] - src[0]) * progress)
            py = int(src[1] + (dst[1] - src[1]) * progress)

            if progress < 0.8:
                cv2.circle(canvas, (px, py), 8, COLOR_SPOOF, -1)
            else:
                # Shield
                cv2.circle(canvas, center, 25, COLOR_AFTER, 3)
                cv2.putText(canvas, "X", (center[0] - 8, center[1] + 8), FONT_BOLD, 0.8, COLOR_AFTER, 2)

        cv2.putText(canvas, "NETWORK", (10, 25), FONT, 0.5, COLOR_WHITE, 1)
        return canvas

    def render_data_pane(self, frame: int) -> np.ndarray:
        """Render shared vs not shared pane."""
        canvas = np.full((DATA_H, DATA_W, 3), BG, dtype=np.uint8)

        y = 30
        cv2.putText(canvas, "SHARED vs NOT SHARED", (10, y), FONT, 0.6, COLOR_WHITE, 1)

        y += 50
        cv2.putText(canvas, "OBJECT PACKETS:", (20, y), FONT, 0.55, COLOR_AFTER, 1)
        y += 35
        shared = ["  [check] class (pedestrian/car)",
                  "  [check] pose (x, y, z, yaw)",
                  "  [check] covariance (2x2)",
                  "  [check] timestamp",
                  "  [check] signature (trust)"]
        for item in shared:
            cv2.putText(canvas, item, (20, y), FONT, 0.45, COLOR_AFTER, 1)
            y += 28

        y += 30
        cv2.putText(canvas, "NOT SHARED:", (20, y), FONT, 0.55, COLOR_BEFORE, 1)
        y += 35
        not_shared = ["  [X] camera frames",
                      "  [X] LiDAR point cloud",
                      "  [X] video stream"]
        for item in not_shared:
            cv2.putText(canvas, item, (20, y), FONT, 0.45, COLOR_BEFORE, 1)
            y += 28

        # Event display
        y += 40
        events = self.data.events_by_frame.get(frame, [])
        for evt in events:
            cv2.putText(canvas, f"EVENT: {evt['event_type']}", (20, y), FONT, 0.5, (100, 255, 255), 1)
            y += 25

        return canvas

    def render_caption(self, canvas: np.ndarray, frame: int):
        """Draw caption bar."""
        beat = self.get_beat(frame)

        caption = ""
        for (start, end), text in CAPTIONS.items():
            if start <= frame < end:
                caption = text
                break

        if caption:
            cv2.rectangle(canvas, (0, HEIGHT - 70), (WIDTH, HEIGHT), (0, 0, 0), -1)
            text_size = cv2.getTextSize(caption, FONT_BOLD, 0.85, 2)[0]
            x = (WIDTH - text_size[0]) // 2
            cv2.putText(canvas, caption, (x, HEIGHT - 25), FONT_BOLD, 0.85, COLOR_WHITE, 2)

    def render_frame(self, frame: int) -> np.ndarray:
        """Render complete frame."""
        canvas = np.full((HEIGHT, WIDTH, 3), BG, dtype=np.uint8)

        is_split = self.is_split_screen(frame)
        beat = self.get_beat(frame)

        if is_split:
            # Split world pane: left=BEFORE, right=AFTER
            half_w = WORLD_W // 2
            self.render_world_half(frame, False, canvas, 0, half_w)
            self.render_world_half(frame, True, canvas, half_w, half_w)

            # Divider
            cv2.line(canvas, (half_w, 0), (half_w, WORLD_H), COLOR_WHITE, 2)
        else:
            # Full AFTER view for thesis and scale beats
            self.render_world_half(frame, True, canvas, 0, WORLD_W)

        # Network pane
        network = self.render_network_pane(frame)
        canvas[:NETWORK_H, WORLD_W:] = network

        # Data pane
        data = self.render_data_pane(frame)
        canvas[NETWORK_H:, WORLD_W:] = data[:HEIGHT - NETWORK_H, :]

        # Borders
        cv2.line(canvas, (WORLD_W, 0), (WORLD_W, HEIGHT), BORDER, 2)
        cv2.line(canvas, (WORLD_W, NETWORK_H), (WIDTH, NETWORK_H), BORDER, 2)

        # Caption
        self.render_caption(canvas, frame)

        # Frame counter
        cv2.putText(canvas, f"Frame: {frame}/900", (WIDTH - 140, 25), FONT, 0.4, COLOR_DIM, 1)

        return canvas


# =============================================================================
# MAIN
# =============================================================================

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", default="./out")
    args = parser.parse_args()

    out_dir = Path(args.out)
    frames_dir = out_dir / "frames"
    frames_dir.mkdir(parents=True, exist_ok=True)

    print("=" * 60)
    print("GodView 30s Demo - Frame Renderer")
    print("=" * 60)

    data = DataLoader(out_dir)
    renderer = Renderer(data)

    total_frames = 900
    print(f"Rendering {total_frames} frames...")

    for frame in range(total_frames):
        img = renderer.render_frame(frame)
        path = frames_dir / f"frame_{frame:05d}.png"
        cv2.imwrite(str(path), img)

        if frame % 150 == 0:
            print(f"  Frame {frame}/900 ({frame/30:.0f}s)")

    print(f"Rendered {total_frames} frames to {frames_dir}")


if __name__ == "__main__":
    main()
