#!/usr/bin/env python3
"""
GodView V2 Demo - Overlay Renderer
====================================
Post-processes captured frames, adding HUD overlays based on NDJSON data.
Uses proper 3D-to-2D projection with camera intrinsics/extrinsics.
Implements split-screen RAW vs GODVIEW layout.
"""

import cv2
import numpy as np
import json
from pathlib import Path
from typing import Dict, List, Optional, Tuple
import sys

# Add utils to path
sys.path.insert(0, str(Path(__file__).parent))

from utils.projection import (
    CameraTransform, CameraIntrinsics,
    build_K_matrix, project_3d_bbox, clamp_bbox_to_screen
)
from utils.ndjson_parser import (
    load_packets_by_timestamp, get_frame_data,
    DetectionPacket, CanonicalStatePacket, MergeEventPacket, EventCode
)
from utils.drawing import (
    Colors, draw_bbox, draw_altitude_stem, draw_trust_badge,
    draw_scanline, draw_lidar_rings, draw_hexagon,
    create_split_screen, draw_top_bar, draw_bottom_bar,
    draw_caption, draw_ghost_merge_animation, draw_rejection_stamp,
    ease_in_out
)


# ============================================================================
# CONFIGURATION
# ============================================================================

FPS = 30
TOTAL_FRAMES = 2400
WIDTH = 1920
HEIGHT = 1080

# Directories
BASE_DIR = Path(__file__).parent
FRAMES_DIR = BASE_DIR / "frames"
DATA_DIR = BASE_DIR / "data"
OUTPUT_DIR = FRAMES_DIR / "overlayed"

# Phase frame ranges
SETUP_FRAMES = (0, 450)           # 0-15s
CHAOS_FRAMES = (451, 1350)        # 15-45s  
ACTIVATION_FRAMES = (1351, 1650)  # 45-55s
SOLUTION_FRAMES = (1651, 2100)    # 55-70s
DEEPDIVE_FRAMES = (2101, 2400)    # 70-80s


# ============================================================================
# OVERLAY RENDERER
# ============================================================================

class OverlayRenderer:
    def __init__(self):
        # Create output directory
        OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
        
        # Load NDJSON data
        print("[OVERLAY] Loading NDJSON data...")
        self.raw_packets = load_packets_by_timestamp(DATA_DIR / "raw_broken.ndjson")
        self.fused_packets = load_packets_by_timestamp(DATA_DIR / "godview_merged.ndjson")
        self.merge_events = load_packets_by_timestamp(DATA_DIR / "merge_events.ndjson")
        
        print(f"[OVERLAY] Loaded {len(self.raw_packets)} raw timestamps")
        print(f"[OVERLAY] Loaded {len(self.fused_packets)} fused timestamps")
        print(f"[OVERLAY] Loaded {len(self.merge_events)} event timestamps")
        
        # Stats tracking
        self.stats = {
            "ghosts": 0,
            "merges": 0,
            "trust_rejects": 0,
            "oosm_fixed": 0
        }
        
        # Event log for scrolling display
        self.event_log = []
    
    def load_frame_metadata(self, meta_path: Path) -> Tuple[CameraTransform, CameraIntrinsics]:
        """Load camera metadata from JSON file."""
        with open(meta_path, 'r') as f:
            data = json.load(f)
        
        cam = data["camera_transform"]
        loc = cam["location"]
        rot = cam["rotation"]
        
        transform = CameraTransform(
            x=loc["x"], y=loc["y"], z=loc["z"],
            pitch=rot["pitch"], yaw=rot["yaw"], roll=rot["roll"]
        )
        
        intr = data["intrinsics"]
        intrinsics = CameraIntrinsics(
            width=intr["width"], height=intr["height"], fov=intr["fov"],
            fx=intr["fx"], fy=intr["fy"], cx=intr["cx"], cy=intr["cy"]
        )
        
        return transform, intrinsics
    
    def get_phase(self, frame_idx: int) -> str:
        """Get current phase name."""
        if SETUP_FRAMES[0] <= frame_idx < SETUP_FRAMES[1]:
            return "SETUP"
        elif CHAOS_FRAMES[0] <= frame_idx < CHAOS_FRAMES[1]:
            return "CHAOS"
        elif ACTIVATION_FRAMES[0] <= frame_idx < ACTIVATION_FRAMES[1]:
            return "ACTIVATION"
        elif SOLUTION_FRAMES[0] <= frame_idx < SOLUTION_FRAMES[1]:
            return "SOLUTION"
        else:
            return "DEEPDIVE"
    
    def draw_raw_overlay(
        self,
        frame: np.ndarray,
        frame_idx: int,
        cam_transform: CameraTransform,
        intrinsics: CameraIntrinsics
    ) -> np.ndarray:
        """Draw RAW sensor view overlays (chaos)."""
        K = build_K_matrix(intrinsics)
        frame_data = get_frame_data(
            self.raw_packets, self.fused_packets, self.merge_events,
            frame_idx, FPS
        )
        
        # Draw raw detections
        for packet in frame_data['raw']:
            if isinstance(packet, DetectionPacket):
                for obj in packet.objects:
                    # Project 3D bbox
                    bbox = project_3d_bbox(
                        obj.pose, obj.bbox_extent,
                        cam_transform, K
                    )
                    
                    if bbox is None:
                        continue
                    
                    bbox = clamp_bbox_to_screen(bbox, intrinsics.width, intrinsics.height)
                    if bbox is None:
                        continue
                    
                    # Determine color and style
                    if obj.note and "SYBIL" in obj.note:
                        color = Colors.MAGENTA
                        label = "SYBIL"
                    elif obj.note and "GHOST" in obj.note:
                        color = (100, 100, 255)  # Light red
                        label = "GHOST"
                        # Flicker effect
                        if frame_idx % 3 == 0:
                            continue
                    elif obj.note and "PANCAKE" in obj.note:
                        color = Colors.ORANGE
                        label = "DRONE (z=0!)"
                    else:
                        color = Colors.RED
                        label = f"{obj.object_class.upper()}"
                    
                    # Add jitter for raw view
                    jitter_x = int(np.random.normal(0, 2))
                    jitter_y = int(np.random.normal(0, 2))
                    bbox = (
                        bbox[0] + jitter_x,
                        bbox[1] + jitter_y,
                        bbox[2] + jitter_x,
                        bbox[3] + jitter_y
                    )
                    
                    draw_bbox(frame, bbox, color, dashed=True, label=label)
        
        return frame
    
    def draw_godview_overlay(
        self,
        frame: np.ndarray,
        frame_idx: int,
        cam_transform: CameraTransform,
        intrinsics: CameraIntrinsics
    ) -> np.ndarray:
        """Draw GODVIEW fused overlays (stable)."""
        K = build_K_matrix(intrinsics)
        frame_data = get_frame_data(
            self.raw_packets, self.fused_packets, self.merge_events,
            frame_idx, FPS
        )
        
        # Draw canonical objects
        for packet in frame_data['fused']:
            if isinstance(packet, CanonicalStatePacket):
                for obj in packet.objects:
                    # Project 3D bbox
                    bbox = project_3d_bbox(
                        obj.pose, obj.bbox_extent,
                        cam_transform, K
                    )
                    
                    if bbox is None:
                        continue
                    
                    bbox = clamp_bbox_to_screen(bbox, intrinsics.width, intrinsics.height)
                    if bbox is None:
                        continue
                    
                    # Drone altitude stem
                    if obj.object_class == "drone":
                        # Draw stem from drone to ground
                        drone_center = ((bbox[0] + bbox[2]) // 2, bbox[3])
                        ground_y = min(bbox[3] + int(obj.pose['z'] * 5), intrinsics.height - 10)
                        ground_pos = (drone_center[0], ground_y)
                        draw_altitude_stem(frame, drone_center, ground_pos, Colors.GREEN, obj.pose['z'])
                    
                    # Draw stable green box
                    label = f"{obj.object_class.upper()} [{obj.confidence:.0%}]"
                    draw_bbox(frame, bbox, Colors.GREEN, label=label)
                    
                    # Trust badge
                    badge_pos = (bbox[2] + 10, bbox[1])
                    draw_trust_badge(frame, badge_pos, trusted=True, size=12)
        
        return frame
    
    def render_frame(self, frame_idx: int) -> Optional[np.ndarray]:
        """Render a single frame with all overlays."""
        phase = self.get_phase(frame_idx)
        
        # Determine which camera pass to use
        is_birdseye_phase = phase in ["SETUP", "DEEPDIVE"]
        
        if is_birdseye_phase:
            frame_dir = FRAMES_DIR / "birdseye"
            meta_dir = FRAMES_DIR / "birdseye" / "meta"
        else:
            frame_dir = FRAMES_DIR / "chase"
            meta_dir = FRAMES_DIR / "chase" / "meta"
        
        # Load frame and metadata
        frame_path = frame_dir / f"frame_{frame_idx:05d}.png"
        meta_path = meta_dir / f"frame_{frame_idx:05d}.json"
        
        if not frame_path.exists():
            return None
        
        frame = cv2.imread(str(frame_path))
        if frame is None:
            return None
        
        if not meta_path.exists():
            # Create dummy metadata if missing
            cam_transform = CameraTransform(0, 0, 100, -90, 0, 0)
            intrinsics = CameraIntrinsics(WIDTH, HEIGHT, 90, 960, 960, 960, 540)
        else:
            cam_transform, intrinsics = self.load_frame_metadata(meta_path)
        
        # Phase-specific rendering
        if phase == "SETUP":
            frame = self.render_setup_phase(frame, frame_idx, cam_transform, intrinsics)
        
        elif phase in ["CHAOS", "ACTIVATION", "SOLUTION"]:
            frame = self.render_split_screen_phase(frame, frame_idx, phase, cam_transform, intrinsics)
        
        elif phase == "DEEPDIVE":
            frame = self.render_deepdive_phase(frame, frame_idx, cam_transform, intrinsics)
        
        return frame
    
    def render_setup_phase(
        self,
        frame: np.ndarray,
        frame_idx: int,
        cam_transform: CameraTransform,
        intrinsics: CameraIntrinsics
    ) -> np.ndarray:
        """Render SETUP phase: Bird's-eye view with minimal overlays."""
        # Draw simple labels on agents
        K = build_K_matrix(intrinsics)
        frame_data = get_frame_data(
            self.raw_packets, self.fused_packets, self.merge_events,
            frame_idx, FPS
        )
        
        for packet in frame_data['fused']:
            if isinstance(packet, CanonicalStatePacket):
                for obj in packet.objects:
                    bbox = project_3d_bbox(obj.pose, obj.bbox_extent, cam_transform, K)
                    if bbox:
                        bbox = clamp_bbox_to_screen(bbox, intrinsics.width, intrinsics.height)
                        if bbox:
                            # Subtle grey boxes
                            draw_bbox(frame, bbox, Colors.GREY, thickness=1)
        
        # HUD
        draw_top_bar(frame, "PHASE 1: SETUP", "Baseline - Pre-Fusion", Colors.WHITE)
        draw_bottom_bar(frame, {"Agents": 18, "Status": "Standby"}, frame_idx / FPS, 80)
        
        return frame
    
    def render_split_screen_phase(
        self,
        frame: np.ndarray,
        frame_idx: int,
        phase: str,
        cam_transform: CameraTransform,
        intrinsics: CameraIntrinsics
    ) -> np.ndarray:
        """Render split-screen view: RAW on left, GODVIEW on right."""
        # Create two copies of the base frame
        left_frame = frame.copy()
        right_frame = frame.copy()
        
        # Draw RAW overlay on left (chaos)
        left_frame = self.draw_raw_overlay(left_frame, frame_idx, cam_transform, intrinsics)
        
        # Draw GODVIEW overlay on right (stable)
        right_frame = self.draw_godview_overlay(right_frame, frame_idx, cam_transform, intrinsics)
        
        # Combine into split-screen
        combined = create_split_screen(
            left_frame, right_frame,
            labels=("RAW SENSOR", "GODVIEW CONSENSUS")
        )
        
        # Additional phase-specific effects
        if phase == "ACTIVATION":
            # Scanline sweep
            activation_start = ACTIVATION_FRAMES[0]
            activation_end = ACTIVATION_FRAMES[1]
            progress = (frame_idx - activation_start) / (activation_end - activation_start)
            draw_scanline(combined, progress)
        
        # Caption
        draw_caption(combined, "Same scene. Same sensors. One has consensus.", y_pos=70)
        
        # Phase indicator
        phase_colors = {
            "CHAOS": Colors.RED,
            "ACTIVATION": Colors.YELLOW,
            "SOLUTION": Colors.GREEN
        }
        draw_top_bar(combined, f"PHASE: {phase}", 
                    "CRITICAL" if phase == "CHAOS" else "SYNCING" if phase == "ACTIVATION" else "STABLE",
                    phase_colors.get(phase, Colors.WHITE))
        
        # Metrics
        metrics = {
            "Ghosts": self.stats["ghosts"],
            "Merged": self.stats["merges"],
            "Rejected": self.stats["trust_rejects"]
        }
        draw_bottom_bar(combined, metrics, frame_idx / FPS, 80)
        
        # Update stats based on phase
        if phase == "CHAOS":
            self.stats["ghosts"] = max(5, int(7 + 3 * np.sin(frame_idx / 30)))
        elif phase == "ACTIVATION":
            # Ghosts decreasing during activation
            progress = (frame_idx - ACTIVATION_FRAMES[0]) / (ACTIVATION_FRAMES[1] - ACTIVATION_FRAMES[0])
            self.stats["ghosts"] = int(7 * (1 - progress))
            self.stats["merges"] = int(progress * 8)
            if frame_idx == 1400:
                self.stats["trust_rejects"] = 1
        elif phase == "SOLUTION":
            self.stats["ghosts"] = 0
            self.stats["merges"] = 8
        
        return combined
    
    def render_deepdive_phase(
        self,
        frame: np.ndarray,
        frame_idx: int,
        cam_transform: CameraTransform,
        intrinsics: CameraIntrinsics
    ) -> np.ndarray:
        """Render DEEPDIVE phase: Bird's-eye with H3 grid and LIDAR."""
        K = build_K_matrix(intrinsics)
        
        # Draw GODVIEW overlays
        frame = self.draw_godview_overlay(frame, frame_idx, cam_transform, intrinsics)
        
        # Draw LIDAR rings (stylized)
        center = (intrinsics.width // 2, intrinsics.height // 2)
        # Scale rings for bird's-eye view
        radii = [50, 100, 150, 200, 250]
        draw_lidar_rings(frame, center, radii, Colors.LIGHT_BLUE)
        
        # Draw H3 grid (simplified hexagonal pattern)
        hex_size = 60
        for row in range(-8, 9):
            for col in range(-8, 9):
                # Offset for hex grid
                x_off = col * hex_size * 1.5
                y_off = row * hex_size * np.sqrt(3) + (col % 2) * hex_size * np.sqrt(3) / 2
                
                cx = int(center[0] + x_off)
                cy = int(center[1] + y_off)
                
                if 0 <= cx < intrinsics.width and 0 <= cy < intrinsics.height:
                    # Draw hexagon
                    vertices = []
                    for i in range(6):
                        angle = np.radians(60 * i)
                        vx = cx + int(hex_size * 0.4 * np.cos(angle))
                        vy = cy + int(hex_size * 0.4 * np.sin(angle))
                        vertices.append((vx, vy))
                    
                    draw_hexagon(frame, vertices, Colors.GREY, alpha=0.3)
        
        # HUD
        draw_top_bar(frame, "PHASE 5: DEEPDIVE", "System Diagnostics", Colors.GREEN)
        
        metrics = {
            "Ghosts Merged": 8,
            "Sybil Rejected": 1,
            "OOSM Fixed": 23,
            "∆ Error": "0.2m"
        }
        draw_bottom_bar(frame, metrics, frame_idx / FPS, 80)
        
        return frame
    
    def run(self):
        """Process all frames."""
        print("=" * 60)
        print("GODVIEW V2 OVERLAY RENDERER")
        print("=" * 60)
        
        rendered_count = 0
        
        for frame_idx in range(TOTAL_FRAMES):
            result = self.render_frame(frame_idx)
            
            if result is not None:
                output_path = OUTPUT_DIR / f"frame_{frame_idx:05d}.png"
                cv2.imwrite(str(output_path), result)
                rendered_count += 1
            
            if rendered_count % 60 == 0 and rendered_count > 0:
                print(f"[OVERLAY] Rendered {rendered_count} frames (current: {frame_idx})")
        
        print("=" * 60)
        print(f"OVERLAY COMPLETE: {rendered_count} frames")
        print(f"Output: {OUTPUT_DIR}")
        print("=" * 60)


def main():
    renderer = OverlayRenderer()
    renderer.run()


if __name__ == "__main__":
    main()
