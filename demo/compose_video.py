#!/usr/bin/env python3
"""
GodView Demo - Video Compositor (The Editor)
=============================================
Creates Before/After comparison video from NDJSON logs.

FIX: This version generates frames PURELY from log data - no CARLA frames needed.
It creates a visual "data view" showing the chaos vs. consensus narrative.

Output modes:
  - split: Side-by-side Before (RED) vs After (GREEN)
  - before: Raw sensor chaos only
  - after: GodView consensus only
"""

import cv2
import numpy as np
import json
import os
import subprocess
import argparse
from collections import defaultdict

# Configuration
RAW_LOG_PATH = "/workspace/godview_demo/logs/raw_broken.ndjson"
MERGED_LOG_PATH = "/workspace/godview_demo/logs/godview_merged.ndjson"
EVENTS_LOG_PATH = "/workspace/godview_demo/logs/merge_events.ndjson"
OUTPUT_PATH = "/workspace/godview_demo/outputs"

# Video settings
FPS = 20
FRAME_WIDTH = 1920
FRAME_HEIGHT = 1080

# Colors (BGR for OpenCV)
COLOR_BG = (20, 20, 25)           # Dark background
COLOR_GRID = (40, 40, 50)         # Grid lines
COLOR_RAW = (50, 50, 255)         # RED - broken
COLOR_GODVIEW = (50, 255, 50)     # GREEN - fixed
COLOR_GHOST = (150, 100, 255)     # Light red for ghosts
COLOR_DRONE = (255, 200, 100)     # Cyan for drones
COLOR_SYBIL = (255, 0, 255)       # Magenta for attacks
COLOR_TEXT = (255, 255, 255)      # White
COLOR_CRITICAL = (0, 0, 255)      # Red
COLOR_STABLE = (0, 255, 0)        # Green

# Fonts
FONT = cv2.FONT_HERSHEY_SIMPLEX
FONT_MONO = cv2.FONT_HERSHEY_PLAIN


def load_ndjson(path):
    """Load NDJSON file."""
    data = []
    if os.path.exists(path):
        with open(path, 'r') as f:
            for line in f:
                if line.strip():
                    try:
                        data.append(json.loads(line))
                    except json.JSONDecodeError:
                        pass
    return data


def group_by_frame(data):
    """Group data by frame number."""
    by_frame = defaultdict(list)
    for item in data:
        by_frame[item.get('frame', 0)].append(item)
    return by_frame


def world_to_screen(pos, scale=8.0, offset_x=0, offset_y=0):
    """Convert world coordinates to screen coordinates."""
    center_x = FRAME_WIDTH // 4 + offset_x
    center_y = FRAME_HEIGHT // 2 + offset_y
    
    x = int(center_x + pos[0] * scale)
    y = int(center_y - pos[1] * scale)  # Y inverted
    
    return x, y


def draw_grid(frame, half=False):
    """Draw coordinate grid."""
    width = FRAME_WIDTH // 2 if half else FRAME_WIDTH
    offset = FRAME_WIDTH // 2 if half else 0
    
    # Vertical lines
    for x in range(offset, offset + width, 50):
        cv2.line(frame, (x, 0), (x, FRAME_HEIGHT), COLOR_GRID, 1)
    
    # Horizontal lines
    for y in range(0, FRAME_HEIGHT, 50):
        cv2.line(frame, (offset, y), (offset + width, y), COLOR_GRID, 1)


def draw_vehicle_box(frame, pos, color, label="", size=20, dashed=False):
    """Draw a vehicle bounding box."""
    x, y = world_to_screen(pos)
    
    # Skip if off-screen
    if x < 0 or x >= FRAME_WIDTH or y < 0 or y >= FRAME_HEIGHT:
        return
    
    half = size // 2
    pt1 = (x - half, y - half)
    pt2 = (x + half, y + half)
    
    if dashed:
        # Draw dashed rectangle
        for i in range(0, size, 6):
            cv2.line(frame, (pt1[0] + i, pt1[1]), (pt1[0] + i + 3, pt1[1]), color, 2)
            cv2.line(frame, (pt1[0] + i, pt2[1]), (pt1[0] + i + 3, pt2[1]), color, 2)
            cv2.line(frame, (pt1[0], pt1[1] + i), (pt1[0], pt1[1] + i + 3), color, 2)
            cv2.line(frame, (pt2[0], pt1[1] + i), (pt2[0], pt1[1] + i + 3), color, 2)
    else:
        cv2.rectangle(frame, pt1, pt2, color, 2)
    
    # Label
    if label:
        cv2.putText(frame, label, (pt1[0], pt1[1] - 5), FONT, 0.4, color, 1)


def draw_drone(frame, pos, true_z, is_raw=True, offset_x=0):
    """Draw drone with altitude indicator."""
    # In raw mode, show pancake (z=0)
    display_z = 0 if is_raw else true_z
    display_pos = [pos[0], pos[1], display_z]
    
    x, y = world_to_screen(display_pos, offset_x=offset_x)
    
    if x < 0 or x >= FRAME_WIDTH or y < 0 or y >= FRAME_HEIGHT:
        return
    
    color = COLOR_RAW if is_raw else COLOR_GODVIEW
    
    # Drone symbol (circle with cross)
    cv2.circle(frame, (x, y), 15, color, 2)
    cv2.line(frame, (x - 10, y), (x + 10, y), color, 1)
    cv2.line(frame, (x, y - 10), (x, y + 10), color, 1)
    
    # Z indicator
    if not is_raw and true_z > 0:
        # Draw altitude stem
        ground_y = y + int(true_z * 2)  # Scale for visibility
        cv2.line(frame, (x, y), (x, ground_y), COLOR_DRONE, 1)
        cv2.putText(frame, f"Z:{true_z:.0f}m", (x + 5, y + 10), FONT, 0.4, COLOR_DRONE, 1)
    else:
        cv2.putText(frame, "Z:0!", (x + 5, y + 10), FONT, 0.4, COLOR_RAW, 1)


def draw_hud_panel(frame, x, y, width, height):
    """Draw semi-transparent panel."""
    overlay = frame.copy()
    cv2.rectangle(overlay, (x, y), (x + width, y + height), (30, 30, 35), -1)
    cv2.addWeighted(overlay, 0.8, frame, 0.2, 0, frame)
    cv2.rectangle(frame, (x, y), (x + width, y + height), (60, 60, 70), 1)


def draw_status_badge(frame, text, pos, is_critical=False):
    """Draw status badge."""
    color = COLOR_CRITICAL if is_critical else COLOR_STABLE
    x, y = pos
    
    (tw, th), _ = cv2.getTextSize(text, FONT, 0.8, 2)
    padding = 10
    
    cv2.rectangle(frame, (x, y - th - padding), (x + tw + padding * 2, y + padding), color, -1)
    cv2.putText(frame, text, (x + padding, y), FONT, 0.8, COLOR_TEXT, 2)


def draw_event_log(frame, events, y_start, max_lines=6):
    """Draw scrolling event log."""
    x = 20
    y = y_start
    line_height = 22
    
    # Panel
    panel_height = (max_lines + 1) * line_height + 10
    draw_hud_panel(frame, 10, y_start - 25, FRAME_WIDTH - 20, panel_height)
    
    # Title
    cv2.putText(frame, "GODVIEW EVENT LOG", (x, y), FONT, 0.6, (255, 255, 100), 1)
    y += line_height
    
    # Events
    for event in events[-max_lines:]:
        event_code = event.get('event_code', 'UNKNOWN')
        details = event.get('details', {})
        
        if event_code == 'ID_MERGE':
            color = COLOR_GODVIEW
            text = f"[HIGHLANDER] Merged: {details.get('incoming_id', '')[:16]}..."
        elif event_code == 'SPATIAL_CORRECTION':
            color = COLOR_DRONE
            text = f"[H3 GRID] Z-Fix: {details.get('entity_id', '')} -> {details.get('corrected_z', 0):.0f}m"
        elif event_code == 'TRUST_REJECT':
            color = COLOR_SYBIL
            text = f"[SECURITY] REJECTED: {details.get('reason', 'invalid')}"
        else:
            color = COLOR_TEXT
            text = f"[{event_code}] {str(details)[:50]}"
        
        cv2.putText(frame, text, (x, y), FONT_MONO, 1.0, color, 1)
        y += line_height


def create_frame(frame_num, raw_data, merged_data, events, mode="split", total_frames=600):
    """Create a single video frame."""
    # Create dark background
    frame = np.full((FRAME_HEIGHT, FRAME_WIDTH, 3), COLOR_BG, dtype=np.uint8)
    
    # Get data for this frame
    raw_items = raw_data.get(frame_num, [])
    merged_items = merged_data.get(frame_num, [])
    frame_events = [e for e in events if e.get('frame', -1) <= frame_num][-15:]
    
    if mode == "split":
        # Left half: RAW (Before)
        # Right half: GODVIEW (After)
        
        # Draw grids
        draw_grid(frame, half=True)
        
        # Divider
        cv2.line(frame, (FRAME_WIDTH // 2, 0), (FRAME_WIDTH // 2, FRAME_HEIGHT), COLOR_TEXT, 3)
        
        # Labels
        cv2.putText(frame, "BEFORE (Raw Sensors)", (50, 50), FONT, 1.0, COLOR_RAW, 2)
        cv2.putText(frame, "AFTER (GodView)", (FRAME_WIDTH // 2 + 50, 50), FONT, 1.0, COLOR_GODVIEW, 2)
        
        # Status badges
        draw_status_badge(frame, "CRITICAL", (FRAME_WIDTH // 4 - 60, 100), is_critical=True)
        draw_status_badge(frame, "STABLE", (FRAME_WIDTH * 3 // 4 - 50, 100), is_critical=False)
        
        # Draw RAW data (left side)
        for item in raw_items:
            pos = item.get('position', [0, 0, 0])
            faults = item.get('faults', {})
            class_id = item.get('class_id', 1)
            
            if class_id == 4:  # Drone
                draw_drone(frame, pos, item.get('true_z', 15), is_raw=True, offset_x=0)
            else:
                # Apply visual jitter for OOSM
                if faults.get('oosm'):
                    pos = [p + np.random.uniform(-2, 2) for p in pos]
                
                label = ""
                if faults.get('ghost'):
                    label = "GHOST"
                if faults.get('sybil'):
                    label = "SYBIL!"
                
                draw_vehicle_box(frame, pos, COLOR_RAW, label, dashed=faults.get('ghost', False))
        
        # Draw GODVIEW data (right side, offset by half width)
        for item in merged_items:
            pos = item.get('position', [0, 0, 0])
            class_id = item.get('class_id', 1)
            
            # Offset for right half
            shifted_pos = [pos[0], pos[1], pos[2]]
            x, y = world_to_screen(shifted_pos, offset_x=FRAME_WIDTH // 2)
            
            if class_id == 4:  # Drone
                draw_drone(frame, pos, pos[2], is_raw=False, offset_x=FRAME_WIDTH // 2)
            else:
                draw_vehicle_box(frame, [pos[0] + FRAME_WIDTH // 16, pos[1], pos[2]], COLOR_GODVIEW)
    
    else:
        # Single view mode
        draw_grid(frame)
        color = COLOR_RAW if mode == "before" else COLOR_GODVIEW
        data = raw_items if mode == "before" else merged_items
        
        for item in data:
            pos = item.get('position', [0, 0, 0])
            draw_vehicle_box(frame, pos, color)
    
    # Draw frame counter
    cv2.putText(frame, f"Frame: {frame_num}/{total_frames}", (20, FRAME_HEIGHT - 200), 
                FONT, 0.6, COLOR_TEXT, 1)
    
    # Draw event log
    draw_event_log(frame, frame_events, FRAME_HEIGHT - 180)
    
    return frame


def encode_video(frames_pattern, output_file, fps=20):
    """Encode frames to MP4."""
    cmd = [
        "ffmpeg", "-y",
        "-framerate", str(fps),
        "-i", frames_pattern,
        "-c:v", "libx264",
        "-pix_fmt", "yuv420p",
        "-crf", "18",
        "-preset", "medium",
        output_file
    ]
    print(f"  Running: {' '.join(cmd)}")
    subprocess.run(cmd, check=True)


def main():
    parser = argparse.ArgumentParser(description="GodView Demo - Video Compositor")
    parser.add_argument("--mode", default="split", choices=["before", "after", "split"],
                        help="Output mode")
    parser.add_argument("--max-frames", type=int, default=600, help="Max frames")
    parser.add_argument("--raw-log", default=RAW_LOG_PATH, help="Raw log path")
    parser.add_argument("--merged-log", default=MERGED_LOG_PATH, help="Merged log path")
    parser.add_argument("--events-log", default=EVENTS_LOG_PATH, help="Events log path")
    parser.add_argument("--output-dir", default=OUTPUT_PATH, help="Output directory")
    args = parser.parse_args()
    
    print("=" * 60)
    print("GodView Demo - Video Compositor")
    print("=" * 60)
    
    # Load data
    print("\n[1/3] Loading NDJSON logs...")
    raw_data = group_by_frame(load_ndjson(args.raw_log))
    merged_data = group_by_frame(load_ndjson(args.merged_log))
    events = load_ndjson(args.events_log)
    
    print(f"  Raw packets: {sum(len(v) for v in raw_data.values())}")
    print(f"  Merged packets: {sum(len(v) for v in merged_data.values())}")
    print(f"  Events: {len(events)}")
    
    if not raw_data and not merged_data:
        print("\n[ERROR] No data found! Run scenario_runner.py and generate_logs.py first.")
        return
    
    # Create output directories
    os.makedirs(args.output_dir, exist_ok=True)
    composite_dir = os.path.join(args.output_dir, "composite_frames")
    os.makedirs(composite_dir, exist_ok=True)
    
    # Determine frame range
    max_frame = max(
        max(raw_data.keys()) if raw_data else 0,
        max(merged_data.keys()) if merged_data else 0
    )
    total_frames = min(max_frame + 1, args.max_frames)
    
    print(f"\n[2/3] Generating {total_frames} frames ({args.mode} mode)...")
    
    for i in range(total_frames):
        frame = create_frame(i, raw_data, merged_data, events, args.mode, total_frames)
        output_path = os.path.join(composite_dir, f"composite_{i:04d}.png")
        cv2.imwrite(output_path, frame)
        
        if i % 100 == 0:
            print(f"    Frame {i}/{total_frames}")
    
    print(f"  Saved frames to {composite_dir}")
    
    # Encode video
    print("\n[3/3] Encoding video with ffmpeg...")
    
    if args.mode == "before":
        output_file = os.path.join(args.output_dir, "video_before.mp4")
    elif args.mode == "after":
        output_file = os.path.join(args.output_dir, "video_after.mp4")
    else:
        output_file = os.path.join(args.output_dir, "final_linkedin.mp4")
    
    try:
        encode_video(
            os.path.join(composite_dir, "composite_%04d.png"),
            output_file,
            FPS
        )
        print(f"\n{'=' * 60}")
        print("COMPLETE!")
        print(f"  Video: {output_file}")
        print("=" * 60)
    except Exception as e:
        print(f"  FFmpeg error: {e}")
        print(f"  Frames available at: {composite_dir}")


if __name__ == "__main__":
    main()
