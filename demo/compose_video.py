#!/usr/bin/env python3
"""
GodView Demo - Video Compositor (The Editor)
Overlays HUD elements on rendered frames and encodes final video.

Based on: carla.md Section 8
"""

import cv2
import numpy as np
import json
import os
import subprocess
import argparse
from collections import defaultdict

# Configuration
FRAMES_PATH = "/workspace/godview_demo/frames/pass2_drone"  # Default to drone view
RAW_LOG_PATH = "/workspace/godview_demo/logs/raw_broken.ndjson"
MERGED_LOG_PATH = "/workspace/godview_demo/logs/godview_merged.ndjson"
EVENTS_LOG_PATH = "/workspace/godview_demo/logs/merge_events.ndjson"
OUTPUT_PATH = "/workspace/godview_demo/outputs"

# Video settings
FPS = 20
FRAME_WIDTH = 1920
FRAME_HEIGHT = 1080

# HUD Colors (BGR)
COLOR_RED = (0, 0, 255)      # Broken/Raw
COLOR_GREEN = (0, 255, 0)    # GodView/Fixed
COLOR_YELLOW = (0, 255, 255) # Warnings
COLOR_WHITE = (255, 255, 255)
COLOR_DARK_BG = (30, 30, 30)
COLOR_PANEL_BG = (50, 50, 50, 180)

# Fonts
FONT = cv2.FONT_HERSHEY_SIMPLEX
FONT_SCALE_LARGE = 1.2
FONT_SCALE_MEDIUM = 0.8
FONT_SCALE_SMALL = 0.6


def load_ndjson(path):
    """Load NDJSON file into list of dicts."""
    data = []
    if os.path.exists(path):
        with open(path, "r") as f:
            for line in f:
                if line.strip():
                    data.append(json.loads(line))
    return data


def group_by_frame(data):
    """Group data by frame number."""
    by_frame = defaultdict(list)
    for item in data:
        by_frame[item.get("frame", 0)].append(item)
    return by_frame


def world_to_screen(pos, camera_params):
    """
    Project 3D world coordinates to 2D screen coordinates.
    Simplified orthographic projection for top-down view.
    """
    # Camera at z=60, looking down, centered at origin
    # Scale: 1 meter = ~10 pixels at this height
    scale = 15.0
    center_x = FRAME_WIDTH // 2
    center_y = FRAME_HEIGHT // 2
    
    screen_x = int(center_x + pos[0] * scale)
    screen_y = int(center_y - pos[1] * scale)  # Y is inverted
    
    return screen_x, screen_y


def draw_bounding_box(frame, pos, color, label="", is_drone=False):
    """Draw a bounding box at the projected screen position."""
    x, y = world_to_screen(pos, None)
    
    # Skip if off-screen
    if x < 0 or x >= FRAME_WIDTH or y < 0 or y >= FRAME_HEIGHT:
        return
    
    # Box size varies by type
    if is_drone:
        box_w, box_h = 40, 40
    else:
        box_w, box_h = 60, 30
    
    # Draw box
    pt1 = (x - box_w//2, y - box_h//2)
    pt2 = (x + box_w//2, y + box_h//2)
    cv2.rectangle(frame, pt1, pt2, color, 2)
    
    # Draw label
    if label:
        cv2.putText(frame, label, (pt1[0], pt1[1] - 5),
                    FONT, FONT_SCALE_SMALL, color, 1)
    
    # For drones, draw vertical stem (Z indicator)
    if is_drone and len(pos) > 2 and pos[2] > 0:
        stem_height = int(pos[2] * 2)  # Scale Z for visibility
        cv2.line(frame, (x, y + box_h//2), (x, y + box_h//2 + stem_height),
                 color, 2)
        cv2.putText(frame, f"Z:{pos[2]:.0f}m", (x + 5, y + stem_height),
                    FONT, FONT_SCALE_SMALL, color, 1)


def draw_hud_panel(frame, x, y, width, height, alpha=0.7):
    """Draw semi-transparent panel for HUD."""
    overlay = frame.copy()
    cv2.rectangle(overlay, (x, y), (x + width, y + height), COLOR_DARK_BG, -1)
    cv2.addWeighted(overlay, alpha, frame, 1 - alpha, 0, frame)
    cv2.rectangle(frame, (x, y), (x + width, y + height), COLOR_WHITE, 1)


def draw_status_badge(frame, text, position, is_critical=False):
    """Draw status badge with background."""
    x, y = position
    color = COLOR_RED if is_critical else COLOR_GREEN
    text_size = cv2.getTextSize(text, FONT, FONT_SCALE_MEDIUM, 2)[0]
    
    # Background
    padding = 10
    cv2.rectangle(frame, 
                  (x - padding, y - text_size[1] - padding),
                  (x + text_size[0] + padding, y + padding),
                  color, -1)
    
    # Text
    cv2.putText(frame, text, (x, y), FONT, FONT_SCALE_MEDIUM, COLOR_WHITE, 2)


def draw_scrolling_log(frame, events, y_start, max_lines=5):
    """Draw scrolling event log at bottom of frame."""
    x = 20
    y = y_start
    line_height = 25
    
    # Draw panel
    panel_height = (max_lines + 1) * line_height
    draw_hud_panel(frame, 10, y_start - 30, FRAME_WIDTH - 20, panel_height)
    
    # Title
    cv2.putText(frame, "GODVIEW EVENT LOG", (x, y), 
                FONT, FONT_SCALE_MEDIUM, COLOR_YELLOW, 2)
    y += line_height
    
    # Events (most recent first)
    for event in events[-max_lines:]:
        event_code = event.get("event_code", "UNKNOWN")
        details = event.get("details", {})
        
        if event_code == "TRUST_REJECT":
            text = f"[TRUST] Ed25519 Verified: REJECTED {details.get('entity_id', 'unknown')}"
            color = COLOR_RED
        elif event_code == "ID_MERGE":
            text = f"[HIGHLANDER] Consensus: Merged {details.get('incoming_id', '')[:12]}... -> {details.get('canonical_id', '')[:12]}..."
            color = COLOR_GREEN
        elif event_code == "SPATIAL_CORRECTION":
            text = f"[H3 GRID] Z-Correction: {details.get('entity_id', '')} -> {details.get('corrected_z', 0):.1f}m"
            color = COLOR_YELLOW
        else:
            text = f"[{event_code}] {json.dumps(details)[:60]}"
            color = COLOR_WHITE
        
        cv2.putText(frame, text, (x, y), FONT, FONT_SCALE_SMALL, color, 1)
        y += line_height


def draw_metrics_panel(frame, frame_num, total_frames, mode="before"):
    """Draw metrics panel in top-left corner."""
    x, y = 20, 20
    draw_hud_panel(frame, 10, 10, 400, 180)
    
    # Title
    if mode == "before":
        cv2.putText(frame, "RAW SENSOR FEED", (x, y + 25), FONT, FONT_SCALE_LARGE, COLOR_RED, 2)
    else:
        cv2.putText(frame, "GODVIEW CONSENSUS", (x, y + 25), FONT, FONT_SCALE_LARGE, COLOR_GREEN, 2)
    
    # Metrics
    y_offset = 60
    metrics = [
        f"Frame: {frame_num}/{total_frames}",
        f"H3 Spatial Grid: {'ACTIVE' if mode == 'after' else 'DISABLED'}",
        f"Highlander Consensus: {'ON' if mode == 'after' else 'OFF'}",
        f"Ed25519 Verification: {'ENFORCED' if mode == 'after' else 'BYPASSED'}"
    ]
    
    for metric in metrics:
        cv2.putText(frame, metric, (x, y + y_offset), FONT, FONT_SCALE_SMALL, COLOR_WHITE, 1)
        y_offset += 25


def compose_frame(base_frame, frame_num, raw_data, merged_data, events, mode="both"):
    """Compose a single frame with HUD overlay."""
    frame = base_frame.copy()
    
    # Get data for this frame
    raw_frame_data = raw_data.get(frame_num, [])
    merged_frame_data = merged_data.get(frame_num, [])
    frame_events = [e for e in events if e.get("frame", -1) <= frame_num][-10:]
    
    # Draw bounding boxes
    if mode in ["before", "both"]:
        for packet in raw_frame_data:
            pos = packet.get("position", [0, 0, 0])
            is_drone = packet.get("class_id") == 4
            faults = packet.get("faults", {})
            
            # Jitter effect for OOSM
            if faults.get("oosm"):
                pos = [p + np.random.uniform(-1, 1) for p in pos]
            
            # Flatten for pancake
            if faults.get("pancake"):
                pos[2] = 0
            
            label = "GHOST" if faults.get("ghost") else ""
            if faults.get("sybil"):
                label = "SYBIL ATTACK!"
            
            draw_bounding_box(frame, pos, COLOR_RED, label, is_drone)
    
    if mode in ["after", "both"]:
        for packet in merged_frame_data:
            pos = packet.get("position", [0, 0, 0])
            is_drone = packet.get("class_id") == 4
            draw_bounding_box(frame, pos, COLOR_GREEN, "", is_drone)
    
    # Draw HUD elements
    draw_metrics_panel(frame, frame_num, 600, mode)  # Assuming 600 total frames
    
    # Status badge
    if mode == "before":
        draw_status_badge(frame, "STATUS: CRITICAL", (FRAME_WIDTH - 280, 50), is_critical=True)
    else:
        draw_status_badge(frame, "STATUS: STABLE", (FRAME_WIDTH - 250, 50), is_critical=False)
    
    # Scrolling log
    draw_scrolling_log(frame, frame_events, FRAME_HEIGHT - 150)
    
    return frame


def encode_video(frames_pattern, output_file, fps=20):
    """Encode frames to MP4 using ffmpeg."""
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
    parser.add_argument("--frames-dir", default=FRAMES_PATH, help="Input frames directory")
    parser.add_argument("--mode", default="split", choices=["before", "after", "split"],
                        help="Output mode: before (red), after (green), split (side-by-side)")
    parser.add_argument("--max-frames", type=int, default=600, help="Max frames to process")
    args = parser.parse_args()
    
    separator = "=" * 60
    print(separator)
    print("GodView Demo - Video Compositor")
    print(separator)
    
    # Load data
    print("\n[1/4] Loading data...")
    raw_data = group_by_frame(load_ndjson(RAW_LOG_PATH))
    merged_data = group_by_frame(load_ndjson(MERGED_LOG_PATH))
    events = load_ndjson(EVENTS_LOG_PATH)
    print(f"  Raw packets: {sum(len(v) for v in raw_data.values())}")
    print(f"  Merged packets: {sum(len(v) for v in merged_data.values())}")
    print(f"  Events: {len(events)}")
    
    # Create output directory
    os.makedirs(OUTPUT_PATH, exist_ok=True)
    composite_dir = os.path.join(OUTPUT_PATH, "composite_frames")
    os.makedirs(composite_dir, exist_ok=True)
    
    # Find input frames
    print(f"\n[2/4] Processing frames from {args.frames_dir}...")
    frame_files = sorted([f for f in os.listdir(args.frames_dir) if f.endswith('.png')])
    total_frames = min(len(frame_files), args.max_frames)
    print(f"  Found {len(frame_files)} frames, processing {total_frames}")
    
    # Process frames
    print("\n[3/4] Compositing frames with HUD...")
    for i, frame_file in enumerate(frame_files[:total_frames]):
        if i >= args.max_frames:
            break
        
        # Load base frame
        frame_path = os.path.join(args.frames_dir, frame_file)
        base_frame = cv2.imread(frame_path)
        
        if base_frame is None:
            print(f"  Warning: Could not load {frame_path}")
            continue
        
        # Compose based on mode
        if args.mode == "split":
            # Side-by-side: before on left, after on right
            left = compose_frame(base_frame, i, raw_data, merged_data, events, "before")
            right = compose_frame(base_frame, i, raw_data, merged_data, events, "after")
            
            # Resize each to half width
            left_half = cv2.resize(left, (FRAME_WIDTH // 2, FRAME_HEIGHT))
            right_half = cv2.resize(right, (FRAME_WIDTH // 2, FRAME_HEIGHT))
            
            # Combine
            composed = np.hstack([left_half, right_half])
            
            # Add center divider
            cv2.line(composed, (FRAME_WIDTH // 2, 0), (FRAME_WIDTH // 2, FRAME_HEIGHT), COLOR_WHITE, 3)
            cv2.putText(composed, "BEFORE", (FRAME_WIDTH // 4 - 50, 50), FONT, 1.5, COLOR_RED, 3)
            cv2.putText(composed, "AFTER", (3 * FRAME_WIDTH // 4 - 40, 50), FONT, 1.5, COLOR_GREEN, 3)
        else:
            composed = compose_frame(base_frame, i, raw_data, merged_data, events, args.mode)
        
        # Save composed frame
        output_path = os.path.join(composite_dir, f"composite_{i:04d}.png")
        cv2.imwrite(output_path, composed)
        
        if i % 50 == 0:
            print(f"    Frame {i}/{total_frames}")
    
    # Encode videos
    print("\n[4/4] Encoding videos with ffmpeg...")
    
    if args.mode == "before":
        output_file = os.path.join(OUTPUT_PATH, "video_before.mp4")
    elif args.mode == "after":
        output_file = os.path.join(OUTPUT_PATH, "video_after.mp4")
    else:
        output_file = os.path.join(OUTPUT_PATH, "final_linkedin.mp4")
    
    encode_video(
        os.path.join(composite_dir, "composite_%04d.png"),
        output_file,
        FPS
    )
    
    print(f"\n{separator}")
    print("COMPLETE!")
    print(f"  Output: {output_file}")
    print(separator)


if __name__ == "__main__":
    main()
