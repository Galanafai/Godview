#!/usr/bin/env python3
"""
GodView V2 Demo - Composer
===========================
Assembles overlayed frames into final MP4 video using FFmpeg.
"""

import subprocess
from pathlib import Path
import argparse
import sys


# ============================================================================
# CONFIGURATION
# ============================================================================

FPS = 30
TOTAL_FRAMES = 2400

BASE_DIR = Path(__file__).parent
FRAMES_DIR = BASE_DIR / "frames" / "overlayed"
OUTPUT_FILE = BASE_DIR / "final_godview_demo.mp4"


def check_ffmpeg() -> bool:
    """Check if FFmpeg is available."""
    try:
        result = subprocess.run(
            ['ffmpeg', '-version'],
            capture_output=True,
            text=True
        )
        return result.returncode == 0
    except FileNotFoundError:
        return False


def count_frames() -> int:
    """Count available frames."""
    if not FRAMES_DIR.exists():
        return 0
    return len(list(FRAMES_DIR.glob("*.png")))


def compose_video(
    input_pattern: str,
    output_file: str,
    fps: int = 30,
    crf: int = 18
) -> bool:
    """
    Compose video from frames using FFmpeg.
    
    Args:
        input_pattern: Input frame pattern (e.g., "frames/overlayed/frame_%05d.png")
        output_file: Output MP4 path
        fps: Frame rate
        crf: Quality (lower = better, 18 is visually lossless)
    
    Returns:
        True if successful
    """
    cmd = [
        'ffmpeg',
        '-y',  # Overwrite output
        '-framerate', str(fps),
        '-i', input_pattern,
        '-c:v', 'libx264',
        '-crf', str(crf),
        '-pix_fmt', 'yuv420p',
        '-preset', 'medium',
        '-movflags', '+faststart',  # Web optimization
        output_file
    ]
    
    print(f"[COMPOSER] Running: {' '.join(cmd)}")
    
    result = subprocess.run(cmd, capture_output=True, text=True)
    
    if result.returncode != 0:
        print(f"[ERROR] FFmpeg failed:")
        print(result.stderr)
        return False
    
    return True


def main():
    parser = argparse.ArgumentParser(description="GodView V2 Video Composer")
    parser.add_argument("--fps", type=int, default=FPS, help="Frame rate")
    parser.add_argument("--crf", type=int, default=18, help="Quality (lower=better)")
    parser.add_argument("--output", type=str, default=str(OUTPUT_FILE), help="Output file")
    args = parser.parse_args()
    
    print("=" * 60)
    print("GODVIEW V2 VIDEO COMPOSER")
    print("=" * 60)
    
    # Check FFmpeg
    if not check_ffmpeg():
        print("[ERROR] FFmpeg not found! Install with: apt install ffmpeg")
        sys.exit(1)
    
    # Check frames
    frame_count = count_frames()
    if frame_count == 0:
        print(f"[ERROR] No frames found in {FRAMES_DIR}")
        print("Run overlay_renderer.py first!")
        sys.exit(1)
    
    print(f"[COMPOSER] Found {frame_count} frames")
    print(f"[COMPOSER] Expected duration: {frame_count / args.fps:.1f}s")
    
    # Compose video
    input_pattern = str(FRAMES_DIR / "frame_%05d.png")
    
    print(f"[COMPOSER] Encoding to {args.output}...")
    success = compose_video(input_pattern, args.output, args.fps, args.crf)
    
    if success:
        # Get file size
        output_path = Path(args.output)
        if output_path.exists():
            size_mb = output_path.stat().st_size / (1024 * 1024)
            print("=" * 60)
            print("COMPOSITION COMPLETE")
            print("=" * 60)
            print(f"Output: {args.output}")
            print(f"Size: {size_mb:.1f} MB")
            print(f"Duration: {frame_count / args.fps:.1f}s @ {args.fps} FPS")
            print("=" * 60)
    else:
        print("[ERROR] Composition failed!")
        sys.exit(1)


if __name__ == "__main__":
    main()
