#!/usr/bin/env python3
"""
GodView Demo Fix - Video Encoder
================================
Encodes PNG frames to MP4 using FFmpeg.
"""

import argparse
import subprocess
import sys
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description="GodView Demo Fix - Video Encoder")
    parser.add_argument("--out", type=str, default="./out", help="Output directory")
    parser.add_argument("--fps", type=int, default=30, help="FPS")
    parser.add_argument("--crf", type=int, default=18, help="CRF quality")
    args = parser.parse_args()

    out_dir = Path(args.out)
    frames_dir = out_dir / "frames"
    output_file = out_dir / "final_godview_demo_fixed.mp4"

    print("=" * 60)
    print("GodView Demo Fix - Video Encoder")
    print("=" * 60)
    print(f"Input: {frames_dir}")
    print(f"Output: {output_file}")
    print("=" * 60)

    frame_pattern = frames_dir / "frame_%05d.png"
    first_frame = frames_dir / "frame_00000.png"
    if not first_frame.exists():
        print(f"ERROR: No frames found at {first_frame}")
        sys.exit(1)

    frame_count = len(list(frames_dir.glob("frame_*.png")))
    duration = frame_count / args.fps
    print(f"Found {frame_count} frames ({duration:.1f}s)")

    cmd = [
        "ffmpeg", "-y",
        "-framerate", str(args.fps),
        "-i", str(frame_pattern),
        "-c:v", "libx264",
        "-crf", str(args.crf),
        "-pix_fmt", "yuv420p",
        "-preset", "medium",
        str(output_file),
    ]

    print(f"\nRunning: {' '.join(cmd)}\n")
    result = subprocess.run(cmd)

    if result.returncode != 0:
        print(f"ERROR: FFmpeg failed with code {result.returncode}")
        sys.exit(result.returncode)

    if output_file.exists():
        size_mb = output_file.stat().st_size / (1024 * 1024)
        print(f"\n✓ Encoded {output_file}")
        print(f"  Size: {size_mb:.1f} MB")
        print(f"  Duration: {duration:.1f}s")


if __name__ == "__main__":
    main()
