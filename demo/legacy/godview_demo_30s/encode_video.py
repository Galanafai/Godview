#!/usr/bin/env python3
"""GodView 30s Demo - Video Encoder"""

import argparse
import subprocess
import sys
from pathlib import Path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", default="./out")
    parser.add_argument("--fps", type=int, default=30)
    args = parser.parse_args()

    out_dir = Path(args.out)
    frames_dir = out_dir / "frames"
    output = out_dir / "final_godview_30s.mp4"

    print("=" * 60)
    print("GodView 30s Demo - Video Encoder")
    print("=" * 60)

    pattern = frames_dir / "frame_%05d.png"
    if not (frames_dir / "frame_00000.png").exists():
        print("ERROR: No frames found")
        sys.exit(1)

    cmd = [
        "ffmpeg", "-y",
        "-framerate", str(args.fps),
        "-i", str(pattern),
        "-c:v", "libx264",
        "-crf", "18",
        "-pix_fmt", "yuv420p",
        str(output),
    ]

    subprocess.run(cmd)

    if output.exists():
        print(f"\n✓ Created {output}")
        print(f"  Size: {output.stat().st_size / 1024 / 1024:.1f} MB")


if __name__ == "__main__":
    main()
