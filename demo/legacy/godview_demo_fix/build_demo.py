#!/usr/bin/env python3
"""
GodView Demo Fix - Build Script
================================
One command runs the complete pipeline:
1. Generate logs
2. Render frames
3. Encode video

Usage:
    python3 build_demo.py
"""

import argparse
import subprocess
import sys
from pathlib import Path


def run_step(name: str, cmd: list):
    print(f"\n{'='*60}")
    print(f"STEP: {name}")
    print(f"{'='*60}")
    print(f"Command: {' '.join(cmd)}")
    print()

    result = subprocess.run(cmd)
    if result.returncode != 0:
        print(f"ERROR: {name} failed with code {result.returncode}")
        sys.exit(result.returncode)

    print(f"✓ {name} complete")


def main():
    parser = argparse.ArgumentParser(description="GodView Demo Fix - Builder")
    parser.add_argument("--out", type=str, default="./out", help="Output directory")
    parser.add_argument("--seed", type=int, default=42, help="Random seed")
    parser.add_argument("--duration_s", type=int, default=85, help="Duration")
    parser.add_argument("--fps", type=int, default=30, help="FPS")
    parser.add_argument("--skip_logs", action="store_true")
    parser.add_argument("--skip_render", action="store_true")
    args = parser.parse_args()

    script_dir = Path(__file__).parent.absolute()
    out_dir = Path(args.out).absolute()

    print("=" * 60)
    print("GODVIEW DEMO FIX - BUILDER")
    print("=" * 60)
    print(f"Output: {out_dir}")
    print(f"Duration: {args.duration_s}s @ {args.fps} FPS")
    print("=" * 60)

    out_dir.mkdir(parents=True, exist_ok=True)

    # Step 1: Generate logs
    if not args.skip_logs:
        run_step("Generate Logs", [
            sys.executable, str(script_dir / "generate_demo_logs.py"),
            "--out", str(out_dir),
            "--seed", str(args.seed),
            "--duration_s", str(args.duration_s),
            "--fps", str(args.fps),
        ])

    # Step 2: Render frames
    if not args.skip_render:
        run_step("Render Frames", [
            sys.executable, str(script_dir / "render_frames.py"),
            "--out", str(out_dir),
            "--fps", str(args.fps),
        ])

    # Step 3: Encode video
    run_step("Encode Video", [
        sys.executable, str(script_dir / "encode_video.py"),
        "--out", str(out_dir),
        "--fps", str(args.fps),
    ])

    final_video = out_dir / "final_godview_demo_fixed.mp4"
    print("\n" + "=" * 60)
    print("BUILD COMPLETE")
    print("=" * 60)
    print(f"Output: {final_video}")
    if final_video.exists():
        size_mb = final_video.stat().st_size / (1024 * 1024)
        print(f"Size: {size_mb:.1f} MB")
    print("=" * 60)


if __name__ == "__main__":
    main()
