#!/usr/bin/env python3
"""GodView 30s Demo - Build Script"""

import subprocess
import sys
from pathlib import Path


def run(name, cmd):
    print(f"\n{'='*60}\nSTEP: {name}\n{'='*60}")
    result = subprocess.run(cmd)
    if result.returncode != 0:
        print(f"ERROR: {name} failed")
        sys.exit(result.returncode)
    print(f"✓ {name} complete")


def main():
    script_dir = Path(__file__).parent.absolute()
    out_dir = script_dir / "out"

    print("=" * 60)
    print("GODVIEW 30s DEMO BUILDER")
    print("=" * 60)

    run("Generate Logs", [sys.executable, str(script_dir / "generate_demo_logs.py"), "--out", str(out_dir)])
    run("Render Frames", [sys.executable, str(script_dir / "render_frames.py"), "--out", str(out_dir)])
    run("Encode Video", [sys.executable, str(script_dir / "encode_video.py"), "--out", str(out_dir)])

    final = out_dir / "final_godview_30s.mp4"
    print("\n" + "=" * 60)
    print("BUILD COMPLETE")
    print("=" * 60)
    print(f"Output: {final}")
    if final.exists():
        print(f"Size: {final.stat().st_size / 1024 / 1024:.1f} MB")


if __name__ == "__main__":
    main()
