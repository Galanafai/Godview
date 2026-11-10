#!/usr/bin/env bash
# Render the GodView demo clip on the pod (GPU).
#   scripts/phase_d/render/run_demo.sh PACKAGE_DIR OUT_DIR BLENDER_BIN [SAMPLES]
set -euo pipefail
PKG=$1; OUT=$2; BLENDER=$3; SAMPLES=${4:-128}
HERE=$(cd "$(dirname "$0")" && pwd)
mkdir -p "$OUT"
start=$(date +%s)
"$BLENDER" -b --python "$HERE/build_scene.py" -- --package "$PKG" --outdir "$OUT/orbit" \
  --res 1920x1080 --samples "$SAMPLES" --device GPU --points 300000 \
  --freeze 145 --orbit-frames 150 --shot orbit --animation
echo "orbit rendered in $(( $(date +%s) - start )) s"
python3 "$HERE/overlay.py" --package "$PKG" --outdir "$OUT/overlay" --no-hud
python3 "$HERE/overlay.py" --package "$PKG" --outdir "$OUT/overlay_hud" --frames 150,160,170
python3 "$HERE/make_clip.py" --dashcam "$PKG/cameras/1/cam1" --overlay "$OUT/overlay" \
  --orbit "$OUT/orbit" --out "$OUT/godview_demo.mp4"
echo "done in $(( $(date +%s) - start )) s"
