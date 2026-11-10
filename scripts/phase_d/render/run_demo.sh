#!/usr/bin/env bash
# Render the GodView demo clip on the pod (GPU).
#   scripts/phase_d/render/run_demo.sh PACKAGE_DIR OUT_DIR BLENDER_BIN [SAMPLES]
set -euo pipefail
PKG=$1; OUT=$2; BLENDER=$3; SAMPLES=${4:-128}
HERE=$(cd "$(dirname "$0")" && pwd)
mkdir -p "$OUT"
start=$(date +%s)
# 1. paused-time orbit (150 frames)
"$BLENDER" -b --python "$HERE/build_scene.py" -- --package "$PKG" --outdir "$OUT/orbit" \
  --res 1920x1080 --samples "$SAMPLES" --device GPU --points 300000 \
  --freeze 145 --orbit-frames 150 --shot orbit --animation
echo "orbit rendered after $(( $(date +%s) - start )) s"
# 2. real-time view from above, dataset frames 145 to 205 (181 frames)
"$BLENDER" -b --python "$HERE/build_scene.py" -- --package "$PKG" --outdir "$OUT/live" \
  --res 1920x1080 --samples "$SAMPLES" --device GPU --points 300000 \
  --shot live --range 145,205 --animation
echo "live rendered after $(( $(date +%s) - start )) s"
# 3. dashcam overlays and stills
python3 "$HERE/overlay.py" --package "$PKG" --outdir "$OUT/overlay" --no-hud
python3 "$HERE/overlay.py" --package "$PKG" --outdir "$OUT/overlay_hud" --frames 150,160,170
# 4. the cut, both formats
for SIZE in 1920x1080 1080x1350; do
  NAME=$([ "$SIZE" = 1920x1080 ] && echo 16x9 || echo 4x5)
  python3 "$HERE/make_clip.py" --dashcam "$PKG/cameras/1/cam1" --overlay "$OUT/overlay" \
    --orbit "$OUT/orbit" --live "$OUT/live" --out "$OUT/godview_demo_$NAME.mp4" --size "$SIZE"
done
echo "done in $(( $(date +%s) - start )) s"
