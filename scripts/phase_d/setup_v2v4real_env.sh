#!/bin/bash
# Phase D0: environment for the V2V4Real (OpenCOOD) PointPillars evaluation.
# Everything lives under /workspace/tools so it survives a pod stop.
# Tested on an A40 (compute capability 8.6), driver 595.91.07, 2026-11-10.
set -euo pipefail

T=/workspace/tools
export CONDA_PKGS_DIRS=$T/cache/conda-pkgs PIP_CACHE_DIR=$T/cache/pip
mkdir -p "$CONDA_PKGS_DIRS" "$PIP_CACHE_DIR" "$T/dl-miniforge"

if [ ! -x "$T/miniforge/bin/conda" ]; then
  curl -sSL -o "$T/dl-miniforge/Miniforge3.sh" \
    https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Linux-x86_64.sh
  bash "$T/dl-miniforge/Miniforge3.sh" -b -p "$T/miniforge"
fi

ENV=$T/envs/v2v4real
[ -x "$ENV/bin/python" ] || "$T/miniforge/bin/conda" create -y -p "$ENV" python=3.7
P=$ENV/bin/python

# PyTorch pip wheels bundle the CUDA 11.3 runtime; no local CUDA 11.3 toolkit needed.
$P -m pip install torch==1.12.0+cu113 torchvision==0.13.0+cu113 \
  --extra-index-url https://download.pytorch.org/whl/cu113
$P -m pip install spconv-cu113

[ -d "$T/V2V4Real" ] || git clone https://github.com/ucla-mobility/V2V4Real.git "$T/V2V4Real"
# Commit used for D0: 5a821e13753bafc611f95c47bc1a306acdcb0f7c
cd "$T/V2V4Real"
$P -m pip install -r requirements.txt

# Fix 1: requirements.txt pulls the generic `cumm` (0.5.3), which breaks
# `import spconv.core_cc` (pybind "tv::Tensor type not registered").
# spconv-cu113 needs only cumm-cu113.
$P -m pip uninstall -y cumm
$P -m pip install --force-reinstall --no-deps cumm-cu113==0.4.11

# Cython box overlap extension (CPU only).
$P opencood/utils/setup.py build_ext --inplace

# `python setup.py develop` is replaced by PYTHONPATH=$T/V2V4Real at run time.
PYTHONPATH=$T/V2V4Real $P -c "import spconv.pytorch, opencood.utils.box_overlaps; print('ok')"
