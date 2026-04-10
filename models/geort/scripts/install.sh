#!/usr/bin/env bash
# GeoRT Phase 0 — install GeoRT into the active conda env.
#
# Idempotent: safe to re-run.
#
# Run from any directory; resolves paths relative to this script.
#
# Pre-requisite: activate the same env you use for dex_retarget. In this
# workspace that is `teleop_operator` (see src/teleop_dev/environment.yaml).
# That env already has numpy<2 / mediapipe==0.10.21 / pinocchio /
# opencv-python — this script only adds the two packages GeoRT needs on
# top: torch + sapien.

set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
GEORT_DIR="${HERE}/../GeoRT"

# 0. Sanity: ensure user has activated *some* env. We do not hard-check
#    teleop_operator because the user may rename it.
if [ -z "${CONDA_DEFAULT_ENV:-}" ]; then
  echo "[install] WARNING: no conda env active. Activate teleop_operator"
  echo "[install]          (or whatever env has dex_retargeting installed)"
  echo "[install]          before re-running this script."
fi

# 1. clone GeoRT (skip if already present)
if [ ! -d "${GEORT_DIR}/.git" ]; then
  echo "[install] cloning GeoRT into ${GEORT_DIR}"
  git clone https://github.com/facebookresearch/GeoRT.git "${GEORT_DIR}"
else
  echo "[install] GeoRT already cloned at ${GEORT_DIR}"
fi

# 2. add GeoRT-specific packages NOT already in teleop_operator.
#    --no-deps where possible to protect mediapipe/numpy pins.
echo "[install] pip install torch torchvision (CPU build by default)"
echo "[install]   for CUDA, override after this script:"
echo "[install]   pip install --force-reinstall torch torchvision \\"
echo "[install]     --index-url https://download.pytorch.org/whl/cu121"
pip install torch torchvision

echo "[install] pip install sapien"
pip install sapien

# 3. GeoRT itself (editable, --no-deps so its requirements.txt does NOT
#    pull in conflicting numpy / mediapipe versions).
cd "${GEORT_DIR}"
echo "[install] pip install -e . --no-deps  (mediapipe / numpy 핀 보호)"
pip install -e . --no-deps

# 4. import verification
echo "[install] verifying imports"
python3 -c "
import geort
from geort import load_model, save_human_data, get_config
import torch
import sapien
import numpy, mediapipe
print('geort      :', geort.__file__)
print('torch      :', torch.__version__, '(cuda:', torch.cuda.is_available(), ')')
print('sapien     :', sapien.__version__)
print('numpy      :', numpy.__version__)
print('mediapipe  :', mediapipe.__version__)
"
echo "[install] done"
