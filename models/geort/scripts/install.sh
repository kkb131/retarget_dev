#!/usr/bin/env bash
# GeoRT Phase 0 — clone upstream + editable install.
# Idempotent: safe to re-run.
#
# Run from any directory; resolves paths relative to this script.

set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
GEORT_DIR="${HERE}/../GeoRT"

if [ ! -d "${GEORT_DIR}/.git" ]; then
  echo "[install] cloning GeoRT into ${GEORT_DIR}"
  git clone https://github.com/facebookresearch/GeoRT.git "${GEORT_DIR}"
else
  echo "[install] GeoRT already cloned at ${GEORT_DIR}"
fi

cd "${GEORT_DIR}"

# 1. GeoRT 본체 (editable). --no-deps 로 mediapipe / numpy 핀 보호.
echo "[install] pip install -e . --no-deps"
pip install -e . --no-deps

# 2. GeoRT requirements.txt — torch/numpy/sapien 등.
#    environment.yaml 의 핀(numpy<2, mediapipe==0.10.21)을 깨지 않도록
#    환경 차원에서 이미 설치된 패키지는 그대로 두고 부족분만 채운다.
echo "[install] pip install -r requirements.txt"
pip install -r requirements.txt || true

# 3. 검증
echo "[install] verifying imports"
python3 -c "
import geort
from geort import load_model, save_human_data, get_config
import torch
import sapien
print('geort      :', geort.__file__)
print('torch      :', torch.__version__, '(cuda:', torch.cuda.is_available(), ')')
print('sapien     :', sapien.__version__)
"
echo "[install] done"
