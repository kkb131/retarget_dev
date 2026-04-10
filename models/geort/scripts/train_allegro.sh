#!/usr/bin/env bash
# GeoRT Phase 0 — Allegro right hand IK trainer wrapper.
#
# Usage:
#   bash scripts/train_allegro.sh [HUMAN_TAG] [CKPT_TAG]
#
# 사전 조건:
#   1) `bash scripts/install.sh` 완료
#   2) `python3 ../scripts/collect_mocap_webcam.py --name <HUMAN_TAG>` 로
#      GeoRT/data/<HUMAN_TAG>.npy 생성됨
#   3) CUDA GPU 사용 가능 (geort.trainer 는 .cuda() 를 강제)
#
# 산출물:
#   GeoRT/checkpoint/allegro_right_<timestamp>_<CKPT_TAG>/{epoch_*.pth, last.pth, config.json}
#   GeoRT/checkpoint/allegro_right_last/{epoch_*.pth, last.pth, config.json}

set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
GEORT_DIR="${HERE}/../GeoRT"
HUMAN_TAG="${1:-human_alex}"
CKPT_TAG="${2:-phase0}"

if [ ! -d "${GEORT_DIR}" ]; then
  echo "[train] ${GEORT_DIR} not found — run scripts/install.sh first" >&2
  exit 1
fi

# trainer.py 는 ./checkpoint/, ./data/, ./assets/... 를 CWD 기준 상대 경로로
# 사용한다. 반드시 GeoRT/ 안에서 실행해야 함.
cd "${GEORT_DIR}"

if [ ! -f "data/${HUMAN_TAG}.npy" ]; then
  # geort.utils.path.get_human_data 가 substring match 이므로 정확히 같지
  # 않아도 prefix 만 매칭되면 동작하지만, 가독성을 위해 명시.
  echo "[train] WARNING: data/${HUMAN_TAG}.npy 가 보이지 않습니다."
  echo "[train]          (geort.get_human_data 는 substring match 이므로"
  echo "[train]           prefix 가 같으면 동작할 수도 있음.)"
fi

echo "[train] -hand allegro_right -human_data ${HUMAN_TAG} -ckpt_tag ${CKPT_TAG}"
python3 -m geort.trainer \
  -hand allegro_right \
  -human_data "${HUMAN_TAG}" \
  -ckpt_tag "${CKPT_TAG}"

echo "[train] done — checkpoints under: ${GEORT_DIR}/checkpoint/"
ls -1 "${GEORT_DIR}/checkpoint/" | grep -i allegro || true
