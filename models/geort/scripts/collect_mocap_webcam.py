#!/usr/bin/env python3
"""GeoRT Phase 0 — webcam human-data collector.

GeoRT 공식 데이터 수집 스크립트인 `geort/mocap/mediapipe_mocap.py` 는
`RealSenseCamera` 를 하드코딩한다. 본 프로젝트는 RealSense 가 없는
조종 PC 에서도 실행해야 하므로, GeoRT 가 이미 제공하는 `WebcamCamera` +
`MediaPipeHandDetector` + `MediaPipeHandProcessor` 를 그대로 재사용해
캐노니컬 프레임으로 변환된 키포인트를 수집한다.

저장 위치는 `geort.save_human_data(arr, tag)` 가 결정 — 결과적으로
`GeoRT/data/<tag>.npy` 가 생성되며, 이후 `geort.trainer` 가 동일한 이름으로
load 한다.

키 컨트롤 (창에 포커스 둔 상태):
    s — 녹화 시작
    e — 녹화 일시정지
    q — 종료 + 저장

Usage (반드시 GeoRT/ 디렉토리에서 실행 — `hand_landmarker.task` 가
GeoRT 루트에 있어야 MediaPipe 가 모델을 로드함):
    cd /workspaces/tamp_ws/src/retarget_dev/models/geort/GeoRT
    python3 ../scripts/collect_mocap_webcam.py --name human_alex
"""

import argparse
import sys

import cv2
import numpy as np

# GeoRT import — install.sh 가 teleop_operator (또는 활성 conda env) 에 설치.
import geort
from geort.mocap.camera.webcam import WebcamCamera
from geort.mocap.mediapipe_mocap import MediaPipeHandDetector


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Collect MediaPipe + webcam human hand mocap for GeoRT training."
    )
    parser.add_argument("--name", default="human_webcam",
                        help="Dataset tag (saved as GeoRT/data/<name>.npy).")
    parser.add_argument("--device", type=int, default=0,
                        help="OpenCV webcam device index (default: 0).")
    parser.add_argument("--max-frames", type=int, default=0,
                        help="Auto-stop after collecting this many frames "
                             "(0 = unlimited; press 'q' to stop).")
    args = parser.parse_args()

    print(f"[collect] webcam device={args.device} → tag={args.name}")
    camera = WebcamCamera(device_index=args.device)
    detector = MediaPipeHandDetector()

    status = "idle"   # idle | recording | quit
    collected: list[np.ndarray] = []

    print("[collect] controls: 's'=start, 'e'=pause, 'q'=quit/save")

    try:
        while True:
            frame = camera.get_frame()
            rgb = frame.get("rgb")
            if rgb is None:
                continue

            result = detector.detect(rgb)

            cv2.imshow("geort_collect_mocap", result["annotated_img"])
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                status = "quit"
            elif key == ord("s"):
                status = "recording"
                print("[collect] recording started")
            elif key == ord("e"):
                if status == "recording":
                    print("[collect] recording paused")
                status = "idle"

            if status == "recording" and result["detected"]:
                kp = result["canonical_coordinates"]
                if kp is not None and len(kp) == 21:
                    collected.append(np.asarray(kp, dtype=np.float32))
                    if len(collected) % 30 == 0:
                        print(f"[collect] frames={len(collected)}")
                    if args.max_frames and len(collected) >= args.max_frames:
                        print(f"[collect] reached max-frames={args.max_frames}, stopping")
                        status = "quit"

            if status == "quit":
                break

    finally:
        try:
            camera.release()
        except Exception:
            pass
        cv2.destroyAllWindows()

    if not collected:
        print("[collect] no frames captured — nothing to save", file=sys.stderr)
        return 1

    arr = np.stack(collected, axis=0)   # (N, 21, 3)
    save_path = geort.save_human_data(arr, args.name)
    print(f"[collect] saved {arr.shape} → {save_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
