#!/usr/bin/env python3
"""GeoRT Phase 0 — RealSense human-data collector.

GeoRT 의 upstream `geort/mocap/mediapipe_mocap.py` 도 RealSenseCamera 를
사용한 거의 동일한 기능을 제공하지만, 본 프로젝트의
`collect_mocap_webcam.py` 와 같은 인터페이스 (--name / --max-frames) 를
제공하기 위해 thin wrapper 로 작성. 카메라 클래스만 RealSenseCamera 로
교체했고 나머지는 webcam 버전과 동일.

키 컨트롤 (OpenCV 창에 포커스 둔 상태):
    s — 녹화 시작
    e — 녹화 일시정지
    q — 종료 + 저장

Usage (반드시 GeoRT/ 디렉토리에서 실행 — `hand_landmarker.task` 가
GeoRT 루트에 있어야 MediaPipe 가 모델을 로드함):
    cd /workspaces/tamp_ws/src/retarget_dev/models/geort/GeoRT
    python3 ../scripts/collect_mocap_realsense.py --name human_alex
"""

import argparse
import sys

import cv2
import numpy as np

import geort
from geort.mocap.camera.realsense import RealSenseCamera
from geort.mocap.mediapipe_mocap import MediaPipeHandDetector


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Collect MediaPipe + RealSense human hand mocap for GeoRT training."
    )
    parser.add_argument("--name", default="human_realsense",
                        help="Dataset tag (saved as GeoRT/data/<name>.npy).")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--max-frames", type=int, default=0,
                        help="Auto-stop after collecting this many frames "
                             "(0 = unlimited; press 'q' to stop).")
    args = parser.parse_args()

    print(f"[collect] RealSense {args.width}x{args.height}@{args.fps} → tag={args.name}")
    camera = RealSenseCamera(width=args.width, height=args.height, fps=args.fps)
    detector = MediaPipeHandDetector()

    status = "idle"   # idle | recording | quit
    collected: list[np.ndarray] = []

    print("[collect] controls: 's'=start, 'e'=pause, 'q'=quit/save")

    try:
        while True:
            frame = camera.get_frame()
            rgb = frame.get("rgb") if frame else None
            if rgb is None:
                continue

            result = detector.detect(rgb)

            cv2.imshow("geort_collect_mocap_rs", result["annotated_img"])
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
