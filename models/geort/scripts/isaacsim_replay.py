#!/usr/bin/env python3
"""GeoRT Phase 0 — replay trained Allegro model into Isaac Sim.

Vulkan 미지원 환경이라 GeoRT 의 SAPIEN 뷰어는 동작하지 않는다. 대신
이미 Isaac Sim 에 로드된 Allegro hand 에 `JointState` 를 publish 하여
시각화한다 (Isaac Sim 측에서 `/allegro_right/joint_commands` 를 구독해야 함).

입력 키포인트는 GeoRT 의 캐노니컬 프레임(+Y=thumb, +Z=middle) 으로 정렬된
(21, 3) numpy. 두 가지 소스 지원:
    --source file   : `GeoRT/data/<tag>.npy` 같은 사전 수집 mocap 재생
    --source webcam : 라이브 웹캠 + GeoRT MediaPipeHandDetector

joint name 순서는 학습 시 저장된 checkpoint config (joint_order) 에서 읽어
Isaac Sim 의 USD/URDF joint 이름과 일치하는지 사용자가 사전 확인해야 한다.
불일치 시 `docs/isaacsim_allegro_bridge.md` 참조.

Usage:
    cd /workspaces/tamp_ws/src/retarget_dev/models/geort/GeoRT
    # T1: Isaac Sim 에 Allegro hand 로드 + JointState 컨트롤러 활성화
    # T2:
    python3 ../scripts/isaacsim_replay.py \\
        --tag phase0 --source file --data-file data/human_alex.npy

    python3 ../scripts/isaacsim_replay.py --tag phase0 --source webcam
"""

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np

# rclpy 는 ROS2 환경 source 후에만 import 가능 — 지연 import.


def load_joint_names_from_checkpoint(tag: str) -> list[str]:
    """학습 시 저장된 checkpoint/config.json 에서 joint_order 추출.

    `geort.export.load_model` 과 동일하게 `tag` substring 매칭으로 ckpt 디렉토리
    찾는다 (load_model 자체가 model 만 로드하고 config 는 별도 저장하지 않으므로
    여기서 같은 규칙으로 한 번 더 읽음).
    """
    from geort.utils.path import get_checkpoint_root
    ckpt_root = Path(get_checkpoint_root())
    matches = [d for d in ckpt_root.iterdir() if tag in d.name]
    if not matches:
        raise FileNotFoundError(
            f"No checkpoint with tag '{tag}' under {ckpt_root}"
        )
    # geort.load_model 은 첫 매치를 사용하므로 동일하게.
    cfg_path = matches[0] / "config.json"
    cfg = json.loads(cfg_path.read_text())
    return cfg["joint_order"]


def iter_keypoints_from_file(npy_path: Path):
    """저장된 (T, 21, 3) 캐노니컬 프레임 mocap 을 한 프레임씩 yield."""
    arr = np.load(str(npy_path))
    if arr.ndim != 3 or arr.shape[1:] != (21, 3):
        raise ValueError(
            f"Expected (T, 21, 3) keypoints, got shape {arr.shape}"
        )
    for frame in arr:
        yield frame.astype(np.float32)


def iter_keypoints_from_webcam(device_index: int = 0):
    """live 웹캠 → MediaPipeHandDetector → 캐노니컬 프레임 (21, 3)."""
    import cv2
    from geort.mocap.camera.webcam import WebcamCamera
    from geort.mocap.mediapipe_mocap import MediaPipeHandDetector

    cam = WebcamCamera(device_index=device_index)
    det = MediaPipeHandDetector()
    print("[replay] webcam mode — press 'q' in the window to stop", flush=True)
    try:
        while True:
            frame = cam.get_frame()
            rgb = frame.get("rgb")
            if rgb is None:
                continue
            result = det.detect(rgb)
            cv2.imshow("geort_isaacsim_replay", result["annotated_img"])
            if (cv2.waitKey(1) & 0xFF) == ord("q"):
                return
            if result["detected"]:
                kp = result["canonical_coordinates"]
                if kp is not None and len(kp) == 21:
                    yield np.asarray(kp, dtype=np.float32)
    finally:
        try:
            cam.release()
        except Exception:
            pass
        cv2.destroyAllWindows()


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Replay GeoRT-trained Allegro model into Isaac Sim "
                    "via JointState publishing.",
    )
    parser.add_argument("--tag", required=True,
                        help="Checkpoint tag (matches `-ckpt_tag` used at training)")
    parser.add_argument("--epoch", type=int, default=0,
                        help="Epoch number to load (0 = last.pth)")
    parser.add_argument("--source", choices=["file", "webcam"], default="file")
    parser.add_argument("--data-file", default=None,
                        help="(--source file) path to (T,21,3) .npy mocap")
    parser.add_argument("--device", type=int, default=0,
                        help="(--source webcam) OpenCV device index")
    parser.add_argument("--topic", default="/allegro_right/joint_commands",
                        help="ROS2 JointState topic to publish")
    parser.add_argument("--hz", type=int, default=30,
                        help="Replay rate (file mode) — webcam mode is "
                             "limited by detector throughput")
    parser.add_argument("--loop", action="store_true",
                        help="(--source file) loop the .npy indefinitely")
    args = parser.parse_args()

    # GeoRT 모델 로드 (CUDA 필요).
    import geort
    print(f"[replay] loading checkpoint tag={args.tag} epoch={args.epoch}")
    model = geort.load_model(tag=args.tag, epoch=args.epoch)

    joint_names = load_joint_names_from_checkpoint(args.tag)
    print(f"[replay] joint_order ({len(joint_names)}):", joint_names)

    # ROS2 publisher.
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState

    rclpy.init()
    node = Node("geort_isaacsim_replay")
    pub = node.create_publisher(JointState, args.topic, 10)
    print(f"[replay] publishing → {args.topic}")

    def make_msg(qpos: np.ndarray) -> JointState:
        msg = JointState()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.name = joint_names
        msg.position = [float(x) for x in qpos]
        return msg

    dt = 1.0 / max(1, args.hz)
    frame_idx = 0

    def play_iter(it):
        nonlocal frame_idx
        for kp in it:
            t0 = time.perf_counter()
            qpos = model.forward(kp)        # (n_joints,) radians
            pub.publish(make_msg(qpos))
            frame_idx += 1
            if frame_idx % max(1, args.hz) == 0:
                preview = " ".join(f"{x:+.2f}" for x in qpos[:6])
                print(f"\r[replay] frame={frame_idx} q[0:6]=[{preview}]",
                      end="", flush=True)
            if args.source == "file":
                elapsed = time.perf_counter() - t0
                if elapsed < dt:
                    time.sleep(dt - elapsed)

    try:
        if args.source == "file":
            if not args.data_file:
                print("[replay] --data-file is required for --source file",
                      file=sys.stderr)
                return 2
            data_path = Path(args.data_file)
            if not data_path.exists():
                print(f"[replay] data file not found: {data_path}",
                      file=sys.stderr)
                return 2
            while True:
                play_iter(iter_keypoints_from_file(data_path))
                if not args.loop:
                    break
                print("\n[replay] looping...")
        else:
            play_iter(iter_keypoints_from_webcam(device_index=args.device))
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print(f"\n[replay] total frames published: {frame_idx}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
