#!/usr/bin/env python3
"""Standalone phone camera hand sensing with live visualization.

Usage:
    # Local webcam (no phone needed — for testing):
    python3 -m retarget_dev.sensing.phone.main

    # Android phone via IP Webcam app (MJPEG):
    python3 -m retarget_dev.sensing.phone.main --url http://192.168.0.3:8080/video

    # Debug mode: print all 21 keypoints with positions and joint angles:
    python3 -m retarget_dev.sensing.phone.main --debug --debug-interval 30

    # Options:
    python3 -m retarget_dev.sensing.phone.main \
        --url http://192.168.0.3:8080/video \
        --hand right \
        --mirror \
        --save keypoints.npy \
        --no-viz

Controls:
    ESC — quit
"""

import argparse
import logging
import signal
import sys
import time

import numpy as np

from retarget_dev.sensing.common import HandKeypoints, KEYPOINT_NAMES
from retarget_dev.sensing.phone.phone_sensing import PhoneSensing
from retarget_dev.sensing.core.visualizer import HandVisualizer

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(name)s] %(levelname)s: %(message)s",
    datefmt="%H:%M:%S",
)
logger = logging.getLogger(__name__)

# Finger bone chains: each finger's keypoint indices from wrist to tip
_FINGER_CHAINS = [
    [0, 1, 2, 3, 4],       # Thumb
    [0, 5, 6, 7, 8],       # Index
    [0, 9, 10, 11, 12],    # Middle
    [0, 13, 14, 15, 16],   # Ring
    [0, 17, 18, 19, 20],   # Pinky
]


def _compute_joint_angles(kp3d: np.ndarray) -> dict[int, float]:
    """Compute flexion angle at each joint from 3D positions.

    For joint J with parent P and child C:
        angle = arccos(dot(P->J, J->C) / (|P->J| * |J->C|))
    Result is in degrees. 180° = fully extended, 0° = fully flexed.

    Returns dict mapping keypoint index → angle in degrees.
    Wrist (0) and tip joints (4,8,12,16,20) have no angle.
    """
    angles = {}
    for chain in _FINGER_CHAINS:
        # chain[0] is wrist (always index 0)
        # chain[1..3] are joints with both parent and child
        # chain[4] is tip (no child)
        for i in range(1, len(chain) - 1):
            parent, joint, child = chain[i - 1], chain[i], chain[i + 1]
            v_in = kp3d[joint] - kp3d[parent]
            v_out = kp3d[child] - kp3d[joint]
            n_in = np.linalg.norm(v_in)
            n_out = np.linalg.norm(v_out)
            if n_in < 1e-8 or n_out < 1e-8:
                angles[joint] = 0.0
                continue
            cos_a = np.dot(v_in, v_out) / (n_in * n_out)
            angles[joint] = float(np.degrees(np.arccos(np.clip(cos_a, -1, 1))))
    return angles


def _print_keypoint_table(kp: HandKeypoints, frame_num: int) -> None:
    """Print formatted table of all 21 keypoints with positions and angles."""
    kp3d = kp.keypoints_3d  # (21, 3) meters, wrist at origin
    angles = _compute_joint_angles(kp3d)

    print(f"\n=== Keypoints (frame {frame_num}, hand={kp.handedness}, conf={kp.confidence:.2f}) ===")
    print(f"     (Flex: 0\u00b0=extended, +\u00b0=flexion)")
    print(f" {'Idx':>3}  {'Name':<16} {'X(mm)':>7} {'Y(mm)':>7} {'Z(mm)':>7}  {'Flex':>7}")
    print(f" {'---':>3}  {'----':<16} {'-----':>7} {'-----':>7} {'-----':>7}  {'----':>7}")

    for idx in range(21):
        x_mm = kp3d[idx, 0] * 1000
        y_mm = kp3d[idx, 1] * 1000
        z_mm = kp3d[idx, 2] * 1000
        name = KEYPOINT_NAMES[idx] if idx < len(KEYPOINT_NAMES) else f"KP_{idx}"

        if idx in angles:
            angle_str = f"{angles[idx]:6.1f}\u00b0"
        else:
            angle_str = "    ---"

        print(f" {idx:3d}  {name:<16} {x_mm:7.1f} {y_mm:7.1f} {z_mm:7.1f}  {angle_str}")


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Phone camera hand keypoint sensing (MediaPipe)",
    )
    p.add_argument(
        "--url", default="0",
        help="Camera source: device index (0), IP Webcam URL, RTSP URL, or video file path.",
    )
    p.add_argument(
        "--hand", default="right", choices=["left", "right"],
        help="Which hand to track (default: right).",
    )
    p.add_argument(
        "--mirror", action="store_true",
        help="Flip frame horizontally (for front-facing camera).",
    )
    p.add_argument(
        "--resolution", default="640x480",
        help="Capture resolution WxH (default: 640x480).",
    )
    p.add_argument(
        "--save", default=None, metavar="PATH",
        help="Save all keypoints to .npy file on exit.",
    )
    p.add_argument(
        "--no-viz", action="store_true",
        help="Disable visualization window (headless mode).",
    )
    p.add_argument(
        "--model", default=None,
        help="Path to MediaPipe hand_landmarker.task model (default: auto-download).",
    )
    p.add_argument(
        "--debug", action="store_true",
        help="Print all 21 keypoints with positions and joint angles.",
    )
    p.add_argument(
        "--debug-interval", type=int, default=30,
        help="Print debug table every N frames (default: 30).",
    )
    return p.parse_args()


def main() -> None:
    args = parse_args()

    # Parse resolution
    try:
        w, h = args.resolution.split("x")
        resolution = (int(w), int(h))
    except ValueError:
        logger.error("Invalid resolution format: %s (expected WxH)", args.resolution)
        sys.exit(1)

    sensing = PhoneSensing(
        camera_url=args.url,
        resolution=resolution,
        mirror=args.mirror,
        hand_side=args.hand,
        model_path=args.model,
    )

    viz = HandVisualizer() if not args.no_viz else None
    saved_keypoints: list[np.ndarray] = []

    # Graceful shutdown on Ctrl+C
    shutdown = False

    def _signal_handler(sig, frame):
        nonlocal shutdown
        shutdown = True

    signal.signal(signal.SIGINT, _signal_handler)

    logger.info("Starting phone sensing (url=%s, hand=%s) ...", args.url, args.hand)
    sensing.start()

    frame_count = 0

    try:
        while not shutdown:
            # Non-blocking: returns latest detection result + matching frame
            kp = sensing.get_keypoints()
            frame = sensing.get_frame()

            if frame is None:
                time.sleep(0.01)
                continue

            frame_count += 1

            if args.save and kp is not None:
                saved_keypoints.append(kp.keypoints_3d.copy())

            # Debug output
            if args.debug and kp is not None and frame_count % args.debug_interval == 0:
                _print_keypoint_table(kp, frame_count)

            if viz is not None:
                annotated = viz.draw(
                    frame,
                    keypoints=kp,
                    fps=sensing.fps,
                    detection_ms=sensing.detection_time_ms,
                )
                if not viz.show(annotated):
                    break
            else:
                # Headless: throttle to avoid busy-wait
                time.sleep(0.01)

    finally:
        sensing.stop()
        if viz is not None:
            viz.close()

        if args.save and saved_keypoints:
            arr = np.stack(saved_keypoints)  # (N, 21, 3)
            np.save(args.save, arr)
            logger.info("Saved %d frames to %s (shape: %s)", len(saved_keypoints), args.save, arr.shape)

        logger.info("Done.")


if __name__ == "__main__":
    main()
