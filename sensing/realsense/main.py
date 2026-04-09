#!/usr/bin/env python3
"""Standalone RealSense D405 hand sensing with live visualization.

Usage:
    # D405 connected:
    python3 -m retarget_dev.sensing.realsense.main

    # Options:
    python3 -m retarget_dev.sensing.realsense.main \\
        --hand right \\
        --save keypoints.npy \\
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

from retarget_dev.sensing.core.visualizer import HandVisualizer
from retarget_dev.sensing.realsense.realsense_sensing import RealSenseSensing

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(name)s] %(levelname)s: %(message)s",
    datefmt="%H:%M:%S",
)
logger = logging.getLogger(__name__)


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="RealSense D405 hand keypoint sensing (MediaPipe + depth)",
    )
    p.add_argument(
        "--hand", default="right", choices=["left", "right"],
        help="Which hand to track (default: right).",
    )
    p.add_argument(
        "--serial", default=None,
        help="RealSense device serial number (default: auto-detect).",
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
        help="Path to MediaPipe hand_landmarker.task model.",
    )
    return p.parse_args()


def main() -> None:
    args = parse_args()

    try:
        w, h = args.resolution.split("x")
        width, height = int(w), int(h)
    except ValueError:
        logger.error("Invalid resolution format: %s (expected WxH)", args.resolution)
        sys.exit(1)

    sensing = RealSenseSensing(
        serial=args.serial,
        width=width,
        height=height,
        hand_side=args.hand,
        model_path=args.model,
    )

    viz = HandVisualizer(window_name="RealSense D405 Hand Sensing") if not args.no_viz else None
    saved_keypoints: list[np.ndarray] = []

    shutdown = False

    def _signal_handler(sig, frame):
        nonlocal shutdown
        shutdown = True

    signal.signal(signal.SIGINT, _signal_handler)

    logger.info("Starting RealSense sensing (hand=%s) ...", args.hand)
    sensing.start()

    try:
        while not shutdown:
            kp = sensing.get_keypoints()
            frame = sensing.get_frame()

            if frame is None:
                time.sleep(0.01)
                continue

            if args.save and kp is not None:
                saved_keypoints.append(kp.keypoints_3d.copy())

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
