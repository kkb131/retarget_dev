#!/usr/bin/env python3
"""Standalone Manus glove hand sensing.

Usage:
    # Mock mode (no hardware):
    python3 -m retarget_dev.sensing.manus.main --mock --hand right

    # ROS2 mode (manus_data_publisher running):
    python3 -m retarget_dev.sensing.manus.main --ros2 --hand right

    # SDK mode (glove connected):
    python3 -m retarget_dev.sensing.manus.main \\
        --sdk-bin sensing/manus/sdk/SDKClient_Linux.out --hand right

    # Save keypoints:
    python3 -m retarget_dev.sensing.manus.main --mock --save keypoints.npy

Controls:
    Ctrl+C — quit
"""

import argparse
import logging
import signal
import sys
import time

import numpy as np

from retarget_dev.sensing.manus.config import FINGERTIP_INDICES
from retarget_dev.sensing.manus.manus_sensing import ManusSensing

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(name)s] %(levelname)s: %(message)s",
    datefmt="%H:%M:%S",
)
logger = logging.getLogger(__name__)


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Manus glove hand keypoint sensing",
    )
    mode = p.add_mutually_exclusive_group(required=True)
    mode.add_argument(
        "--mock", action="store_true",
        help="Use mock data (sine-wave, no hardware).",
    )
    mode.add_argument(
        "--ros2", action="store_true",
        help="Subscribe to /manus_glove_* ROS2 topics.",
    )
    mode.add_argument(
        "--sdk-bin", default=None, metavar="PATH",
        help="Path to SDKClient_Linux.out binary.",
    )
    p.add_argument(
        "--hand", default="right", choices=["left", "right"],
        help="Which hand to track (default: right).",
    )
    p.add_argument(
        "--hz", type=int, default=60,
        help="Polling rate in Hz (default: 60).",
    )
    p.add_argument(
        "--save", default=None, metavar="PATH",
        help="Save all keypoints to .npy file on exit.",
    )
    return p.parse_args()


def main() -> None:
    args = parse_args()

    # Create provider
    if args.mock:
        from retarget_dev.sensing.manus.mock_provider import MockManusProvider
        provider = MockManusProvider(hand_side=args.hand)
        logger.info("Using MOCK provider")
    elif args.ros2:
        from retarget_dev.sensing.manus.ros2_provider import Ros2ManusProvider
        provider = Ros2ManusProvider(hand_side=args.hand)
        logger.info("Using ROS2 provider (/manus_glove_* topics)")
    else:
        from retarget_dev.sensing.manus.sdk_provider import SdkManusProvider
        provider = SdkManusProvider(sdk_bin_path=args.sdk_bin, hand_side=args.hand)
        logger.info("Using SDK provider: %s", args.sdk_bin)

    sensing = ManusSensing(provider, hand_side=args.hand)
    saved_keypoints: list[np.ndarray] = []

    shutdown = False

    def _signal_handler(sig, frame):
        nonlocal shutdown
        shutdown = True

    signal.signal(signal.SIGINT, _signal_handler)

    sensing.start()
    dt = 1.0 / args.hz
    frame_count = 0

    try:
        while not shutdown:
            kp = sensing.get_keypoints()

            if kp is not None:
                if args.save:
                    saved_keypoints.append(kp.keypoints_3d.copy())

                frame_count += 1
                if frame_count % args.hz == 0:  # print every ~1 second
                    tip_dists = np.linalg.norm(
                        kp.keypoints_3d[FINGERTIP_INDICES], axis=1
                    )
                    logger.info(
                        "hand=%s tips_cm=[%.1f %.1f %.1f %.1f %.1f]",
                        kp.handedness,
                        *(tip_dists * 100),
                    )

            time.sleep(dt)

    finally:
        sensing.stop()

        if args.save and saved_keypoints:
            arr = np.stack(saved_keypoints)
            np.save(args.save, arr)
            logger.info("Saved %d frames to %s (shape: %s)", len(saved_keypoints), args.save, arr.shape)

        logger.info("Done.")


if __name__ == "__main__":
    main()
