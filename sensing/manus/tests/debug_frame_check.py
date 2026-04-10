"""Debug helper for the Manus → MANO frame issue.

Prints the (21, 3) keypoint buffer at three pipeline stages so the user
can visually verify whether each step does what it claims:

  1. raw          : positions[:, :3] from the provider, untouched
  2. wrist_shift  : raw - raw[wrist], still in SDK world frame
  3. mano         : wrist_shift @ wrist_rot @ OPERATOR2MANO_RIGHT/LEFT
                    (= what dex-retargeting actually sees)

Use this when adding a new sensing source or when keypoints look off.
A correct fist gesture in MANO frame should make every fingertip move
**in the same direction** (palm normal). If different fingertips move
in different directions across raw vs mano, the frame fix is needed.

Usage
-----
    # Mock data (no hardware)
    cd /workspaces/tamp_ws/src
    python3 -m retarget_dev.sensing.manus.tests.debug_frame_check --provider mock

    # Real ROS2 data (publisher must be running)
    python3 -m retarget_dev.sensing.manus.tests.debug_frame_check --provider ros2

    # SDK subprocess
    python3 -m retarget_dev.sensing.manus.tests.debug_frame_check --provider sdk
"""

from __future__ import annotations

import argparse
import sys
import time

import numpy as np

from retarget_dev.sensing.core.mano_transform import apply_mano_transform
from retarget_dev.sensing.manus.config import WRIST_IDX

# 21-node MANO names — useful for labelled output.
_NAMES = [
    "WRIST",
    "T_CMC", "T_MCP", "T_IP", "T_TIP",
    "I_MCP", "I_PIP", "I_DIP", "I_TIP",
    "M_MCP", "M_PIP", "M_DIP", "M_TIP",
    "R_MCP", "R_PIP", "R_DIP", "R_TIP",
    "P_MCP", "P_PIP", "P_DIP", "P_TIP",
]
_FINGERTIP_IDX = [4, 8, 12, 16, 20]


def _make_provider(kind: str, hand: str):
    if kind == "mock":
        from retarget_dev.sensing.manus.mock_provider import MockManusProvider
        return MockManusProvider(hand_side=hand)
    if kind == "ros2":
        from retarget_dev.sensing.manus.ros2_provider import Ros2ManusProvider
        return Ros2ManusProvider(hand_side=hand)
    if kind == "sdk":
        from retarget_dev.sensing.manus.sdk_provider import SdkManusProvider
        return SdkManusProvider(hand_side=hand)
    raise ValueError(f"unknown provider kind: {kind}")


def _print_buffer(label: str, buf: np.ndarray) -> None:
    print(f"\n=== {label} ===")
    print(f"shape={buf.shape} dtype={buf.dtype}")
    for i in range(21):
        x, y, z = buf[i]
        marker = "  ←tip" if i in _FINGERTIP_IDX else ""
        print(f"  [{i:2d}] {_NAMES[i]:6s}: ({x:+.4f}, {y:+.4f}, {z:+.4f}){marker}")


def _print_fingertip_summary(stage: str, mano_buf: np.ndarray) -> None:
    """Tight one-line summary that reveals frame mismatches at a glance."""
    print(f"\n[{stage}] fingertip MANO coords:")
    print(f"  thumb : {mano_buf[4]}")
    print(f"  index : {mano_buf[8]}")
    print(f"  middle: {mano_buf[12]}")
    print(f"  ring  : {mano_buf[16]}")
    print(f"  pinky : {mano_buf[20]}")
    # Sanity: in MANO, all fingertips should have similar Y component (palm
    # normal). Wide spread → frame fit failed or palm plane was degenerate.
    ys = mano_buf[_FINGERTIP_IDX, 1]
    print(f"  fingertip Y stddev: {float(np.std(ys)):.4f}")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--provider", default="mock", choices=("mock", "ros2", "sdk"))
    ap.add_argument("--hand", default="right", choices=("left", "right"))
    ap.add_argument("--frames", type=int, default=1,
                    help="number of frames to capture (default 1)")
    ap.add_argument("--interval", type=float, default=1.0,
                    help="seconds between frames")
    args = ap.parse_args()

    provider = _make_provider(args.provider, args.hand)
    if hasattr(provider, "start"):
        provider.start()

    captured = 0
    try:
        while captured < args.frames:
            data = provider.get_hand_data()
            if data is None or not data.has_skeleton or data.skeleton is None:
                print("(waiting for data ...)")
                time.sleep(0.1)
                continue

            raw = data.skeleton[:, :3].astype(np.float32)
            wrist_shift = raw - raw[WRIST_IDX]
            mano = apply_mano_transform(wrist_shift, hand_type=args.hand)

            print(f"\n########## frame {captured + 1}/{args.frames} ##########")
            _print_buffer("raw (SDK world)", raw)
            _print_buffer("wrist-shifted (still world rotation)", wrist_shift)
            _print_buffer("MANO frame (after apply_mano_transform)", mano)
            _print_fingertip_summary("MANO", mano)

            captured += 1
            if captured < args.frames:
                time.sleep(args.interval)
    finally:
        if hasattr(provider, "stop"):
            provider.stop()

    return 0


if __name__ == "__main__":
    sys.exit(main())
