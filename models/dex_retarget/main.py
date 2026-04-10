#!/usr/bin/env python3
"""dex-retargeting based hand retargeting: sensing → optimizer → ROS2 JointState.

Usage:
    # Phone → DG-5F Vector retargeting → IsaacSim
    python3 -m retarget_dev.models.dex_retarget.main \
        --sensing phone --config retarget_dev/models/dex_retarget/config/dg5f_right_vector.yml

    # Phone → DG-5F DexPilot → IsaacSim
    python3 -m retarget_dev.models.dex_retarget.main \
        --sensing phone --config retarget_dev/models/dex_retarget/config/dg5f_right_dexpilot.yml

    # Manus ROS2 → DG-5F
    python3 -m retarget_dev.models.dex_retarget.main \
        --sensing manus-ros2 --config retarget_dev/models/dex_retarget/config/dg5f_right_vector.yml

    # Manus egocentric dataset replay (offline) → DG-5F
    python3 -m retarget_dev.models.dex_retarget.main \
        --sensing manus-egocentric \
        --offline-source /workspaces/tamp_ws/src/manus-egocentric-sample/data/manus-egocentric-sample/data/chunk-000/episode_000000.parquet \
        --config retarget_dev/models/dex_retarget/config/dg5f_right_vector.yml

    # Console only (no ROS2)
    python3 -m retarget_dev.models.dex_retarget.main \
        --sensing manus-mock --config retarget_dev/models/dex_retarget/config/dg5f_right_vector.yml --no-ros2

Controls:
    Ctrl+C — quit
"""

import argparse
import logging
import signal
import time

import numpy as np

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(name)s] %(levelname)s: %(message)s",
    datefmt="%H:%M:%S",
)
logger = logging.getLogger(__name__)


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="dex-retargeting: sensing → DG5F joint commands",
    )
    p.add_argument(
        "--sensing", required=True,
        choices=["manus-mock", "manus-ros2", "manus-sdk", "manus-egocentric",
                 "phone", "realsense"],
        help="Sensing source.",
    )
    p.add_argument(
        "--rs-serial", default=None,
        help="RealSense camera serial number (for --sensing realsense).",
    )
    p.add_argument(
        "--offline-source", default=None,
        help=(
            "Path to a manus-egocentric-sample episode_NNNNNN.parquet file "
            "(required for --sensing manus-egocentric)."
        ),
    )
    p.add_argument(
        "--offline-loop", action="store_true",
        help="Loop the offline episode forever (default: stop at end).",
    )
    p.add_argument(
        "--config", required=True,
        help="dex-retargeting YAML config path.",
    )
    p.add_argument("--hand", default="right", choices=["left", "right"])
    p.add_argument("--hz", type=int, default=30, help="Loop rate (Hz).")
    p.add_argument("--url", default="http://192.168.0.3:8080/video",
                    help="Phone camera URL.")
    p.add_argument("--sdk-bin", default=None, help="Manus SDK binary path.")
    p.add_argument(
        "--topic", default="/dg5f_right/joint_commands",
        help="ROS2 JointState topic.",
    )
    p.add_argument("--no-ros2", action="store_true", help="Disable ROS2.")
    return p.parse_args()


def create_sensing(args):
    if args.sensing == "manus-mock":
        from retarget_dev.sensing.manus.mock_provider import MockManusProvider
        from retarget_dev.sensing.manus.manus_sensing import ManusSensing
        return ManusSensing(MockManusProvider(args.hand), args.hand)

    elif args.sensing == "manus-ros2":
        from retarget_dev.sensing.manus.ros2_provider import Ros2ManusProvider
        from retarget_dev.sensing.manus.manus_sensing import ManusSensing
        return ManusSensing(Ros2ManusProvider(args.hand), args.hand)

    elif args.sensing == "manus-sdk":
        from retarget_dev.sensing.manus.sdk_provider import SdkManusProvider
        from retarget_dev.sensing.manus.manus_sensing import ManusSensing
        return ManusSensing(SdkManusProvider(args.sdk_bin, args.hand), args.hand)

    elif args.sensing == "manus-egocentric":
        if not args.offline_source:
            raise ValueError(
                "--sensing manus-egocentric requires --offline-source "
                "<path to episode_NNNNNN.parquet>"
            )
        from retarget_dev.sensing.manus.offline_egocentric_provider import (
            OfflineEgocentricProvider,
        )
        return OfflineEgocentricProvider(
            parquet_path=args.offline_source,
            hand=args.hand,
            loop=args.offline_loop,
            fps=args.hz,
        )

    elif args.sensing == "phone":
        from retarget_dev.sensing.phone.phone_sensing import PhoneSensing
        return PhoneSensing(camera_url=args.url, hand_side=args.hand)

    elif args.sensing == "realsense":
        from retarget_dev.sensing.realsense.realsense_sensing import RealSenseSensing
        return RealSenseSensing(serial=args.rs_serial, hand_side=args.hand)

    raise ValueError(f"Unknown sensing: {args.sensing}")


def main() -> None:
    args = parse_args()

    sensing = create_sensing(args)

    from retarget_dev.models.dex_retarget.dex_retarget_model import DexRetargetModel
    model = DexRetargetModel(config_path=args.config)
    joint_names = model.get_joint_names()

    # ROS2 publisher
    ros2_node = None
    publisher = None
    if not args.no_ros2:
        try:
            import rclpy
            from rclpy.node import Node
            from sensor_msgs.msg import JointState

            rclpy.init()
            ros2_node = Node("dex_retarget")
            publisher = ros2_node.create_publisher(JointState, args.topic, 10)
            logger.info("ROS2 publisher: %s", args.topic)
        except Exception as e:
            logger.warning("ROS2 init failed: %s", e)

    shutdown = False

    def _signal_handler(sig, frame):
        nonlocal shutdown
        shutdown = True

    signal.signal(signal.SIGINT, _signal_handler)

    sensing.start()
    dt = 1.0 / args.hz
    frame_count = 0

    logger.info("Starting dex-retarget (sensing=%s, hz=%d)", args.sensing, args.hz)

    try:
        while not shutdown:
            kp = sensing.get_keypoints()
            if kp is None:
                time.sleep(dt)
                continue

            q = model.retarget(kp)

            # Publish
            if publisher is not None:
                from sensor_msgs.msg import JointState as JS
                msg = JS()
                msg.header.stamp = ros2_node.get_clock().now().to_msg()
                msg.name = joint_names
                msg.position = q.tolist()
                publisher.publish(msg)

            # Log every ~1 second
            frame_count += 1
            if frame_count % args.hz == 0:
                debug = model.get_debug_info(kp, q)
                angles = debug["dg5f_angles_deg"]
                logger.info(
                    "solve=%.1fms q=[%s]",
                    debug["solve_time_ms"],
                    " ".join(f"{a:+.1f}" for a in angles[:8]) + " ...",
                )

            time.sleep(dt)

    finally:
        sensing.stop()
        if ros2_node is not None:
            ros2_node.destroy_node()
            import rclpy
            if rclpy.ok():
                rclpy.shutdown()
        logger.info("Done.")


if __name__ == "__main__":
    main()
