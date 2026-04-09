#!/usr/bin/env python3
"""Fingertip IK retargeting: sensing → IK → ROS2 JointState publish.

Usage:
    # Manus mock sensing → Fingertip IK → IsaacSim
    python3 -m retarget_dev.models.fingertip_ik.main --sensing manus-mock --hand right

    # Phone camera → Fingertip IK → IsaacSim
    python3 -m retarget_dev.models.fingertip_ik.main --sensing phone --url http://192.168.0.3:8080/video --hand right

    # Manus SDK → Fingertip IK → IsaacSim
    python3 -m retarget_dev.models.fingertip_ik.main \\
        --sensing manus-sdk --sdk-bin path/to/SDKClient_Linux.out --hand right

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
        description="Fingertip IK retargeting: sensing → DG5F joint commands",
    )
    p.add_argument(
        "--sensing", required=True,
        choices=["manus-mock", "manus-sdk", "phone"],
        help="Sensing source.",
    )
    p.add_argument("--hand", default="right", choices=["left", "right"])
    p.add_argument("--hz", type=int, default=30, help="Loop rate (Hz).")
    p.add_argument("--url", default="0", help="Phone camera URL (for --sensing phone).")
    p.add_argument("--sdk-bin", default=None, help="Manus SDK binary path.")
    p.add_argument(
        "--topic", default="/dg5f_right/joint_commands",
        help="ROS2 JointState topic to publish.",
    )
    p.add_argument("--no-ros2", action="store_true", help="Disable ROS2 (console only).")
    return p.parse_args()


def create_sensing(args):
    """Create the appropriate SensingSource based on args."""
    if args.sensing == "manus-mock":
        from retarget_dev.sensing.manus.mock_provider import MockManusProvider
        from retarget_dev.sensing.manus.manus_sensing import ManusSensing
        provider = MockManusProvider(hand_side=args.hand)
        return ManusSensing(provider, hand_side=args.hand)

    elif args.sensing == "manus-sdk":
        from retarget_dev.sensing.manus.sdk_provider import SdkManusProvider
        from retarget_dev.sensing.manus.manus_sensing import ManusSensing
        provider = SdkManusProvider(sdk_bin_path=args.sdk_bin, hand_side=args.hand)
        return ManusSensing(provider, hand_side=args.hand)

    elif args.sensing == "phone":
        from retarget_dev.sensing.phone.phone_sensing import PhoneSensing
        return PhoneSensing(camera_url=args.url, hand_side=args.hand)

    raise ValueError(f"Unknown sensing: {args.sensing}")


def main() -> None:
    args = parse_args()

    # Create sensing source
    sensing = create_sensing(args)

    # Create retargeting model
    from retarget_dev.models.fingertip_ik.fingertip_ik_model import FingertipIKModel
    model = FingertipIKModel(hand_side=args.hand)
    joint_names = model.get_joint_names()

    # ROS2 publisher (optional)
    ros2_node = None
    publisher = None
    if not args.no_ros2:
        try:
            import rclpy
            from rclpy.node import Node
            from sensor_msgs.msg import JointState

            rclpy.init()
            ros2_node = Node("fingertip_ik_retarget")
            publisher = ros2_node.create_publisher(JointState, args.topic, 10)
            logger.info("ROS2 publisher: %s (JointState)", args.topic)
        except Exception as e:
            logger.warning("ROS2 init failed: %s (running without ROS2)", e)
            ros2_node = None

    # Graceful shutdown
    shutdown = False

    def _signal_handler(sig, frame):
        nonlocal shutdown
        shutdown = True

    signal.signal(signal.SIGINT, _signal_handler)

    # Start sensing
    sensing.start()
    dt = 1.0 / args.hz
    frame_count = 0

    logger.info("Starting Fingertip IK (sensing=%s, hand=%s, hz=%d)", args.sensing, args.hand, args.hz)

    try:
        while not shutdown:
            kp = sensing.get_keypoints()
            if kp is None:
                time.sleep(dt)
                continue

            # Retarget
            q = model.retarget(kp)

            # Publish via ROS2
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
                logger.info(
                    "solve=%.1fms err=%.1f° [%.1f° %.1f° %.1f° %.1f° %.1f°]",
                    debug["solve_time_ms"],
                    debug["mean_error_deg"],
                    *debug["angle_errors_deg"],
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
