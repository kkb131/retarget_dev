#!/usr/bin/env python3
"""Direct Mapping retargeting: sensing → angle extraction → ROS2 JointState publish.

Usage:
    # Manus mock sensing → Direct Mapping → IsaacSim
    python3 -m retarget_dev.models.direct_mapping.main --sensing manus-mock --hand right

    # Phone camera → Direct Mapping → IsaacSim
    python3 -m retarget_dev.models.direct_mapping.main --sensing phone --url http://192.168.0.3:8080/video

    # With config (baseline + scale)
    python3 -m retarget_dev.models.direct_mapping.main --sensing phone --config /workspaces/tamp_ws/src/retarget_dev/models/direct_mapping/config.yaml

    # Auto-calibrate: hold hand open for ~1s, then baseline is captured
    python3 -m retarget_dev.models.direct_mapping.main --sensing phone --calibrate

    # Calibrate and save to config file
    python3 -m retarget_dev.models.direct_mapping.main --sensing phone --calibrate --save-config my_config.yaml

    # Console only (no ROS2)
    python3 -m retarget_dev.models.direct_mapping.main --sensing manus-mock --no-ros2

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
        description="Direct Mapping retargeting: sensing → DG5F joint commands",
    )
    p.add_argument(
        "--sensing", required=True,
        choices=["manus-mock", "manus-ros2", "manus-sdk", "phone"],
        help="Sensing source.",
    )
    p.add_argument("--hand", default="right", choices=["left", "right"])
    p.add_argument("--hz", type=int, default=30, help="Loop rate (Hz).")
    p.add_argument("--url", default="http://192.168.0.3:8080/video", help="Phone camera URL (for --sensing phone).")
    p.add_argument("--sdk-bin", default=None, help="Manus SDK binary path.")
    p.add_argument(
        "--topic", default="/dg5f_right/joint_commands",
        help="ROS2 JointState topic to publish.",
    )
    p.add_argument("--no-ros2", action="store_true", help="Disable ROS2 (console only).")
    p.add_argument("--config", default=None, help="YAML config for baseline/scale/offsets.")
    p.add_argument(
        "--calibrate", action="store_true",
        help="Auto-calibrate: hold hand open for ~1s on startup.",
    )
    p.add_argument(
        "--save-config", default=None, metavar="PATH",
        help="Save calibrated config to YAML after calibration.",
    )
    return p.parse_args()


def create_sensing(args):
    """Create the appropriate SensingSource based on args."""
    if args.sensing == "manus-mock":
        from retarget_dev.sensing.manus.mock_provider import MockManusProvider
        from retarget_dev.sensing.manus.manus_sensing import ManusSensing
        provider = MockManusProvider(hand_side=args.hand)
        return ManusSensing(provider, hand_side=args.hand)

    elif args.sensing == "manus-ros2":
        from retarget_dev.sensing.manus.ros2_provider import Ros2ManusProvider
        from retarget_dev.sensing.manus.manus_sensing import ManusSensing
        provider = Ros2ManusProvider(hand_side=args.hand)
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
    from retarget_dev.models.direct_mapping.direct_mapping_model import DirectMappingModel
    model = DirectMappingModel(
        hand_side=args.hand,
        config_path=args.config,
        auto_calibrate=args.calibrate,
    )
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
            ros2_node = Node("direct_mapping_retarget")
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
    config_saved = False

    # 2-step calibration state machine
    CAL_NONE = "none"
    CAL_WAIT_SPREAD = "wait_spread"  # waiting for Enter to start spread capture
    CAL_SPREAD = "spread"            # collecting open hand frames
    CAL_WAIT_FIST = "wait_fist"      # waiting for Enter to start fist capture
    CAL_FIST = "fist"                # collecting fist frames
    CAL_DONE = "done"

    cal_state = CAL_WAIT_SPREAD if args.calibrate else CAL_NONE
    cal_frames: list[np.ndarray] = []
    cal_count = 30  # frames to collect per step

    logger.info(
        "Starting Direct Mapping (sensing=%s, hand=%s, hz=%d, calibrate=%s)",
        args.sensing, args.hand, args.hz, args.calibrate,
    )
    if args.calibrate:
        logger.info("=== Step 1/2: Hold hand OPEN (spread), then press [Enter] ===")

    try:
        while not shutdown:
            kp = sensing.get_keypoints()
            if kp is None:
                time.sleep(dt)
                continue

            # ── Calibration state machine ──
            if cal_state == CAL_WAIT_SPREAD:
                input()  # block until Enter
                cal_state = CAL_SPREAD
                logger.info("Capturing spread... (%d frames)", cal_count)
                continue

            if cal_state == CAL_SPREAD:
                cal_frames.append(kp.keypoints_3d.copy())
                if len(cal_frames) >= cal_count:
                    model.calibrate_from_frames(cal_frames)
                    cal_frames.clear()
                    cal_state = CAL_WAIT_FIST
                    logger.info(
                        "=== Step 2/2: Now make a FIST, then press [Enter] ==="
                    )
                time.sleep(dt)
                continue

            if cal_state == CAL_WAIT_FIST:
                input()  # block until Enter
                cal_state = CAL_FIST
                logger.info("Capturing fist... (%d frames)", cal_count)
                continue

            if cal_state == CAL_FIST:
                cal_frames.append(kp.keypoints_3d.copy())
                if len(cal_frames) >= cal_count:
                    from retarget_dev.models.direct_mapping.angle_extractor import extract_all_angles as _ea
                    fist_frames_angles = np.array([_ea(f) for f in cal_frames])
                    fist_avg = np.mean(fist_frames_angles, axis=0)
                    model._compute_scale_from_fist(fist_avg)
                    cal_frames.clear()
                    cal_state = CAL_DONE
                    logger.info("=== Calibration complete! Running... ===")

                    if args.save_config:
                        model.save_config(args.save_config)
                        config_saved = True
                        logger.info("Config saved to %s", args.save_config)
                time.sleep(dt)
                continue

            # ── Normal retarget loop ──
            q = model.retarget(kp)

            # Publish via ROS2
            if publisher is not None and model.is_calibrated:
                from sensor_msgs.msg import JointState as JS
                msg = JS()
                msg.header.stamp = ros2_node.get_clock().now().to_msg()
                msg.name = joint_names
                msg.position = q.tolist()
                publisher.publish(msg)

            # Log every ~1 second
            frame_count += 1
            if frame_count % args.hz == 0 and model.is_calibrated:
                debug = model.get_debug_info(kp, q)
                lines = []
                for f_idx, pf in enumerate(debug["per_finger"]):
                    base = f_idx * 4
                    delta = debug["delta_deg"]
                    dg5f = debug["dg5f_angles_deg"]
                    scale = debug["scale_factors"]
                    lines.append(
                        f"  {pf['name']:>6}: "
                        f"delta=[{delta[base+1]:5.1f}° {delta[base+2]:5.1f}° "
                        f"{delta[base+3]:5.1f}°] "
                        f"x[{scale[base+1]:.2f} {scale[base+2]:.2f} {scale[base+3]:.2f}] "
                        f"→ dg5f=[{dg5f[base+1]:5.1f}° {dg5f[base+2]:5.1f}° "
                        f"{dg5f[base+3]:5.1f}°]"
                    )
                logger.info(
                    "Retarget (0°=spread, +°=flexion):\n%s", "\n".join(lines),
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
