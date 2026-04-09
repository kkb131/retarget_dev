#!/usr/bin/env python3
"""Play dex-retargeting pkl file to Isaac Sim via ROS2 JointState.

Supports both Allegro and DG-5F (auto-detected from pkl joint names).
Joint name reordering is handled automatically by publishing with names.

Usage:
    # Allegro
    python3 -m retarget_dev.models.dex_retarget.play_pkl \
        --pkl-path data/allegro_joints.pkl --topic /joint_command

    # DG-5F
    python3 -m retarget_dev.models.dex_retarget.play_pkl \
        --pkl-path /tmp/dg5f_joints.pkl --topic /dg5f_right/joint_commands
"""

import argparse
import pickle
import signal
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

# Allegro: dex-retargeting joint names → Isaac Sim joint names
_ALLEGRO_DEX_TO_ISAAC = {
    "joint_0.0": "index_joint_0", "joint_1.0": "index_joint_1",
    "joint_2.0": "index_joint_2", "joint_3.0": "index_joint_3",
    "joint_4.0": "middle_joint_0", "joint_5.0": "middle_joint_1",
    "joint_6.0": "middle_joint_2", "joint_7.0": "middle_joint_3",
    "joint_8.0": "ring_joint_0", "joint_9.0": "ring_joint_1",
    "joint_10.0": "ring_joint_2", "joint_11.0": "ring_joint_3",
    "joint_12.0": "thumb_joint_0", "joint_13.0": "thumb_joint_1",
    "joint_14.0": "thumb_joint_2", "joint_15.0": "thumb_joint_3",
}


def resolve_names(dex_names: list[str]) -> list[str]:
    """Resolve dex-retargeting joint names to Isaac Sim names.

    For DG-5F (rj_dg_*), names are used as-is.
    For Allegro (joint_*.0), mapped via lookup table.
    """
    if dex_names[0].startswith("rj_dg_"):
        # DG-5F: names match directly
        return dex_names
    elif dex_names[0].startswith("joint_"):
        # Allegro: needs mapping
        return [_ALLEGRO_DEX_TO_ISAAC[n] for n in dex_names]
    else:
        # Unknown robot: pass through
        return dex_names


def main():
    parser = argparse.ArgumentParser(description="Play pkl to Isaac Sim")
    parser.add_argument("--pkl-path", required=True, help="Path to .pkl file")
    parser.add_argument("--topic", default="/joint_command", help="ROS2 topic")
    parser.add_argument("--hz", type=int, default=30, help="Playback rate")
    parser.add_argument("--loop", action="store_true", help="Loop playback")
    args = parser.parse_args()

    with open(args.pkl_path, "rb") as f:
        pkl = pickle.load(f)

    meta = pkl["meta_data"]
    data = pkl["data"]
    dex_names = meta["joint_names"]
    isaac_names = resolve_names(dex_names)

    print(f"Loaded {len(data)} frames, {len(dex_names)} DOF")
    print(f"  dex names: {dex_names[:3]} ...")
    print(f"  isaac names: {isaac_names[:3]} ...")
    print(f"Publishing to {args.topic} at {args.hz} Hz")

    rclpy.init()
    node = Node("play_pkl")
    pub = node.create_publisher(JointState, args.topic, 10)

    shutdown = False

    def _sig(sig, frame):
        nonlocal shutdown
        shutdown = True

    signal.signal(signal.SIGINT, _sig)

    dt = 1.0 / args.hz
    frame_idx = 0

    try:
        while not shutdown:
            q = np.array(data[frame_idx])

            msg = JointState()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.name = isaac_names
            msg.position = q.tolist()
            pub.publish(msg)

            frame_idx += 1
            if frame_idx >= len(data):
                if args.loop:
                    frame_idx = 0
                    print(f"Looping... (frame 0/{len(data)})")
                else:
                    print(f"Done. {len(data)} frames played.")
                    break

            if frame_idx % args.hz == 0:
                print(f"Frame {frame_idx}/{len(data)}")

            time.sleep(dt)

    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
