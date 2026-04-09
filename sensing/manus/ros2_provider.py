"""Manus glove data provider via ROS2 topics.

Subscribes to /manus_glove_{0-3} topics published by the manus_data_publisher
C++ node and converts ManusGlove messages to HandData.

No dependency on teleop_dev — standalone implementation.

Requires:
    - manus_ros2 + manus_ros2_msgs packages built and sourced
    - manus_data_publisher running: ros2 run manus_ros2 manus_data_publisher

Usage:
    provider = Ros2ManusProvider(hand_side="right")
    provider.start()
    data = provider.get_hand_data()  # -> HandData or None
    provider.stop()
"""

import logging
import threading
import time
from typing import Optional

import numpy as np

from retarget_dev.sensing.manus.config import NUM_JOINTS, NUM_FINGERS
from retarget_dev.sensing.manus.manus_hand_data import HandData

logger = logging.getLogger(__name__)

# Ergonomics type string → 20-joint index mapping
# Published by manus_data_publisher as ManusErgonomics.type strings
_ERGO_TYPE_MAP = {
    # Thumb (0-3)
    "ThumbMCPSpread": 0, "ThumbMCPStretch": 1,
    "ThumbPIPStretch": 2, "ThumbDIPStretch": 3,
    # Index (4-7)
    "IndexSpread": 4, "IndexMCPStretch": 5,
    "IndexPIPStretch": 6, "IndexDIPStretch": 7,
    # Middle (8-11)
    "MiddleSpread": 8, "MiddleMCPStretch": 9,
    "MiddlePIPStretch": 10, "MiddleDIPStretch": 11,
    # Ring (12-15)
    "RingSpread": 12, "RingMCPStretch": 13,
    "RingPIPStretch": 14, "RingDIPStretch": 15,
    # Pinky (16-19)
    "PinkySpread": 16, "PinkyMCPStretch": 17,
    "PinkyPIPStretch": 18, "PinkyDIPStretch": 19,
}


class Ros2ManusProvider:
    """Receives Manus glove data from ROS2 topics.

    Subscribes to /manus_glove_{0-3} and converts ManusGlove messages
    to HandData with the same interface as MockManusProvider / SdkManusProvider.

    Parameters
    ----------
    hand_side : str
        "left", "right", or "both".
    num_glove_topics : int
        Number of glove topics to subscribe (0..N-1).
    """

    def __init__(self, hand_side: str = "right", num_glove_topics: int = 4):
        self._hand_side = hand_side.lower()
        self._num_topics = num_glove_topics

        # Data cache (thread-safe)
        self._lock = threading.Lock()
        self._left_data: Optional[HandData] = None
        self._right_data: Optional[HandData] = None
        self._data_received = threading.Event()
        self._msg_count = 0

        # ROS2 state
        self._node = None
        self._spin_thread: Optional[threading.Thread] = None
        self._running = False

    def start(self) -> None:
        """Initialize ROS2 node and subscribe to manus_glove topics."""
        import rclpy
        from rclpy.node import Node
        from manus_ros2_msgs.msg import ManusGlove

        if not rclpy.ok():
            rclpy.init()

        self._node = rclpy.create_node('retarget_manus_ros2')

        for i in range(self._num_topics):
            topic = f'/manus_glove_{i}'
            self._node.create_subscription(
                ManusGlove, topic, self._glove_callback, 10,
            )
            logger.debug("Subscribed to %s", topic)

        self._running = True
        self._spin_thread = threading.Thread(target=self._spin_loop, daemon=True)
        self._spin_thread.start()

        logger.info(
            "Ros2ManusProvider started (hand=%s, topics=/manus_glove_{0-%d})",
            self._hand_side, self._num_topics - 1,
        )

        # Wait for first message
        if self._data_received.wait(timeout=10.0):
            logger.info("Receiving data from manus_data_publisher")
        else:
            logger.warning(
                "No data received after 10s. Is manus_data_publisher running?"
            )

    def stop(self) -> None:
        """Shutdown ROS2 node."""
        self._running = False
        if self._node is not None:
            self._node.destroy_node()
            self._node = None
        logger.info("Ros2ManusProvider stopped.")

    def get_hand_data(self, side: Optional[str] = None) -> Optional[HandData]:
        """Get latest HandData for the specified side."""
        target = side or self._hand_side
        with self._lock:
            return self._left_data if target == "left" else self._right_data

    # ── Private ──────────────────────────────────────

    def _spin_loop(self) -> None:
        import rclpy
        try:
            while self._running and rclpy.ok():
                rclpy.spin_once(self._node, timeout_sec=0.05)
        except Exception:
            pass

    def _glove_callback(self, msg) -> None:
        """Convert ManusGlove message → HandData."""
        try:
            hand_side = msg.side.lower() if msg.side else "right"

            # Filter by hand side
            if self._hand_side != "both" and hand_side != self._hand_side:
                return

            # Ergonomics → joint_angles (degrees → radians)
            joint_angles = np.zeros(NUM_JOINTS, dtype=np.float32)
            for ergo in msg.ergonomics:
                idx = _ERGO_TYPE_MAP.get(ergo.type)
                if idx is not None:
                    joint_angles[idx] = np.deg2rad(ergo.value)

            # Finger spread (indices 0, 4, 8, 12, 16)
            finger_spread = np.array([
                joint_angles[f * 4] for f in range(NUM_FINGERS)
            ], dtype=np.float32)

            # Raw skeleton → ndarray[N, 7] (x, y, z, qw, qx, qy, qz)
            skeleton = None
            has_skeleton = False
            if msg.raw_nodes and len(msg.raw_nodes) > 0:
                skel_list = []
                for node in msg.raw_nodes:
                    p = node.pose.position
                    q = node.pose.orientation
                    skel_list.append([p.x, p.y, p.z, q.w, q.x, q.y, q.z])
                skeleton = np.array(skel_list, dtype=np.float32)
                has_skeleton = True

            # Wrist from first skeleton node
            wrist_pos = np.zeros(3, dtype=np.float32)
            wrist_quat = np.array([1.0, 0, 0, 0], dtype=np.float32)
            if skeleton is not None and len(skeleton) > 0:
                wrist_pos = skeleton[0, :3].copy()
                wrist_quat = skeleton[0, 3:].copy()

            hd = HandData(
                joint_angles=joint_angles,
                finger_spread=finger_spread,
                wrist_pos=wrist_pos,
                wrist_quat=wrist_quat,
                hand_side=hand_side,
                timestamp=time.time(),
                skeleton=skeleton,
                has_skeleton=has_skeleton,
            )

            with self._lock:
                if hand_side == "left":
                    self._left_data = hd
                else:
                    self._right_data = hd

            self._msg_count += 1
            self._data_received.set()

        except Exception as e:
            logger.warning("Glove callback error: %s", e)
