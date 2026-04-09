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

# Manus raw skeleton (chain_type, joint_type) → MANO 21-keypoint index.
#
# The publisher emits 25 raw nodes per hand: 1 wrist + thumb (4 joints, no
# Intermediate) + 4 fingers × 5 joints. MANO uses 21: wrist + 5 × 4 joints.
# The 4 non-thumb "MCP" nodes (= SDK Metacarpal at the base of the metacarpal
# bone) have no MANO counterpart and are dropped.
#
# WARNING: the publisher's joint string labels are anatomically off-by-one
# vs. their semantic meaning (see ManusDataPublisher.cpp:JointTypeToString):
#   SDK enum         publisher string   anatomical
#   Metacarpal       "MCP"              CMC / metacarpal-base
#   Proximal         "PIP"              MCP (knuckle)
#   Intermediate     "IP"               PIP
#   Distal           "DIP"              DIP
#   Tip              "TIP"              fingertip
# We match the publisher strings (the actual wire format), not the labels.
_MANO_REMAP = {
    # Thumb (4 joints — no Intermediate)
    ("Thumb", "MCP"): 1,    # → MANO thumb CMC
    ("Thumb", "PIP"): 2,    # → MANO thumb MCP
    ("Thumb", "DIP"): 3,    # → MANO thumb IP
    ("Thumb", "TIP"): 4,    # → MANO thumb tip
    # Index — drop SDK "MCP" (Metacarpal)
    ("Index", "PIP"): 5,
    ("Index", "IP"): 6,
    ("Index", "DIP"): 7,
    ("Index", "TIP"): 8,
    # Middle
    ("Middle", "PIP"): 9,
    ("Middle", "IP"): 10,
    ("Middle", "DIP"): 11,
    ("Middle", "TIP"): 12,
    # Ring
    ("Ring", "PIP"): 13,
    ("Ring", "IP"): 14,
    ("Ring", "DIP"): 15,
    ("Ring", "TIP"): 16,
    # Pinky
    ("Pinky", "PIP"): 17,
    ("Pinky", "IP"): 18,
    ("Pinky", "DIP"): 19,
    ("Pinky", "TIP"): 20,
}


def _remap_to_mano_21(raw_nodes) -> Optional[np.ndarray]:
    """Convert Manus raw skeleton (~25 nodes) → MANO 21-node (21, 7) layout.

    Uses the (chain_type, joint_type) string pair on each ManusRawNode to
    deterministically place each node, regardless of publish order. The
    wrist comes from any node with chain_type="Hand". The 4 non-thumb
    Metacarpal nodes have no MANO counterpart and are dropped.

    Returns
    -------
    np.ndarray of shape (21, 7) with rows [x, y, z, qw, qx, qy, qz],
    or None if any required MANO slot is unfilled (e.g., partial dropout).
    """
    skel = np.full((21, 7), np.nan, dtype=np.float32)
    for node in raw_nodes:
        if node.chain_type == "Hand":
            idx = 0  # wrist root — joint_type may be "Invalid"
        else:
            idx = _MANO_REMAP.get((node.chain_type, node.joint_type))
            if idx is None:
                continue  # SDK Metacarpal of non-thumb finger → drop
        p = node.pose.position
        q = node.pose.orientation
        skel[idx] = [p.x, p.y, p.z, q.w, q.x, q.y, q.z]

    if np.isnan(skel).any():
        return None
    return skel


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

        # One-shot diagnostic flags for the 25→21 MANO remap
        self._remap_logged_ok = False
        self._remap_logged_fail = False

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

            # Raw skeleton (~25 nodes) → MANO 21-node (21, 7) layout.
            # The publisher forwards all SDK nodes including 4 non-thumb
            # Metacarpal joints that have no MANO counterpart; the helper
            # uses (chain_type, joint_type) metadata to drop them and
            # reorder to wrist=0, thumb=[1..4], index=[5..8], ..., pinky=[17..20].
            skeleton = None
            has_skeleton = False
            if msg.raw_nodes and len(msg.raw_nodes) > 0:
                skeleton = _remap_to_mano_21(msg.raw_nodes)
                has_skeleton = skeleton is not None
                self._diagnose_remap(msg.raw_nodes, skeleton)

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

    def _diagnose_remap(self, raw_nodes, mapped) -> None:
        """One-shot logging for the 25→21 MANO remap result.

        Logs INFO once on the first successful remap, WARNING once on the
        first failure (showing which (chain, joint) pairs were received so
        the user can debug missing fingers / unknown labels).
        """
        if mapped is not None and not self._remap_logged_ok:
            logger.info(
                "Manus raw skeleton: %d raw nodes → MANO 21 (remap OK)",
                len(raw_nodes),
            )
            self._remap_logged_ok = True
        elif mapped is None and not self._remap_logged_fail:
            keys = sorted({(n.chain_type, n.joint_type) for n in raw_nodes})
            logger.warning(
                "Manus raw skeleton remap incomplete — MANO slots unfilled. "
                "Received %d nodes with (chain, joint) pairs: %s",
                len(raw_nodes), keys,
            )
            self._remap_logged_fail = True
