"""Mock Manus data provider for testing without hardware.

Generates sine-wave-based joint angles and a simple skeleton model
that mimics a hand curling and uncurling its fingers.
"""

import math
import time

import numpy as np

from retarget_dev.sensing.manus.config import (
    JOINTS_PER_FINGER,
    NUM_FINGERS,
    NUM_JOINTS,
    NUM_SKELETON_NODES,
)
from retarget_dev.sensing.manus.manus_hand_data import HandData


# Base finger directions (unit vectors from palm, spread fan-like)
_FINGER_DIRECTIONS = np.array([
    [-0.6, 0.0, 0.8],   # Thumb — angled outward
    [-0.15, 0.0, 1.0],  # Index
    [0.0, 0.0, 1.0],    # Middle
    [0.15, 0.0, 1.0],   # Ring
    [0.3, 0.0, 1.0],    # Pinky
], dtype=np.float32)
# Normalize
_FINGER_DIRECTIONS /= np.linalg.norm(_FINGER_DIRECTIONS, axis=1, keepdims=True)

# Approximate bone lengths (meters) per joint segment
_BONE_LENGTHS = np.array([
    [0.035, 0.030, 0.025, 0.020],  # Thumb
    [0.045, 0.035, 0.022, 0.018],  # Index
    [0.045, 0.040, 0.025, 0.018],  # Middle
    [0.045, 0.035, 0.022, 0.018],  # Ring
    [0.040, 0.028, 0.018, 0.015],  # Pinky
], dtype=np.float32)


class MockManusProvider:
    """Sine-wave mock data that mimics a hand curling/uncurling."""

    def __init__(self, hand_side: str = "right"):
        self.hand_side = hand_side
        self._t0 = time.time()

    def get_hand_data(self) -> HandData:
        t = time.time() - self._t0
        angles = np.zeros(NUM_JOINTS, dtype=np.float32)

        for f in range(NUM_FINGERS):
            base = f * JOINTS_PER_FINGER
            freq = 0.5 + f * 0.15
            phase = f * 0.4

            # Spread: small oscillation
            angles[base + 0] = math.sin(t * freq + phase) * 0.15
            # Flexion: wave-like curl (0 → ~1.5 rad)
            curl = (math.sin(t * freq + phase) + 1.0) * 0.5
            angles[base + 1] = curl * 1.2   # MCP
            angles[base + 2] = curl * 1.5   # PIP
            angles[base + 3] = curl * 1.0   # DIP

        spread = np.array(
            [angles[f * JOINTS_PER_FINGER] for f in range(NUM_FINGERS)],
            dtype=np.float32,
        )
        skeleton = self._build_skeleton(angles)

        return HandData(
            joint_angles=angles,
            finger_spread=spread,
            wrist_pos=np.zeros(3, dtype=np.float32),
            wrist_quat=np.array([1, 0, 0, 0], dtype=np.float32),
            hand_side=self.hand_side,
            timestamp=time.time(),
            skeleton=skeleton,
            has_skeleton=True,
        )

    @staticmethod
    def _build_skeleton(angles: np.ndarray) -> np.ndarray:
        """Build a (21, 7) skeleton from joint angles.

        Simple kinematic chain: each joint bends toward the palm plane.
        """
        skel = np.zeros((NUM_SKELETON_NODES, 7), dtype=np.float32)
        # Wrist at origin, identity quaternion
        skel[0] = [0, 0, 0, 1, 0, 0, 0]

        for f in range(NUM_FINGERS):
            base_angle_idx = f * JOINTS_PER_FINGER
            direction = _FINGER_DIRECTIONS[f].copy()
            pos = np.zeros(3, dtype=np.float32)

            for j in range(4):
                node_idx = 1 + f * 4 + j
                bone_len = _BONE_LENGTHS[f, j]

                # Bend the direction downward (toward -y) by flexion angle
                flexion = angles[base_angle_idx + j] if j > 0 else 0.0
                # Rotate direction around x-axis by flexion
                cos_f, sin_f = math.cos(flexion), math.sin(flexion)
                d = direction.copy()
                direction[1] = d[1] * cos_f - d[2] * sin_f
                direction[2] = d[1] * sin_f + d[2] * cos_f
                direction /= (np.linalg.norm(direction) + 1e-8)

                pos = pos + direction * bone_len
                skel[node_idx, :3] = pos
                skel[node_idx, 3] = 1.0  # identity quat w

        return skel
