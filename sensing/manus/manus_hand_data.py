"""Manus glove hand data container (independent of teleop_dev)."""

from dataclasses import dataclass, field
from typing import Optional

import numpy as np

from retarget_dev.sensing.manus.config import NUM_FINGERS, NUM_JOINTS


@dataclass
class HandData:
    """Hand tracking data from a single Manus glove.

    Mirrors teleop_dev's HandData fields but is self-contained.
    """
    joint_angles: np.ndarray = field(
        default_factory=lambda: np.zeros(NUM_JOINTS, dtype=np.float32))
    finger_spread: np.ndarray = field(
        default_factory=lambda: np.zeros(NUM_FINGERS, dtype=np.float32))
    wrist_pos: np.ndarray = field(
        default_factory=lambda: np.zeros(3, dtype=np.float32))
    wrist_quat: np.ndarray = field(
        default_factory=lambda: np.array([1.0, 0, 0, 0], dtype=np.float32))
    hand_side: str = "right"
    timestamp: float = 0.0
    skeleton: Optional[np.ndarray] = None   # (21, 7) [x,y,z,qw,qx,qy,qz]
    has_skeleton: bool = False
