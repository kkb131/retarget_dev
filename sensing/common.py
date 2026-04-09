"""Common data types and abstract interface for all hand sensing sources.

Every sensing source (phone camera, RealSense D405, Manus glove, etc.)
produces the same HandKeypoints output so that downstream retargeting
code is source-agnostic.
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
import time
from typing import Optional

import numpy as np

# MediaPipe 21-keypoint convention (standard for dex-retargeting / AnyTeleop)
NUM_KEYPOINTS = 21

KEYPOINT_NAMES: list[str] = [
    "WRIST",
    "THUMB_CMC", "THUMB_MCP", "THUMB_IP", "THUMB_TIP",
    "INDEX_MCP", "INDEX_PIP", "INDEX_DIP", "INDEX_TIP",
    "MIDDLE_MCP", "MIDDLE_PIP", "MIDDLE_DIP", "MIDDLE_TIP",
    "RING_MCP", "RING_PIP", "RING_DIP", "RING_TIP",
    "PINKY_MCP", "PINKY_PIP", "PINKY_DIP", "PINKY_TIP",
]

FINGER_TIP_INDICES = [4, 8, 12, 16, 20]

FINGER_GROUPS = {
    "Thumb":  [1, 2, 3, 4],
    "Index":  [5, 6, 7, 8],
    "Middle": [9, 10, 11, 12],
    "Ring":   [13, 14, 15, 16],
    "Pinky":  [17, 18, 19, 20],
}

# Bone connections for skeleton visualization (pairs of keypoint indices)
BONE_CONNECTIONS: list[tuple[int, int]] = [
    # Thumb
    (0, 1), (1, 2), (2, 3), (3, 4),
    # Index
    (0, 5), (5, 6), (6, 7), (7, 8),
    # Middle
    (0, 9), (9, 10), (10, 11), (11, 12),
    # Ring
    (0, 13), (13, 14), (14, 15), (15, 16),
    # Pinky
    (0, 17), (17, 18), (18, 19), (19, 20),
    # Palm
    (5, 9), (9, 13), (13, 17),
]

# Finger colors (Thumb, Index, Middle, Ring, Pinky) — BGR for OpenCV
FINGER_COLORS_BGR = [
    (100, 100, 255),   # Thumb  — red
    (100, 255, 100),   # Index  — green
    (255, 200, 50),    # Middle — blue
    (0, 200, 255),     # Ring   — yellow
    (255, 100, 200),   # Pinky  — pink
]


@dataclass
class HandKeypoints:
    """Unified hand keypoint output — common across all sensing sources.

    Attributes:
        keypoints_3d: (21, 3) xyz positions in meters, wrist at origin.
        keypoints_2d: (21, 2) normalized image coordinates [0..1], or None.
        handedness: "left" or "right".
        confidence: Detection confidence [0, 1].
        timestamp: time.time() when captured.
        source: Identifier for the sensing source.
    """
    keypoints_3d: np.ndarray                   # (21, 3)
    keypoints_2d: Optional[np.ndarray] = None  # (21, 2)
    handedness: str = "right"
    confidence: float = 0.0
    timestamp: float = field(default_factory=time.time)
    source: str = "unknown"


class SensingSource(ABC):
    """Abstract base for any hand sensing input.

    Lifecycle: start() → get_keypoints()/get_frame() loop → stop()
    Supports context manager: ``with source: ...``
    """

    @abstractmethod
    def start(self) -> None:
        """Initialize hardware / open connections."""

    @abstractmethod
    def stop(self) -> None:
        """Release resources."""

    @abstractmethod
    def get_keypoints(self) -> Optional[HandKeypoints]:
        """Get the latest hand keypoints, or None if unavailable."""

    @abstractmethod
    def get_frame(self) -> Optional[np.ndarray]:
        """Get the latest camera frame (BGR), or None."""

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, *args):
        self.stop()
