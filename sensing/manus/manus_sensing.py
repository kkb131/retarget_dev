"""Manus glove hand sensing pipeline.

Reads HandData from a provider (MockManusProvider or SdkManusProvider)
and converts skeleton positions to HandKeypoints (21, 3) in MANO frame.

No MediaPipe needed — the glove provides 3D keypoints directly.

The Manus SDK publishes raw skeleton positions in WORLD coordinates
(VUH = X-into-scene, Z-up, right-handed; see ManusDataPublisher.hpp).
After wrist shift the rotation is still world-aligned, so the same
SVD-based palm-plane fit used by phone/realsense is applied to land in
the MANO frame expected by dex-retargeting. See docs/manus_debug.md §3.3.

Usage:
    provider = MockManusProvider(hand_side="right")
    with ManusSensing(provider) as ms:
        while True:
            kp = ms.get_keypoints()
            if kp is not None:
                print(kp.keypoints_3d.shape)  # (21, 3)
"""

import logging
import time
from typing import Optional

import numpy as np

from retarget_dev.sensing.common import HandKeypoints, SensingSource
from retarget_dev.sensing.core.mano_transform import apply_mano_transform
from retarget_dev.sensing.manus.config import WRIST_IDX
from retarget_dev.sensing.manus.manus_hand_data import HandData

logger = logging.getLogger(__name__)


class ManusSensing(SensingSource):
    """Manus glove → HandKeypoints pipeline.

    The provider (mock or SDK) supplies HandData with a skeleton array.
    This class extracts xyz positions and shifts to wrist origin.
    """

    def __init__(self, provider, hand_side: str = "right"):
        """
        Args:
            provider: Object with ``get_hand_data() -> HandData | None``.
                      MockManusProvider or SdkManusProvider.
            hand_side: "left" or "right".
        """
        self._provider = provider
        self._hand_side = hand_side.lower()
        self._latest_keypoints: Optional[HandKeypoints] = None
        self._latest_ergonomics: Optional[np.ndarray] = None

    # ── SensingSource implementation ──────────────────────

    def start(self) -> None:
        if hasattr(self._provider, 'start'):
            self._provider.start()
        logger.info("ManusSensing started (hand=%s)", self._hand_side)

    def stop(self) -> None:
        if hasattr(self._provider, 'stop'):
            self._provider.stop()
        logger.info("ManusSensing stopped.")

    def get_keypoints(self) -> Optional[HandKeypoints]:
        """Get the latest hand keypoints from the Manus provider."""
        if hasattr(self._provider, 'get_hand_data'):
            data = self._provider.get_hand_data()
        else:
            return None

        if data is None:
            return None

        # Store ergonomics for optional access
        self._latest_ergonomics = data.joint_angles

        if not data.has_skeleton or data.skeleton is None:
            return None

        # Extract xyz positions from skeleton (21, 7) → (21, 3)
        positions = data.skeleton[:, :3].astype(np.float32)

        # Shift to wrist origin (still in SDK world frame after this).
        keypoints_3d = positions - positions[WRIST_IDX]

        # Rotate to MANO frame. Manus SDK publishes raw skeleton in world
        # coordinates (see module docstring), so without this step the
        # finger curl direction depends on glove orientation in space and
        # dex-retargeting sees a wrong axis (the historical fist→spread
        # inversion bug). The "manus" convention selects a +y-flipped
        # operator → MANO matrix to compensate for the SDK's VUH chirality;
        # phone/realsense use the "mediapipe" convention via the same helper.
        keypoints_3d = apply_mano_transform(
            keypoints_3d, hand_type=self._hand_side, convention="manus",
        )

        return HandKeypoints(
            keypoints_3d=keypoints_3d,
            keypoints_2d=None,
            handedness=data.hand_side,
            confidence=1.0,
            timestamp=data.timestamp or time.time(),
            source="manus",
        )

    def get_frame(self) -> Optional[np.ndarray]:
        """Manus has no camera frame — always returns None."""
        return None

    # ── Extra accessors ───────────────────────────────────

    @property
    def latest_ergonomics(self) -> Optional[np.ndarray]:
        """Latest joint angles (20,) in radians, if available."""
        return self._latest_ergonomics
