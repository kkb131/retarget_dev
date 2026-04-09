"""Abstract base class for hand retargeting models.

All retargeting models (Fingertip IK, AnyTeleop, etc.) implement this
interface so they can be swapped interchangeably in the pipeline:

    SensingSource → RetargetingModel.retarget(keypoints) → joint angles
"""

from abc import ABC, abstractmethod
from typing import Optional

import numpy as np

from retarget_dev.sensing.common import HandKeypoints


class RetargetingModel(ABC):
    """Abstract base for hand retargeting: HandKeypoints → robot joint angles."""

    @abstractmethod
    def retarget(self, keypoints: HandKeypoints) -> np.ndarray:
        """Convert hand keypoints to robot joint angles.

        Args:
            keypoints: 21-point hand keypoints (wrist-frame, meters).

        Returns:
            Joint angles array (e.g., 20 for DG-5F) in radians.
        """

    @abstractmethod
    def get_joint_names(self) -> list[str]:
        """Return ordered list of robot joint names."""

    def get_debug_info(self, keypoints: HandKeypoints, q: np.ndarray) -> Optional[dict]:
        """Return optional debug info (errors, timing, etc.). Override if needed."""
        return None
