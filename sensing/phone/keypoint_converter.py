"""Convert MediaPipe landmarks to MANO-frame 3D keypoints.

MediaPipe ``world_landmarks`` provides 3D positions in meters with origin
at the hand's approximate geometric center, in MediaPipe's camera frame.

For dex-retargeting (AnyTeleop), keypoints must be in the **MANO convention**.
The actual transform is delegated to :mod:`sensing.core.mano_transform` so the
same logic is reused by RealSense and any future sensing source.

Output: np.ndarray shape (21, 3), float32, meters, wrist = [0, 0, 0],
        aligned to MANO convention.
"""

import numpy as np

from retarget_dev.sensing.core.hand_detector import HandDetection
from retarget_dev.sensing.core.mano_transform import apply_mano_transform


class KeypointConverter:
    """Convert MediaPipe HandDetection → MANO-frame (21, 3) keypoints.

    Parameters
    ----------
    hand_type : str
        "right" or "left". Determines which operator2mano rotation is used.
    apply_mano : bool
        If True (default), apply the full MANO transform required by
        dex-retargeting. Set to False for raw wrist-centered output.
    extra_rotation : np.ndarray or None
        Optional additional (3, 3) rotation applied after MANO transform
        (for user-defined calibration).
    """

    WRIST_INDEX = 0

    def __init__(
        self,
        hand_type: str = "right",
        apply_mano: bool = True,
        extra_rotation: np.ndarray | None = None,
    ):
        self._hand_type = hand_type.lower()
        self._apply_mano = apply_mano
        self._extra_rotation = (
            extra_rotation.astype(np.float32) if extra_rotation is not None else None
        )

        if self._hand_type not in ("right", "left"):
            raise ValueError(f"hand_type must be 'right' or 'left', got: {hand_type}")

    def convert(self, detection: HandDetection) -> np.ndarray:
        """Convert a single HandDetection to MANO-frame keypoints.

        Args:
            detection: Raw MediaPipe detection with world_landmarks.

        Returns:
            (21, 3) float32 array in meters, wrist at origin, MANO-aligned.
        """
        wl = detection.world_landmarks.astype(np.float32)
        # Shift origin to wrist
        kp = wl - wl[self.WRIST_INDEX]

        if self._apply_mano:
            kp = apply_mano_transform(kp, hand_type=self._hand_type)

        if self._extra_rotation is not None:
            kp = (self._extra_rotation @ kp.T).T

        return kp.astype(np.float32)

    @staticmethod
    def extract_2d(detection: HandDetection) -> np.ndarray:
        """Extract normalized 2D image coordinates (21, 2) for visualization."""
        return detection.landmarks_2d[:, :2].astype(np.float32)
