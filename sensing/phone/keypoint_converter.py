"""Convert MediaPipe landmarks to MANO-frame 3D keypoints.

MediaPipe ``world_landmarks`` provides 3D positions in meters with origin
at the hand's approximate geometric center, in MediaPipe's camera frame.

For dex-retargeting (AnyTeleop), keypoints must be in the **MANO convention**:
- Wrist at origin
- Palm plane estimated via SVD (stable across camera angles)
- Fixed operator2mano rotation applied

This implementation mirrors ``single_hand_detector.py`` from dex-retargeting's
vector_retargeting example. Without this transform, the optimizer receives
camera-frame vectors and produces incorrect hand orientations.

Reference:
    dex-retargeting/example/vector_retargeting/single_hand_detector.py
    - estimate_frame_from_hand_points()
    - OPERATOR2MANO_RIGHT / OPERATOR2MANO_LEFT

Output: np.ndarray shape (21, 3), float32, meters, wrist = [0, 0, 0],
        aligned to MANO convention.
"""

import numpy as np

from retarget_dev.sensing.core.hand_detector import HandDetection

# Fixed rotation from the operator (wrist-aligned) frame to MANO convention.
# Copied verbatim from dex-retargeting's single_hand_detector.py.
_OPERATOR2MANO_RIGHT = np.array(
    [
        [0, 0, -1],
        [-1, 0, 0],
        [0, 1, 0],
    ],
    dtype=np.float32,
)

_OPERATOR2MANO_LEFT = np.array(
    [
        [0, 0, -1],
        [1, 0, 0],
        [0, -1, 0],
    ],
    dtype=np.float32,
)


def _estimate_wrist_frame(keypoints: np.ndarray) -> np.ndarray:
    """Estimate the wrist coordinate frame from 21-point hand keypoints.

    Uses wrist (0), index MCP (5), middle MCP (9), pinky MCP (17 — optional)
    to define the palm plane via SVD, then orthonormalizes.

    This is the ``estimate_frame_from_hand_points`` function from
    dex-retargeting's single_hand_detector.py, adapted to numpy float32.

    Args:
        keypoints: (21, 3) hand keypoints (already wrist-centered).

    Returns:
        (3, 3) rotation matrix: rows are x/y/z basis vectors of the wrist frame.
    """
    assert keypoints.shape == (21, 3)
    # Use wrist(0), index_MCP(5), middle_MCP(9) to estimate palm plane
    points = keypoints[[0, 5, 9], :]

    # x direction: palm → middle finger base (inverted from dex-retargeting
    # convention: their x_vector = wrist - middle_mcp because of their axis sign)
    x_vector = points[0] - points[2]

    # SVD to find palm plane normal
    centered = points - np.mean(points, axis=0, keepdims=True)
    _u, _s, v = np.linalg.svd(centered)
    normal = v[2, :]

    # Gram-Schmidt: make x orthogonal to normal
    x = x_vector - np.sum(x_vector * normal) * normal
    x = x / (np.linalg.norm(x) + 1e-10)
    z = np.cross(x, normal)

    # Orientation disambiguation: z should roughly point from pinky → index
    if np.sum(z * (points[1] - points[2])) < 0:
        normal = -normal
        z = -z

    frame = np.stack([x, normal, z], axis=1)  # columns = basis vectors
    return frame.astype(np.float32)


class KeypointConverter:
    """Convert MediaPipe HandDetection → MANO-frame (21, 3) keypoints.

    Parameters
    ----------
    hand_type : str
        "right" or "left". Determines which operator2mano rotation is used.
    apply_mano_transform : bool
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
        apply_mano_transform: bool = True,
        extra_rotation: np.ndarray | None = None,
    ):
        self._hand_type = hand_type.lower()
        self._apply_mano = apply_mano_transform
        self._extra_rotation = (
            extra_rotation.astype(np.float32) if extra_rotation is not None else None
        )

        if self._hand_type == "right":
            self._operator2mano = _OPERATOR2MANO_RIGHT
        elif self._hand_type == "left":
            self._operator2mano = _OPERATOR2MANO_LEFT
        else:
            raise ValueError(f"hand_type must be 'right' or 'left', got: {hand_type}")

    def convert(self, detection: HandDetection) -> np.ndarray:
        """Convert a single HandDetection to MANO-frame keypoints.

        Pipeline:
            1. Shift origin to wrist
            2. (optional) Estimate wrist frame via SVD on palm plane
            3. (optional) Apply wrist rotation + operator2mano
            4. (optional) Apply user-defined extra rotation

        Args:
            detection: Raw MediaPipe detection with world_landmarks.

        Returns:
            (21, 3) float32 array in meters, wrist at origin, MANO-aligned.
        """
        wl = detection.world_landmarks.astype(np.float32)  # (21, 3)

        # 1) Shift origin to wrist
        kp = wl - wl[self.WRIST_INDEX]

        if self._apply_mano:
            # 2+3) Estimate wrist frame, align to MANO convention
            wrist_rot = _estimate_wrist_frame(kp)
            kp = kp @ wrist_rot @ self._operator2mano

        if self._extra_rotation is not None:
            kp = (self._extra_rotation @ kp.T).T

        return kp.astype(np.float32)

    @staticmethod
    def extract_2d(detection: HandDetection) -> np.ndarray:
        """Extract normalized 2D image coordinates (21, 2) for visualization."""
        return detection.landmarks_2d[:, :2].astype(np.float32)
