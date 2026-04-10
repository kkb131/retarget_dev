"""Shared MANO coordinate frame transform for hand keypoints.

Any sensing source must apply this transform so its keypoints land in the
MANO convention expected by dex-retargeting's optimizers.

Pipeline:
    1. Shift origin to wrist (index 0)
    2. Estimate wrist frame via SVD on palm plane (wrist, index_MCP, middle_MCP)
       — gives an operator basis (x, y, z) where:
         * x_op = wrist - middle_MCP   (points from middle_MCP toward wrist)
         * y_op = palm normal          (after disambiguation)
         * z_op = pinky → index        (forced by the disambiguation step)
    3. Rotate into MANO convention via ``kp @ wrist_rot @ operator2mano``

Two conventions live side by side
---------------------------------
Different sensing sources hand the SVD palm-plane fit data with different
chirality, so the operator → MANO rotation matrix that lands them in the
**same canonical MANO frame** is different per source. We keep two matrix
pairs and select between them via the ``convention`` argument of
``apply_mano_transform``:

* ``"mediapipe"`` (default) — phone, RealSense, anything driven by MediaPipe
  HandLandmarker. Matches dex-retargeting upstream verbatim. Calibrated
  against MediaPipe's ``world_landmarks`` (image-derived 3D, camera-frame).
  Verified by playing back ``human_hand_video.mp4`` through phone-style
  detect_dg5f and confirming DG-5F follows the source video gesture.

* ``"manus"`` — Manus glove (live ROS2 publisher, SDK subprocess, offline
  egocentric replay). Manus SDK publishes raw skeleton in VUH world space
  which has the opposite +y chirality of the MediaPipe-derived input, so
  the matrix differs by a single sign flip on row 1. Verified end-to-end
  via the manus-egocentric-sample episode 0 regression test
  (`scripts/regression_test_fixes.py`) and the user-confirmed playback of
  ``human_hand_video.mp4`` against DG-5F when the matrix was originally
  patched.

The two RIGHT matrices are related by ``MANUS = diag(1, -1, 1) @ MEDIAPIPE``
— a sign flip on row 1 of the matrix. Because row 1 controls how the
operator y-axis (palm normal) maps to MANO, the resulting output MANO
frames differ by a sign on **MANO +x**: the same physical hand pose
produces output with ``mediapipe.x = -manus.x`` (and identical y/z).
Likewise for LEFT.

History
-------
A single shared matrix used to live here, copied from upstream
``single_hand_detector.py``. The phone path worked. After adding
``apply_mano_transform`` to the Manus path it produced fist→spread inversion;
flipping row 1 fixed Manus but broke phone. Two source-specific matrices
solve both at once. See
``models/dex_retarget/docs/manus_debug.md`` §4.4 for the full investigation.
"""

import numpy as np

# ─────────────────────────────────────────────────────────────────
# MediaPipe convention — phone, RealSense (MediaPipe HandLandmarker)
# Matches dex-retargeting upstream
# ``example/vector_retargeting/single_hand_detector.py`` verbatim. The phone
# path was originally calibrated against these matrices, so any change here
# will silently break phone → DG-5F.
# ─────────────────────────────────────────────────────────────────
MEDIAPIPE_OPERATOR2MANO_RIGHT = np.array(
    [
        [0, 0, -1],
        [-1, 0, 0],
        [0, 1, 0],
    ],
    dtype=np.float32,
)

MEDIAPIPE_OPERATOR2MANO_LEFT = np.array(
    [
        [0, 0, -1],
        [1, 0, 0],
        [0, -1, 0],
    ],
    dtype=np.float32,
)

# ─────────────────────────────────────────────────────────────────
# Manus convention — manus_sensing, offline_egocentric_provider
# Differs from the MediaPipe matrices by a sign flip on row 1, equivalent to
# flipping the MANO +y axis. Required because Manus SDK publishes raw
# skeleton in VUH world space which has opposite +y chirality vs MediaPipe's
# image-derived coordinates.
# Verified by playing back human_hand_video.mp4 against DG-5F (user) and by
# the manus-egocentric-sample episode 0 regression test (agg flex delta
# +0.193 → +2.251 once these matrices are used).
# ─────────────────────────────────────────────────────────────────
MANUS_OPERATOR2MANO_RIGHT = np.array(
    [
        [0, 0, -1],
        [1, 0, 0],
        [0, 1, 0],
    ],
    dtype=np.float32,
)

# WARNING: not yet hardware-verified on a left-hand DG-5F (no left URDF in
# the current setup). Derived from MANUS_RIGHT by the same bilateral
# symmetry that relates MEDIAPIPE_LEFT to MEDIAPIPE_RIGHT.
MANUS_OPERATOR2MANO_LEFT = np.array(
    [
        [0, 0, -1],
        [-1, 0, 0],
        [0, -1, 0],
    ],
    dtype=np.float32,
)

# ─────────────────────────────────────────────────────────────────
# Backwards-compat aliases. Kept so any external import that still does
# ``from retarget_dev.sensing.core.mano_transform import OPERATOR2MANO_RIGHT``
# resolves to the MediaPipe convention (which was the historical default
# before the per-source split).
# ─────────────────────────────────────────────────────────────────
OPERATOR2MANO_RIGHT = MEDIAPIPE_OPERATOR2MANO_RIGHT
OPERATOR2MANO_LEFT = MEDIAPIPE_OPERATOR2MANO_LEFT


def estimate_wrist_frame(keypoints: np.ndarray) -> np.ndarray:
    """Estimate the wrist coordinate frame from 21-point hand keypoints.

    Uses wrist (0), index MCP (5), middle MCP (9) to define the palm plane
    via SVD, then orthonormalizes the basis.

    This mirrors ``estimate_frame_from_hand_points`` in dex-retargeting's
    single_hand_detector.py and is shared by every sensing source — the
    per-source difference is in the constant matrix applied AFTER this, not
    in the SVD step.

    Args:
        keypoints: (21, 3) hand keypoints (already wrist-centered).

    Returns:
        (3, 3) rotation matrix: columns are x/y/z basis vectors of the wrist frame.
    """
    assert keypoints.shape == (21, 3)
    points = keypoints[[0, 5, 9], :]

    # x direction: palm → middle finger base
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

    frame = np.stack([x, normal, z], axis=1)
    return frame.astype(np.float32)


def apply_mano_transform(
    keypoints: np.ndarray,
    hand_type: str = "right",
    convention: str = "mediapipe",
) -> np.ndarray:
    """Apply full MANO transform to wrist-centered keypoints.

    Args:
        keypoints: (21, 3) wrist-centered keypoints.
        hand_type: "right" or "left".
        convention: which operator → MANO matrix to use:

            * ``"mediapipe"`` (default) — phone, RealSense, anything driven by
              MediaPipe HandLandmarker. Matches dex-retargeting upstream.
            * ``"manus"`` — Manus glove (live ROS2 / SDK / offline replay).
              Differs from mediapipe by a +y sign flip; required because the
              SDK publishes in VUH world space with opposite chirality.

            See module docstring for the full rationale and the verified
            sources for each convention.

    Returns:
        (21, 3) keypoints in MANO frame.
    """
    wrist_rot = estimate_wrist_frame(keypoints)
    is_left = hand_type.lower() == "left"
    if convention == "manus":
        op2mano = MANUS_OPERATOR2MANO_LEFT if is_left else MANUS_OPERATOR2MANO_RIGHT
    elif convention == "mediapipe":
        op2mano = (
            MEDIAPIPE_OPERATOR2MANO_LEFT if is_left else MEDIAPIPE_OPERATOR2MANO_RIGHT
        )
    else:
        raise ValueError(
            f"unknown convention: {convention!r} (use 'mediapipe' or 'manus')"
        )
    return keypoints @ wrist_rot @ op2mano
