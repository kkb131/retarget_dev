"""Shared MANO coordinate frame transform for hand keypoints.

This module implements the transform that any sensing source (phone camera,
RealSense D405, Manus glove, …) must apply so its keypoints land in the
**MANO convention** expected by dex-retargeting's optimizers.

Pipeline:
    1. Shift origin to wrist (index 0)
    2. Estimate wrist frame via SVD on palm plane (wrist, index_MCP, middle_MCP)
       — gives an operator basis (x, y, z) where:
         * x_op = wrist - middle_MCP   (points from middle_MCP toward wrist)
         * y_op = palm normal          (after disambiguation, palm-out for
                                        right hand; for left hand the SVD
                                        chirality flips so y_op points toward
                                        the back-of-hand instead)
         * z_op = pinky → index        (forced by the disambiguation step;
                                        always points to the hand's thumb side)
    3. Rotate into MANO convention via ``kp @ wrist_rot @ operator2mano``

MANO frame convention (after applying the matrices below)
---------------------------------------------------------
For BOTH hands the MANO axes have the same hand-relative meaning:

    +x = palm-out direction        (palm normal, away from the palm face)
    +y = thumb-side direction      (from pinky toward index)
    +z = forward                   (from wrist toward the fingertips)

This is the convention DG-5F's right URDF expects (verified empirically by
playing back ``human_hand_video.mp4`` and confirming DG-5F follows the same
gesture as the source video). Note that this matrix has determinant -1 (it
is a reflection rather than a pure rotation) — this is intentional and
compensates for the chirality of the operator basis returned by
``estimate_wrist_frame``.

Reference (for historical context):
    dex-retargeting/example/vector_retargeting/single_hand_detector.py
    Our matrices differ from upstream in the sign of row 1 — upstream uses
    ``[-1, 0, 0]`` for RIGHT row 1 (a pure rotation, det = +1) but with our
    DG-5F URDF that yields a frame whose palm normal axis is flipped, which
    makes the optimizer get stuck. The corrected matrices below match
    DG-5F's empirical behaviour.
"""

import numpy as np

# Operator → MANO rotation, RIGHT hand.
# Row interpretation: row i = where operator basis vector e_i lands in MANO.
#   row 0: e_x_op (toward wrist)         → MANO -z   (so MANO +z = forward)
#   row 1: e_y_op (palm normal, palm-out) → MANO +x   (so MANO +x = palm-out)
#   row 2: e_z_op (pinky→index)           → MANO +y   (so MANO +y = thumb-side)
OPERATOR2MANO_RIGHT = np.array(
    [
        [0, 0, -1],
        [1, 0, 0],
        [0, 1, 0],
    ],
    dtype=np.float32,
)

# Operator → MANO rotation, LEFT hand.
# For a left hand, the SVD palm normal returned by ``estimate_wrist_frame``
# points to the BACK of the hand (opposite of right hand) due to chirality
# of ``z = x × normal`` combined with the disambiguation step. To keep the
# same MANO semantic (+x = palm-out), we negate row 1's contribution.
# Row 2 is also negated to mirror across the body plane so the resulting
# left-hand MANO frame is the bilateral counterpart of the right-hand frame
# — same convention dex-retargeting upstream uses (LEFT = diag(1,-1,-1) @ RIGHT).
#
# WARNING: this matrix has not been verified end-to-end on a left-hand DG-5F
# (no left-hand URDF / robot in the current setup). It is the corresponding
# correction to OPERATOR2MANO_RIGHT. Test before relying on it.
OPERATOR2MANO_LEFT = np.array(
    [
        [0, 0, -1],
        [-1, 0, 0],
        [0, -1, 0],
    ],
    dtype=np.float32,
)


def estimate_wrist_frame(keypoints: np.ndarray) -> np.ndarray:
    """Estimate the wrist coordinate frame from 21-point hand keypoints.

    Uses wrist (0), index MCP (5), middle MCP (9) to define the palm plane
    via SVD, then orthonormalizes the basis.

    This mirrors ``estimate_frame_from_hand_points`` in dex-retargeting's
    single_hand_detector.py.

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
) -> np.ndarray:
    """Apply full MANO transform to wrist-centered keypoints.

    Args:
        keypoints: (21, 3) wrist-centered keypoints.
        hand_type: "right" or "left".

    Returns:
        (21, 3) keypoints in MANO frame.
    """
    wrist_rot = estimate_wrist_frame(keypoints)
    if hand_type.lower() == "left":
        return keypoints @ wrist_rot @ OPERATOR2MANO_LEFT
    return keypoints @ wrist_rot @ OPERATOR2MANO_RIGHT
