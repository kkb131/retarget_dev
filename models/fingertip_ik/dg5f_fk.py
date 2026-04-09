"""DG5F forward kinematics via Pinocchio (independent of teleop_dev).

Loads DG5F URDF and provides fingertip positions, direction vectors,
and Jacobians for optimization-based retargeting.

Reference: teleop_dev/sender/hand/dg5f_fk.py (same logic, independent impl).
"""

import os
from pathlib import Path

import numpy as np

try:
    import pinocchio as pin
except ImportError:
    raise ImportError("pinocchio required: pip install pin")


# Default URDF paths (dg5f_ros2 package in workspace)
_URDF_DIR = (
    Path(__file__).resolve().parent.parent.parent.parent
    / "dg5f_ros2" / "dg5f_description" / "urdf"
)

RIGHT_URDF = str(_URDF_DIR / "dg5f_right.urdf")
LEFT_URDF = str(_URDF_DIR / "dg5f_left.urdf")

# Frame and joint naming conventions
RIGHT_TIP_FRAMES = [f"rl_dg_{i}_tip" for i in range(1, 6)]
LEFT_TIP_FRAMES = [f"ll_dg_{i}_tip" for i in range(1, 6)]
RIGHT_PALM = "rl_dg_palm"
LEFT_PALM = "ll_dg_palm"
RIGHT_JOINTS = [f"rj_dg_{f}_{j}" for f in range(1, 6) for j in range(1, 5)]
LEFT_JOINTS = [f"lj_dg_{f}_{j}" for f in range(1, 6) for j in range(1, 5)]


class DG5FKinematics:
    """Forward kinematics for DG5F hand using Pinocchio.

    Parameters
    ----------
    hand_side : str
        "left" or "right".
    urdf_path : str or None
        Override URDF path. None uses default.
    """

    def __init__(self, hand_side: str = "right", urdf_path: str = None):
        self._side = hand_side
        urdf = urdf_path or (RIGHT_URDF if hand_side == "right" else LEFT_URDF)

        if not os.path.exists(urdf):
            raise FileNotFoundError(f"URDF not found: {urdf}")

        self._model = pin.buildModelFromUrdf(urdf)
        self._data = self._model.createData()

        # Resolve frame IDs
        tip_names = RIGHT_TIP_FRAMES if hand_side == "right" else LEFT_TIP_FRAMES
        palm_name = RIGHT_PALM if hand_side == "right" else LEFT_PALM

        self._tip_frame_ids = []
        for name in tip_names:
            fid = self._model.getFrameId(name)
            if fid >= self._model.nframes:
                raise ValueError(f"Frame '{name}' not found in URDF")
            self._tip_frame_ids.append(fid)

        self._palm_frame_id = self._model.getFrameId(palm_name)

        # Build joint index mapping: canonical order → pinocchio q index
        joint_names = RIGHT_JOINTS if hand_side == "right" else LEFT_JOINTS
        self._joint_names = joint_names
        self._q_indices = []
        for jname in joint_names:
            jid = self._model.getJointId(jname)
            if jid >= self._model.njoints:
                raise ValueError(f"Joint '{jname}' not found in URDF")
            idx_q = self._model.joints[jid].idx_q
            self._q_indices.append(idx_q)

        self._nq = self._model.nq

    @property
    def joint_names(self) -> list[str]:
        return list(self._joint_names)

    @property
    def q_min(self) -> np.ndarray:
        """Joint lower limits in canonical order (20,)."""
        return np.array([self._model.lowerPositionLimit[i] for i in self._q_indices])

    @property
    def q_max(self) -> np.ndarray:
        """Joint upper limits in canonical order (20,)."""
        return np.array([self._model.upperPositionLimit[i] for i in self._q_indices])

    def _to_pin_q(self, q_canonical: np.ndarray) -> np.ndarray:
        """Convert 20-element canonical q to pinocchio q vector."""
        q_pin = pin.neutral(self._model)
        for i, idx in enumerate(self._q_indices):
            q_pin[idx] = q_canonical[i]
        return q_pin

    def fingertip_positions(self, q: np.ndarray) -> np.ndarray:
        """Compute 5 fingertip positions in world frame.

        Returns: ndarray[5, 3]
        """
        q_pin = self._to_pin_q(q)
        pin.forwardKinematics(self._model, self._data, q_pin)
        pin.updateFramePlacements(self._model, self._data)

        tips = np.zeros((5, 3))
        for i, fid in enumerate(self._tip_frame_ids):
            tips[i] = self._data.oMf[fid].translation
        return tips

    def palm_position(self, q: np.ndarray) -> np.ndarray:
        """Compute palm position in world frame. Returns: ndarray[3]"""
        q_pin = self._to_pin_q(q)
        pin.forwardKinematics(self._model, self._data, q_pin)
        pin.updateFramePlacements(self._model, self._data)
        return self._data.oMf[self._palm_frame_id].translation.copy()

    def fingertip_vectors(self, q: np.ndarray) -> np.ndarray:
        """Compute palm→fingertip normalized direction vectors.

        Returns: ndarray[5, 3]
        """
        return self._compute_vectors(q, origin="palm")

    def fingertip_vectors_from_wrist(self, q: np.ndarray) -> np.ndarray:
        """Compute wrist(origin)→fingertip normalized direction vectors.

        Uses URDF world origin (≈wrist mount) instead of palm.
        This matches the human side where vectors are wrist→fingertip.

        Returns: ndarray[5, 3]
        """
        return self._compute_vectors(q, origin="wrist")

    def _compute_vectors(self, q: np.ndarray, origin: str = "palm") -> np.ndarray:
        """Compute origin→fingertip normalized direction vectors."""
        q_pin = self._to_pin_q(q)
        pin.forwardKinematics(self._model, self._data, q_pin)
        pin.updateFramePlacements(self._model, self._data)

        if origin == "palm":
            origin_pos = self._data.oMf[self._palm_frame_id].translation
        else:
            origin_pos = np.zeros(3)  # URDF world origin ≈ wrist mount

        vectors = np.zeros((5, 3))
        for i, fid in enumerate(self._tip_frame_ids):
            v = self._data.oMf[fid].translation - origin_pos
            norm = np.linalg.norm(v)
            if norm > 1e-6:
                vectors[i] = v / norm
            else:
                vectors[i] = v
        return vectors
