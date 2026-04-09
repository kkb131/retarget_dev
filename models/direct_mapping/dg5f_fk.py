"""DG5F joint metadata loaded from URDF via Pinocchio.

Provides canonical joint names and joint limits for the 20-DOF DG5F hand.
The full forward-kinematics surface (frame placements, fingertip positions,
Jacobians) is intentionally NOT included — direct_mapping only needs
joint names and per-joint q_min/q_max for clamping.
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

# Canonical joint name patterns
RIGHT_JOINTS = [f"rj_dg_{f}_{j}" for f in range(1, 6) for j in range(1, 5)]
LEFT_JOINTS = [f"lj_dg_{f}_{j}" for f in range(1, 6) for j in range(1, 5)]


class DG5FKinematics:
    """Joint metadata source for DG5F hand.

    Parameters
    ----------
    hand_side : str
        "left" or "right".
    urdf_path : str or None
        Override URDF path. None uses default.
    """

    def __init__(self, hand_side: str = "right", urdf_path: str = None):
        urdf = urdf_path or (RIGHT_URDF if hand_side == "right" else LEFT_URDF)

        if not os.path.exists(urdf):
            raise FileNotFoundError(f"URDF not found: {urdf}")

        self._model = pin.buildModelFromUrdf(urdf)

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
