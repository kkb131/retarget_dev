"""2nd-generation Fingertip IK retargeting model.

Uses a hybrid Position + Direction cost function:
- Position term: matches fingertip xyz positions (scaled) → captures curl/fist
- Direction term: matches fingertip direction vectors → preserves finger orientation

Algorithm:
    1. Extract 5 fingertip positions from HandKeypoints (indices [4,8,12,16,20])
    2. Apply SVD calibration (rotation + scale) to align human→robot frame
    3. Optimize DG5F q via scipy SLSQP
    4. Warm-start from previous solution for temporal smoothness

Cost: Σ w_i * [α * ||robot_pos_i - scaled_human_pos_i||²
             + β * ||robot_dir_i - human_dir_i||²]
     + λ * ||q - q_prev||²
"""

import logging
import time

import numpy as np
from scipy.optimize import minimize

from retarget_dev.models.base import RetargetingModel
from retarget_dev.models.fingertip_ik.dg5f_fk import DG5FKinematics
from retarget_dev.sensing.common import FINGER_TIP_INDICES, HandKeypoints

logger = logging.getLogger(__name__)


def compute_alignment_rotation(source_vecs: np.ndarray, target_vecs: np.ndarray) -> np.ndarray:
    """Compute optimal rotation R via SVD Procrustes: R @ source ≈ target."""
    H = source_vecs.T @ target_vecs
    U, _, Vt = np.linalg.svd(H)
    d = np.linalg.det(Vt.T @ U.T)
    S = np.diag([1, 1, np.sign(d)])
    return Vt.T @ S @ U.T


class FingertipIKModel(RetargetingModel):
    """Fingertip IK with hybrid position+direction matching.

    Parameters
    ----------
    hand_side : str
        "left" or "right".
    urdf_path : str or None
        Override DG5F URDF path.
    pos_weight : float
        Weight for position matching term (α).
    dir_weight : float
        Weight for direction matching term (β).
    reg_weight : float
        Temporal smoothness regularization (λ).
    max_iter : int
        Maximum SLSQP iterations per frame.
    auto_calibrate : bool
        If True, auto-calibrate on first keypoints (assumes open hand).
    """

    def __init__(
        self,
        hand_side: str = "right",
        urdf_path: str = None,
        pos_weight: float = 10.0,
        dir_weight: float = 1.0,
        reg_weight: float = 0.01,
        max_iter: int = 50,
        auto_calibrate: bool = True,
    ):
        self._fk = DG5FKinematics(hand_side=hand_side, urdf_path=urdf_path)
        self._pos_weight = pos_weight
        self._dir_weight = dir_weight
        self._reg_weight = reg_weight
        self._max_iter = max_iter
        self._auto_calibrate = auto_calibrate

        # Joint limits and warm-start
        self._q_min = self._fk.q_min
        self._q_max = self._fk.q_max
        self._bounds = list(zip(self._q_min, self._q_max))
        self._q_prev = np.zeros(20)

        # Per-finger weights
        self._finger_weights = np.array([1.5, 1.0, 1.0, 1.0, 0.8])

        # Robot reference at q=0
        self._robot_ref_positions = self._fk.fingertip_positions(np.zeros(20))
        self._robot_ref_vecs = self._fk.fingertip_vectors_from_wrist(np.zeros(20))

        # Calibration state
        self._R = np.eye(3, dtype=np.float64)
        self._scale = 1.0
        self._calibrated = False

        self._last_solve_ms = 0.0

    def calibrate(self, keypoints: HandKeypoints) -> None:
        """Calibrate frame alignment + scale from an open-hand pose.

        Computes:
        - R: SVD rotation matrix (human frame → robot frame)
        - scale: ratio of robot/human fingertip distances
        """
        human_vecs = self._extract_human_vectors(keypoints)
        human_positions = self._extract_human_positions(keypoints)

        # Rotation alignment
        self._R = compute_alignment_rotation(human_vecs, self._robot_ref_vecs)

        # Scale: ratio of mean fingertip distances (robot / human)
        human_dists = np.linalg.norm(human_positions, axis=1)  # wrist=origin
        robot_dists = np.linalg.norm(self._robot_ref_positions, axis=1)
        valid = human_dists > 1e-6
        if valid.any():
            self._scale = float(np.mean(robot_dists[valid]) / np.mean(human_dists[valid]))
        else:
            self._scale = 1.0

        self._calibrated = True

        # Log calibration quality
        aligned_vecs = (self._R @ human_vecs.T).T
        errors = []
        for i in range(5):
            cos_sim = np.dot(aligned_vecs[i], self._robot_ref_vecs[i])
            errors.append(np.degrees(np.arccos(np.clip(cos_sim, -1, 1))))
        logger.info(
            "Calibrated! scale=%.2f  dir_errors: [%.1f° %.1f° %.1f° %.1f° %.1f°] mean=%.1f°",
            self._scale, *errors, np.mean(errors),
        )

    def retarget(self, keypoints: HandKeypoints) -> np.ndarray:
        """Convert HandKeypoints → DG5F joint angles (20,)."""
        if self._auto_calibrate and not self._calibrated:
            self.calibrate(keypoints)

        # Extract and align human data
        human_vecs = self._extract_human_vectors(keypoints)
        human_pos = self._extract_human_positions(keypoints)
        aligned_vecs = (self._R @ human_vecs.T).T
        aligned_pos = (self._R @ human_pos.T).T * self._scale

        t0 = time.perf_counter()

        alpha = self._pos_weight
        beta = self._dir_weight

        def cost(q):
            robot_pos = self._fk.fingertip_positions(q)
            robot_vecs = self._fk.fingertip_vectors_from_wrist(q)
            err = 0.0
            for i in range(5):
                w = self._finger_weights[i]
                # Position term (captures distance/curl)
                pos_diff = robot_pos[i] - aligned_pos[i]
                err += w * alpha * np.dot(pos_diff, pos_diff)
                # Direction term (captures orientation)
                dir_diff = robot_vecs[i] - aligned_vecs[i]
                err += w * beta * np.dot(dir_diff, dir_diff)
            # Temporal regularization
            delta = q - self._q_prev
            err += self._reg_weight * np.dot(delta, delta)
            return err

        result = minimize(
            cost,
            self._q_prev,
            method='SLSQP',
            bounds=self._bounds,
            options={'maxiter': self._max_iter, 'ftol': 1e-7},
        )

        q_opt = np.clip(result.x, self._q_min, self._q_max)
        self._q_prev = q_opt.copy()
        self._last_solve_ms = (time.perf_counter() - t0) * 1000

        return q_opt

    def get_joint_names(self) -> list[str]:
        return self._fk.joint_names

    def get_debug_info(self, keypoints: HandKeypoints, q: np.ndarray) -> dict:
        """Return per-finger errors and solve time."""
        human_vecs = self._extract_human_vectors(keypoints)
        human_pos = self._extract_human_positions(keypoints)
        aligned_vecs = (self._R @ human_vecs.T).T
        aligned_pos = (self._R @ human_pos.T).T * self._scale

        robot_pos = self._fk.fingertip_positions(q)
        robot_vecs = self._fk.fingertip_vectors_from_wrist(q)

        angle_errors = []
        pos_errors_mm = []
        for i in range(5):
            cos_sim = np.dot(aligned_vecs[i], robot_vecs[i])
            angle_errors.append(float(np.degrees(np.arccos(np.clip(cos_sim, -1, 1)))))
            pos_errors_mm.append(float(np.linalg.norm(robot_pos[i] - aligned_pos[i]) * 1000))

        return {
            "angle_errors_deg": angle_errors,
            "mean_error_deg": float(np.mean(angle_errors)),
            "pos_errors_mm": pos_errors_mm,
            "mean_pos_error_mm": float(np.mean(pos_errors_mm)),
            "solve_time_ms": self._last_solve_ms,
            "calibrated": self._calibrated,
            "scale": self._scale,
        }

    @property
    def solve_time_ms(self) -> float:
        return self._last_solve_ms

    @property
    def is_calibrated(self) -> bool:
        return self._calibrated

    @staticmethod
    def _extract_human_vectors(keypoints: HandKeypoints) -> np.ndarray:
        """Extract 5 normalized wrist→fingertip direction vectors. Returns (5, 3)."""
        kp = keypoints.keypoints_3d
        wrist = kp[0]
        vectors = np.zeros((5, 3))
        for i, tip_idx in enumerate(FINGER_TIP_INDICES):
            v = kp[tip_idx] - wrist
            norm = np.linalg.norm(v)
            if norm > 1e-6:
                vectors[i] = v / norm
        return vectors

    @staticmethod
    def _extract_human_positions(keypoints: HandKeypoints) -> np.ndarray:
        """Extract 5 fingertip positions (raw, not normalized). Returns (5, 3)."""
        kp = keypoints.keypoints_3d
        return np.array([kp[idx] for idx in FINGER_TIP_INDICES], dtype=np.float64)
