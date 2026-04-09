"""dex-retargeting library wrapper for retarget_dev pipeline.

Wraps SeqRetargeting from the dex-retargeting library to implement
our RetargetingModel ABC. Supports vector/position/DexPilot modes
via YAML config files.

Usage:
    model = DexRetargetModel.from_config("config/dg5f_right_vector.yml")
    q = model.retarget(hand_keypoints)
"""

import logging
import time
from pathlib import Path
from typing import Optional

import numpy as np

from retarget_dev.models.base import RetargetingModel
from retarget_dev.sensing.common import HandKeypoints

logger = logging.getLogger(__name__)


class DexRetargetModel(RetargetingModel):
    """Wrapper around dex-retargeting's SeqRetargeting.

    Converts HandKeypoints (21, 3) → ref_value format expected by
    the specific optimizer type, then calls retarget().
    """

    def __init__(self, config_path: str):
        from dex_retargeting.retargeting_config import RetargetingConfig

        self._config_path = config_path
        config = RetargetingConfig.load_from_file(config_path)
        self._retargeting = config.build()
        self._retarget_type = config.type if isinstance(config.type, str) else config.type.name

        # Parse human indices from config to know how to build ref_value
        cfg_data = self._load_yaml(config_path)
        retarget_cfg = cfg_data.get("retargeting", cfg_data)
        self._type_str = retarget_cfg.get("type", "vector").lower()

        # For DexPilot: need fingertip indices
        if self._type_str == "dexpilot":
            # Standard MANO fingertip indices
            self._tip_indices = [4, 8, 12, 16, 20]
            n_tips = len(self._tip_indices)
            self._n_vectors = n_tips + n_tips * (n_tips - 1) // 2
        else:
            # Vector/Position: indices from config
            indices = retarget_cfg.get("target_link_human_indices", [[0], [4]])
            self._origin_indices = indices[0]
            self._task_indices = indices[1]
            self._n_vectors = len(self._origin_indices)

        self._last_solve_ms = 0.0

        logger.info(
            "DexRetargetModel loaded (type=%s, config=%s, joints=%d)",
            self._type_str, Path(config_path).name, len(self._retargeting.joint_names),
        )

    @classmethod
    def from_config(cls, config_path: str) -> "DexRetargetModel":
        return cls(config_path)

    def retarget(self, keypoints: HandKeypoints) -> np.ndarray:
        """Convert HandKeypoints → robot joint angles."""
        kp = keypoints.keypoints_3d  # (21, 3)
        ref_value = self._build_ref_value(kp)

        t0 = time.perf_counter()
        q = self._retargeting.retarget(ref_value)
        self._last_solve_ms = (time.perf_counter() - t0) * 1000

        return q

    def get_joint_names(self) -> list[str]:
        return list(self._retargeting.joint_names)

    def get_debug_info(self, keypoints: HandKeypoints, q: np.ndarray) -> Optional[dict]:
        return {
            "solve_time_ms": self._last_solve_ms,
            "type": self._type_str,
            "dg5f_angles_deg": np.degrees(q),
        }

    @property
    def solve_time_ms(self) -> float:
        return self._last_solve_ms

    def _build_ref_value(self, kp: np.ndarray) -> np.ndarray:
        """Convert (21, 3) keypoints to optimizer-specific input."""
        if self._type_str == "dexpilot":
            return self._build_dexpilot_input(kp)
        else:
            # Vector or Position: compute origin→task vectors
            ref = np.zeros((self._n_vectors, 3))
            for i in range(self._n_vectors):
                ref[i] = kp[self._task_indices[i]] - kp[self._origin_indices[i]]
            return ref

    def _build_dexpilot_input(self, kp: np.ndarray) -> np.ndarray:
        """Build DexPilot input: wrist→tip vectors + pairwise tip vectors."""
        tips = kp[self._tip_indices]  # (5, 3)
        wrist = kp[0]
        n_tips = len(self._tip_indices)

        ref = np.zeros((self._n_vectors, 3))

        # First N: wrist → fingertip vectors
        for i in range(n_tips):
            ref[i] = tips[i] - wrist

        # Remaining: pairwise tip_i - tip_j
        idx = n_tips
        for i in range(n_tips):
            for j in range(i + 1, n_tips):
                ref[idx] = tips[i] - tips[j]
                idx += 1

        return ref

    @staticmethod
    def _load_yaml(path: str) -> dict:
        import yaml
        with open(path) as f:
            return yaml.safe_load(f)
