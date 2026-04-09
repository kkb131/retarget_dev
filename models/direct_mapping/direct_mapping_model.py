"""1st-generation Direct Mapping retargeting model.

The simplest possible retargeting: compute joint angles from human
keypoint geometry, subtract open-hand baseline, apply scale, clamp.

No optimization, no FK iteration — just direct angle extraction and
linear transformation. Useful as a baseline and for quick testing.

Pipeline:
    HandKeypoints (21, 3) → angle_extractor → scale * (angle - baseline) → clip → DG5F q (20,)
"""

import logging
from pathlib import Path
from typing import Optional

import numpy as np

try:
    import yaml
except ImportError:
    yaml = None

from retarget_dev.models.base import RetargetingModel
from retarget_dev.models.direct_mapping.angle_extractor import (
    extract_all_angles,
    extract_all_angles_debug,
    FINGER_NAMES,
)
from retarget_dev.models.direct_mapping.dg5f_fk import DG5FKinematics
from retarget_dev.sensing.common import HandKeypoints

logger = logging.getLogger(__name__)


class DirectMappingModel(RetargetingModel):
    """Direct angle mapping with baseline calibration.

    Formula:
        delta = human_angles - baseline     (subtract open-hand resting angles)
        delta = max(delta, 0)               (flexion only: no negative values)
        q     = scale * delta + offset      (map to DG5F range)
        q     = clip(q, q_min, q_max)       (enforce joint limits)

    Parameters
    ----------
    hand_side : str
        "left" or "right".
    scale_factors : ndarray or None
        (20,) per-joint scale factors.
    offsets : ndarray or None
        (20,) per-joint offsets in radians.
    baseline : ndarray or None
        (20,) open-hand resting angles in radians. Subtracted before scaling.
    auto_calibrate : bool
        If True, capture first frame as baseline automatically.
    config_path : str or None
        Optional YAML config with baseline, scale_factors, offsets.
    """

    def __init__(
        self,
        hand_side: str = "right",
        scale_factors: np.ndarray = None,
        offsets: np.ndarray = None,
        baseline: np.ndarray = None,
        auto_calibrate: bool = False,
        config_path: str = None,
    ):
        self._fk = DG5FKinematics(hand_side=hand_side)
        self._q_min = self._fk.q_min
        self._q_max = self._fk.q_max

        # Default scale: 1:1 mapping (delta → DG5F directly)
        # Joint limits clip handles overflow. Use --calibrate (fist step)
        # to compute accurate per-joint scale = q_max / human_ROM.
        self._scale = np.ones(20, dtype=np.float64)
        self._offsets = np.zeros(20, dtype=np.float64)
        self._baseline = np.zeros(20, dtype=np.float64)

        # Auto-calibration state
        self._auto_calibrate = auto_calibrate
        self._calibrated = not auto_calibrate  # True if no calibration needed
        self._calibration_frames: list[np.ndarray] = []
        self._calibration_count = 30  # frames to average

        # Override from arguments
        if scale_factors is not None:
            self._scale = np.asarray(scale_factors, dtype=np.float64)
        if offsets is not None:
            self._offsets = np.asarray(offsets, dtype=np.float64)
        if baseline is not None:
            self._baseline = np.asarray(baseline, dtype=np.float64)
            self._calibrated = True

        # Override from config file (takes precedence)
        if config_path is not None:
            self._load_config(config_path)

        # Cache for debug
        self._last_human_angles: Optional[np.ndarray] = None
        self._last_delta: Optional[np.ndarray] = None

    def calibrate(self, keypoints: HandKeypoints) -> None:
        """Set baseline from a single open-hand frame."""
        self._baseline = extract_all_angles(keypoints.keypoints_3d)
        self._calibrated = True
        logger.info(
            "Calibrated baseline (single frame). Per-finger MCP baselines: "
            "[%.1f° %.1f° %.1f° %.1f° %.1f°]",
            *[np.degrees(self._baseline[f * 4 + 1]) for f in range(5)],
        )

    def calibrate_from_frames(self, frames: list[np.ndarray]) -> None:
        """Set baseline from averaged multiple frames."""
        all_angles = np.array([extract_all_angles(f) for f in frames])
        self._baseline = np.mean(all_angles, axis=0)
        self._calibrated = True
        logger.info(
            "Calibrated baseline (%d frames averaged). Per-finger MCP baselines: "
            "[%.1f° %.1f° %.1f° %.1f° %.1f°]",
            len(frames),
            *[np.degrees(self._baseline[f * 4 + 1]) for f in range(5)],
        )

    def calibrate_fist(self, keypoints: HandKeypoints) -> None:
        """Compute scale_factors from a single fist frame.

        Requires baseline to be set first.
        scale[i] = q_max[i] / (fist_angle[i] - baseline[i])
        """
        fist_angles = extract_all_angles(keypoints.keypoints_3d)
        self._compute_scale_from_fist(fist_angles)

    def calibrate_fist_from_frames(self, frames: list[np.ndarray]) -> None:
        """Compute scale_factors from averaged fist frames."""
        all_angles = np.array([extract_all_angles(f) for f in frames])
        fist_angles = np.mean(all_angles, axis=0)
        self._compute_scale_from_fist(fist_angles)

    def _compute_scale_from_fist(self, fist_angles: np.ndarray) -> None:
        """Internal: compute scale from fist measurement.

        For each flexion joint: scale = q_max / human_ROM
        where human_ROM = baseline - fist_angle (fist has smaller values = more flexed).
        Spread joints keep their default scale.
        """
        human_rom = self._baseline - fist_angles
        min_rom = np.radians(5.0)  # ignore ROM < 5° (noise)

        for f in range(5):
            base = f * 4
            # Spread (idx 0): keep default scale
            # MCP, PIP, DIP (idx 1, 2, 3): compute from ROM
            for j in [1, 2, 3]:
                idx = base + j
                if human_rom[idx] > min_rom:
                    self._scale[idx] = self._q_max[idx] / human_rom[idx]
                # else: keep default scale

        logger.info(
            "Fist calibrated! Per-finger MCP scales: "
            "[%.2f %.2f %.2f %.2f %.2f]",
            *[self._scale[f * 4 + 1] for f in range(5)],
        )
        logger.info(
            "  Human ROM (fist-baseline): MCP=[%.1f° %.1f° %.1f° %.1f° %.1f°]",
            *[np.degrees(human_rom[f * 4 + 1]) for f in range(5)],
        )

    def retarget(self, keypoints: HandKeypoints) -> np.ndarray:
        """Convert HandKeypoints → DG5F joint angles (20,)."""
        human_angles = extract_all_angles(keypoints.keypoints_3d)
        self._last_human_angles = human_angles.copy()

        # Auto-calibrate: collect first N frames then average
        if not self._calibrated:
            self._calibration_frames.append(keypoints.keypoints_3d.copy())
            if len(self._calibration_frames) >= self._calibration_count:
                self.calibrate_from_frames(self._calibration_frames)
                self._calibration_frames.clear()
            else:
                remaining = self._calibration_count - len(self._calibration_frames)
                if len(self._calibration_frames) == 1:
                    logger.info(
                        "Auto-calibrating: hold hand open... (%d frames remaining)",
                        remaining,
                    )
                return np.zeros(20)  # Output zeros during calibration

        # Compute delta from baseline
        # compute_flexion: π=extended, 0=folded → flexing DECREASES the value
        # So for flexion joints: delta = baseline - human (positive = flexion)
        # For spread: delta = human - baseline (signed angle from reference)
        delta = human_angles.copy()
        for f in range(5):
            base = f * 4
            # Spread (idx 0): human - baseline (signed)
            delta[base + 0] = human_angles[base + 0] - self._baseline[base + 0]
            # Flexion (MCP, PIP, DIP): baseline - human → positive = flexion
            for j in [1, 2, 3]:
                idx = base + j
                delta[idx] = max(self._baseline[idx] - human_angles[idx], 0.0)

        # Thumb MCP (idx 1): DG5F uses negative direction for flexion [-180°, 0°]
        delta[1] = -delta[1]

        self._last_delta = delta.copy()

        q = self._scale * delta + self._offsets
        return np.clip(q, self._q_min, self._q_max)

    def get_joint_names(self) -> list[str]:
        return self._fk.joint_names

    def get_debug_info(self, keypoints: HandKeypoints, q: np.ndarray) -> Optional[dict]:
        """Return human angles, delta (after baseline), DG5F angles, and per-finger breakdown."""
        debug = extract_all_angles_debug(keypoints.keypoints_3d)
        delta_deg = np.degrees(self._last_delta) if self._last_delta is not None else np.zeros(20)
        return {
            "human_angles_deg": np.degrees(debug["angles"]),
            "delta_deg": delta_deg,
            "dg5f_angles_deg": np.degrees(q),
            "per_finger": debug["per_finger"],
            "baseline_deg": np.degrees(self._baseline),
            "scale_factors": self._scale.copy(),
            "offsets_deg": np.degrees(self._offsets),
            "calibrated": self._calibrated,
        }

    @property
    def baseline(self) -> np.ndarray:
        """Current baseline (20,) in radians."""
        return self._baseline.copy()

    @property
    def is_calibrated(self) -> bool:
        return self._calibrated

    def save_config(self, path: str) -> None:
        """Save current baseline, scale_factors, offsets to YAML."""
        if yaml is None:
            logger.warning("PyYAML not installed, cannot save config")
            return

        cfg = {
            "baseline": [round(v, 4) for v in self._baseline.tolist()],
            "scale_factors": [round(v, 4) for v in self._scale.tolist()],
            "offsets": [round(v, 4) for v in self._offsets.tolist()],
        }
        p = Path(path)
        with open(p, "w") as f:
            yaml.dump(cfg, f, default_flow_style=False, sort_keys=False)
        logger.info("Config saved to %s", p)

    def _load_config(self, path: str) -> None:
        """Load baseline, scale_factors, offsets from YAML config."""
        if yaml is None:
            logger.warning("PyYAML not installed, cannot load config: %s", path)
            return

        p = Path(path)
        if not p.exists():
            logger.warning("Config file not found: %s", path)
            return

        with open(p) as f:
            cfg = yaml.safe_load(f)

        if cfg is None:
            return

        if "baseline" in cfg:
            bl = cfg["baseline"]
            if len(bl) == 20:
                self._baseline = np.array(bl, dtype=np.float64)
                self._calibrated = True
                logger.info("Loaded baseline from %s", path)
            else:
                logger.warning("baseline must have 20 elements, got %d", len(bl))

        if "scale_factors" in cfg:
            sf = cfg["scale_factors"]
            if len(sf) == 20:
                self._scale = np.array(sf, dtype=np.float64)
                logger.info("Loaded scale_factors from %s", path)
            else:
                logger.warning("scale_factors must have 20 elements, got %d", len(sf))

        if "offsets" in cfg:
            of = cfg["offsets"]
            if len(of) == 20:
                self._offsets = np.array(of, dtype=np.float64)
                logger.info("Loaded offsets from %s", path)
            else:
                logger.warning("offsets must have 20 elements, got %d", len(of))
