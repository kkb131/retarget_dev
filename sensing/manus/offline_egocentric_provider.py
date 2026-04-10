"""Offline replay of the manus-egocentric-sample HuggingFace dataset.

Loads one episode of OpenGraphLabs-Research/manus-egocentric-sample (LeRobot
v2.1 format) and exposes it through the :class:`SensingSource` ABC, so it can
be plugged into the retargeting pipeline like any other sensing source:

    sensing = OfflineEgocentricProvider(
        "/workspaces/tamp_ws/src/manus-egocentric-sample/data/.../episode_000000.parquet",
        hand="right",
    )
    sensing.start()
    while True:
        kp = sensing.get_keypoints()
        if kp is None:
            break  # episode ended (loop=False)
        q = model.retarget(kp)

Why this is a SensingSource (not a HandData provider wrapped by ManusSensing)
---------------------------------------------------------------------------
The other Manus providers (mock / ros2 / sdk) hand back ``HandData`` and let
``ManusSensing`` perform the wrist shift + MANO frame rotation. We could in
principle do the same here, but this dataset has one quirk that makes a
self-contained SensingSource cleaner:

**25-node array order is NOT the SDK ``FingerJointType`` enum order.**
Empirically — verified by checking that inter-joint bone lengths are constant
across all 1158 frames of episode 0 — the per-finger order is
``[Metacarpal, Intermediate, Proximal, Distal, Tip]``, i.e. SDK Intermediate
sits BEFORE Proximal. The chain order needed to land in MANO 21 is therefore
``[start, start+2, start+1, start+3, start+4]``. The live ROS2 publisher path
is not affected (it matches by string label, not array index), so we keep
this remap in the offline provider only.

After the remap the keypoints go through the standard ``apply_mano_transform``
with ``hand_type=self._hand``, the same call phone / realsense use. No mirror
fix is needed — earlier analysis claimed there was one, but that was an
artefact of an incorrect ``OPERATOR2MANO_RIGHT`` matrix in
``sensing/core/mano_transform.py``. After fixing that matrix (verified by
playing back ``human_hand_video.mp4`` against DG-5F), the dataset replays
correctly with ``hand_type='right'`` directly.

See ``src/manus-egocentric-sample/docs/usage.md`` §5 for the data layout and
``src/retarget_dev/models/dex_retarget/docs/manus_debug.md`` §4.4 for the
regression test results.
"""

from __future__ import annotations

import logging
import threading
import time
from pathlib import Path
from typing import Optional

import numpy as np

from retarget_dev.sensing.common import HandKeypoints, SensingSource
from retarget_dev.sensing.core.mano_transform import apply_mano_transform

logger = logging.getLogger(__name__)


# 25 → 21 index map for the manus-egocentric-sample dataset.
#
# Per-non-thumb-finger array layout in this dataset:
#   [Metacarpal_base, Intermediate (=PIP), Proximal (=MCP), Distal (=DIP), Tip]
# We drop the Metacarpal_base and re-order so the result follows MANO:
#   [MCP, PIP, DIP, Tip] = [Proximal, Intermediate, Distal, Tip]
#                       = [start+2,  start+1,      start+3, start+4]
#
# Thumb has only 4 joints in chain order [Metacarpal, Proximal, Distal, Tip],
# which already matches MANO [CMC, MCP, IP, Tip].
_KEEP_25_TO_21 = np.array(
    [0,                          # wrist
     1, 2, 3, 4,                 # thumb (chain order matches MANO)
     7, 6, 8, 9,                 # index
     12, 11, 13, 14,             # middle
     17, 16, 18, 19,             # ring
     22, 21, 23, 24],            # pinky
    dtype=np.int64,
)
assert _KEEP_25_TO_21.size == 21

# Hand index in the dataset's (T, 2, 25, 3) layout.
_HAND_DIM = {"right": 0, "left": 1}


def _opposite_hand(hand: str) -> str:
    return "left" if hand.lower() == "right" else "right"


class OfflineEgocentricProvider(SensingSource):
    """Replays a manus-egocentric-sample episode as a SensingSource.

    Parameters
    ----------
    parquet_path : str or Path
        Path to one ``episode_NNNNNN.parquet`` file from the
        manus-egocentric-sample snapshot. The file must contain the
        ``observation.state.hand_joints`` column with shape ``(T, 150)``.
    hand : str
        Which hand to retarget — ``"right"`` (dataset dim 0) or ``"left"``
        (dim 1). Default ``"right"``.
    loop : bool
        If True (default), wrap around to frame 0 after the last frame.
        If False, ``get_keypoints()`` returns ``None`` once the episode ends.
    fps : float
        Playback rate in Hz. ``get_keypoints()`` blocks (sleeps) so callers
        that just poll naturally see the episode at this rate. Default 30
        (matches the dataset's native rate).
    apply_dataset_mirror_fix : bool
        Legacy escape hatch — if True, invert the ``hand_type`` passed to
        ``apply_mano_transform``. This was needed when ``OPERATOR2MANO_RIGHT``
        had a sign error in row 1; with the corrected matrix the default
        ``False`` is correct and ``hand_type=self._hand`` works directly.
    """

    def __init__(
        self,
        parquet_path: str | Path,
        hand: str = "right",
        loop: bool = True,
        fps: float = 30.0,
        apply_dataset_mirror_fix: bool = False,
    ):
        self._parquet_path = Path(parquet_path)
        self._hand = hand.lower()
        if self._hand not in _HAND_DIM:
            raise ValueError(f"hand must be 'left' or 'right', got: {hand!r}")
        self._loop = loop
        self._fps = float(fps)
        self._dt = 1.0 / self._fps if self._fps > 0 else 0.0
        # apply_dataset_mirror_fix is a legacy escape hatch — see class
        # docstring. Default False (matrix-correct path) is what real users
        # should use.
        self._mano_hand = _opposite_hand(self._hand) if apply_dataset_mirror_fix else self._hand

        # Lazily filled by start()
        self._kp25: Optional[np.ndarray] = None     # (T, 25, 3) world frame, wrist-centered
        self._timestamps: Optional[np.ndarray] = None
        self._n_frames: int = 0
        self._cursor: int = 0

        # Pacing
        self._next_emit_time: float = 0.0
        self._lock = threading.Lock()
        self._started = False

    # ── SensingSource implementation ────────────────────────────────

    def start(self) -> None:
        if self._started:
            return

        if not self._parquet_path.exists():
            raise FileNotFoundError(
                f"Parquet not found: {self._parquet_path}. "
                "Run scripts/download_dataset.py first."
            )

        # Lazy import — pyarrow is not a hard dep of retarget_dev
        try:
            import pandas as pd
        except ImportError as e:
            raise ImportError(
                "pandas + pyarrow are required to load the manus-egocentric "
                "dataset. Install with: pip install pandas pyarrow"
            ) from e

        df = pd.read_parquet(self._parquet_path)
        if "observation.state.hand_joints" not in df.columns:
            raise ValueError(
                f"{self._parquet_path.name} does not look like a "
                "manus-egocentric-sample episode (missing "
                "'observation.state.hand_joints' column)."
            )

        # (T, 150) → (T, 2, 25, 3)
        hj = np.stack(
            [np.asarray(r, dtype=np.float32) for r in df["observation.state.hand_joints"]],
            axis=0,
        ).reshape(-1, 2, 25, 3)

        # Pull the requested hand and apply the verified 25→21 swap.
        kp25_full = hj[:, _HAND_DIM[self._hand]]              # (T, 25, 3)
        self._kp25 = kp25_full[:, _KEEP_25_TO_21, :]           # (T, 21, 3) — name is legacy

        self._timestamps = df["timestamp"].to_numpy(np.float32)
        self._n_frames = self._kp25.shape[0]
        self._cursor = 0
        self._next_emit_time = time.perf_counter()
        self._started = True

        logger.info(
            "OfflineEgocentricProvider started: %s (frames=%d, hand=%s, "
            "mano_hand=%s, fps=%.1f, loop=%s)",
            self._parquet_path.name, self._n_frames, self._hand,
            self._mano_hand, self._fps, self._loop,
        )

    def stop(self) -> None:
        self._started = False
        logger.info("OfflineEgocentricProvider stopped.")

    def get_keypoints(self) -> Optional[HandKeypoints]:
        if not self._started:
            return None

        with self._lock:
            if self._cursor >= self._n_frames:
                if not self._loop:
                    return None
                self._cursor = 0

            kp21_world = self._kp25[self._cursor]
            ts = float(self._timestamps[self._cursor])
            self._cursor += 1

        # Pace playback. Sleep until the next emit slot so callers polling at
        # > fps still see the data spread out at the dataset's native rate.
        if self._dt > 0:
            now = time.perf_counter()
            sleep_for = self._next_emit_time - now
            if sleep_for > 0:
                time.sleep(sleep_for)
            self._next_emit_time = max(self._next_emit_time + self._dt, now)

        # Apply MANO transform with the (possibly inverted) hand to compensate
        # for the dataset's mirrored coords. ``kp21_world`` is already
        # wrist-centered (verified — every frame has hand_joints[hand, 0] = 0).
        kp_mano = apply_mano_transform(
            kp21_world.astype(np.float32), hand_type=self._mano_hand,
        )

        return HandKeypoints(
            keypoints_3d=kp_mano,
            keypoints_2d=None,
            handedness=self._hand,
            confidence=1.0,
            timestamp=ts,
            source=f"manus-egocentric:{self._parquet_path.stem}",
        )

    def get_frame(self) -> Optional[np.ndarray]:
        # Video frame extraction is intentionally not implemented here — the
        # mp4 lives next to the parquet but decoding it on every call would
        # add an OpenCV dependency. Use scripts/inspect_episode.py or load
        # the mp4 separately if you need synchronized RGB.
        return None

    # ── Convenience accessors ──────────────────────────────────────

    @property
    def num_frames(self) -> int:
        return self._n_frames

    @property
    def cursor(self) -> int:
        return self._cursor

    def reset(self) -> None:
        """Rewind to frame 0 without restarting."""
        with self._lock:
            self._cursor = 0
            self._next_emit_time = time.perf_counter()
