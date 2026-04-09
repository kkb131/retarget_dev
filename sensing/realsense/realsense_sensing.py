"""RealSense D405 hand sensing pipeline.

Captures color + depth from D405, runs MediaPipe hand detection on color,
then uses real depth to compute accurate 3D keypoints via deprojection.

Detection runs in a background thread (same pattern as PhoneSensing).

Usage:
    with RealSenseSensing(hand_side="right") as rs:
        while True:
            kp = rs.get_keypoints()
            if kp is not None:
                print(kp.keypoints_3d.shape)  # (21, 3)
"""

import logging
import threading
import time
from typing import Optional

import numpy as np

from retarget_dev.sensing.common import HandKeypoints, SensingSource
from retarget_dev.sensing.core.hand_detector import HandDetection, HandDetector
from retarget_dev.sensing.realsense.config import RS_FPS, RS_HEIGHT, RS_WIDTH
from retarget_dev.sensing.realsense.depth_keypoint_converter import DepthKeypointConverter
from retarget_dev.sensing.realsense.rs_camera import RSCamera

logger = logging.getLogger(__name__)


class RealSenseSensing(SensingSource):
    """Complete RealSense D405 → hand keypoints pipeline.

    Detection runs in a daemon thread. ``get_keypoints()`` and
    ``get_frame()`` are non-blocking and always return the latest result.
    """

    def __init__(
        self,
        serial: str | None = None,
        width: int = RS_WIDTH,
        height: int = RS_HEIGHT,
        fps: int = RS_FPS,
        num_hands: int = 1,
        hand_side: str = "right",
        model_path: str | None = None,
    ):
        self._hand_side = hand_side.lower()
        self._camera = RSCamera(serial=serial, width=width, height=height, fps=fps)
        self._detector = HandDetector(model_path=model_path, num_hands=num_hands)
        self._converter: Optional[DepthKeypointConverter] = None  # created after start

        # Timing
        self._start_time_ms = 0
        self._detection_time_ms = 0.0

        # Latest results (protected by lock)
        self._lock = threading.Lock()
        self._latest_keypoints: Optional[HandKeypoints] = None
        self._latest_frame: Optional[np.ndarray] = None

        # Detection thread control
        self._running = False
        self._thread: Optional[threading.Thread] = None

    # ── SensingSource implementation ──────────────────────

    def start(self) -> None:
        self._camera.start()
        self._detector.start()
        # MANO transform applied inside DepthKeypointConverter (required by dex-retargeting)
        self._converter = DepthKeypointConverter(
            self._camera.intrinsics, hand_type=self._hand_side,
        )
        self._start_time_ms = int(time.monotonic() * 1000)
        self._running = True
        self._thread = threading.Thread(target=self._detect_loop, daemon=True)
        self._thread.start()
        logger.info("RealSenseSensing started (hand=%s)", self._hand_side)

    def stop(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        self._detector.close()
        self._camera.stop()
        logger.info("RealSenseSensing stopped.")

    def get_keypoints(self) -> Optional[HandKeypoints]:
        """Return the latest keypoints (non-blocking)."""
        with self._lock:
            return self._latest_keypoints

    def get_frame(self) -> Optional[np.ndarray]:
        """Return the color frame that corresponds to the latest keypoints."""
        with self._lock:
            if self._latest_frame is not None:
                return self._latest_frame.copy()
            return None

    # ── Properties ────────────────────────────────────────

    @property
    def fps(self) -> float:
        return 0.0  # TODO: measure actual FPS

    @property
    def detection_time_ms(self) -> float:
        return self._detection_time_ms

    # ── Detection thread ──────────────────────────────────

    def _detect_loop(self) -> None:
        """Background thread: grab frames → detect → update latest results."""
        while self._running:
            ok, color, depth = self._camera.read()
            if not ok or color is None or depth is None:
                time.sleep(0.001)
                continue

            ts_ms = int(time.monotonic() * 1000) - self._start_time_ms
            h, w = color.shape[:2]

            t0 = time.perf_counter()
            detections = self._detector.detect(color, ts_ms)
            det_ms = (time.perf_counter() - t0) * 1000
            self._detection_time_ms = det_ms

            keypoints: Optional[HandKeypoints] = None
            if detections:
                best = self._select_hand(detections)
                if best is not None:
                    keypoints_3d = self._converter.convert(best, depth, w, h)
                    keypoints_2d = DepthKeypointConverter.extract_2d(best)
                    keypoints = HandKeypoints(
                        keypoints_3d=keypoints_3d,
                        keypoints_2d=keypoints_2d,
                        handedness=best.handedness.lower(),
                        confidence=best.handedness_score,
                        timestamp=time.time(),
                        source="realsense",
                    )

            with self._lock:
                self._latest_keypoints = keypoints
                self._latest_frame = color

    # ── Private ───────────────────────────────────────────

    def _select_hand(
        self, detections: list[HandDetection]
    ) -> Optional[HandDetection]:
        """Select the hand matching self._hand_side with highest confidence."""
        target = self._hand_side.capitalize()
        candidates = [d for d in detections if d.handedness == target]
        if not candidates:
            candidates = detections
        return max(candidates, key=lambda d: d.handedness_score)
