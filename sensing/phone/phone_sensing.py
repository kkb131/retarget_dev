"""Top-level phone camera hand sensing pipeline.

Facade that connects CameraSource → HandDetector → KeypointConverter
and implements the SensingSource ABC so downstream code (retargeting,
visualization, recording) is source-agnostic.

Detection runs in a background thread so that the main loop (visualization)
is never blocked by slow MediaPipe inference.

Usage:
    with PhoneSensing(camera_url="http://192.168.0.100:8080/video") as ps:
        while True:
            kp = ps.get_keypoints()
            frame = ps.get_frame()   # same frame used for detection
            if kp is not None:
                print(kp.keypoints_3d.shape)  # (21, 3)
"""

import logging
import threading
import time
from typing import Optional

import numpy as np

from retarget_dev.sensing.common import HandKeypoints, SensingSource
from retarget_dev.sensing.phone.camera_source import CameraSource
from retarget_dev.sensing.phone.config import DEFAULT_CAMERA_URL, DEFAULT_FPS, DEFAULT_RESOLUTION
from retarget_dev.sensing.core.hand_detector import HandDetection, HandDetector
from retarget_dev.sensing.phone.keypoint_converter import KeypointConverter

logger = logging.getLogger(__name__)


class PhoneSensing(SensingSource):
    """Complete phone camera → hand keypoints pipeline.

    Detection runs in a daemon thread.  ``get_keypoints()`` and
    ``get_frame()`` are non-blocking and always return the latest result.
    """

    def __init__(
        self,
        camera_url: str = DEFAULT_CAMERA_URL,
        resolution: tuple[int, int] = DEFAULT_RESOLUTION,
        fps: int = DEFAULT_FPS,
        mirror: bool = False,
        num_hands: int = 1,
        hand_side: str = "right",
        model_path: str | None = None,
    ):
        self._hand_side = hand_side.lower()
        self._camera = CameraSource(
            source=camera_url, resolution=resolution, fps=fps, mirror=mirror,
        )
        self._detector = HandDetector(
            model_path=model_path, num_hands=num_hands,
        )
        # Apply MANO frame transform (required by dex-retargeting)
        self._converter = KeypointConverter(hand_type=self._hand_side)

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
        self._start_time_ms = int(time.monotonic() * 1000)
        self._running = True
        self._thread = threading.Thread(target=self._detect_loop, daemon=True)
        self._thread.start()
        logger.info("PhoneSensing started (hand=%s)", self._hand_side)

    def stop(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        self._detector.close()
        self._camera.stop()
        logger.info("PhoneSensing stopped.")

    def get_keypoints(self) -> Optional[HandKeypoints]:
        """Return the latest keypoints (non-blocking)."""
        with self._lock:
            return self._latest_keypoints

    def get_frame(self) -> Optional[np.ndarray]:
        """Return the frame that corresponds to the latest keypoints (non-blocking)."""
        with self._lock:
            if self._latest_frame is not None:
                return self._latest_frame.copy()
            return None

    # ── Properties ────────────────────────────────────────

    @property
    def fps(self) -> float:
        return self._camera.actual_fps

    @property
    def detection_time_ms(self) -> float:
        return self._detection_time_ms

    # ── Detection thread ──────────────────────────────────

    def _detect_loop(self) -> None:
        """Background thread: grab frame → detect → update latest results."""
        while self._running:
            ok, frame = self._camera.read()
            if not ok or frame is None:
                time.sleep(0.001)
                continue

            ts_ms = int(time.monotonic() * 1000) - self._start_time_ms

            t0 = time.perf_counter()
            detections = self._detector.detect(frame, ts_ms)
            det_ms = (time.perf_counter() - t0) * 1000
            self._detection_time_ms = det_ms

            keypoints: Optional[HandKeypoints] = None
            if detections:
                best = self._select_hand(detections)
                if best is not None:
                    keypoints_3d = self._converter.convert(best)
                    keypoints_2d = KeypointConverter.extract_2d(best)
                    keypoints = HandKeypoints(
                        keypoints_3d=keypoints_3d,
                        keypoints_2d=keypoints_2d,
                        handedness=best.handedness.lower(),
                        confidence=best.handedness_score,
                        timestamp=time.time(),
                        source="phone",
                    )

            with self._lock:
                self._latest_keypoints = keypoints
                self._latest_frame = frame

    # ── Private ───────────────────────────────────────────

    def _select_hand(
        self, detections: list[HandDetection]
    ) -> Optional[HandDetection]:
        """Select the hand matching self._hand_side with highest confidence.

        Note: MediaPipe reports handedness as seen from the camera (mirrored).
        "Left" label = user's right hand when facing camera.
        """
        target = self._hand_side.capitalize()
        candidates = [d for d in detections if d.handedness == target]
        if not candidates:
            candidates = detections
        return max(candidates, key=lambda d: d.handedness_score)
