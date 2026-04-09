"""Threaded camera capture with minimal latency.

Solves the well-known RTSP/MJPEG buffering problem by continuously
grabbing frames in a background thread and keeping only the latest one.

Supported source strings:
    "0", "1"                                  → local USB webcam / laptop camera
    "http://192.168.0.100:8080/video"         → IP Webcam MJPEG
    "rtsp://192.168.0.100:8080/h264_pcm.sdp"  → IP Webcam RTSP
    "/path/to/video.mp4"                       → offline video file
"""

import logging
import threading
import time

import cv2
import numpy as np

from retarget_dev.sensing.phone.config import DEFAULT_RESOLUTION, DEFAULT_FPS

logger = logging.getLogger(__name__)


class CameraSource:
    """Threaded camera reader that always provides the latest frame."""

    def __init__(
        self,
        source: str = "0",
        resolution: tuple[int, int] = DEFAULT_RESOLUTION,
        fps: int = DEFAULT_FPS,
        mirror: bool = False,
    ):
        self._source_str = source
        self._resolution = resolution
        self._target_fps = fps
        self._mirror = mirror

        self._cap: cv2.VideoCapture | None = None
        self._frame: np.ndarray | None = None
        self._lock = threading.Lock()
        self._running = False
        self._thread: threading.Thread | None = None

        # FPS measurement
        self._frame_count = 0
        self._fps_start = 0.0
        self._actual_fps = 0.0

    # ── Public API ────────────────────────────────────────

    def start(self) -> None:
        """Open the camera and start the grab thread."""
        source = self._parse_source(self._source_str)
        self._cap = cv2.VideoCapture(source)
        if not self._cap.isOpened():
            raise RuntimeError(f"Cannot open camera source: {self._source_str}")

        # Try to set resolution and fps
        w, h = self._resolution
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
        self._cap.set(cv2.CAP_PROP_FPS, self._target_fps)
        # Minimize buffer for network streams
        self._cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        self._running = True
        self._fps_start = time.monotonic()
        self._frame_count = 0
        self._thread = threading.Thread(target=self._grab_loop, daemon=True)
        self._thread.start()
        logger.info("CameraSource started: %s (%dx%d)", self._source_str, w, h)

    def stop(self) -> None:
        """Release the camera and stop the grab thread."""
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        if self._cap is not None:
            self._cap.release()
            self._cap = None
        logger.info("CameraSource stopped.")

    def read(self) -> tuple[bool, np.ndarray | None]:
        """Return the most recently grabbed frame (BGR).

        Returns (True, frame) on success, (False, None) if no frame yet.
        """
        with self._lock:
            if self._frame is None:
                return False, None
            frame = self._frame.copy()
        if self._mirror:
            frame = cv2.flip(frame, 1)
        return True, frame

    def is_opened(self) -> bool:
        return self._cap is not None and self._cap.isOpened()

    @property
    def actual_fps(self) -> float:
        return self._actual_fps

    # ── Private ───────────────────────────────────────────

    def _grab_loop(self) -> None:
        """Background thread: continuously grab frames, keep only latest."""
        while self._running:
            if self._cap is None or not self._cap.isOpened():
                break
            ok = self._cap.grab()
            if not ok:
                time.sleep(0.001)
                continue
            ok, frame = self._cap.retrieve()
            if ok and frame is not None:
                with self._lock:
                    self._frame = frame
                self._frame_count += 1
                elapsed = time.monotonic() - self._fps_start
                if elapsed > 1.0:
                    self._actual_fps = self._frame_count / elapsed
                    self._frame_count = 0
                    self._fps_start = time.monotonic()

    @staticmethod
    def _parse_source(source_str: str) -> int | str:
        """Convert source string to int (device index) or keep as URL/path."""
        try:
            return int(source_str)
        except ValueError:
            return source_str

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, *args):
        self.stop()
