"""Phone camera sensing configuration constants."""

# ── Camera defaults ──────────────────────────────────────
DEFAULT_CAMERA_URL = "0"           # Local webcam (fallback for testing)
DEFAULT_RESOLUTION = (640, 480)
DEFAULT_FPS = 30

# ── MediaPipe defaults ───────────────────────────────────
MP_NUM_HANDS = 1
MP_MIN_DETECTION_CONFIDENCE = 0.5
MP_MIN_TRACKING_CONFIDENCE = 0.5
MP_MODEL_COMPLEXITY = 1            # 0=lite, 1=full

# ── Hand landmark model ──────────────────────────────────
# MediaPipe Tasks API model file.  None = auto-download from Google.
MP_MODEL_PATH = None
