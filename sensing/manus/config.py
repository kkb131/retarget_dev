"""Manus glove sensing configuration constants."""

# ── Joint layout ─────────────────────────────────────────
NUM_FINGERS = 5
JOINTS_PER_FINGER = 4
NUM_JOINTS = NUM_FINGERS * JOINTS_PER_FINGER  # 20

FINGER_NAMES = ["Thumb", "Index", "Middle", "Ring", "Pinky"]
JOINT_SUFFIXES = ["MCP_Spread", "MCP_Stretch", "PIP_Stretch", "DIP_Stretch"]

# 20 joint names (for JointState / logging)
MANUS_JOINT_NAMES = [f"{f}_{s}" for f in FINGER_NAMES for s in JOINT_SUFFIXES]

# ── Skeleton node layout (MediaPipe/AnyTeleop compatible) ─
NUM_SKELETON_NODES = 21
WRIST_IDX = 0
FINGERTIP_INDICES = [4, 8, 12, 16, 20]

# ── Defaults ─────────────────────────────────────────────
DEFAULT_HZ = 60
DEFAULT_HAND_SIDE = "right"
DEFAULT_SDK_BIN = "sdk/SDKClient_Linux.out"  # relative to this module
