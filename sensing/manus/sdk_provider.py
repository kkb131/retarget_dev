"""Manus SDK provider — reads glove data via SDKClient_Linux subprocess.

Launches the C++ SDK binary with ``--stream-json`` and parses
newline-delimited JSON from stdout. Independent of teleop_dev.

The SDK binary and ManusSDK libraries must be present.
Default location: ``sensing/manus/sdk/SDKClient_Linux.out``
or specify via ``sdk_bin_path`` parameter.
"""

import json
import logging
import os
import re
import subprocess
import threading
import time
from pathlib import Path
from typing import Optional

import numpy as np

from retarget_dev.sensing.manus.config import NUM_FINGERS, NUM_JOINTS
from retarget_dev.sensing.manus.manus_hand_data import HandData

logger = logging.getLogger(__name__)

_ANSI_RE = re.compile(r'\x1b\[[0-9;]*[A-Za-z]')


class SdkManusProvider:
    """Reads hand data from Manus gloves via SDK subprocess."""

    def __init__(self, sdk_bin_path: str, hand_side: str = "right"):
        self._bin_path = Path(sdk_bin_path)
        self._hand_side = hand_side
        self._proc: Optional[subprocess.Popen] = None
        self._connected = False

        self._lock = threading.Lock()
        self._left_data: Optional[HandData] = None
        self._right_data: Optional[HandData] = None
        self._data_received = threading.Event()
        self._reader_thread: Optional[threading.Thread] = None

    def start(self) -> None:
        """Launch SDK binary and start reading JSON."""
        if not self._bin_path.exists():
            raise FileNotFoundError(
                f"SDK binary not found: {self._bin_path.resolve()}\n"
                f"Build it first: cd {self._bin_path.parent} && make"
            )

        bin_resolved = self._bin_path.resolve()
        cwd = bin_resolved.parent
        sdk_lib_dir = cwd / "ManusSDK" / "lib"

        env = dict(os.environ)
        ld_path = env.get("LD_LIBRARY_PATH", "")
        env["LD_LIBRARY_PATH"] = f"{sdk_lib_dir}:{ld_path}" if ld_path else str(sdk_lib_dir)

        logger.info("Launching SDK: %s --stream-json (cwd=%s)", bin_resolved, cwd)
        self._proc = subprocess.Popen(
            [str(bin_resolved), "--stream-json"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            cwd=str(cwd),
            env=env,
        )
        self._connected = True

        self._reader_thread = threading.Thread(target=self._read_loop, daemon=True)
        self._reader_thread.start()

        # Wait for first data
        logger.info("Waiting for Manus data...")
        if self._data_received.wait(timeout=10.0):
            logger.info("Manus data received.")
        else:
            logger.warning("Timeout waiting for Manus data.")

    def stop(self) -> None:
        """Terminate the SDK subprocess."""
        self._connected = False
        if self._proc is not None:
            self._proc.terminate()
            try:
                self._proc.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                self._proc.kill()
            self._proc = None
        logger.info("SdkManusProvider stopped.")

    def get_hand_data(self, side: str | None = None) -> Optional[HandData]:
        """Get latest hand data for the specified side."""
        target = side or self._hand_side
        with self._lock:
            if target == "left":
                return self._left_data
            return self._right_data

    def _read_loop(self) -> None:
        """Parse JSON lines from SDK subprocess stdout."""
        try:
            while self._connected and self._proc and self._proc.stdout:
                raw_line = self._proc.stdout.readline()
                if not raw_line:
                    break
                line = _ANSI_RE.sub('', raw_line.decode(errors="replace")).strip()
                if not line or not line.startswith("{"):
                    continue

                try:
                    pkt = json.loads(line)
                except json.JSONDecodeError:
                    continue

                if pkt.get("type") != "manus":
                    continue

                hand_side = pkt.get("hand", "right")
                angles_deg = pkt.get("joint_angles", [])
                spread_deg = pkt.get("finger_spread", [])

                if len(angles_deg) != NUM_JOINTS:
                    continue

                joint_angles = np.deg2rad(np.array(angles_deg, dtype=np.float32))
                finger_spread = (
                    np.deg2rad(np.array(spread_deg[:NUM_FINGERS], dtype=np.float32))
                    if len(spread_deg) >= NUM_FINGERS
                    else np.zeros(NUM_FINGERS, dtype=np.float32)
                )

                skel_raw = pkt.get("skeleton")
                skel_arr = None
                has_skel = pkt.get("has_skeleton", False)
                if skel_raw and isinstance(skel_raw, list):
                    try:
                        skel_arr = np.array(skel_raw, dtype=np.float32)
                    except (ValueError, TypeError):
                        skel_arr = None

                hd = HandData(
                    joint_angles=joint_angles,
                    finger_spread=finger_spread,
                    wrist_pos=np.array(pkt.get("wrist_pos", [0, 0, 0]), dtype=np.float32),
                    wrist_quat=np.array(pkt.get("wrist_quat", [1, 0, 0, 0]), dtype=np.float32),
                    hand_side=hand_side,
                    timestamp=pkt.get("timestamp", time.time()),
                    skeleton=skel_arr,
                    has_skeleton=has_skel,
                )

                with self._lock:
                    if hand_side == "left":
                        self._left_data = hd
                    else:
                        self._right_data = hd

                self._data_received.set()

        except Exception as e:
            logger.error("SDK reader error: %s", e)
