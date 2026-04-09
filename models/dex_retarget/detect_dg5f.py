#!/usr/bin/env python3
"""Detect hand pose from video and retarget to DG-5F using dex-retargeting.

Uses our custom DG-5F YAML config instead of built-in robot configs.
Reuses dex-retargeting's SingleHandDetector for MediaPipe detection.

Usage:
    cd /workspaces/tamp_ws/src

    # mp4 → DG-5F pkl (vector retargeting)
    python3 -m retarget_dev.models.dex_retarget.detect_dg5f \
        --video-path retarget_dev/dex-retargeting/example/vector_retargeting/data/human_hand_video.mp4 \
        --config retarget_dev/models/dex_retarget/config/dg5f_right_vector.yml \
        --output-path /tmp/dg5f_joints.pkl

    # mp4 → DG-5F pkl (DexPilot)
    python3 -m retarget_dev.models.dex_retarget.detect_dg5f \
        --video-path retarget_dev/dex-retargeting/example/vector_retargeting/data/human_hand_video.mp4 \
        --config retarget_dev/models/dex_retarget/config/dg5f_right_dexpilot.yml \
        --output-path /tmp/dg5f_dexpilot.pkl
"""

import argparse
import pickle
import sys
from pathlib import Path

import cv2
import numpy as np
import tqdm

from dex_retargeting.retargeting_config import RetargetingConfig

# Add dex-retargeting example dir to path for SingleHandDetector
_EXAMPLE_DIR = str(
    Path(__file__).resolve().parent.parent.parent
    / "dex-retargeting" / "example" / "vector_retargeting"
)
sys.path.insert(0, _EXAMPLE_DIR)
from single_hand_detector import SingleHandDetector


def detect_and_retarget(config_path: str, video_path: str, output_path: str,
                        hand_type: str = "Right"):
    config = RetargetingConfig.load_from_file(config_path)
    retargeting = config.build()

    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        print(f"Error: Could not open video: {video_path}")
        return

    detector = SingleHandDetector(hand_type=hand_type, selfie=False)
    length = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    data = []

    with tqdm.tqdm(total=length) as pbar:
        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                break

            rgb = frame[..., ::-1]
            num_box, joint_pos, keypoint_2d, mediapipe_wrist_rot = detector.detect(rgb)
            if num_box == 0:
                pbar.update(1)
                continue

            # Build ref_value based on optimizer type
            retargeting_type = retargeting.optimizer.retargeting_type
            indices = retargeting.optimizer.target_link_human_indices

            if retargeting_type == "POSITION":
                ref_value = joint_pos[indices, :]
            else:
                origin_indices = indices[0, :]
                task_indices = indices[1, :]
                ref_value = joint_pos[task_indices, :] - joint_pos[origin_indices, :]

            qpos = retargeting.retarget(ref_value)
            data.append(qpos)
            pbar.update(1)

    cap.release()

    if not data:
        print("No hand detected in video!")
        return

    # Save pkl
    meta_data = dict(
        config_path=str(config_path),
        dof=len(retargeting.optimizer.robot.dof_joint_names),
        joint_names=retargeting.optimizer.robot.dof_joint_names,
    )

    out = Path(output_path)
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("wb") as f:
        pickle.dump(dict(data=data, meta_data=meta_data), f)

    print(f"\nSaved {len(data)} frames to {output_path}")
    print(f"  DOF: {meta_data['dof']}")
    print(f"  Joints: {meta_data['joint_names'][:5]} ...")
    retargeting.verbose()


def main():
    parser = argparse.ArgumentParser(description="Detect hand from video → DG-5F pkl")
    parser.add_argument("--video-path", required=True, help="Input mp4 path")
    parser.add_argument("--config", required=True, help="dex-retargeting YAML config")
    parser.add_argument("--output-path", required=True, help="Output pkl path")
    parser.add_argument("--hand-type", default="Right", choices=["Left", "Right"])
    args = parser.parse_args()

    detect_and_retarget(args.config, args.video_path, args.output_path, args.hand_type)


if __name__ == "__main__":
    main()
