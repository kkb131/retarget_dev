# dex-retargeting for DG-5F

[dex-retargeting](https://github.com/dexsuite/dex-retargeting) 라이브러리를 사용한 Manus/Phone/MP4 → DG-5F 리타게팅 파이프라인.

## 개요

```
mp4 / 카메라 / Manus
    ↓ MediaPipe (21-point hand detection)
Human keypoints (21, 3)
    ↓ dex-retargeting (Vector / DexPilot optimizer)
DG-5F joint angles (20)
    ↓ ROS2 JointState publish
Isaac Sim DG-5F (/dg5f_right/joint_commands)
```

## 빠른 시작

```bash
# 1) 환경 설정 (최초 1회)
# → setup.md 참고

# 2) mp4 → DG-5F pkl 생성
cd /workspaces/tamp_ws/src
python3 -m retarget_dev.models.dex_retarget.detect_dg5f \
  --video-path retarget_dev/dex-retargeting/example/vector_retargeting/data/human_hand_video.mp4 \
  --config retarget_dev/models/dex_retarget/config/dg5f_right_dexpilot.yml \
  --output-path /tmp/dg5f_joints.pkl

# 3) pkl → Isaac Sim 재생
python3 -m retarget_dev.models.dex_retarget.play_pkl \
  --pkl-path /tmp/dg5f_joints.pkl \
  --topic /dg5f_right/joint_commands --hz 30
```

## 문서

| 파일 | 내용 |
|---|---|
| [setup.md](setup.md) | 환경 설정 (pip, clone, 의존성, numpy/mediapipe 제약) |
| [usage.md](usage.md) | Allegro/DG-5F 실행법, ROS2 토픽 구조, joint name 매핑 |
| [dg5f_tuning.md](dg5f_tuning.md) | DG-5F 튜닝 경험 (scaling, vector vs DexPilot, URDF 수정) |
| [phone_realtime.md](phone_realtime.md) | Phone (IP Webcam) → 실시간 DG-5F, MANO 변환 파이프라인 |
| [realsense_d405.md](realsense_d405.md) | RealSense D405 → 실시간 DG-5F, real depth 기반 정밀 제어 |
| [manus_realtime.md](manus_realtime.md) | Manus Glove → 실시간 DG-5F, 25→21 raw skeleton 리매핑 이슈 포함 |

## 파일 구조

```
retarget_dev/models/dex_retarget/
├── config/
│   ├── dg5f_right_vector.yml        # Vector optimizer config
│   ├── dg5f_right_dexpilot.yml      # DexPilot optimizer config (권장)
│   └── dg5f_right_retarget.urdf     # PIP/DIP 음수 제한 URDF (튜닝됨)
├── docs/                            # ← 이 문서들
├── dex_retarget_model.py            # RetargetingModel ABC wrapper
├── detect_dg5f.py                   # mp4 → DG-5F pkl
├── play_pkl.py                      # pkl → Isaac Sim ROS2 publish
└── main.py                          # sensing → 실시간 retarget → ROS2
```

## 참고

- 이론/수학 분석: [retarget_dev/docs/dex_retargeting.md](../../../docs/dex_retargeting.md)
- dex-retargeting 공식: https://github.com/dexsuite/dex-retargeting
- AnyTeleop 논문: Qin et al., RSS 2023
