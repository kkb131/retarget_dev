# 사용법

## 1. Allegro Hand 기본 예제 (동작 확인용)

dex-retargeting 공식 예제. 환경 설정이 잘 되었는지 확인하는 용도.

### 1-1. mp4 → Allegro pkl

```bash
cd /workspaces/tamp_ws/src/retarget_dev/dex-retargeting/example/vector_retargeting

python3 detect_from_video.py \
  --robot-name allegro \
  --video-path data/human_hand_video.mp4 \
  --retargeting-type dexpilot \
  --hand-type right \
  --output-path data/allegro_joints.pkl
```

출력:
```
621 frames 처리 완료 (~2.3초)
→ data/allegro_joints.pkl 생성 (16 DOF)
```

### 1-2. pkl → Isaac Sim (Allegro)

Isaac Sim에서 Allegro Hand가 `/joint_states` pub, `/joint_command` sub로 실행 중일 때:

```bash
cd /workspaces/tamp_ws/src
python3 -m retarget_dev.models.dex_retarget.play_pkl \
  --pkl-path retarget_dev/dex-retargeting/example/vector_retargeting/data/allegro_joints.pkl \
  --topic /joint_command --hz 30
```

Isaac Sim에서 Allegro 손가락이 영상 속 손 동작을 따라 움직이면 성공.

## 2. DG-5F 파이프라인

### 2-1. mp4 → DG-5F pkl

```bash
cd /workspaces/tamp_ws/src

# 권장: DexPilot (pinch 보존 우수)
python3 -m retarget_dev.models.dex_retarget.detect_dg5f \
  --video-path retarget_dev/dex-retargeting/example/vector_retargeting/data/human_hand_video.mp4 \
  --config retarget_dev/models/dex_retarget/config/dg5f_right_dexpilot.yml \
  --output-path /tmp/dg5f_joints.pkl

# 대안: Vector (더 빠름, 일반 동작)
python3 -m retarget_dev.models.dex_retarget.detect_dg5f \
  --video-path retarget_dev/dex-retargeting/example/vector_retargeting/data/human_hand_video.mp4 \
  --config retarget_dev/models/dex_retarget/config/dg5f_right_vector.yml \
  --output-path /tmp/dg5f_joints.pkl
```

출력: 20 DOF × 621 frames

### 2-2. pkl → Isaac Sim (DG-5F)

```bash
python3 -m retarget_dev.models.dex_retarget.play_pkl \
  --pkl-path /tmp/dg5f_joints.pkl \
  --topic /dg5f_right/joint_commands --hz 30
```

### 2-3. 루프 재생

```bash
python3 -m retarget_dev.models.dex_retarget.play_pkl \
  --pkl-path /tmp/dg5f_joints.pkl \
  --topic /dg5f_right/joint_commands --hz 30 --loop
```

## 3. ROS2 토픽 구조

### 3-1. Allegro Hand (Isaac Sim)

| 토픽 | 방향 | 순서 |
|---|---|---|
| `/joint_states` | Isaac Sim → | finger-by-joint_idx (index_0, middle_0, ring_0, thumb_0, index_1, ...) |
| `/joint_command` | → Isaac Sim | 위와 동일 |

### 3-2. DG-5F (Isaac Sim)

| 토픽 | 방향 | 순서 |
|---|---|---|
| `/dg5f_right/joint_states` | Isaac Sim → | joint_idx-by-finger (rj_dg_1_1, rj_dg_2_1, ..., rj_dg_5_1, rj_dg_1_2, ...) |
| `/dg5f_right/joint_commands` | → Isaac Sim | 위와 동일 |

**중요**: Isaac Sim과 dex-retargeting의 joint 순서가 다릅니다.
`play_pkl.py`는 JointState 메시지에 `name` 필드를 함께 publish하므로
Isaac Sim이 자동으로 name 기반 매칭을 합니다 (순서 무관).

## 4. Joint Name 매핑

### 4-1. DG-5F (순서 매핑 불필요)

dex-retargeting이 URDF에서 파싱한 joint name:
```
rj_dg_1_1, rj_dg_1_2, rj_dg_1_3, rj_dg_1_4,   # Thumb
rj_dg_2_1, rj_dg_2_2, rj_dg_2_3, rj_dg_2_4,   # Index
...
rj_dg_5_1, rj_dg_5_2, rj_dg_5_3, rj_dg_5_4    # Pinky
```

Isaac Sim DG-5F도 동일한 name을 사용하므로 **직접 매칭 가능**.

### 4-2. Allegro (name 변환 필요)

dex-retargeting 출력 (URDF name):
```
joint_0.0, joint_1.0, ..., joint_15.0
```

Isaac Sim Allegro name:
```
index_joint_0, middle_joint_0, ring_joint_0, thumb_joint_0, index_joint_1, ...
```

`play_pkl.py`의 `_ALLEGRO_DEX_TO_ISAAC` 딕셔너리가 자동 매핑:
```python
"joint_0.0" → "index_joint_0"
"joint_4.0" → "middle_joint_0"
"joint_12.0" → "thumb_joint_0"
...
```

`resolve_names()` 함수가 joint name prefix로 로봇을 자동 감지:
- `rj_dg_*` → DG-5F (그대로 사용)
- `joint_*` → Allegro (매핑 적용)

## 5. 실시간 리타게팅 (카메라 입력)

mp4 대신 카메라/Manus 입력으로 실시간 동작:

```bash
# Phone 카메라 (IP Webcam 앱) — 자세한 내용은 phone_realtime.md
python3 -m retarget_dev.models.dex_retarget.main \
  --sensing phone \
  --config retarget_dev/models/dex_retarget/config/dg5f_right_dexpilot.yml \
  --url http://192.168.0.3:8080/video

# RealSense D405 — 자세한 내용은 realsense_d405.md
python3 -m retarget_dev.models.dex_retarget.main \
  --sensing realsense \
  --config retarget_dev/models/dex_retarget/config/dg5f_right_dexpilot.yml \
  --topic /dg5f_right/joint_commands --hz 30

# Manus ROS2
python3 -m retarget_dev.models.dex_retarget.main \
  --sensing manus-ros2 \
  --config retarget_dev/models/dex_retarget/config/dg5f_right_dexpilot.yml
```

각 프레임마다 즉시 `/dg5f_right/joint_commands`에 publish.

### MANO 좌표계 변환 (phone 실시간 사용 시 필수)

Phone 입력을 dex-retargeting에 전달할 때는 `PhoneSensing`의 `KeypointConverter`가
MediaPipe world_landmarks를 **MANO 좌표계**로 변환해야 합니다.

변환 파이프라인:
1. wrist(0)를 원점으로 이동
2. wrist, index_MCP(5), middle_MCP(9)로 **palm 평면 추정** (SVD)
3. 추정된 wrist 프레임을 basis로 회전
4. 고정 `operator2mano` 회전 적용 (right: `[[0,0,-1],[-1,0,0],[0,1,0]]`)

**이 변환 없이는** 카메라 시점에 따라 손의 절대 방향이 벡터에 그대로 들어가서,
optimizer가 잘못된 포즈를 만들어 냅니다.

참조: `retarget_dev/dex-retargeting/example/vector_retargeting/single_hand_detector.py`
의 `estimate_frame_from_hand_points()` 함수.

우리 구현: `retarget_dev/sensing/phone/keypoint_converter.py` (dex-retargeting 로직 이식).
기본적으로 `apply_mano_transform=True`이므로 `PhoneSensing` 사용 시 자동 적용됩니다.

## 6. 검증 명령어

```bash
# 현재 발행 중인 토픽 확인
ros2 topic list | grep dg5f

# 관절 상태 실시간 확인
ros2 topic echo /dg5f_right/joint_states --once

# 발행 주기 확인
ros2 topic hz /dg5f_right/joint_commands
```
