# GeoRT Phase 0 — Allegro 검증 walkthrough

목표: GeoRT 공식 Allegro 예제가 본 프로젝트 환경에서 종단간 동작함을 한 번
확인. DG-5F 통합은 별도 작업이며
[../../../docs/geort.md](../../../docs/geort.md) §9 의 Phase 1+ 에서 진행.

성공 기준:
- ✅ trainer 가 5가지 손실을 출력하며 수렴
- ✅ `checkpoint/allegro_right_*/{last,epoch_*}.pth` 생성
- ✅ Isaac Sim 의 Allegro hand 가 mocap 입력에 따라 움직임

전체 명령은 모두 `models/geort/` 에서 시작한다고 가정.

```bash
cd /workspaces/tamp_ws/src/retarget_dev/models/geort
conda activate teleop_operator   # docs/setup.md §1 — 기존 env 재사용
```

---

## Step 1 — install (1회)

```bash
bash scripts/install.sh
```

상세는 [setup.md](setup.md). 첫 실행 시 GeoRT 저장소를 clone + editable
install 하고 import 검증을 출력. 두 번째 실행부터는 idempotent.

성공 로그:
```
[install] cloning GeoRT into .../models/geort/GeoRT
...
geort      : .../GeoRT/geort/__init__.py
torch      : 2.x.x (cuda: True)
sapien     : 3.x.x
[install] done
```

---

## Step 2 — mocap 수집 (5분)

본 프로젝트는 두 가지 카메라 소스를 지원한다. 두 스크립트 모두 GeoRT 의
`MediaPipeHandDetector` + `MediaPipeHandProcessor` 를 그대로 재사용하므로
출력 데이터 (캐노니컬 프레임 (21, 3) per frame) 는 동일하다. 학습 단계에서
구분되지 않음.

| 스크립트 | 카메라 | 비고 |
|---|---|---|
| `collect_mocap_webcam.py` | OpenCV USB 웹캠 | GeoRT upstream 은 미지원 — 본 프로젝트가 추가 |
| `collect_mocap_realsense.py` | Intel RealSense (D405 등) | upstream `mediapipe_mocap.py` 와 동일 카메라이지만 인터페이스 통일 |

```bash
# 반드시 GeoRT/ 안에서 실행 — `hand_landmarker.task` 가 GeoRT 루트에 있음
cd GeoRT

# Option A: USB 웹캠
python3 ../scripts/collect_mocap_webcam.py --name human_alex

# Option B: RealSense (예: D405)
python3 ../scripts/collect_mocap_realsense.py --name human_alex \
    --width 640 --height 480 --fps 30
```

키 컨트롤 (OpenCV 창에 포커스 둔 상태):
- `s` — 녹화 시작
- `e` — 녹화 일시정지
- `q` — 종료 + 저장

권장 동작 (geort.md §8 허들 3 기반):
- 각 손가락을 펴고 자유롭게 움직이기 — 작업공간 탐색
- 다양한 pinch 조합 (엄지-검지, 엄지-중지, 엄지-약지, 엄지-새끼)
- 주먹 쥐기 / 펴기
- 손목 회전 (단, MediaPipe 는 손목 회전에 약하므로 너무 빠르지 않게)

목표: 약 5,000+ frame (≈ 30Hz × 3분).

성공 출력:
```
[collect] frames=30
[collect] frames=60
...
[collect] saved (5234, 21, 3) → .../GeoRT/data/human_alex.npy
```

---

## Step 3 — IK trainer (가장 위험한 단계)

```bash
cd /workspaces/tamp_ws/src/retarget_dev/models/geort
bash scripts/train_allegro.sh human_alex phase0
```

내부적으로는:
```bash
cd GeoRT
python3 -m geort.trainer -hand allegro_right -human_data human_alex -ckpt_tag phase0
```

trainer 는 두 단계로 동작:
1. **Neural FK 학습** (약 200 epoch, ~2분, RTX 3060 기준) — 100k 랜덤 joint
   샘플로 robot URDF 의 FK 를 신경망에 baking. 캐시되므로 2회차부터 skip.
2. **IK 학습** (200 epoch 기본, 1∼5분) — 5가지 손실 (Direction / Chamfer /
   Curvature / Pinch / Collision[비활성]) 으로 학습.

성공 로그:
```
Train Neural Forward Kinematics (FK) from Scratch
Neural FK Training Epoch: 0; Training Loss: 0.0123
...
Epoch 0 | Losses - Direction: -0.5xxx - Chamfer: 0.xxxx - Curvature: 0.xxxx ...
...
[train] done — checkpoints under: .../GeoRT/checkpoint/
allegro_right_2026-04-10_HH-MM-SS_phase0
allegro_right_last
```

위험 — SAPIEN Vulkan 의존성:
- `HandKinematicModel.build_from_config(config, render=False)` 가 trainer 의
  default. 따라서 정상적으로는 Vulkan 이 필요 없음.
- 만약 SAPIEN 이 trainer 단계에서 Vulkan 에러를 내면 setup.md §5 의 트러블
  슈팅 항목 적용. 그래도 막히면 사용자에게 보고하고 다음 옵션 검토:
  1. `geort/env/hand.py` 의 `sapien.Engine()` 호출에 headless 옵션 강제
  2. trainer 의 FK 단계만 Pinocchio 로 교체 (geort.md §4.1 — "SAPIEN/
     Pinocchio FK")
  3. 다른 PC 에서 학습 후 checkpoint 만 import

---

## Step 4 — checkpoint 확인

```bash
ls GeoRT/checkpoint/
# allegro_right_2026-04-10_HH-MM-SS_phase0/
# allegro_right_last/
ls GeoRT/checkpoint/allegro_right_last/
# config.json  epoch_0.pth  epoch_1.pth  ...  last.pth
```

`config.json` 안에는 학습 시 사용한 joint_order + joint limit 이 함께 저장
되어 있어서 inference 시 이것만으로 모델 복원이 가능하다.

빠른 sanity check (CUDA 필요):
```bash
cd GeoRT
python3 -c "
import numpy as np, geort
m = geort.load_model(tag='phase0')
fake_kp = np.random.randn(21, 3).astype('float32') * 0.05
q = m.forward(fake_kp)
print('qpos shape:', q.shape, 'sample:', q[:4])
"
```

---

## Step 5 — Isaac Sim replay

### 5.1 사전 요건
- Isaac Sim 에 Allegro right hand USD 가 로드되어 있어야 함
- USD 가 `JointState` 를 `/allegro_right/joint_commands` topic 에서 구독
- joint name 이 GeoRT 측 `joint_order` 와 일치해야 함

상세는 [isaacsim_allegro_bridge.md](isaacsim_allegro_bridge.md) — 불일치
시 처리 방법 포함.

### 5.2 file replay (저장된 mocap 재생)

```bash
# T1: Isaac Sim 켜고 Allegro hand 로드
# T2:
cd /workspaces/tamp_ws/src/retarget_dev/models/geort/GeoRT
source /opt/ros/humble/setup.bash
python3 ../scripts/isaacsim_replay.py \
  --tag phase0 \
  --source file \
  --data-file data/human_alex.npy \
  --hz 30 \
  --loop
```

### 5.3 webcam live replay

```bash
cd /workspaces/tamp_ws/src/retarget_dev/models/geort/GeoRT
source /opt/ros/humble/setup.bash
python3 ../scripts/isaacsim_replay.py \
  --tag phase0 \
  --source webcam
```

OpenCV 창에 손이 보이면 자동으로 추적 시작. 'q' 로 종료.

성공 신호:
```
[replay] loading checkpoint tag=phase0 epoch=0
[replay] joint_order (16): ['joint_0.0', 'joint_1.0', ...]
[replay] publishing → /allegro_right/joint_commands
[replay] frame=30 q[0:6]=[+0.12 -0.04 +0.31 ...]
```
+ Isaac Sim 의 Allegro hand 가 손 움직임에 따라 움직임.

---

## 다음 단계

Phase 0 가 성공하면 [../../../docs/geort.md](../../../docs/geort.md) §9 의
Phase 1 (DG-5F URDF + JSON 등록) 으로 진행. 현재 `models/geort/` 의 코드는
Phase 0 만 다루며, DG-5F 코드는 별도 PR 로 추가될 예정.
