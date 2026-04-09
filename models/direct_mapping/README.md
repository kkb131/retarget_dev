# 1세대 Direct Mapping 리타게팅 모델

MediaPipe 21개 3D 키포인트에서 관절각을 직접 계산하여 DG5F 핸드에 매핑하는 가장 단순한 리타게팅 방식.

## 알고리즘

```
HandKeypoints (21, 3)
    ↓ angle_extractor.extract_all_angles()
Human angles (20,)     ← [spread, MCP, PIP, DIP] × 5 fingers
    ↓ delta = baseline - human (flexion), human - baseline (spread)
Delta (20,)            ← 0 = open hand, + = flexion
    ↓ q = scale * delta + offset
DG5F angles (20,)      ← clip to joint limits
```

### Flexion (굽힘각) 계산

각 관절에서 들어오는 뼈(parent→joint)와 나가는 뼈(joint→child)의 사이각:

```
raw_angle = π - arccos(dot(v_in, v_out) / (|v_in| × |v_out|))
```

**주의: raw_angle의 convention은 DG5F와 반대 방향**

| 상태 | raw_angle | delta (baseline=166°) | DG5F |
|---|---|---|---|
| 손 펴짐 (extended) | ~166° (≈π) | 0° | 0° |
| 주먹 (flexed) | ~81° | 85° | 85° × scale |

- `compute_flexion` 출력: **π(180°)=펴짐, 0°=접힘** (접을수록 값 감소)
- DG5F convention: **0°=펴짐, +=접힘** (접을수록 값 증가)
- 따라서 delta 계산 시 **`baseline - human`** (방향 반전)

### Spread (벌림각) 계산

팜 평면 기준으로 중지(Middle) 방향 대비 각 손가락의 좌우 벌림:

```
palm_normal = cross(WRIST→INDEX, WRIST→PINKY)
reference   = project(WRIST→MIDDLE, palm_plane)
finger_dir  = project(WRIST→finger_base, palm_plane)
spread      = atan2(sin, cos)   ← signed angle
```

Spread는 `human - baseline` (부호 유지).

### Thumb MCP 특수 처리

DG5F Thumb MCP (`rj_dg_1_2`)는 다른 손가락과 **반대 방향**:

| Joint | 범위 | flexion 방향 |
|---|---|---|
| Thumb MCP | [-180°, 0°] | **음수** = flexion |
| 나머지 MCP | [0°, +115°] | **양수** = flexion |

따라서 Thumb MCP의 delta에 `-1`을 곱하여 부호 반전:
```python
delta[1] = -delta[1]  # Thumb MCP: 양수 delta → 음수 DG5F
```

## Calibration

### 2-Point Calibration (권장)

```bash
python3 -m retarget_dev.models.direct_mapping.main \
    --sensing phone --calibrate --save-config my_config.yaml
```

**Step 1**: 손 펴고 (spread) → Enter → baseline 캡처 (30 frames 평균)
**Step 2**: 주먹 쥐고 (fist) → Enter → scale 자동 계산

```
scale[i] = q_max[i] / (baseline[i] - fist_angle[i])
```

### Baseline Only

```bash
python3 -m retarget_dev.models.direct_mapping.main \
    --sensing phone --calibrate --save-config my_config.yaml
```

Step 1만 수행 후 Ctrl+C → baseline만 저장, scale=1.0 (기본)

### Config 재사용

```bash
python3 -m retarget_dev.models.direct_mapping.main \
    --sensing phone --config my_config.yaml
```

### config.yaml 구조

```yaml
baseline:        # (20,) open hand raw angles (radians)
  - 0.0          # Thumb spread
  - 2.87         # Thumb MCP (~164°)
  - 2.82         # Thumb PIP
  - ...
scale_factors:   # (20,) q_max / human_ROM (auto-computed by fist calibration)
  - 1.0          # Thumb spread
  - 2.50         # Thumb MCP
  - ...
offsets:         # (20,) 보통 0.0 (2-point calibration에서는 불필요)
  - 0.0
  - ...
```

## 사용법

### 기본 실행 (IsaacSim 연동)

```bash
# Phone 카메라 입력 → DG5F 명령 퍼블리시
python3 -m retarget_dev.models.direct_mapping.main \
    --sensing phone --hand right

# Manus mock 입력 (테스트용)
python3 -m retarget_dev.models.direct_mapping.main \
    --sensing manus-mock --hand right
```

### 콘솔 전용 (ROS2 없이)

```bash
python3 -m retarget_dev.models.direct_mapping.main \
    --sensing manus-mock --hand right --no-ros2
```

### CLI 옵션

| 옵션 | 기본값 | 설명 |
|---|---|---|
| `--sensing` | (필수) | `phone`, `manus-mock`, `manus-sdk` |
| `--hand` | `right` | `left` / `right` |
| `--hz` | `30` | 루프 주기 |
| `--url` | `http://192.168.0.3:8080/video` | Phone 카메라 URL |
| `--config` | None | YAML config 경로 |
| `--calibrate` | False | 2-point calibration 수행 |
| `--save-config` | None | calibration 결과 저장 경로 |
| `--topic` | `/dg5f_right/joint_commands` | ROS2 JointState 토픽 |
| `--no-ros2` | False | ROS2 비활성화 |

## 관절 매핑 상세

| DG5F Joint | 의미 | 범위 | MediaPipe 키포인트 |
|---|---|---|---|
| `rj_dg_1_1` | Thumb spread | [-22°, +51°] | 팜 평면 기준 WRIST→CMC(1) |
| `rj_dg_1_2` | Thumb MCP flex | [-180°, 0°] **음수=flex** | CMC(1)→MCP(2) vs WRIST(0)→CMC(1) |
| `rj_dg_1_3` | Thumb PIP flex | [-90°, +90°] | MCP(2)→IP(3) vs CMC(1)→MCP(2) |
| `rj_dg_1_4` | Thumb DIP flex | [-90°, +90°] | IP(3)→TIP(4) vs MCP(2)→IP(3) |
| `rj_dg_2_1` | Index spread | [-24°, +35°] | 팜 평면 기준 WRIST→MCP(5) |
| `rj_dg_2_2` | Index MCP flex | [0°, +115°] | MCP(5)→PIP(6) vs WRIST(0)→MCP(5) |
| `rj_dg_2_3` | Index PIP flex | [-90°, +90°] | PIP(6)→DIP(7) vs MCP(5)→PIP(6) |
| `rj_dg_2_4` | Index DIP flex | [-90°, +90°] | DIP(7)→TIP(8) vs PIP(6)→DIP(7) |
| ... | (Middle, Ring, Pinky 동일 패턴) | ... | ... |

## 시행착오 & 핵심 교훈

### 1. Flexion convention 반전 문제

**문제**: `compute_flexion`은 `π-arccos` → 펴짐=180°, 접힘=0°. DG5F는 펴짐=0°, 접힘=+.
**증상**: baseline calibration 후 주먹을 쥐면 delta가 음수 → `max(0)` clamp → 0°로 잘림.
**해결**: flexion joints에 `delta = baseline - human` (방향 반전). Spread는 `human - baseline`.

### 2. Baseline이 0일 때 vs calibrated일 때 변화량 차이

**문제**: baseline=0이면 human raw값(81~170°)이 그대로 delta → 변화량 큼. baseline=166°이면 delta 최대 14° → 변화량 극소.
**원인**: `human - baseline` 방향이었을 때, flexion(값 감소)이 음수가 되어 clamp됨.
**해결**: 방향 반전으로 해결 (위 1번).

### 3. Default scale_factors 문제

**문제**: 초기 기본값 `MCP=0.64, PIP=0.5`는 baseline=0 + human ROM=180° 가정. Calibrated baseline에서는 human ROM이 ~85°로 줄어 scale이 부족.
**해결**: 기본 scale을 1.0으로 변경. Fist calibration (2-point)으로 정확한 scale 자동 계산.

### 4. Thumb MCP 음수 방향

**문제**: DG5F Thumb MCP 범위가 [-180°, 0°] (음수=flexion). 양수 delta가 clip에 의해 0°로 잘림.
**해결**: Thumb MCP (index 1)의 delta에 `-1` 곱하여 부호 반전.

## 한계

- **최적화 없음**: 사람과 로봇의 기구학적 차이를 보상하지 않음
- **Calibration 의존**: baseline + scale 튜닝 필수 (2-point calibration 권장)
- **깊이 노이즈**: 단안 카메라의 z-depth 노이즈가 각도 계산에 직접 영향
- **Spread 불안정**: 팜 평면 추정이 노이즈에 민감
- **선형 매핑**: 실제 손가락 관절의 비선형 커플링을 반영하지 못함

→ 이러한 한계를 해결하기 위해 **dex-retargeting (3세대)** 기반 모델 사용 (`models/dex_retarget/`)
