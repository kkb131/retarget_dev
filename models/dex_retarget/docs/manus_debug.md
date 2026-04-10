# Manus → DG-5F 역방향 동작 디버그 기록

Manus 글러브 → dex-retargeting → DG-5F 경로에서 발견된 두 개의 독립적인 버그와
그 해결 과정을 기록한다. 향후 유사한 증상 (sensor 추가 시 frame 누락, MANO index
off-by-one) 이 다시 발생할 때 reading order 로 사용한다.

## 1. 증상

- **사용자 보고**: Manus 글러브로 fist (주먹) 자세를 취하면 DG-5F 로봇 손가락이
  **spread (펴짐)** 방향으로 움직임. 정반대 동작.
- **영향 범위**: Manus → dex_retarget → DG-5F 경로 한정.
- **정상 경로**: Phone (MediaPipe) → DG-5F 경로는 (대체로) 정상으로 보고됨.
- **버전**: dex_retargeting `v0.5.0`, DG-5F `dg5f_right_retarget.urdf`,
  `dg5f_right_vector.yml` (vector mode), 작업 일자 2026-04-10.

## 2. 의심 요소 4가지 (조사 시작 시점)

1. **★ Suspect 1** — `ManusSensing` 이 MANO frame 회전을 건너뜀.
2. **★ Suspect 2** — `dg5f_right_vector.yml` 의 `target_link_human_indices` 가
   off-by-one (MANO 인덱싱 혼동).
3. **(가능성 낮음) Suspect 3** — DG-5F URDF joint limit 이 음수 굽힘 허용.
4. **(가능성 낮음) Suspect 4** — Manus 25 → 21 MANO remap 매핑 오류.

각 항목의 상세 분석과 검증 결과는 §3 에서 다룬다.

## 3. 진단 결과

### 3.1 Suspect 3 — URDF joint limit (검증: EXCLUDED)

**가설**: PIP/DIP joint limit 이 `[-π/2, π/2]` 라 optimizer 가 음수 (손등 쪽 굽힘)
해를 선택. `dg5f_tuning.md` 의 이전 버그 기록과 동일한 패턴인지 확인.

**검증**: [`dg5f_right_retarget.urdf`](../config/dg5f_right_retarget.urdf) 의
`rj_dg_*_3` 와 `rj_dg_*_4` joint 의 `<limit>` 태그를 직접 확인:

```xml
<joint name="rj_dg_2_3" type="revolute">
  ...
  <limit lower="0.0" upper="1.5707963267948966" effort="7.5" velocity="3.141592653589793"/>
</joint>
<joint name="rj_dg_2_4" type="revolute">
  ...
  <limit lower="0.0" upper="1.5707963267948966" effort="7.5" velocity="3.141592653589793"/>
</joint>
```

전체 손가락 (1~5, joint 3 과 4 각각) 모두 `lower="0.0"`, `upper=π/2`. **이미 양수
방향만 허용** (= 손등 쪽 굽힘 차단).

**결론**: Suspect 3 은 `dg5f_tuning.md` 시점에 이미 fix 되어 있음. 현재 증상의
원인이 아님.

### 3.2 Suspect 4 — 25 → 21 MANO remap (검증: EXCLUDED)

**가설**: [`sensing/manus/ros2_provider.py`](../../../sensing/manus/ros2_provider.py)
의 `_MANO_REMAP` 가 잘못된 매핑.

**검증**: `_MANO_REMAP` 의 모든 항목을 [`ManusDataPublisher.cpp:669-686`](../../../../teleop_dev/sender/hand/sdk/ROS2/manus_ros2/src/ManusDataPublisher.cpp#L669-L686)
의 `JointTypeToString` 출력 + [`sensing/common.py:18-35`](../../../sensing/common.py#L18-L35)
의 `KEYPOINT_NAMES` (MANO 21) 와 대조.

publisher 의 string label 이 SDK enum 과 anatomical 의미가 한 단계 어긋난다는 것을
이미 [`manus_realtime.md §5-4`](manus_realtime.md#5-4-함정-publisher의-string-라벨이-해부학과-어긋남)
에서 다룸:

| SDK enum | publisher string | anatomical |
|---|---|---|
| `Metacarpal` | `"MCP"` | CMC |
| `Proximal` | `"PIP"` | MCP (knuckle) |
| `Intermediate` | `"IP"` | PIP |
| `Distal` | `"DIP"` | DIP |
| `Tip` | `"TIP"` | TIP |

`_MANO_REMAP` 는 publisher string 을 정확히 MANO index 로 매핑:
- `("Index", "PIP"): 5` ← publisher `"PIP"` (= SDK Proximal = 해부학적 MCP) → MANO
  Index MCP (5) ✓
- `("Index", "IP"): 6` ← publisher `"IP"` (= SDK Intermediate = 해부학적 PIP) → MANO
  Index PIP (6) ✓
- 등등 모든 항목 검증.

**결론**: 25 → 21 remap 자체는 올바름. 비-thumb Metacarpal (= 해부학적 CMC) 는
MANO 에 없으므로 drop 하는 것도 정상. 25 - 4 = 21 ✓.

### 3.3 Suspect 1 — Manus 가 MANO frame 회전을 건너뜀 (검증: CONFIRMED ★)

**가설**: [`sensing/manus/manus_sensing.py:77-81`](../../../sensing/manus/manus_sensing.py#L77-L81)
이 wrist 만 원점화하고 MANO frame 회전을 건너뛰는데, Manus SDK 출력이 MANO
frame 이 아닐 가능성.

**검증 1 — 코드 비교**:

Manus 의 현재 코드:
```python
positions = data.skeleton[:, :3].astype(np.float32)
keypoints_3d = positions - positions[WRIST_IDX]   # ← 회전 없음
```

Phone 의 동일 단계 [`sensing/phone/keypoint_converter.py:60-66`](../../../sensing/phone/keypoint_converter.py#L60-L66):
```python
wl = detection.world_landmarks.astype(np.float32)
kp = wl - wl[self.WRIST_INDEX]
if self._apply_mano:
    kp = apply_mano_transform(kp, hand_type=self._hand_type)   # ← MANO 회전
```

Phone 은 wrist shift + MANO 회전 두 단계, Manus 는 wrist shift 만. ManusSensing 의
docstring 가정 ("Manus 글러브 출력은 이미 MANO frame이라 회전 불필요") 은 검증되지
않은 상태.

**검증 2 — Manus SDK 좌표계 헤더 + publisher 설정**:

[`sdk/ManusSDK/include/ManusSDKTypes.h:980-981`](../../../sensing/manus/sdk/ManusSDK/include/ManusSDKTypes.h#L980-L981):
```c
/// The transform is defined as a local or global transform depending on the
/// coordinate system set when initializing the SDK.
```

Publisher 의 초기화 [`ManusDataPublisher.hpp:128-129`](../../../../teleop_dev/sender/hand/sdk/ROS2/manus_ros2/src/ManusDataPublisher.hpp#L128-L129):
```cpp
bool m_WorldSpace = true;
CoordinateSystemVUH m_CoordinateSystem = {
    AxisView::AxisView_XFromViewer,   // X axis points away from viewer
    AxisPolarity::AxisPolarity_PositiveZ, // Z is up
    Side::Side_Right,                  // right-handed
    1.0f                               // unit = 1 meter
};
```

→ **Manus SDK 의 raw skeleton 은 world coordinates 로 publish 됨.** Right-handed,
Z-up, X-into-scene VR 표준 frame. **글러브를 어떻게 들고 있느냐에 따라 (palm down,
palm up, fingers up …) skeleton 의 회전이 달라짐.**

wrist shift 만 적용하면:
- 원점은 wrist 로 옮겨지지만 **회전은 여전히 world frame** 에 정렬됨
- dex-retargeting 은 keypoints 가 MANO frame (x = palm 안 wrist→middle MCP, y =
  palm normal, z = x×y) 에 있다고 가정
- 글러브를 palm down 으로 들고 fist 를 쥐면 finger curl 은 world -Z (아래) 방향이지만
  optimizer 는 palm normal (-Y in MANO) 방향으로 해석 → **두 축이 다르므로 손가락
  굽힘이 spread / twist 등 엉뚱한 방향으로 매핑됨.**

**검증 3 — Mock provider 가 버그를 가린 이유**:

[`mock_provider.py:22-28`](../../../sensing/manus/mock_provider.py#L22-L28):
```python
_FINGER_DIRECTIONS = np.array([
    [-0.6, 0.0, 0.8],   # Thumb — angled outward
    [-0.15, 0.0, 1.0],  # Index
    [0.0, 0.0, 1.0],    # Middle
    [0.15, 0.0, 1.0],   # Ring
    [0.3, 0.0, 1.0],    # Pinky
], dtype=np.float32)
```

Mock 은 fingers 가 +z 방향으로 뻗도록 합성 → 마침 MANO frame (palm normal +y, x =
across palm, z = along finger) 와 우연히 일치. 따라서 mock 만 돌리면 MANO 회전이
없어도 그럴듯한 결과가 나옴 → 실 글러브 사용 전까지 버그가 발견되지 않음.

**결론**: Suspect 1 = **CONFIRMED**. 수정 방법은 §4.1 참조.

### 3.4 Suspect 2 — DG-5F config indices off-by-one (검증: CONFIRMED ★)

**가설**: [`config/dg5f_right_vector.yml:32-34`](../config/dg5f_right_vector.yml#L32-L34)
의 `target_link_human_indices` 가 MANO 인덱싱과 어긋남.

**현재 config (라인 32-34)**:
```yaml
target_link_human_indices:
  - [0, 0, 0, 0, 0, 2, 6, 10, 14, 18]   # origins: wrist x5, then MCP x5
  - [4, 8, 12, 16, 20, 3, 7, 11, 15, 19] # tasks: tip x5, then PIP x5
```

**검증 1 — MANO 21 convention**: [`sensing/common.py:18-35`](../../../sensing/common.py#L18-L35)
:
- Wrist = 0
- Thumb: CMC=1, MCP=2, IP=3, TIP=4
- Index: MCP=**5**, PIP=**6**, DIP=**7**, TIP=8
- Middle: MCP=**9**, PIP=**10**, DIP=**11**, TIP=12
- Ring: MCP=**13**, PIP=**14**, DIP=**15**, TIP=16
- Pinky: MCP=**17**, PIP=**18**, DIP=**19**, TIP=20

→ 두 번째 batch (5 vector, indices 5-9 of the array):
- Origins `[2, 6, 10, 14, 18]` 의 비-thumb 항목 `[6, 10, 14, 18]` 은 **PIP** indices
  (MCP 가 아님). thumb idx 2 만 MCP. comment 의 "MCP x5" 와 어긋남.
- Tasks `[3, 7, 11, 15, 19]` 의 비-thumb 항목 `[7, 11, 15, 19]` 는 **DIP** indices
  (PIP 가 아님). thumb idx 3 = thumb IP. comment 의 "PIP x5" 와 어긋남.

**검증 2 — DG-5F URDF link 위치 trace**:

Index finger (finger 2) 의 joint chain ([dg5f_right_retarget.urdf:233-319](../config/dg5f_right_retarget.urdf#L233-L319)):

| joint | 부모 → 자식 | origin (parent frame) | 의미 |
|---|---|---|---|
| `rj_dg_2_1` | palm → `rl_dg_2_1` | (-0.0071, 0.027, 0.0661) | 손가락 base abduction |
| `rj_dg_2_2` | `rl_dg_2_1` → `rl_dg_2_2` | (0.01765, 0, 0.0265) | MCP flexion (small offset) |
| `rj_dg_2_3` | `rl_dg_2_2` → `rl_dg_2_3` | (0, 0, 0.0388) | PIP flexion (38.8mm = proximal phalanx) |
| `rj_dg_2_4` | `rl_dg_2_3` → `rl_dg_2_4` | (0, 0, 0.0388) | DIP flexion (38.8mm = middle phalanx) |
| `rj_dg_2_tip` | `rl_dg_2_4` → `rl_dg_2_tip` | (fixed) | TIP |

→ Pinocchio 에서 link 의 frame 은 그 link 를 만든 joint 의 위치에 놓임:
- `rl_dg_2_2` ≈ **MCP joint 위치** (post-abduction, pre-flexion)
- `rl_dg_2_3` ≈ **PIP joint 위치**
- `rl_dg_2_4` ≈ **DIP joint 위치**
- `rl_dg_2_tip` ≈ **TIP**

따라서 YAML link comment (`rl_dg_X_2 = "MCP"`, `rl_dg_X_4 = "DIP"`) 는 **올바름**.
DG-5F vector `rl_dg_X_2 → rl_dg_X_4` = MCP → DIP (proximal + middle phalanx).

**검증 3 — 매칭 확인**:

DG-5F link 정의는 MCP → DIP, MANO indices 는 PIP → DIP. **불일치**:

| 손가락 | DG-5F link vector | 현재 MANO 인덱스 | 올바른 MANO 인덱스 |
|---|---|---|---|
| Thumb | `rl_dg_1_2 → rl_dg_1_4` (CMC twist 포함) | 2 → 3 (MCP → IP) | 2 → 3 (보존) |
| Index | `rl_dg_2_2 → rl_dg_2_4` (MCP → DIP) | **6 → 7 (PIP → DIP)** | **5 → 7 (MCP → DIP)** |
| Middle | `rl_dg_3_2 → rl_dg_3_4` (MCP → DIP) | **10 → 11 (PIP → DIP)** | **9 → 11 (MCP → DIP)** |
| Ring | `rl_dg_4_2 → rl_dg_4_4` (MCP → DIP) | **14 → 15 (PIP → DIP)** | **13 → 15 (MCP → DIP)** |
| Pinky | `rl_dg_5_2 → rl_dg_5_4` (MCP → DIP) | **18 → 19 (PIP → DIP)** | **17 → 19 (MCP → DIP)** |

**Origins 4개가 off-by-one**. Tasks 는 모두 정확.

**검증 4 — 왜 phone 에서는 큰 문제가 안 보였는가**:

- Phone 도 같은 config 를 사용하므로 동일한 off-by-one 영향을 받음.
- Vector mode + scaling_factor=1.2 가 magnitude 차이 (PIP→DIP ≈ 2cm vs MCP→DIP ≈
  4cm) 를 부분적으로 흡수.
- direction 은 손가락이 펴진 상태에서 두 vector 가 거의 같은 방향이므로 큰 차이
  없음. 굽혀질수록 차이가 커짐.
- 결과: phone 에서는 미세한 부정확함이 있지만 사용자가 dramatic 한 inversion 을
  보고할 정도는 아님.
- Manus 에서는 Suspect 1 (frame 누락) 이 직접적인 inversion 을 일으키고, 그 위에
  Suspect 2 가 추가로 노이즈를 더함 → "fist → spread" 라는 dramatic 증상.

**검증 5 — 원래 config 작성 시점의 의도**:

같은 YAML 의 commented-out 구버전 ([dg5f_right_vector.yml:36-38](../config/dg5f_right_vector.yml#L36-L38)):
```yaml
# target_link_human_indices:
#   - [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
#   - [4, 8, 12, 16, 20, 2, 6, 10, 14, 18]
```

`[2, 6, 10, 14, 18]` 을 task 로 사용 — 작성자는 이를 "thumb MCP + 비-thumb MCPs"
로 의도했을 것 (palm → 손가락 첫 마디). 하지만 6/10/14/18 은 실제로 PIPs.
**작성 시점부터 동일한 MANO 인덱싱 혼동이 존재**. 의도가 잘못 인코딩된 버그.

**결론**: Suspect 2 = **CONFIRMED**. 수정 방법은 §4.2 참조.

## 4. 수정 사항

### 4.1 Manus → MANO frame 회전 추가

[`sensing/manus/manus_sensing.py`](../../../sensing/manus/manus_sensing.py) 의
`get_keypoints()` 를 phone/realsense 와 동일한 패턴으로 수정:

```python
from retarget_dev.sensing.core.mano_transform import apply_mano_transform

...

# Extract xyz positions from skeleton (21, 7) → (21, 3)
positions = data.skeleton[:, :3].astype(np.float32)

# Shift to wrist origin
keypoints_3d = positions - positions[WRIST_IDX]

# Manus SDK publishes in WORLD coordinates (VUH = X-into-scene, Z-up,
# right-handed; see ManusDataPublisher.hpp:128). After wrist shift the
# rotation is still world-aligned, NOT MANO. Apply the same SVD-based
# palm-plane fit used by phone/realsense to land in MANO frame.
keypoints_3d = apply_mano_transform(keypoints_3d, hand_type=self._hand_side)
```

`apply_mano_transform` 는 `(21, 3)` ndarray + `hand_type` 인수만 받으므로 input
이 wrist origin 인 것만 가정. Manus 는 이미 wrist shift 가 끝난 상태이므로 그대로
호출 가능. 같은 함수가 phone / realsense 에서 검증됨.

### 4.2 DG-5F config indices off-by-one fix

[`config/dg5f_right_vector.yml`](../config/dg5f_right_vector.yml) 의 라인 32-34
수정:

```yaml
# Before
target_link_human_indices:
  - [0, 0, 0, 0, 0, 2, 6, 10, 14, 18]   # origins: wrist x5, then MCP x5
  - [4, 8, 12, 16, 20, 3, 7, 11, 15, 19] # tasks: tip x5, then PIP x5

# After
target_link_human_indices:
  - [0, 0, 0, 0, 0, 2, 5,  9, 13, 17]   # origins: wrist x5, then MCP x5 (MANO MCP indices)
  - [4, 8, 12, 16, 20, 3, 7, 11, 15, 19] # tasks: tip x5, then DIP x5 (MANO DIP indices)
```

변경:
- 비-thumb origins `[6, 10, 14, 18]` (PIPs) → `[5, 9, 13, 17]` (MCPs)
- Thumb origins `[2]` (MCP) 와 모든 tasks 는 동일

설명:
- 두 번째 batch 의 5 vector 는 DG-5F link `rl_dg_X_2 → rl_dg_X_4` (= MCP → DIP)
  와 매칭되는 human MCP → DIP vector 이어야 함
- task 는 이미 DIP indices (`[7, 11, 15, 19]`) 로 정확
- origin 은 MCP indices (`[5, 9, 13, 17]`) 이어야 하지만 PIPs 로 잘못 적힘
- thumb 은 anatomical 구조가 다르므로 (CMC twist 포함) 기존 mapping (`2 → 3`,
  MCP → IP) 을 보존. 완벽하지는 않지만 phone 에서 검증된 동작.

### 4.3 영향 범위 / 회귀 가능성

| 수정 | 영향 | 검증 필요 |
|---|---|---|
| `manus_sensing.py` | Manus 경로만 (mock + ros2 + sdk providers 모두) | ✓ Mock 으로 dry-run, 실 글러브로 fist/open hand |
| `dg5f_right_vector.yml` | Phone + Manus 모두 (vector mode 만, dexpilot 영향 없음) | ✓ Phone 회귀 테스트 (이전과 비슷한 정도여야) |
| URDF | 미수정 | — |

DexPilot config (`dg5f_right_dexpilot.yml`) 는 `target_link_human_indices` 를 직접
지정하지 않고 `wrist_link_name` + `finger_tip_link_names` 만 사용 → 라이브러리가
auto-generate. 따라서 Suspect 2 와 무관.

## 4.4 회귀 테스트 결과 (manus-egocentric-sample episode 0)

본 dataset 의 episode 0 (1158 frame, fold_laundry task) 을 사용해 5 가지 fix
조합을 비교한 결정적 회귀 테스트.

스크립트: [`src/manus-egocentric-sample/scripts/regression_test_fixes.py`](../../../../manus-egocentric-sample/scripts/regression_test_fixes.py)

지표: 비-thumb 4개 손가락의 `rj_dg_X_3 + rj_dg_X_4` (PIP + DIP) qpos 의
fist 자세 평균 - open 자세 평균. 양수 = fist 가 DG-5F 손가락을 굽힘 (정상).

### 4.4.1 첫 번째 실행 (잘못된 OPERATOR2MANO_RIGHT 매트릭스 사용 시)

| 조건 | MANO | indices | aggregate flex delta (rad) |
|---|---|---|---|
| A | OFF | OLD (`[..2,6,10,14,18]`) | +0.193 |
| B | ON, hand_type=R | OLD | -0.114 (BAD) |
| C | OFF | NEW (`[..2,5,9,13,17]`) | +0.427 |
| D | ON, hand_type=R | NEW (current state) | -0.065 (BAD) |
| E | ON, hand_type=L | NEW | +1.802 |

→ 당시 잘못된 결론: "dataset 의 좌표가 MediaPipe 와 미러됨". 실제로는
[`mano_transform.py`](../../../sensing/core/mano_transform.py) 의
`OPERATOR2MANO_RIGHT` 행렬 row 1 부호 오류로 발생한 위장 증상이었음.

### 4.4.2 두 번째 실행 (`OPERATOR2MANO_RIGHT` 수정 후)

[`mano_transform.py`](../../../sensing/core/mano_transform.py) 의 row 1 을
`[-1, 0, 0]` → `[1, 0, 0]` 로 수정하고 동일 dataset/episode 로 재실행:

| 조건 | MANO | indices | aggregate flex delta (rad) |
|---|---|---|---|
| A | OFF | OLD | +0.193 |
| **B** | **ON, hand_type=R** | **OLD** | **+2.332** ★ |
| C | OFF | NEW | +0.427 |
| **D** | **ON, hand_type=R** | **NEW** (current state) | **+2.251** ★ |
| E | ON, hand_type=L | NEW | +1.802 |

**올바른 결론**:

1. **★ Suspect 1 (MANO transform 적용) 은 본 파이프라인의 핵심 fix**. A→B 에서
   +0.193 → +2.332 로 약 12배 향상 (delta +2.139). MANO frame 회전 없이는
   optimizer 가 손가락 굽힘을 제대로 풀지 못함.
2. **★ `OPERATOR2MANO_RIGHT` 행렬에 sign error 가 있었음**. 위 결과는 사용자가
   [`human_hand_video.mp4`](../dex-retargeting/example/vector_retargeting/data/human_hand_video.mp4)
   재생으로 직접 검증한 수정 후 매트릭스 (`row 1 = [1, 0, 0]`) 기준.
3. **첫 번째 실행에서 "dataset 미러" 결론은 잘못이었음**. 실제로는 dataset 이
   미러된 게 아니라 행렬이 잘못되어 있어 `hand_type='left'` 호출이 우연히 보정 효과를
   냈을 뿐. 매트릭스 fix 후에는 `hand_type='right'` 가 정상 동작 (B/D > E).
4. **Suspect 2 (config indices off-by-one) 의 영향은 작음**. B (+2.332) 가 D (+2.251)
   보다 약간 높음 → NEW indices 가 이 dataset 에서는 약간 손해. 다만 차이가 작고
   per-finger 로 보면 NEW indices 가 균등하게 좋아짐 (index 손가락이 D 에서 +1.63
   vs B 에서 +1.57). 단일 dataset 한정의 결과이며 결정적 결론에는 더 많은 검증 필요.
5. **이 dataset 의 25-node 배열 순서는 SDK enum 순서가 아님**: 비-thumb 손가락에서
   Intermediate (= 해부학적 PIP) 가 Proximal (= 해부학적 MCP) **앞에** 옴. mapping 은
   `[start, start+2, start+1, start+3, start+4]`. 이 quirk 는 dataset 한정이며 live
   ROS2 publisher 의 string-label 매핑은 영향받지 않음. 자세한 내용:
   [`manus-egocentric-sample/docs/usage.md §5.1`](../../../../manus-egocentric-sample/docs/usage.md#51-25--21-노드-변환-검증된-매핑).

### 4.4.3 MANO 좌표계의 의미 (수정 후 RIGHT 매트릭스 기준)

`apply_mano_transform(hand_type='right')` 의 출력 (21, 3) 키포인트는 다음 축을
가짐:

| 축 | 방향 | 설명 |
|---|---|---|
| **+x** | palm-out | 손바닥의 normal 방향, 손바닥이 향하는 쪽 |
| **+y** | thumb-side | pinky → index 방향 (오른손 기준 엄지 쪽) |
| **+z** | forward | wrist → fingertips 방향 (손가락이 펴진 방향) |

검증: 사용자가 dex-retargeting upstream 의 `human_hand_video.mp4` 를 재생했을 때
DG-5F 가 영상의 손동작과 동일한 방향으로 움직임을 확인.

LEFT 매트릭스는 아직 hardware 검증 미완료이며, 위 RIGHT 매트릭스에 다음 변환을
적용해 LEFT/RIGHT 의 bilateral symmetry 를 유지: `LEFT = diag(1,-1,-1) @ RIGHT`.

```
OPERATOR2MANO_LEFT = [[ 0,  0, -1],
                     [-1,  0,  0],
                     [ 0, -1,  0]]
```

### 4.4.4 향후 검증 항목

- [ ] **실 Manus 글러브 검증**: dataset 회귀는 다른 origin 의 데이터라 live 글러브
  와 100% 동일하지 않음. 실 글러브로 fist/open 시퀀스를 녹화해 동일한 결과가
  나오는지 확인.
- [ ] **LEFT 매트릭스 hardware 검증**: 현재 LEFT 는 RIGHT 와의 대칭성으로부터 유추한
  값. 좌측 손 dataset 또는 좌측 hand DG-5F URDF 가 생기면 검증 필요.
- [ ] **Suspect 2 (NEW indices) 재평가**: dataset 단독으로는 NEW vs OLD 차이가 작음.
  실 글러브 또는 다른 episode 에서 NEW indices 의 우월성을 더 검증할 필요.

### 4.4.5 후속: Phone 회귀 발견 & per-source 매트릭스 분리

**증상**: 4.4.2 의 매트릭스 fix (`OPERATOR2MANO_RIGHT` row 1 = `[1, 0, 0]`) 이후
사용자가 phone → DG-5F 경로가 깨졌다고 보고. Manus 가 정상화되는 대신 phone 이
망가짐.

**원인**: 단일 매트릭스를 모든 sensing source 가 공유하고 있었기 때문. Phone 의
MediaPipe HandLandmarker 출력은 dex-retargeting upstream 의 단일 매트릭스
(`row 1 = [-1, 0, 0]`) 에 calibrate 되어 있고, Manus 글러브 출력은 다른 chirality
를 가져 row 1 = `[1, 0, 0]` 이 필요. 두 source 에 같은 매트릭스를 강제하면 한 쪽이
반드시 깨짐.

**해결**: [`mano_transform.py`](../../../sensing/core/mano_transform.py) 에서 매트릭스
쌍을 두 개로 분리하고 `apply_mano_transform` 에 `convention` 파라미터 추가.

| convention | 사용 source | RIGHT row 1 | 검증 출처 |
|---|---|---|---|
| `"mediapipe"` (default) | phone, RealSense | `[-1, 0, 0]` | dex-retargeting upstream + phone path 동작 |
| `"manus"` | live ROS2, SDK, offline egocentric | `[1, 0, 0]` | manus-egocentric-sample regression test (4.4.2) + 사용자의 human_hand_video.mp4 재생 검증 |

두 매트릭스의 관계: `MANUS = diag(1, -1, 1) @ MEDIAPIPE` (row 1 sign flip).
출력 좌표계 비교: 같은 input 에 대해 `mediapipe.x = -manus.x`, y/z 동일 (= MANO +x
축이 반대 방향을 가리킴).

**호출 site 5곳** 모두 `convention=` 명시:
- phone/realsense → `convention="mediapipe"` (default 와 같지만 의도를 명시)
- manus_sensing / offline_egocentric_provider / debug_frame_check / regression_test_fixes
  → `convention="manus"`

**회귀 결과** (4.4.2 와 동일 조건 재실행):
- A (no MANO + OLD indices): `+0.193`
- B (MANO=R + OLD indices): `+2.332` ★
- D (MANO=R + NEW indices, current state): `+2.251` ★

→ Manus 경로 동작은 변화 없이 보존, phone 경로는 upstream 매트릭스로 복원.

## 5. 재현 / 회귀 검증

### 5.1 실 글러브 사용

```bash
# Manus ROS2 (manus_data_publisher 가 별도 터미널에서 동작 중이어야 함)
python3 -m retarget_dev.models.dex_retarget.main \
    --sensing manus-ros2 \
    --config retarget_dev/models/dex_retarget/config/dg5f_right_vector.yml
```

확인:
- Open hand → DG-5F 손가락이 펴진 자세
- Fist → DG-5F 손가락이 접힌 자세 (spread 가 아니라)
- Pinch (엄지 + 검지) → DG-5F 도 pinch
- 각 손가락 개별 굽힘 → 해당 DG-5F 손가락만 굽힘

### 5.2 디버그 스크립트로 keypoint 검증

[`sensing/manus/tests/debug_frame_check.py`](../../../sensing/manus/tests/debug_frame_check.py)
실행 → fist / open hand 두 자세에 대해 raw / wrist-shifted / MANO-transformed
buffer 를 출력. 기대 동작:
- Open hand → 모든 fingertip y > 0 (palm normal 양의 방향)
- Fist → 모든 fingertip y 가 open 시점보다 작음 (palm normal 방향으로 curl)

### 5.3 Phone 회귀

같은 config 를 phone 으로도 검증 (Suspect 2 fix 의 회귀 확인):

```bash
python3 -m retarget_dev.models.dex_retarget.main \
    --sensing phone --hand right \
    --camera-url http://<phone-ip>:8080/video \
    --config retarget_dev/models/dex_retarget/config/dg5f_right_vector.yml
```

**예상**: phone 의 동작이 fix 전과 비슷하거나 약간 더 정확해짐. Dramatic 한 회귀
는 없어야 함 (이전에 부정확함이 미세했으므로).

### 5.4 (선택) 오프라인 replay

`OpenGraphLabs-Research/manus-egocentric-sample` (HuggingFace) 데이터를
[`sensing/manus/offline_provider.py`](../../../sensing/manus/offline_provider.py)
로 로드하고 동일한 frame sequence 에 대해 fix 전/후 출력 qpos 비교. 결정적
재현 가능. 자세한 사용법은 해당 모듈의 docstring 참조.

## 6. 회귀 방지 체크리스트

향후 새 sensing source (예: Quest hand tracking, Leap Motion, OptiTrack) 추가 시
다음을 반드시 검증:

- [ ] **Frame 명시**: SDK / 입력의 좌표계가 무엇인지 docstring 에 명시
  - "world frame", "camera frame", "wrist frame", "MANO frame" 중 어느 것?
  - 어떤 축이 up / forward / right 인가?
- [ ] **MANO 변환 적용**: dex-retargeting 입력은 반드시 MANO frame 이어야 함.
  필요 시 [`sensing/core/mano_transform.py`](../../../sensing/core/mano_transform.py)
  의 `apply_mano_transform` 호출
- [ ] **Mock 만으로 검증 금지**: mock provider 는 임의의 좌표계로 합성 가능 →
  실제 SDK 와 일치하지 않을 수 있음. 새 sensor 는 반드시 **실 데이터** 로 검증
- [ ] **Open hand vs fist 출력 비교**: 두 자세에서 fingertip 좌표가 일관된 방향
  (palm normal) 으로 변하는지 확인
- [ ] **Phone 과 같은 config 로 동작 비교**: 같은 DG-5F config 를 사용한 출력이
  phone 과 비교했을 때 합리적으로 비슷한지 확인

## 7. 참고 자료

### Manus 공식 문서
- Skeletons & Hand Motions: https://docs.manus-meta.com/3.1.0/Software/Skeletons/
- Coordinate System (VUH): SDK 헤더 [`ManusSDKTypes.h:1707-1714`](../../../sensing/manus/sdk/ManusSDK/include/ManusSDKTypes.h#L1707-L1714)

### 관련 코드
| 역할 | path | 주요 line |
|---|---|---|
| Manus 입력 처리 | [`sensing/manus/manus_sensing.py`](../../../sensing/manus/manus_sensing.py) | get_keypoints() |
| Phone (참조 모범) | [`sensing/phone/keypoint_converter.py`](../../../sensing/phone/keypoint_converter.py) | 60-66 |
| MANO 변환 | [`sensing/core/mano_transform.py`](../../../sensing/core/mano_transform.py) | apply_mano_transform |
| 25 → 21 remap | [`sensing/manus/ros2_provider.py`](../../../sensing/manus/ros2_provider.py) | 67-93 |
| DG-5F vector config | [`config/dg5f_right_vector.yml`](../config/dg5f_right_vector.yml) | 32-34 |
| DG-5F URDF | [`config/dg5f_right_retarget.urdf`](../config/dg5f_right_retarget.urdf) | rj_dg_2_1..4 |
| Publisher 좌표계 | [`ManusDataPublisher.hpp`](../../../../teleop_dev/sender/hand/sdk/ROS2/manus_ros2/src/ManusDataPublisher.hpp) | 128-129 |
| dex_retarget 래퍼 | [`dex_retarget_model.py`](../dex_retarget_model.py) | _build_ref_value |
| 학습용 walkthrough | [`pipeline_walkthrough.md`](../../../docs/pipeline_walkthrough.md) | §6 MANO transform |

### 관련 문서
- [`manus_realtime.md`](manus_realtime.md) — Manus 사용 가이드 + 25→21 issue
- [`dg5f_tuning.md`](dg5f_tuning.md) — 이전 URDF joint limit 튜닝 기록
- [`pipeline_walkthrough.md`](../../../docs/pipeline_walkthrough.md) — 학습용
  end-to-end 변환 안내
