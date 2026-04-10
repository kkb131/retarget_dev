# Manus Glove 실시간 Retargeting 가이드

Manus Quantum / Prime 글러브를 사용하여 DG-5F (Isaac Sim)를 실시간 제어.

```
Manus Glove
    ↓ (USB dongle / Integrated)
ManusSDK Core
    ↓ Raw skeleton stream (~25 nodes/hand)
[3가지 백엔드 중 선택]
  ┌─ ros2_provider  ←  /manus_glove_{0..3} ROS2 topic    (manus_data_publisher)
  ├─ sdk_provider   ←  SDKClient_Linux --stream-json     (subprocess JSON)
  └─ mock_provider  ←  sine-wave 가짜 데이터              (하드웨어 없이 테스트)
    ↓
ManusSensing
    ↓ HandKeypoints (21, 3) — MANO frame
DexRetargetModel (VectorOptimizer / DexPilot)
    ↓ 20 DOF
ROS2 JointState → Isaac Sim DG-5F
```

`--sensing` 옵션으로 백엔드를 선택합니다:

| 옵션 | 백엔드 | 용도 |
|---|---|---|
| `--sensing manus-mock` | `mock_provider` | 글러브 없이 파이프라인 검증 |
| `--sensing manus-ros2` | `ros2_provider` | **권장** — `manus_data_publisher` 와 함께 사용 |
| `--sensing manus-sdk` | `sdk_provider` | SDKClient_Linux subprocess (제약 있음, §6 참조) |

## 1. Manus 글러브 하드웨어 특성

| 항목 | 값 |
|---|---|
| 갱신율 | 120 Hz (raw skeleton) |
| 노드 수 (raw) | **25 / 한 손** (1 wrist + thumb 4 + 4 finger × 5) |
| 출력 | per-joint quaternion + translation |
| Ergonomics | 20 finger angles (°) — **dex-retargeting에서는 사용 안 함** |
| 인터페이스 | USB 2.4 GHz dongle (`Glongle`) 또는 Integrated |
| Occlusion 내성 | ★★★★★ (센서 직접) |

**Phone / RealSense / Manus 비교:**

| | Phone (MediaPipe) | RealSense D405 | Manus Glove |
|---|---|---|---|
| 입력 형식 | RGB → MediaPipe 21 keypoints | Color + depth → 3D 21 keypoints | 25 raw skeleton nodes (직접 IMU 측정) |
| 갱신율 | 30 Hz | 30 Hz (depth bottleneck) | **120 Hz** |
| Z 정밀도 | MediaPipe 추정 (~5-15mm) | sub-mm | Per-joint IMU + 캘리브레이션 |
| Occlusion | 가려지면 fail | 가려지면 fail | **무관** (착용 센서) |
| 카메라 시점 의존성 | 있음 | 있음 | **없음** |
| 설치 난이도 | 쉬움 | 중간 (USB 3.0) | 어려움 (SDK + dongle + 캘리브레이션) |
| 라이선스 | 무료 | 무료 | **유료** (SDK 라이선스) |

## 2. 준비물

### 공통
- Manus 글러브 (Quantum / Prime / Pro) + Glongle 또는 Integrated dongle
- Manus Core 라이선스 (SDK / Integrated)
- Isaac Sim에서 DG-5F 실행 중 (`/dg5f_right/joint_commands` subscribe)

### `--sensing manus-ros2` 추가 요구사항
- `manus_ros2_msgs` + `manus_ros2` 패키지 빌드 + source
- `manus_data_publisher` 노드 실행 중

### `--sensing manus-sdk` 추가 요구사항
- 빌드된 `SDKClient_Linux.out` (`--stream-json` 모드 지원)
- ManusSDK 라이브러리 (`ManusSDK/lib/*.so`)
- ⚠️ **현재 raw skeleton 좌표를 정확히 매핑하지 못함** — §6 참조

## 3. 실행

### 3-1. Mock (글러브 없이 파이프라인 검증)

```bash
cd /workspaces/tamp_ws/src

python3 -m retarget_dev.models.dex_retarget.main \
  --sensing manus-mock \
  --config retarget_dev/models/dex_retarget/config/dg5f_right_dexpilot.yml \
  --hand right --hz 30
```

→ Isaac Sim DG-5F가 sine-wave로 손을 폈다 접었다 해야 함.
이게 동작하면 retargeting 파이프라인은 정상 — 실제 글러브 연결 문제만 남음.

### 3-2. ROS2 publisher 백엔드 (권장)

```bash
# Terminal 1: Manus 데이터 publisher
ros2 run manus_ros2 manus_data_publisher

# Terminal 2: 토픽 확인 (선택)
ros2 topic echo /manus_glove_0 --once
# raw_node_count: 25 가 보여야 정상

# Terminal 3: 실시간 retargeting
python3 -m retarget_dev.models.dex_retarget.main \
  --sensing manus-ros2 \
  --config retarget_dev/models/dex_retarget/config/dg5f_right_dexpilot.yml \
  --hand right \
  --topic /dg5f_right/joint_commands \
  --hz 30
```

기대 로그 (첫 메시지 수신 시):
```
[INFO] Manus raw skeleton: 25 raw nodes → MANO 21 (remap OK)
```

### 3-3. SDK subprocess 백엔드 (실험적)

```bash
python3 -m retarget_dev.models.dex_retarget.main \
  --sensing manus-sdk \
  --sdk-bin /workspaces/tamp_ws/src/retarget_dev/sensing/manus/sdk/SDKClient_Linux.out \
  --config retarget_dev/models/dex_retarget/config/dg5f_right_dexpilot.yml \
  --hand right
```

⚠️ §6 참조 — JSON 출력에 노드 메타데이터가 없어 25→21 매핑이 미해결 상태.

## 4. 파이프라인 구조

### 4-1. 센싱 계층 (`retarget_dev/sensing/manus/`)

```
[Provider]                      [Output: HandData]
  ros2_provider.py              {
    Ros2ManusProvider             joint_angles: (20,),    ← Ergonomics (사용 안 함)
      └─ _glove_callback          finger_spread: (5,),
         ├─ _ERGO_TYPE_MAP        wrist_pos: (3,),
         └─ _remap_to_mano_21 ★   wrist_quat: (4,),
                                  skeleton: (21, 7),       ← MANO 21
                                  has_skeleton: True
                                }
  sdk_provider.py                       │
    SdkManusProvider                    │
      └─ JSON parser                    ▼
                                  ManusSensing
  mock_provider.py                  └─ get_keypoints()
    MockManusProvider                  → positions = skeleton[:, :3]
      └─ _build_skeleton (21, 7)       → wrist 원점화
                                       ▼
                                  HandKeypoints (21, 3) — MANO frame
                                       ▼
                                  DexRetargetModel
                                       ▼
                                  q (20 DOF) → ROS2 → Isaac Sim
```

**핵심 불변식**: `HandData.skeleton.shape == (21, 7)` 이어야 downstream `ManusSensing.get_keypoints()` 가 dex-retargeting과 호환되는 (21, 3) 키포인트를 만듭니다. 이를 보장하는 책임이 각 provider에 있습니다.

### 4-2. Phone / RealSense와의 차이

| 단계 | Phone | RealSense | Manus |
|---|---|---|---|
| 2D 검출 | MediaPipe | MediaPipe | **불필요** (직접 3D) |
| 3D 복원 | MediaPipe `world_landmarks` | `rs2_deproject_pixel_to_point` + depth | **글러브 IMU 직접** |
| 좌표계 변환 | MANO transform 필수 | MANO transform 필수 | **MANO transform 필수** ([수정 이력](manus_debug.md#33-suspect-1--manus-가-mano-frame-회전을-건너뜀-검증-confirmed-)) |
| 노드 개수 정렬 | 21 (MediaPipe 기본) | 21 (MediaPipe 기본) | **25 → 21 리매핑 필수** ⚠️ |

→ Manus는 카메라 기반 대비 2D detection / depth 추정 단계가 빠지지만, **MANO 좌표계 변환은 phone/realsense 와 마찬가지로 필수입니다**. SDK 가 raw skeleton 을 world frame (`AxisView_XFromViewer`, `AxisPolarity_PositiveZ`, `Side_Right`) 로 publish 하므로 글러브 자세에 따라 회전이 달라지며, 회전 보정 없이 dex-retargeting 에 넘기면 손가락 굽힘 방향이 거꾸로 매핑됩니다 (이전 fist→spread 버그). 자세한 분석은 [`manus_debug.md`](manus_debug.md) 참조. 이 외에도 **25→21 노드 리매핑 이라는 고유 문제가 있습니다** (§5).

## 5. ⚠️ 핵심 이슈: 25 → 21 MANO 리매핑

### 5-1. 문제 배경

Manus SDK의 raw skeleton stream은 손당 **25개 노드**를 publish합니다 ([`ManusSDKTypes.h:776-781`](../../../sensing/manus/sdk/ManusSDK/include/ManusSDKTypes.h#L776-L781) `FingerJointType` enum):

| 손가락 | 노드 구성 | 개수 |
|---|---|---|
| Wrist (Hand root) | 1 | 1 |
| Thumb | Metacarpal, Proximal, Distal, Tip (Intermediate 없음) | 4 |
| Index/Middle/Ring/Pinky | Metacarpal, Proximal, **Intermediate**, Distal, Tip | 5 × 4 = 20 |
| **합계** | | **25** |

반면 dex-retargeting (및 MediaPipe / MANO 표준) 은 손당 **21개 노드**를 가정합니다:

| | MANO 21 layout |
|---|---|
| Wrist | idx 0 |
| Thumb | idx 1 (CMC), 2 (MCP), 3 (IP), 4 (TIP) |
| Index | idx 5 (MCP), 6 (PIP), 7 (DIP), 8 (TIP) |
| Middle | idx 9, 10, 11, 12 |
| Ring | idx 13, 14, 15, 16 |
| Pinky | idx 17, 18, 19, 20 |

차이: MANO는 비-thumb 4 손가락의 **Metacarpal 노드(손바닥 안 metacarpal 뼈 base)** 를 가지지 않습니다. 25 - 4 = 21 ✓

### 5-2. 그냥 두면 어떻게 깨지나

[`manus_data_publisher.cpp:304-336`](../../../../teleop_dev/sender/hand/sdk/ROS2/manus_ros2/src/ManusDataPublisher.cpp#L304-L336) C++ publisher는 SDK가 주는 25 노드를 **필터링/리매핑 없이 그대로** publish합니다.

리매핑 없이 25 노드를 그대로 사용하면:
- `keypoints_3d.shape == (25, 3)` 이 dex-retargeting으로 들어감
- dex-retargeting은 `target_link_human_indices = [4, 8, 12, 16, 20]` (MANO 손가락 끝 인덱스) 로 fingertip을 가져오는데, **(25, 3) 배열의 인덱스 4는 thumb tip이 아니라 SDK가 임의 순서로 publish한 4번째 노드** (예: thumb Distal)
- 결과: dex-retargeting이 엉뚱한 노드를 fingertip으로 매칭 → DG-5F가 이상한 자세에서 굳음
- 이 증상은 phone DexPilot 버그(이전에 수정된 ref_value ordering 버그)와 매우 유사하지만 **원인은 다름**

### 5-3. 해결 방법: `_remap_to_mano_21()`

[`ros2_provider.py`](../../../sensing/manus/ros2_provider.py) 에 추가된 헬퍼가 `ManusRawNode` 의 `chain_type` + `joint_type` 메타데이터를 사용해 25 노드를 21 MANO 슬롯에 결정론적으로 배치합니다 (publish 순서 무관).

```python
def _remap_to_mano_21(raw_nodes) -> Optional[np.ndarray]:
    skel = np.full((21, 7), np.nan, dtype=np.float32)
    for node in raw_nodes:
        if node.chain_type == "Hand":
            idx = 0  # wrist root
        else:
            idx = _MANO_REMAP.get((node.chain_type, node.joint_type))
            if idx is None:
                continue  # SDK Metacarpal of non-thumb finger → drop
        p, q = node.pose.position, node.pose.orientation
        skel[idx] = [p.x, p.y, p.z, q.w, q.x, q.y, q.z]
    if np.isnan(skel).any():
        return None  # incomplete frame
    return skel
```

`_MANO_REMAP` 는 `(chain_type, joint_type)` → MANO index 매핑 표 (총 20 항목, wrist는 별도 처리).

### 5-4. ⚠️ 함정: publisher의 string 라벨이 해부학과 어긋남

[`ManusDataPublisher.cpp:669-686`](../../../../teleop_dev/sender/hand/sdk/ROS2/manus_ros2/src/ManusDataPublisher.cpp#L669-L686) `JointTypeToString` 가 SDK enum을 문자열로 변환할 때, **anatomical 명칭과 한 단계 어긋난** label을 emit합니다:

| SDK enum (`FingerJointType_*`) | publisher string | 해부학적 의미 |
|---|---|---|
| `Metacarpal` | `"MCP"` | 손바닥 안 metacarpal 뼈 base (실제로는 CMC) |
| `Proximal` | `"PIP"` | proximal phalanx base (실제로는 MCP, 너클) |
| `Intermediate` | `"IP"` | middle phalanx base (실제로는 PIP) |
| `Distal` | `"DIP"` | distal phalanx base (실제로는 DIP) |
| `Tip` | `"TIP"` | 손가락 끝 |

→ 처음 코드를 읽으면 매핑 표가 매우 헷갈립니다. 예를 들어 `("Index", "PIP"): 5` 는 "MANO Index PIP" (idx 6) 가 아니라 **MANO Index MCP (idx 5)** 에 매핑됩니다 — `"PIP"` 라는 publisher 문자열이 실제로는 SDK Proximal joint (= 해부학적 MCP) 를 의미하기 때문.

매핑 표를 수정할 일이 있다면 [`ros2_provider.py`](../../../sensing/manus/ros2_provider.py) 의 `_MANO_REMAP` docstring + `JointTypeToString` 의 C++ source를 반드시 함께 봐야 합니다.

### 5-5. 진단 로깅

`Ros2ManusProvider._diagnose_remap()` 가 1회성으로 다음을 출력합니다:

**성공 (정상)**:
```
[INFO] Manus raw skeleton: 25 raw nodes → MANO 21 (remap OK)
```

**실패 (디버깅 필요)**:
```
[WARNING] Manus raw skeleton remap incomplete — MANO slots unfilled.
          Received N nodes with (chain, joint) pairs: [('Hand', 'Invalid'), ...]
```

→ WARNING 이 뜨면 출력된 `(chain, joint)` 키 set 을 보고 `_MANO_REMAP` 에 누락된 것이 있는지, publisher가 다른 string을 emit하는지 확인.

### 5-6. Failure 모드 처리

`_remap_to_mano_21` 가 `None` 을 반환하는 경우 (글러브 일부 손가락 dropout 등):
1. `_glove_callback` 이 `has_skeleton=False` 로 `HandData` 저장
2. `ManusSensing.get_keypoints()` 가 None 반환
3. `dex_retarget.main` 의 retarget 호출 skip
4. 다음 정상 프레임까지 DG-5F 는 마지막 자세 유지 (warm start)

→ 에러 폭주 없이 graceful하게 회복.

## 6. SDK provider의 알려진 제약

[`sdk_provider.py`](../../../sensing/manus/sdk_provider.py) (`--sensing manus-sdk`) 도 동일한 25→21 문제를 가지지만, **Python 단에서 고칠 수 없습니다**.

이유: backend인 [`SDKClient.cpp:1525-1550`](../../../../teleop_dev/sender/hand/sdk/SDKClient_Linux/SDKClient.cpp#L1525-L1550) 의 `--stream-json` 출력은 각 노드의 좌표 7개 (`x, y, z, qw, qx, qy, qz`) 만 dump하고 **`chain_type` / `joint_type` 메타데이터를 JSON에 포함하지 않습니다**. 따라서 어떤 노드가 어느 손가락의 어느 joint인지 구분할 방법이 없음.

해결 옵션 (별도 작업):
- (a) `SDKClient.cpp` 를 수정해 JSON 출력에 메타데이터를 추가
- (b) 노드 publish 순서가 SDK 버전에 걸쳐 고정이라고 가정하고 hard-coded order 매핑 사용 (취약)
- (c) `--sensing manus-sdk` 자체를 deprecate 하고 `--sensing manus-ros2` 로 통일

→ 현재 권장: **`--sensing manus-ros2` 를 사용하세요**. SDK 백엔드는 mock 검증용으로만.

## 7. 트러블슈팅

### 7-1. `No data received after 10s. Is manus_data_publisher running?`

- T1에서 `ros2 run manus_ros2 manus_data_publisher` 가 실행 중인지
- `ros2 topic list | grep manus_glove` 로 토픽이 보이는지
- `ros2 topic echo /manus_glove_0 --once` 가 비어있지 않은지
- 같은 ROS_DOMAIN_ID 인지 확인

### 7-2. `[WARNING] Manus raw skeleton remap incomplete`

§5-5 참조. 출력된 `(chain, joint)` 키 set을 확인:

- 모든 손가락에 대해 5(또는 thumb는 4) 개씩 들어와야 함
- `("Pinky", "TIP")` 같은 키가 빠져 있으면 글러브 센서 dropout일 가능성
- 처음 보는 string이 있으면 (예: `"Carpal"` 등) `_MANO_REMAP` 에 추가 필요

### 7-3. Mock은 동작하는데 ROS2 모드가 안 됨

→ §5-5의 INFO/WARNING 로그를 우선 확인. WARNING 이 없는데 손이 이상하게 움직이면:

```bash
# raw 메시지 직접 확인
ros2 topic echo /manus_glove_0 --once | head -50
# raw_node_count, ergonomics_count 가 0이 아닌지
# raw_nodes[0].chain_type, joint_type 이 string으로 들어오는지 확인
```

빈 메시지가 publish되는 경우 publisher의 라이선스 / dongle 연결을 확인하세요 ([`ManusDataPublisher.cpp:259-285`](../../../../teleop_dev/sender/hand/sdk/ROS2/manus_ros2/src/ManusDataPublisher.cpp#L259-L285)).

### 7-4. 손이 카메라 기반 입력과 반대 방향으로 움직임

Manus 좌표계가 우리 가정 (y-up, z-forward 등) 과 다를 수 있습니다. 캘리브레이션 자세에서 한 손가락만 굽혀보고 DG-5F의 어느 손가락이 반응하는지 확인.
필요시 `ros2_provider._remap_to_mano_21` 에서 좌표축 부호 반전 추가 가능 (현재는 적용 없음 — Manus가 손 로컬 좌표계로 출력한다고 가정).

### 7-5. DG-5F가 손가락을 충분히 굽히지 않음

→ Phone과 동일하게 `dg5f_right_dexpilot.yml` 의 `scaling_factor` 조정 (1.2~1.4 권장). [`dg5f_tuning.md`](dg5f_tuning.md) 참조.

### 7-6. 검지/중지가 손등으로 뒤집힘

→ `config`의 `urdf_path` 가 `dg5f_right_retarget.urdf` (PIP/DIP joint 음수 제한) 인지 확인. [`dg5f_tuning.md`](dg5f_tuning.md) 의 "URDF joint limits 수정" 섹션.

## 8. 성능 참고치

- Manus → ROS2 publisher: **120 Hz** (네이티브 SDK 레이트)
- ROS2 callback + remap: ~0.5ms/frame (25노드 dict lookup)
- ManusSensing.get_keypoints(): ~0.1ms
- dex-retargeting DexPilot: ~5-10ms
- ROS2 publish + Isaac Sim 반영: ~5-10ms
- **Total round-trip**: ~30-40ms → 안정적 30Hz (--hz 30)

→ 이론적으로는 60-100 Hz 도 가능하지만, dex-retargeting 옵티마이저의 warm-start가 안정화되는 데 ~5 frames 필요하므로 30 Hz 가 sweet spot.

## 9. Phone / RealSense / Manus 조합 비교

동일한 `dg5f_right_dexpilot.yml` config를 세 백엔드에 swap 가능:

```bash
# Phone
--sensing phone --url http://<ip>:8080/video

# RealSense D405
--sensing realsense

# Manus (ROS2)
--sensing manus-ros2
```

| 기준 | Phone | RealSense | Manus |
|---|---|---|---|
| 응답성 | 보통 | 좋음 | **최상** (120Hz native) |
| 장기 사용 (~30분) | 카메라 시점 drift | 안정 | **최상** (시점 무관) |
| 손가락 교차 / 자기 가림 | 검출 실패 | 검출 실패 | **무관** (착용형) |
| 도입 비용 | 무료 (앱) | $300 (D405) | $$$$ (글러브 + SDK) |
| 셋업 복잡도 | ★ | ★★ | ★★★★ |
| 정밀 pinch | DexPilot+튜닝 필요 | 좋음 | **최상** |

권장:
- **빠른 데모/프로토타입**: Phone
- **정밀한 단발 작업** (예: 객체 잡기 시연): RealSense D405
- **장시간 / 반복 작업** (예: 학습 데이터 수집): Manus
