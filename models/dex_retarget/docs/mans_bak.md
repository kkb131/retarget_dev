# Plan: Manus raw skeleton 25→21 MANO 리매퍼 추가

## Context

`--sensing manus-ros2 --config dg5f_right_dexpilot.yml` 경로는 **mock에서는 동작하지만 실제 글러브로는 동작하지 않을** 것이 코드 리뷰로 확인됨.

원인:
- Manus SDK의 raw skeleton stream 은 손당 **25개 노드**를 publish (1 wrist + thumb 4 + 4 finger × 5 = 25). 출처: [`ManusSDKTypes.h`](src/retarget_dev/sensing/manus/sdk/ManusSDK/include/ManusSDKTypes.h#L776-L781) 의 `FingerJointType` enum 및 [`ManusDataPublisher.cpp:304-336`](src/teleop_dev/sender/hand/sdk/ROS2/manus_ros2/src/ManusDataPublisher.cpp#L304-L336) 의 `for (const auto &node : t_RawSkel.nodes)` 무필터 publish.
- dex-retargeting / direct_mapping 등 downstream 은 **MANO 21 노드 layout** (wrist=0, thumb=[1..4], index=[5..8], …, pinky=[17..20], `FINGERTIP_INDICES = [4, 8, 12, 16, 20]`) 을 가정. 출처: [`sensing/manus/config.py:14-17`](src/retarget_dev/sensing/manus/config.py#L14-L17).
- 그 사이에 있는 [`ros2_provider._glove_callback`](src/retarget_dev/sensing/manus/ros2_provider.py#L141-L202) 는 `for node in msg.raw_nodes: skel_list.append(...)` 로 **25개를 publish 순서 그대로** 저장하고 끝. `joint_type`/`chain_type` 메타데이터를 무시.
- 결과: `keypoints_3d.shape == (25, 3)` 이 dex-retargeting 으로 들어가고, MANO 인덱스 4 (thumb tip을 의미)가 SDK가 publish 한 4번째 임의 노드를 가리킴 → 손이 엉뚱한 자세에서 굳음 (이전 phone DexPilot 버그와 동일한 증상이지만 원인은 다름).
- mock 이 동작하는 이유: [`mock_provider._build_skeleton`](src/retarget_dev/sensing/manus/mock_provider.py#L84-L115) 가 처음부터 `node_idx = 1 + f * 4 + j` 패턴으로 21노드 MANO layout 을 직접 만들어 줌.

## SDK 노드 25개 ↔ MANO 21노드 매핑

**중요한 함정**: [`ManusDataPublisher.cpp:669-686`](src/teleop_dev/sender/hand/sdk/ROS2/manus_ros2/src/ManusDataPublisher.cpp#L669-L686) 의 `JointTypeToString` 가 emit 하는 string 이 **anatomical 명칭과 한 단계 어긋나** 있음:

| SDK enum (`FingerJointType_*`) | publisher string | 해부학적 의미 |
|---|---|---|
| `Metacarpal` | `"MCP"` | 손바닥 안 metacarpal 뼈 base (= 해부학적 CMC) |
| `Proximal` | `"PIP"` | proximal phalanx base (= 해부학적 MCP, 너클) |
| `Intermediate` | `"IP"` | middle phalanx base (= 해부학적 PIP) |
| `Distal` | `"DIP"` | distal phalanx base (= 해부학적 DIP) |
| `Tip` | `"TIP"` | 손가락 끝 |

엄지는 `Intermediate` 가 없어서 4 노드 (MCP/PIP/DIP/TIP), 나머지 손가락은 5 노드.

`ChainTypeToString` ([`L688-L724`](src/teleop_dev/sender/hand/sdk/ROS2/manus_ros2/src/ManusDataPublisher.cpp#L688-L724)) 는 정상적인 string emit:

| SDK chain | publisher string |
|---|---|
| `ChainType_Hand` | `"Hand"` (= wrist root) |
| `ChainType_FingerThumb` | `"Thumb"` |
| `ChainType_FingerIndex` | `"Index"` |
| `ChainType_FingerMiddle` | `"Middle"` |
| `ChainType_FingerRing` | `"Ring"` |
| `ChainType_FingerPinky` | `"Pinky"` |

→ Python 쪽에서는 publisher string 그대로 매칭. 헷갈리는 명칭이지만 그 string 들이 우리가 받는 ground truth.

**Manus(SDK) → MANO 매핑 표** (string 키 기준):

| 입력 (chain, joint string) | MANO idx | MANO 의미 |
|---|---|---|
| `("Hand", *)` | 0 | wrist |
| `("Thumb", "MCP")` | 1 | thumb CMC |
| `("Thumb", "PIP")` | 2 | thumb MCP |
| `("Thumb", "DIP")` | 3 | thumb IP |
| `("Thumb", "TIP")` | 4 | thumb tip |
| `("Index", "PIP")` | 5 | index MCP |
| `("Index", "IP")` | 6 | index PIP |
| `("Index", "DIP")` | 7 | index DIP |
| `("Index", "TIP")` | 8 | index tip |
| `("Middle", "PIP")` | 9 | middle MCP |
| `("Middle", "IP")` | 10 | middle PIP |
| `("Middle", "DIP")` | 11 | middle DIP |
| `("Middle", "TIP")` | 12 | middle tip |
| `("Ring", "PIP")` | 13 | ring MCP |
| `("Ring", "IP")` | 14 | ring PIP |
| `("Ring", "DIP")` | 15 | ring DIP |
| `("Ring", "TIP")` | 16 | ring tip |
| `("Pinky", "PIP")` | 17 | pinky MCP |
| `("Pinky", "IP")` | 18 | pinky PIP |
| `("Pinky", "DIP")` | 19 | pinky DIP |
| `("Pinky", "TIP")` | 20 | pinky tip |

→ **(Index/Middle/Ring/Pinky × "MCP")** 4개 노드가 자동으로 drop 됨 (= SDK Metacarpal, MANO에 대응 없음).
→ 25 - 4 = 21 노드가 정확히 MANO 인덱스에 채워짐.

## 수정 파일 (단 하나)

[`src/retarget_dev/sensing/manus/ros2_provider.py`](src/retarget_dev/sensing/manus/ros2_provider.py)

### 변경 1 — 모듈 레벨 매핑 상수 추가

기존 `_ERGO_TYPE_MAP` 상수 ([`ros2_provider.py:33-49`](src/retarget_dev/sensing/manus/ros2_provider.py#L33-L49)) 아래에 `_MANO_REMAP` 추가. 위 표와 1:1.

### 변경 2 — `_remap_to_mano_21()` 헬퍼 함수 추가

```python
def _remap_to_mano_21(raw_nodes) -> Optional[np.ndarray]:
    """Convert Manus raw skeleton (~25 nodes) → MANO 21-node (21, 7) layout.

    Uses the (chain_type, joint_type) string pair on each ManusRawNode to
    deterministically place each node, regardless of publish order. The 4
    non-thumb Metacarpal nodes have no MANO counterpart and are dropped.

    Returns None if any required MANO slot is left unfilled (e.g., partial
    glove drop-out, missing fingers).
    """
    skel = np.full((21, 7), np.nan, dtype=np.float32)
    for node in raw_nodes:
        if node.chain_type == "Hand":
            idx = 0  # wrist root — joint_type may be "Invalid"
        else:
            idx = _MANO_REMAP.get((node.chain_type, node.joint_type))
            if idx is None:
                continue  # SDK Metacarpal of non-thumb finger → drop
        p = node.pose.position
        q = node.pose.orientation
        skel[idx] = [p.x, p.y, p.z, q.w, q.x, q.y, q.z]

    if np.isnan(skel).any():
        return None
    return skel
```

### 변경 3 — `_glove_callback` 에서 헬퍼 호출

[`ros2_provider.py:163-172`](src/retarget_dev/sensing/manus/ros2_provider.py#L163-L172) 의 raw 노드 직접 순회 블록을 헬퍼 호출로 교체:

**Before:**
```python
skeleton = None
has_skeleton = False
if msg.raw_nodes and len(msg.raw_nodes) > 0:
    skel_list = []
    for node in msg.raw_nodes:
        p = node.pose.position
        q = node.pose.orientation
        skel_list.append([p.x, p.y, p.z, q.w, q.x, q.y, q.z])
    skeleton = np.array(skel_list, dtype=np.float32)
    has_skeleton = True
```

**After:**
```python
skeleton = None
has_skeleton = False
if msg.raw_nodes and len(msg.raw_nodes) > 0:
    skeleton = _remap_to_mano_21(msg.raw_nodes)
    has_skeleton = skeleton is not None
    self._diagnose_remap(msg.raw_nodes, skeleton)
```

### 변경 4 — 1회성 진단 로깅 (`_diagnose_remap`)

처음 메시지를 받았을 때 한 번만 INFO 로그로 "n raw nodes → MANO 21 OK" 를 출력. 만약 remap 실패(None) 가 처음 발생하면 메시지에 들어 있는 `(chain, joint)` 키 set 을 WARNING 으로 출력해서 디버깅이 쉽도록.

```python
# In __init__
self._remap_logged_ok = False
self._remap_logged_fail = False

# Helper method on Ros2ManusProvider
def _diagnose_remap(self, raw_nodes, mapped) -> None:
    if mapped is not None and not self._remap_logged_ok:
        logger.info(
            "Manus raw skeleton: %d raw nodes → MANO 21 (remap OK)",
            len(raw_nodes),
        )
        self._remap_logged_ok = True
    elif mapped is None and not self._remap_logged_fail:
        keys = sorted({(n.chain_type, n.joint_type) for n in raw_nodes})
        logger.warning(
            "Manus raw skeleton remap incomplete — MANO slots unfilled. "
            "Received %d nodes with (chain, joint) pairs: %s",
            len(raw_nodes), keys,
        )
        self._remap_logged_fail = True
```

`__init__` 에 두 플래그 추가만 하면 됨.

### 변경 없음 (자동으로 호환됨)

- [`manus_sensing.py:74-90`](src/retarget_dev/sensing/manus/manus_sensing.py#L74-L90) `get_keypoints()` 는 이미 `data.skeleton[:, :3]` + `positions - positions[WRIST_IDX]` (WRIST_IDX=0) 를 한다. 우리가 MANO[0]에 wrist를 보장해 넣으므로 그대로 동작.
- [`mock_provider.py`](src/retarget_dev/sensing/manus/mock_provider.py) 는 ros2 경로와 무관 — 변경 없음.
- [`config.py`](src/retarget_dev/sensing/manus/config.py) `NUM_SKELETON_NODES = 21` 가정이 이제 ros2 경로에서도 사실이 됨.

## 손대지 않는 것 / 의도적으로 미루는 것

- [`sdk_provider.py`](src/retarget_dev/sensing/manus/sdk_provider.py) 도 동일한 구조적 문제 보유 — 하지만 그 backend 인 [`SDKClient.cpp:1525-1550`](src/teleop_dev/sender/hand/sdk/SDKClient_Linux/SDKClient.cpp#L1525-L1550) 의 `--stream-json` 출력은 **각 노드의 chain/joint 메타데이터를 JSON 에 포함하지 않음** (좌표 7개만 dump). 따라서 Python 쪽 만으로는 고칠 수 없음. 사용자가 `--sensing manus-sdk` 를 실제로 쓰기 시작할 때 별도 작업으로 다룸. 이번 PR 범위 밖.
- C++ publisher 자체는 손대지 않음 — Python 단에서 string 매칭으로 충분.

## 검증

### 단위 검증 (하드웨어 없이)

`__main__` 블록에서 가짜 `raw_nodes` 25개를 만들어 헬퍼만 직접 호출 — 실제 글러브 없이도 매핑 정확성을 확인 가능. 가벼운 ad-hoc smoke test:

```python
python3 -c "
from retarget_dev.sensing.manus.ros2_provider import _remap_to_mano_21
class FakePos: x=0; y=0; z=0
class FakeRot: w=1; x=0; y=0; z=0
class FakePose: position=FakePos(); orientation=FakeRot()
class FakeNode:
    def __init__(self, chain, joint, x):
        self.chain_type = chain
        self.joint_type = joint
        p = FakePos(); p.x = x
        self.pose = type('P', (), {'position': p, 'orientation': FakeRot()})()

# 25 nodes: 1 hand + 4 thumb + 4*5 finger
nodes = [FakeNode('Hand', 'Invalid', 0.0)]
nodes += [FakeNode('Thumb', j, 0.1) for j in ['MCP','PIP','DIP','TIP']]
for finger in ['Index','Middle','Ring','Pinky']:
    nodes += [FakeNode(finger, j, 0.2) for j in ['MCP','PIP','IP','DIP','TIP']]
print(f'Input nodes: {len(nodes)} (expected 25)')

import numpy as np
skel = _remap_to_mano_21(nodes)
print(f'Output shape: {skel.shape} (expected (21, 7))')
print(f'Has NaN: {np.isnan(skel).any()} (expected False)')
print(f'Wrist (MANO 0) x = {skel[0, 0]} (expected 0.0)')
print(f'Thumb tip (MANO 4) x = {skel[4, 0]} (expected 0.1)')
print(f'Index tip (MANO 8) x = {skel[8, 0]} (expected 0.2)')
"
```

### End-to-end 검증 (실제 글러브 필요)

```bash
# T1: manus publisher
ros2 run manus_ros2 manus_data_publisher

# T2: 직접 raw 메시지 확인
ros2 topic echo /manus_glove_0 --once   # raw_node_count, joint_type/chain_type 확인

# T3: 우리 sensing 단독 (logging level INFO)
python3 -m retarget_dev.sensing.manus.main --ros2 --hand right
# 기대 로그: "Manus raw skeleton: 25 raw nodes → MANO 21 (remap OK)"

# T4: dex-retargeting까지
python3 -m retarget_dev.models.dex_retarget.main \
  --sensing manus-ros2 \
  --config retarget_dev/models/dex_retarget/config/dg5f_right_dexpilot.yml
# IsaacSim DG-5F 가 손 동작을 따라가야 함
```

### Failure 모드 검증

손가락 하나가 트래킹에서 빠진 경우 (e.g., glove 일부 센서 dropout) → `_remap_to_mano_21` 가 `None` 반환 → `has_skeleton=False` → `manus_sensing.get_keypoints()` 가 None 반환 → dex-retargeting 호출 skip → 이전 프레임 자세 유지. 첫 발생 시 WARNING 로그 1회 + 어떤 (chain, joint) 키들이 들어왔는지 출력.

## 회귀 가능성

- mock 경로: 변경 없음 (mock provider 가 직접 21 MANO 빌드 → 그대로 동작) ✓
- sdk 경로: 변경 없음 (의도적 미루기) ✓
- ros2 경로 with 21-node publisher: 만약 어떤 변형 publisher 가 이미 21노드만 보낸다면, 같은 string 매칭이 동작하므로 동일하게 작동 ✓
- ros2 경로 with 알 수 없는 string: 첫 프레임 WARNING + skip — 사용자가 즉시 인지 가능 ✓
