# Hand Retargeting Research: Input Sources → Tesollo DG5F

> 조사일: 2026-04-07
> 목적: Manus Skeleton, Manus Ergonomics, MediaPipe Hands → Tesollo DG5F 리타게팅을 위한 사전 조사

---

## 1. Tesollo DG5F (Target Robot Hand)

### 1.1 개요
- **URDF**: `src/dg5f_ros2/dg5f_description/urdf/dg5f_right.urdf`
- **총 관절 수**: 20 revolute joints (+ 7 fixed joints)
- **손가락 수**: 5 (Thumb=finger1, Index=finger2, Middle=finger3, Ring=finger4, Pinky=finger5)
- **각 손가락 DOF**: 4 revolute joints
- **총 Actuated DOF**: 20

### 1.2 Kinematic Chain (상세)

```
rl_dg_mount (fixed base)
  └─ rl_dg_base  [rj_dg_base, fixed]
      └─ rl_dg_palm  [rj_dg_palm, fixed]
          ├─ Finger 1 (Thumb): 4 DOF
          │   ├─ rj_dg_1_1  X축  (abduction/adduction)
          │   ├─ rj_dg_1_2  Z축  (opposition/rotation)
          │   ├─ rj_dg_1_3  X축  (flexion)
          │   └─ rj_dg_1_4  X축  (flexion)
          │       └─ rl_dg_1_tip [fixed]
          │
          ├─ Finger 2 (Index): 4 DOF
          │   ├─ rj_dg_2_1  X축  (abduction/adduction)
          │   ├─ rj_dg_2_2  Y축  (MCP flexion)
          │   ├─ rj_dg_2_3  Y축  (PIP flexion)
          │   └─ rj_dg_2_4  Y축  (DIP flexion)
          │       └─ rl_dg_2_tip [fixed]
          │
          ├─ Finger 3 (Middle): 4 DOF
          │   ├─ rj_dg_3_1  X축  (abduction/adduction)
          │   ├─ rj_dg_3_2  Y축  (MCP flexion)
          │   ├─ rj_dg_3_3  Y축  (PIP flexion)
          │   └─ rj_dg_3_4  Y축  (DIP flexion)
          │       └─ rl_dg_3_tip [fixed]
          │
          ├─ Finger 4 (Ring): 4 DOF
          │   ├─ rj_dg_4_1  X축  (abduction/adduction)
          │   ├─ rj_dg_4_2  Y축  (MCP flexion)
          │   ├─ rj_dg_4_3  Y축  (PIP flexion)
          │   └─ rj_dg_4_4  Y축  (DIP flexion)
          │       └─ rl_dg_4_tip [fixed]
          │
          └─ Finger 5 (Pinky): 4 DOF
              ├─ rj_dg_5_1  Z축  (abduction/adduction)
              ├─ rj_dg_5_2  X축  (MCP spread)
              ├─ rj_dg_5_3  Y축  (PIP flexion)
              └─ rj_dg_5_4  Y축  (DIP flexion)
                  └─ rl_dg_5_tip [fixed]
```

### 1.3 Joint Limits (URDF에서 추출)

| Joint | Axis | Lower (rad) | Upper (rad) | Lower (deg) | Upper (deg) | 역할 |
|-------|------|-------------|-------------|-------------|-------------|------|
| **Finger 1 (Thumb)** |
| rj_dg_1_1 | X | -0.384 | 0.890 | -22.0 | 51.0 | Abduction |
| rj_dg_1_2 | Z | -3.142 | 0.000 | -180.0 | 0.0 | Opposition/Rotation |
| rj_dg_1_3 | X | -1.571 | 1.571 | -90.0 | 90.0 | Proximal Flexion |
| rj_dg_1_4 | X | -1.571 | 1.571 | -90.0 | 90.0 | Distal Flexion |
| **Finger 2 (Index)** |
| rj_dg_2_1 | X | -0.419 | 0.611 | -24.0 | 35.0 | Abduction |
| rj_dg_2_2 | Y | 0.000 | 2.007 | 0.0 | 115.0 | MCP Flexion |
| rj_dg_2_3 | Y | -1.571 | 1.571 | -90.0 | 90.0 | PIP Flexion |
| rj_dg_2_4 | Y | -1.571 | 1.571 | -90.0 | 90.0 | DIP Flexion |
| **Finger 3 (Middle)** |
| rj_dg_3_1 | X | -0.611 | 0.611 | -35.0 | 35.0 | Abduction |
| rj_dg_3_2 | Y | 0.000 | 1.955 | 0.0 | 112.0 | MCP Flexion |
| rj_dg_3_3 | Y | -1.571 | 1.571 | -90.0 | 90.0 | PIP Flexion |
| rj_dg_3_4 | Y | -1.571 | 1.571 | -90.0 | 90.0 | DIP Flexion |
| **Finger 4 (Ring)** |
| rj_dg_4_1 | X | -0.611 | 0.419 | -35.0 | 24.0 | Abduction |
| rj_dg_4_2 | Y | 0.000 | 1.902 | 0.0 | 109.0 | MCP Flexion |
| rj_dg_4_3 | Y | -1.571 | 1.571 | -90.0 | 90.0 | PIP Flexion |
| rj_dg_4_4 | Y | -1.571 | 1.571 | -90.0 | 90.0 | DIP Flexion |
| **Finger 5 (Pinky)** |
| rj_dg_5_1 | Z | -0.017 | 1.047 | -1.0 | 60.0 | Abduction |
| rj_dg_5_2 | X | -0.419 | 0.611 | -24.0 | 35.0 | MCP Spread |
| rj_dg_5_3 | Y | -1.571 | 1.571 | -90.0 | 90.0 | PIP Flexion |
| rj_dg_5_4 | Y | -1.571 | 1.571 | -90.0 | 90.0 | DIP Flexion |

> 모든 joint effort: 7.5 Nm, velocity: 3.14 rad/s (π rad/s)

### 1.4 Link Lengths (URDF origin offset 기준, 단위: mm)

| Finger | Segment | Length (mm) | 비고 |
|--------|---------|-------------|------|
| **Thumb** | Palm→J1 | 28.0 (xy) | (-16.2, 19.0, 12.8) |
| | J1→J2 | 42.0 | x방향 |
| | J2→J3 | 31.0 | y방향 |
| | J3→J4 | 38.8 | y방향 |
| | J4→Tip | 36.3 | y방향 |
| **Index** | Palm→J1 | 71.9 | (-7.1, 27.0, 66.1) |
| | J1→J2 | 31.6 | (17.65, 0, 26.5) |
| | J2→J3 | 38.8 | z방향 |
| | J3→J4 | 38.8 | z방향 |
| | J4→Tip | 25.5 | z방향 |
| **Middle** | Palm→J1 | 74.4 | (-7.1, 2.5, 70.1) |
| | J1→J2 | 31.6 | (17.65, 0, 26.5) |
| | J2→J3 | 38.8 | z방향 |
| | J3→J4 | 38.8 | z방향 |
| | J4→Tip | 25.5 | z방향 |
| **Ring** | Palm→J1 | 66.3 | (-7.1, -22.0, 62.1) |
| | J1→J2 | 31.6 | (17.65, 0, 26.5) |
| | J2→J3 | 38.8 | z방향 |
| | J3→J4 | 38.8 | z방향 |
| | J4→Tip | 25.5 | z방향 |
| **Pinky** | Palm→J1 | 28.6 | (10.3, -19.5, 18.2) |
| | J1→J2 | 47.3 | (0, -28.0, 38.1) |
| | J2→J3 | 31.0 | z방향 |
| | J3→J4 | 38.8 | z방향 |
| | J4→Tip | 36.3 | z방향 |

### 1.5 DG5F DOF 요약
| 손가락 | Abduction | MCP Flex | PIP Flex | DIP Flex | 합계 |
|--------|-----------|----------|----------|----------|------|
| Thumb  | 1 (rj_1_1) | 1 (rj_1_2, opposition) | 1 (rj_1_3) | 1 (rj_1_4) | 4 |
| Index  | 1 (rj_2_1) | 1 (rj_2_2) | 1 (rj_2_3) | 1 (rj_2_4) | 4 |
| Middle | 1 (rj_3_1) | 1 (rj_3_2) | 1 (rj_3_3) | 1 (rj_3_4) | 4 |
| Ring   | 1 (rj_4_1) | 1 (rj_4_2) | 1 (rj_4_3) | 1 (rj_4_4) | 4 |
| Pinky  | 1 (rj_5_1) | 1 (rj_5_2) | 1 (rj_5_3) | 1 (rj_5_4) | 4 |
| **합계** | **5** | **5** | **5** | **5** | **20** |

---

## 2. Manus Skeleton (Raw Skeleton)

### 2.1 개요
- **소스**: Manus Glove SDK (ManusSDK)
- **노드 수**: 25개 per hand (Wrist 1 + Thumb 4 + Index/Middle/Ring/Pinky 각 5)
- **데이터 타입**: 각 노드별 position (float x,y,z) + quaternion (float w,x,y,z) + scale (float x,y,z)
- **주파수**: 120 Hz (ROS2 topic)
- **좌표계**: Configurable (기본: X-from-viewer, Z-up, right-handed, meters)

### 2.2 Node Hierarchy

| Node ID | Parent | Chain | Joint Type | 해부학적 대응 |
|---------|--------|-------|------------|-------------|
| 0 | 0 (self) | Hand | Root | **Wrist** |
| **Thumb** |
| 1 | 0 | Thumb | Metacarpal | Thumb CMC |
| 2 | 1 | Thumb | Proximal | Thumb MCP |
| 3 | 2 | Thumb | Distal | Thumb IP |
| 4 | 3 | Thumb | Tip | Thumb Tip |
| **Index** |
| 5 | 0 | Index | Metacarpal | Index MC |
| 6 | 5 | Index | Proximal | Index MCP |
| 7 | 6 | Index | Intermediate | Index PIP |
| 8 | 7 | Index | Distal | Index DIP |
| 9 | 8 | Index | Tip | Index Tip |
| **Middle** |
| 10 | 0 | Middle | Metacarpal | Middle MC |
| 11 | 10 | Middle | Proximal | Middle MCP |
| 12 | 11 | Middle | Intermediate | Middle PIP |
| 13 | 12 | Middle | Distal | Middle DIP |
| 14 | 13 | Middle | Tip | Middle Tip |
| **Ring** |
| 15 | 0 | Ring | Metacarpal | Ring MC |
| 16 | 15 | Ring | Proximal | Ring MCP |
| 17 | 16 | Ring | Intermediate | Ring PIP |
| 18 | 17 | Ring | Distal | Ring DIP |
| 19 | 18 | Ring | Tip | Ring Tip |
| **Pinky** |
| 20 | 0 | Pinky | Metacarpal | Pinky MC |
| 21 | 20 | Pinky | Proximal | Pinky MCP |
| 22 | 21 | Pinky | Intermediate | Pinky PIP |
| 23 | 22 | Pinky | Distal | Pinky DIP |
| 24 | 23 | Pinky | Tip | Pinky Tip |

### 2.3 Data Format (C++ SDK)

```cpp
struct ManusVec3 { float x, y, z; };
struct ManusQuaternion { float w, x, y, z; };  // Scalar-first (WXYZ)
struct ManusTransform {
    ManusVec3 position;
    ManusQuaternion rotation;
    ManusVec3 scale;
};
struct SkeletonNode {
    uint32_t id;
    ManusTransform transform;
};
```

### 2.4 ROS2 메시지 (ManusRawNode.msg)

| Field | Type | 설명 |
|-------|------|------|
| `node_id` | int32 | 노드 ID (0-24) |
| `parent_node_id` | int32 | 부모 노드 ID |
| `joint_type` | string | "MCP", "PIP", "IP", "DIP", "TIP", "Invalid" |
| `chain_type` | string | "Thumb", "Index", "Middle", "Ring", "Pinky", "Hand" |
| `pose` | geometry_msgs/Pose | position + quaternion (XYZW, ROS convention) |

### 2.5 주요 특성
- **World/Local 좌표**: `p_UseWorldCoordinates` 설정으로 전환 가능
  - World: 모든 노드가 wrist 기준 절대 좌표
  - Local: 부모 노드 기준 상대 좌표 (리타게팅에 더 유용)
- **Quaternion 변환 주의**: SDK는 WXYZ, ROS2는 XYZW
- **Node ID 불안정**: 런타임에 `GetRawSkeletonNodeInfoArray()`로 조회 권장
- **Thumb**: 4노드 (Intermediate 없음), 다른 손가락: 5노드

---

## 3. Manus Ergonomics

### 3.1 개요
- **소스**: Manus SDK Ergonomics API (Raw Skeleton과 동일 glove topic에서 전송)
- **데이터 수**: 20개 per hand (5 fingers x 4 values)
- **데이터 타입**: float32 (각도, degrees)
- **주파수**: 120 Hz (Raw Skeleton과 동일 메시지)

### 3.2 Ergonomics Data List (Right Hand, 20 values)

| # | Type Name | 설명 | DOF 대응 |
|---|-----------|------|---------|
| 1 | ThumbMCPSpread | Thumb CMC abduction (palmar) | Abduction |
| 2 | ThumbMCPStretch | Thumb MCP flexion/extension | MCP Flex |
| 3 | ThumbPIPStretch | Thumb PIP flexion/extension | PIP Flex |
| 4 | ThumbDIPStretch | Thumb DIP flexion/extension | DIP Flex |
| 5 | IndexMCPSpread | Index MCP abduction | Abduction |
| 6 | IndexMCPStretch | Index MCP flexion/extension | MCP Flex |
| 7 | IndexPIPStretch | Index PIP flexion/extension | PIP Flex |
| 8 | IndexDIPStretch | Index DIP flexion/extension | DIP Flex |
| 9 | MiddleMCPSpread | Middle MCP abduction | Abduction |
| 10 | MiddleMCPStretch | Middle MCP flexion/extension | MCP Flex |
| 11 | MiddlePIPStretch | Middle PIP flexion/extension | PIP Flex |
| 12 | MiddleDIPStretch | Middle DIP flexion/extension | DIP Flex |
| 13 | RingMCPSpread | Ring MCP abduction | Abduction |
| 14 | RingMCPStretch | Ring MCP flexion/extension | MCP Flex |
| 15 | RingPIPStretch | Ring PIP flexion/extension | PIP Flex |
| 16 | RingDIPStretch | Ring DIP flexion/extension | DIP Flex |
| 17 | PinkyMCPSpread | Pinky MCP abduction | Abduction |
| 18 | PinkyMCPStretch | Pinky MCP flexion/extension | MCP Flex |
| 19 | PinkyPIPStretch | Pinky PIP flexion/extension | PIP Flex |
| 20 | PinkyDIPStretch | Pinky DIP flexion/extension | DIP Flex |

### 3.3 ROS2 메시지 (ManusErgonomics.msg)

| Field | Type | 설명 |
|-------|------|------|
| `type` | string | e.g., "RightFingerIndexMCPStretch" |
| `value` | float32 | 각도 (degrees) |

### 3.4 값 범위 (공식 문서 기준)
- **Stretch (Flexion)**: 약 -30° ~ +108° (음수=과신전, 양수=굴곡)
- **Spread (Abduction)**: 약 -27° ~ +44°
- 양수 = 손가락 굴곡/벌림, 음수 = 신전/모음

### 3.5 주요 특성
- **DG5F와 1:1 DOF 매칭 가능**: 20 Ergonomics values ↔ 20 DG5F joints
- **가장 간단한 리타게팅 경로**: 각도→각도 직접 매핑 (스케일링만 필요)
- **Stretch 정의**: 이전 bone과 현재 bone 사이 각도 (local frame)
- **Spread 정의**: MCP에서의 좌우 벌림 각도 (hand forward vector 기준)

---

## 4. MediaPipe Hands

### 4.1 개요
- **소스**: Google MediaPipe (camera-based, markerless)
- **랜드마크 수**: 21개 per hand
- **데이터 타입**: float32 x, y, z (+ optional visibility, presence)
- **출력**: 3D positions만 (관절 각도/회전 정보 없음)
- **주파수**: ~30-60 FPS (카메라 의존), Pixel 6 기준 ~58 FPS (17ms CPU)

### 4.2 Landmark List (21 landmarks)

| Index | Name | 해부학적 대응 |
|-------|------|-------------|
| 0 | WRIST | 손목 중심 |
| 1 | THUMB_CMC | 엄지 CMC 관절 |
| 2 | THUMB_MCP | 엄지 MCP 관절 |
| 3 | THUMB_IP | 엄지 IP 관절 |
| 4 | THUMB_TIP | 엄지 끝 |
| 5 | INDEX_FINGER_MCP | 검지 MCP 관절 |
| 6 | INDEX_FINGER_PIP | 검지 PIP 관절 |
| 7 | INDEX_FINGER_DIP | 검지 DIP 관절 |
| 8 | INDEX_FINGER_TIP | 검지 끝 |
| 9 | MIDDLE_FINGER_MCP | 중지 MCP 관절 |
| 10 | MIDDLE_FINGER_PIP | 중지 PIP 관절 |
| 11 | MIDDLE_FINGER_DIP | 중지 DIP 관절 |
| 12 | MIDDLE_FINGER_TIP | 중지 끝 |
| 13 | RING_FINGER_MCP | 약지 MCP 관절 |
| 14 | RING_FINGER_PIP | 약지 PIP 관절 |
| 15 | RING_FINGER_DIP | 약지 DIP 관절 |
| 16 | RING_FINGER_TIP | 약지 끝 |
| 17 | PINKY_MCP | 소지 MCP 관절 |
| 18 | PINKY_PIP | 소지 PIP 관절 |
| 19 | PINKY_DIP | 소지 DIP 관절 |
| 20 | PINKY_TIP | 소지 끝 |

### 4.3 Skeleton Topology (HAND_CONNECTIONS, 21 edges)

```
Palm 구조:
  WRIST(0) ── THUMB_CMC(1)
  WRIST(0) ── INDEX_MCP(5)
  WRIST(0) ── PINKY_MCP(17)
  INDEX_MCP(5) ── MIDDLE_MCP(9)
  MIDDLE_MCP(9) ── RING_MCP(13)
  RING_MCP(13) ── PINKY_MCP(17)

Thumb:   1 → 2 → 3 → 4
Index:   5 → 6 → 7 → 8
Middle:  9 → 10 → 11 → 12
Ring:   13 → 14 → 15 → 16
Pinky:  17 → 18 → 19 → 20
```

### 4.4 좌표계

| 유형 | x | y | z | 원점 | 단위 |
|------|---|---|---|------|------|
| **Normalized** | [0,1] 이미지 폭 | [0,1] 이미지 높이 | 깊이 (wrist 기준) | 이미지 좌상단 | normalized |
| **World** | meters | meters | meters | 손 중심 | m |

- **World landmarks 사용 권장**: 리타게팅에는 `world_landmarks` (미터 단위) 사용
- **Z축 정확도 낮음**: depth는 x,y 대비 노이즈가 큼

### 4.5 출력 데이터 (Python)

```python
# Legacy API
result = hands.process(image)
result.multi_hand_landmarks       # NormalizedLandmarkList[]
result.multi_hand_world_landmarks # LandmarkList[] (meters)
result.multi_handedness           # Classification[] ("Left"/"Right" + score)

# Tasks API (newer)
result = landmarker.detect(image)
result.hand_landmarks        # NormalizedLandmark[][]
result.hand_world_landmarks  # Landmark[][] (meters)
result.handedness            # Categories[][]
```

### 4.6 주요 특성
- **관절 각도 미제공**: 3D position만 출력 → IK 또는 벡터 연산으로 각도 계산 필요
- **Metacarpal 미분리**: 손바닥 구조가 rigid 가정 (Manus와 다름)
- **Thumb**: CMC→MCP→IP→TIP (4 landmarks), Manus Skeleton과 동일 구조
- **Other fingers**: MCP→PIP→DIP→TIP (4 landmarks, Metacarpal 없음)
- **Handedness 미러링**: 셀카 카메라 기준 → 후면 카메라 시 label 반전 필요

---

## 5. 소스 간 비교표

### 5.1 데이터 개요 비교

| 항목 | Manus Skeleton | Manus Ergonomics | MediaPipe Hands | DG5F |
|------|---------------|-----------------|----------------|------|
| **데이터 포인트 수** | 25 nodes/hand | 20 values/hand | 21 landmarks/hand | 20 joints |
| **데이터 타입** | Pose (pos+quat) | float (degrees) | float (x,y,z) | revolute angle |
| **좌표 정보** | position + rotation | joint angle only | position only | joint angle |
| **주파수** | 120 Hz | 120 Hz | 30-60 FPS | - |
| **센서** | IMU + stretch sensor | 위와 동일 (파생) | RGB camera | encoder |
| **Metacarpal 포함** | O (5개) | X | X | X |
| **Tip 노드** | O (5개) | X | O (5개) | O (fixed link) |
| **실제 DOF** | 25x7=175 float | 20 float | 21x3=63 float | 20 float |
| **ROS2 지원** | O (manus_ros2) | O (동일 topic) | X (별도 구현) | O (dg5f_ros2) |

### 5.2 손가락별 노드/DOF 비교

| 손가락 | Manus Skeleton Nodes | Manus Ergo DOF | MediaPipe Landmarks | DG5F DOF |
|--------|---------------------|----------------|--------------------|---------|
| Thumb | 4 (CMC,MCP,IP,Tip) | 4 (Spread,MCP,PIP,DIP) | 4 (CMC,MCP,IP,Tip) | 4 (Abd,Opp,Flex,Flex) |
| Index | 5 (MC,MCP,PIP,DIP,Tip) | 4 (Spread,MCP,PIP,DIP) | 4 (MCP,PIP,DIP,Tip) | 4 (Abd,MCP,PIP,DIP) |
| Middle | 5 (MC,MCP,PIP,DIP,Tip) | 4 (Spread,MCP,PIP,DIP) | 4 (MCP,PIP,DIP,Tip) | 4 (Abd,MCP,PIP,DIP) |
| Ring | 5 (MC,MCP,PIP,DIP,Tip) | 4 (Spread,MCP,PIP,DIP) | 4 (MCP,PIP,DIP,Tip) | 4 (Abd,MCP,PIP,DIP) |
| Pinky | 5 (MC,MCP,PIP,DIP,Tip) | 4 (Spread,MCP,PIP,DIP) | 4 (MCP,PIP,DIP,Tip) | 4 (Abd,Spread,PIP,DIP) |
| **합계** | **24+1 wrist** | **20** | **20+1 wrist** | **20** |

---

## 6. Kinematic Similarity Analysis (리타게팅 유사도 분석)

### 6.1 Manus Ergonomics → DG5F (가장 유사, **권장 경로**)

#### 유사도: ★★★★★ (매우 높음)

**DOF 대응 (20:20 완전 매칭)**

| Manus Ergonomics | → | DG5F Joint | 매핑 방식 |
|-----------------|---|-----------|----------|
| ThumbMCPSpread | → | rj_dg_1_1 (X, abd) | Linear scale |
| ThumbMCPStretch | → | rj_dg_1_2 (Z, opp) | Linear scale + offset |
| ThumbPIPStretch | → | rj_dg_1_3 (X, flex) | Linear scale |
| ThumbDIPStretch | → | rj_dg_1_4 (X, flex) | Linear scale |
| IndexMCPSpread | → | rj_dg_2_1 (X, abd) | Linear scale |
| IndexMCPStretch | → | rj_dg_2_2 (Y, flex) | Linear scale |
| IndexPIPStretch | → | rj_dg_2_3 (Y, flex) | Linear scale |
| IndexDIPStretch | → | rj_dg_2_4 (Y, flex) | Linear scale |
| MiddleMCPSpread | → | rj_dg_3_1 (X, abd) | Linear scale |
| MiddleMCPStretch | → | rj_dg_3_2 (Y, flex) | Linear scale |
| MiddlePIPStretch | → | rj_dg_3_3 (Y, flex) | Linear scale |
| MiddleDIPStretch | → | rj_dg_3_4 (Y, flex) | Linear scale |
| RingMCPSpread | → | rj_dg_4_1 (X, abd) | Linear scale |
| RingMCPStretch | → | rj_dg_4_2 (Y, flex) | Linear scale |
| RingPIPStretch | → | rj_dg_4_3 (Y, flex) | Linear scale |
| RingDIPStretch | → | rj_dg_4_4 (Y, flex) | Linear scale |
| PinkyMCPSpread | → | rj_dg_5_1 (Z, abd) | Linear scale |
| PinkyMCPStretch | → | rj_dg_5_2 (X, spread) | Linear scale |
| PinkyPIPStretch | → | rj_dg_5_3 (Y, flex) | Linear scale |
| PinkyDIPStretch | → | rj_dg_5_4 (Y, flex) | Linear scale |

#### 매핑 공식 (기본)
```
q_dg5f[i] = clamp(
    scale[i] * deg2rad(ergo_value[i]) + offset[i],
    joint_lower[i],
    joint_upper[i]
)
```

#### 주의점
1. **Thumb opposition (rj_dg_1_2)**: DG5F는 Z축 회전 (-180°~0°)으로 opposition 구현. Manus ThumbMCPStretch는 단순 flexion. 비선형 매핑 또는 별도 캘리브레이션 필요.
2. **부호 방향**: Manus Stretch 양수=굴곡, DG5F joint 양수 방향은 축마다 다름. 각 joint별 부호 확인 필수.
3. **Range 스케일링**: Manus 범위(-30°~108°)와 DG5F 범위(0°~115° 등)가 다름 → min-max normalization 권장.
4. **Pinky 구조**: DG5F pinky는 J1=Z축(abd), J2=X축(spread)으로 일반 손가락과 축이 다름.

---

### 6.2 Manus Skeleton → DG5F

#### 유사도: ★★★★☆ (높음, 추가 계산 필요)

**매핑 방법**: Quaternion → Joint Angle 변환

```python
# 각 노드 쌍의 local quaternion에서 joint angle 추출
# 예: Index MCP flexion
q_parent = skeleton_nodes[5].rotation  # Index Metacarpal
q_child  = skeleton_nodes[6].rotation  # Index Proximal
q_local  = q_parent.inverse() * q_child
flex_angle = extract_euler_component(q_local, axis='Y')  # flexion axis
```

#### 주의점
1. **25 nodes → 20 DOF**: Metacarpal 노드와 Tip 노드는 비활성 → 인접 관절 간 상대 회전만 사용
2. **Quaternion decomposition**: 각 관절에서 의미 있는 축 성분만 추출 필요 (swing-twist decomposition)
3. **Coordinate system 변환**: Manus 좌표계(configurable) → DG5F URDF 좌표계 변환 필요
4. **World vs Local**: Local 모드 사용 시 부모-자식 상대 회전 = 관절 각도로 직접 사용 가능
5. **Fingertip position 활용 가능**: Tip 노드 position으로 IK 기반 리타게팅도 가능 (more robust for grasping)

---

### 6.3 MediaPipe Hands → DG5F

#### 유사도: ★★★☆☆ (중간, 상당한 처리 필요)

**매핑 방법**: 3D Position → Joint Angle 계산

```python
# 예: Index MCP flexion angle
v_metacarpal = landmarks[5].xyz - landmarks[0].xyz   # Wrist → Index MCP
v_proximal   = landmarks[6].xyz - landmarks[5].xyz   # Index MCP → PIP
flex_angle = arccos(dot(v_metacarpal, v_proximal) / (|v_metacarpal| * |v_proximal|))

# Abduction angle (MCP spread)
# Palm normal 기준으로 좌우 각도 계산
palm_normal = cross(landmarks[5].xyz - landmarks[0].xyz,
                    landmarks[17].xyz - landmarks[0].xyz)
# ... project v_proximal onto palm plane, measure lateral angle
```

#### 주의점
1. **관절 각도 직접 미제공**: 모든 각도를 벡터 연산으로 계산해야 함
2. **Metacarpal 부재**: 손바닥이 rigid body → MCP abduction 각도 계산 시 인접 손가락 간 각도로 추정
3. **Depth 노이즈**: Z축 부정확 → flexion 각도보다 abduction 각도 추정이 더 불안정
4. **Scale 미포함**: world_landmarks는 미터 단위이나, 손 크기 개인차 존재
5. **Thumb CMC**: MediaPipe는 CMC 위치만 제공 → 2DOF (flexion + abduction) 분리 계산 필요
6. **프레임 정의 필요**: 각 관절에서의 local frame을 명시적으로 정의해야 함
   - palm plane normal 계산 (WRIST, INDEX_MCP, PINKY_MCP 이용)
   - 각 bone vector의 flexion/abduction 성분 분리

---

## 7. 리타게팅 전략 비교

### 7.1 전략별 장단점

| 전략 | 입력 소스 | 복잡도 | 정확도 | 실시간성 | 권장 |
|------|---------|--------|--------|---------|------|
| **A. Direct Angle Mapping** | Manus Ergonomics | ★☆☆ (간단) | ★★★★ | ★★★★★ (120Hz) | **1순위** |
| **B. Quaternion Decomposition** | Manus Skeleton | ★★★ (중간) | ★★★★★ | ★★★★☆ (120Hz) | 2순위 |
| **C. Multi-Cost Optimization** | Manus Skeleton + MediaPipe | ★★★★ (복잡) | ★★★★★ | ★★★☆☆ | 고급 (`models/dex_retarget`) |
| **D. Vector Angle Extraction** | MediaPipe Hands | ★★★ (중간) | ★★★ | ★★★☆☆ (30-60fps) | 3순위 |

### 7.2 권장 구현 순서

**Phase 1: Manus Ergonomics Direct Mapping (가장 빠른 프로토타입)**
```
Manus Ergo (20 angles, deg) → deg2rad → scale+offset → clamp → DG5F joints (20 rad)
```
- 구현 시간: ~2시간
- 캘리브레이션: min-max로 range 매핑
- 장점: 가장 간단, 1:1 매핑, 실시간 보장

**Phase 2: MediaPipe Vector Angle (카메라 기반 대안)**
```
MediaPipe (21 positions) → bone vectors → joint angles → scale+clamp → DG5F joints
```
- 구현 시간: ~1일
- 추가 구현: palm frame 정의, flexion/abduction 분리, noise filter
- 장점: 글러브 불필요, 접근성 높음

**Phase 3: Manus Skeleton IK (고급 리타게팅)**
```
Manus Skeleton (25 poses) → fingertip positions → DG5F IK → joint angles
```
- 구현 시간: ~2-3일
- 추가 구현: DG5F finger FK/IK solver, cost function 설계
- 장점: 파지 자세 정확도 최고

---

## 8. 주의사항 및 고려사항

### 8.1 Kinematic 차이점

| 항목 | Human Hand | DG5F |
|------|-----------|------|
| Thumb DOF | 5 (CMC 2 + MCP 2 + IP 1) | 4 (abd + opp + flex + flex) |
| Finger DOF | 4 (MCP 2 + PIP 1 + DIP 1) | 4 (abd + MCP + PIP + DIP) |
| MCP coupling | MCP flex와 abd 독립 | 독립 (각각 별도 actuator) |
| PIP-DIP coupling | DIP ≈ 2/3 * PIP (건에 의한 커플링) | 독립 (각각 별도 actuator) |
| Thumb opposition | 다축 복합 운동 | 단일 Z축 revolute |
| Metacarpal | 유연 (약간의 아치 형성) | 없음 (palm 고정) |

### 8.2 Critical Retargeting Issues

#### Issue 1: Thumb Opposition Mapping
- **문제**: 인간 엄지의 opposition은 CMC에서 2DOF (flexion + abduction) 복합 운동
- **DG5F**: rj_dg_1_1 (X, abd) + rj_dg_1_2 (Z, opp) 로 근사
- **해결**: Manus ThumbMCPSpread → rj_dg_1_1, ThumbMCPStretch → rj_dg_1_2 매핑, 단 rj_dg_1_2의 범위(-180°~0°)는 비선형 변환 필요

#### Issue 2: PIP-DIP Coupling
- **문제**: 인간 손은 DIP가 PIP에 기계적으로 커플링됨 (DIP ≈ 0.67 * PIP)
- **DG5F**: PIP, DIP 독립 제어 가능
- **선택**:
  - (a) Manus DIP 값을 그대로 사용 (독립 제어)
  - (b) DIP = k * PIP 커플링 강제 (더 자연스러운 동작)
  - (c) 사용자 설정 가능 (권장)

#### Issue 3: MediaPipe Depth Noise
- **문제**: 단일 카메라 depth 추정 노이즈 → abduction 각도 부정확
- **해결**:
  - Temporal smoothing (EMA filter)
  - Abduction 값에 deadzone 적용
  - PIP/DIP flexion에 coupling 제약 추가
  - Stereo camera 사용 시 개선 가능

#### Issue 4: Hand Size Normalization
- **문제**: MediaPipe는 실제 크기 출력하지만, 사람마다 손 크기 다름
- **해결**: Bone length ratio 기반 정규화 (bone 길이 비율은 개인차 작음)

#### Issue 5: Coordinate Frame Alignment
- **문제**: 각 소스의 좌표계가 다름
- **Manus SDK**: Configurable (Z-up, right-handed 기본)
- **MediaPipe**: Camera frame (y-down, z-into-screen for normalized; metric for world)
- **DG5F URDF**: Z-up (palm 방향), Y-forward (손가락 방향)
- **해결**: 각 소스에서 palm frame을 정의하고 DG5F palm frame으로 변환

### 8.3 Safety Considerations
1. **Joint limit clamping**: 모든 매핑 결과를 URDF joint limit 내로 clamp
2. **Velocity limiting**: 급격한 입력 변화 시 속도 제한 (3.14 rad/s max)
3. **Singularity avoidance**: Thumb opposition 근처에서 gimbal lock 주의
4. **Timeout handling**: 입력 소스 끊김 시 현재 자세 유지 (fail-safe)

---

## 9. 소스별 ROS2 Integration 요약

### 9.1 Manus (Skeleton + Ergonomics)

```python
# 동일 토픽에서 두 데이터 모두 수신
from manus_ros2.msg import ManusGlove

def callback(msg: ManusGlove):
    # Raw Skeleton
    for node in msg.raw_nodes:
        node.node_id, node.pose  # geometry_msgs/Pose

    # Ergonomics
    for ergo in msg.ergonomics:
        ergo.type   # "RightFingerIndexMCPStretch"
        ergo.value  # float32, degrees
```

**토픽**: `manus_glove_0` (ManusGlove type, 120Hz)

### 9.2 MediaPipe Hands

- ROS2 wrapper 없음 → 직접 구현 필요
- 권장 구현: Camera node → MediaPipe → `sensor_msgs/JointState` publish

```python
import mediapipe as mp
# ... camera capture ...
results = hands.process(frame)
if results.multi_hand_world_landmarks:
    landmarks = results.multi_hand_world_landmarks[0]
    # 21 landmarks, each with .x, .y, .z (meters)
```

### 9.3 DG5F Target

```python
from sensor_msgs.msg import JointState

msg = JointState()
msg.name = [
    'rj_dg_1_1', 'rj_dg_1_2', 'rj_dg_1_3', 'rj_dg_1_4',  # Thumb
    'rj_dg_2_1', 'rj_dg_2_2', 'rj_dg_2_3', 'rj_dg_2_4',  # Index
    'rj_dg_3_1', 'rj_dg_3_2', 'rj_dg_3_3', 'rj_dg_3_4',  # Middle
    'rj_dg_4_1', 'rj_dg_4_2', 'rj_dg_4_3', 'rj_dg_4_4',  # Ring
    'rj_dg_5_1', 'rj_dg_5_2', 'rj_dg_5_3', 'rj_dg_5_4',  # Pinky
]
msg.position = [...]  # 20 floats, radians
```

---

## Appendix A: Joint Name Quick Reference

```
DG5F Joint Names (20 revolute):
  Thumb:  rj_dg_1_1, rj_dg_1_2, rj_dg_1_3, rj_dg_1_4
  Index:  rj_dg_2_1, rj_dg_2_2, rj_dg_2_3, rj_dg_2_4
  Middle: rj_dg_3_1, rj_dg_3_2, rj_dg_3_3, rj_dg_3_4
  Ring:   rj_dg_4_1, rj_dg_4_2, rj_dg_4_3, rj_dg_4_4
  Pinky:  rj_dg_5_1, rj_dg_5_2, rj_dg_5_3, rj_dg_5_4

Manus Ergonomics Names (20 per hand, Right):
  Thumb:  ThumbMCPSpread, ThumbMCPStretch, ThumbPIPStretch, ThumbDIPStretch
  Index:  IndexMCPSpread, IndexMCPStretch, IndexPIPStretch, IndexDIPStretch
  Middle: MiddleMCPSpread, MiddleMCPStretch, MiddlePIPStretch, MiddleDIPStretch
  Ring:   RingMCPSpread, RingMCPStretch, RingPIPStretch, RingDIPStretch
  Pinky:  PinkyMCPSpread, PinkyMCPStretch, PinkyPIPStretch, PinkyDIPStretch

MediaPipe Landmark Indices (21):
  Wrist: 0
  Thumb: 1,2,3,4    Index: 5,6,7,8    Middle: 9,10,11,12
  Ring: 13,14,15,16  Pinky: 17,18,19,20

Manus Skeleton Node IDs (25):
  Wrist: 0
  Thumb: 1,2,3,4    Index: 5,6,7,8,9   Middle: 10,11,12,13,14
  Ring: 15,16,17,18,19  Pinky: 20,21,22,23,24
```
