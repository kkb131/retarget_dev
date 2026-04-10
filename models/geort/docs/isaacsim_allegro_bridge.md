# Isaac Sim ↔ GeoRT Allegro bridge

[`scripts/isaacsim_replay.py`](../scripts/isaacsim_replay.py) 가 publish 하는
`JointState` 를 Isaac Sim 측이 정상적으로 받아들이려면 **두 가지** 가
일치해야 한다: ① topic 이름, ② joint 이름 순서.

## 1. Topic
- 기본값: `/allegro_right/joint_commands`
- 메시지: `sensor_msgs/msg/JointState`
- QoS: rclpy 의 기본 (depth=10)

다른 topic 으로 publish 하려면 `isaacsim_replay.py --topic /your/topic`.

## 2. joint name 순서

GeoRT 측은 학습 시 저장된 `checkpoint/<tag>/config.json` 의 `joint_order`
필드를 그대로 publish 한다. Allegro right 의 경우:

```
joint_0.0, joint_1.0, joint_2.0, joint_3.0,    # index
joint_4.0, joint_5.0, joint_6.0, joint_7.0,    # middle
joint_8.0, joint_9.0, joint_10.0, joint_11.0,  # ring
joint_12.0, joint_13.0, joint_14.0, joint_15.0 # thumb
```

(`models/geort/GeoRT/geort/config/allegro_right.json` 참조 — 4-fingered 핸드
이며 새끼손가락은 없음.)

Isaac Sim 의 Allegro USD 가 위와 정확히 같은 이름을 갖는지 확인:

```bash
ros2 topic echo /allegro_right/joint_states sensor_msgs/msg/JointState --once
```

`name:` 필드의 값이 GeoRT 의 `joint_order` 와 같으면 그대로 publish 하면
동작. 다르면 §3 의 매핑 적용.

## 3. 이름이 다를 때 처리

현재 `isaacsim_replay.py` 는 매핑 옵션을 노출하지 않는다. Phase 0 첫 시도에
서 불일치가 확인되면 다음 중 하나로 해결:

### 옵션 A — Isaac Sim 측 USD 의 joint name 을 변경
가장 청결하지만 USD 편집이 필요. `omni.isaac.core.articulations` 의
`get_joint_names()` 결과를 GeoRT 와 일치하도록 수정.

### 옵션 B — Isaac Sim 의 ROS2 controller 에 alias 등록
Isaac Sim 의 `JointState Subscriber` 노드가 incoming `name` 필드를 무시하고
순서만 사용하도록 설정 (보통 `usePathTracking=False`). 16개의 joint 가
**같은 순서** 로 정렬되어 있다면 이름이 달라도 동작.

### 옵션 C — `isaacsim_replay.py` 에 매핑 추가 (TBD)
1차 시도 결과를 보고 옵션 A/B 모두 비현실적이면 본 스크립트에 다음과 같은
플래그를 추가:
```
--rename-map "joint_0.0=allegro_index_joint_0,..."
```

## 4. 모터 제어 모드

Isaac Sim 의 Allegro 가 position 모드로 구동되어야 한다 (`JointState.position`
필드를 사용). velocity / effort 모드라면 publish 해도 무시됨.

USD 의 `physxArticulation` 또는 `JointStateController` 설정을 확인.

## 5. 디버깅

### 5.1 Replay 가 publish 하는데 hand 가 안 움직임
1. `ros2 topic hz /allegro_right/joint_commands` — 30Hz 로 들어오는지 확인
2. `ros2 topic echo /allegro_right/joint_commands --once` — name/position
   필드가 채워졌는지
3. Isaac Sim 의 ROS2 bridge extension 활성화 여부
4. Isaac Sim 측 controller 의 input topic 이 정확한지

### 5.2 일부 손가락만 동작
joint name 매핑 중 일부만 매치된 경우. ros2 topic echo 결과의 `name` 과
Isaac Sim 의 articulation joint 이름을 모두 비교.

### 5.3 Hand 가 폭주 (jitter / NaN)
- `--hz` 를 낮춰서 (예: 10) 데이터가 쌓이는지 확인
- mocap 데이터에 NaN 이 있는지: `np.any(np.isnan(np.load('data/human_alex.npy')))`
- 학습 epoch 가 너무 적었는지 — `epoch_X.pth` 중간 epoch 로 비교 (`--epoch 50`)
