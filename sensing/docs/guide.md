# Hand Sensing 모듈 가이드

## 1. 개요

`retarget_dev/sensing/`은 다양한 입력 장치에서 **사람 손의 21개 3D 키포인트**를 실시간으로 추출하는 모듈이다.
추출된 키포인트는 AnyTeleop(dex-retargeting)의 입력으로 사용되어 DG-5F 핸드 리타게팅에 활용된다.

```
입력 장치 → [Sensing 모듈] → HandKeypoints (21, 3) → [AnyTeleop] → DG-5F 관절각
```

현재 지원하는 입력 장치:

| 모듈 | 입력 장치 | 3D 방식 | 정확도 |
|------|----------|---------|--------|
| `phone/` | Android 폰 카메라 | MediaPipe 추정 z | 보통 (~5-15mm z 오차) |
| `realsense/` | Intel RealSense D405 | 실제 depth | 높음 (sub-mm) |
| `manus/` | Manus Quantum 글러브 | SDK 스켈레톤 | 높음 (직접 측정) |

---

## 2. 아키텍처

### 디렉토리 구조

```
sensing/
├── common.py              # HandKeypoints 데이터클래스 + SensingSource ABC
├── core/                  # 공유 컴포넌트
│   ├── hand_detector.py   #   MediaPipe HandLandmarker (phone, realsense 공유)
│   ├── visualizer.py      #   OpenCV 디버그 시각화 (phone, realsense 공유)
│   └── models/
│       └── hand_landmarker.task  # MediaPipe 모델 파일 (번들)
├── phone/                 # 폰 카메라 모듈
│   ├── camera_source.py   #   Threaded 카메라 캡처 (MJPEG/RTSP/USB)
│   ├── keypoint_converter.py  # world_landmarks → wrist-frame (21,3)
│   ├── phone_sensing.py   #   SensingSource 구현
│   └── main.py
├── realsense/             # RealSense D405 모듈
│   ├── rs_camera.py       #   pyrealsense2 파이프라인 (color+depth)
│   ├── depth_keypoint_converter.py  # 2D + real depth → 3D
│   ├── realsense_sensing.py  # SensingSource 구현
│   └── main.py
├── manus/                 # Manus 글러브 모듈
│   ├── manus_hand_data.py #   HandData 데이터클래스
│   ├── mock_provider.py   #   Mock 데이터 (테스트용)
│   ├── sdk_provider.py    #   SDK subprocess 연동
│   ├── ros2_provider.py   #   ROS2 토픽 구독 (/manus_glove_*)
│   ├── manus_sensing.py   #   SensingSource 구현
│   ├── main.py
│   └── sdk/               #   Manus SDK 소스 (독립 복사)
└── docs/
    └── guide.md           # 이 문서
```

### 공통 인터페이스

모든 sensing 모듈은 동일한 `SensingSource` ABC를 구현한다:

```python
from retarget_dev.sensing.common import SensingSource, HandKeypoints

class MySensing(SensingSource):
    def start(self) -> None: ...       # 하드웨어 초기화
    def stop(self) -> None: ...        # 리소스 해제
    def get_keypoints(self) -> HandKeypoints | None: ...  # 최신 키포인트
    def get_frame(self) -> np.ndarray | None: ...         # 최신 카메라 프레임 (없으면 None)
```

context manager 지원: `with MySensing() as s: ...`

---

## 3. Phone 모듈

### 의존성 설치

```bash
pip install mediapipe opencv-python numpy
```

> MediaPipe 모델 파일(`hand_landmarker.task`)은 `core/models/`에 번들되어 있어 네트워크 불필요.

### 실행

```bash
cd /workspaces/tamp_ws/src

# 로컬 웹캠 (테스트)
python3 -m retarget_dev.sensing.phone.main

# Android 폰 (IP Webcam 앱)
python3 -m retarget_dev.sensing.phone.main --url http://<폰IP>:8080/video

# 전체 옵션
python3 -m retarget_dev.sensing.phone.main \
    --url http://192.168.0.100:8080/video \
    --hand right \
    --mirror \
    --save keypoints.npy \
    --no-viz
```

### Android IP Webcam 설정

1. Play Store에서 "IP Webcam" 앱 설치
2. 앱 실행 → 하단 "Start server" 터치
3. 화면에 표시된 URL 확인 (예: `http://192.168.0.100:8080`)
4. `--url http://192.168.0.100:8080/video` 로 실행

### 옵션 설명

| 옵션 | 설명 | 기본값 |
|------|------|--------|
| `--url` | 카메라 소스 (디바이스 번호, URL, 파일 경로) | `0` |
| `--hand` | 추적할 손 (`left` / `right`) | `right` |
| `--mirror` | 프레임 좌우 반전 (전면 카메라용) | 미사용 |
| `--resolution` | 캡처 해상도 `WxH` | `640x480` |
| `--save` | 키포인트 저장 경로 (`.npy`) | 미사용 |
| `--no-viz` | 시각화 창 비활성화 | 미사용 |
| `--model` | MediaPipe 모델 경로 | 번들 모델 |

### 조작

- **ESC** — 종료

---

## 4. RealSense D405 모듈

### 의존성 설치

```bash
pip install pyrealsense2 mediapipe opencv-python "numpy<2"
```

### 실행

```bash
cd /workspaces/tamp_ws/src

# D405 연결 후
python3 -m retarget_dev.sensing.realsense.main --hand right

# 전체 옵션
python3 -m retarget_dev.sensing.realsense.main \
    --hand right \
    --serial <시리얼번호> \
    --resolution 640x480 \
    --save keypoints.npy \
    --no-viz
```

### Phone vs RealSense 비교

| | Phone | RealSense D405 |
|---|---|---|
| 3D depth 소스 | MediaPipe 추정 z (~5-15mm 오차) | **실제 depth** (sub-mm) |
| 변환 방식 | `world_landmarks - wrist` | `rs2_deproject_pixel_to_point(intrinsics, pixel, depth)` |
| depth 실패 시 | 없음 (항상 추정) | MediaPipe world_landmarks로 fallback |
| 적합한 용도 | 빠른 프로토타이핑 | 정밀 리타게팅 |

### 옵션 설명

| 옵션 | 설명 | 기본값 |
|------|------|--------|
| `--hand` | 추적할 손 | `right` |
| `--serial` | RealSense 시리얼 번호 (복수 카메라 시) | 자동 감지 |
| `--resolution` | 캡처 해상도 `WxH` | `640x480` |
| `--save` | 키포인트 저장 경로 | 미사용 |
| `--no-viz` | 시각화 비활성화 | 미사용 |

---

## 5. Manus 글러브 모듈

### ROS2 모드 (권장)

manus_data_publisher C++ 노드가 발행하는 `/manus_glove_*` 토픽을 구독.

```bash
# 터미널 1: manus_data_publisher 실행
ros2 run manus_ros2 manus_data_publisher

# 터미널 2: 키포인트 수신
cd /workspaces/tamp_ws/src
python3 -m retarget_dev.sensing.manus.main --ros2 --hand right
```

**ROS2 토픽 구조:**

| 토픽 | 메시지 타입 | 설명 |
|------|-----------|------|
| `/manus_glove_0` | `manus_ros2_msgs/ManusGlove` | 첫 번째 글러브 |
| `/manus_glove_1` | `manus_ros2_msgs/ManusGlove` | 두 번째 글러브 |
| `/manus_glove_{N}` | 상동 | 추가 글러브 (최대 4개) |

**ManusGlove 메시지 필드:**
- `side`: "Left" 또는 "Right"
- `raw_nodes[]`: 21개 스켈레톤 노드 (position + quaternion)
- `ergonomics[]`: 20개 관절각 (type string + value in degrees)

**Ergonomics type string 매핑:**
```
ThumbMCPSpread(0), ThumbMCPStretch(1), ThumbPIPStretch(2), ThumbDIPStretch(3)
IndexSpread(4), IndexMCPStretch(5), IndexPIPStretch(6), IndexDIPStretch(7)
MiddleSpread(8), MiddleMCPStretch(9), MiddlePIPStretch(10), MiddleDIPStretch(11)
RingSpread(12), RingMCPStretch(13), RingPIPStretch(14), RingDIPStretch(15)
PinkySpread(16), PinkyMCPStretch(17), PinkyPIPStretch(18), PinkyDIPStretch(19)
```

**필요 패키지:**
```bash
# manus_ros2 + manus_ros2_msgs가 빌드 및 소싱되어 있어야 함
source ~/manus_ws/install/setup.bash
```

### Direct Mapping과 연동

```bash
# ROS2 Manus → Direct Mapping → DG5F (IsaacSim)
python3 -m retarget_dev.models.direct_mapping.main \
    --sensing manus-ros2 --hand right

# 2-point calibration 포함
python3 -m retarget_dev.models.direct_mapping.main \
    --sensing manus-ros2 --calibrate --save-config my_config.yaml
```

### Mock 모드 (SDK/하드웨어 없이 테스트)

```bash
cd /workspaces/tamp_ws/src

python3 -m retarget_dev.sensing.manus.main --mock --hand right
```

사인파 기반 가짜 스켈레톤 데이터를 생성하여 파이프라인 테스트 가능.

### SDK 모드 (subprocess, 레거시)

```bash
# 1. SDK 바이너리 빌드 (최초 1회)
cd /workspaces/tamp_ws/src/retarget_dev/sensing/manus/sdk
make

# 2. 실행
cd /workspaces/tamp_ws/src
python3 -m retarget_dev.sensing.manus.main \
    --sdk-bin retarget_dev/sensing/manus/sdk/SDKClient_Linux.out \
    --hand right
```

### 옵션 설명

| 옵션 | 설명 | 기본값 |
|------|------|--------|
| `--ros2` | ROS2 모드 (/manus_glove_* 구독) | — |
| `--mock` | Mock 모드 (사인파 데이터) | — |
| `--sdk-bin` | SDK 바이너리 경로 (레거시) | — |
| `--hand` | 추적할 손 | `right` |
| `--hz` | 폴링 주기 (Hz) | `60` |
| `--save` | 키포인트 저장 경로 | 미사용 |

> `--ros2`, `--mock`, `--sdk-bin`은 상호 배타적 (셋 중 하나 필수).

---

## 6. 공통 출력 포맷: HandKeypoints

모든 sensing 모듈의 출력은 동일한 `HandKeypoints` 데이터클래스:

```python
@dataclass
class HandKeypoints:
    keypoints_3d: np.ndarray      # (21, 3) float32, 미터, wrist 원점
    keypoints_2d: np.ndarray | None  # (21, 2) 정규화 이미지 좌표 (시각화용)
    handedness: str               # "left" 또는 "right"
    confidence: float             # 탐지 신뢰도 [0, 1]
    timestamp: float              # time.time()
    source: str                   # "phone", "realsense", "manus"
```

### 21개 키포인트 인덱스 (MediaPipe 표준)

```
 0: WRIST            ← 항상 [0, 0, 0] (원점)
 1: THUMB_CMC         5: INDEX_MCP        9: MIDDLE_MCP      13: RING_MCP       17: PINKY_MCP
 2: THUMB_MCP         6: INDEX_PIP       10: MIDDLE_PIP      14: RING_PIP       18: PINKY_PIP
 3: THUMB_IP          7: INDEX_DIP       11: MIDDLE_DIP      15: RING_DIP       19: PINKY_DIP
 4: THUMB_TIP ★      8: INDEX_TIP ★     12: MIDDLE_TIP ★   16: RING_TIP ★     20: PINKY_TIP ★
```

★ = fingertip (인덱스: `[4, 8, 12, 16, 20]`)

### AnyTeleop(dex-retargeting) 연동

dex-retargeting의 `SeqRetargeting.retarget(ref_value)`에 `keypoints_3d`를 직접 전달:

```python
# Vector retargeting의 경우:
ref_value = keypoints_3d[task_indices] - keypoints_3d[origin_indices]
```

---

## 7. 새 Sensing Source 추가 방법

새로운 입력 장치(예: Leap Motion, VR 컨트롤러 등)를 추가하려면:

### 1단계: 디렉토리 생성

```
sensing/
└── my_device/
    ├── __init__.py
    ├── my_device_sensing.py
    └── main.py
```

### 2단계: SensingSource 구현

```python
from retarget_dev.sensing.common import HandKeypoints, SensingSource

class MyDeviceSensing(SensingSource):
    def start(self) -> None:
        # 장치 연결, 초기화

    def stop(self) -> None:
        # 장치 해제

    def get_keypoints(self) -> HandKeypoints | None:
        # 장치에서 데이터 읽기
        # → (21, 3) keypoints 변환 (wrist 원점)
        # → HandKeypoints 반환
        keypoints_3d = ...  # np.ndarray (21, 3), float32, meters
        keypoints_3d -= keypoints_3d[0]  # wrist 원점
        return HandKeypoints(
            keypoints_3d=keypoints_3d,
            handedness="right",
            confidence=1.0,
            source="my_device",
        )

    def get_frame(self) -> np.ndarray | None:
        # 카메라 프레임이 있으면 반환, 없으면 None
        return None
```

### 3단계: main.py 작성

`phone/main.py`를 참고하여 CLI 진입점 작성.
`core/visualizer.py`의 `HandVisualizer`를 재사용하면 시각화도 바로 가능.

### 핵심 규칙

- 출력은 반드시 `HandKeypoints(keypoints_3d=(21, 3), wrist=[0,0,0])`
- 키포인트 순서는 MediaPipe 21-point 표준을 따름
- `source` 필드에 장치 이름 기입
- MediaPipe가 필요하면 `core/hand_detector.py`를 import하여 재사용

---

## 8. 트러블슈팅

### MediaPipe 모델 파일 없음

```
FileNotFoundError: Bundled model not found at .../core/models/hand_landmarker.task
```

→ 모델 파일이 누락된 경우. Google에서 다운로드:
```bash
wget -O src/retarget_dev/sensing/core/models/hand_landmarker.task \
  "https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/latest/hand_landmarker.task"
```

### numpy 버전 충돌

mediapipe가 numpy 2.x를 원하지만, pinocchio/ROS Humble은 numpy 1.x 필요.
→ `pip install "numpy<2"` 로 1.x 유지. mediapipe는 numpy 1.x에서도 정상 동작.

### Phone 스트리밍 지연

카메라 스트리밍은 정상이지만 화면이 느린 경우:
→ MediaPipe detection이 tracking 실패 시 full detection으로 fallback (~50-100ms).
detection은 별도 스레드에서 실행되므로 화면 자체는 끊기지 않음.
OSD의 `Det: XXms` 값으로 detection 지연 모니터링 가능.

### RealSense 연결 안 됨

```
RuntimeError: Cannot open camera source
```

→ D405가 USB에 연결되어 있는지 확인. `rs-enumerate-devices` 명령으로 장치 목록 확인.
Docker 환경에서는 `--device` 옵션으로 USB 장치를 컨테이너에 전달해야 함.
