# RealSense D405 실시간 Retargeting 가이드

Intel RealSense D405를 사용하여 DG-5F (Isaac Sim)를 실시간 제어.

```
RealSense D405
    ↓ Color + aligned Depth
CameraSource (pyrealsense2 pipeline)
    ↓
MediaPipe HandLandmarker (color stream)
    ↓ 21 landmarks_2d
DepthKeypointConverter
    ↓ rs2_deproject_pixel_to_point (real depth)
    ↓ MANO transform (same as phone)
HandKeypoints (21, 3) in MANO frame
    ↓
DexRetargetModel → 20 DOF → ROS2 → Isaac Sim DG-5F
```

## 1. D405 하드웨어 특성

| 항목 | 값 |
|---|---|
| Depth range | 7cm ~ 50cm (권장 20~30cm) |
| 정밀도 | sub-mm (0.1mm @ 20cm) |
| 해상도 | 1280×720 (depth + color) |
| FPS | 90 (depth), 30 (color) |
| 인터페이스 | USB 3.1 Type-C |
| 용도 | 근거리 정밀 3D 캡처 |

**Phone vs D405:**

| | Phone (MediaPipe) | RealSense D405 |
|---|---|---|
| Z 축 | MediaPipe 추정 (~5-15mm 오차) | **실제 depth** (sub-mm) |
| 안정성 | 카메라 시점에 의존 | 카메라 시점 무관 |
| 지연 | 네트워크 의존 (~100-150ms) | USB 직결 (~30-50ms) |
| 설치 | 쉬움 (앱만 설치) | 드라이버 + USB 3.0 필요 |
| 용도 | 빠른 프로토타이핑 | 정밀 리타게팅 |

## 2. 설치

### 2-1. pyrealsense2

```bash
pip install pyrealsense2
```

### 2-2. librealsense SDK (드라이버)

Ubuntu 22.04:
```bash
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo jammy main" -u
sudo apt install librealsense2-utils librealsense2-dev
```

### 2-3. 확인

```bash
# 카메라 연결 확인
rs-enumerate-devices

# 또는 Python에서
python3 -c "
import pyrealsense2 as rs
ctx = rs.context()
for d in ctx.devices:
    print(d.get_info(rs.camera_info.name), d.get_info(rs.camera_info.serial_number))
"
```

D405가 인식되어야 합니다 (serial number 출력).

## 3. 실행

```bash
cd /workspaces/tamp_ws/src

# DexPilot (권장)
python3 -m retarget_dev.models.dex_retarget.main \
  --sensing realsense \
  --config retarget_dev/models/dex_retarget/config/dg5f_right_dexpilot.yml \
  --topic /dg5f_right/joint_commands \
  --hz 30

# Vector
python3 -m retarget_dev.models.dex_retarget.main \
  --sensing realsense \
  --config retarget_dev/models/dex_retarget/config/dg5f_right_vector.yml

# 특정 시리얼 지정 (여러 D405 연결 시)
python3 -m retarget_dev.models.dex_retarget.main \
  --sensing realsense \
  --rs-serial 123456789012 \
  --config retarget_dev/models/dex_retarget/config/dg5f_right_dexpilot.yml
```

Isaac Sim에서 DG-5F 손가락이 실시간으로 사람 손 동작을 따라 하면 성공.

## 4. 파이프라인 구조

### 4-1. 센싱 계층 (`retarget_dev/sensing/realsense/`)

```
RSCamera            ← pyrealsense2 pipeline (color + aligned depth)
    ↓ (color, depth_m) numpy arrays
HandDetector        ← MediaPipe HandLandmarker (VIDEO mode, color입력)
    ↓ landmarks_2d + world_landmarks
DepthKeypointConverter
    ↓ 1) rs2_deproject_pixel_to_point (2D + real depth → 3D)
    ↓ 2) wrist 원점 이동
    ↓ 3) apply_mano_transform (공통 모듈)
HandKeypoints (21, 3) in MANO frame
```

### 4-2. Phone과의 차이

| 단계 | Phone | RealSense D405 |
|---|---|---|
| 2D 검출 | MediaPipe (color만) | MediaPipe (color만) |
| 3D 복원 | MediaPipe `world_landmarks` (추정 z) | **`rs2_deproject_pixel_to_point` + real depth** |
| 원점 이동 | wrist → origin | wrist → origin |
| MANO 변환 | `apply_mano_transform()` | `apply_mano_transform()` ← **동일** |

둘 다 `sensing/core/mano_transform.py`의 공통 함수를 사용하므로 **dex-retargeting 호환성이 보장**됩니다.

## 5. 공통 MANO 변환 모듈

`sensing/core/mano_transform.py`:

```python
from retarget_dev.sensing.core.mano_transform import apply_mano_transform

# 사용:
kp_mano = apply_mano_transform(kp_wrist_centered, hand_type="right")
```

제공하는 함수/상수:
- `estimate_wrist_frame(kp)` — 팜 평면 SVD로 wrist 프레임 추정
- `apply_mano_transform(kp, hand_type)` — 전체 변환 적용
- `OPERATOR2MANO_RIGHT`, `OPERATOR2MANO_LEFT` — 고정 회전 상수

dex-retargeting 레퍼런스(`single_hand_detector.py`)와 **max diff 0.0000mm** 완전 일치 검증됨.

## 6. 트러블슈팅

### 6-1. `No Intel RealSense device detected`

```bash
# USB 연결 확인
lsusb | grep Intel

# D405는 USB 3.0 필요
lsusb -t | grep -A2 Intel
# speed: 5000M 또는 10000M이어야 함 (USB 2.0 = 480M이면 동작 안 함)
```

Docker 환경:
```bash
# 컨테이너 실행 시 USB 디바이스 전달
docker run --privileged --device=/dev/bus/usb:/dev/bus/usb ...
```

### 6-2. `pyrealsense2` import 에러

```bash
pip install --upgrade pyrealsense2
# 또는
pip install pyrealsense2==2.55.1.6486
```

### 6-3. Depth가 대부분 무효값

- D405 최소 거리 **7cm** 미만에 손이 있으면 무효
- 손이 너무 멀면 (>50cm) 노이즈 증가
- 권장 거리: **20~30cm**

### 6-4. FPS가 낮음

- 해상도 축소: `sensing/realsense/config.py`에서 `RS_WIDTH=640, RS_HEIGHT=480`
- MediaPipe가 병목일 수 있음 (D405 raw는 90 FPS)

### 6-5. Isaac Sim DG-5F가 움직이지 않음

- ROS2 토픽 확인: `ros2 topic hz /dg5f_right/joint_commands`
- hand detection 확인: 콘솔에 "Hand not detected" 로그 없어야 함
- config URDF가 `dg5f_right_retarget.urdf` (joint limit 수정본)인지 확인

## 7. 성능 참고치

- RealSense 캡처: ~10ms (color+depth aligned @ 640×480)
- MediaPipe detection: ~15-20ms
- DepthKeypointConverter (deproject + MANO): ~2-3ms
- dex-retargeting DexPilot: ~5-10ms
- **Total**: ~30-40ms/frame → 안정적 30Hz

## 8. Phone과 동시 비교

동일한 config로 둘 다 실행하여 결과 비교:

```bash
# 터미널 1: RealSense
python3 -m retarget_dev.models.dex_retarget.main \
  --sensing realsense \
  --config retarget_dev/models/dex_retarget/config/dg5f_right_dexpilot.yml \
  --topic /dg5f_right/joint_commands

# 터미널 2: Phone (별도 DG-5F 인스턴스가 있을 때)
python3 -m retarget_dev.models.dex_retarget.main \
  --sensing phone \
  --config retarget_dev/models/dex_retarget/config/dg5f_right_dexpilot.yml \
  --url http://<phone-ip>:8080/video \
  --topic /dg5f_right_phone/joint_commands
```

RealSense가 손의 절대 3D 위치를 정확하게 알기 때문에:
- Pinch 동작이 더 정확
- 카메라 각도가 기울어져도 안정적
- 손가락 개별 움직임 탐지 우수
