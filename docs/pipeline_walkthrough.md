# Pipeline Walkthrough — Phone/RealSense → DG-5F

**학습 목적**: 카메라 픽셀에서 시작해 ROS2 `JointState` 가 publish 되기까지, 데이터가
어떤 변환들을 거치는지 한 번에 따라가며 이해한다.

이 문서는 "무엇이 일어나는가" 보다 "**왜** 이 단계가 존재하는가" 에 초점을 둔다.
각 섹션은 실제 소스 코드의 핵심 5–15줄을 verbatim 으로 인용하며, 모든 인용에
`file#L1-L10` 형태의 링크를 달아 학습자가 바로 소스로 점프할 수 있게 한다.

---

## 목차

1. [개요 — 한 장의 다이어그램](#1-개요--한-장의-다이어그램)
2. [공통 인터페이스: HandKeypoints 와 SensingSource](#2-공통-인터페이스-handkeypoints-와-sensingsource)
3. [좌표계 입문 — 4가지 frame 의 차이](#3-좌표계-입문--4가지-frame-의-차이)
4. [Phone 경로 walkthrough (4단계)](#4-phone-경로-walkthrough)
5. [RealSense D405 경로 (Phone 과 다른 부분)](#5-realsense-d405-경로)
6. [MANO 변환 deep-dive](#6-mano-변환-deep-dive)
7. [DexRetargetModel 래퍼 — HandKeypoints → q](#7-dexretargetmodel-래퍼)
8. [dex-retargeting 라이브러리 내부 (학습용 minimum)](#8-dex-retargeting-라이브러리-내부)
9. [ROS2 publish 루프](#9-ros2-publish-루프)
10. [다른 입력 소스와의 비교](#10-다른-입력-소스와의-비교)
11. [부록: 학습자용 reading order](#11-부록-학습자용-reading-order)

---

## 1. 개요 — 한 장의 다이어그램

```
[Camera URL or RealSense D405]
        │ BGR (H, W, 3) uint8 — pixel space
        ▼
[CameraSource / RSCamera]                   ← thread 1: grab
        │ BGR frame (+ depth_m for D405)
        ▼
[HandDetector — MediaPipe HandLandmarker]   ← thread 2: detect
        │ HandDetection { landmarks_2d (21,2),
        │                 world_landmarks (21,3) ← meters, geometric-center frame }
        ▼
[KeypointConverter / DepthKeypointConverter]
        │ ① wrist 원점화
        │ ② apply_mano_transform (공통 모듈)
        ▼
[HandKeypoints]                             ← ★ 시스템의 lingua franca
        │ keypoints_3d (21, 3) — meters, wrist origin, MANO frame
        ▼
[DexRetargetModel]                          ← thread 3: main loop
        │ ① _build_ref_value (target_link_human_indices 사용)
        │ ② SeqRetargeting.retarget (NLOPT + warm start)
        │ ③ LPFilter (EMA)
        ▼
q (20,) radians — DG-5F joint angles
        │
        ▼
[main.py 루프]
        │ sensor_msgs/JointState { name=joint_names, position=q }
        ▼
[ROS2 topic /dg5f_right/joint_commands]
        │
        ▼
[Isaac Sim DG-5F]
```

**핵심 mental model**: 각 상자는 **책임 단위**, 각 화살표는 **변환**이다. 변환이 일어날
때마다 데이터의 (shape, 단위, 좌표계) 가 바뀌므로, 파이프라인을 읽을 때 "지금 이 시점의
데이터가 어느 frame 에 있는가?" 를 계속 추적하면 혼란이 없다.

---

## 2. 공통 인터페이스: HandKeypoints 와 SensingSource

파이프라인을 이해하는 가장 빠른 길은 **모든 sensing source 가 수렴하는 공통 타입**부터
읽는 것이다. Phone, RealSense, Manus, Mock — 넷 다 아래 하나의 클래스만 출력한다.

[`sensing/common.py`](../sensing/common.py#L63-L80):

```python
@dataclass
class HandKeypoints:
    """Unified hand keypoint output — common across all sensing sources."""
    keypoints_3d: np.ndarray                   # (21, 3)
    keypoints_2d: Optional[np.ndarray] = None  # (21, 2)
    handedness: str = "right"
    confidence: float = 0.0
    timestamp: float = field(default_factory=time.time)
    source: str = "unknown"
```

필드 의미:

| 필드 | 내용 |
|---|---|
| `keypoints_3d` | **(21, 3) float32, meters, wrist 가 [0,0,0], MANO frame**. 이 한 줄이 전체 시스템의 lingua franca |
| `keypoints_2d` | 이미지 좌표 [0..1] normalized — 시각화 용도, 필수 아님 |
| `handedness` | `"left"` / `"right"` — 어느 손인지 |
| `confidence` | 검출 신뢰도 [0, 1] |
| `timestamp` | `time.time()` — latency 측정용 |
| `source` | `"phone"`, `"realsense"`, `"manus"` 등 디버깅 용도 |

### SensingSource ABC

모든 sensing 구현체는 [`sensing/common.py:83-104`](../sensing/common.py#L83-L104) 의
추상 인터페이스를 따라야 한다:

```python
class SensingSource(ABC):
    """Abstract base for any hand sensing input."""

    @abstractmethod
    def start(self) -> None: ...
    @abstractmethod
    def stop(self) -> None: ...
    @abstractmethod
    def get_keypoints(self) -> Optional[HandKeypoints]: ...
    @abstractmethod
    def get_frame(self) -> Optional[np.ndarray]: ...
```

**왜 ABC 가 존재하는가**: Phone / RealSense / Manus 어떤 하드웨어가 들어와도 downstream
retargeting 은 이 4개 메서드만 호출하면 된다. "입력이 무엇인지 모르는 상태에서도
동일한 코드가 돈다" — 이게 전체 아키텍처의 계약이다.

`keypoints_3d` 가 항상 **(21, 3) MANO frame** 이어야 한다는 불변식은 각 sensing 구현체의
책임이다. 이 불변식을 깨면 downstream (dex-retargeting) 이 조용히 엉뚱한 결과를 낸다.
과거에 phone DexPilot 버그(§8.1)와 Manus 25→21 노드 버그(§10)가 둘 다 이 불변식을 깨서
발생했다.

MANO 표준 키포인트 인덱스 ([`sensing/common.py:18-35`](../sensing/common.py#L18-L35)):

```
0 : WRIST
1..4 : THUMB_CMC, THUMB_MCP, THUMB_IP, THUMB_TIP
5..8 : INDEX_MCP, INDEX_PIP, INDEX_DIP, INDEX_TIP
9..12: MIDDLE_MCP, MIDDLE_PIP, MIDDLE_DIP, MIDDLE_TIP
13..16: RING_MCP, RING_PIP, RING_DIP, RING_TIP
17..20: PINKY_MCP, PINKY_PIP, PINKY_DIP, PINKY_TIP
```

`FINGERTIP_INDICES = [4, 8, 12, 16, 20]` — 이 5개는 §8 에서 DexPilot 의 pinch 쌍 계산에
사용된다. 기억해 두자.

---

## 3. 좌표계 입문 — 4가지 frame 의 차이

파이프라인 전체에서 **4개의 서로 다른 좌표계**가 등장한다. 이 차이를 정확히 구분해야
§4–§6 이 이해된다.

| Frame | 원점 | 축 방향 | 어디서 등장하나 |
|---|---|---|---|
| **(A) Camera frame** | 카메라 렌즈 중심 | 카메라가 어떻게 놓였느냐에 따라 결정됨 | RGB 픽셀 / D405 raw depth |
| **(B) MediaPipe geometric center** | 21개 keypoint 의 평균 위치 | Camera frame 과 같은 회전 (축만 translate) | `world_landmarks` 출력 |
| **(C) Wrist-centered** | wrist (idx 0) | 아직 camera 회전 영향을 받음 | MANO 변환 직전 |
| **(D) MANO frame** | wrist | x = palm 평면 내 wrist→middle MCP, y = palm normal, z = x×y | dex-retargeting 입력 |

### 가장 중요한 통찰

> **Camera 가 30° 기울면 (A), (B), (C) frame 도 같이 30° 기운다.**
> **오직 (D) MANO frame 만이 "손 자체"에 고정되어 카메라 시점에 완전히 무관하다.**

dex-retargeting optimizer 는 (D) MANO frame 을 가정한다. 만약 (C) wrist-centered 상태에서
바로 optimizer 로 넘기면, 카메라가 기울 때마다 로봇 손이 같이 기운다. 사용자가 손을
가만히 두고 카메라만 돌려도 DG-5F 가 움직이는 버그가 생긴다. 이게 §6 MANO 변환이 존재하는
근본 이유이다.

변환 순서:
```
(A) Camera frame                    ← 카메라가 보는 세상
    │ wrist_frame_rotation (손마다 추정)
    ▼
wrist-aligned intermediate frame    ← 손바닥 평면 위의 자연스러운 축
    │ OPERATOR2MANO_RIGHT (고정 상수)
    ▼
(D) MANO frame                      ← dex-retargeting 이 기대하는 표준
```

(B) → (C) 는 단순 translation (wrist 위치만큼 빼기). (C) → (D) 는 rotation 2번 합성. §6 에서
자세히 본다.

---

## 4. Phone 경로 walkthrough

Phone (IP Webcam) 을 예로 들어 4단계를 순차적으로 읽는다. RealSense 는 §5 에서 차이만 다룬다.

### 4.1 HTTP MJPEG 캡처

**파일**: [`sensing/phone/camera_source.py:53-72`](../sensing/phone/camera_source.py#L53-L72)
**입력**: URL string (`"http://192.168.0.3:8080/video"`) 또는 device index
**출력**: BGR frame (H, W, 3) uint8 — 최신 프레임만 유지
**왜**: 네트워크 스트림(MJPEG/RTSP) 의 내부 버퍼가 쌓이면 latency 가 수 초까지 늘어난다.
OpenCV 의 `CAP_PROP_BUFFERSIZE=1` + 백그라운드 `_grab_loop` 로 항상 "가장 최근 프레임 하나"만
유지한다.

```python
def start(self) -> None:
    """Open the camera and start the grab thread."""
    source = self._parse_source(self._source_str)
    self._cap = cv2.VideoCapture(source)
    if not self._cap.isOpened():
        raise RuntimeError(f"Cannot open camera source: {self._source_str}")

    # Try to set resolution and fps
    w, h = self._resolution
    self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
    self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
    self._cap.set(cv2.CAP_PROP_FPS, self._target_fps)
    # Minimize buffer for network streams
    self._cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    self._running = True
    self._thread = threading.Thread(target=self._grab_loop, daemon=True)
    self._thread.start()
```

Grab 루프는 [`camera_source.py:108-126`](../sensing/phone/camera_source.py#L108-L126) — 단순히
`cap.grab() → cap.retrieve()` 를 무한 반복하며 최신 프레임만 `self._frame` 에 덮어쓴다. `read()`
를 호출하는 consumer 는 잠금 안에서 `frame.copy()` 로 안전하게 가져간다.

**이 시점의 데이터**: (A) Camera frame, pixel space, uint8 BGR.

### 4.2 MediaPipe HandLandmarker

**파일**: [`sensing/core/hand_detector.py:73-116`](../sensing/core/hand_detector.py#L73-L116)
**입력**: BGR frame + 타임스탬프 (ms)
**출력**: `list[HandDetection]` — 각 요소는 `landmarks_2d (21, 3)` + `world_landmarks (21, 3)`
**왜**: MediaPipe 는 "이 프레임에 손이 있나? 어느 위치에? 어떤 손?" 이라는 3가지를 한 번에
답한다. 우리 코드는 이 답에서 `world_landmarks` (3D) 만 실제 retargeting 에 사용한다.

```python
def detect(self, frame_bgr, timestamp_ms) -> list[HandDetection]:
    frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
    mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=frame_rgb)
    result = self._landmarker.detect_for_video(mp_image, timestamp_ms)

    detections: list[HandDetection] = []
    for i, landmarks in enumerate(result.hand_landmarks):
        # 2D normalized image coordinates
        lm_2d = np.array([[lm.x, lm.y, lm.z] for lm in landmarks], dtype=np.float32)
        # 3D world landmarks (meters)
        wl = result.hand_world_landmarks[i]
        lm_3d = np.array([[lm.x, lm.y, lm.z] for lm in wl], dtype=np.float32)
        h = result.handedness[i][0]
        detections.append(HandDetection(
            landmarks_2d=lm_2d,
            world_landmarks=lm_3d,
            handedness=h.category_name,
            handedness_score=h.score,
        ))
    return detections
```

### ★ landmarks_2d 와 world_landmarks 의 차이 (가장 헷갈리는 부분)

MediaPipe HandLandmarker 는 **두 가지 다른 출력**을 한 번에 반환한다:

| | landmarks_2d (= `hand_landmarks`) | world_landmarks |
|---|---|---|
| 좌표계 | 이미지 픽셀 [0..1] normalized | meters (실제 단위) |
| 원점 | 이미지 왼쪽 위 | 손의 **geometric center** (21개 점의 평균) |
| 축 | x=이미지 가로, y=이미지 세로, z=추정 상대 깊이 | 3D 카메라 frame |
| 주 용도 | 화면에 스켈레톤 그리기, ROI 계산 | 3D retargeting |

즉 `world_landmarks[0]` (wrist) 는 **[0, 0, 0] 이 아니다** — 손의 평균 위치 기준으로 떨어진
곳에 있다. 이 때문에 §4.3 에서 "wrist 원점화" 단계가 필요하다.

**Running mode = VIDEO**: [`hand_detector.py:62-70`](../sensing/core/hand_detector.py#L62-L70)
는 MediaPipe 를 `RunningMode.VIDEO` 로 생성하는데, 이 모드는 **연속된 프레임 사이의 tracking**
을 활용하여 프레임 간 안정성을 높인다. 대신 monotonically increasing `timestamp_ms` 를 반드시
넘겨야 하며, 뒤로 감는 타임스탬프를 주면 에러가 난다.

**이 시점의 데이터**: (B) MediaPipe geometric center frame, meters, 21개 점.

### 4.3 Wrist 원점화 + MANO 변환 호출

**파일**: [`sensing/phone/keypoint_converter.py:52-71`](../sensing/phone/keypoint_converter.py#L52-L71)
**입력**: `HandDetection.world_landmarks` (21, 3) meters, geometric center frame
**출력**: (21, 3) float32, meters, **wrist = [0, 0, 0]**, **MANO frame**
**왜**: dex-retargeting 은 (D) MANO frame 을 기대한다. 이 메서드가 (B) → (C) → (D) 변환을
한 번에 처리하는 gateway.

```python
def convert(self, detection: HandDetection) -> np.ndarray:
    """Convert a single HandDetection to MANO-frame keypoints."""
    wl = detection.world_landmarks.astype(np.float32)
    # Shift origin to wrist
    kp = wl - wl[self.WRIST_INDEX]           # ← (B) → (C): translation only

    if self._apply_mano:
        kp = apply_mano_transform(kp, hand_type=self._hand_type)  # ← (C) → (D)

    if self._extra_rotation is not None:
        kp = (self._extra_rotation @ kp.T).T

    return kp.astype(np.float32)
```

두 단계:

1. **`kp = wl - wl[0]`** — wrist 위치를 원점으로 이동. §4.2 에서 설명한 대로 `world_landmarks[0]`
   은 보통 [0, 0, 0] 이 아니므로 이 단계가 필수.
2. **`apply_mano_transform(kp, hand_type)`** — 손바닥 평면에 기반한 회전 변환. 자세한 수학은
   §6 에서 본다.

**이 시점의 데이터**: (D) MANO frame, (21, 3), wrist 가 [0, 0, 0].

### 4.4 PhoneSensing 비동기 wrapper

**파일**: [`sensing/phone/phone_sensing.py:119-151`](../sensing/phone/phone_sensing.py#L119-L151)
**입력**: 카메라 프레임 stream (async)
**출력**: `HandKeypoints` — `get_keypoints()` 호출 시 최신 결과만 반환
**왜**: Detection 이 프레임당 ~15ms 걸린다. main loop 에서 동기 호출하면 전체가 느려진다.
백그라운드 thread 에서 돌리고 main loop 는 latest 만 pull 한다.

```python
def _detect_loop(self) -> None:
    """Background thread: grab frame → detect → update latest results."""
    while self._running:
        ok, frame = self._camera.read()
        if not ok or frame is None:
            time.sleep(0.001)
            continue

        ts_ms = int(time.monotonic() * 1000) - self._start_time_ms

        t0 = time.perf_counter()
        detections = self._detector.detect(frame, ts_ms)
        det_ms = (time.perf_counter() - t0) * 1000
        self._detection_time_ms = det_ms

        keypoints: Optional[HandKeypoints] = None
        if detections:
            best = self._select_hand(detections)
            if best is not None:
                keypoints_3d = self._converter.convert(best)
                keypoints_2d = KeypointConverter.extract_2d(best)
                keypoints = HandKeypoints(
                    keypoints_3d=keypoints_3d,
                    keypoints_2d=keypoints_2d,
                    handedness=best.handedness.lower(),
                    confidence=best.handedness_score,
                    timestamp=time.time(),
                    source="phone",
                )

        with self._lock:
            self._latest_keypoints = keypoints
            self._latest_frame = frame
```

핵심 패턴:
- `self._camera.read()` → `detector.detect()` → `converter.convert()` → `HandKeypoints` 생성
- `self._lock` 안에서 최신 결과만 덮어씀 — consumer 는 `get_keypoints()` 로 lock 안에서 copy
- 검출 실패 시 `keypoints = None` 으로 덮어쓰지 않고 이전 값을 유지 (손이 frame 밖으로
  잠깐 나갔다 돌아올 때 유용)

`_select_hand()` 는 여러 손이 검출됐을 때 `handedness` 로 필터 + confidence 최댓값 선택
([`phone_sensing.py:155-167`](../sensing/phone/phone_sensing.py#L155-L167)).

**이 시점의 데이터**: `HandKeypoints` 객체. 여기서부터는 sensor 구분이 사라진다 — downstream
는 이게 어디서 왔는지 모른다.

---

## 5. RealSense D405 경로

Phone 경로와 **거의 같다**. 차이는 한 단계 뿐 — 2D landmarks + 실 depth 로 3D 복원을
더 정확하게 한다는 점.

### 5.1 pyrealsense2 pipeline

**파일**: [`sensing/realsense/rs_camera.py:36-70`](../sensing/realsense/rs_camera.py#L36-L70)
**입력**: serial number (선택), width/height/fps
**출력**: `(ok, color_bgr, depth_m)` 튜플 — color 는 uint8 (H,W,3), depth 는 float32 (H,W) meters
**왜**: D405 는 color 와 depth 두 stream 을 내보낸다. 이 둘을 같은 픽셀 좌표계에서 비교하려면
`rs.align` 으로 depth 를 color frame 에 맞춰야 한다.

```python
def start(self) -> None:
    """Open the RealSense pipeline and start streaming."""
    import pyrealsense2 as rs

    self._pipeline = rs.pipeline()
    config = rs.config()

    if self._serial:
        config.enable_device(self._serial)

    config.enable_stream(
        rs.stream.color, self._width, self._height, rs.format.bgr8, self._fps,
    )
    config.enable_stream(
        rs.stream.depth, self._width, self._height, rs.format.z16, self._fps,
    )

    profile = self._pipeline.start(config)

    # Depth scale (D405 default: 0.0001m per unit, i.e. 0.1mm)
    depth_sensor = profile.get_device().first_depth_sensor()
    self._depth_scale = depth_sensor.get_depth_scale()

    # Align depth → color coordinate frame
    self._align = rs.align(rs.stream.color)

    # Camera intrinsics for deprojection
    color_profile = profile.get_stream(rs.stream.color)
    self._intrinsics = color_profile.as_video_stream_profile().get_intrinsics()
```

핵심 4가지:

1. **`enable_stream(color)` + `enable_stream(depth)`**: 두 stream 을 함께 시작
2. **`depth_scale`**: D405 의 경우 0.0001 m/unit. raw depth 가 uint16 이므로 `depth_m =
   depth_raw * depth_scale` 로 meter 변환
3. **`rs.align(rs.stream.color)`**: depth 를 color 좌표계에 정렬. 정렬 전에는 color 픽셀 (px, py)
   에 해당하는 depth 픽셀이 약간 어긋나 있는데, 이 step 이 "같은 (px, py) 에서 color 와 depth 를
   동시에 읽을 수 있게" 맞춰준다
4. **`intrinsics`**: (fx, fy, ppx, ppy) — §5.2 의 deprojection 에 필요

### 5.2 Depth 기반 3D 복원 + MANO 변환

**파일**: [`sensing/realsense/depth_keypoint_converter.py:74-101`](../sensing/realsense/depth_keypoint_converter.py#L74-L101)
**입력**: `HandDetection.landmarks_2d` + depth_m + img_w + img_h + intrinsics
**출력**: (21, 3), meters, **wrist 원점**, **MANO frame** — phone 과 동일 형식
**왜**: MediaPipe 의 `world_landmarks[i].z` 는 단안 이미지에서 **추정된** 깊이로 오차가 크다
(특히 손가락이 카메라를 향할 때). D405 는 sub-mm 정밀도의 **실제** depth 를 제공하므로 이걸
그대로 사용하면 훨씬 안정적이다.

```python
pts_3d = np.zeros((21, 3), dtype=np.float32)

for i in range(21):
    nx, ny = detection.landmarks_2d[i, 0], detection.landmarks_2d[i, 1]
    px = int(nx * img_w)
    py = int(ny * img_h)

    # Clamp to image bounds
    px = max(0, min(px, img_w - 1))
    py = max(0, min(py, img_h - 1))

    d = self._sample_depth(depth_m, px, py)

    if DEPTH_MIN_M < d < DEPTH_MAX_M:
        # True 3D via camera intrinsics
        pts_3d[i] = rs.rs2_deproject_pixel_to_point(
            self._intrinsics, [float(px), float(py)], d,
        )
    else:
        # Fallback to MediaPipe's estimated world_landmarks
        pts_3d[i] = detection.world_landmarks[i]

# Shift to wrist origin
pts_3d -= pts_3d[self.WRIST_INDEX]

# Apply MANO transform (required by dex-retargeting)
if self._apply_mano:
    pts_3d = apply_mano_transform(pts_3d, hand_type=self._hand_type)
```

알아야 할 4가지:

1. **픽셀 좌표 변환**: MediaPipe 는 `[0..1]` normalized 2D 를 주므로 `px = nx * img_w` 로 픽셀화
2. **`rs2_deproject_pixel_to_point(intrinsics, [px, py], d)`**: pyrealsense2 가 제공하는 표준 함수.
   (pixel, depth) → 3D point in camera frame 을 intrinsics 로 역투영. 수식:
   ```
   x = (px - ppx) / fx * d
   y = (py - ppy) / fy * d
   z = d
   ```
3. **Fallback**: depth 가 무효일 때 (hole, out-of-range) `detection.world_landmarks[i]` 로 대체.
   "가려져서 depth 가 없을 때 MediaPipe 의 추정값으로 돌아가는 것" — 완전히 fail 하지 않는
   graceful degradation. `DEPTH_MIN_M` = 0.02m, `DEPTH_MAX_M` = 1.0m
   ([`sensing/realsense/config.py`](../sensing/realsense/config.py))
4. **공통 MANO 변환**: phone 과 **같은** `apply_mano_transform` 함수 호출.
   이 공통화 덕에 downstream 코드는 어떤 sensor 인지 모른다 → §6 에서 deep-dive.

### 5.3 RealSenseSensing 비동기 wrapper

[`sensing/realsense/realsense_sensing.py:115-144`](../sensing/realsense/realsense_sensing.py#L115-L144)
는 §4.4 의 phone wrapper 와 **거의 똑같다**. 차이는 `self._camera.read()` 가 `(ok, color, depth)`
튜플을 돌려준다는 것과 `self._converter.convert(best, depth, w, h)` 로 depth/해상도를 같이
넘긴다는 것뿐.

```python
def _detect_loop(self) -> None:
    while self._running:
        ok, color, depth = self._camera.read()
        if not ok or color is None or depth is None:
            time.sleep(0.001)
            continue
        ...
        detections = self._detector.detect(color, ts_ms)
        ...
        if detections:
            best = self._select_hand(detections)
            if best is not None:
                keypoints_3d = self._converter.convert(best, depth, w, h)
                keypoints_2d = DepthKeypointConverter.extract_2d(best)
                keypoints = HandKeypoints(
                    keypoints_3d=keypoints_3d,
                    ...
                    source="realsense",
                )
```

**핵심 메시지**: sensor 가 달라도 **`HandKeypoints` 형식만 맞추면** downstream 은 바뀌지 않는다.
phone 과 RealSense 가 서로 다른 `_converter` 를 사용하지만 둘 다 최종적으로 `(21, 3) MANO frame`
을 내놓는다.

---

## 6. MANO 변환 deep-dive

파이프라인의 가장 수학적이고 가장 헷갈리는 부분. 별도 섹션으로 두는 이유:
1. Phone 과 RealSense 둘 다 호출하는 **공통 모듈**
2. 학습자가 "왜 SVD 가 갑자기 나오지?" 하고 가장 막히는 부분
3. 과거 phone DexPilot 버그 + Manus 25→21 버그 둘 다 이 frame 이슈와 간접적으로 연결

파일: [`sensing/core/mano_transform.py`](../sensing/core/mano_transform.py) (단 98줄)

### 6.1 OPERATOR2MANO_RIGHT / LEFT — 고정 회전 행렬

[`mano_transform.py:23-39`](../sensing/core/mano_transform.py#L23-L39):

```python
OPERATOR2MANO_RIGHT = np.array(
    [
        [0, 0, -1],
        [-1, 0, 0],
        [0, 1, 0],
    ],
    dtype=np.float32,
)

OPERATOR2MANO_LEFT = np.array(
    [
        [0, 0, -1],
        [1, 0, 0],
        [0, -1, 0],
    ],
    dtype=np.float32,
)
```

이건 단순한 **축 재배열 행렬** (signed permutation matrix). 각 row 는 하나의 축 치환:
- `[0, 0, -1]` = 입력의 z 축이 출력의 x 축 (부호 반전)
- `[-1, 0, 0]` = 입력의 x 축이 출력의 y 축 (부호 반전)
- `[0, 1, 0]` = 입력의 y 축이 출력의 z 축

이 상수는 dex-retargeting 의 [`single_hand_detector.py:9-24`](../models/dex_retarget/dex-retargeting/example/vector_retargeting/single_hand_detector.py#L9-L24)
에서 그대로 복사된 값이다. "MANO paper 가 정의한 표준 hand coordinate" 와 "우리의 wrist-aligned
intermediate frame" 사이의 정렬을 맞추는 **고정** 변환.

좌우 손의 차이는 단순 sign flip — x 축과 z 축의 부호가 뒤집힌다. 이 때문에 왼손/오른손
구분이 필요하다.

### 6.2 estimate_wrist_frame — 손마다 동적으로 계산되는 회전

이 함수가 MANO 변환의 진짜 핵심이다. [`mano_transform.py:42-79`](../sensing/core/mano_transform.py#L42-L79):

```python
def estimate_wrist_frame(keypoints: np.ndarray) -> np.ndarray:
    """Estimate the wrist coordinate frame from 21-point hand keypoints.

    Uses wrist (0), index MCP (5), middle MCP (9) to define the palm plane
    via SVD, then orthonormalizes the basis.
    """
    assert keypoints.shape == (21, 3)
    points = keypoints[[0, 5, 9], :]

    # x direction: palm → middle finger base
    x_vector = points[0] - points[2]

    # SVD to find palm plane normal
    centered = points - np.mean(points, axis=0, keepdims=True)
    _u, _s, v = np.linalg.svd(centered)
    normal = v[2, :]

    # Gram-Schmidt: make x orthogonal to normal
    x = x_vector - np.sum(x_vector * normal) * normal
    x = x / (np.linalg.norm(x) + 1e-10)
    z = np.cross(x, normal)

    # Orientation disambiguation: z should roughly point from pinky → index
    if np.sum(z * (points[1] - points[2])) < 0:
        normal = -normal
        z = -z

    frame = np.stack([x, normal, z], axis=1)
    return frame.astype(np.float32)
```

4단계로 해부:

#### Step 1 — 3점 선택

```python
points = keypoints[[0, 5, 9], :]   # wrist, index_MCP, middle_MCP
```

이 3점은 **거의 항상 손바닥 평면 위에 있다** (wrist + 두 손가락의 관절 base). 3점이 평면을
정의하는 최솟값이고, 4점 이상은 중복 정보이며 SVD 의 robust ness 는 3점으로 이미 충분하다.

#### Step 2 — SVD 로 손바닥 평면의 normal 찾기

```python
centered = points - np.mean(points, axis=0, keepdims=True)
_u, _s, v = np.linalg.svd(centered)
normal = v[2, :]
```

직관: SVD 는 데이터의 "주 방향" (principal directions) 을 찾는다. 3개 점의 mean 을 빼서
centered 한 뒤 SVD 를 하면:
- `v[0]` = 데이터가 가장 많이 퍼진 방향 (손바닥 평면 위의 한 방향)
- `v[1]` = 두 번째로 많이 퍼진 방향 (손바닥 평면 위의 다른 방향, v[0] 에 수직)
- `v[2]` = **가장 적게 퍼진 방향 = 평면에 수직 = normal**

이게 "3점으로 평면 fitting" 의 SVD 버전. 3점만으로는 normal 이 unique 하게 결정되므로
실질적으로 `cross(p1-p0, p2-p0)` 와 동등하지만, SVD 는 점이 4개 이상일 때도 잘 작동한다는
장점이 있어 확장성이 좋다.

#### Step 3 — Gram-Schmidt 직교화로 x 축 만들기

```python
x_vector = points[0] - points[2]       # wrist → middle_MCP (원하는 x 방향)
x = x_vector - np.sum(x_vector * normal) * normal   # normal 성분 제거
x = x / np.linalg.norm(x)
z = np.cross(x, normal)
```

`x_vector` (wrist → middle_MCP) 가 손바닥 평면 위에 **정확히** 있다고 보장되지 않는다 (관절의
두께, MediaPipe 의 노이즈 등). 그래서 `normal` 방향 성분을 빼서 강제로 평면 위로 투영 →
normalize 한다. z 는 `x × normal` 로 자동 결정.

결과: 세 축 `x`, `normal`, `z` 가 **서로 수직인 정규 basis**.

#### Step 4 — 방향 모호성 해소

```python
if np.sum(z * (points[1] - points[2])) < 0:
    normal = -normal
    z = -z
```

여기가 미묘한 부분. SVD 의 `v[2]` (normal) 는 **부호가 임의**이다 — 손등을 가리킬 수도,
손바닥을 가리킬 수도 있다. 우리는 이 모호성을 "index_MCP → middle_MCP 방향이 z 축과 대략
같은 부호여야 한다" 는 anatomical prior 로 제거한다.

- `points[1] - points[2]` = middle_MCP → index_MCP (실제로는 pinky→index 방향에 가깝지만 인덱스
  세팅상 이렇게 씀)
- 이 벡터와 우리가 계산한 `z` 의 내적이 음수이면, `z` 가 반대 방향이라는 뜻 → `normal` 과 `z`
  모두 뒤집음

이 단계가 없으면 같은 손 모양인데 어떤 프레임에서는 손등이 위, 어떤 프레임에서는 손바닥이
위로 나와서 dex-retargeting 출력이 미친 듯이 흔들린다.

#### 최종 결과

```python
frame = np.stack([x, normal, z], axis=1)
return frame
```

`frame` 은 (3, 3) 행렬로, **columns** 이 x / normal / z basis vector. 이게 "camera frame → wrist
frame 회전 행렬" 이다.

### 6.3 apply_mano_transform — 두 회전의 합성

[`mano_transform.py:82-98`](../sensing/core/mano_transform.py#L82-L98):

```python
def apply_mano_transform(keypoints, hand_type="right") -> np.ndarray:
    wrist_rot = estimate_wrist_frame(keypoints)
    if hand_type.lower() == "left":
        return keypoints @ wrist_rot @ OPERATOR2MANO_LEFT
    return keypoints @ wrist_rot @ OPERATOR2MANO_RIGHT
```

한 줄의 본질:

```
MANO_keypoints = keypoints @ wrist_rot @ OPERATOR2MANO_RIGHT
                 └─ (C) ─┘   └─ 동적 ─┘   └─── 고정 ───┘
```

- 첫 번째 `@ wrist_rot`: (C) wrist-centered frame → wrist-aligned intermediate frame
  (손마다 다르게 계산)
- 두 번째 `@ OPERATOR2MANO_RIGHT`: intermediate frame → (D) MANO 표준 frame
  (모든 프레임에서 동일)

두 회전을 합성해서 한 번에 처리한다. `@` 는 matrix multiply 이고, row-vector 규약
(`point @ R`) 을 쓰고 있다.

### 6.4 왜 이 변환이 필수인가 (요약)

- **MANO 없이 optimizer 에 넘기면**: 카메라가 30° 기울 때 로봇 손도 같이 30° 기울어짐
- **MANO 적용 후**: 카메라를 어떻게 돌리든 dex-retargeting 은 손 모양만 보고 retarget
- **검증**: 우리의 `estimate_wrist_frame` 은 dex-retargeting upstream 의 `single_hand_detector.
  estimate_frame_from_hand_points` 와 max diff **0.0000mm** 로 완전히 일치 (이전 디버깅에서 실제
  mp4 데이터로 검증)

이 공통 모듈이 생기기 전에는 phone 과 realsense converter 가 각자 다른 구현을 복사해 갖고
있었는데, 그러다 한 쪽만 버그가 나서 실시간 파이프라인이 깨진 적이 있다. 그 재발 방지를
위해 `sensing/core/mano_transform.py` 로 단일화되었다.

---

## 7. DexRetargetModel 래퍼

`HandKeypoints` 를 받으면 dex-retargeting 라이브러리를 돌려 `q` (20 DOF joint angles) 를 돌려주는
얇은 wrapper. 파일 전체 100줄. [`models/dex_retarget/dex_retarget_model.py`](../models/dex_retarget/dex_retarget_model.py)

### 7.1 __init__ — config → optimizer

[`dex_retarget_model.py:33-57`](../models/dex_retarget/dex_retarget_model.py#L33-L57):

```python
def __init__(self, config_path: str):
    from dex_retargeting.retargeting_config import RetargetingConfig

    self._config_path = config_path
    config = RetargetingConfig.load_from_file(config_path)
    self._retargeting = config.build()

    # Single source of truth: the optimizer's own indices.
    # - vector:   (2, N) array from YAML target_link_human_indices
    # - dexpilot: (2, num_pairs + num_fingers) array auto-generated by
    #             DexPilotOptimizer.__init__ via generate_link_indices()
    # - position: (N,) array of fingertip indices
    optimizer = self._retargeting.optimizer
    self._retargeting_type = optimizer.retargeting_type
    self._human_indices = optimizer.target_link_human_indices
```

핵심 포인트:

1. **YAML → RetargetingConfig → SeqRetargeting**: [`config/dg5f_right_dexpilot.yml`](../models/dex_retarget/config/dg5f_right_dexpilot.yml)
   같은 파일에서 optimizer type (`vector` / `dexpilot` / `position`), URDF 경로, fingertip link 이름,
   scaling factor 등을 모두 읽어 들임
2. **`optimizer.target_link_human_indices` 가 single source of truth**: 이 필드 하나만 읽으면
   vector / dexpilot / position 어느 모드든 ref_value 를 만들 수 있다. 과거에 wrapper 가 이걸
   무시하고 자체 ordering 을 손으로 짰던 phone DexPilot 버그가 있었음 (§8.1 참조).

### 7.2 _build_ref_value — keypoints → optimizer 입력

[`dex_retarget_model.py:88-100`](../models/dex_retarget/dex_retarget_model.py#L88-L100):

```python
def _build_ref_value(self, kp: np.ndarray) -> np.ndarray:
    """Convert (21, 3) keypoints to optimizer-specific input."""
    indices = self._human_indices
    if self._retargeting_type == "POSITION":
        return kp[indices, :]
    origin_indices = indices[0, :]
    task_indices = indices[1, :]
    return kp[task_indices, :] - kp[origin_indices, :]
```

3가지 type 을 한 코드로 처리:

- **POSITION**: `indices` shape 은 (N,). fingertip 인덱스를 직접 gather → `kp[indices]` 그대로 반환
- **VECTOR**: `indices` shape 은 (2, N). origin 과 task 두 배열로 나뉨 → `kp[task] - kp[origin]`
  으로 N 개의 벡터를 만듬. YAML 의 `target_link_human_indices` 에서 직접 지정됨
- **DEXPILOT**: `indices` shape 은 (2, num_pairs + num_fingers) = (2, 15) for 5 fingers. 라이브러리가
  `DexPilotOptimizer.__init__` 에서 자동 생성함 (§8.1 에서 자세히)

모든 type 이 **같은 코드 경로**를 타기 때문에 wrapper 에 type 별 분기가 필요 없다. 이게
"optimizer.target_link_human_indices 가 single source of truth" 라는 말의 구체적 의미.

### 7.3 retarget — pipeline 진입점

[`dex_retarget_model.py:63-72`](../models/dex_retarget/dex_retarget_model.py#L63-L72):

```python
def retarget(self, keypoints: HandKeypoints) -> np.ndarray:
    """Convert HandKeypoints → robot joint angles."""
    kp = keypoints.keypoints_3d  # (21, 3)
    ref_value = self._build_ref_value(kp)

    t0 = time.perf_counter()
    q = self._retargeting.retarget(ref_value)
    self._last_solve_ms = (time.perf_counter() - t0) * 1000

    return q
```

3줄로 모든 변환이 trigger 된다:
1. `kp = keypoints.keypoints_3d` — (21, 3) 추출
2. `ref_value = self._build_ref_value(kp)` — optimizer 입력 형식으로 변환
3. `q = self._retargeting.retarget(ref_value)` — §8 의 라이브러리 내부로 넘김

반환 `q` 는 numpy 배열 (20,) float32, radians, DG-5F joint angle. §9 에서 ROS2 publish 된다.

---

## 8. dex-retargeting 라이브러리 내부

라이브러리에 깊이 들어가지 않고 "이 세 가지만 알면 black box 가 white box 가 된다" 수준만
본다. 더 궁금하면 [`docs/dex_retargeting.md`](dex_retargeting.md) 의 수학 분석 참조.

### 8.1 DexPilotOptimizer.generate_link_indices — pairwise pinch 쌍 생성

[`dex-retargeting/src/dex_retargeting/optimizer.py:407-428`](../models/dex_retarget/dex-retargeting/src/dex_retargeting/optimizer.py#L407-L428):

```python
@staticmethod
def generate_link_indices(num_fingers):
    """
    Example:
    >>> generate_link_indices(4)
    ([2, 3, 4, 3, 4, 4, 0, 0, 0, 0], [1, 1, 1, 2, 2, 3, 1, 2, 3, 4])
    """
    origin_link_index = []
    task_link_index = []

    # Add indices for connections between fingers
    for i in range(1, num_fingers):
        for j in range(i + 1, num_fingers + 1):
            origin_link_index.append(j)
            task_link_index.append(i)

    # Add indices for connections to the base (0)
    for i in range(1, num_fingers + 1):
        origin_link_index.append(0)
        task_link_index.append(i)

    return origin_link_index, task_link_index
```

`num_fingers=5` 일 때 출력을 직접 계산:

```
pairwise (10개, 앞부분):
  (2,1) (3,1) (4,1) (5,1)  ← finger 1 (index) vs 2,3,4,5
  (3,2) (4,2) (5,2)        ← finger 2 (middle) vs 3,4,5
  (4,3) (5,3)              ← finger 3 (ring) vs 4,5
  (5,4)                    ← finger 4 (pinky) vs 5

wrist→tip (5개, 뒷부분):
  (0,1) (0,2) (0,3) (0,4) (0,5)

origin: [2, 3, 4, 5, 3, 4, 5, 4, 5, 5, 0, 0, 0, 0, 0]
task:   [1, 1, 1, 1, 2, 2, 2, 3, 3, 4, 1, 2, 3, 4, 5]
```

그다음 `DexPilotOptimizer.__init__` 에서 이 인덱스에 **×4** 를 곱해서 MANO fingertip 인덱스로
변환한다 ([`optimizer.py:357-364`](../models/dex_retarget/dex-retargeting/src/dex_retargeting/optimizer.py#L357-L364)):

```python
origin_human_indices: [8, 12, 16, 20, 12, 16, 20, 16, 20, 20, 0, 0, 0, 0, 0]
task_human_indices:   [4,  4,  4,  4,  8,  8,  8, 12, 12, 16, 4, 8, 12, 16, 20]
```

이게 `DexRetargetModel._build_ref_value` 가 읽는 `self._human_indices` 다.
`kp[task] - kp[origin]` 을 하면 (15, 3) 벡터가 나오는데:

| row | task - origin | 의미 |
|---|---|---|
| 0 | kp[4] - kp[8] | thumb_tip - index_tip (pinch 1) |
| 1 | kp[4] - kp[12] | thumb_tip - middle_tip (pinch 2) |
| 2 | kp[4] - kp[16] | thumb_tip - ring_tip |
| 3 | kp[4] - kp[20] | thumb_tip - pinky_tip |
| 4 | kp[8] - kp[12] | index_tip - middle_tip |
| 5 | kp[8] - kp[16] | index_tip - ring_tip |
| 6 | kp[8] - kp[20] | index_tip - pinky_tip |
| 7 | kp[12] - kp[16] | middle_tip - ring_tip |
| 8 | kp[12] - kp[20] | middle_tip - pinky_tip |
| 9 | kp[16] - kp[20] | ring_tip - pinky_tip |
| 10 | kp[4] - kp[0] | wrist → thumb_tip |
| 11 | kp[8] - kp[0] | wrist → index_tip |
| 12 | kp[12] - kp[0] | wrist → middle_tip |
| 13 | kp[16] - kp[0] | wrist → ring_tip |
| 14 | kp[20] - kp[0] | wrist → pinky_tip |

→ 앞 10개는 **손가락 쌍 사이 거리**, 뒤 5개는 **wrist 에서 각 손가락 끝까지 방향**.
이 15개 벡터를 로봇의 대응 벡터와 맞추는 것이 DexPilot cost function 의 정의다.

**이전 버그 기록**: 초기 구현에서 wrapper 가 이 ordering 을 거꾸로 짜서 (wrist→tip 을 앞에,
pairwise 를 뒤에) DexPilot 모드가 손이 이상한 자세에서 굳는 버그가 있었다. 그래서 지금은
"wrapper 가 자체 ordering 을 쓰지 않고 optimizer 가 갖고 있는 인덱스를 그대로 사용" 한다.
자세한 건 [`models/dex_retarget/dex_retarget_model.py`](../models/dex_retarget/dex_retarget_model.py)
의 주석에 기록되어 있다.

### 8.2 SeqRetargeting.retarget — warm start + NLOPT + LP filter

[`dex-retargeting/src/dex_retargeting/seq_retarget.py:112-134`](../models/dex_retarget/dex-retargeting/src/dex_retargeting/seq_retarget.py#L112-L134):

```python
def retarget(self, ref_value, fixed_qpos=np.array([])):
    tic = time.perf_counter()

    qpos = self.optimizer.retarget(
        ref_value=ref_value.astype(np.float32),
        fixed_qpos=fixed_qpos.astype(np.float32),
        last_qpos=np.clip(
            self.last_qpos, self.joint_limits[:, 0], self.joint_limits[:, 1]
        ),
    )
    self.accumulated_time += time.perf_counter() - tic
    self.num_retargeting += 1
    self.last_qpos = qpos
    robot_qpos = np.zeros(self.optimizer.robot.dof)
    robot_qpos[self.optimizer.idx_pin2fixed] = fixed_qpos
    robot_qpos[self.optimizer.idx_pin2target] = qpos

    if self.optimizer.adaptor is not None:
        robot_qpos = self.optimizer.adaptor.forward_qpos(robot_qpos)

    if self.filter is not None:
        robot_qpos = self.filter.next(robot_qpos)
    return robot_qpos
```

3가지 핵심:

1. **`last_qpos`** (warm start): 이전 프레임의 해를 현재 NLOPT 의 초기값으로 쓴다. 손이 프레임
   사이에 크게 움직이지 않기 때문에 이전 해가 좋은 출발점. 이게 없으면 수렴이 2–3배 느려진다.
   `clip` 은 joint limit 밖으로 나간 값 (수치 오차) 을 다시 안으로 넣는 안전장치.
2. **`self.optimizer.retarget(...)`**: 실제 NLOPT (SLSQP) 호출 — 내부에서 cost function 을
   gradient 기반으로 minimize. DG-5F 에서는 프레임당 ~5-10ms.
3. **`self.filter.next(robot_qpos)`**: LPFilter (§8.3) 로 출력 smoothing. 이게 없으면 NLOPT 해의
   프레임간 작은 jitter 가 DG-5F 에 그대로 전달되어 손가락이 떨린다.

### 8.3 LPFilter — 단순 EMA smoothing

[`dex-retargeting/src/dex_retargeting/optimizer_utils.py:1-13`](../models/dex_retarget/dex-retargeting/src/dex_retargeting/optimizer_utils.py#L1-L13) — 이 라이브러리의 가장 짧은 파일:

```python
class LPFilter:
    def __init__(self, alpha):
        self.alpha = alpha
        self.y = None
        self.is_init = False

    def next(self, x):
        if not self.is_init:
            self.y = x
            self.is_init = True
            return self.y.copy()
        self.y = self.y + self.alpha * (x - self.y)
        return self.y.copy()
```

공식: `y[t] = y[t-1] + α (x[t] - y[t-1])` = **exponential moving average**.

- `α` = `low_pass_alpha` in YAML, 권장값 0.2
- α 가 작으면 → 부드럽지만 반응이 느림 (lag)
- α 가 크면 → 반응 빠르지만 떨림이 남음
- 모든 joint 에 **같은 α** 적용 — 손가락별로 다른 latency 가 필요하면 별도 작업 필요

이게 바로 파이프라인 끝에서 "20개 joint angle 의 마지막 smoothing 단계". `robot_qpos` 가 이
필터를 거쳐 최종적으로 `DexRetargetModel.retarget` 의 반환값이 된다.

---

## 9. ROS2 publish 루프

[`models/dex_retarget/main.py:137-165`](../models/dex_retarget/main.py#L137-L165) — 파이프라인을
실제로 돌리는 15줄의 main loop:

```python
while not shutdown:
    kp = sensing.get_keypoints()
    if kp is None:
        time.sleep(dt)
        continue

    q = model.retarget(kp)

    # Publish
    if publisher is not None:
        from sensor_msgs.msg import JointState as JS
        msg = JS()
        msg.header.stamp = ros2_node.get_clock().now().to_msg()
        msg.name = joint_names
        msg.position = q.tolist()
        publisher.publish(msg)

    # Log every ~1 second
    frame_count += 1
    if frame_count % args.hz == 0:
        debug = model.get_debug_info(kp, q)
        angles = debug["dg5f_angles_deg"]
        logger.info(
            "solve=%.1fms q=[%s]",
            debug["solve_time_ms"],
            " ".join(f"{a:+.1f}" for a in angles[:8]) + " ...",
        )

    time.sleep(dt)
```

알아야 할 4가지:

1. **`kp is None` 은 skip**: sensing 이 아직 첫 detection 을 못 했거나, 손이 frame 밖이거나,
   MediaPipe 가 실패하면 None 이 반환된다. main loop 는 skip 하고 다음 iteration 으로 넘어간다
   → DG-5F 는 마지막으로 받은 명령을 그대로 유지.
2. **`model.retarget(kp)`**: §7.3 의 한 호출이 §6 (MANO) → §7.2 (ref_value) → §8.2 (NLOPT+filter) 를
   모두 trigger.
3. **JointState 메시지 구조**: `msg.name` 은 joint 이름 list (`model.get_joint_names()` 로 가져옴,
   DG-5F 는 20개), `msg.position` 은 같은 순서의 float list (radians). Isaac Sim 은 `name` 필드로
   매칭하므로 **순서 무관** — 이걸 모르고 순서를 바꿀까 걱정하는 학습자가 많음.
4. **`time.sleep(dt)`**: `dt = 1.0 / args.hz` — 30Hz 기본. sleep 이 정확하진 않지만 대략적인
   루프 rate 제어에 충분.

전체 main 함수 ([`main.py:97-174`](../models/dex_retarget/main.py#L97-L174)) 는 sensing 생성,
model 로드, ROS2 init, signal handling, cleanup 등을 다루지만, **본질은 위의 15줄**이다.

---

## 10. 다른 입력 소스와의 비교

| | Phone | RealSense | Manus | Mock |
|---|---|---|---|---|
| 입력 매체 | RGB pixel (HTTP) | color + depth (USB3) | 글러브 IMU (wireless) | sine wave |
| 2D detection | MediaPipe | MediaPipe | **불필요** | 불필요 |
| 3D 복원 | MediaPipe `world_landmarks` (추정 z) | `rs2_deproject` + 실 depth | SDK 직접 출력 | 합성 |
| MANO 변환 | **필수** | **필수** | **필수** (SDK 가 world frame) | 불필요 (이미 21노드 빌드) |
| 노드 수 | 21 (MediaPipe 기본) | 21 | **25 → 21 리매핑 필요** | 21 |
| Occlusion 내성 | ★★ | ★★★ | ★★★★★ | — |
| 상세 가이드 | [`phone_realtime.md`](../models/dex_retarget/docs/phone_realtime.md) | [`realsense_d405.md`](../models/dex_retarget/docs/realsense_d405.md) | [`manus_realtime.md`](../models/dex_retarget/docs/manus_realtime.md) | — |

### Manus 가 다른 부분

Manus Glove 는 카메라 기반이 아니라 글러브 IMU 로 직접 21 노드의 3D 좌표를 측정한다. 그래서:
- MediaPipe 2D detection 이 필요 없음
- depth 추정도 필요 없음 (sensor 가 직접 측정)

**MANO 좌표계 변환은 phone/realsense 와 마찬가지로 필수**다. SDK 가 raw skeleton 을 wrist 가 아닌
**world frame** ([VUH = `XFromViewer`, `PositiveZ`, right-handed](../sensing/manus/sdk/ManusSDK/include/ManusSDKTypes.h#L1707-L1714))
로 publish 하므로, 글러브를 어떻게 들고 있느냐 (palm down, palm up, …) 에 따라 손가락 방향이 회전한다.
회전 보정 없이 dex-retargeting 에 넘기면 손가락 굽힘 축이 어긋나 fist → spread 같은 inversion
이 발생한다 (실제로 발생했던 디버그 사례: [`manus_debug.md`](../models/dex_retarget/docs/manus_debug.md)).
그래서 [`sensing/manus/manus_sensing.py`](../sensing/manus/manus_sensing.py) 도 phone 과 동일하게
wrist shift 후 [`apply_mano_transform`](../sensing/core/mano_transform.py) 을 호출한다.

또한 Manus 고유의 노드 개수 문제가 추가로 있다: **SDK 가 25개 노드를 publish** 한다 (1 wrist + thumb 4 +
4 fingers × 5). MANO 표준은 21개이므로 [`sensing/manus/ros2_provider.py`](../sensing/manus/ros2_provider.py)
의 `_remap_to_mano_21()` 헬퍼가 `(chain_type, joint_type)` 메타데이터를 이용해 4개의 non-thumb
Metacarpal 노드를 drop 하고 21개로 재배치한다. 자세한 건 [`manus_realtime.md` §5](../models/dex_retarget/docs/manus_realtime.md)
참조.

### 핵심 통찰

> **어떤 sensor 를 쓰든 결국 `HandKeypoints { keypoints_3d: (21, 3) MANO frame }` 한 형식으로 수렴한다.**
> **이게 파이프라인 전체가 동일한 downstream 코드로 동작할 수 있는 이유다.**

---

## 11. 부록: 학습자용 reading order

처음 코드 읽는 사람을 위한 추천 reading 순서.

### 5분 코스 — 이해의 뼈대

30줄만 읽으면 전체 구조가 잡힌다:

1. [`sensing/common.py:63-104`](../sensing/common.py#L63-L104) — `HandKeypoints` dataclass + `SensingSource` ABC
2. [`models/dex_retarget/main.py:137-165`](../models/dex_retarget/main.py#L137-L165) — main loop 15줄
3. [`models/dex_retarget/dex_retarget_model.py:63-72`](../models/dex_retarget/dex_retarget_model.py#L63-L72) — `retarget()` 10줄

이 3개를 순서대로 읽으면 "sensor → HandKeypoints → q → ROS2" 흐름이 한 번에 보인다.

### 30분 코스 — Phone 한 sensor 따라가기

위 + §4 전체 + §6 MANO. 이 정도면 한 sensor 의 full path 가 이해되고, 새로운 sensor 를 추가할
때 어디에 무엇을 두어야 하는지 감이 온다.

### 2시간 코스 — 전체 변환 마스터

이 문서 전부 + 다음 3개를 소스에서 직접 읽기:

1. [`dex-retargeting/src/dex_retargeting/optimizer.py`](../models/dex_retarget/dex-retargeting/src/dex_retargeting/optimizer.py) —
   특히 `DexPilotOptimizer.__init__` (line 333–405) 와 `get_objective_function` (line 456–577). NLOPT
   로 넘어가는 cost function 의 실제 구조가 보인다.
2. [`sensing/core/mano_transform.py`](../sensing/core/mano_transform.py) 전체 — 98줄. SVD 부분을
   numpy 로 한 번 손으로 돌려보면 직관이 확실히 잡힌다.
3. [`docs/dex_retargeting.md`](dex_retargeting.md) — dex-retargeting 라이브러리의 수학/비용함수 분석.
   이 walkthrough 의 §8 은 그 문서의 요약.

이 순서대로 읽으면 **어느 버그를 만나도 어디를 봐야 할지 알 수 있게 된다** 는 것이 목표다.
