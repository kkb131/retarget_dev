# Phone 실시간 Retargeting 가이드

Android 폰 카메라를 사용하여 DG-5F (Isaac Sim)를 실시간 제어.

```
Android Phone (IP Webcam)
    ↓ HTTP MJPEG stream
PhoneSensing (MediaPipe + MANO transform)
    ↓ HandKeypoints (21, 3)
DexRetargetModel (VectorOptimizer / DexPilot)
    ↓ 20 DOF
ROS2 JointState → Isaac Sim DG-5F
```

## 1. 준비물

- Android 폰 + IP Webcam 앱 (Play Store)
- 폰과 개발 PC가 같은 Wi-Fi 네트워크
- Isaac Sim에서 DG-5F가 실행 중 (`/dg5f_right/joint_commands` subscribe)

## 2. IP Webcam 앱 설정

1. Play Store에서 "IP Webcam" (Pavel Khlebovich) 설치
2. 앱 실행 → 하단 **Start server** 탭
3. 화면에 표시되는 URL 확인. 예시:
   ```
   http://192.168.0.3:8080
   ```
4. 스트리밍 URL:
   ```
   http://<phone-ip>:8080/video
   ```

### 권장 설정 (앱 내부)

- **Video resolution**: 640×480 (지연 최소화)
- **Video quality**: 50~70%
- **FPS limit**: 30
- **Orientation**: Landscape (고정)
- **Focus mode**: Continuous

## 3. 실행

```bash
cd /workspaces/tamp_ws/src

# DexPilot (권장, pinch 정확)
python3 -m retarget_dev.models.dex_retarget.main \
  --sensing phone \
  --config retarget_dev/models/dex_retarget/config/dg5f_right_dexpilot.yml \
  --url http://192.168.0.3:8080/video \
  --topic /dg5f_right/joint_commands \
  --hz 30

# Vector (더 빠름, 일반 동작)
python3 -m retarget_dev.models.dex_retarget.main \
  --sensing phone \
  --config retarget_dev/models/dex_retarget/config/dg5f_right_vector.yml \
  --url http://192.168.0.3:8080/video
```

Isaac Sim에서 DG-5F 손가락이 실시간으로 사람 손 동작을 따라 하면 성공.

## 4. 파이프라인 구조

### 4-1. 센싱 계층 (`retarget_dev/sensing/phone/`)

```
CameraSource      ← HTTP MJPEG 캡처 (threaded)
    ↓
HandDetector      ← MediaPipe HandLandmarker (VIDEO mode)
    ↓
KeypointConverter ← MANO 좌표계 변환 (필수!)
    ↓
HandKeypoints (21, 3)
```

**중요**: `KeypointConverter`는 `hand_type` 파라미터를 받아 `operator2mano`
회전을 자동 선택합니다. `PhoneSensing(hand_side="right")`로 생성하면
우측 손 매핑이 적용됩니다.

### 4-2. 리타게팅 계층

```
HandKeypoints (21, 3)
    ↓
DexRetargetModel  ← dex-retargeting SeqRetargeting wrapper
    ↓
q (20 DOF)
    ↓
JointState publish → /dg5f_right/joint_commands
```

## 5. MANO 변환이 필수인 이유

MediaPipe가 반환하는 `world_landmarks`는 **카메라 프레임 기준 3D 좌표**입니다.
카메라가 기울거나 손의 방향이 바뀌면 벡터의 절대 방향이 그대로 바뀝니다.

dex-retargeting optimizer는 **MANO 표준 프레임**(손바닥 평면이 x-z, 손가락이 +z 쪽)
에서의 벡터를 기대합니다. 변환을 건너뛰면:

- 손의 절대 방향이 카메라 시점에 의존 → 로봇이 엉뚱한 방향으로 회전
- 같은 손 모양이라도 각도에 따라 다른 joint 명령 출력
- Vector optimizer가 방향을 맞추려고 손등으로 뒤집힘

변환 과정 (`KeypointConverter._estimate_wrist_frame()`):

1. wrist(0), index_MCP(5), middle_MCP(9) 3점으로 손바닥 평면을 SVD로 추정
2. x축: wrist→middle_MCP 방향 (손바닥 평면 내)
3. y축: 평면 normal
4. z축: x × y
5. `operator2mano` 고정 회전으로 MANO 규약에 맞춤

결과: 카메라 시점에 무관하게 **항상 일관된 프레임**에서의 키포인트 벡터.

## 6. 트러블슈팅

### 6-1. 폰 연결 안 됨

```
Error: Could not open camera source
```

- 폰 IP 확인: 앱 화면 하단에 표시
- 같은 Wi-Fi인지 확인
- 브라우저에서 `http://<phone-ip>:8080`로 접속 → 앱 UI가 보여야 함
- URL에 `/video` 꼭 붙이기

### 6-2. 지연이 큼

- 해상도 낮춤 (640×480 → 480×360)
- 앱에서 **Power management** → **Keep screen on** 활성화
- 폰을 5GHz Wi-Fi에 연결 (2.4GHz는 지연 큼)
- USB tethering 사용 시 더 안정적

### 6-3. Hand detection 실패가 잦음

- 밝은 배경에 손을 배치
- 손 전체가 프레임 안에 들어오도록
- MediaPipe는 손목이 보여야 잘 동작함
- 카메라 10~50cm 거리 유지

### 6-4. 로봇이 카메라 시점에 따라 다른 방향으로 움직임

→ MANO 변환이 적용되지 않음. 다음 확인:
```python
from retarget_dev.sensing.phone.keypoint_converter import KeypointConverter
conv = KeypointConverter()
print(conv._apply_mano)  # True여야 함
```

### 6-5. DG-5F가 손가락을 구부리지 않음

→ `dg5f_right_dexpilot.yml`의 `scaling_factor` 조정 (1.2~1.4 권장).
자세한 내용은 `dg5f_tuning.md` 참조.

### 6-6. 검지/중지가 손등으로 뒤집힘

→ `config`의 `urdf_path`가 `dg5f_right_retarget.urdf` (joint limit 수정본)를
가리키는지 확인. `dg5f_tuning.md`의 "URDF joint limits 수정" 섹션 참조.

## 7. 성능 참고치

- MediaPipe detection: ~15-20ms/frame (640×480)
- dex-retargeting VectorOptimizer: ~2-5ms/frame
- dex-retargeting DexPilot: ~5-10ms/frame
- Total round-trip (phone → Isaac Sim): ~100-150ms (네트워크 포함)

30Hz 제어 루프가 안정적으로 유지됩니다 (33ms budget).

## 8. 다른 센싱 소스로 교체

동일한 `main.py`가 다양한 입력을 지원합니다:

```bash
# Manus Glove (ROS2 topic)
--sensing manus-ros2

# Mock (테스트용)
--sensing manus-mock

# Manus SDK binary
--sensing manus-sdk --sdk-bin /path/to/SDKClient_Linux.out
```

모두 동일한 `HandKeypoints (21, 3)` 출력을 생성하므로 retargeting 파이프라인은
그대로 사용됩니다.
