# 환경 설정 가이드

dex-retargeting을 DG-5F와 함께 사용하기 위한 환경 구축 방법.

## 1. dex-retargeting 설치

### 1-1. pip 설치 + GitHub clone (둘 다 필요)

```bash
# pip으로 라이브러리 설치
pip install dex_retargeting

# example 코드와 데이터를 위해 GitHub clone
# 위치: retarget_dev/models/dex_retarget/dex-retargeting/ (dex_retarget 모델 내부)
cd /workspaces/tamp_ws/src/retarget_dev/models/dex_retarget
git clone https://github.com/dexsuite/dex-retargeting.git

# 예제 의존성 설치 (sapien, mediapipe, loguru, tyro 등)
cd dex-retargeting
pip install -e ".[example]"
```

### 1-2. git submodule 초기화 (필수)

dex-retargeting은 `assets/robots/hands` (URDF 모음)를 git submodule로 관리합니다.
pip 설치만으로는 URDF가 포함되지 않으므로 clone + submodule init이 필요합니다.

```bash
cd /workspaces/tamp_ws/src/retarget_dev/models/dex_retarget/dex-retargeting
git submodule update --init --recursive
```

**확인:**
```bash
ls assets/robots/hands/
# allegro_hand, leap_hand, shadow_hand, ... 폴더들이 보여야 함
```

## 2. 의존성 버전 제약 (중요!)

### 2-1. numpy: **반드시 1.x**

```bash
pip install "numpy<2"
```

**이유:**
- dex-retargeting은 pinocchio (FK/IK 엔진) 기반
- pinocchio는 eigenpy (numpy↔Eigen 바인딩)를 사용
- ROS Humble apt pinocchio, pip eigenpy 모두 **numpy 1.x ABI로 컴파일된 바이너리**
- numpy 2.x 설치 시 `_ARRAY_API not found` 에러 → segfault

**일부 다른 패키지(dex-retargeting 0.5.0 자체)가 numpy 2를 요구해도 무시하고 numpy 1.x 유지.**
실제 동작에 문제 없음.

### 2-2. mediapipe: **0.10.21 정확히 (== 으로 핀, latest 금지)**

```bash
pip install mediapipe==0.10.21
```

**검증된 호환 버전 매트릭스:**

| mediapipe 버전 | 동작 | 비고 |
|---|---|---|
| **0.10.21** | ✅ 동작 | 권장. dex-retargeting upstream도 이 버전 기준 |
| 0.10.22 ~ 0.10.32 | ❌ 깨짐 | `mediapipe.framework` 모듈 제거됨 |
| **0.10.33** | ❌ 깨짐 (검증됨) | 사용자 보고. `single_hand_detector.py` import 단계에서 ImportError |
| 0.10.x latest | ⚠️ 미검증 | 사용 금지 — 위와 동일한 원인으로 깨질 가능성 큼 |

**이유:**
- dex-retargeting 예제의 [`single_hand_detector.py`](../dex-retargeting/example/vector_retargeting/single_hand_detector.py)가 `import mediapipe.framework` 사용 (line 2) + `landmark_pb2` 등 추가 import
- mediapipe 0.10.22 부터 내부 리팩터링으로 `mediapipe.framework` Python 모듈이 제거됨
- 우리 코드 중 [`detect_dg5f.py`](../../detect_dg5f.py) (mp4 → DG-5F pkl) 만 이 파일에 의존. phone / realsense / manus 실시간 경로는 우리 자체 [`sensing/phone/hand_detector.py`](../../../sensing/phone/hand_detector.py) 사용 → mediapipe 버전 제약 영향 없음
- 따라서 mp4 변환 기능을 쓰지 않으면 사실상 mediapipe 버전이 자유롭지만, **혼란 방지를 위해 0.10.21 로 통일** 권장

**`pip install` 시 다른 패키지가 mediapipe latest를 끌어올 수 있음에 주의:**
```bash
# 안전하게 핀 + 강제 재설치
pip install --force-reinstall --no-deps "mediapipe==0.10.21"

# 설치 후 반드시 버전 확인
python3 -c "import mediapipe; print(mediapipe.__version__)"
# 출력: 0.10.21 이어야 함
```

### 2-3. 설치 후 검증

```bash
python3 -c "
from dex_retargeting.retargeting_config import RetargetingConfig
from dex_retargeting.seq_retarget import SeqRetargeting
import pinocchio
import mediapipe
import numpy
print(f'numpy: {numpy.__version__}')
print(f'mediapipe: {mediapipe.__version__}')
print(f'pinocchio: {pinocchio.__version__}')
print('dex_retargeting: OK')
"
```

예상 출력:
```
numpy: 1.26.4
mediapipe: 0.10.21
pinocchio: 3.9.0
dex_retargeting: OK
```

## 3. 렌더링 관련 (선택)

### SAPIEN 뷰어 (Allegro 기본 시각화용)

dex-retargeting 예제의 `render_robot_hand.py`는 SAPIEN Vulkan 렌더러를 사용합니다.
**Vulkan GPU 드라이버가 없으면 동작하지 않습니다.**

우리 환경에서는:
- SAPIEN 시각화는 **동작 안 함** (Vulkan 미지원 환경)
- 대신 **Isaac Sim에서 직접 시각화** (이 프로젝트 방식)

### Isaac Sim

DG-5F가 Isaac Sim에서 아래 토픽을 pub/sub하고 있어야 합니다:
- `/dg5f_right/joint_states` (publish)
- `/dg5f_right/joint_commands` (subscribe)

## 4. 자주 발생하는 문제

### 문제 0: `ModuleNotFoundError: No module named 'dex_retargeting'` (혹은 stale path)

이전에 `pip install -e .` 로 editable install 한 적이 있고 dex-retargeting 디렉터리를 옮겼다면 이 에러가 발생합니다. pip 가 옛 경로를 가리키고 있어서 import 가 깨집니다.

→ **새 위치에서 editable install 재실행**:
```bash
cd /workspaces/tamp_ws/src/retarget_dev/models/dex_retarget/dex-retargeting
pip install -e . --no-deps

# 확인
pip show dex_retargeting | grep "Editable project location"
# → /workspaces/tamp_ws/src/retarget_dev/models/dex_retarget/dex-retargeting 가 보여야 함
```

`--no-deps` 는 mediapipe 등이 함께 upgrade 되어 §2-2 의 0.10.21 핀이 깨지지 않도록 보호합니다.

### 문제 1: `ModuleNotFoundError: No module named 'mediapipe.framework'`

`detect_dg5f.py` 또는 `single_hand_detector.py` 실행 시 발생. mediapipe 0.10.22 이상 (특히 사용자 환경의 0.10.33 에서 검증됨) 에서 `mediapipe.framework` 모듈이 제거되어 import 단계에서 실패합니다.

→ mediapipe 를 0.10.21 로 정확히 핀:
```bash
pip install --force-reinstall --no-deps "mediapipe==0.10.21"
python3 -c "import mediapipe; print(mediapipe.__version__)"  # → 0.10.21
```

`pip install --upgrade mediapipe` 같은 명령으로 다시 latest 가 설치되지 않도록 주의. requirements.txt 에 핀하거나, 다른 패키지 설치 후에는 항상 mediapipe 버전을 재확인하세요.

§2-2 참조 — 호환 버전 매트릭스 + `pip install` 가 mediapipe 를 의도치 않게 upgrade 하는 케이스 회피법.

### 문제 2: `_ARRAY_API not found` 또는 segfault
→ numpy 다운그레이드: `pip install "numpy<2"`

### 문제 3: `ValueError: URDF dir ... not exists`
→ git submodule 초기화: `git submodule update --init --recursive`

### 문제 4: `vk::PhysicalDevice::createDeviceUnique: ErrorExtensionNotPresent`
→ SAPIEN Vulkan 렌더러 실패. `render_robot_hand.py`는 이 환경에서 사용 불가.
Isaac Sim으로 시각화하거나 pkl 데이터만 검증.

## 5. ROS2 환경

ROS2 Humble + rclpy가 설치되어 있어야 합니다 (Isaac Sim 연동용).

```bash
# ROS2 환경 소싱 (필요 시)
source /opt/ros/humble/setup.bash
```

확인:
```bash
python3 -c "import rclpy; from sensor_msgs.msg import JointState; print('ROS2 OK')"
```
