# dex-retargeting 코드 분석 및 적용 계획

**작성일:** 2026.03.27  
**대상:** https://github.com/dexsuite/dex-retargeting  
**목적:** 코드 구조·수학적 이론 분석 + Manus Glove → DG-5F 적용 전략 수립

---

## 1. 저장소 개요

### 1.1 프로젝트 정보

| 항목 | 내용 |
|---|---|
| 저자 | Yuzhe Qin 등 (AnyTeleop 팀, UC San Diego/NVIDIA) |
| 출처 논문 | AnyTeleop (RSS 2023), From One Hand to Multiple Hands (RA-L 2022) |
| 라이선스 | 오픈소스 |
| 설치 | `pip install dex_retargeting` 또는 소스 클론 |
| 로봇 기구학 | pinocchio 기반 |
| 시각화 | SAPIEN 렌더러 |
| 지원 핸드 | Allegro, Shadow, LEAP, Schunk SVH, Ability, DClaw, Panda 등 |

### 1.2 이 저장소가 하는 것

인간 손의 포즈 정보(카메라 영상 또는 키포인트 데이터)를 입력받아, 다양한 로봇 핸드의 관절 각도로 변환하는 **리타게팅 최적화 라이브러리**이다. AnyTeleop 프로젝트에서 리타게팅 엔진만 분리하여 독립 패키지로 만든 것이다.

---

## 2. 코드 아키텍처

### 2.1 핵심 클래스 구조

```
dex_retargeting/
├── retargeting_config.py    → RetargetingConfig (설정 관리)
├── seq_retarget.py          → SeqRetargeting (메인 인터페이스)
├── optimizer.py             → Optimizer 기반 클래스
│   ├── VectorOptimizer      → 벡터 방향 매칭 (3세대 기본)
│   ├── PositionOptimizer    → 끝점 위치 매칭 (2세대)
│   └── DexPilotOptimizer    → DexPilot 스타일 (3세대 고급)
├── robot_wrapper.py         → RobotWrapper (pinocchio 기반 기구학)
└── constants.py             → 상수 정의
```

### 2.2 데이터 흐름 파이프라인

```
[입력 소스]                    [리타게팅 엔진]              [출력]
                                    
mp4 영상 ──→ MediaPipe ──┐                                
webcam ──→ MediaPipe ──→ 21개 키포인트 ──→ SeqRetargeting ──→ 로봇 관절 각도
DexYCB 데이터셋 ──────────┘     │                              │
                                ├── RetargetingConfig          │
                                ├── RobotWrapper (FK/Jacobian) │
                                └── Optimizer (비용 최소화)     │
                                                               ↓
                                                         SAPIEN 시각화
                                                         또는 실제 로봇
```

### 2.3 실행 흐름 (단일 프레임)

```python
# 1. 설정 로드
config = RetargetingConfig.load_from_file(yaml_path)
retargeting = config.build()  # SeqRetargeting 인스턴스 생성

# 2. 인간 키포인트 입력 (21개 또는 그 부분집합)
# MediaPipe 또는 MANO에서 추출된 3D 키포인트
human_keypoints = detect_hand(image)  # shape: (N, 3)

# 3. 리타게팅 실행
robot_qpos = retargeting.retarget(human_keypoints)  # shape: (num_joints,)

# 4. 로봇에 적용
robot.set_qpos(robot_qpos)
```

---

## 3. 세 가지 Optimizer의 수학적 이론

### 3.1 VectorOptimizer (벡터 방향 매칭) — 3세대 기본

**핵심 아이디어:** 인간 손과 로봇 손에서 대응되는 "벡터"(예: 손목→검지 끝, 손목→엄지 끝)의 **방향**을 일치시킨다.

**벡터 정의:**

```
인간 손에서 벡터 v^h_i 정의:
  v^h_1 = 손목 → 엄지 끝 (정규화)
  v^h_2 = 손목 → 검지 끝 (정규화)
  v^h_3 = 손목 → 중지 끝 (정규화)
  ...
  v^h_k = 손목 → 새끼 끝 (정규화)
  + 추가 벡터: 관절 간 벡터 (MCP→PIP, PIP→DIP 등)

로봇 손에서 대응 벡터 v^r_i(θ) 정의:
  로봇 관절 각도 θ가 주어지면, FK(Forward Kinematics)로
  각 링크의 위치를 계산하고 대응 벡터를 구함
```

**최적화 문제:**

```
minimize  Σ_i  w_i × || v^h_i / ||v^h_i|| - v^r_i(θ) / ||v^r_i(θ)|| ||²
   θ

subject to:  θ_min ≤ θ ≤ θ_max  (관절 범위)
```

**직관:** "인간 손에서 손목→검지 방향이 오른쪽 위를 가리키면, 로봇 손에서도 같은 방향을 가리켜라." 벡터의 **길이**는 무시하고 **방향**만 맞추므로, 인간과 로봇의 손가락 길이가 달라도 작동한다.

**솔버:** scipy.minimize의 L-BFGS-B (bound 제약 지원), 이전 프레임의 해를 초기값으로 사용(warm-start)하여 실시간 성능 확보.

**장점:**
- 손가락 길이 차이에 강건 (방향만 매칭하므로)
- 계산 비용이 상대적으로 낮음
- 텔레오퍼레이션에 가장 적합 (실시간)

**단점:**
- 끝점의 절대 위치를 보장하지 않음 (방향만 맞으면 거리가 달라도 OK)
- 손가락 간 거리(pinch 등)가 정확히 보존되지 않을 수 있음

---

### 3.2 PositionOptimizer (끝점 위치 매칭) — 2세대

**핵심 아이디어:** 인간 손의 키포인트(fingertip, 관절)의 **절대 3D 위치**를 로봇 손에서 재현한다.

**최적화 문제:**

```
minimize  Σ_i  w_i × || p^h_i - p^r_i(θ) ||²
   θ

p^h_i: 인간 키포인트 i의 3D 위치 (스케일 조정 후)
p^r_i(θ): 로봇 관절 각도 θ에서 대응 키포인트의 FK 위치

subject to:  θ_min ≤ θ ≤ θ_max
```

**직관:** "인간 검지 끝이 (x, y, z)에 있으면, 로봇 검지 끝도 (x, y, z)에 가라." From One Hand to Multiple Hands (Qin et al., 2022) 논문의 방법론.

**스케일 문제:** 인간 손과 로봇 손의 크기가 다르므로, 인간 키포인트를 로봇 workspace에 맞게 스케일링하는 전처리가 필요하다. 이 스케일링 파라미터가 성능에 큰 영향을 미침.

**장점:**
- 끝점 위치가 직접적으로 매칭 → 물체와의 접촉 위치가 정확
- hand-object interaction 데이터(DexYCB 등) 처리에 적합

**단점:**
- 스케일 차이에 민감
- 자기충돌 방지가 내장되어 있지 않음
- IK 다해성 문제 (같은 끝점 위치에 여러 관절 해 가능)

---

### 3.3 DexPilotOptimizer (DexPilot 스타일) — 3세대 고급

**핵심 아이디어:** VectorOptimizer를 기반으로, DexPilot 논문의 핵심 기여인 **손가락 쌍 간 거리 매칭**을 추가한다.

**추가되는 비용 항:**

```
기본 벡터 매칭 (VectorOptimizer와 동일):
  L_vector = Σ_i w_i × || v^h_i_norm - v^r_i(θ)_norm ||²

DexPilot 추가 — 손가락 쌍 거리 매칭:
  L_pairwise = Σ_{i,j} w_{ij} × (d^h_{ij} - d^r_{ij}(θ))²

  d^h_{ij}: 인간 손에서 fingertip i와 fingertip j 사이의 거리
  d^r_{ij}(θ): 로봇 손에서 fingertip i와 fingertip j 사이의 거리

전체 비용:
  L = L_vector + λ × L_pairwise
```

**왜 이게 중요한가:** 예를 들어 엄지와 검지로 물체를 집는 pinch grasp에서, 두 손가락 끝 사이의 거리가 핵심이다. VectorOptimizer는 각 손가락의 방향은 맞추지만 두 손가락 사이의 거리까지는 보장하지 않는다. DexPilot의 pairwise distance 항이 이 간극을 메운다.

**특수 처리 — 임계값 기반 가중치:**

```
DexPilot에서는 d^h_{ij}가 작을 때(손가락이 가까울 때)
가중치 w_{ij}를 크게 하여 pinch 형태를 강하게 보존한다.

if d^h_{ij} < threshold:
    w_{ij} = w_close  (큰 값)
else:
    w_{ij} = w_far    (작은 값)

→ 손가락이 "거의 닿을 듯한" 상태를 정확히 재현하는 것이 핵심
```

**장점:**
- 파지 형태 보존이 가장 뛰어남 (pinch, power grasp 형태 유지)
- 텔레오퍼레이션에서 가장 높은 조작 성공률

**단점:**
- VectorOptimizer보다 계산 비용 높음 (pairwise 항 추가)
- 임계값 파라미터 튜닝 필요

---

## 4. 설정 파일 (YAML) 구조

```yaml
type: DexPilot  # 또는 vector, position

# 로봇 정의
urdf_path: robot/allegro/allegro_hand_right.urdf

# 리타게팅 벡터/포인트 매핑
target_joint_names:  # 최적화 대상 관절
  - joint_0
  - joint_1
  ...

# 인간→로봇 키포인트 대응
target_link_human_indices:  # 인간 키포인트 인덱스 (MediaPipe 21점 중)
  - [0, 4]    # 손목(0) → 엄지끝(4): 벡터 1
  - [0, 8]    # 손목(0) → 검지끝(8): 벡터 2
  ...

target_link_names:  # 대응하는 로봇 링크
  - [palm_link, thumb_tip]
  - [palm_link, index_tip]
  ...

# 최적화 파라미터
scaling: 1.0
low: [-0.5, -0.3, ...]   # 관절 하한
high: [0.5, 0.3, ...]    # 관절 상한
```

**새 로봇을 추가하려면:** YAML에서 URDF 경로와 키포인트 매핑을 변경하면 된다. 최적화 엔진 자체는 수정 불필요.

---

## 5. 우리 환경(Manus Glove → DG-5F) 적용을 위한 허들 분석

### 허들 1: DG-5F URDF 준비

```
상태: dex-retargeting에 DG-5F가 포함되어 있지 않음
필요 작업:
  1. Tesollo DG-5F의 URDF 파일 확보 또는 작성
  2. 관절 이름, 관절 축, 관절 범위, 링크 길이가 실제와 일치하는지 검증
  3. 충돌 메시(선택, 자기충돌 검사용)

난이도: 중 (URDF가 이미 있으면 낮음, 없으면 높음)
```

### 허들 2: DG-5F 키포인트 매핑 정의

```
상태: 대응 관계를 새로 정의해야 함
필요 작업:
  1. DG-5F의 각 fingertip, 관절의 링크 이름 확인
  2. MediaPipe 21점(또는 Manus 21점)과 DG-5F 링크의 대응 테이블 작성
  3. YAML 설정 파일 작성

예시:
  MediaPipe/Manus point 0 (wrist)    ↔ DG-5F palm_link
  MediaPipe/Manus point 4 (thumb tip) ↔ DG-5F thumb_fingertip
  MediaPipe/Manus point 8 (index tip) ↔ DG-5F index_fingertip
  ...

난이도: 중 (DG-5F 기구학 구조 파악이 선행되어야 함)
```

### 허들 3: Manus Glove 입력 연결

```
상태: dex-retargeting은 MediaPipe(카메라)를 기본 입력으로 가정
필요 작업:
  1. Manus SDK의 skeleton 출력(21 joints 3D 위치)을 MediaPipe와 동일한 포맷으로 변환
  2. 좌표계 변환 (Manus → dex-retargeting 표준)
  3. 기존 MediaPipe 포즈 감지 부분을 Manus 입력으로 교체

변환의 핵심:
  MediaPipe 21점과 Manus skeleton 21점의 관절 정의가 다를 수 있음
  → 매핑 테이블이 필요

난이도: 중-하 (포맷 변환 수준, 알고리즘 변경 불필요)
```

### 허들 4: 실시간 성능

```
상태: dex-retargeting은 scipy.minimize 기반으로 Python에서 실행
필요 작업:
  1. warm-start가 잘 작동하는지 확인 (이전 해를 초기값으로)
  2. DG-5F의 DOF 수에 따른 최적화 시간 측정
  3. 필요시 최적화 반복 횟수 제한 또는 솔버 교체

기대치:
  Allegro (16 DOF): ~2-5ms/프레임 (충분)
  DG-5F (~20 DOF): ~5-10ms/프레임 (예상, 확인 필요)

난이도: 낮음 (warm-start가 기본 내장)
```

### 허들 5: Isaac Sim 연동

```
상태: dex-retargeting은 SAPIEN 렌더러를 기본 사용
필요 작업:
  1. dex-retargeting의 출력(관절 각도)을 Isaac Sim의 DG-5F 모델에 전달
  2. 관절 순서(joint ordering) 차이 처리
     → dex-retargeting은 이 문제를 인지하고 있으며,
        joint name 기반 매핑을 권장함
  3. Isaac Sim의 joint name과 dex-retargeting의 joint name 대응 확인

난이도: 중-하 (관절 순서 매핑만 정확히 하면 됨)
```

---

## 6. 단계별 적용 계획

### Phase 0: 기본 동작 확인 (mp4 → 기존 로봇)

```
목표: dex-retargeting이 우리 환경에서 설치/실행되는지 확인

단계:
  1. pip install dex_retargeting
  2. 제공된 mp4 예제로 vector_retargeting 실행
     python detect_from_video.py \
       --robot-name allegro \
       --video-path data/human_hand_video.mp4 \
       --retargeting-type dexpilot
  3. SAPIEN 뷰어에서 Allegro Hand가 영상 속 손을 따라하는지 확인

허들: 없음 (예제 그대로 실행)
소요: 1-2시간
```

### Phase 1: 안드로이드 카메라(IP Webcam) → 기존 로봇 (sim)

```
목표: 실시간 카메라 입력으로 리타게팅 동작 확인

단계:
  1. IP Webcam 앱으로 안드로이드 카메라를 네트워크 스트림으로 전송
  2. show_realtime_retargeting.py에서 카메라 소스를 IP Webcam URL로 변경
     → cv2.VideoCapture("http://<phone-ip>:8080/video") 등
  3. MediaPipe가 폰 카메라 영상에서 손 추적 → Allegro 실시간 리타게팅

허들: IP Webcam의 지연(latency), MediaPipe 추적 품질
소요: 반나절
```

### Phase 2: 웹캠 → Isaac Sim (DG-5F)

```
목표: DG-5F를 Isaac Sim에서 리타게팅으로 제어

단계:
  1. DG-5F URDF를 dex-retargeting에 등록
     → dex-urdf/ 디렉토리에 추가 또는 외부 경로 지정
  2. DG-5F 전용 YAML 설정 파일 작성
     → 키포인트 매핑, 관절 범위, 벡터 정의
  3. detect_from_video.py 또는 show_realtime_retargeting.py에서
     --robot-name을 DG-5F로 변경
  4. SAPIEN 대신 Isaac Sim에 관절 각도 전달
     → 관절 이름 기반 매핑으로 순서 차이 해결
  5. Isaac Sim에서 DG-5F가 카메라 입력을 따라하는지 확인

허들: URDF 준비, 키포인트 매핑 (허들 1, 2)
소요: 1-2주
```

### Phase 3: Manus Glove → Isaac Sim (DG-5F)

```
목표: 최종 목표 환경에서 리타게팅 동작

단계:
  1. Manus SDK skeleton 출력을 MediaPipe 포맷으로 변환하는 어댑터 개발
  2. show_realtime_retargeting.py의 MediaPipe 부분을 Manus 어댑터로 교체
  3. 나머지 파이프라인(Optimizer → Isaac Sim)은 Phase 2와 동일
  4. Manus 착용 후 실시간 리타게팅 확인

허들: Manus-MediaPipe 관절 매핑 (허들 3)
소요: 1주
```

### Phase 4: Manus Glove → 실제 DG-5F

```
목표: 실제 로봇에서 리타게팅 원격조작

단계:
  1. Phase 3의 관절 각도 출력을 DG-5F ROS2 드라이버에 연결
  2. 안전 제약 추가 (관절 속도 제한, 비상 정지 등)
  3. 실제 DG-5F에서 Manus 입력을 따라하는지 확인

허들: 실시간 성능, 안전 (허들 4)
소요: 1주
```

---

## 7. 세 가지 Optimizer 중 어떤 것을 쓸 것인가

### 용도별 권장

| 용도 | 권장 Optimizer | 이유 |
|---|---|---|
| **실시간 텔레오퍼레이션** | **DexPilotOptimizer** | 파지 형태 보존이 가장 우수, pairwise distance로 pinch 정확 |
| 시연 데이터 후처리 | PositionOptimizer | 끝점 위치가 중요한 hand-object 데이터 변환 |
| 빠른 프로토타이핑 | VectorOptimizer | 가장 간단하고 빠름, 초기 테스트용 |

기봉님의 주 용도가 **실시간 텔레오퍼레이션**이므로, **DexPilotOptimizer를 메인으로 사용**하되, VectorOptimizer를 비교 기준으로 함께 테스트하는 것을 권장한다.

### Retargeting Objective 분석(Yu et al.)과의 연결

dex-retargeting의 DexPilotOptimizer에 **추가 비용 항**을 넣어 고도화할 수 있다:

```python
# 기존 DexPilotOptimizer 비용
L = L_vector + λ_pair × L_pairwise

# Yu et al. 기반 추가 항
L += λ_orient × L_fingertip_orientation   # 손가락 끝 방향 (DIP→tip 벡터)
L += λ_collision × L_self_collision        # 자기충돌 페널티
L += λ_smooth × L_temporal_smoothness      # 시간적 매끄러움

# 과제별 가중치 프리셋으로 전환 가능
```

이 확장은 dex-retargeting의 Optimizer 클래스를 상속하여 CustomOptimizer를 만들면 깔끔하게 구현 가능하다.

---

## 8. 핵심 요약

### dex-retargeting은 무엇인가

AnyTeleop의 리타게팅 엔진을 독립 라이브러리로 분리한 것. 3가지 최적화 방식(Vector, Position, DexPilot)을 제공하며, YAML 설정만 바꾸면 다양한 로봇 핸드에 적용 가능.

### 수학적 핵심

모든 Optimizer는 **"인간 손의 기하학적 특징(벡터 방향 또는 끝점 위치)과 로봇 손의 대응 특징 간 차이를 최소화하는 관절 각도 θ를 찾는 비선형 최적화"**이다. DexPilot이 가장 고급으로, 손가락 쌍 간 거리 보존을 추가하여 파지 형태를 정확히 재현한다.

### 우리 환경 적용의 핵심 허들

1. **DG-5F URDF 확보** (가장 중요, 없으면 처음부터 작성 필요)
2. **DG-5F 키포인트 매핑 정의** (어떤 링크가 어떤 키포인트에 대응하는지)
3. **Manus → MediaPipe 포맷 변환** (관절 정의 차이 해소)
4. 나머지(실시간 성능, Isaac Sim 연동)는 라이브러리가 이미 지원하므로 비교적 쉬움

### 권장 진행 순서

```
Phase 0 (2시간):   mp4 예제 → Allegro (기본 동작 확인)
Phase 1 (반나절):  IP Webcam → Allegro sim (실시간 확인)
Phase 2 (1-2주):   웹캠 → Isaac Sim DG-5F (URDF+매핑 개발)
Phase 3 (1주):     Manus → Isaac Sim DG-5F (입력 교체)
Phase 4 (1주):     Manus → 실제 DG-5F (실기 연동)
```

Phase 0-1은 dex-retargeting 자체를 이해하고 동작을 확인하는 단계이고, Phase 2에서 DG-5F 적용의 핵심 작업(URDF, 매핑)이 집중된다. Phase 2만 넘기면 Phase 3-4는 입출력 교체 수준이다.
