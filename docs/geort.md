# GeoRT 코드 분석 및 적용 계획

**작성일:** 2026-04-09
**대상:** https://github.com/facebookresearch/GeoRT
**목적:** 코드 구조·수학적 이론 분석 + Manus Glove → DG-5F 적용 전략 수립

---

## 1. 저장소 개요

### 1.1 프로젝트 정보

| 항목 | 내용 |
|---|---|
| 저자 | Zhao-Heng Yin, Changhao Wang, Luis Pineda, Krishna Bodduluri, Tingfan Wu, Pieter Abbeel, Mustafa Mukadam (Meta AI / FAIR + UC Berkeley) |
| 출처 논문 | *Geometric Retargeting: A Principled, Ultrafast Neural Hand Retargeting Algorithm* (arXiv:2503.07541, 2025-03) |
| 프로젝트 페이지 | https://zhaohengyin.github.io/geort/ |
| 라이선스 | **CC-BY-NC 4.0** (비상업적 연구용만 허용) |
| 설치 | 소스 클론 후 `pip install -e .` (PyPI 패키지 없음) |
| 로봇 기구학 | SAPIEN + Pinocchio (학습 시), 추론 시는 학습된 MLP |
| 시각화 | SAPIEN 렌더러 (mocap 평가 스크립트 내장) |
| 지원 핸드 | **Allegro Hand 좌/우 만 기본 제공**, 다른 핸드는 사용자가 URDF + JSON 작성 후 재학습 필요 |
| 상태 | 2025-03-04 마지막 커밋, ★~167, 활발하지 않은 연구 코드 드롭 |

### 1.2 이 저장소가 하는 것

GeoRT는 **사람 손가락 끝 키포인트 → 로봇 손 관절 각도** 변환을 수행하는 리타게팅 라이브러리이다. dex-retargeting과 같은 목적이지만 **접근 방식이 근본적으로 다르다**:

> dex-retargeting: 매 프레임 비선형 최적화 (scipy.minimize)
> GeoRT: 오프라인에서 작은 MLP를 학습 → **추론 시 단 한 번의 forward pass**

핵심 메시지: **"리타게팅을 1 kHz로 풀어라"**. 매 프레임 QP/SLSQP를 푸는 대신, 사람-로봇 손의 기하학적 대응을 미분가능한 손실로 정의해 작은 신경망에 한 번 baking해 둔다. DexGen (Meta의 텔레오퍼레이션 시스템)의 리타게팅 모듈로 개발됨.

---

## 2. 코드 아키텍처

### 2.1 핵심 파일 구조

```
GeoRT/
├── assets/                     → URDF + 메쉬 (Allegro 포함)
├── checkpoint/                 → 학습된 체크포인트 출력 위치
├── data/                       → 수집한 사람 mocap (.npy)
├── geort/
│   ├── __init__.py             → 공개 API: load_model, save_human_data
│   ├── config/
│   │   ├── allegro_right.json  → 핸드 정의 (URDF 경로, joint_order, fingertip 매핑)
│   │   ├── allegro_left.json
│   │   └── template.py         → 새 핸드 추가용 dict 템플릿
│   ├── env/                    → SAPIEN 환경 / 핸드 wrapper
│   ├── mocap/
│   │   ├── mediapipe_mocap.py  → MediaPipe → 캐노니컬 손바닥 프레임 변환
│   │   ├── manus_mocap.py      → Manus Glove (ROS2) → 캐노니컬 프레임
│   │   ├── manus_client/       → Manus ROS2 클라이언트
│   │   ├── replay_mocap.py     → .npy 재생용
│   │   └── README.md           → "MediaPipe는 실전 teleop 금지" 경고 포함
│   ├── dataset.py              → 사람/로봇 point cloud 데이터셋
│   ├── model.py                → FKModel, IKModel (per-finger MLP)
│   ├── trainer.py              → GeoRTTrainer (학습 엔트리)
│   ├── loss.py                 → Chamfer distance 등 기하학적 손실
│   ├── export.py               → 학습된 모델 로드 + .forward() 추론 wrapper
│   └── utils/
├── requirements.txt
└── setup.py
```

### 2.2 데이터 흐름 파이프라인

```
[학습 단계 — 1회만]                              [추론 단계 — 매 프레임]

URDF + joint 범위                                Manus Glove (또는 MediaPipe)
   │                                                │
   ▼                                                ▼
랜덤 joint 100k 샘플링                          21점 키포인트 (N, 3)
   │                                                │
   ▼                                                ▼
SAPIEN/Pinocchio FK                             손바닥 캐노니컬 프레임 변환
   │                                                │
   ▼                                                ▼
로봇 fingertip point cloud ─────┐               IKModel.forward(keypoints)
                                 │                │   ↓
사람 mocap 5분 수집 (∼5,000+ frames)               qpos 정규화 [-1,1]
   │                              │                │   ↓
   ▼                              │                URDF 한계로 역정규화
사람 fingertip point cloud ──────┤                │
                                  │                ▼
                                  ▼            로봇 명령 (radians)
              5가지 기하학 손실
              ┌─────────────────────────┐
              │ 1. Motion Preservation  │
              │ 2. C-space Coverage     │
              │ 3. High Flatness        │
              │ 4. Pinch Correspondence │
              │ 5. Self-Collision       │
              └────────────┬────────────┘
                           │
                           ▼
                       AdamW 학습
                  (1∼5분, RTX 3060)
                           │
                           ▼
                   IKModel 체크포인트
```

### 2.3 실행 흐름 (한 프레임 추론)

```python
import geort

# 1. 미리 학습된 retargeter 로드
model = geort.load_model(tag="geort_1", epoch=50)

# 2. 사람 키포인트 입력 (캐노니컬 손바닥 프레임으로 정렬되어 있어야 함)
#    [N, 3] np.ndarray, MediaPipe 21점 또는 Manus 21점
keypoints = mocap.get()

# 3. 한 번의 forward pass로 qpos 추출
qpos = model.forward(keypoints)   # shape: (n_joints,) in radians

# 4. 로봇 명령
robot.command(qpos)
```

**dex-retargeting과의 본질적 차이:** 위 코드에서 `model.forward()`는 단순한 MLP 추론(<1ms)이다. 매 프레임 SLSQP/L-BFGS를 돌리는 dex-retargeting과 달리 **runtime 최적화가 전혀 없다**. 이것이 1 kHz 처리 속도의 비결이다.

---

## 3. 다섯 가지 기하학적 손실의 수학적 이론

GeoRT의 핵심은 **사람 손과 로봇 손의 모양/길이가 달라도, 기하학적 의미만 보존하면 된다**는 가정이다. 이를 5가지 미분 가능한 손실로 표현한다.

### 3.1 손실 ① — Motion Preservation (방향 보존)

**핵심 아이디어:** "사람 검지가 위로 움직이면 로봇 검지도 위로 움직여야 한다."

**수식:**

```
사람 키포인트 변화: Δp^h = p^h(t+1) - p^h(t)
로봇 fingertip 변화: Δp^r(θ) = FK(f(p^h(t+1))) - FK(f(p^h(t)))

L_dir = 1 - cos_sim(Δp^h, Δp^r(θ))

f(·): IKModel (학습 대상 — 사람 키포인트 → 로봇 qpos)
FK(·): 학습된 FKModel (로봇 qpos → fingertip 위치)
```

**직관:** 절대 위치는 무시하고, **방향성**만 매칭. dex-retargeting의 VectorOptimizer와 유사한 아이디어이지만, **이산 차분으로 미분가능**하게 풀어 신경망 학습 손실로 활용.

### 3.2 손실 ② — C-space Coverage (작업공간 커버리지)

**핵심 아이디어:** "사람의 모든 손가락 자세에 대해 로봇이 *대응되는* 자세를 가져야 한다 (mode collapse 방지)."

**수식 (Chamfer distance):**

```
P^r = {FK(f(p^h_i)) : i = 1..N}    ← 사람 데이터를 통과시킨 로봇 fingertip cloud
P^r_random = {FK(θ_j) : θ_j ~ U(low, high)}    ← 로봇의 진짜 도달 가능 cloud

L_cover = Σ_p ∈ P^r_random  min_{q ∈ P^r}  ||p - q||²
        + Σ_q ∈ P^r        min_{p ∈ P^r_random}  ||p - q||²
```

**직관:** 만약 IKModel이 모든 사람 입력을 로봇 자세 한 점으로 매핑하면 다른 손실은 0에 가까울 수 있어도 Chamfer가 폭증한다. 따라서 **로봇의 도달 가능한 작업공간이 사람 입력 분포에 비례하게 펼쳐지도록** 강제한다.

**가중치:** trainer.py에서 80배 가중 (가장 큰 비중).

### 3.3 손실 ③ — High Flatness (지역적 평탄성)

**핵심 아이디어:** "입력에 작은 노이즈가 있어도 출력이 격렬하게 흔들리지 않아야 한다."

**수식 (이산 라플라시안):**

```
L_flat = || FK(f(x + d)) + FK(f(x - d)) - 2 · FK(f(x)) ||²

d: 작은 무작위 perturbation
```

**직관:** 출력이 **선형에 가까울수록** 이 값이 작아진다. mocap 노이즈가 폭발적으로 증폭되는 것을 막아 부드러운 텔레오퍼레이션을 보장. 시간 평활화 없이도 깔끔한 명령이 나오는 이유.

**dex-retargeting과의 차이:** dex-retargeting은 매 프레임 EMA / low-pass로 부드럽게 만든다. GeoRT는 **모델 자체가 부드럽도록 학습 시점에 강제**한다.

### 3.4 손실 ④ — Pinch Correspondence (집기 대응)

**핵심 아이디어:** "사람이 엄지-검지를 닿게 하면, 로봇 엄지-검지도 닿아야 한다."

**수식:**

```
L_pinch = Σ_{(i,j) ∈ pinch_pairs}  || ||p^h_i - p^h_j|| - ||FK_i(f(x)) - FK_j(f(x))|| ||²
```

**직관:** dex-retargeting의 DexPilotOptimizer가 매 프레임 푸는 pairwise distance 손실과 동일한 아이디어이지만, 학습 시점에 모델에 baking된다. 추론 시 추가 비용 없음.

**가중치:** λ ∈ [10³, 10⁶] (논문 설정), 매우 큰 가중치 → pinch 대응이 핵심 보존 항.

### 3.5 손실 ⑤ — Self-Collision Prevention (자기충돌 회피)

**핵심 아이디어:** "로봇 손가락이 서로 관통하면 안 된다."

**수식:**

```
L_col = C(f(x))

C(·): 사전 학습된 충돌 분류기 (qpos → collision likelihood)
```

**직관:** 별도로 학습한 binary classifier (qpos → 충돌 여부)를 사용해 학습 손실에 페널티로 추가.

**⚠ 주의:** 현재 공개된 `trainer.py`에서는 **이 손실이 비활성화 (placeholder)** 상태이다. 즉 **공개 코드는 실제로 5가지 중 4가지만 학습**한다. 자기충돌 회피가 필요하면 사용자가 직접 분류기를 만들고 활성화해야 함.

### 3.6 전체 손실 함수

```
L = L_dir
  + λ₁ · L_cover    (λ₁ ≈ 10∼100, trainer.py에서는 80)
  + λ₂ · L_flat     (λ₂ ≈ 1, trainer.py에서는 0.1)
  + λ₃ · L_pinch    (λ₃ ≈ 10³ ∼ 10⁶)
  + λ₄ · L_col      (λ₄ ≈ 10⁻⁴ ∼ 10⁻², 공개 코드 비활성)
```

**감독 없이 학습:** 사람-로봇 페어 데이터가 필요 없다. 사람 손 mocap 5분 + 로봇 URDF만으로 충분.

---

## 4. 모델 구조 및 학습 절차

### 4.1 두 단계 신경망

**Stage A — FKModel (Forward Kinematics 신경망)**

```python
# 손가락 별 작은 MLP
class FKModel(nn.ModuleList):
    fingers = [Sequential(
        Linear(n_joint, hidden),
        LeakyReLU(),
        BatchNorm1d(hidden),
        ...
        Linear(hidden, 3),   # → fingertip 3D 위치
    ) for _ in range(n_fingers)]
```

- 학습 데이터: 100,000개의 무작위 joint 샘플 + SAPIEN/Pinocchio의 정답 FK
- 손실: MSE
- 200 epoch, batch 256, lr = 5e-4
- **목적:** 미분 가능한 FK가 필요. URDF 분석 FK는 미분 불가능 (또는 chain rule 깨기 쉬움)하므로 신경망으로 근사.

**Stage B — IKModel (역기구학, 본 retargeter)**

```python
class IKModel(nn.ModuleList):
    fingers = [Sequential(
        Linear(3, hidden),
        LeakyReLU(),
        BatchNorm1d(hidden),
        ...
        Linear(hidden, n_joint),
        Tanh(),    # → [-1, 1] 정규화된 관절 각도
    ) for _ in range(n_fingers)]
```

- FKModel은 동결, IKModel과 결합하여 5가지 기하학 손실로 학습
- 옵티마이저: AdamW, lr = 1e-4
- Batch: 2048 (사람 point cloud), Epochs: 30~50 (default 200, early stop)
- **학습 시간: RTX 3060에서 1~5분**

### 4.2 정규화 / 역정규화

- 출력은 Tanh로 [-1, 1] 범위
- 추론 시 URDF의 joint limit으로 unnormalize → 실제 radian 값
- **URDF의 joint limit이 잘못되어 있으면 출력이 잘못된다** (자주 발생하는 함정)

### 4.3 학습 명령

```bash
python ./geort/trainer.py \
    -hand allegro_right \
    -human_data human_alex \
    -ckpt_tag geort_1
```

체크포인트는 `checkpoint/geort_1/` 와 `checkpoint/geort_1/last/` 양쪽에 저장.

---

## 5. 입력/출력 사양 (적용 시 가장 중요)

### 5.1 입력 — 캐노니컬 손바닥 프레임 (필수)

GeoRT는 입력 키포인트가 **반드시 손바닥 중심에 정렬된 표준 좌표계**에 있어야 한다. 좌표 규약은 엄격히 정해져 있다:

```
+Y 축: 손바닥 중심 → 엄지
+Z 축: 손바닥 중심 → 중지
+X 축: 손바닥 법선 (오른손이면 손등 반대 방향)
원점:  손목 (landmark 0)
```

**참고용 변환 예시 (`geort/mocap/mediapipe_mocap.py`):**
1. landmark 0 (wrist)을 원점으로 사용
2. z축 = landmark 9 (middle MCP) - landmark 0
3. 보조 y축 = landmark 5 (index MCP) ↔ landmark 13 (ring MCP)
4. x = y × z, 그 후 정규직교화
5. 4×4 변환행렬 구성 후 모든 21점에 적용

**왜 중요한가:** README가 명시적으로 "리타게팅 실패의 #1 원인이 좌표계 불일치"라고 경고한다. dex-retargeting의 MANO 변환과 유사한 역할이지만 **축 규약이 다르다** (dex-retargeting은 OPERATOR2MANO 행렬을 사용; GeoRT는 +Y=thumb, +Z=middle).

### 5.2 입력 데이터 타입

| 항목 | 사양 |
|---|---|
| 형상 | `(N, 3)` numpy array — MediaPipe면 N=21 |
| 단위 | meters (URDF와 일치해야 함) |
| 좌표계 | 손바닥 캐노니컬 프레임 (위 5.1) |
| 추론에 실제로 사용되는 인덱스 | JSON 설정의 `human_hand_id`만 picks. 예: Allegro `[8, 12, 16, 4]` (Index/Middle/Ring/Thumb tip) |

**즉, 21점 전부를 넣어도 4점 만 사용**한다. 하지만 캐노니컬 프레임 변환을 위해서는 wrist + index_MCP + middle_MCP + ring_MCP가 필요하다.

### 5.3 출력

| 항목 | 사양 |
|---|---|
| 형상 | 1D numpy array, 길이 = `joint_order` 길이 |
| 단위 | radians (URDF native) |
| 정규화 | **이미 역정규화 완료** ([-1,1] → 실제 radian으로 변환됨) |
| 순서 | JSON 설정의 `joint_order` 리스트 순서 |
| 의미 | qpos (관절 각도). IK 타겟이 아니라 직접 명령 가능 |
| 지연 | <1ms 추론. 1 kHz teleop 가능 (mocap 소스 속도가 병목) |

### 5.4 지원 mocap 소스

| 소스 | 파일 | 추천도 |
|---|---|---|
| **Manus Gloves** | `geort/mocap/manus_mocap.py` (ROS2 + Windows manus_client) | ✅ 권장 (실전용) |
| MediaPipe + RealSense | `mediapipe_mocap.py`, `mediapipe_evaluation.py` | ❌ **공식 비추천** ("DO NOT use MediaPipe for real teleop" — 손목 회전 시 불안정) |
| 재생 (replay) | `replay_mocap.py`, `replay_evaluation.py` | 평가/디버깅용 |
| Apple Vision Pro / Quest | 지원 없음 | 사용자 직접 어댑터 작성 |

---

## 6. 설정 파일 (JSON) 구조

GeoRT는 dex-retargeting과 달리 **YAML이 아닌 JSON**을 사용하며, 핸드 정의가 단순하다 (`geort/config/allegro_right.json` 발췌):

```json
{
    "name": "allegro_right",
    "urdf_path": "assets/allegro/allegro_right.urdf",
    "base_link": "palm_link",
    "joint_order": [
        "joint_0.0", "joint_1.0", "joint_2.0", "joint_3.0",
        "joint_4.0", "joint_5.0", "joint_6.0", "joint_7.0",
        "joint_8.0", "joint_9.0", "joint_10.0", "joint_11.0",
        "joint_12.0", "joint_13.0", "joint_14.0", "joint_15.0"
    ],
    "fingertip_link": [
        {
            "name": "index",
            "link": "link_3.0_tip",
            "joint": ["joint_0.0", "joint_1.0", "joint_2.0", "joint_3.0"],
            "center_offset": [0, 0, 0],
            "human_hand_id": 8
        },
        {
            "name": "middle",
            "link": "link_7.0_tip",
            "joint": ["joint_4.0", "joint_5.0", "joint_6.0", "joint_7.0"],
            "center_offset": [0, 0, 0],
            "human_hand_id": 12
        },
        {
            "name": "ring",
            "link": "link_11.0_tip",
            "joint": ["joint_8.0", "joint_9.0", "joint_10.0", "joint_11.0"],
            "center_offset": [0, 0, 0],
            "human_hand_id": 16
        },
        {
            "name": "thumb",
            "link": "link_15.0_tip",
            "joint": ["joint_12.0", "joint_13.0", "joint_14.0", "joint_15.0"],
            "center_offset": [0, 0, 0],
            "human_hand_id": 4
        }
    ]
}
```

**주요 필드 의미:**
- `urdf_path`: assets 디렉토리 기준 상대 경로
- `base_link`: 손바닥 링크 이름 (캐노니컬 프레임 기준점)
- `joint_order`: 출력 qpos의 순서 (모든 활성 joint)
- `fingertip_link[]`: 각 손가락 정의
  - `link`: URDF의 fingertip link 이름
  - `joint[]`: 그 손가락에 속하는 joint 체인 (per-finger MLP에 사용)
  - `center_offset`: link origin → 실제 접촉점 보정
  - `human_hand_id`: 사람 키포인트 인덱스 (MediaPipe 4/8/12/16)

**새 핸드 추가 절차:**
1. URDF + 메쉬를 `assets/`에 추가
2. JSON 설정 작성 (`geort/config/`)
3. 사람 mocap 5분 수집 (`geort.save_human_data(...)`)
4. `python -m geort.trainer -hand <name> -human_data <tag> -ckpt_tag <ckpt>` 실행
5. 1∼5분 후 체크포인트 사용 가능

dex-retargeting처럼 "설정만 바꾸면 즉시 작동"이 아니라 **반드시 학습 단계가 필요**하다.

---

## 7. dex-retargeting과의 비교

| 항목 | **GeoRT** | **dex-retargeting** |
|---|---|---|
| **런타임 알고리즘** | MLP forward pass 1회 | 매 프레임 비선형 최적화 |
| **추론 속도** | ~1 kHz (RTX 3060) | ~100~300 Hz |
| **사전 학습 필요?** | **예** (사용자 별 1~5분) | 아니오 (즉시 사용) |
| **학습 데이터** | 사람 mocap 5분 + URDF (페어 불필요) | 없음 |
| **하이퍼파라미터 수** | ≤5 | ≥10 (옵티마이저 별 가중치, smoothing 등) |
| **출력 부드러움** | 학습 시점에 강제 (flatness loss) | 런타임 EMA / regularization |
| **Pinch 정확도** | pinch loss로 학습 시 baking | DexPilotOptimizer가 매 프레임 풀이 |
| **자기충돌 회피** | 사전 학습된 분류기 (공개 코드 비활성) | 옵티마이저 제약 |
| **기본 지원 핸드** | Allegro 좌/우 만 | Allegro, Shadow, LEAP, SVH, Ability, Inspire 등 다수 |
| **라이선스** | **CC-BY-NC 4.0 (상업 금지)** | MIT (상업 가능) |
| **PyPI 패키지** | 없음 (`pip install -e .`) | `pip install dex_retargeting` |
| **mocap 어댑터 내장** | MediaPipe, Manus (ROS2), replay | 없음 (사용자가 [N,3] 만 만들면 됨) |
| **설정 파일** | JSON | YAML |
| **활동성** | 2025-03-04 마지막 커밋, ★167 | 활발 유지 |

### GeoRT를 선택하는 이유

- **1 kHz 텔레오퍼레이션**이 필요한 경우 (토크 제어, 고대역 응용)
- **Manus 등 정확한 mocap**을 보유하고 사용자 별 5분 학습을 감수할 수 있는 경우
- **부드러운 출력** (QP 떨림 없는)이 필요한 연구 셋업
- 학술 연구 (CC-BY-NC 라이선스 허용)

### dex-retargeting을 선택하는 이유

- 즉시 사용 가능한 plug-and-play (학습 불요)
- **여러 로봇 핸드**를 다양하게 시험 (DG-5F도 포함시키기 용이)
- 상업 라이선스 필요 (MIT)
- 60~120 Hz teleop이 충분
- 활발한 커뮤니티 / PyPI 유지보수

---

## 8. 우리 환경(Manus Glove → DG-5F) 적용을 위한 허들 분석

### 허들 1: DG-5F URDF 정비 (가장 큰 함정)

```
상태: dex-retargeting과 동일하게 DG-5F 미지원

필요 작업:
  1. dg5f_right.urdf를 GeoRT의 assets/ 디렉토리에 복사
  2. ⚠ collision mesh 단순화 또는 제거 — README의 명시적 경고:
     "complex collision meshes frequently cause segfaults in SAPIEN"
  3. joint limit 정확성 검증 (저장 시점에 정규화에 사용됨)

난이도: 중 (URDF는 있으나 SAPIEN 호환성 검증 필요)
```

### 허들 2: DG-5F JSON 설정 작성

```
상태: 새로 정의해야 함 (Allegro만 기본 제공)

필요 작업:
  1. fingertip_link 5개 정의 (DG-5F는 5손가락):
     - thumb (rl_dg_1_tip), human_hand_id=4
     - index (rl_dg_2_tip), human_hand_id=8
     - middle (rl_dg_3_tip), human_hand_id=12
     - ring   (rl_dg_4_tip), human_hand_id=16
     - pinky  (rl_dg_5_tip), human_hand_id=20  ← 새로움
  2. 각 손가락의 joint chain 명시 (4 joint × 5 finger = 20 DoF)
  3. joint_order 정의 (rj_dg_1_1 ... rj_dg_5_4 순서)
  4. base_link = rl_dg_palm

⚠ 주의: 기본 제공 Allegro는 4-fingered 손이고 GeoRT의 모든 예제가 4 fingertip 가정.
       DG-5F는 5-fingered → JSON에 5번째 entry 추가하면 자동 동작할 가능성이 있으나
       (per-finger MLP 구조이므로) 검증되지 않음. 첫 시도 시 trainer 코드가
       n_finger를 hardcoded로 4로 가정하지 않는지 확인 필요.

난이도: 중-상 (구조 자체는 단순, 5-finger 가정 검증이 미지)
```

### 허들 3: 사람 mocap 데이터 수집

```
상태: 사용자 별 5분 데이터가 필요 (dex-retargeting과의 가장 큰 차이)

필요 작업:
  1. Manus Glove 착용
  2. 5분 동안 다음 동작 수행:
     - 각 손가락을 완전히 펴고 자유롭게 움직임 (각 손가락 작업공간 탐색)
     - 다양한 pinch 동작 (엄지-검지, 엄지-중지, ...)
     - 주먹 쥐기 / 펴기
  3. geort.save_human_data(arr, tag="human_<user>") 로 .npy 저장
  4. 약 5,000+ frame 권장

⚠ 캐노니컬 프레임 변환을 거쳐서 저장해야 함.
   Manus의 raw skeleton (월드 좌표) → +Y=thumb, +Z=middle 프레임 변환 코드 필요.

난이도: 중 (수집 자체는 쉬우나 좌표 변환 검증 필요)
```

### 허들 4: 학습 및 검증

```
상태: 1∼5분의 학습 시간 (RTX 3060 기준)

필요 작업:
  1. python -m geort.trainer -hand dg5f_right -human_data <tag> -ckpt_tag dg5f_v1
  2. 학습 로그에서 5가지 손실의 수렴 확인
  3. SAPIEN으로 시각화 (geort/mocap/replay_evaluation.py)
  4. 손목 회전, pinch, 주먹 쥐기 등 표준 동작 시 DG-5F가 자연스러운지 확인

⚠ Self-collision loss는 공개 코드에서 비활성. 활성화하려면:
  - DG-5F의 자기충돌 분류기 사전 학습 필요
  - 또는 SAPIEN 충돌 검사 API 직접 호출로 손실 함수 작성

난이도: 중
```

### 허들 5: Manus → 캐노니컬 프레임 어댑터

```
상태: GeoRT는 Manus mocap 어댑터를 제공하지만 ROS2 + Windows 환경 전제
      우리 프로젝트는 Linux + ROS2이므로 일부 호환성 이슈 가능

필요 작업:
  1. geort/mocap/manus_mocap_core.py 확인 후 우리 프로젝트의
     src/retarget_dev/sensing/manus/와 통합
  2. Manus 21점 → +Y=thumb, +Z=middle, +X=palm normal 프레임 변환
  3. 필요 시 Manus wrist quaternion을 활용해 SVD 단계 생략 (정확도 향상)

⚠ 본 프로젝트의 MANO 변환과 좌표 규약이 다름. 두 가지를 혼동하면 안 됨:
   - dex-retargeting: OPERATOR2MANO 행렬
   - GeoRT: +Y thumb / +Z middle / +X palm normal

난이도: 중 (sensing/manus/ 코드와 통합 시 분기 필요)
```

### 허들 6: 라이선스

```
상태: CC-BY-NC 4.0 (상업적 사용 금지)

영향:
  - 학술 연구 / 내부 프로토타이핑은 가능
  - 제품화나 상업 이전 불가능 → 향후 dex-retargeting으로 교체해야 함
  - 본 프로젝트 (tamp_ws)가 학술/내부 연구 용도이면 문제 없음

대응: 의사결정 단계에서 명시 필요
```

---

## 9. 단계별 적용 계획

### Phase 0: GeoRT 기본 동작 확인 (Allegro)

```
목표: 공식 예제 (Allegro + MediaPipe)로 GeoRT가 동작하는지 확인

단계:
  1. git clone https://github.com/facebookresearch/GeoRT
  2. conda env 생성 후 pip install -e .
  3. 사람 mocap 데이터 한 번 수집
     python geort/mocap/mediapipe_evaluation.py (혹은 사전 제공 .npy 활용)
  4. python -m geort.trainer -hand allegro_right -human_data <tag> -ckpt_tag test1
  5. SAPIEN 뷰어에서 학습 결과 시각화

소요: 2~4시간
```

### Phase 1: DG-5F URDF 등록 + JSON 설정 작성

```
목표: GeoRT가 DG-5F 핸드를 인식하고 SAPIEN에서 로드 가능

단계:
  1. dg5f_right.urdf와 메쉬를 GeoRT/assets/dg5f/에 복사
  2. collision mesh 단순화 (필요 시 trimesh로 convex hull로 교체)
  3. dg5f_right.json 설정 작성 (5 fingertip)
  4. SAPIEN으로 import 테스트 (FK 동작 확인)

소요: 1∼2일
허들: 5-finger 가정에 대한 trainer.py 검증
```

### Phase 2: 사람 mocap 수집 + DG-5F retargeter 학습

```
목표: 첫 DG-5F 체크포인트 생성

단계:
  1. Manus Glove 착용 후 5분 데이터 수집
  2. Manus → 캐노니컬 프레임 변환 어댑터 작성 (허들 5)
  3. python -m geort.trainer -hand dg5f_right -human_data <tag> -ckpt_tag dg5f_v1
  4. SAPIEN replay_evaluation으로 결과 시각화
  5. 손목 회전 invariance, pinch 보존, mode collapse 여부 확인

소요: 2∼3일
```

### Phase 3: Isaac Sim 통합 + 우리 sensing 파이프라인 연결

```
목표: src/retarget_dev/models/에 GeoRT 모델 추가

단계:
  1. src/retarget_dev/models/geort/ 신규 디렉토리
     - geort_model.py (RetargetingModel ABC 구현)
     - main.py (--sensing manus-ros2 등과 호환)
  2. GeoRT의 load_model을 wrapping하여 HandKeypoints → qpos 변환
  3. 좌표 규약 변환: 우리 HandKeypoints는 wrist-shifted MediaPipe 프레임
     → GeoRT 캐노니컬 프레임 (+Y thumb, +Z middle)으로 추가 회전
  4. JointState publisher로 Isaac Sim의 DG-5F에 명령

소요: 3∼5일
```

### Phase 4: 실제 DG-5F 연동 + 1 kHz 검증

```
목표: GeoRT의 1 kHz 추론을 실제 로봇에서 활용

단계:
  1. Phase 3 출력을 DG-5F ROS2 드라이버에 연결
  2. forward() 호출 빈도 측정 (mocap 속도가 병목인지 확인)
  3. 안전 제약 추가 (속도 limit, dead-man switch)
  4. dex-retargeting과 정성적 비교 (부드러움, pinch 정확도)

소요: 1주
```

---

## 10. 사용 시 주의점 (반드시 확인)

1. **좌표계 규약은 dex-retargeting과 다르다.**
   - GeoRT: `+Y=thumb, +Z=middle, +X=palm normal` (palm-centered)
   - dex-retargeting: `OPERATOR2MANO_RIGHT/LEFT` 행렬 사용
   - 두 모델을 동시에 운용하면 sensing 단에서 분기 필요. 우리 프로젝트의 [sensing/manus/manus_sensing.py](src/retarget_dev/sensing/manus/manus_sensing.py)에 두 가지 출력 모드를 구현하는 것이 깔끔.

2. **MediaPipe 입력은 GeoRT 자체 README가 비추천한다.**
   - 손목 회전 시 MediaPipe의 wrist tracking이 흔들려 학습이 망가짐
   - 본 프로젝트에서 phone → GeoRT 경로는 권장되지 않음
   - phone은 dex-retargeting, Manus는 GeoRT로 분리하는 것이 안전

3. **사용자 별 학습이 필수.** 다른 사람의 체크포인트를 그대로 쓰면 손 크기/관절 범위 차이로 출력이 어색해진다. 본 프로젝트의 `data/` 디렉토리에 user별 mocap을 누적해야 함.

4. **5-finger 핸드 지원이 검증되지 않았다.** Allegro만 기본 제공되며 모두 4-fingered. trainer 코드의 `n_fingers` 가정을 확인하지 않은 채 DG-5F (5 finger)를 학습하면 실패할 수 있다. **Phase 1 단계에서 가장 먼저 검증해야 할 항목**이다.

5. **Self-collision loss는 비활성.** 공개 코드의 trainer.py에서 충돌 손실이 placeholder 상태. DG-5F의 손가락 간 충돌이 우려되면 사용자가 직접 분류기를 만들어 활성화해야 한다.

6. **collision mesh segfault.** SAPIEN이 복잡한 collision mesh에서 자주 segfault를 낸다. URDF의 `<collision>` 블록을 단순화하거나 제거 권장.

7. **CC-BY-NC 라이선스는 상업화 금지.** 본 프로젝트가 학술/연구용이면 문제 없으나, 향후 product transition 시 dex-retargeting으로 교체해야 함을 인지.

8. **체크포인트 휴대성.** 학습된 체크포인트는 (사용자, 손, URDF, 좌표계 변환)에 종속. URDF나 변환을 바꾸면 재학습이 필요하다.

9. **공개 PyPI 패키지가 없다.** `pip install geort`는 동작하지 않음. 소스 클론이 유일한 설치 경로.

10. **활동성이 낮다.** 2025-03-04 이후 커밋 없음. 버그 수정이나 기능 추가를 기대하기 어려움. fork & maintain이 필요할 수 있다.

---

## 11. 핵심 요약

### GeoRT는 무엇인가

**"리타게팅을 학습 시점에 한 번에 풀어두자"**는 접근. 사람-로봇 손의 기하학적 대응을 5가지 미분 가능한 손실로 정의해 작은 MLP에 baking. 추론 시는 단순 forward pass → **1 kHz 처리 가능**.

### 수학적 핵심

**5가지 기하학적 손실** (Motion Preservation, C-space Coverage, Flatness, Pinch Correspondence, Self-Collision)을 unsupervised로 학습. dex-retargeting의 VectorOptimizer + DexPilotOptimizer 아이디어를 합친 것을 학습 목적함수로 baking + Chamfer distance로 작업공간 커버리지 보장.

### 우리 환경 적용의 핵심 허들

1. **DG-5F 5-finger 지원 검증** (Allegro 기준 4-finger 가정 여부)
2. **사용자 별 mocap 5분 + 학습 1∼5분** (dex-retargeting과 가장 큰 차이)
3. **캐노니컬 프레임 변환** (Manus → +Y thumb / +Z middle)
4. **CC-BY-NC 라이선스** (상업화 시 차단)
5. **Self-collision loss 비활성** (DG-5F는 손가락 간 충돌 가능성 높음 → 직접 활성화 필요)

### dex-retargeting과의 위치 관계

- 본 프로젝트는 **dex-retargeting을 메인으로 사용 중** ([models/dex_retarget/](src/retarget_dev/models/dex_retarget/))
- GeoRT는 **고대역(1 kHz) teleop 또는 부드러운 출력**이 필요할 때 사용할 **선택적 백엔드**로 추가하는 것이 자연스러움
- 두 모델을 같은 sensing 파이프라인 (`HandKeypoints`)에 plug-in 가능하도록 추상화
- **권장 진행 순서**: dex-retargeting을 안정화 → GeoRT를 phase별로 추가 → 둘 다 비교 후 용도별 분담

### 권장 진행 순서

```
Phase 0 (2-4시간):  GeoRT 공식 예제 (Allegro) 동작 확인
Phase 1 (1-2일):    DG-5F URDF + JSON 설정, 5-finger 가정 검증
Phase 2 (2-3일):    Manus mocap 5분 수집 + DG-5F retargeter 학습
Phase 3 (3-5일):    src/retarget_dev/models/geort/ 작성, sensing 파이프라인 연결
Phase 4 (1주):      실제 DG-5F 연동 + 1 kHz 검증 + dex-retargeting과 비교
```

가장 큰 위험은 **Phase 1의 5-finger 가정 검증**이다. 만약 trainer.py가 `n_fingers=4` hardcoded이면 코드 fork + 수정이 필수가 되어 후속 phase 일정이 크게 늘어날 수 있다. 이를 첫 1일 안에 확인하는 것이 가장 중요.
