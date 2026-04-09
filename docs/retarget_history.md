# 핸드 리타게팅 방법론의 발전 역사

**작성일:** 2026.03.27  
**목적:** Dexterous hand 원격조작을 위한 리타게팅 기법의 발전 흐름 분석

---

## 1. 리타게팅이란?

리타게팅(Retargeting)은 인간 손의 자세(포즈)를 로봇 핸드의 관절 명령으로 변환하는 과정이다. 인간 손과 로봇 핸드는 관절 수, 관절 축 방향, 손가락 길이, 접촉 면적 등이 다르기 때문에, 단순 복사가 아닌 변환 알고리즘이 필요하다.

```
인간 손 자세 (입력 장치: 장갑, 카메라, 엑소스켈레톤 등)
    ↓ [리타게팅 알고리즘]
로봇 핸드 관절 명령 (출력: 관절 각도 또는 관절 목표 위치)
```

리타게팅의 품질이 원격조작의 파지 안정성, 조작 정밀도, 작업자 직관성에 직접적으로 영향을 미치므로, dexterous manipulation 원격조작의 핵심 기술이다.

---

## 2. 발전 흐름 개관

리타게팅 방법론은 5세대로 분류할 수 있다. 각 세대는 순차적으로 대체되는 것이 아니라 **누적적으로 발전**하며, 후기 세대가 이전 세대의 기법을 기반으로 확장하는 형태이다.

```
1세대 (초기~2010s)    직접 관절 매핑
   ↓
2세대 (2015~2019)     손가락 끝 공간 + 역기구학(IK)
   ↓
3세대 (2020~현재 주류)  다중 제약 최적화 (DexPilot, AnyTeleop 등)
   ↓
4세대 (2019~현재)      신경망 학습 기반 (End-to-End)
   ↓
5세대 (2024~최신)      잔차 보정 + 물리 인식 (ResPilot, MoReL 등)
```

---

## 3. 세대별 상세 분석

### 3.1 1세대: 직접 관절 매핑 (Direct Joint Mapping)

**시기:** 초기 ~ 2010년대

**방법:**

인간 손가락의 관절 각도를 측정하여, 선형 스케일링/오프셋으로 로봇 핸드의 대응 관절에 직접 매핑한다.

```
θ_robot = α × θ_human + β

예: Manus Glove 엄지 MCP 굴곡 30° → DG-5F 엄지 MCP 굴곡 25°
    (α=0.83, β=0 의 단순 선형 변환)
```

**입력 장치:** CyberGlove, 벤딩 센서 장갑, IMU 장갑

**기본 가정:** 인간 손과 로봇 손의 관절 구조가 대략 유사하며, 관절 각도를 1:1로 대응시킬 수 있다.

**대표 연구:**
- Kumar & Todorov (2015): CyberGlove로 Shadow Hand를 제어하는 VR 기반 원격조작. 관절 각도를 직접 매핑.
- Rajeswaran et al. (2018): 원격조작 시연 데이터 수집을 위해 관절 직접 매핑 사용. 이후 RL의 초기 시연으로 활용.
- Su et al. (2021): 다중 Leap Motion + 칼만 필터 기반 적응 융합으로 벤딩 각도를 로봇 손에 매핑.

**장점:**
- 구현이 매우 간단하고 계산 비용이 거의 없음
- 실시간 지연 없음
- 직관적으로 이해하기 쉬움

**단점:**
- 인간 손과 로봇 손의 관절 축 방향, 손가락 길이, 관절 수가 다르면 매핑이 무너짐
- 인간 엄지의 대립(opposition) 운동 등 로봇에 없는 자유도 처리 불가
- 물리적으로 불가능한 자세(자기 충돌, 관절 범위 초과)가 생성될 수 있음
- 파지의 "기능적 의도"(예: 물체를 얼마나 세게 잡을지)가 전달되지 않음

---

### 3.2 2세대: 손가락 끝(Fingertip) 공간 + 역기구학 (Fingertip IK)

**시기:** 2015년 전후 ~ 2019년

**핵심 전환:** 관절 각도가 아니라 **손가락 끝(fingertip)의 3D 위치**를 매칭 목표로 설정하고, 역기구학(IK)으로 해당 위치에 도달하는 로봇 관절 각도를 계산한다.

```
1. 인간 손의 손가락 끝 5개의 3D 위치를 추정
2. 이 위치들을 로봇 핸드의 작업 공간에 맞게 스케일/정규화
3. 로봇 핸드의 역기구학(IK)으로 해당 위치에 도달하는 관절 각도를 계산

인간 fingertip 위치 → 스케일링 → 로봇 IK → 로봇 관절 각도
```

**왜 1세대보다 나은가:** 인간과 로봇의 관절 구조가 달라도, "손가락 끝이 같은 위치에 가면" 비슷한 파지가 된다. 관절 축이 다르더라도 IK 솔버가 적절한 관절 각도를 찾아준다.

**대표 연구:**
- Humberston & Pai (SCA 2015): 손가락 끝 키포인트 기반 IK로 로봇 핸드를 인터랙티브하게 제어. 순수 fingertip IK 기반 리타게팅의 초기 연구.
- Li et al. (ICRA 2019): BioIK 솔버를 사용하여 인간 depth 이미지에서 Shadow Hand 관절 각도를 계산. 40만 쌍의 데이터셋을 구축하여 후속 학습 기반 접근의 토대를 마련.
- Antotsiou et al. (ECCV Workshop 2018): 인간 포즈 추정기(HPE)로 21개 스켈레톤 키포인트를 추출하고 IK로 29-DOF 핸드 모델에 리타게팅. **IK 베이스라인의 한계를 확인하고 PSO 최적화를 추가**하여 3세대로의 전환점을 제시. IK만으로는 파지 성공률이 낮아, 과제 목표(물체 들기)를 비용 함수에 추가하면 크게 개선됨을 보임.

**장점:**
- 관절 구조가 달라도 동작 (1세대의 핵심 한계 극복)
- 파지의 "기능적 형태"가 더 잘 전달됨
- IK 솔버가 물리적 관절 범위를 자동으로 고려 가능

**단점:**
- IK 해의 다해성 (같은 끝점에 여러 관절 자세가 가능)
- 손가락 끝 위치만으로는 손가락의 굴곡 방식이 정해지지 않음 (같은 끝점에 펴진 채 또는 구부린 채 도달 가능)
- 손가락 간 관계(엄지-검지 간 거리 등)를 고려하지 않음
- 자기 충돌 회피가 자동으로 보장되지 않음

---

### 3.3 3세대: 다중 제약 최적화 기반 리타게팅 (Optimization-Based Retargeting)

**시기:** 2020년 ~ 현재까지 **주류(mainstream)**

**핵심 전환:** 단일 메트릭(관절 각도 또는 끝점 위치)이 아니라, **여러 제약 조건을 동시에 만족시키는 최적화 문제**로 리타게팅을 정식화한다.

```
최소화:
  w₁ × ||fingertip 위치 차이||²       (끝점 매칭)
  + w₂ × ||손가락 간 거리 비율 차이||²  (파지 형태 보존) ← 3세대의 핵심 추가
  + w₃ × ||관절 각도 차이||²           (자세 유사성)
  + w₄ × ||자기 충돌 페널티||           (물리적 실현 가능성)
  + w₅ × ||관절 한계 위반||             (관절 범위 내)

제약 조건:
  관절 각도 ∈ [θ_min, θ_max]
  손가락 간 최소 거리 > 0 (관통 방지)
```

**2세대와의 핵심 차이:** 2세대는 "각 손가락 끝이 목표 위치에 도달"만을 목적으로 하지만, 3세대는 "끝점 매칭 + 손가락 간 관계 + 자세 유사성 + 물리적 실현 가능성"을 **동시에** 최적화한다. 특히 **손가락 쌍 사이의 거리 비율 보존**이 3세대의 핵심 혁신으로, 파지 형태(예: "엄지와 검지가 가까이, 나머지는 멀리")가 리타게팅 후에도 유지된다.

**대표 연구:**

**DexPilot (Handa et al., ICRA 2020) — 이 세대의 출발점이자 가장 영향력 있는 연구:**
- Allegro Hand (4손가락, 16-DOF) 대상
- 컬러 장갑을 낀 인간 손을 RGB 카메라로 촬영 → PointNet++로 관절 추정
- Fingertip 위치 + **손가락 쌍 간 거리(pairwise finger distance)**를 비용 함수로 설정
- 최적화로 Allegro 관절 각도 계산
- 지갑에서 돈 빼기, 서랍 열기 같은 복잡한 과제를 시연
- **핵심 통찰:** fingertip 위치만 맞추면 안 되고, 손가락 쌍 사이의 거리 비율을 보존해야 파지 형태가 유지된다.

**AnyTeleop (Qin et al., RSS 2023) — DexPilot을 일반화한 프레임워크:**
- 인간 손의 MANO 모델에서 fingertip/joint 키포인트를 추출
- 다양한 로봇 핸드(Allegro, LEAP, xArm 등)에 적용 가능한 표준화된 키포인트 기반 비용 함수
- 관절 제한 제약을 명시적으로 포함
- 오픈소스로 공개되어 커뮤니티에서 널리 사용

**Retargeting Objective 분석 (Yu et al., 2024/2025) — 각 비용 항의 중요도를 체계적으로 분석:**
- 최적화 기반 리타게팅의 각 비용 항(fingertip 위치, 관절 각도, 자기충돌, 손가락 간 거리, 손가락 끝 방향 등)을 ablation
- 실제 조작 과제에서 어떤 항이 가장 중요한지를 실험적으로 밝힘
- LEAP Hand와 Shadow Hand에서 검증

**장점:**
- 여러 제약을 동시에 만족하여 물리적으로 실현 가능한 자세 보장
- 다양한 핸드에 일반화 가능 (비용 함수의 키포인트만 재정의)
- 자기 충돌 회피, 관절 범위 제한 등이 자동으로 반영
- 파지 형태(기능적 의도)가 잘 보존됨

**단점:**
- 최적화 풀이에 매 프레임마다 시간이 걸림 (실시간성 이슈, 특히 고DOF 핸드에서)
- 비용 함수의 가중치(w₁~w₅) 튜닝이 과제에 따라 필요
- 접촉력은 고려하지 않음 (순수 운동학적 접근)
- 최적화의 로컬 미니멈에 빠질 수 있음

---

### 3.4 4세대: 학습 기반 리타게팅 (Learning-Based Retargeting)

**시기:** 2019년 ~ 현재

**핵심 전환:** 매 프레임마다 최적화를 풀지 말고, **신경망이 인간 포즈에서 로봇 포즈를 직접 예측**하도록 학습한다.

**접근 A: 지도학습 (Paired Data)**
```
1. 최적화 기반 리타게팅(3세대)으로 (인간 포즈, 로봇 포즈) 쌍 데이터 대량 생성
2. 이 데이터셋으로 신경망 학습: f(인간 포즈) → 로봇 포즈
3. 추론 시 신경망 한 번 forward pass로 즉시 리타게팅
```

**접근 B: 비전 End-to-End**
```
인간 손 이미지(RGB 또는 depth) → 신경망 → 로봇 관절 각도 직접 출력
중간 단계(관절 추정, IK, 최적화 등) 없이 이미지에서 바로 로봇 명령으로
```

**대표 연구:**

**Li et al. (ICRA 2019) — 학습 기반 접근의 초기 연구:**
- BioIK 솔버로 40만 쌍의 (인간 depth 이미지, Shadow Hand 관절 각도) 데이터셋 구축
- Teacher-Student 네트워크로 학습
- 인간 손 depth 이미지만 입력하면 Shadow Hand 관절 각도를 직접 출력
- Teacher(특권 정보 사용) → Student(depth만 사용) 증류 구조

**Robotic Telekinesis (Sivakumar et al., RSS 2022) — 인터넷 비디오 활용:**
- YouTube 영상에서 인간 손 포즈를 추출하고, 리타게팅 네트워크를 학습
- 인터넷 비디오라는 대규모 데이터를 활용하여 다양한 파지 패턴 커버
- 특수 장갑 없이 단일 RGB 카메라로 원격조작 가능

**Shaw et al. (2024, Learning Dexterity from Human Hand Motion in Internet Videos) — Robotic Telekinesis 확장:**
- 단일 RGB 카메라로 인간 손 촬영 → 핸드 포즈 추정 → 학습된 리타게팅 네트워크 → LEAP Hand 제어
- EMA(지수 이동 평균) 필터로 포즈 추정 잡음을 매끄럽게 처리
- 1인칭/3인칭 비디오 모두에서 리타게팅 궤적 추출 가능

**Customized Robot Hand (Qin et al., 2022) — 리타게팅 자체를 우회하는 접근:**
- 인간 작업자의 손 형상을 스캔하여 그 사람 전용 로봇 핸드 모델을 시뮬레이션에서 생성
- 로봇 핸드의 기구학이 인간 손과 동일하므로 리타게팅이 불필요
- 시뮬레이션에서 수집한 궤적을 실제 로봇 핸드(Allegro 등)로 전이할 때만 리타게팅

**장점:**
- 추론 속도가 매우 빠름 (신경망 1회 forward pass, 실시간 보장)
- 최적화의 로컬 미니멈 문제 없음
- 대규모 데이터(인터넷 비디오 등)로 학습하면 다양한 파지 패턴에 일반화

**단점:**
- 학습 데이터 구축 비용 (3세대 최적화를 대량으로 돌려야 쌍 데이터 생성 가능)
- 새로운 로봇 핸드에 적용하려면 재학습 또는 파인튜닝 필요
- 학습 데이터에 없는 극단적 포즈에서 실패 가능
- 물리적 실현 가능성이 학습 데이터 품질에 의존

---

### 3.5 5세대: 잔차 보정 + 물리 인식 리타게팅 (Residual + Physics-Aware)

**시기:** 2024년 ~ 현재 (최신)

**핵심 전환:** 기존 리타게팅(3세대 또는 4세대)을 기본(base)으로 하되, 그 위에 **물리적 상호작용(접촉, 미끄러짐 등)과 개인별 차이를 고려한 잔차 보정**을 추가한다.

```
기본 리타게팅 (최적화 or 학습 기반)
  → 기본 로봇 포즈 (운동학적으로는 맞지만 접촉 역학은 미반영)
    ↓
잔차 보정 (GP, RL 등)
  → 접촉 안정성, 미끄러짐 방지, 작업자 개인 특성 반영
    ↓
최종 로봇 포즈
```

**왜 필요한가:** 3세대까지의 리타게팅은 순수 운동학(kinematics) 기반이다. "손가락 끝이 같은 위치에 가면 파지가 같다"고 가정하지만, 실제로는 로봇 손가락의 마찰 계수, 강성, 접촉 면적이 인간과 다르기 때문에 **같은 자세라도 접촉 역학이 다르다.** 또한 작업자마다 손 크기와 습관이 달라 동일한 리타게팅 알고리즘이 모든 사람에게 최적이 아니다.

**대표 연구:**

**ResPilot (2024) — GP(Gaussian Process) 잔차 학습:**
- Allegro Hand + Manus Quantum Metaglove 세팅
- 기존 최적화 기반 리타게팅에 GP residual을 추가
- **소수의 캘리브레이션 포즈**(작업자별 5-10개)에서 GP를 피팅
- 작업자의 고유한 손 크기/습관에 맞게 리타게팅을 미세 조정
- **Finger Constraint**: 발 페달로 특정 손가락의 움직임을 고정하여, 파지를 유지한 채 다른 손가락만 재배치하는 finger gaiting 가능
- 기존 리타게팅으로는 불가능했던 **6가지 고난이도 in-hand manipulation**을 시연
- 핵심 기여: 리타게팅 + 잔차 보정이 teleoperated dexterity의 새로운 수준을 열었음을 입증

**MoReL (Modular Residual RL for Retargeting, 최신) — RL 기반 잔차:**
- 리타게팅을 **손가락별 서브정책 + 잔차 협응 모듈**로 분해하여 RL로 학습
- 최적화 기반의 정확성과 학습 기반의 속도를 결합
- 물리적 상호작용(접촉, 미끄러짐)까지 고려하는 보정
- 하드웨어 적응성이 높음 (새 핸드에 서브정책만 추가/교체)

**TypeTele (2025) — 조작 타입 기반 리타게팅:**
- MLLM(멀티모달 대형 언어 모델)이 현재 과제에 맞는 조작 타입(precision pinch, power grasp 등)을 자동 선택
- 타입별로 사전 정의된 stretching/contracting 포즈를 리타게팅 타겟으로 설정
- 리타게팅 자체를 연속적 자세 추종이 아니라 **이산적 타입 전환**으로 단순화
- 기존 리타게팅 기반 시스템으로는 불가능한 과제(가위 사용, 무거운 주전자 등)를 성공

**장점:**
- 기존 리타게팅의 운동학적 정확성 + 물리적/개인적 보정
- 작업자별 빠른 캘리브레이션 가능 (ResPilot: 포즈 5-10개)
- 접촉 안정성이 구조적으로 고려되어 파지 실패율 감소
- Finger gaiting 등 고난이도 조작이 가능해짐

**단점:**
- 잔차 모델(GP 또는 RL) 훈련에 추가 비용
- 보정이 특정 핸드/환경에 특화될 수 있음
- 아직 연구 초기 단계로 표준화된 프레임워크 부재

---

## 4. 세대별 비교 요약

| 세대 | 핵심 기법 | 매칭 대상 | 대표 연구 | 장점 | 단점 |
|---|---|---|---|---|---|
| 1세대 | 직접 관절 매핑 | 관절 각도 | Kumar & Todorov 2015 | 간단, 빠름 | 구조 차이에 취약 |
| 2세대 | Fingertip IK | 끝점 3D 위치 | Humberston & Pai 2015, Li et al. 2019 | 구조 차이 극복 | 자세 모호성, 형태 미보존 |
| 3세대 | 다중 제약 최적화 | 끝점+거리+자세+충돌 | **DexPilot 2020, AnyTeleop 2023** | 물리적 실현 가능, 범용 | 계산 비용, 운동학만 |
| 4세대 | 신경망 학습 | End-to-End | Sivakumar 2022, Shaw 2024 | 빠른 추론, 대규모 데이터 | 재학습 필요, 데이터 의존 |
| 5세대 | 잔차 보정 | 기존 + 접촉/개인 보정 | **ResPilot 2024, MoReL 최신** | 물리 인식, 개인화 | 초기 단계, 추가 비용 |

---

## 5. 메인스트림 흐름과 현재 위치

```
[현재 메인스트림]

산업/연구 현장에서 가장 널리 쓰이는 것은 3세대(최적화 기반)이다.
AnyTeleop이 사실상 표준 프레임워크 역할을 하고 있으며,
대부분의 최신 원격조작 논문(DexCap, Open-TeleVision, Bunny-VisionPro 등)이
AnyTeleop의 최적화 기반 리타게팅을 사용하거나 참조한다.

[최신 연구 방향]

5세대(잔차 보정)가 급부상 중이다.
ResPilot(2024)이 GP 잔차로 teleoperated dexterity의 새 수준을 보였고,
MoReL이 RL 잔차로 하드웨어 적응성을 추가했으며,
TypeTele(2025)이 조작 타입 기반으로 리타게팅의 패러다임 자체를 전환했다.
```

---

## 6. 리타게팅 분류 체계와 입력 장치의 관계

| 입력 장치 | 제공하는 정보 | 적용 가능 세대 |
|---|---|---|
| 벤딩 센서 장갑 | 관절 굴곡 각도 | 1세대 |
| IMU 장갑 (Manus Glove 등) | 관절 각도 + 손가락 끝 위치 추정 | 1~3세대 |
| RGB 카메라 | 2D 키포인트 → 3D 포즈 추정 | 2~4세대 |
| RGB-D 카메라 | depth + 포즈 추정 | 2~4세대 |
| 엑소스켈레톤 장갑 | 정밀 관절 각도 + 힘 피드백 | 1~5세대 |

---

## 7. 프로젝트 적용 시사점

현재 세팅(Manus Metaglove → Tesollo DG-5F)에서의 리타게팅 고도화 방향:

```
현재 상태 확인 필요:
  manusGlove ROS2 패키지가 어떤 세대의 리타게팅을 쓰는가?
  → 단순 관절 매핑(1세대)이면 개선 여지가 매우 큼
  → 최적화 기반(3세대)이면 현재 주류 수준

개선 우선순위:
  1순위: 3세대 확인/적용 (AnyTeleop 스타일 최적화, 미적용 시)
  2순위: 5세대 잔차 보정 검토 (ResPilot 스타일, 작업자별 캘리브레이션)
  3순위: 이전 논의의 Shared Autonomy 보조 (Residual Copilot, DOF 분할 등)

핵심 인사이트:
  Shared Autonomy(이전 논의)에서 "핸드 파지 안정화를 위한 Residual 보정"이라고 했던 것과
  5세대 리타게팅의 "기존 리타게팅 위에 물리 인식 잔차 보정을 추가"하는 것은
  사실상 같은 문제를 다른 관점에서 접근한 것이다.
```

---

## 참고 문헌

| # | 논문 | 저자 | 게재 | 세대 |
|---|---|---|---|---|
| 1 | Teleoperation with CyberGlove + Shadow Hand | Kumar, Todorov | RSS 2015 Workshop | 1세대 |
| 2 | Learning from demonstrations for dexterous manipulation | Rajeswaran et al. | RSS 2018 | 1세대 |
| 3 | Interactive Animation of Precision Manipulation | Humberston, Pai | SCA 2015 | 2세대 |
| 4 | Vision-based teleoperation of Shadow Hand using E2E DNN | Li et al. | ICRA 2019 | 2/4세대 |
| 5 | Task-Oriented Hand Motion Retargeting | Antotsiou et al. | ECCV Workshop 2018 | 2→3 과도기 |
| 6 | **DexPilot: Vision-Based Teleoperation of Dexterous Hand-Arm** | **Handa et al.** | **ICRA 2020** | **3세대 (출발점)** |
| 7 | **AnyTeleop: A General Vision-Based Dexterous Teleoperation** | **Qin et al.** | **RSS 2023** | **3세대 (표준)** |
| 8 | Analyzing Key Objectives in Human-to-Robot Retargeting | Yu et al. | arXiv 2024/2025 | 3세대 (분석) |
| 9 | Robotic Telekinesis: Learning from YouTube | Sivakumar et al. | RSS 2022 | 4세대 |
| 10 | Learning Dexterity from Human Hand Motion in Internet Videos | Shaw et al. | IJRR 2024 | 4세대 |
| 11 | Customized Robot Hand (리타게팅 우회) | Qin et al. | ICLR 2022 | 4세대 (우회) |
| 12 | **ResPilot: Teleoperated Finger Gaiting via GP Residual** | **ResPilot team** | **arXiv 2024** | **5세대** |
| 13 | **MoReL: Modular Residual RL for Retargeting** | **MoReL team** | **최신** | **5세대** |
| 14 | **TypeTele: Releasing Dexterity by Manipulation Types** | **TypeTele team** | **arXiv 2025** | **5세대** |

---

*본 문서는 Frontiers in Neurorobotics (2022), PMC Survey (2024), ResPilot (2024), Yu et al. Retargeting Objective 분석 (2025) 등의 서베이 및 관련 논문을 기반으로 작성되었다.*

---
---

# Part II: DG5F 프로젝트 실전 개발 전략 (1~3세대)

> 작성일: 2026-04-08
> 기반 자료: [retargeting_research.md](retargeting_research.md)
> Target: Tesollo DG5F (20 DOF, 5 fingers x 4 joints)
> Input Sources: Manus Ergonomics (20 angles), Manus Skeleton (25 poses), MediaPipe Hands (21 landmarks)

---

## 8. 입력 소스 × 세대 조합 매트릭스

### 8.1 전체 조합 평가

```
             │  1세대 (Direct)   │  3세대 (Multi-Cost Opt)
─────────────┼───────────────────┼──────────────────────────
Manus        │ [1A] ★★★★★       │
Ergonomics   │ 각도→각도 직접     │
             │                   │
Manus        │ [1B] ★★★         │
Skeleton     │ quat→angle 분해   │
             │                   │
Manus        │                   │ [3A] ★★★★★
Ergo+Skeleton│      N/A          │ 모든 정보 통합 최적화
             │                   │
MediaPipe    │ [1C] ★★★★        │ [3B] ★★★★
Hands        │ position→angle    │ position 기반 최적화
```

> **2세대 (Fingertip IK)는 의도적으로 제외**: 별도 IK 솔버를 만드는 대신, 같은 fingertip 위치 정보를 3세대 multi-cost 최적화의 cost 항으로 직접 통합 (`models/dex_retarget/`). 단독 fingertip IK 구현(`fingertip_ik` 모듈)은 2026-04-09에 deprecate.

### 8.2 합리적 조합 5개 선정

| ID | 입력 소스 | 세대 | 전략 | 권장도 | 개발순서 |
|----|----------|------|------|--------|---------|
| **1A** | Manus Ergonomics | 1세대 | Ergo Direct Mapping | ★★★★★ | **1번째** |
| **1B** | Manus Skeleton | 1세대 | Skeleton Quat Decomposition | ★★★☆☆ | 선택적 |
| **1C** | MediaPipe | 1세대 | Vector Angle Extraction | ★★★★☆ | **2번째** |
| **3A** | Manus Ergo+Skeleton | 3세대 | Manus Multi-Cost Optimization | ★★★★★ | **3번째** |
| **3B** | MediaPipe | 3세대 | MediaPipe Multi-Cost Optimization | ★★★★☆ | **4번째** |

### 8.3 제외된 조합과 그 이유

| 조합 | 제외 이유 |
|------|----------|
| Manus Ergonomics 단독 + 3세대 | 3세대의 핵심인 fingertip position cost를 계산하려면 Skeleton position 필요 |
| Manus Skeleton 단독 + 3세대 | 가능하나, Ergonomics angle prior(C3)가 없으면 자세 자연스러움 저하. 3A에 Ergo 포함이 더 우월 |
| 2세대 (Fingertip IK) 단독 구현 | 같은 fingertip 정보가 3세대 cost 항으로 통합 가능. 별도 IK 솔버를 유지할 이유 없음 |

---

## 9. 1세대: 직접 관절 매핑 (Direct Joint Mapping) — 실전 설계

### 핵심 원리
```
θ_robot = clamp(scale × θ_human + offset, lower, upper)
```
IK도 최적화도 없이, 입력의 관절 각도(또는 파생 각도)를 DG5F 관절 각도로 **직접 선형 변환**.

---

### [1A] Manus Ergonomics → Direct Mapping → DG5F

#### 파이프라인
```
Manus Glove
  → manus_ros2 node (120Hz)
    → ManusGlove.ergonomics[] (20 float, degrees)
      → deg2rad
        → per-joint: q_i = scale_i × θ_i + offset_i
          → clamp(q_i, lower_i, upper_i)
            → DG5F JointState (20 radians)
```

#### 매핑 테이블

| # | Manus Ergonomics | → | DG5F Joint | 축 | 변환 | 비고 |
|---|-----------------|---|-----------|---|------|------|
| 1 | ThumbMCPSpread | → | rj_dg_1_1 | X | linear | 부호 확인 필요 |
| 2 | ThumbMCPStretch | → | rj_dg_1_2 | Z | **비선형** | Opposition, 가장 까다로운 매핑 |
| 3 | ThumbPIPStretch | → | rj_dg_1_3 | X | linear | |
| 4 | ThumbDIPStretch | → | rj_dg_1_4 | X | linear | |
| 5 | IndexMCPSpread | → | rj_dg_2_1 | X | linear | |
| 6 | IndexMCPStretch | → | rj_dg_2_2 | Y | linear | |
| 7 | IndexPIPStretch | → | rj_dg_2_3 | Y | linear | |
| 8 | IndexDIPStretch | → | rj_dg_2_4 | Y | linear | |
| 9 | MiddleMCPSpread | → | rj_dg_3_1 | X | linear | |
| 10 | MiddleMCPStretch | → | rj_dg_3_2 | Y | linear | |
| 11 | MiddlePIPStretch | → | rj_dg_3_3 | Y | linear | |
| 12 | MiddleDIPStretch | → | rj_dg_3_4 | Y | linear | |
| 13 | RingMCPSpread | → | rj_dg_4_1 | X | linear | |
| 14 | RingMCPStretch | → | rj_dg_4_2 | Y | linear | |
| 15 | RingPIPStretch | → | rj_dg_4_3 | Y | linear | |
| 16 | RingDIPStretch | → | rj_dg_4_4 | Y | linear | |
| 17 | PinkyMCPSpread | → | rj_dg_5_1 | Z | linear | DG5F에서 Z축 (다른 손가락은 X) |
| 18 | PinkyMCPStretch | → | rj_dg_5_2 | X | linear | DG5F에서 X축 (다른 손가락은 Y) |
| 19 | PinkyPIPStretch | → | rj_dg_5_3 | Y | linear | |
| 20 | PinkyDIPStretch | → | rj_dg_5_4 | Y | linear | |

#### 캘리브레이션 방법
```python
# Step 1: 사용자가 손을 완전히 편 상태 → Manus Ergo 값 기록
rest_values = record_manus_ergonomics()  # 20 floats

# Step 2: 사용자가 주먹을 완전히 쥔 상태 → Manus Ergo 값 기록
fist_values = record_manus_ergonomics()  # 20 floats

# Step 3: min-max normalization → DG5F range 매핑
for i in range(20):
    human_range = fist_values[i] - rest_values[i]
    if abs(human_range) < 1e-6:
        continue  # division by zero 방지
    norm = (ergo[i] - rest_values[i]) / human_range
    q[i] = dg5f_lower[i] + norm * (dg5f_upper[i] - dg5f_lower[i])
    q[i] = clamp(q[i], dg5f_lower[i], dg5f_upper[i])
```

#### 장점
| 항목 | 설명 |
|------|------|
| **구현 최소** | ~100줄, 외부 의존성 없음 (numpy만) |
| **실시간 완벽** | 120Hz 입력 그대로 출력, 계산 ~0.01ms |
| **디버깅 용이** | 각 관절이 독립 → 문제 격리 쉬움 |
| **1:1 DOF 매칭** | 20 Ergo ↔ 20 DG5F, 정보 손실 zero |
| **기반 검증** | 부호 방향, 범위, 축 문제를 1A에서 먼저 발견 → 이후 세대에서 재사용 |

#### 한계점
| 항목 | 설명 |
|------|------|
| **Fingertip 위치 부정확** | 같은 관절 각도라도 link 길이가 다르면 fingertip 위치 불일치 → 파지 실패 가능 |
| **Thumb opposition 불완전** | rj_dg_1_2는 Z축 revolute (-180°~0°). Ergo ThumbMCPStretch는 단순 flexion → 직접 매핑으로 opposition 부정확 |
| **비선형 특성 무시** | 인간 관절의 비선형 커플링(예: MCP flex 증가 시 abd 가동범위 감소)을 반영 불가 |
| **개인차 보정 한계** | 캘리브레이션으로 범위만 보정. 관절 간 커플링이나 비대칭성은 보정 불가 |

#### 개발 시 주의점

**1. 부호 방향 (가장 중요한 첫 번째 검증)**
```
Manus: Stretch 양수 = 굴곡 (손가락 접기)
DG5F:  양수 방향은 축/손가락마다 다름

반드시 실기 테스트:
  Manus에서 Index MCP +30° 입력 → DG5F rj_dg_2_2가 실제로 굴곡되는지 확인
  부호 반전 필요 시: scale = -1.0
```

**2. Thumb opposition (rj_dg_1_2) 특수 처리**
```
문제: DG5F rj_dg_1_2 범위 = -180°~0° (Z축 회전, 매우 넓음)
      Manus ThumbMCPStretch 범위 = ~-30°~+108° (단순 flexion)

선택지:
  (a) Linear: q = scale * ergo + offset  (간단하지만 부정확)
  (b) Piecewise linear: ergo 0°~40° → q -90°~0°, ergo 40°~108° → q -180°~-90°
  (c) Lookup table: 5-10개 캘리브레이션 포즈에서 측정한 매핑 테이블
  권장: (a)로 시작, 문제 시 (b)로 전환
```

**3. Pinky 축 배치 주의**
```
Index/Middle/Ring: J1=X축(abd), J2=Y축(MCP flex)
Pinky:             J1=Z축(abd), J2=X축(spread)
→ 같은 Ergo Spread/Stretch인데 DG5F 축이 다름
→ 코드에서 Pinky만 별도 분기 필요
```

**4. EMA filter 추가 권장**
```python
# Manus 센서 노이즈 완화
alpha = 0.4  # 0.3(부드러움)~0.5(반응)
q_filtered[i] = alpha * q_new[i] + (1 - alpha) * q_filtered_prev[i]
```

---

### [1B] Manus Skeleton → Quaternion Decomposition → DG5F

#### 파이프라인
```
Manus Glove
  → manus_ros2 node (120Hz)
    → ManusGlove.raw_nodes[] (25 nodes, local quaternions)
      → 인접 노드 쌍의 relative quaternion 계산
        → swing-twist decomposition으로 flex/abd 추출
          → clamp → DG5F JointState
```

#### 핵심 알고리즘: Swing-Twist Decomposition
```python
def extract_joint_angle(q_parent, q_child, twist_axis):
    """부모-자식 quaternion에서 twist_axis 기준 회전 각도 추출"""
    q_rel = q_parent.inverse() * q_child

    # Twist component (twist_axis에 투영)
    proj = np.dot(q_rel.vec, twist_axis)
    twist = Quaternion(w=q_rel.w, xyz=proj * twist_axis).normalized()

    # Twist → angle
    angle = 2 * np.arctan2(np.linalg.norm(twist.vec), twist.w)
    if np.dot(twist.vec, twist_axis) < 0:
        angle = -angle
    return angle

# 사용 예: Index MCP flexion
flex = extract_joint_angle(
    nodes[5].quat,   # Index Metacarpal
    nodes[6].quat,   # Index Proximal
    twist_axis=[0, 1, 0]  # Y축 = flexion
)
```

#### 장점
- **풀 6DOF 정보**: Quaternion에서 flex/abd를 독립 추출 가능
- **SDK 보간 전 원시 데이터**: Ergonomics보다 원본에 가까움

#### 한계점
- **구현 복잡도 높음**: Quaternion math + 좌표계 변환 필수
- **Gimbal lock 위험**: 특정 자세에서 Euler 변환 불안정
- **1A 대비 이점 미미**: Ergonomics가 이미 동일 센서에서 파생 → 정확도 차이 작음

#### 개발 시 주의점
1. **Local 모드 필수**: World 좌표는 wrist 기준 절대값 → 부모-자식 상대 회전 재계산 필요
2. **Quaternion convention**: Manus SDK=WXYZ, ROS2 geometry_msgs=XYZW → 변환 누락 시 완전히 다른 결과
3. **Metacarpal/Tip 노드 제외**: 실제 관절 회전을 나타내지 않음
4. **Thumb 4노드 vs 다른 손가락 5노드**: 파싱 로직 분기 필요

#### 1A vs 1B 판단 기준

| | 1A (Ergonomics) | 1B (Skeleton Quat) |
|---|---|---|
| 구현 시간 | 2시간 | 1~2일 |
| 정확도 | ★★★★ | ★★★★☆ |
| 디버깅 | 매우 쉬움 | 어려움 |
| **결론** | **프로토타입에 선택** | 1A에 문제 발견 시 대안 |

> **권장**: 1B는 1A가 이미 충분한 경우 건너뛰고 3세대로 진행. 1A의 Thumb opposition 등에서 품질 문제가 발생할 때만 1B 검토.

---

### [1C] MediaPipe Hands → Vector Angle Extraction → DG5F

#### 파이프라인
```
RGB Camera (30-60fps)
  → MediaPipe Hands
    → world_landmarks[] (21 positions, meters)
      → bone vector 계산 (인접 landmark 차분)
        → palm frame 정의 (WRIST, INDEX_MCP, PINKY_MCP)
          → flexion/abduction 각도 계산 (벡터 내적/외적)
            → EMA filter → clamp → DG5F JointState
```

#### 핵심 알고리즘: Palm Frame + Angle Extraction
```python
def compute_palm_frame(landmarks):
    """WRIST(0), INDEX_MCP(5), PINKY_MCP(17)로 palm 좌표계 정의"""
    wrist = landmarks[0]
    idx_mcp = landmarks[5]
    pky_mcp = landmarks[17]

    palm_x = normalize(idx_mcp - pky_mcp)          # 좌→우
    palm_temp = normalize(idx_mcp - wrist)          # wrist→손가락
    palm_z = normalize(np.cross(palm_x, palm_temp)) # palm normal (손등 방향)
    palm_y = np.cross(palm_z, palm_x)               # 손가락 전방

    return palm_x, palm_y, palm_z

def compute_finger_angles(landmarks, finger_ids, palm_frame):
    """한 손가락의 4 DOF 각도 계산"""
    palm_x, palm_y, palm_z = palm_frame

    # Bone vectors
    bones = [normalize(landmarks[finger_ids[i+1]] - landmarks[finger_ids[i]])
             for i in range(len(finger_ids) - 1)]

    # Abduction: bone[0]을 palm plane에 투영 → palm_y(전방)과의 각도
    proj = bones[0] - np.dot(bones[0], palm_z) * palm_z
    abd = np.arctan2(np.dot(proj, palm_x), np.dot(proj, palm_y))

    # Flexion: 인접 bone 간 각도
    mcp_flex = np.arccos(np.clip(np.dot(bones[0], bones[1]), -1, 1))
    pip_flex = np.arccos(np.clip(np.dot(bones[1], bones[2]), -1, 1))
    dip_flex = np.arccos(np.clip(np.dot(bones[2], bones[3]), -1, 1))

    return abd, mcp_flex, pip_flex, dip_flex

# Index finger 예시
INDEX_IDS = [0, 5, 6, 7, 8]  # Wrist → MCP → PIP → DIP → TIP
abd, mcp, pip, dip = compute_finger_angles(landmarks, INDEX_IDS, palm_frame)
```

#### 장점
| 항목 | 설명 |
|------|------|
| **글러브 불필요** | RGB 카메라만으로 동작 → 접근성 최고 |
| **설치 간편** | `pip install mediapipe` |
| **양손 동시 추적** | `max_num_hands=2` |
| **비접촉** | 착용감 문제 없음, 원격 데모에 유리 |

#### 한계점
| 항목 | 설명 |
|------|------|
| **낮은 주파수** | 30-60fps (Manus 120Hz의 1/4) |
| **Depth 노이즈** | Z축 부정확 → abduction 추정 오차 큼 |
| **Occlusion** | 주먹 등 손가락 겹침 시 landmark 부정확 |
| **Thumb 어려움** | CMC 2DOF를 4 landmark로 분리 추출하기 어려움 |
| **조명 민감** | 어두운 환경, 복잡한 배경에서 성능 저하 |

#### 개발 시 주의점

**1. world_landmarks 사용 필수**
```python
# normalized landmarks는 카메라 왜곡에 의존 → 각도 계산 불가
# 반드시 world_landmarks (미터 단위) 사용
landmarks = result.multi_hand_world_landmarks[0]
```

**2. Temporal filter 필수**
```python
# One Euro Filter 또는 EMA (α=0.3~0.4)
# 적용하지 않으면 DG5F가 심하게 떨림
filtered_q = alpha * new_q + (1 - alpha) * prev_q
```

**3. Abduction deadzone**
```python
# Depth 노이즈로 인한 불필요한 벌림 방지
if abs(abd) < deg2rad(5.0):
    abd = 0.0
```

**4. PIP-DIP coupling 권장**
```python
# MediaPipe DIP 정확도가 낮으므로 해부학적 커플링 적용
dip = 0.67 * pip  # DIP ≈ 2/3 × PIP (인간 건 구조 모사)
```

**5. Landmark confidence 체크**
```python
# visibility/presence 낮으면 이전 프레임 값 유지
if landmark.visibility < 0.5 or landmark.presence < 0.5:
    use_previous_frame()
```

---

### 1세대 조합 비교 요약

| | 1A (Manus Ergo) | 1B (Manus Quat) | 1C (MediaPipe) |
|---|---|---|---|
| 구현 시간 | **2시간** | 1~2일 | **반나절~1일** |
| 실시간성 | **120Hz** | **120Hz** | 30-60Hz |
| 정확도 | ★★★★ | ★★★★☆ | ★★★ |
| 하드웨어 비용 | Manus 글러브 | Manus 글러브 | **카메라만** |
| 파지 정확도 | ★★ | ★★ | ★★ |
| **권장 시나리오** | **Manus 보유 시 최우선** | 1A 문제 시 대안 | **글러브 없을 때** |

> **공통 한계**: 1세대 전체가 link 길이 차이를 무시 → fingertip 위치 불일치 → 파지 부정확. 이것이 fingertip 위치 정보를 cost 항으로 통합하는 3세대로 넘어가는 동기.

---

## 10. 3세대: 다중 제약 최적화 (Multi-Cost Optimization) — 실전 설계

### 핵심 원리
```
θ* = argmin  Σ w_i × C_i(θ, human_data)
      θ
     s.t.    θ_lower ≤ θ ≤ θ_upper
```
**다중 cost function의 가중합을 최소화**하여 관절 각도를 구한다.
1세대(각도 매핑)와 2세대(위치 매핑)의 장점을 **동시에 활용**하며,
smoothness, collision, contact 등의 제약을 자연스럽게 추가 가능.

---

### [3A] Manus Ergo + Skeleton → Multi-Cost Optimization → DG5F

#### 파이프라인
```
Manus Glove → manus_ros2 (120Hz)
  ┌── ManusGlove.ergonomics[] (20 angles, deg) ──── C3 angle prior
  └── ManusGlove.raw_nodes[] (25 poses) ─────────── C1 fingertip pos, C2 bone dir
        │
        ▼
  scipy.optimize.minimize (method='SLSQP')
    minimize:
      w1 × C1_fingertip_position    ← Skeleton tip nodes
    + w2 × C2_bone_direction        ← Skeleton bone vectors
    + w3 × C3_angle_prior           ← Ergonomics angles (1세대 결과)
    + w4 × C4_smoothness            ← θ_prev
    + w5 × C5_self_collision        ← DG5F link 간 최소 거리
    + w6 × C6_joint_limit_penalty   ← URDF limits 근처 soft penalty
    bounds: [(lower_i, upper_i) for i in range(20)]
        │
        ▼
  DG5F JointState (20 radians)
```

#### 이것이 가장 강력한 조합인 이유

```
3A가 사용하는 정보:
  ┌─ Ergonomics (20 angles) ─→ "인간이 이 각도로 움직임" (C3)
  ├─ Skeleton positions ─────→ "fingertip이 여기 있음" (C1)
  ├─ Skeleton quaternions ───→ "bone이 이 방향으로 향함" (C2)
  ├─ θ_prev ─────────────────→ "이전 프레임과 부드럽게" (C4)
  ├─ DG5F FK ────────────────→ "로봇 손가락 간 충돌 방지" (C5)
  └─ URDF limits ────────────→ "관절 한계 부드럽게 준수" (C6)

1세대 = C3만 사용
2세대 = C1만 사용
3세대 = C1 + C2 + C3 + C4 + C5 + C6 모두 통합
```

#### Cost Functions 상세 설계

```python
def total_cost(theta, human, theta_prev, weights, dg5f_fk):
    """
    theta:      (20,) DG5F 관절 각도 후보
    human:      ManusGlove 메시지 (ergonomics + raw_nodes)
    theta_prev: (20,) 이전 프레임 관절 각도
    weights:    (w1, w2, w3, w4, w5, w6)
    dg5f_fk:    DG5F FK solver (Pinocchio)
    """
    w1, w2, w3, w4, w5, w6 = weights
    cost = 0.0

    # ──────────────────────────────────────────────
    # C1: Fingertip Position Cost (2세대의 핵심)
    # "DG5F fingertip이 인간 fingertip과 같은 위치에"
    # ──────────────────────────────────────────────
    for f in range(5):
        p_human = scale_to_robot(human.skeleton_tip[f])
        p_robot = dg5f_fk.fingertip_pos(theta, finger=f)
        cost += w1 * np.sum((p_human - p_robot) ** 2)

    # ──────────────────────────────────────────────
    # C2: Bone Direction Cost (자세 모호성 해결)
    # "DG5F bone이 인간 bone과 같은 방향을 향함"
    # → 같은 tip 위치라도 자세가 인간과 유사하게 됨
    # ──────────────────────────────────────────────
    for f in range(5):
        for seg in ['proximal', 'middle', 'distal']:
            d_human = normalize(human.skeleton_bone_dir[f][seg])
            d_robot = normalize(dg5f_fk.bone_dir(theta, finger=f, segment=seg))
            cost += w2 * (1.0 - np.dot(d_human, d_robot))  # 0(일치)~2(반대)

    # ──────────────────────────────────────────────
    # C3: Joint Angle Prior Cost (1세대의 장점 활용)
    # "Ergonomics 각도와 크게 벗어나지 않게"
    # → C1+C2만으로 해가 발산할 때 안전망 역할
    # ──────────────────────────────────────────────
    theta_ergo = ergo_direct_mapping(human.ergonomics)  # 1A 결과
    cost += w3 * np.sum((theta - theta_ergo) ** 2)

    # ──────────────────────────────────────────────
    # C4: Smoothness Cost
    # "프레임 간 급격한 변화 방지"
    # ──────────────────────────────────────────────
    cost += w4 * np.sum((theta - theta_prev) ** 2)

    # ──────────────────────────────────────────────
    # C5: Self-Collision Cost
    # "DG5F 손가락 간 관통 방지"
    # ──────────────────────────────────────────────
    D_SAFE = 0.005  # 5mm 최소 간격
    for (fi, fj) in [(1,2), (2,3), (3,4), (4,5), (1,5)]:  # 인접+대각 쌍
        dist = dg5f_fk.min_link_distance(theta, fi, fj)
        if dist < D_SAFE:
            cost += w5 * (D_SAFE - dist) ** 2

    # ──────────────────────────────────────────────
    # C6: Joint Limit Soft Penalty
    # "hard clamp 대신 soft penalty → 최적화가 부드럽게 동작"
    # ──────────────────────────────────────────────
    MARGIN = 0.05  # ~3° margin
    for i in range(20):
        over = max(0, theta[i] - (dg5f_upper[i] - MARGIN))
        under = max(0, (dg5f_lower[i] + MARGIN) - theta[i])
        cost += w6 * (over ** 2 + under ** 2)

    return cost
```

#### 가중치 프리셋

```yaml
# weight_config.yaml
presets:
  free_motion:  # 자유로운 손 동작 (파지 아닐 때)
    w1_fingertip_pos: 5.0
    w2_bone_direction: 8.0   # 자세 유사성 강조
    w3_angle_prior: 3.0
    w4_smoothness: 2.0
    w5_self_collision: 20.0
    w6_joint_limit: 15.0

  grasping:  # 물체 잡기
    w1_fingertip_pos: 15.0   # fingertip 위치 최우선
    w2_bone_direction: 3.0
    w3_angle_prior: 1.0
    w4_smoothness: 1.0
    w5_self_collision: 20.0
    w6_joint_limit: 15.0

  stable_hold:  # 물체 잡고 유지
    w1_fingertip_pos: 10.0
    w2_bone_direction: 2.0
    w3_angle_prior: 1.0
    w4_smoothness: 8.0       # 안정성 강조
    w5_self_collision: 20.0
    w6_joint_limit: 15.0
```

#### Optimizer 설정

```python
from scipy.optimize import minimize

result = minimize(
    fun=total_cost,
    x0=theta_prev,            # warm start (핵심!)
    args=(human, theta_prev, weights, dg5f_fk),
    method='SLSQP',
    bounds=[(dg5f_lower[i], dg5f_upper[i]) for i in range(20)],
    options={
        'maxiter': 30,         # 120Hz에서 시간 제약
        'ftol': 1e-6,
        'disp': False,
    },
    jac=analytical_jacobian,   # 제공 시 2~3배 빠름
)
theta_optimal = result.x
```

#### 장점
| 항목 | 설명 |
|------|------|
| **1+2세대 통합** | 각도(정확성) + fingertip(파지) 동시 활용 |
| **자세 모호성 해결** | C2(bone dir) + C3(angle prior)가 IK redundancy 문제 제거 |
| **Self-collision 방지** | 1,2세대에서 불가능했던 손가락 간 관통 방지 |
| **확장 가능** | 새 제약을 cost 항으로 추가 가능 (접촉력, 물체 형상 등) |
| **가중치로 행동 모드 전환** | grasping/free_motion/stable_hold 등 상황별 프리셋 |

#### 한계점
| 항목 | 설명 |
|------|------|
| **연산 비용** | 20-DOF SLSQP → 1~5ms/frame (120Hz에서는 가능하지만 marginal) |
| **Local minima** | 비볼록 → 초기값 의존 → warm start 필수 |
| **FK 필요** | Pinocchio FK를 매 iteration마다 호출 → 속도 병목 |
| **가중치 튜닝** | 6개 가중치 × 여러 프리셋 → 경험적 조정 필요 |
| **Thumb 여전히 어려움** | Opposition 축의 비선형성이 optimizer를 혼란시킬 수 있음 |

#### 개발 시 주의점

**1. Warm start 필수 (가장 중요)**
```python
# 매 프레임 초기값 = 이전 프레임 해
x0 = theta_prev  # ← 이것 하나로 수렴 속도 3~5배 향상
# 절대 x0 = np.zeros(20)으로 시작하지 말 것
```

**2. Analytical Jacobian 제공**
```python
# scipy finite difference 대비 2~3배 빠름
# Pinocchio computeJointJacobians() → dCost/dTheta 체인룰로 계산
# 또는 CasADi 자동 미분 활용
```

**3. C5 self-collision 간소화**
```
모든 link pair = C(20,2) = 190개 → 너무 많음
실제 충돌 가능 쌍만 체크:
  (Thumb, Index), (Index, Middle), (Middle, Ring), (Ring, Pinky), (Thumb, Pinky)
→ 5쌍만 체크해도 충분
추가 최적화: 매 N프레임(N=5)마다만 C5 계산
```

**4. C1 위치 단위 스케일링**
```
C1: 위치 오차 (단위: meters, 값: ~0.01)  → 제곱하면 ~1e-4
C3: 각도 오차 (단위: radians, 값: ~0.5)  → 제곱하면 ~0.25
→ 단위 차이로 가중치 의미가 달라짐
→ C1에 scale factor 적용: cost += w1 * 1000 * ||p_err||²  (mm 단위로 변환)
```

**5. Profiling 필수**
```python
import time
t0 = time.perf_counter()
result = minimize(...)
dt = time.perf_counter() - t0
if dt > 0.007:  # 7ms = 120Hz에서 84% 시간 budget
    logger.warning(f"Optimization too slow: {dt*1000:.1f}ms")
    # 대응: maxiter 줄이기 (30→15) 또는 C5 건너뛰기
```

**6. 120Hz 실시간 확보 실패 시 대안**
```
(a) maxiter 줄이기: 30 → 15 → 10 (해 품질 ↓, 속도 ↑)
(b) C5 매 5프레임만: collision check 빈도 ↓
(c) C++ 전환: Pinocchio C++ + Eigen → 5~10배 빠름
(d) CasADi: 자동 미분 + JIT 컴파일 → Python에서도 빠른 최적화
(e) 2-thread: optimization을 별도 thread에서 비동기 실행, main thread는 보간
```

---

### [3B] MediaPipe → Multi-Cost Optimization → DG5F

#### 파이프라인
```
RGB Camera (30-60fps)
  → MediaPipe Hands
    → world_landmarks[] (21 positions, meters)
      │
      ▼
  scipy.optimize.minimize (method='SLSQP')
    minimize:
      w1 × C1_fingertip_position      ← landmarks[4,8,12,16,20]
    + w2 × C2_bone_direction          ← landmark 간 벡터 방향
    + w3 × C3_natural_pose_prior      ← 해부학적 제약 (PIP-DIP coupling 등)
    + w4 × C4_smoothness              ← θ_prev (가중치 높게!)
    + w5 × C5_self_collision
    + w6 × C6_joint_limit_penalty
        │
        ▼
  DG5F JointState (20 radians)
```

#### 3A와의 핵심 차이: C3 대체

3A는 Ergonomics 각도를 C3 angle prior로 직접 사용.
3B는 Ergonomics가 없으므로 **해부학적 prior**로 대체:

```python
def C3_natural_pose_prior(theta):
    """Ergonomics 없이 해부학적 제약으로 자세 자연스러움 유지"""
    cost = 0.0

    for finger in [2, 3, 4, 5]:  # Index ~ Pinky
        abd, mcp, pip, dip = split_finger_joints(theta, finger)

        # 1) PIP-DIP coupling: DIP ≈ 0.67 × PIP
        cost += (dip - 0.67 * pip) ** 2

        # 2) MCP > PIP > DIP 경향 (자연스러운 굽힘 순서)
        cost += max(0, pip - mcp) ** 2 * 0.5

        # 3) Abduction은 작아야 자연스러움
        cost += 0.3 * abd ** 2

    # Thumb: opposition과 flexion 관계
    t_abd, t_opp, t_pip, t_dip = split_finger_joints(theta, 1)
    cost += (t_dip - 0.5 * t_pip) ** 2  # Thumb DIP coupling

    return cost
```

#### 3A vs 3B 비교

| | 3A (Manus) | 3B (MediaPipe) |
|---|---|---|
| C1 fingertip 정확도 | ★★★★★ | ★★★ (depth 노이즈) |
| C2 bone direction 정확도 | ★★★★★ | ★★★ (position 차분) |
| C3 angle prior | **Ergonomics 직접** (정확) | 해부학적 heuristic (근사) |
| 입력 주파수 | 120Hz | 30-60Hz |
| Occlusion 내성 | ★★★★★ | ★★ |
| 접근성 | Manus 글러브 필요 | **카메라만** |

#### 장점
- **글러브 없는 최고 품질**: 카메라만으로 3세대 수준 가능
- **3A 코드 90% 재사용**: C3만 교체, 나머지 동일

#### 한계점
- **C3가 약함**: Heuristic prior는 Ergonomics 직접 참조보다 부정확
- **Depth 노이즈 → C1/C2 품질 저하**: 근본적으로 카메라 한계
- **Occlusion 대응 필요**: 가려진 landmark 감지 + fallback 로직

#### 개발 시 주의점

**1. Landmark confidence-weighted cost**
```python
# visibility 낮은 landmark는 cost 가중치 감소
for f in range(5):
    tip_idx = [4, 8, 12, 16, 20][f]
    conf = landmarks[tip_idx].visibility * landmarks[tip_idx].presence
    cost += w1 * conf * np.sum((p_target - p_robot) ** 2)
    # conf=0이면 해당 finger의 C1 무시 → 다른 cost로만 해 결정
```

**2. C4 smoothness 가중치 높게**
```yaml
# 3B에서는 3A 대비 w4를 2~3배 증가
# MediaPipe 노이즈로 인한 프레임 간 떨림 억제
w4_smoothness: 5.0  # 3A에서는 2.0
```

**3. Occlusion fallback**
```python
# 연속 N프레임 이상 confidence 낮으면 해당 finger 동결
if consecutive_low_conf[f] > 5:
    # 해당 finger의 C1, C2 가중치 = 0
    # C4(smoothness)만으로 이전 자세 유지
    finger_weights[f] = 0.0
```

---

## 11. 세대별 진화 다이어그램

```
1세대                       2세대                       3세대
───────────────────────    ───────────────────────    ────────────────────────
"같은 각도 → 같은 자세"     "같은 tip → 같은 접촉"      "전부 통합"

입력: θ_human              입력: p_fingertip            입력: θ + p + direction
  ↓                          ↓                           ↓
scale + offset              IK(p_target)                minimize Σ w_i C_i
  ↓                          ↓                           ↓
θ_robot                    θ_robot                     θ_robot

✓ 빠름                     ✓ 파지 정확                  ✓ 파지 + 자세 + 안전
✓ 간단                     ✗ 자세 모호                  ✓ 확장 가능
✗ tip 위치 불일치          ✗ 중간 관절 부자연            ✗ 연산 비용
✗ 파지 실패 가능           ✗ collision 미대응            ✗ 튜닝 복잡

        ┌──────────────────┐
        │ 1세대의 각도 정보  │───→ 3세대 C3 (angle prior)
        └──────────────────┘
        ┌──────────────────┐
        │ 2세대의 tip 위치  │───→ 3세대 C1 (fingertip position)
        └──────────────────┘
        ┌──────────────────┐
        │ 새로 추가         │───→ C2 bone dir, C4 smooth, C5 collision, C6 limit
        └──────────────────┘
```

---

## 12. 권장 개발 로드맵

### Phase 1: 기반 구축 + 1세대 (1~2주)

```
목표: 전체 파이프라인 검증, 부호/범위/축 문제 발견

구현:
  [1A] Manus Ergonomics Direct Mapping  ← 최우선
  [1C] MediaPipe Vector Angles           ← 병렬 개발

산출물:
  retarget_dev/
  ├── core/
  │   ├── dg5f_config.py         # Joint names, limits, link lengths (URDF 파생)
  │   ├── dg5f_fk.py             # Pinocchio joint metadata (joint_names, q_min/q_max)
  │   └── filters.py             # EMA, One Euro Filter
  ├── gen1/
  │   ├── ergo_direct.py         # [1A]
  │   └── mediapipe_angles.py    # [1C]
  └── calibration/
      └── range_calibrator.py    # min-max 캘리브레이션 도구
```

**Phase 1 체크리스트**:
- [ ] 모든 20 joint 부호 방향 확인 (실기 테스트)
- [ ] Thumb opposition (rj_dg_1_2) 매핑 품질 확인
- [ ] Pinky J1=Z축, J2=X축 확인
- [ ] 캘리브레이션 후 가동 범위 커버리지
- [ ] MediaPipe depth 노이즈 수준 정량 파악

### Phase 2: 3세대 최적화 (2~3주)

```
목표: Production-quality 리타게팅

구현:
  [3A] Manus Multi-Cost Optimization  ← 최우선
  [3B] MediaPipe Multi-Cost           ← 3A 완성 후
전제:
  Phase 1 (1A 매핑, FK/joint metadata)

산출물:
  retarget_dev/
  └── gen3/
      ├── cost_functions.py      # C1~C6 개별 cost 함수 모듈
      ├── optimizer.py           # scipy SLSQP wrapper + warm start
      ├── weight_config.yaml     # 가중치 프리셋 (free/grasp/hold)
      └── retargeter.py          # 통합 retargeting 클래스 (1A/3A 모드 전환)
```

**Phase 2 체크리스트**:
- [ ] Per-frame 최적화 시간 < 5ms (120Hz에서 OK)
- [ ] Self-collision 발생 빈도 → 0에 수렴
- [ ] 가중치 프리셋별 행동 차이 확인
- [ ] 3A vs 1A 파지 성공률 비교
- [ ] 3A vs 3B 품질 비교

---

## 13. 의사결정 트리: 어떤 조합을 쓸 것인가

```
Q1: Manus 글러브 사용 가능한가?
│
├── YES
│   Q2: 필요한 정확도 수준은?
│   │
│   ├── 빠른 프로토타입 / 데모 / 검증
│   │   → [1A] Manus Ergonomics Direct Mapping
│   │     (2시간 구현, 즉시 동작)
│   │
│   └── 물체 파지(grasping) / Production 수준
│       → [3A] Manus Ergo + Skeleton Multi-Cost
│         (다중 cost 최적화, self-collision 방지 포함)
│
└── NO (카메라만)
    Q2: 필요한 정확도 수준은?
    │
    ├── 빠른 프로토타입
    │   → [1C] MediaPipe Vector Angles
    │     (반나절 구현, 글러브 불필요)
    │
    └── 물체 파지 / 최고 품질
        → [3B] MediaPipe Multi-Cost
          (해부학적 prior로 Ergo 대체)
```

---

## 14. 핵심 기술 요소 재사용 매트릭스

각 세대에서 구현한 모듈이 다음 세대에서 어떻게 재사용되는지:

```
                    1A     1B     1C     3A     3B
                   (Ergo  (Skel  (MP    (Multi (Multi
                    Dir)   Quat)  Vec)   Manus) MP)
─────────────────────────────────────────────────────
dg5f_config.py      ✓      ✓      ✓      ✓      ✓
dg5f_fk.py          ✓                    ✓      ✓
filters.py          ✓      ✓      ✓      ✓      ✓
ergo_direct.py      ✓                    ✓(C3)
cost_functions.py                        ✓      ✓
optimizer.py                             ✓      ✓
calibration.py      ✓      ✓      ✓      ✓      ✓
```

> **핵심 관찰**: 1세대의 `ergo_direct.py`는 3세대에서 C3 cost의 기반으로 재사용된다. 1A를 잘 만들면 3A의 C3가 자연스럽게 완성된다. 마찬가지로 1세대 단계에서 만든 `dg5f_fk.py` (joint metadata)는 3세대 cost function의 forward kinematics 계산에 그대로 재사용된다.
