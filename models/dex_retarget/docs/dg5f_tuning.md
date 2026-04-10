# DG-5F 튜닝 경험

dex-retargeting을 DG-5F에 적용하면서 발견한 문제들과 해결 방법.
실험으로 검증된 내용만 기록합니다.

> **관련**: Manus 글러브 경로에서 발견된 fist→spread inversion 버그 (frame 누락 +
> MANO indices off-by-one 두 가지 동시) 는 별도 문서 [`manus_debug.md`](manus_debug.md)
> 에서 다룹니다.

## 초기 문제

DG-5F 기본 config(vector + scaling_factor=1.0)로 mp4를 재생하면:

1. **손가락이 펴진 상태에서 약간 구부정함** — open hand인데 완전히 펴지지 않음
2. **손등으로 뒤집어짐** — 음수 각도(-PIP/-DIP)가 사용되어 손가락이 반대로 꺾임
3. **Pinch 동작 부정확** — 엄지-검지가 실제로 안 만남

## 원인 분석

### 1. 사람 손과 DG-5F 크기 차이

q=0 (open hand)에서 FK로 계산한 fingertip 거리:

| Finger | Human (mm) | DG-5F (mm) | 비율 (DG5F/Human) |
|---|---|---|---|
| Thumb | 130 | 128 | 0.98 |
| Index | 148 | 198 | **1.33** |
| Middle | 165 | 200 | **1.21** |
| Ring | 157 | 193 | **1.23** |
| Pinky | 144 | 170 | **1.18** |

→ DG-5F가 사람 손보다 **약 1.2~1.3배 큼** (엄지 제외).

### 2. URDF joint limits가 너무 넓음

원본 DG-5F URDF:
- PIP (`rj_dg_*_3`): `[-90°, +90°]` — 음수 방향은 손등으로 꺾임!
- DIP (`rj_dg_*_4`): `[-90°, +90°]` — 동일

dex-retargeting optimizer는 joint limits 내에서만 해를 찾으므로,
음수 영역이 허용되면 "방향만 맞추는" 해로 음수를 사용해버림.

## 튜닝 파라미터 & 실험 결과

### 1. `scaling_factor` (사람→로봇 벡터 크기 비율)

기본: 1.0

| 값 | 결과 |
|---|---|
| 1.0 | 손가락이 구부정함, 손등 뒤집힘 빈발 |
| **1.2** | 유의미한 개선 ✓ |
| **1.3+** | 손등 뒤집힘 거의 사라짐 ✓ |
| 1.4+ | vector 타입일 때 pinch 실패 ✗ |

**원리**: 사람 손 벡터를 1.2~1.3배 확대하여 DG-5F 스케일에 맞춤.
scale이 너무 작으면 로봇 기구학이 해당 범위를 커버 못 해 구부정한 자세가 최적해가 됨.

### 2. `type: vector` vs `DexPilot`

| Type | Pinch 성능 | 일반 동작 | scaling_factor 허용 범위 |
|---|---|---|---|
| vector | **❌ 1.3+에서 실패** | ✓ 안정적 | 1.0~1.2 |
| **DexPilot** | **✓ 정확** | ✓ 안정적 | 1.2~1.4 권장 |

**DexPilot의 pinch 보존 원리**:
- Vector cost + **손가락 쌍 간 거리 항** (`L_pairwise`)
- 엄지-검지가 가까우면 (`d < threshold=0.03m`) 가중치 증가
- → 두 손가락 끝의 거리를 정확히 보존

**결론**: 일반 용도는 DexPilot 권장.

### 3. 중간 관절 벡터 추가 (MCP→DIP)

`target_link_human_indices`에 MCP-DIP 벡터 5개 추가:
```yaml
target_origin_link_names:
  - "rl_dg_palm"  # x5
  - "rl_dg_1_2"   # Thumb MCP
  - "rl_dg_2_2"   # Index MCP
  - ...
target_task_link_names:
  - "rl_dg_*_tip"  # x5
  - "rl_dg_*_4"    # DIP link x5
target_link_human_indices:
  - [0, 0, 0, 0, 0, 2, 6, 10, 14, 18]
  - [4, 8, 12, 16, 20, 3, 7, 11, 15, 19]
```

**결과**: 유의미한 차이 없음.
→ palm→tip 5개 벡터만으로도 충분. 추가 벡터는 과구속(over-constrained).

**향후 실험 아이디어**: DIP→tip 같은 다른 조합은 차이를 만들 수도 있음 (미검증).

### 4. URDF joint limits 수정 (**최종 해결**)

PIP/DIP의 lower limit을 `-90°` → `0°`로 변경하여 optimizer가 음수 해를 선택할 수 없도록 강제.

**방법**: retargeting 전용 URDF 복사본 생성 (원본은 건드리지 않음)
```bash
cp dg5f_right.urdf retarget_dev/models/dex_retarget/config/dg5f_right_retarget.urdf
```

수정된 관절 (10개):
```
rj_dg_1_3, rj_dg_1_4  (Thumb PIP/DIP)
rj_dg_2_3, rj_dg_2_4  (Index PIP/DIP)
rj_dg_3_3, rj_dg_3_4  (Middle PIP/DIP)
rj_dg_4_3, rj_dg_4_4  (Ring PIP/DIP)
rj_dg_5_3, rj_dg_5_4  (Pinky PIP/DIP)
```

변경 내용 (XML):
```xml
<!-- 변경 전 -->
<limit lower="-1.5707963267948966" upper="1.5707963267948966" .../>
<!-- 변경 후 -->
<limit lower="0.0" upper="1.5707963267948966" .../>
```

**주의**: Thumb MCP (`rj_dg_1_2`)는 `[-180°, 0°]`로 **음수가 flexion**이므로 건드리지 않음.

config에서 새 URDF 경로 지정:
```yaml
urdf_path: /workspaces/tamp_ws/src/retarget_dev/models/dex_retarget/config/dg5f_right_retarget.urdf
```

**결과**: 손등으로 뒤집히는 현상 **완전 제거**.

## 권장 설정

검증된 설정:

```yaml
# dg5f_right_dexpilot.yml
retargeting:
  type: DexPilot                    # pinch 정확
  urdf_path: .../dg5f_right_retarget.urdf  # PIP/DIP 음수 제한

  wrist_link_name: "rl_dg_palm"
  finger_tip_link_names:
    - "rl_dg_1_tip"
    - "rl_dg_2_tip"
    - "rl_dg_3_tip"
    - "rl_dg_4_tip"
    - "rl_dg_5_tip"

  scaling_factor: 1.2               # 1.2~1.4 범위에서 조정
  low_pass_alpha: 0.2               # 스무딩 (작을수록 smooth, latency ↑)
```

## 향후 실험 가능한 파라미터

### `low_pass_alpha`
- 현재 0.2 (적당히 부드러움)
- 0.1: 더 부드러움, 지연 증가
- 0.3+: 빠른 반응, jitter 가능

### `DexPilot project_dist`, `escape_dist`
- 기본: project_dist=0.03m, escape_dist=0.05m
- 손가락이 "붙었다"고 판단하는 거리 기준
- pinch가 너무 쉽게/어렵게 되면 조정

### Optimization weights (`normal_delta`, `huber_delta`)
- `normal_delta`: L2 regularization (이전 해 대비 변화 제한, temporal smoothness)
- `huber_delta`: Huber loss threshold (outlier robustness)
- 기본값이 대부분 잘 동작

## 트러블슈팅 체크리스트

| 증상 | 원인 | 해결 |
|---|---|---|
| 손가락이 살짝 구부정함 | scaling_factor 낮음 | 1.2~1.3으로 증가 |
| 손등으로 뒤집어짐 | PIP/DIP 음수 허용 | retarget URDF 사용 |
| Pinch 실패 (엄지-검지 안 만남) | vector + 높은 scale | DexPilot으로 변경 |
| 움직임이 끊김/jitter | low_pass_alpha 높음 | 0.1~0.2로 감소 |
| 전체적으로 반응 느림 | low_pass_alpha 낮음 | 0.2~0.3으로 증가 |
| 엄지가 이상함 | Thumb 특수성 무시 | Thumb MCP는 [-180°, 0°] 범위 유지 확인 |
