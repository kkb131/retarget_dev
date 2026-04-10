# retarget_dev/models/geort

GeoRT (https://github.com/facebookresearch/GeoRT) 통합. 신경망 기반 hand
retargeting (1 kHz 추론 가능). **현재 Phase 0 (Allegro 검증) 만 구현되어
있음** — DG-5F 통합 (Phase 1+) 은 별도 작업이며
[../../docs/geort.md](../../docs/geort.md) §9 의 로드맵을 따른다.

## 빠른 시작

```bash
conda activate teleop_operator             # 기존 env 재사용 (별도 env 불필요)
cd /workspaces/tamp_ws/src/retarget_dev/models/geort
bash scripts/install.sh                    # GeoRT clone + torch + sapien
```

| 단계 | 명령 | 가이드 |
|---|---|---|
| 설치 | `bash scripts/install.sh` | [docs/setup.md](docs/setup.md) |
| Phase 0 walkthrough | `docs/phase0_allegro.md` | [docs/phase0_allegro.md](docs/phase0_allegro.md) |
| Isaac Sim 연결 | (사전 요건 체크리스트) | [docs/isaacsim_allegro_bridge.md](docs/isaacsim_allegro_bridge.md) |

> **conda env**: 별도 `geort_env` 를 만들지 않는다. teleop_operator 가
> 이미 numpy<2 / mediapipe==0.10.21 / pinocchio / opencv / pyrealsense2 를
> 모두 갖고 있어서, GeoRT 가 추가로 필요한 것은 `torch + sapien` 두 가지뿐.
> install.sh 가 활성화된 env 에 그 두 패키지 + GeoRT 본체를 설치한다.

## 디렉토리

```
models/geort/
├── README.md                          # 이 파일
├── docs/
│   ├── setup.md                       # 환경 + GeoRT install
│   ├── phase0_allegro.md              # 5단계 walkthrough
│   └── isaacsim_allegro_bridge.md     # Isaac Sim 측 사전 요건
├── scripts/
│   ├── install.sh                     # GeoRT clone + torch/sapien 설치
│   ├── collect_mocap_webcam.py        # 웹캠으로 사람 mocap 수집
│   ├── collect_mocap_realsense.py     # RealSense D405 로 사람 mocap 수집
│   ├── train_allegro.sh               # geort.trainer 래퍼
│   └── isaacsim_replay.py             # 학습 결과를 Isaac Sim Allegro 에 publish
├── data/                              # mocap .npy 저장소 (gitignored)
├── checkpoint/                        # 학습 산출물 (gitignored)
└── GeoRT/                             # vendored upstream (gitignored —
                                       #   install.sh 가 clone)
```

## 라이선스 주의

GeoRT 본체는 **CC-BY-NC 4.0** (비상업적 연구용). 본 프로젝트가 학술/내부
연구 용도이면 문제 없으나, 향후 product transition 시 dex_retarget 으로
교체해야 함을 인지. 자세한 내용은 [../../docs/geort.md](../../docs/geort.md)
§2.1, §10 참조.
