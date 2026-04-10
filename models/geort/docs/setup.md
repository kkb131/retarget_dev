# GeoRT Phase 0 — 설치 가이드

본 페이지는 `models/geort/` 안에서 GeoRT 를 처음 셋업하는 절차를 다룬다.
실행 walkthrough 는 [phase0_allegro.md](phase0_allegro.md) 참고.

## 0. 사전 한계 — Vulkan / SAPIEN

dex_retarget 의 [setup 가이드](../../dex_retarget/docs/setup.md) §3 에서 이미
다룬 바와 같이, 본 프로젝트의 환경은 **Vulkan 미지원** 이라 SAPIEN 의 GUI
뷰어 (`sapien.utils.Viewer`) 를 사용할 수 없다.

| GeoRT 기능 | Vulkan 필요? | 본 프로젝트에서 |
|---|---|---|
| `geort.trainer` (학습) | ❌ | 사용 가능 — `HandKinematicModel(render=False)` 사용 |
| `geort.export.load_model` + `forward()` | ❌ | 사용 가능 — pure PyTorch |
| `geort.env.hand.HandViewerEnv` (GUI) | ✅ | **사용 불가** |
| `geort.mocap.mediapipe_evaluation` (live SAPIEN viewer) | ✅ | **사용 불가** |

→ Phase 0 의 시각화는 SAPIEN 대신 [`scripts/isaacsim_replay.py`](../scripts/isaacsim_replay.py)
가 Isaac Sim 의 Allegro hand 에 `JointState` 를 publish 하는 경로로 처리.

## 1. conda env 선택 — `teleop_operator` 재사용

GeoRT 는 dex_retarget 과 의존성이 거의 같으므로 (numpy<2,
mediapipe==0.10.21, pinocchio, opencv 등) **별도 env 를 만들 필요가 없다**.
조종 PC 의 [`teleop_operator`](../../../../teleop_dev/environment.yaml) env
를 그대로 활용하고, GeoRT 가 필요로 하는 두 가지 (`torch`, `sapien`) 만
추가하는 것이 가장 깔끔하다.

```bash
conda activate teleop_operator   # teleop_dev/environment.yaml 에서 생성됨
```

teleop_operator 가 이미 갖고 있는 패키지:

| 패키지 | 핀 | GeoRT 와의 관계 |
|---|---|---|
| numpy | <2 | eigenpy/pinocchio ABI 호환 — GeoRT 도 동일 핀 |
| mediapipe | ==0.10.21 | GeoRT MediaPipeHandDetector 호환 |
| pinocchio (`pin`) | apt 또는 conda | (Vulkan 실패 시 FK 백엔드 fallback) |
| opencv-python | latest | 웹캠 + 시각화 |
| pyrealsense2 | latest | RealSense mocap collector |

GeoRT 만을 위해 추가되는 것:

| 패키지 | 용도 |
|---|---|
| torch + torchvision | GeoRT IK/FK MLP 학습/추론 |
| sapien | 학습 시 FK ground-truth (`render=False` 모드만 사용) |
| geort (editable) | 본체 (`pip install -e GeoRT/`) |

## 2. 한 번에 설치 — `install.sh`

```bash
cd /workspaces/tamp_ws/src/retarget_dev/models/geort
bash scripts/install.sh
```

`install.sh` 가 수행하는 작업 (현재 활성화된 conda env 에 설치됨):

1. `models/geort/GeoRT/` 가 비어 있으면 `git clone
   https://github.com/facebookresearch/GeoRT.git`
2. `pip install torch torchvision` — CPU 빌드 default. CUDA 가 필요하면
   다음 §3 참조.
3. `pip install sapien`
4. `pip install -e GeoRT --no-deps` — `--no-deps` 로 mediapipe / numpy 핀
   보호 (GeoRT requirements.txt 가 numpy 를 unpin 으로 끌어오는 것 차단).
5. import 검증:
   ```
   geort      : .../GeoRT/geort/__init__.py
   torch      : 2.x.x (cuda: True)
   sapien     : 3.x.x
   numpy      : 1.26.4
   mediapipe  : 0.10.21
   ```

## 3. CUDA 가용성 확인

GeoRT 의 `IKModel` / `FKModel` 은 코드 곳곳에서 `.cuda()` 를 직접 호출한다.
즉 **CPU-only PyTorch 로는 trainer 와 inference 모두 동작하지 않음**.

```bash
python3 -c "import torch; print('cuda:', torch.cuda.is_available())"
# 출력이 'cuda: True' 여야 함
```

`install.sh` 는 CPU 빌드 PyTorch 를 default 로 설치하므로, GPU 가 있는
PC 에서는 다음 명령으로 override:

```bash
pip install --force-reinstall torch torchvision \
    --index-url https://download.pytorch.org/whl/cu121
```

(`cu121` 은 CUDA 12.1 기준 — 본인 GPU 에 맞게 변경.)

CPU-only 환경이라면 다음 옵션을 고려:
- 다른 PC 에서 학습 후 `checkpoint/` 만 가져오기
- 코드 fork 후 `.cuda()` → device-aware 로 패치 (Phase 0 범위 밖)

## 4. Isaac Sim 측 사전 요건

[isaacsim_allegro_bridge.md](isaacsim_allegro_bridge.md) 참조 — joint name
일치 / topic 이름 / Allegro USD 위치 등을 사전 확인.

## 5. 자주 발생하는 문제

### `vk::PhysicalDevice::createDeviceUnique: ErrorExtensionNotPresent`
SAPIEN 이 Vulkan 렌더러를 초기화하려 함. trainer 자체는 `render=False` 로
초기화하므로 이 에러가 trainer 에서 발생하면 sapien 빌드/링크 차원의 문제일
가능성. `pip install sapien==3.0.0.dev<X>` 등으로 다운그레이드 시도.

### `mediapipe.framework not found`
mediapipe 0.10.22+ 에서 발생. teleop_operator 의 `mediapipe==0.10.21` 핀이
다른 패키지에 의해 upgrade 된 경우. 강제 재설치:
```bash
pip install --force-reinstall --no-deps "mediapipe==0.10.21"
python3 -c "import mediapipe; print(mediapipe.__version__)"  # 0.10.21
```

### `numpy._ARRAY_API not found`
numpy 2.x 가 설치됨. `pip install "numpy<2"` 로 다운그레이드. install.sh 는
`--no-deps` 로 GeoRT 본체를 설치하므로 이 에러가 발생하면 보통 torch 설치
가 numpy 를 끌어올린 케이스.

### `RuntimeError: CUDA error: no kernel image is available for execution`
PyTorch 의 CUDA 빌드와 GPU 의 compute capability 가 안 맞음. PC 의 GPU 에
맞는 인덱스 URL 로 PyTorch 재설치 (위 §3 참조).

### Webcam 'Cannot open webcam (device index 0)'
다른 프로세스가 카메라 점유 / `/dev/video0` 권한 / Wayland 등. `--device 1`
시도, `ls /dev/video*` 로 사용 가능 디바이스 확인.
