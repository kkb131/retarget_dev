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

## 1. conda env 생성

```bash
cd /workspaces/tamp_ws/src/retarget_dev/models/geort
conda env create -f environment.yaml
conda activate geort_env
```

설치되는 패키지:

| 패키지 | 용도 |
|---|---|
| numpy<2 | eigenpy/pinocchio ABI 호환 (dex_retarget 와 동일 정책) |
| torch + torchvision | GeoRT IK/FK MLP 학습/추론 |
| sapien | 학습 시 FK ground-truth (Pinocchio fallback 포함) |
| pin (pinocchio) | Vulkan 실패 시 FK 백엔드 |
| mediapipe==0.10.21 | dex_retarget setup.md 의 호환 매트릭스와 동일 핀 |
| opencv-python | 웹캠 + 시각화 |
| tyro / loguru / tqdm | GeoRT 의존 |

> CUDA 변종 PyTorch 가 필요하면 `install.sh` 후 별도로 override:
> ```bash
> pip install --force-reinstall torch torchvision \
>     --index-url https://download.pytorch.org/whl/cu121
> ```

## 2. GeoRT clone + editable install

```bash
bash scripts/install.sh
```

`install.sh` 가 수행하는 작업:
1. `models/geort/GeoRT/` 가 비어 있으면 `git clone
   https://github.com/facebookresearch/GeoRT.git`
2. `pip install -e . --no-deps` — mediapipe / numpy 핀 보호
3. `pip install -r requirements.txt` — torch / sapien / scipy 등 GeoRT
   런타임 의존성. environment.yaml 에서 이미 핀된 패키지는 그대로 둠.
4. import 검증:
   ```
   geort      : .../GeoRT/geort/__init__.py
   torch      : 2.x.x (cuda: True)
   sapien     : 3.x.x
   ```

## 3. CUDA 가용성 확인

GeoRT 의 `IKModel` / `FKModel` 은 코드 곳곳에서 `.cuda()` 를 직접 호출한다.
즉 **CPU-only PyTorch 로는 trainer 와 inference 모두 동작하지 않음**.

```bash
python3 -c "import torch; print('cuda:', torch.cuda.is_available())"
# 출력이 'cuda: True' 여야 함
```

CPU 환경이라면 다음 옵션을 고려:
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
mediapipe 0.10.22+ 에서 발생. environment.yaml 의 `mediapipe==0.10.21` 핀이
다른 패키지에 의해 upgrade 된 경우. 강제 재설치:
```bash
pip install --force-reinstall --no-deps "mediapipe==0.10.21"
python3 -c "import mediapipe; print(mediapipe.__version__)"  # 0.10.21
```

### `numpy._ARRAY_API not found`
numpy 2.x 가 설치됨. `pip install "numpy<2"` 로 다운그레이드.

### `RuntimeError: CUDA error: no kernel image is available for execution`
PyTorch 의 CUDA 빌드와 GPU 의 compute capability 가 안 맞음. PC 의 GPU 에
맞는 인덱스 URL 로 PyTorch 재설치 (위 §1 참조).

### Webcam 'Cannot open webcam (device index 0)'
다른 프로세스가 카메라 점유 / `/dev/video0` 권한 / Wayland 등. `--device 1`
시도, `ls /dev/video*` 로 사용 가능 디바이스 확인.
