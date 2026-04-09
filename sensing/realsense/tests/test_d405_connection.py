"""RealSense D405 connectivity diagnostic.

Runs a layered check that isolates *where* a D405 connection fails,
so we can tell apart:

    1. pyrealsense2 not installed
    2. librealsense sees no USB devices (Docker USB passthrough missing)
    3. Device visible but on USB 2.x (D405 requires USB 3.0)
    4. pipeline.start() fails (resolution / device busy / permissions)
    5. wait_for_frames() times out (driver / kernel module)
    6. The production `RSCamera` wrapper itself fails

Usage:
    cd /workspaces/tamp_ws/src
    python3 -m retarget_dev.sensing.realsense.tests.test_d405_connection

    # with options
    python3 -m retarget_dev.sensing.realsense.tests.test_d405_connection \\
        --serial 123456789012 --frames 30 --width 640 --height 480 --fps 30

Exit codes:
    0 — all stages passed
    1 — pyrealsense2 import failed
    2 — no devices enumerated (Docker passthrough issue)
    3 — pipeline.start() failed
    4 — wait_for_frames() timed out
    5 — RSCamera wrapper failed
"""

import argparse
import sys
import time

import numpy as np

from retarget_dev.sensing.realsense.config import RS_FPS, RS_HEIGHT, RS_WIDTH


def _pass(msg: str) -> None:
    print(f"[PASS] {msg}")


def _fail(msg: str) -> None:
    print(f"[FAIL] {msg}")


def _warn(msg: str) -> None:
    print(f"[WARN] {msg}")


def _info(msg: str) -> None:
    print(f"       {msg}")


# ── Stage 1 ───────────────────────────────────────────────

def stage1_import():
    print("\n=== Stage 1: pyrealsense2 import ===")
    try:
        import pyrealsense2 as rs
    except ImportError as e:
        _fail(f"pyrealsense2 not importable: {e}")
        _info("Hint: pip install pyrealsense2")
        sys.exit(1)

    version = getattr(rs, "__version__", "unknown")
    _pass(f"pyrealsense2 imported (version={version})")
    return rs


# ── Stage 2 ───────────────────────────────────────────────

def stage2_enumerate(rs):
    print("\n=== Stage 2: device enumeration (rs.context) ===")
    ctx = rs.context()
    devices = list(ctx.devices)
    if not devices:
        _fail("rs.context().devices is empty — librealsense sees no USB devices")
        _info("Host 에서는 lsusb 로 D405 가 보여도 컨테이너에서 못 보는 경우:")
        _info("  Docker 실행 옵션 확인:")
        _info("    1) docker run --privileged ...")
        _info("    2) 또는 --device=/dev/bus/usb:/dev/bus/usb")
        _info("    3) 또는 --device-cgroup-rule='c 189:* rmw'")
        _info("  컨테이너 안에서 수동 확인:")
        _info("    apt-get install -y usbutils && lsusb | grep -i intel")
        _info("    ls -la /dev/bus/usb/")
        sys.exit(2)

    _pass(f"Found {len(devices)} RealSense device(s)")
    return devices


# ── Stage 3 ───────────────────────────────────────────────

def stage3_device_info(rs, devices) -> bool:
    """Returns True if at least one device is on USB 3.x."""
    print("\n=== Stage 3: device info ===")
    any_usb3 = False
    for i, dev in enumerate(devices):
        def _get(info_key):
            try:
                return dev.get_info(info_key)
            except Exception:
                return "<unavailable>"

        name = _get(rs.camera_info.name)
        serial = _get(rs.camera_info.serial_number)
        firmware = _get(rs.camera_info.firmware_version)
        usb_type = _get(rs.camera_info.usb_type_descriptor)
        product_id = _get(rs.camera_info.product_id)

        _info(
            f"[{i}] name={name} serial={serial} fw={firmware} "
            f"usb={usb_type} pid={product_id}"
        )

        if isinstance(usb_type, str) and usb_type.startswith("3"):
            any_usb3 = True
        elif isinstance(usb_type, str) and usb_type.startswith("2"):
            _warn(
                f"[{i}] USB {usb_type} detected — D405 requires USB 3.0 for streaming"
            )

    if any_usb3:
        _pass("At least one device is on USB 3.x")
    else:
        _warn("No USB 3.x devices — streaming may fail in the next stages")
    return any_usb3


# ── Stage 4 ───────────────────────────────────────────────

def stage4_pipeline_start(rs, args):
    print("\n=== Stage 4: pipeline.start() (raw pyrealsense2) ===")
    pipeline = rs.pipeline()
    config = rs.config()

    if args.serial:
        config.enable_device(args.serial)
        _info(f"Using serial: {args.serial}")

    config.enable_stream(
        rs.stream.color, args.width, args.height, rs.format.bgr8, args.fps,
    )
    config.enable_stream(
        rs.stream.depth, args.width, args.height, rs.format.z16, args.fps,
    )

    try:
        profile = pipeline.start(config)
    except RuntimeError as e:
        _fail(f"pipeline.start() failed: {e}")
        _info("Hints:")
        _info("  - 다른 해상도 시도: --width 848 --height 480")
        _info("  - 디바이스 점유 프로세스 확인: fuser /dev/video* 2>/dev/null")
        _info("  - --serial 지정하여 특정 디바이스 선택")
        sys.exit(3)

    _pass("pipeline started")

    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()
    _info(f"depth_scale = {depth_scale:.6f} m/unit")

    color_profile = profile.get_stream(rs.stream.color).as_video_stream_profile()
    intr = color_profile.get_intrinsics()
    _info(
        f"color intrinsics: fx={intr.fx:.1f} fy={intr.fy:.1f} "
        f"ppx={intr.ppx:.1f} ppy={intr.ppy:.1f} "
        f"size={intr.width}x{intr.height} model={intr.model}"
    )

    return pipeline, depth_scale


# ── Stage 5 ───────────────────────────────────────────────

def stage5_read_frames(rs, pipeline, depth_scale, args):
    print("\n=== Stage 5: frame reception ===")
    align = rs.align(rs.stream.color)

    ok_count = 0
    first_printed = False
    t0 = time.perf_counter()

    for i in range(args.frames):
        try:
            frames = pipeline.wait_for_frames(timeout_ms=1000)
        except RuntimeError as e:
            _fail(f"wait_for_frames() timed out at frame {i}: {e}")
            _info("Hints:")
            _info("  - 조명이 매우 어둡지 않은지 확인 (D405 IR 프로젝터)")
            _info("  - USB 케이블 / 포트 재연결")
            _info("  - dmesg | tail 로 커널 로그 확인")
            pipeline.stop()
            sys.exit(4)

        aligned = align.process(frames)
        cf = aligned.get_color_frame()
        df = aligned.get_depth_frame()
        if not cf or not df:
            continue
        ok_count += 1

        if not first_printed:
            color = np.asanyarray(cf.get_data())
            depth = np.asanyarray(df.get_data()).astype(np.float32) * depth_scale
            valid = depth[depth > 0]
            if valid.size > 0:
                _info(
                    f"frame 0: color={color.shape} depth={depth.shape} "
                    f"valid={valid.size}/{depth.size} "
                    f"range=[{valid.min():.3f}, {valid.max():.3f}] m"
                )
            else:
                _info(
                    f"frame 0: color={color.shape} depth={depth.shape} "
                    f"valid=0 (모든 depth 가 무효 — 물체가 너무 가깝거나 먼지 확인)"
                )
            first_printed = True

    elapsed = time.perf_counter() - t0
    fps = ok_count / elapsed if elapsed > 0 else 0.0
    pipeline.stop()

    if ok_count == 0:
        _fail(f"0/{args.frames} frames received")
        sys.exit(4)

    _pass(f"Received {ok_count}/{args.frames} frames ({fps:.1f} FPS, {elapsed:.2f}s)")


# ── Stage 6 ───────────────────────────────────────────────

def stage6_wrapper(args):
    print("\n=== Stage 6: RSCamera wrapper (production code path) ===")
    from retarget_dev.sensing.realsense.rs_camera import RSCamera

    cam = RSCamera(
        serial=args.serial,
        width=args.width,
        height=args.height,
        fps=args.fps,
    )
    try:
        cam.start()
    except Exception as e:
        _fail(f"RSCamera.start() failed: {e}")
        sys.exit(5)

    try:
        ok_count = 0
        for _ in range(5):
            ok, color, depth_m = cam.read()
            if ok:
                ok_count += 1
        if ok_count == 0:
            _fail("RSCamera.read() returned ok=False for all 5 attempts")
            sys.exit(5)
        _pass(
            f"RSCamera wrapper works ({ok_count}/5 frames) — "
            f"same class used by RealSenseSensing"
        )
    finally:
        cam.stop()


# ── Main ──────────────────────────────────────────────────

def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="RealSense D405 connectivity diagnostic",
    )
    p.add_argument("--serial", default=None, help="Specific device serial")
    p.add_argument(
        "--frames", type=int, default=30,
        help="Number of frames to read in Stage 5 (default: 30)",
    )
    p.add_argument("--width", type=int, default=RS_WIDTH)
    p.add_argument("--height", type=int, default=RS_HEIGHT)
    p.add_argument("--fps", type=int, default=RS_FPS)
    p.add_argument(
        "--skip-wrapper", action="store_true",
        help="Skip Stage 6 (RSCamera wrapper check)",
    )
    return p.parse_args()


def main() -> None:
    args = parse_args()
    print(
        f"RealSense D405 diagnostic — {args.width}x{args.height}@{args.fps} "
        f"serial={args.serial or 'auto'} frames={args.frames}"
    )

    rs = stage1_import()
    devices = stage2_enumerate(rs)
    stage3_device_info(rs, devices)
    pipeline, depth_scale = stage4_pipeline_start(rs, args)
    stage5_read_frames(rs, pipeline, depth_scale, args)

    if not args.skip_wrapper:
        stage6_wrapper(args)

    print("\n=== All stages passed ===")
    print("D405 연결 정상. 이제 원래 명령을 실행해 보세요:")
    print(
        "  python3 -m retarget_dev.models.dex_retarget.main "
        "--sensing realsense --config <config.yml>"
    )


if __name__ == "__main__":
    main()
