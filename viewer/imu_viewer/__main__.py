"""GUI and independently runnable acquisition/saving/verification commands."""
import argparse
import json
import sys
import time
from pathlib import Path
from .capture import Config, Recorder, BASE, discover_ports, camera_inventory, select_port
from .storage import finalize
from .workflow import validate_output_root, resolve_raw_session

def main():
    parser = argparse.ArgumentParser(description="H743 stereo viewer — read-only CDC; host receive timestamps; exposure timing unverified")
    sub = parser.add_subparsers(dest="command")
    gui = sub.add_parser("gui", help="Open the desktop viewer")
    gui.add_argument("--smoke-seconds", type=float, default=0)
    sub.add_parser("devices", help="List USB serial identities and Windows camera inventory")
    record = sub.add_parser("record", help="Headless joint acquisition with direct saved IMU/image associations")
    record.add_argument("--port")
    record.add_argument("--serial-number")
    record.add_argument("--camera", type=int, default=0)
    record.add_argument("--backend", choices=["MSMF", "DSHOW", "ANY"], default="MSMF")
    record.add_argument("--width", type=int, default=2560)
    record.add_argument("--height", type=int, default=720)
    record.add_argument("--fps", type=float, default=30)
    record.add_argument("--seconds", type=float, default=600)
    record.add_argument("--output", default=str(BASE / "recordings"))
    record.add_argument("--disk-reserve-gib", type=float, default=1)
    recalc = sub.add_parser("finalize", help="Rebuild saved time and IMU/image association indexes (stopped sessions only)")
    recalc.add_argument("session", type=Path)
    verify = sub.add_parser("verify", help="Audit raw USB/CSV/stereo/model integrity without devices")
    verify.add_argument("session", type=Path)
    verify.add_argument("--decode-all", action="store_true")
    args = parser.parse_args()
    try:
        if args.command in (None, "gui"):
            from .gui import run
            return run(getattr(args, "smoke_seconds", 0))
        if args.command == "devices":
            print(json.dumps(dict(ports=discover_ports(), cameras=camera_inventory(),
                             camera_index_mapping="OpenCV indices must be verified by preview"),
                             ensure_ascii=False, indent=2))
        elif args.command == "verify":
            from .verify import verify_session
            print(json.dumps(verify_session(resolve_raw_session(args.session), args.decode_all), ensure_ascii=False, indent=2))
        elif args.command == "finalize":
            print(json.dumps(finalize(resolve_raw_session(args.session)), ensure_ascii=False, indent=2))
        elif args.command == "record":
            if args.seconds <= 0:
                raise ValueError("--seconds must be positive")
            output_root = validate_output_root(args.output)
            identity = select_port(args.port, args.serial_number)
            recorder = Recorder(Config(port=identity["port"], serial_number=identity["serial_number"],
                camera_index=args.camera, backend=args.backend, width=args.width,
                height=args.height, fps=args.fps, output=str(output_root),
                min_free_bytes=int(args.disk_reserve_gib * 1024**3),
                camera_inventory=camera_inventory()))
            path = recorder.start()
            print(f"SESSION={path}", flush=True)
            deadline = time.perf_counter() + args.seconds
            try:
                while not recorder.done.wait(min(5, max(0.01, deadline - time.perf_counter()))):
                    s = recorder.snapshot()
                    print(json.dumps(s, ensure_ascii=False), flush=True)
                    if time.perf_counter() >= deadline:
                        break
            except KeyboardInterrupt:
                print("Stopping and draining writer...", flush=True)
            finally:
                recorder.stop()
                if not recorder.wait(100):
                    print("Recorder shutdown timed out; session may be incomplete", file=sys.stderr)
                    return 2
            if recorder.error:
                print(f"保存失败: {recorder.error}；已写入数据保留在 {path}，可能不完整", file=sys.stderr)
                return 2
            print("保存完成", flush=True)
            print(json.dumps(dict(path=str(path), association_table=str(path / "imu_image.csv"),
                                  quality=recorder.snapshot(), timing=recorder.report),
                             ensure_ascii=False, indent=2), flush=True)
            return 0
    except Exception as exc:
        print(f"{type(exc).__name__}: {exc}", file=sys.stderr)
        return 2
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
