"""GUI and independently runnable acquisition/export/verification commands."""
import argparse
import json
import math
import sys
import time
from pathlib import Path
from .capture import Config, Recorder, BASE, discover_ports, camera_inventory, select_port
from .storage import finalize, export_euroc
from .workflow import validate_output_root, resolve_raw_session, export_recording

def main():
    parser = argparse.ArgumentParser(description="H743 stereo viewer — read-only CDC; host receive timestamps; exposure timing unverified")
    sub = parser.add_subparsers(dest="command")
    gui = sub.add_parser("gui", help="Open the desktop viewer")
    gui.add_argument("--smoke-seconds", type=float, default=0)
    sub.add_parser("devices", help="List USB serial identities and Windows camera inventory")
    record = sub.add_parser("record", help="Headless joint acquisition followed by automatic export")
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
    recalc = sub.add_parser("finalize", help="Rebuild time indexes (legacy sessions retain their original model)")
    recalc.add_argument("session", type=Path)
    export = sub.add_parser("export", help="Export host-time EuRoC-shaped data and all-IMU image associations")
    export.add_argument("session", type=Path)
    export.add_argument("destination", type=Path)
    export.add_argument("--match-tolerance-ms", type=float, default=20.0,
                        help="One-to-one image association tolerance in ms (default: 20)")
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
        elif args.command == "export":
            if not math.isfinite(args.match_tolerance_ms * 1_000_000) or args.match_tolerance_ms < 0:
                raise ValueError("--match-tolerance-ms must be finite and non-negative")
            print(json.dumps(export_euroc(resolve_raw_session(args.session), args.destination,
                             round(args.match_tolerance_ms * 1_000_000)), ensure_ascii=False, indent=2))
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
            print("Raw recording finalized; exporting...", flush=True)
            result = export_recording(path)
            print(json.dumps(dict(path=str(path), quality=recorder.snapshot(),
                                  alignment=recorder.report, export=result), ensure_ascii=False, indent=2), flush=True)
            return 2 if recorder.error else 0
    except Exception as exc:
        print(f"{type(exc).__name__}: {exc}", file=sys.stderr)
        return 2
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
