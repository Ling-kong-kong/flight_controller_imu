#!/usr/bin/env python3
"""Read-only BMM150 diagnostics, preserving concurrent BMI088/attitude USB bytes.

No command is sent to the MCU. Opening CDC asserts DTR only. Raw and compensated
magnetic vectors use BMM150 register axes, NOT body FRD. This tool does not apply
hard/soft iron calibration or heading correction; firmware may independently aid
relative yaw (inspect IMU v3 flags), without changing the raw magnetic frames. Timestamps are register-read completion times.
"""
from __future__ import annotations
import argparse
import contextlib
import csv
from dataclasses import asdict
import json
import math
from pathlib import Path
import time

try:
    from .imu_capture import MagFrame, PairedFrame, PairedStats, SensorStats, StreamParser, ValueStats
except ImportError:
    from imu_capture import MagFrame, PairedFrame, PairedStats, SensorStats, StreamParser, ValueStats

MAG_FLAGS = {1: "valid", 2: "ready", 4: "trim_ok", 8: "configured", 16: "diagnostic"}
COLUMNS = [*MagFrame.__dataclass_fields__, "valid", "norm_ut"]


class Capture:
    """Constant-memory statistics and streaming CSV/raw output."""
    def __init__(self, raw_file=None, csv_file=None):
        self.parser = StreamParser()
        self.imu = PairedStats()
        self.mag = SensorStats(MAG_FLAGS, session_mask=0)
        self.values = {k: ValueStats() for k in ("x_raw", "y_raw", "z_raw", "rhall",
                                               "x_ut", "y_ut", "z_ut", "norm_ut")}
        self.raw_file = raw_file
        self.writer = csv.DictWriter(csv_file, COLUMNS) if csv_file else None
        if self.writer:
            self.writer.writeheader()
        self.valid_samples = 0
        self.invalid_frames = 0
        self.first_valid_us = self.last_valid_us = None
        self.latest = None

    def feed(self, data):
        if self.raw_file:
            self.raw_file.write(data)
        frames = self.parser.feed(data)
        for frame in frames:
            if isinstance(frame, PairedFrame):
                self.imu.add(frame)
        for frame in self.parser.records:
            if not isinstance(frame, MagFrame):
                continue
            self.mag.add(frame)
            self.latest = asdict(frame)
            if not frame.valid:
                for key in ("x_ut", "y_ut", "z_ut"):
                    self.latest[key] = None
            row = {**self.latest, "valid": int(frame.valid), "norm_ut": ""}
            if frame.valid:
                self.valid_samples += 1
                if self.first_valid_us is None:
                    self.first_valid_us = frame.timestamp_us
                self.last_valid_us = frame.timestamp_us
                row["norm_ut"] = math.hypot(frame.x_ut, frame.y_ut, frame.z_ut)
                for key, stat in self.values.items():
                    stat.add(row[key])
            else:
                self.invalid_frames += 1
                for key in ("x_ut", "y_ut", "z_ut"):
                    row[key] = ""
            if self.writer:
                self.writer.writerow(row)

    def summary(self):
        elapsed = ((self.last_valid_us - self.first_valid_us)
                   if self.last_valid_us is not None else 0)
        return dict(protocol="IMU2 v1 type4 72B; IMU2 v2/type3 48B or v3/type3 76B",
                    axes="raw magnetic frames: BMM150 chip/register axes; no mounting rotation",
                    magnetic_calibration="raw magnetic frames: factory trim compensation only; no hard/soft iron calibration",
                    timestamp="MCU register read completion, not conversion/DRDY time",
                    yaw="relative yaw; optional magnetic aid, inspect IMU v3 flags; no north reference",
                    parser=self.parser.summary(), imu=self.imu.summary(),
                    mag=self.mag.summary(), valid_samples=self.valid_samples,
                    invalid_frames=self.invalid_frames,
                    valid_received_hz=((self.valid_samples-1)*1e6/elapsed if elapsed > 0 else None),
                    values={key: value.summary() for key, value in self.values.items()},
                    latest=self.latest)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--port")
    source.add_argument("--input", type=Path)
    parser.add_argument("--seconds", type=float, default=30)
    parser.add_argument("--raw", type=Path)
    parser.add_argument("--csv", type=Path)
    parser.add_argument("--summary", type=Path)
    args = parser.parse_args(argv)
    if not math.isfinite(args.seconds) or args.seconds <= 0:
        parser.error("--seconds must be finite and > 0")
    paths = [p.resolve() for p in (args.input, args.raw, args.csv, args.summary) if p]
    if len(paths) != len(set(paths)):
        parser.error("input and output files must all be different")
    try:
        with contextlib.ExitStack() as stack:
            if args.port:
                import serial
                handle = stack.enter_context(serial.Serial(args.port, 115200, timeout=0.1))
            else:
                handle = stack.enter_context(args.input.open("rb"))
            raw = stack.enter_context(args.raw.open("wb")) if args.raw else None
            output = stack.enter_context(args.csv.open("w", newline="", encoding="utf-8")) if args.csv else None
            capture = Capture(raw, output)
            deadline = time.monotonic() + args.seconds
            try:
                while not args.port or time.monotonic() < deadline:
                    data = handle.read(min(4096, max(1, handle.in_waiting)) if args.port else 4096)
                    if data:
                        capture.feed(data)
                    elif not args.port:
                        break
            except KeyboardInterrupt:
                pass
            result = json.dumps(capture.summary(), indent=2, ensure_ascii=False, allow_nan=False)
            if args.summary:
                args.summary.write_text(result + "\n", encoding="utf-8")
            print(result)
    except (OSError, ImportError) as error:
        parser.exit(1, f"mag_capture: {error}\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
