#!/usr/bin/env python3
"""Read-only BMI088 1000 Hz paired USB capture; IMU2 / legacy IMU1 decoder."""

from __future__ import annotations

import argparse
import contextlib
import csv
from dataclasses import dataclass
import json
import math
from pathlib import Path
import struct
import sys
import time
from typing import BinaryIO
import zlib

MAGIC = b"IMU2"
VERSION = 2
FRAME_SIZE = 48
FRAME_STRUCT = struct.Struct("<4sBBHIQhhhhhhIIHHI")
LEGACY_MAGIC = b"IMU1"
LEGACY_FRAME_SIZE = 32
LEGACY_FRAME_STRUCT = struct.Struct("<4sBBHIQhhhHI")
SENSOR_NAMES = {1: "accel", 2: "gyro"}
LEGACY_FLAG_NAMES = {1: "drdy_interval", 2: "spi_error", 4: "ring_overflow", 8: "session_start"}
FLAG_NAMES = {
    1: "output_timing", 2: "accel_spi_error", 4: "gyro_spi_error", 8: "ring_overflow",
    16: "session_start", 32: "accel_stale", 64: "gyro_stale",
    128: "accel_drdy_interval", 256: "gyro_drdy_interval", 512: "history_missing",
}


@dataclass(frozen=True)
class Frame:
    """Legacy IMU1 sensor frame, retained for decoding existing captures."""
    sensor_type: int
    sequence: int
    timestamp_us: int
    x: int
    y: int
    z: int
    flags: int = 0

    @property
    def name(self) -> str:
        return SENSOR_NAMES[self.sensor_type]

    def encode(self) -> bytes:
        """For fixtures/offline work; the capture utility never transmits frames."""
        data = LEGACY_FRAME_STRUCT.pack(LEGACY_MAGIC, 1, self.sensor_type, LEGACY_FRAME_SIZE,
                                       self.sequence, self.timestamp_us,
                                       self.x, self.y, self.z, self.flags, 0)
        return data[:28] + struct.pack("<I", zlib.crc32(data[:28]))


@dataclass(frozen=True)
class PairedFrame:
    sequence: int
    timestamp_us: int
    ax: int
    ay: int
    az: int
    gx: int
    gy: int
    gz: int
    accel_age_us: int
    gyro_age_us: int
    flags: int = 0
    tick_late_us: int = 0

    @property
    def accel_time_us(self) -> int:
        return self.timestamp_us - self.accel_age_us

    @property
    def gyro_time_us(self) -> int:
        return self.timestamp_us - self.gyro_age_us

    def encode(self) -> bytes:
        """For offline fixtures only; the capture utility does not transmit."""
        data = FRAME_STRUCT.pack(MAGIC, VERSION, 3, FRAME_SIZE,
                                 self.sequence, self.timestamp_us,
                                 self.ax, self.ay, self.az, self.gx, self.gy, self.gz,
                                 self.accel_age_us, self.gyro_age_us,
                                 self.flags, self.tick_late_us, 0)
        return data[:44] + struct.pack("<I", zlib.crc32(data[:44]))


class StreamParser:
    """Incremental decoder. Internal storage is bounded even for a huge feed()."""

    BUFFER_LIMIT = 4096

    def __init__(self) -> None:
        self.buffer = bytearray()
        self.bytes_received = 0
        self.bytes_discarded = 0
        self.crc_errors = 0
        self.header_errors = 0
        self.frames = 0

    def feed(self, data: bytes) -> list[Frame | PairedFrame]:
        self.bytes_received += len(data)
        result: list[Frame | PairedFrame] = []
        for start in range(0, len(data), self.BUFFER_LIMIT - FRAME_SIZE):
            self.buffer.extend(data[start:start + self.BUFFER_LIMIT - FRAME_SIZE])
            self._drain(result)
        return result

    def _discard(self, count: int) -> None:
        del self.buffer[:count]
        self.bytes_discarded += count

    def _drain(self, result: list[Frame | PairedFrame]) -> None:
        while True:
            indices = [index for magic in (MAGIC, LEGACY_MAGIC)
                       if (index := self.buffer.find(magic)) >= 0]
            if not indices:
                self._discard(max(0, len(self.buffer) - (len(MAGIC) - 1)))
                return
            index = min(indices)
            if index:
                self._discard(index)
            if len(self.buffer) < 8:
                return
            paired = self.buffer[:4] == MAGIC
            size = FRAME_SIZE if paired else LEGACY_FRAME_SIZE
            expected_version = VERSION if paired else 1
            valid_type = self.buffer[5] == 3 if paired else self.buffer[5] in SENSOR_NAMES
            if (self.buffer[4] != expected_version or not valid_type
                    or struct.unpack_from("<H", self.buffer, 6)[0] != size):
                self.header_errors += 1
                self._discard(1)
                continue
            if len(self.buffer) < size:
                return
            wanted_crc = struct.unpack_from("<I", self.buffer, size - 4)[0]
            if zlib.crc32(self.buffer[:size - 4]) != wanted_crc:
                self.crc_errors += 1
                self._discard(1)
                continue
            if paired:
                fields = FRAME_STRUCT.unpack_from(self.buffer)
                result.append(PairedFrame(*fields[4:16]))
            else:
                fields = LEGACY_FRAME_STRUCT.unpack_from(self.buffer)
                result.append(Frame(fields[2], fields[4], fields[5],
                                    fields[6], fields[7], fields[8], fields[9]))
            del self.buffer[:size]
            self.frames += 1

    def summary(self) -> dict:
        return {"bytes_received": self.bytes_received, "valid_frames": self.frames,
                "bytes_discarded": self.bytes_discarded,
                "crc_errors": self.crc_errors, "header_errors": self.header_errors,
                "trailing_bytes": len(self.buffer)}


class SensorStats:
    """Sequence/timestamp statistics; legacy defaults are preserved for offline use."""

    def __init__(self, flag_names=None, session_mask=8) -> None:
        self.samples = 0
        self.sessions = 0
        self.missing_sequences = 0
        self.duplicate_sequences = 0
        self.sequence_regressions = 0
        self.timestamp_regressions = 0
        self.flag_names = LEGACY_FLAG_NAMES if flag_names is None else flag_names
        self.session_mask = session_mask
        self.flags = {name: 0 for name in self.flag_names.values()}
        self.previous: Frame | PairedFrame | None = None
        self.elapsed_us = 0
        self.received_intervals = 0
        self.sequence_intervals = 0
        self.period_count = 0
        self.period_mean = 0.0
        self.period_m2 = 0.0
        self.period_min: int | None = None
        self.period_max: int | None = None

    def add(self, frame: Frame | PairedFrame) -> None:
        self.samples += 1
        for mask, name in self.flag_names.items():
            if frame.flags & mask:
                self.flags[name] += 1
        previous = self.previous
        self.previous = frame
        if previous is None or frame.flags & self.session_mask:
            self.sessions += 1
            return
        delta_seq = (frame.sequence - previous.sequence) & 0xFFFFFFFF
        if delta_seq == 0:
            self.duplicate_sequences += 1
        elif delta_seq >= 0x80000000:
            self.sequence_regressions += 1
        elif delta_seq > 1:
            self.missing_sequences += delta_seq - 1
        delta_time = frame.timestamp_us - previous.timestamp_us
        if delta_time <= 0:
            self.timestamp_regressions += 1
            return
        if not (0 < delta_seq < 0x80000000):
            return
        self.elapsed_us += delta_time
        self.received_intervals += 1
        self.sequence_intervals += delta_seq
        if delta_seq == 1:
            self.period_count += 1
            difference = delta_time - self.period_mean
            self.period_mean += difference / self.period_count
            self.period_m2 += difference * (delta_time - self.period_mean)
            self.period_min = delta_time if self.period_min is None else min(self.period_min, delta_time)
            self.period_max = delta_time if self.period_max is None else max(self.period_max, delta_time)

    def summary(self) -> dict:
        return {
            "samples": self.samples, "sessions_observed": self.sessions,
            "received_hz": (self.received_intervals * 1e6 / self.elapsed_us
                            if self.elapsed_us else None),
            "sequence_hz": (self.sequence_intervals * 1e6 / self.elapsed_us
                            if self.elapsed_us else None),
            "missing_sequences": self.missing_sequences,
            "duplicate_sequences": self.duplicate_sequences,
            "sequence_regressions": self.sequence_regressions,
            "timestamp_regressions": self.timestamp_regressions,
            "continuous_periods": self.period_count,
            "period_us_min": self.period_min, "period_us_max": self.period_max,
            "period_us_mean": self.period_mean if self.period_count else None,
            "period_us_std": (math.sqrt(max(0.0, self.period_m2 / self.period_count))
                              if self.period_count else None),
            "flag_counts": self.flags.copy(),
        }


class ValueStats:
    def __init__(self) -> None:
        self.count = 0
        self.mean = 0.0
        self.m2 = 0.0
        self.minimum = None
        self.maximum = None

    def add(self, value: int) -> None:
        self.count += 1
        difference = value - self.mean
        self.mean += difference / self.count
        self.m2 += difference * (value - self.mean)
        self.minimum = value if self.minimum is None else min(value, self.minimum)
        self.maximum = value if self.maximum is None else max(value, self.maximum)

    def summary(self) -> dict:
        return {"count": self.count, "min": self.minimum, "max": self.maximum,
                "mean": self.mean if self.count else None,
                "std": math.sqrt(max(0.0, self.m2 / self.count)) if self.count else None}


class PairedStats(SensorStats):
    def __init__(self) -> None:
        super().__init__(FLAG_NAMES, session_mask=16)
        self.accel_age = ValueStats()
        self.gyro_age = ValueStats()
        self.source_skew = ValueStats()
        self.tick_late = ValueStats()
        self.source_repeats = {"accel": 0, "gyro": 0}
        self.source_regressions = {"accel": 0, "gyro": 0}
        self.invalid_source_times = 0

    def add(self, frame: PairedFrame) -> None:
        previous = self.previous
        super().add(frame)
        self.accel_age.add(frame.accel_age_us)
        self.gyro_age.add(frame.gyro_age_us)
        self.source_skew.add(abs(frame.accel_age_us - frame.gyro_age_us))
        self.tick_late.add(frame.tick_late_us)
        if min(frame.accel_time_us, frame.gyro_time_us) < 0:
            self.invalid_source_times += 1
        if previous is not None and not (frame.flags & 16):
            for sensor in ("accel", "gyro"):
                delta = getattr(frame, sensor + "_time_us") - getattr(previous, sensor + "_time_us")
                if delta == 0:
                    self.source_repeats[sensor] += 1
                elif delta < 0:
                    self.source_regressions[sensor] += 1

    def summary(self) -> dict:
        return {**super().summary(), "accel_age_us": self.accel_age.summary(),
                "gyro_age_us": self.gyro_age.summary(),
                "source_skew_us": self.source_skew.summary(),
                "tick_late_us": self.tick_late.summary(),
                "source_repeats": self.source_repeats.copy(),
                "source_regressions": self.source_regressions.copy(),
                "invalid_source_times": self.invalid_source_times}


class Capture:
    def __init__(self, raw_file: BinaryIO | None = None, csv_file=None, units: bool = False) -> None:
        self.parser = StreamParser()
        self.imu = PairedStats()
        self.sensors = {sensor: SensorStats() for sensor in SENSOR_NAMES}
        self.raw_file = raw_file
        self.units = units
        self.csv_writer = csv.writer(csv_file) if csv_file is not None else None
        if self.csv_writer:
            columns = ["time_us", "sequence", "ax_raw", "ay_raw", "az_raw",
                       "gx_raw", "gy_raw", "gz_raw", "accel_age_us", "gyro_age_us",
                       "accel_time_us", "gyro_time_us", "flags", "tick_late_us",
                       "protocol_version", "legacy_sensor"]
            if units:
                columns += ["ax_g", "ay_g", "az_g", "gx_deg_s", "gy_deg_s", "gz_deg_s"]
            self.csv_writer.writerow(columns)

    def feed(self, data: bytes) -> None:
        if self.raw_file is not None:
            self.raw_file.write(data)
        for frame in self.parser.feed(data):
            if isinstance(frame, PairedFrame):
                self.imu.add(frame)
                axes = [frame.ax, frame.ay, frame.az, frame.gx, frame.gy, frame.gz]
                row = [frame.timestamp_us, frame.sequence, *axes,
                       frame.accel_age_us, frame.gyro_age_us,
                       frame.accel_time_us, frame.gyro_time_us,
                       frame.flags, frame.tick_late_us, 2, ""]
            else:
                self.sensors[frame.sensor_type].add(frame)
                axes = ([frame.x, frame.y, frame.z, "", "", ""] if frame.sensor_type == 1
                        else ["", "", "", frame.x, frame.y, frame.z])
                row = [frame.timestamp_us, frame.sequence, *axes,
                       "", "", frame.timestamp_us if frame.sensor_type == 1 else "",
                       frame.timestamp_us if frame.sensor_type == 2 else "",
                       frame.flags, "", 1, frame.name]
            if self.csv_writer:
                if self.units:
                    row += [value * ((24.0 if i < 3 else 2000.0) / 32768.0)
                            if value != "" else "" for i, value in enumerate(axes)]
                self.csv_writer.writerow(row)

    def summary(self) -> dict:
        return {"parser": self.parser.summary(), "imu": self.imu.summary(),
                "legacy": {name: self.sensors[sensor].summary()
                           for sensor, name in SENSOR_NAMES.items()}}


def make_arguments() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--port", help="USB CDC port, e.g. /dev/ttyACM0 or COM5")
    source.add_argument("--input", type=Path, help="decode a previously captured raw binary file")
    parser.add_argument("--seconds", type=float, default=0,
                        help="live capture duration; 0 means until Ctrl+C (default: 0)")
    parser.add_argument("--raw", type=Path, help="save unchanged USB bytes, including corrupt data/noise")
    parser.add_argument("--csv", type=Path, help="save decoded raw samples to CSV")
    parser.add_argument("--units", action="store_true", help="add g / deg/s columns to CSV")
    parser.add_argument("--stats", action="store_true", help="print final JSON statistics")
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = make_arguments()
    args = parser.parse_args(argv)
    if not math.isfinite(args.seconds) or args.seconds < 0:
        parser.error("--seconds must be finite and >= 0")
    if args.input and args.seconds:
        parser.error("--seconds is only applicable to --port")
    paths = [path.resolve() for path in (args.input, args.raw, args.csv) if path is not None]
    if len(set(paths)) != len(paths):
        parser.error("--input, --raw and --csv must use different files")
    if args.units and not args.csv:
        parser.error("--units requires --csv")

    try:
        with contextlib.ExitStack() as stack:
            if args.port:
                try:
                    import serial
                except ImportError as error:
                    raise RuntimeError("Live capture needs pyserial: python3 -m pip install pyserial") from error
                source = stack.enter_context(serial.Serial(args.port, baudrate=115200, timeout=0.1))
            else:
                source = stack.enter_context(args.input.open("rb"))
            raw_file = stack.enter_context(args.raw.open("wb")) if args.raw else None
            csv_file = stack.enter_context(args.csv.open("w", newline="", encoding="utf-8")) if args.csv else None
            capture = Capture(raw_file, csv_file, args.units)
            deadline = time.monotonic() + args.seconds if args.seconds else None
            try:
                while deadline is None or time.monotonic() < deadline:
                    if args.port:
                        data = source.read(min(4096, max(1, source.in_waiting)))
                    else:
                        data = source.read(4096)
                        if not data:
                            break
                    if data:
                        capture.feed(data)
            except KeyboardInterrupt:
                pass
            if args.stats:
                print(json.dumps(capture.summary(), ensure_ascii=False, indent=2))
    except (OSError, RuntimeError) as error:
        parser.exit(1, f"imu_capture: {error}\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
