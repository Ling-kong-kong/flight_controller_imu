#!/usr/bin/env python3
"""Replay paired usb.bin through actual native C fusion; no serial/flash access.

Capture timestamps, raw registers and source ages are preserved. Output attitude
flags are recomputed, not copied. A USB capture may omit on-board fusion inputs;
this is a fresh software replay, NOT a reconstruction of hidden queue events or
proof of hardware attitude accuracy. Legacy IMU1 cannot supply paired inputs.
"""
from __future__ import annotations

import argparse
import ctypes
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import tempfile

try:
    from .imu_capture import ATTITUDE_FIELDS, ControlReply, PairedFrame, SensorStats, StreamParser, FLAG_NAMES
except ImportError:
    from imu_capture import ATTITUDE_FIELDS, ControlReply, PairedFrame, SensorStats, StreamParser, FLAG_NAMES

ROOT = Path(__file__).resolve().parents[1]


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as handle:
        for chunk in iter(lambda: handle.read(65536), b''):
            digest.update(chunk)
    return digest.hexdigest()


class NativeFusion:
    """Temporary native build of supplied real firmware sources, with provenance."""
    def __init__(self, source, include, compiler=None):
        self.source, self.include = Path(source).resolve(), Path(include).resolve()
        self.env = os.environ.copy()
        if os.name == 'nt':
            self.env['PATH'] = r'D:\msys64\ucrt64\bin' + os.pathsep + self.env['PATH']
        self.compiler = compiler or shutil.which('cc', path=self.env['PATH'])
        if not self.compiler:
            raise RuntimeError('Native GCC/Clang required; supply --cc or add cc to PATH')
        self.temp = tempfile.TemporaryDirectory(prefix='imu-fusion-replay-')
        self.lib = None
        try:
            library = Path(self.temp.name) / ('fusion.dll' if os.name == 'nt' else 'fusion.so')
            command = [self.compiler, '-std=c11', '-O2', '-Wall', '-Wextra', '-Werror',
                       '-fPIC', '-shared', '-I', str(self.include),
                       '-I', str(ROOT/'firmware/include'), str(ROOT/'tools/native/fusion_replay.c'),
                       str(self.source), '-lm', '-o', str(library)]
            subprocess.run(command, env=self.env, check=True, capture_output=True, text=True)
            self.provenance = {
                'fusion_source': str(self.source), 'fusion_source_sha256': sha256(self.source),
                'fusion_header': str(self.include/'fusion.h'),
                'fusion_header_sha256': sha256(self.include/'fusion.h'),
                'adapter_sha256': sha256(ROOT/'tools/native/fusion_replay.c'),
                'stream_header_sha256': sha256(ROOT/'firmware/include/stream.h'),
                'replay_script_sha256': sha256(Path(__file__)),
                'parser_sha256': sha256(ROOT/'tools/imu_capture.py'),
                'compiler': subprocess.check_output([self.compiler, '--version'], env=self.env, text=True).splitlines()[0],
                'compile_command': command,
            }
            self.lib = ctypes.CDLL(str(library))
            self.lib.replay_update.argtypes = [ctypes.c_uint64, ctypes.POINTER(ctypes.c_int16),
                ctypes.POINTER(ctypes.c_int16), ctypes.c_uint32, ctypes.c_uint32, ctypes.c_uint16]
            self.lib.replay_update.restype = ctypes.c_uint
            self.lib.replay_reset.argtypes = []
            self.lib.replay_reset.restype = None
            self.lib.replay_values.restype = None
            self.lib.replay_resets.argtypes = []
            self.lib.replay_values.argtypes = [ctypes.POINTER(ctypes.c_float)]
            self.lib.replay_resets.restype = ctypes.c_uint
            self.lib.replay_recalibrate.argtypes = []
            self.lib.replay_recalibrate.restype = None
            self.lib.replay_reset()
        except BaseException:
            self.close()
            raise

    def update(self, frame):
        a = (ctypes.c_int16*3)(frame.ax, frame.ay, frame.az)
        g = (ctypes.c_int16*3)(frame.gx, frame.gy, frame.gz)
        # Only acquisition flags are inputs. Recorded calibration/reset/valid
        # bits are the old firmware's output, NOT replay control instructions.
        flags = self.lib.replay_update(frame.timestamp_us, a, g, frame.accel_age_us,
                                       frame.gyro_age_us, frame.flags & 0x3ff)
        values = (ctypes.c_float*10)()
        self.lib.replay_values(values)
        return flags, list(values)

    def close(self):
        if self.lib is not None:
            if os.name == 'nt':
                import _ctypes
                _ctypes.FreeLibrary(self.lib._handle)
            self.lib = None
        self.temp.cleanup()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()


class Moments:
    def __init__(self):
        self.n = 0
        self.mean = self.m2 = 0.0
        self.minimum = self.maximum = None

    def add(self, value):
        self.n += 1
        delta = value-self.mean
        self.mean += delta/self.n
        self.m2 += delta*(value-self.mean)
        self.minimum = value if self.minimum is None else min(self.minimum, value)
        self.maximum = value if self.maximum is None else max(self.maximum, value)

    def summary(self):
        return dict(n=self.n, mean=self.mean if self.n else None,
                    std=math.sqrt(max(0, self.m2/self.n)) if self.n else None,
                    min=self.minimum, max=self.maximum)


def replay(path, native, writer=None, chunk_size=65536):
    native.lib.replay_reset()  # Each input is an independent fresh-state replay.
    parser = StreamParser()
    timing = SensorStats(FLAG_NAMES, session_mask=16)
    moments = {name: Moments() for name in ATTITUDE_FIELDS[4:]}
    qnorm = Moments()
    frames = valid = calibrating = recorded_valid = resets = nonpaired = 0
    first_t = last_t = first_valid = last_valid = None
    previous_valid = None
    transitions = []  # bounded even for a damaged/long capture
    transition_count = 0
    last_values = None
    pending_boundary = None
    last_control = None
    manual_resets = 0
    with Path(path).open('rb') as handle:
        for chunk in iter(lambda: handle.read(chunk_size), b''):
            parser.feed(chunk)
            for f in parser.records:
                if isinstance(f, ControlReply):
                    key = (f.request_id, f.timestamp_us, f.sequence)
                    if f.status == 0 and key != last_control:
                        pending_boundary = f.timestamp_us
                        last_control = key
                    continue
                if not isinstance(f, PairedFrame):
                    nonpaired += 1
                    continue
                timing.add(f)
                frames += 1
                if first_t is None:
                    first_t = f.timestamp_us
                last_t = f.timestamp_us
                recorded_valid += int(f.attitude_valid)
                if pending_boundary is not None and f.timestamp_us >= pending_boundary:
                    native.lib.replay_recalibrate()
                    manual_resets += 1
                    pending_boundary = None
                flags, values = native.update(f)
                last_values = values
                is_valid = bool(flags & 2048) and not flags & (1024 | 4096)
                calibrating += bool(flags & 1024)
                resets += bool(flags & 4096)
                elapsed = (last_t-first_t)/1e6
                if is_valid != previous_valid:
                    transition_count += 1
                    if len(transitions) < 200:
                        transitions.append(dict(frame_index=frames-1, elapsed_s=elapsed,
                                                timestamp_us=last_t, valid=is_valid))
                    previous_valid = is_valid
                if is_valid:
                    valid += 1
                    if first_valid is None:
                        first_valid = elapsed
                    last_valid = elapsed
                    for name, value in zip(ATTITUDE_FIELDS[4:], values[4:7]):
                        moments[name].add(value)
                    qnorm.add(sum(v*v for v in values[:4]))
                if writer:
                    writer.writerow([f.sequence, last_t, elapsed, flags, int(is_valid),
                                     *(values[:7] if is_valid else ['']*7)])
    if not frames:
        raise ValueError('No paired IMU2 v2/v3 frames found (IMU1 is not replayable)')
    return {
        'input': str(Path(path).resolve()), 'input_sha256': sha256(path),
        'parser': parser.summary(), 'nonpaired_frames_skipped': nonpaired,
        'paired_frames': frames, 'recorded_valid_attitude_frames': recorded_valid,
        'first_timestamp_us': first_t, 'last_timestamp_us': last_t,
        'elapsed_s': (last_t-first_t)/1e6,
        'input_sequence_missing': timing.missing_sequences,
        'input_sequence_duplicates': timing.duplicate_sequences,
        'input_sequence_regressions': timing.sequence_regressions,
        'input_timing': timing.summary(),
        'input_timestamp_regressions': timing.timestamp_regressions,
        'valid_frames': valid, 'invalid_frames': frames-valid,
        'calibrating_frames': calibrating, 'reset_flag_frames': resets,
        'manual_recalibrations_applied': manual_resets,
        'fusion_resets_including_initial': native.lib.replay_resets(),
        'first_valid_elapsed_s': first_valid, 'last_valid_elapsed_s': last_valid,
        'validity_transition_count': transition_count, 'validity_transitions': transitions,
        'final_bias_frd_dps': [math.degrees(v) for v in last_values[7:]],
        'valid_euler_summary_degrees_not_accuracy': {k: v.summary() for k,v in moments.items()},
        'valid_quaternion_norm_squared': qnorm.summary(),
        'native_build': native.provenance,
        'interpretation': 'Fresh native software replay only; not flashed, not measured accuracy, not hardware acceptance. USB-only gaps cannot reconstruct hidden on-board fusion inputs. Relative yaw may drift; installation axes remain unverified.',
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input', type=Path)
    parser.add_argument('--report', type=Path, required=True)
    parser.add_argument('--csv', type=Path, help='Optional replay output, invalid attitudes blank')
    parser.add_argument('--fusion-source', type=Path, default=ROOT/'firmware/src/fusion.c')
    parser.add_argument('--fusion-include', type=Path, default=ROOT/'firmware/include')
    parser.add_argument('--cc', help='Native GCC/Clang executable, NOT arm-none-eabi-gcc')
    args = parser.parse_args(argv)
    outputs = [p.resolve() for p in (args.report, args.csv) if p]
    protected = {p.resolve() for p in (args.input, args.fusion_source,
                                     args.fusion_include/'fusion.h', Path(__file__),
                                     ROOT/'tools/native/fusion_replay.c',
                                     ROOT/'firmware/include/stream.h',
                                     ROOT/'tools/imu_capture.py')}
    if len(set(outputs)) != len(outputs) or protected.intersection(outputs):
        parser.error('Output paths must be distinct and must not overwrite inputs or sources')
    import contextlib
    with NativeFusion(args.fusion_source, args.fusion_include, args.cc) as native:
        with (args.csv.open('w', newline='', encoding='utf-8') if args.csv else contextlib.nullcontext()) as handle:
            writer = csv.writer(handle) if handle else None
            if writer:
                writer.writerow(['sequence', 'timestamp_us', 'elapsed_s', 'replay_flags',
                                 'attitude_valid', *ATTITUDE_FIELDS])
            result = replay(args.input, native, writer)
    args.report.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    print(json.dumps(result, indent=2, allow_nan=False))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
