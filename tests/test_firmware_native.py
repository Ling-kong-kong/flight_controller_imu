"""Execute the real paired C stream/codec with mocked USB and IRQ primitives.

These tests exercise protocol interoperability, historical sample selection,
loss reporting, bounded queues and CDC sessions; they do not emulate STM32 pins.
"""

import ctypes
import os
import importlib.util
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

if os.name == "nt":
    os.environ["PATH"] = r"D:\msys64\ucrt64\bin" + os.pathsep + os.environ["PATH"]

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("native_imu_decoder", ROOT / "tools/imu_capture.py")
imu = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = imu
SPEC.loader.exec_module(imu)

SESSION = 0x10
TIMING = 1
ACC_SPI = 2
GYRO_SPI = 4
OVERFLOW = 8
ACC_STALE = 0x20
GYRO_STALE = 0x40
ACC_CADENCE = 0x80
GYRO_CADENCE = 0x100
HISTORY_MISSING = 0x200


@unittest.skipUnless(shutil.which("cc"), "native C tests require cc (GCC/Clang)")
class FirmwareNativeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.build = tempfile.TemporaryDirectory(prefix="imu-native-")
        cls.library = Path(cls.build.name) / "stream.so"
        command = [shutil.which("cc"), "-std=c11", "-O2", "-Wall", "-Wextra", "-Werror",
                   "-fPIC", "-shared",
                   "-I", str(ROOT / "tests/native"), "-I", str(ROOT / "firmware/include"),
                   str(ROOT / "tests/native/stream_harness.c"),
                   str(ROOT / "firmware/src/protocol.c"), str(ROOT / "firmware/src/stream.c"),
                   str(ROOT / "firmware/src/fusion.c"), "-lm", "-o", str(cls.library)]
        if os.name != "nt":
            command[1:1] = ["-fsanitize=undefined", "-fno-sanitize-recover=undefined"]
        subprocess.run(command, check=True, capture_output=True, text=True)
        cls.case_index = 0

    @classmethod
    def tearDownClass(cls):
        cls.build.cleanup()

    def setUp(self):
        # Each copy has its own zeroed static source history, as at MCU reset.
        # DTR reset intentionally preserves that history, so mock_reset alone
        # must not be used to simulate a new power-on between test cases.
        type(self).case_index += 1
        copy = Path(self.build.name) / f"case-{self.case_index}.so"
        shutil.copy2(self.library, copy)
        self.lib = ctypes.CDLL(str(copy))
        self.lib.mock_tick.argtypes = [ctypes.c_uint64, ctypes.c_uint32]
        self.lib.mock_tick.restype = ctypes.c_uint
        self.lib.mock_output.restype = ctypes.POINTER(ctypes.c_uint8)
        self.lib.mock_output_size.restype = ctypes.c_uint32
        self.lib.mock_ring_capacity.restype = ctypes.c_uint32
        for name in ("mock_connected", "mock_suspended", "mock_partial_write", "mock_line_state"):
            getattr(self.lib, name).argtypes = [ctypes.c_bool]
        self.lib.mock_set_rx.argtypes = [ctypes.c_uint32]
        self.lib.stream_is_active.restype = ctypes.c_bool
        self.lib.stream_sample.argtypes = [ctypes.c_uint8, ctypes.c_uint64,
                                         ctypes.POINTER(ctypes.c_int16), ctypes.c_bool, ctypes.c_bool]
        self.lib.stream_output_tick.argtypes = [ctypes.c_uint64, ctypes.c_uint64, ctypes.c_uint32]
        self.lib.imu_encode_frame_v2.argtypes = [ctypes.POINTER(ctypes.c_uint8), ctypes.c_uint32,
                                            ctypes.c_uint64, ctypes.POINTER(ctypes.c_int16),
                                            ctypes.POINTER(ctypes.c_int16), ctypes.c_uint32,
                                            ctypes.c_uint32, ctypes.c_uint16, ctypes.c_uint16]
        self.lib.mock_process_at.argtypes = [ctypes.c_uint64]
        self.lib.mock_reset()
        self.now = 10000
        self.next_accel = 100000
        self.next_gyro = 100100

    def tearDown(self):
        if os.name == "nt":
            import _ctypes
            _ctypes.FreeLibrary(self.lib._handle)

    def activate(self):
        self.lib.mock_connected(True)
        self.usb_pump()
        self.assertTrue(self.lib.stream_is_active())

    def usb_pump(self, available=4096, increment=1000):
        self.now += increment
        self.assertEqual(self.lib.mock_tick(self.now, available), 0)

    def sample(self, sensor, timestamp, xyz=(-1234, 2345, -32768), valid=True, slow=False):
        self.lib.stream_sample(sensor, timestamp, (ctypes.c_int16 * 3)(*xyz), valid, slow)

    def output(self, target, actual=None, skipped=0):
        actual = target if actual is None else actual
        self.now = max(self.now, actual)
        self.lib.stream_output_tick(target, actual, skipped)
        self.lib.mock_process_at(actual)

    def populate_until(self, target):
        while min(self.next_accel, self.next_gyro) <= target:
            if self.next_accel <= self.next_gyro:
                index = (self.next_accel - 100000) // 625
                self.sample(1, self.next_accel, (index, 100 + index, -index))
                self.next_accel += 625
            else:
                index = (self.next_gyro - 100100) // 500
                self.sample(2, self.next_gyro, (10 + index, -200 - index, index * 2))
                self.next_gyro += 500

    def emit(self, target, actual=None, skipped=0):
        self.populate_until(target)
        self.output(target, actual, skipped)

    def wire(self):
        return ctypes.string_at(self.lib.mock_output(), self.lib.mock_output_size())

    def frames(self):
        parser = imu.StreamParser()
        frames = []
        wire = self.wire()
        for start in range(0, len(wire), 17):
            frames.extend(parser.feed(wire[start:start + 17]))
        self.assertEqual(len(wire) % 76, 0)
        self.assertEqual(parser.crc_errors, 0)
        self.assertEqual(parser.header_errors, 0)
        self.assertEqual(parser.bytes_discarded, 0)
        self.assertEqual(len(parser.buffer), 0)
        self.assertTrue(all(isinstance(frame, imu.PairedFrame) for frame in frames))
        return frames

    def publish_mag(self, sequence):
        frame = imu.MagFrame(sequence, sequence * 40000, 123, -456, 500, 12000,
                             12.5, -32.0, 35.0, 15, 16, 50, 6, 0, 4, 7,
                             0, 0, 3, 0, 1100)
        self.lib.stream_publish_mag.argtypes = [ctypes.POINTER(ctypes.c_uint8)]
        self.lib.stream_publish_mag.restype = ctypes.c_bool
        overwritten = self.lib.stream_publish_mag((ctypes.c_uint8 * 72).from_buffer_copy(frame.encode()))
        return frame, overwritten

    def test_mag_single_slot_backpressure_and_imu_priority(self):
        self.activate()
        first, overwritten = self.publish_mag(1)
        self.assertFalse(overwritten)
        self.usb_pump(available=71)
        self.assertEqual(self.wire(), b"")
        latest, overwritten = self.publish_mag(2)
        self.assertTrue(overwritten)
        self.emit(101000)
        self.usb_pump(available=76)
        self.assertEqual(len(self.wire()), 76)  # IMU first, no room for magnetic frame.
        self.assertIsInstance(imu.StreamParser().feed(self.wire())[0], imu.PairedFrame)
        self.usb_pump(available=72)
        self.assertEqual(self.wire()[76:], latest.encode())
        self.usb_pump()
        self.assertEqual(len(self.wire()), 148)  # Slot consumed, never repeated.

    def test_mag_session_discards_pending_and_disconnected_samples(self):
        self.assertFalse(self.publish_mag(1)[1])
        self.activate()
        self.usb_pump()
        self.assertEqual(self.wire(), b"")
        self.publish_mag(2)
        self.lib.mock_line_state(False)
        self.lib.mock_line_state(True)
        self.usb_pump()
        self.assertEqual(self.wire(), b"")
        current, overwritten = self.publish_mag(3)
        self.assertFalse(overwritten)
        self.usb_pump()
        self.assertEqual(self.wire(), current.encode())

    def test_c_encoder_matches_paired_golden_vector_and_python(self):
        expected = bytes.fromhex(
            "494d55320203300078563412f0debc9a785634120080ff7fffff"
            "7b0038fe150370020000f30100001f03410169bb9c26")
        encoded = (ctypes.c_uint8 * 48)()
        accel = (ctypes.c_int16 * 3)(-32768, 32767, -1)
        gyro = (ctypes.c_int16 * 3)(123, -456, 789)
        self.lib.imu_encode_frame_v2(encoded, 0x12345678, 0x123456789ABCDEF0,
                                  accel, gyro, 624, 499, 0x031F, 321)
        self.assertEqual(bytes(encoded), expected)
        frame = imu.PairedFrame(0x12345678, 0x123456789ABCDEF0,
                                -32768, 32767, -1, 123, -456, 789,
                                624, 499, 0x031F, 321)
        self.assertEqual(frame.encode(), expected)
        self.assertEqual(imu.StreamParser().feed(bytes(encoded))[0], frame)

    def test_one_thousand_hz_six_axis_frames_preserve_native_raw_values(self):
        self.activate()
        for target in range(101000, 111000, 1000):
            self.emit(target)
            self.usb_pump()
        frames = self.frames()
        self.assertEqual(len(frames), 10)
        self.assertEqual([frame.sequence for frame in frames], list(range(10)))
        self.assertEqual([frame.timestamp_us for frame in frames], list(range(101000, 111000, 1000)))
        self.assertEqual([frame.flags & 0x3ff for frame in frames], [SESSION] + [0] * 9)
        for frame in frames:
            acc_index = (frame.timestamp_us - 100000) // 625
            gyro_index = (frame.timestamp_us - 100100) // 500
            self.assertEqual((frame.ax, frame.ay, frame.az),
                             (acc_index, 100 + acc_index, -acc_index))
            self.assertEqual((frame.gx, frame.gy, frame.gz),
                             (10 + gyro_index, -200 - gyro_index, gyro_index * 2))
            self.assertEqual(frame.accel_age_us, frame.timestamp_us - (100000 + acc_index * 625))
            self.assertEqual(frame.gyro_age_us, frame.timestamp_us - (100100 + gyro_index * 500))
            self.assertEqual(frame.tick_late_us, 0)
            self.assertLess(frame.accel_age_us, 625)
            self.assertLess(frame.gyro_age_us, 500)

    def test_drdy_only_updates_history_without_extra_usb_frames(self):
        self.activate()
        self.populate_until(110000)
        self.usb_pump()
        self.assertEqual(self.wire(), b"")
        self.output(110000)
        self.usb_pump()
        self.assertEqual(len(self.frames()), 1)

    def test_late_timer_uses_latest_past_sample_and_rejects_future(self):
        self.activate()
        self.sample(1, 100625, (1, 2, 3))
        self.sample(2, 100900, (4, 5, 6))
        # EXTI may run before a delayed output ISR; these are too new for t=101000.
        self.sample(1, 101250, (11, 12, 13))
        self.sample(2, 101400, (14, 15, 16))
        self.output(101000, actual=101600)
        self.usb_pump()
        frame = self.frames()[0]
        self.assertEqual((frame.ax, frame.ay, frame.az, frame.gx, frame.gy, frame.gz),
                         (1, 2, 3, 4, 5, 6))
        self.assertEqual((frame.accel_age_us, frame.gyro_age_us), (375, 100))
        self.assertEqual(frame.tick_late_us, 600)
        self.assertEqual(frame.flags & 0x3ff, SESSION | TIMING)

    def test_missing_or_overwritten_past_history_skips_tick_and_reports_gap(self):
        self.activate()
        self.output(100000)  # Neither source exists; no fabricated zero sample.
        self.sample(1, 100625)
        self.sample(2, 100500)
        self.output(101000)
        self.usb_pump()
        first = self.frames()[0]
        self.assertEqual(first.sequence, 1)
        self.assertEqual(first.flags & 0x3ff, SESSION | HISTORY_MISSING)
        # Fill eight-history slots with values newer than t=102000.
        for index in range(9):
            self.sample(1, 103000 + 625 * index)
            self.sample(2, 103000 + 500 * index)
        self.output(102000, actual=109000)
        self.usb_pump()
        self.assertEqual(len(self.frames()), 1)
        self.output(109000)
        self.usb_pump()
        last = self.frames()[-1]
        self.assertEqual(last.sequence, 3)
        self.assertTrue(last.flags & HISTORY_MISSING)

    def test_stale_sources_retain_actual_values_with_age_flags(self):
        self.activate()
        self.sample(1, 100000, (1, 2, 3))
        self.sample(2, 100500, (4, 5, 6))
        self.output(102000)
        self.usb_pump()
        frame = self.frames()[0]
        self.assertEqual((frame.accel_age_us, frame.gyro_age_us), (2000, 1500))
        self.assertEqual(frame.flags & 0x3ff, SESSION | ACC_STALE | GYRO_STALE)
        self.assertEqual((frame.ax, frame.gx), (1, 4))

    def test_spi_failures_mark_only_affected_source_and_clear_after_enqueue(self):
        self.activate()
        self.sample(1, 100000, (1, 2, 3))
        self.sample(2, 100000, (4, 5, 6))
        self.sample(1, 100625, valid=False)
        self.sample(2, 100500, (7, 8, 9))
        self.output(101000)
        self.usb_pump()
        frame = self.frames()[0]
        self.assertEqual(frame.flags & 0x3ff, SESSION | ACC_SPI)
        self.assertEqual((frame.ax, frame.gx), (1, 7))
        self.sample(1, 101250, (10, 11, 12))
        self.sample(2, 101000, valid=False)
        self.sample(2, 101500, (13, 14, 15))
        self.output(102000)
        self.usb_pump()
        second = self.frames()[1]
        self.assertEqual(second.flags & 0x3ff, GYRO_SPI)
        self.assertEqual((second.ax, second.gx), (10, 13))

    def test_cadence_and_slow_read_flags_are_source_specific(self):
        self.activate()
        self.sample(1, 100000)
        self.sample(2, 100000)
        self.sample(1, 100100)  # Too short accel interval.
        self.sample(2, 100500, slow=True)
        self.output(101000)
        self.usb_pump()
        self.assertEqual(self.frames()[0].flags & 0x3ff, SESSION | ACC_CADENCE | GYRO_CADENCE)
        self.sample(1, 100725)
        self.sample(2, 101000)
        self.output(101500)
        self.usb_pump()
        self.assertEqual(self.frames()[1].flags & 0x3ff, 0)

    def test_skipped_ticks_consume_sequences_and_lateness_saturates(self):
        self.activate()
        self.emit(101000)
        self.usb_pump()
        self.emit(105000, actual=105125, skipped=3)
        self.usb_pump()
        self.assertEqual([frame.sequence for frame in self.frames()], [0, 4])
        self.assertEqual(self.frames()[-1].flags & 0x3ff, TIMING)
        self.output(106000, actual=206000)
        self.usb_pump()
        self.assertEqual(self.frames()[-1].tick_late_us, 65535)
        self.assertTrue(self.frames()[-1].flags & TIMING)

    def test_usb_backpressure_preserves_whole_frames_and_batch_limit(self):
        self.activate()
        for index in range(20):
            self.emit(101000 + index * 1000)
        self.usb_pump(available=75)
        self.assertEqual(self.wire(), b"")
        self.usb_pump(available=76)
        self.assertEqual([frame.sequence for frame in self.frames()], [0])
        self.usb_pump(available=4096)
        self.assertEqual(len(self.frames()), 17)
        self.usb_pump()
        self.assertEqual([frame.sequence for frame in self.frames()], list(range(20)))

    def test_ring_overflow_preserves_old_records_and_marks_next_queued_frame(self):
        self.activate()
        capacity = self.lib.mock_ring_capacity()
        for index in range(capacity):
            self.emit(101000 + index * 1000)
        self.emit(101000 + capacity * 1000)  # Queue full; sequence still consumed.
        self.emit(102000 + capacity * 1000)
        self.usb_pump(available=76)  # Free one slot.
        self.emit(103000 + capacity * 1000)
        for _ in range((capacity + 15) // 16):
            self.usb_pump()
        frames = self.frames()
        self.assertEqual(len(frames), capacity + 1)
        self.assertEqual([frame.sequence for frame in frames], list(range(capacity)) + [capacity + 2])
        self.assertEqual(frames[0].flags & 0x3ff, SESSION)
        self.assertEqual(frames[-1].flags & 0x3ff, OVERFLOW)
        self.assertEqual(frames[0].timestamp_us, 101000)
        self.emit(104000 + capacity * 1000)
        self.usb_pump()
        self.assertEqual(self.frames()[-1].flags & 0x3ff, 0)

    def test_dropped_stale_frame_does_not_mark_next_fresh_pair_stale(self):
        self.activate()
        capacity = self.lib.mock_ring_capacity()
        for index in range(capacity):
            self.emit(101000 + index * 1000)
        last = 101000 + (capacity - 1) * 1000
        self.output(last + 2000)  # Full queue and both cached samples now stale.
        self.usb_pump(available=76)
        self.emit(last + 3000)  # Real fresh samples are available again.
        for _ in range((capacity + 15) // 16):
            self.usb_pump()
        frame = self.frames()[-1]
        self.assertEqual(frame.flags & 0x3ff, OVERFLOW)
        self.assertLess(frame.accel_age_us, 625)
        self.assertLess(frame.gyro_age_us, 500)

    def test_source_ages_and_common_time_remain_correct_across_32_bit_timer_wrap(self):
        self.activate()
        before_wrap = 0xFFFFFF00
        self.sample(1, before_wrap - 375, (1, 2, 3))
        self.sample(2, before_wrap - 250, (4, 5, 6))
        self.output(before_wrap)
        self.usb_pump()
        self.sample(1, before_wrap + 250, (7, 8, 9))
        self.sample(1, before_wrap + 875, (10, 11, 12))
        self.sample(2, before_wrap + 250, (13, 14, 15))
        self.sample(2, before_wrap + 750, (16, 17, 18))
        self.output(before_wrap + 1000)
        self.usb_pump()
        frames = self.frames()
        self.assertEqual([frame.timestamp_us for frame in frames], [before_wrap, before_wrap + 1000])
        self.assertEqual([(frame.accel_age_us, frame.gyro_age_us) for frame in frames],
                         [(375, 250), (125, 250)])
        self.assertEqual((frames[-1].ax, frames[-1].gx), (10, 16))
        self.assertEqual(frames[-1].flags & 0x3ff, 0)

    def test_usb_flush_interval_is_one_millisecond(self):
        self.activate()
        self.sample(1, 10000)
        self.sample(2, 10000)
        self.output(11000)
        self.usb_pump(increment=999)
        self.assertEqual(self.wire(), b"")
        self.usb_pump(increment=1)
        self.assertEqual(len(self.frames()), 1)

    def test_close_reopen_resets_sequence_without_clearing_source_history(self):
        self.activate()
        self.emit(101000)
        self.lib.mock_line_state(False)  # Old queued frame is discarded.
        self.assertFalse(self.lib.stream_is_active())
        self.populate_until(102000)
        self.lib.mock_line_state(True)  # No stream task between the two events.
        self.output(102000)
        self.usb_pump()
        frame = self.frames()[0]
        self.assertEqual(frame.sequence, 0)
        self.assertEqual(frame.flags & 0x3ff, SESSION)
        self.assertEqual(frame.timestamp_us, 102000)
        self.assertEqual(self.lib.mock_get_clears(), 3)

    def test_suspend_unmount_and_inactive_ticks_do_not_emit_stale_queue(self):
        self.populate_until(100000)
        self.output(100000)
        self.usb_pump()
        self.assertEqual(self.wire(), b"")
        self.activate()
        self.emit(101000)
        self.usb_pump()
        self.lib.mock_suspend_event()
        self.assertFalse(self.lib.stream_is_active())
        self.emit(102000)
        self.lib.mock_suspended(False)
        self.usb_pump()
        self.emit(103000)
        self.usb_pump()
        self.lib.mock_unmount_event()
        self.assertFalse(self.lib.stream_is_active())
        self.lib.mock_connected(True)
        self.usb_pump()
        self.emit(104000)
        self.usb_pump()
        self.assertEqual([(frame.sequence, frame.flags & 0x3ff) for frame in self.frames()],
                         [(0, SESSION), (0, SESSION), (0, SESSION)])

    def test_partial_cdc_write_stops_on_invariant_failure(self):
        self.activate()
        self.emit(101000)
        self.lib.mock_partial_write(True)
        self.now += 1000
        self.assertEqual(self.lib.mock_tick(self.now, 4096), 8)
        self.assertEqual(len(self.wire()), 75)
        # Real firmware remains in board_fatal and never retries a partial frame.

    def test_rx_bytes_ignored_and_irq_mask_preserved(self):
        primask = ctypes.c_uint32.in_dll(self.lib, "mock_primask")
        primask.value = 1
        self.activate()
        self.assertEqual(primask.value, 1)
        self.lib.mock_set_rx(100)
        self.emit(101000)
        self.usb_pump()
        self.assertEqual(self.lib.mock_get_rx(), 36)
        self.usb_pump()
        self.assertEqual(self.lib.mock_get_rx(), 0)
        self.assertEqual(len(self.frames()), 1)
        self.assertEqual(primask.value, 1)


@unittest.skipUnless(shutil.which("cc"), "native C tests require cc")
class AttitudeNativeTests(unittest.TestCase):
    # Share the build/harness, not the legacy test methods.
    setUpClass = classmethod(FirmwareNativeTests.setUpClass.__func__)
    tearDownClass = classmethod(FirmwareNativeTests.tearDownClass.__func__)
    tearDown = FirmwareNativeTests.tearDown
    activate = FirmwareNativeTests.activate
    usb_pump = FirmwareNativeTests.usb_pump
    sample = FirmwareNativeTests.sample
    output = FirmwareNativeTests.output
    wire = FirmwareNativeTests.wire
    frames = FirmwareNativeTests.frames

    def setUp(self):
        FirmwareNativeTests.setUp(self)
        self.lib.mock_fusion_update.argtypes = [ctypes.c_uint64,
            ctypes.POINTER(ctypes.c_int16), ctypes.POINTER(ctypes.c_int16),
            ctypes.c_uint32, ctypes.c_uint32, ctypes.c_uint16]
        self.lib.mock_attitude.argtypes = [ctypes.c_uint]
        self.lib.mock_attitude.restype = ctypes.c_float
        for name in ("mock_stationary_gyro", "mock_bias"):
            getattr(self.lib, name).argtypes = [ctypes.c_uint]
            getattr(self.lib, name).restype = ctypes.c_float
        self.lib.mock_fusion_reset()
        self.t = 100000

    def update(self, a=(0, 0, -1), g=(0, 0, 0), aa=0, ga=0, flags=0, step=1000):
        # Physical FRD input -> native registers (inverse provisional pitch180).
        av = (ctypes.c_int16*3)(*[round(v*32768/24*s) for v,s in zip(a,(-1,1,-1))])
        gv = (ctypes.c_int16*3)(*[round(v*32768/2000*s) for v,s in zip(g,(-1,1,-1))])
        self.t += step
        self.lib.mock_fusion_update(self.t, av, gv, aa, ga, flags)
        return self.lib.mock_attitude_flags()

    def calibrate(self, a=(0,0,-1), g=(0,0,0)):
        for _ in range(2250):
            flags = self.update(a,g)
        self.assertEqual(flags, 2048)

    def attitude(self):
        return [self.lib.mock_attitude(i) for i in range(7)]

    def test_static_bias_calibration_and_motion_restarts_timer(self):
        for _ in range(1800):
            self.assertFalse(self.update(g=(1,-2,3)) & 2048)
        for _ in range(100):
            self.update(g=(0,0,10))
        for _ in range(2000):
            self.assertFalse(self.update(g=(1,-2,3)) & 2048)
        for _ in range(250):
            flags = self.update(g=(1,-2,3))
        self.assertEqual(flags,2048)
        for _ in range(2000):
            self.update(g=(1,-2,3))
        self.assertAlmostEqual(self.attitude()[6],0,places=4)

    def test_high_frequency_gyro_noise_calibrates_without_relaxing_thresholds(self):
        # 500 Hz noise has raw std ~1 deg/s, above the unchanged 0.5 gate.
        # Only the detector is filtered; a quiet board must finish at 2.199 s.
        first = None
        for i in range(2600):
            flags = self.update(g=(.3 + (1 if i % 2 else -1), -.2, .1))
            if flags & 2048 and first is None:
                first = i
        self.assertEqual(first, 2199)

    def test_detector_causal_response_seeds_first_sample_without_zero_transient(self):
        import math
        # Check the specified 30 Hz single-pole response, not an acausal smoother.
        alpha = 1 - math.exp(-2 * math.pi * 30 / 1000)
        quantized = round(2 * 32768 / 2000) * 2000 / 32768
        self.update(g=(2, -2, 2))
        expected = [quantized, -quantized, quantized]
        for axis, value in enumerate(expected):
            self.assertAlmostEqual(self.lib.mock_stationary_gyro(axis), value, places=6)
        for i in range(1, 100):
            self.update()
            for axis in range(3):
                expected[axis] *= 1 - alpha
                self.assertAlmostEqual(self.lib.mock_stationary_gyro(axis), expected[axis], delta=2e-6)
        self.assertEqual(self.lib.mock_stationary_count(), 100)

    def test_bias_is_mean_of_raw_accepted_samples_not_filtered_detector(self):
        import math
        accepted = []
        first = None
        for i in range(2200):
            g = .3 + 1.2 * math.sin(2 * math.pi * 87 * i / 1000)
            flags = self.update(g=(g, -.2, .1))
            if i >= 199:
                accepted.append(round(g * 32768 / 2000) * 2000 / 32768)
            if flags & 2048 and first is None:
                first = i
        self.assertEqual(first, 2199)
        self.assertAlmostEqual(self.lib.mock_bias(0), math.radians(sum(accepted)/len(accepted)), delta=2e-9)
        self.assertAlmostEqual(self.lib.mock_bias(1), math.radians(round(-.2*32768/2000)*2000/32768), delta=2e-9)

    def test_raw_gyro_integration_has_no_detector_step_delay_or_tail(self):
        import math
        self.calibrate()
        quantized = round(90 * 32768 / 2000) * 2000 / 32768
        expected = math.degrees(2 * math.atan(math.radians(quantized) * .0005))
        self.update(g=(0,0,90))
        self.assertAlmostEqual(self.attitude()[6], expected, delta=1e-7)
        for _ in range(20):
            self.update()
        self.assertAlmostEqual(self.attitude()[6], expected, delta=1e-6)

    def test_low_frequency_gyro_motion_still_blocks_stationary_calibration(self):
        import math
        for i in range(4500):
            # All raw norms <5, but 5 Hz motion survives the detector LPF.
            self.assertFalse(self.update(g=(0,0,2*math.sin(2*math.pi*5*i/1000))) & 2048)
        self.calibrate()

    def test_raw_norm_spike_restarts_continuous_two_seconds_despite_filter(self):
        for _ in range(2100):
            self.assertFalse(self.update() & 2048)
        self.assertFalse(self.update(g=(0,0,6)) & 2048)
        self.assertLess(self.lib.mock_stationary_gyro(2), 5)
        for _ in range(1999):
            self.assertFalse(self.update() & 2048)
        for _ in range(250):
            flags = self.update()
        self.assertTrue(flags & 2048)

    def test_detector_and_calibration_history_reset_on_discontinuities(self):
        # Each failure must discard history, reseed and require a fresh window.
        cases = (dict(ga=1001), dict(ga=1000), dict(ga=99999999),
                 dict(flags=GYRO_SPI), dict(aa=1001), dict(aa=99999999),
                 dict(flags=ACC_SPI), dict(step=5000), dict(step=11000),
                 dict(step=-1000), "explicit_reset")
        for case in cases:
            with self.subTest(case=case):
                self.lib.mock_fusion_reset()
                for _ in range(2100):
                    self.assertFalse(self.update(g=(0,0,3)) & 2048)
                gap = isinstance(case, dict) and "step" in case
                if case == "explicit_reset":
                    self.lib.mock_fusion_reset()
                else:
                    self.assertFalse(self.update(g=(0,0,1), **case) & 2048)
                self.assertEqual(self.lib.mock_stationary_count(), 1 if gap else 0)
                # The gap's current good pair is the first fresh window sample.
                expected_first = 2198 if gap else 2199
                first = None
                for i in range(2250):
                    flags = self.update(g=(0,0,1))
                    if flags & 2048 and first is None:
                        first = i
                self.assertEqual(first, expected_first)
                quantized = round(32768/2000)*2000/32768
                self.assertAlmostEqual(self.lib.mock_stationary_gyro(2), quantized, places=5)

    def test_constant_slow_yaw_is_still_ambiguous_with_bias(self):
        # Explicit known limitation: LPF does not add an absolute yaw observation.
        self.calibrate(g=(0,0,1))
        for _ in range(1000):
            self.update()
        self.assertAlmostEqual(self.attitude()[6], -round(32768/2000)*2000/32768, delta=.002)

    def test_short_window_variance_rejects_false_stationary(self):
        # Norms pass; gyro and acceleration per-axis variances do not.
        for i in range(4000):
            self.assertFalse(self.update(a=((.06 if i%2 else -.06),0,-.998),
                                        g=((1 if i%2 else -1),0,0)) & 2048)
        self.calibrate()

    def test_known_initial_roll_pitch_and_zero_heading(self):
        import math
        r,p = map(math.radians,(30,-20))
        a=(math.sin(p),-math.sin(r)*math.cos(p),-math.cos(r)*math.cos(p))
        self.calibrate(a)
        q = self.attitude()
        self.assertAlmostEqual(q[4],30,delta=.05)
        self.assertAlmostEqual(q[5],-20,delta=.05)
        self.assertAlmostEqual(q[6],0,delta=.001)
        self.assertAlmostEqual(sum(x*x for x in q[:4]),1,places=6)

    def test_positive_frd_rotations_and_relative_yaw_drift(self):
        # Out-of-range acceleration explicitly disables gravity feedback.
        for axis in range(3):
            self.lib.mock_fusion_reset()
            self.calibrate()
            g=tuple(90 if i==axis else 0 for i in range(3))
            for _ in range(500):
                self.assertTrue(self.update(a=(0,0,-2),g=g) & 2048)
            self.assertAlmostEqual(self.attitude()[4+axis],45,delta=.05)
        self.lib.mock_fusion_reset()
        self.calibrate()
        for _ in range(2000):
            self.update(g=(0,0,1))
        # No absolute heading observation: residual z bias is allowed to drift.
        self.assertAlmostEqual(self.attitude()[6],2,delta=.06)

    def test_gravity_feedback_converges_and_variance_gates_correction(self):
        self.calibrate()
        for _ in range(500):
            self.update(a=(0,0,-2),g=(60,0,0))
        self.assertAlmostEqual(self.attitude()[4],30,delta=.05)
        for i in range(1000):
            self.update(a=((.15 if i%2 else -.15),0,-.989))
        self.assertAlmostEqual(self.attitude()[4],30,delta=.1)
        for _ in range(3000):
            self.update()
        self.assertLess(abs(self.attitude()[4]),.2)

    def test_stale_error_repeated_gyro_not_integrated_and_interrupt_resets(self):
        self.calibrate()
        for kw in (dict(ga=1001),dict(flags=GYRO_SPI),dict(ga=1000)):
            before=self.attitude()
            self.assertFalse(self.update(g=(0,0,90),**kw) & 2048)
            self.assertEqual(self.attitude(),before)
            self.update()
        # Fresh gyro with stale accel is still a valid gyro-only update.
        self.assertTrue(self.update(g=(0,0,90),aa=1001) & 2048)
        self.assertAlmostEqual(self.attitude()[6],.09,delta=.001)
        before=self.attitude()[6]
        self.assertTrue(self.update(g=(0,0,90),aa=1001,step=5000) & 2048)
        self.assertAlmostEqual(self.attitude()[6]-before,.09,delta=.001)
        flags=self.update(step=11000)
        self.assertTrue(flags & 4096)
        self.assertTrue(flags & 1024)
        self.assertFalse(flags & 2048)
        self.calibrate()
        for _ in range(11):
            flags=self.update(ga=2000)
        self.assertTrue(flags & 1024)
        self.assertFalse(flags & 2048)

    def test_accel_error_prevents_calibration_and_timestamp_regression_resets(self):
        for _ in range(2500):
            self.assertFalse(self.update(flags=ACC_SPI) & 2048)
        self.calibrate()
        self.assertTrue(self.update(step=-1000) & 4096)

    def stream_static(self, t, g=(0,0,0), process=True):
        self.sample(1,t,(0,0,1365))
        self.sample(2,t,g)
        self.lib.stream_output_tick(t,t,0)
        if process:
            self.lib.mock_process_at(t)
        self.now=t

    def test_isr_queue_overflow_invalidates_backlog_and_keeps_sequence_gap(self):
        self.activate()
        self.stream_static(100000)
        self.usb_pump()
        self.assertEqual(len(self.frames()),1)
        for i in range(65):
            self.stream_static(101000+i*1000,process=False)
        self.lib.mock_process_at(self.now)
        self.assertEqual(self.lib.mock_fusion_drops(),65)
        self.stream_static(166000)
        self.usb_pump()
        frames=self.frames()
        self.assertEqual([f.sequence for f in frames],[0,66])
        self.assertTrue(frames[-1].flags & (4096|1024))
        self.assertFalse(frames[-1].flags & 2048)

    def test_queue_loss_after_valid_state_requires_fresh_window_and_calibration(self):
        self.activate()
        t = 100000
        for _ in range(2250):
            self.stream_static(t)
            self.usb_pump()
            t += 1000
        self.assertTrue(self.frames()[-1].attitude_valid)
        resets = self.lib.mock_fusion_resets()
        for _ in range(65):
            self.stream_static(t, g=(0,0,-65), process=False)
            t += 1000
        self.lib.mock_process_at(t-1000)
        self.assertGreater(self.lib.mock_fusion_resets(), resets)
        first = None
        for i in range(2250):
            self.stream_static(t)
            self.usb_pump()
            # Decode only the latest wire frame (the full capture is tested below).
            f = imu.StreamParser().feed(self.wire()[-76:])[0]
            if f.attitude_valid and first is None:
                first = i
            t += 1000
        self.assertEqual(first, 2199)
        self.assertGreater(self.lib.mock_fusion_drops(), 0)
        self.assertTrue(self.frames()[-1].attitude_valid)

    def test_detector_filter_never_changes_raw_usb_register_values(self):
        self.activate()
        for i in range(2300):
            g = (16 if i % 2 else -16, -5, 2)
            self.stream_static(100000+i*1000, g=g)
            self.usb_pump()
        frames = self.frames()
        for i, f in enumerate(frames):
            self.assertEqual((f.ax,f.ay,f.az), (0,0,1365))
            self.assertEqual((f.gx,f.gy,f.gz), (16 if i%2 else -16,-5,2))
        self.assertTrue(frames[-1].attitude_valid)

    def test_fusion_runs_disconnected_and_reconnect_does_not_reset_heading(self):
        for i in range(2250):
            self.stream_static(100000+i*1000)
        self.assertEqual(self.wire(),b'')
        self.activate()
        for i in range(1000):
            self.stream_static(2350000+i*1000,(0,0,-1475)) # +90 deg/s FRD z
            self.usb_pump()
        yaw=self.frames()[-1].yaw_deg
        self.assertAlmostEqual(yaw,90,delta=.1)
        resets=self.lib.mock_fusion_resets()
        self.lib.mock_line_state(False)
        for i in range(10):
            self.stream_static(3350000+i*1000)
        self.lib.mock_line_state(True)
        self.stream_static(3360000)
        self.usb_pump()
        f=self.frames()[-1]
        self.assertTrue(f.flags & SESSION)
        self.assertEqual(f.sequence,0)
        self.assertTrue(f.attitude_valid)
        self.assertAlmostEqual(f.yaw_deg,yaw,delta=.01)
        self.assertEqual(self.lib.mock_fusion_resets(),resets)

    def test_usb_ring_loss_does_not_reset_fusion_and_idle_loss_does(self):
        self.activate()
        for i in range(2250):
            self.stream_static(100000+i*1000)
        self.assertGreater(self.lib.mock_ring_drops(),0)
        resets=self.lib.mock_fusion_resets()
        for _ in range(32):
            self.usb_pump()
        self.stream_static(2350000)
        self.usb_pump()
        self.assertEqual(self.lib.mock_fusion_resets(),resets)
        self.assertTrue(self.frames()[-1].attitude_valid)
        self.lib.mock_process_at(2370000)
        self.stream_static(2371000)
        self.usb_pump()
        self.assertFalse(self.frames()[-1].attitude_valid)
        self.assertTrue(self.frames()[-1].flags & 4096)

    def test_v3_crc_reference_vector(self):
        import struct,zlib
        buf=(ctypes.c_uint8*76)()
        self.lib.mock_encode_v3(buf)
        wire=bytes(buf)
        self.assertEqual(wire[:8],bytes.fromhex('494d553203034c00'))
        self.assertEqual(struct.unpack('<I',wire[-4:])[0],zlib.crc32(wire[:72]))
        f=imu.StreamParser().feed(wire)[0]
        self.assertEqual(f.protocol_version,3)
        self.assertTrue(f.attitude_valid)
        self.assertEqual(f.yaw_deg,90)
        self.assertEqual(f.encode(),wire)
        self.assertEqual(wire.hex(),
            '494d553203034c0078563412f0debc9a785634120080ff7fffff7b0038fe150370020000f301000000084101'
            '0000803f00000000000000000000000000000000000000000000b442187ba85c')


if __name__ == "__main__":
    unittest.main()
