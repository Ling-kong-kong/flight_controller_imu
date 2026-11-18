"""Execute the real paired C stream/codec with mocked USB and IRQ primitives.

These tests exercise protocol interoperability, historical sample selection,
loss reporting, bounded queues and CDC sessions; they do not emulate STM32 pins.
"""

import ctypes
import importlib.util
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

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
                   "-fPIC", "-shared", "-fsanitize=undefined", "-fno-sanitize-recover=undefined",
                   "-I", str(ROOT / "tests/native"), "-I", str(ROOT / "firmware/include"),
                   str(ROOT / "tests/native/stream_harness.c"),
                   str(ROOT / "firmware/src/protocol.c"), str(ROOT / "firmware/src/stream.c"),
                   "-o", str(cls.library)]
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
        self.lib.imu_encode_frame.argtypes = [ctypes.POINTER(ctypes.c_uint8), ctypes.c_uint32,
                                            ctypes.c_uint64, ctypes.POINTER(ctypes.c_int16),
                                            ctypes.POINTER(ctypes.c_int16), ctypes.c_uint32,
                                            ctypes.c_uint32, ctypes.c_uint16, ctypes.c_uint16]
        self.lib.mock_reset()
        self.now = 10000
        self.next_accel = 100000
        self.next_gyro = 100100

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
        self.assertEqual(len(wire) % 48, 0)
        self.assertEqual(parser.crc_errors, 0)
        self.assertEqual(parser.header_errors, 0)
        self.assertEqual(parser.bytes_discarded, 0)
        self.assertEqual(len(parser.buffer), 0)
        self.assertTrue(all(isinstance(frame, imu.PairedFrame) for frame in frames))
        return frames

    def test_c_encoder_matches_paired_golden_vector_and_python(self):
        expected = bytes.fromhex(
            "494d55320203300078563412f0debc9a785634120080ff7fffff"
            "7b0038fe150370020000f30100001f03410169bb9c26")
        encoded = (ctypes.c_uint8 * 48)()
        accel = (ctypes.c_int16 * 3)(-32768, 32767, -1)
        gyro = (ctypes.c_int16 * 3)(123, -456, 789)
        self.lib.imu_encode_frame(encoded, 0x12345678, 0x123456789ABCDEF0,
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
        self.assertEqual([frame.flags for frame in frames], [SESSION] + [0] * 9)
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
        self.assertEqual(frame.flags, SESSION | TIMING)

    def test_missing_or_overwritten_past_history_skips_tick_and_reports_gap(self):
        self.activate()
        self.output(100000)  # Neither source exists; no fabricated zero sample.
        self.sample(1, 100625)
        self.sample(2, 100500)
        self.output(101000)
        self.usb_pump()
        first = self.frames()[0]
        self.assertEqual(first.sequence, 1)
        self.assertEqual(first.flags, SESSION | HISTORY_MISSING)
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
        self.assertEqual(frame.flags, SESSION | ACC_STALE | GYRO_STALE)
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
        self.assertEqual(frame.flags, SESSION | ACC_SPI)
        self.assertEqual((frame.ax, frame.gx), (1, 7))
        self.sample(1, 101250, (10, 11, 12))
        self.sample(2, 101000, valid=False)
        self.sample(2, 101500, (13, 14, 15))
        self.output(102000)
        self.usb_pump()
        second = self.frames()[1]
        self.assertEqual(second.flags, GYRO_SPI)
        self.assertEqual((second.ax, second.gx), (10, 13))

    def test_cadence_and_slow_read_flags_are_source_specific(self):
        self.activate()
        self.sample(1, 100000)
        self.sample(2, 100000)
        self.sample(1, 100100)  # Too short accel interval.
        self.sample(2, 100500, slow=True)
        self.output(101000)
        self.usb_pump()
        self.assertEqual(self.frames()[0].flags, SESSION | ACC_CADENCE | GYRO_CADENCE)
        self.sample(1, 100725)
        self.sample(2, 101000)
        self.output(101500)
        self.usb_pump()
        self.assertEqual(self.frames()[1].flags, 0)

    def test_skipped_ticks_consume_sequences_and_lateness_saturates(self):
        self.activate()
        self.emit(101000)
        self.usb_pump()
        self.emit(105000, actual=105125, skipped=3)
        self.usb_pump()
        self.assertEqual([frame.sequence for frame in self.frames()], [0, 4])
        self.assertEqual(self.frames()[-1].flags, TIMING)
        self.output(106000, actual=206000)
        self.usb_pump()
        self.assertEqual(self.frames()[-1].tick_late_us, 65535)
        self.assertTrue(self.frames()[-1].flags & TIMING)

    def test_usb_backpressure_preserves_whole_frames_and_batch_limit(self):
        self.activate()
        for index in range(20):
            self.emit(101000 + index * 1000)
        self.usb_pump(available=47)
        self.assertEqual(self.wire(), b"")
        self.usb_pump(available=48)
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
        self.usb_pump(available=48)  # Free one slot.
        self.emit(103000 + capacity * 1000)
        for _ in range((capacity + 15) // 16):
            self.usb_pump()
        frames = self.frames()
        self.assertEqual(len(frames), capacity + 1)
        self.assertEqual([frame.sequence for frame in frames], list(range(capacity)) + [capacity + 2])
        self.assertEqual(frames[0].flags, SESSION)
        self.assertEqual(frames[-1].flags, OVERFLOW)
        self.assertEqual(frames[0].timestamp_us, 101000)
        self.emit(104000 + capacity * 1000)
        self.usb_pump()
        self.assertEqual(self.frames()[-1].flags, 0)

    def test_dropped_stale_frame_does_not_mark_next_fresh_pair_stale(self):
        self.activate()
        capacity = self.lib.mock_ring_capacity()
        for index in range(capacity):
            self.emit(101000 + index * 1000)
        last = 101000 + (capacity - 1) * 1000
        self.output(last + 2000)  # Full queue and both cached samples now stale.
        self.usb_pump(available=48)
        self.emit(last + 3000)  # Real fresh samples are available again.
        for _ in range((capacity + 15) // 16):
            self.usb_pump()
        frame = self.frames()[-1]
        self.assertEqual(frame.flags, OVERFLOW)
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
        self.assertEqual(frames[-1].flags, 0)

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
        self.assertEqual(frame.flags, SESSION)
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
        self.assertEqual([(frame.sequence, frame.flags) for frame in self.frames()],
                         [(0, SESSION), (0, SESSION), (0, SESSION)])

    def test_partial_cdc_write_stops_on_invariant_failure(self):
        self.activate()
        self.emit(101000)
        self.lib.mock_partial_write(True)
        self.now += 1000
        self.assertEqual(self.lib.mock_tick(self.now, 4096), 8)
        self.assertEqual(len(self.wire()), 47)
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


if __name__ == "__main__":
    unittest.main()
