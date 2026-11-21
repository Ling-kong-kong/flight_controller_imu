"""Magnetic ON integration tests of production fusion/queues, not hardware acceptance."""
import ctypes
from dataclasses import replace
import json
import math
from pathlib import Path
import tempfile
import unittest

from tools import imu_fusion_replay as replay
from tools.imu_capture import MagFrame, PairedFrame
import test_mag_heading as gate
import test_firmware_native as stream
import zlib
from tools.imu_capture import CONTROL

ROOT = replay.ROOT


def control_command(request_id, operation=1):
    raw = CONTROL.pack(b"IMU2", 1, 0x10, 32, request_id, operation, 0, 0, 0)
    return raw[:-4] + zlib.crc32(raw[:-4]).to_bytes(4, "little")


class SyntheticField:
    candidate = json.loads((ROOT/'config/bmm150-calibration-candidate.json').read_text())
    chip = gate.MagneticHeadingTests.chip

    @staticmethod
    def frame(i, **kwargs):
        return replace(PairedFrame(i, 100000+i*1000, 0, 0, 1365, 0, 0, 0,
                                  0, 0, protocol_version=3), **kwargs)

    def mag_frame(self, i, heading=0, sequence=None, **kwargs):
        return MagFrame(i//40 if sequence is None else sequence, self.frame(i).timestamp_us,
                        123, -456, 500, 12000, *self.chip(heading=heading, **kwargs),
                        15, 16, 50, 6, 0, 4, 7, 0, 0, 3, 0, 1100)


class MagneticFusionIntegrationTests(SyntheticField, unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.on = replay.NativeFusion(ROOT/'firmware/src/fusion.c', ROOT/'firmware/include', magnetic=True)
        cls.off = replay.NativeFusion(ROOT/'firmware/src/fusion.c', ROOT/'firmware/include', magnetic=False)

    @classmethod
    def tearDownClass(cls):
        cls.on.close()
        cls.off.close()

    def setUp(self):
        self.on.lib.replay_reset()
        self.off.lib.replay_reset()

    def run_ticks(self, start=0, stop=3400, gz=0, heading=None, magnetic=True, **kwargs):
        for i in range(start, stop):
            if magnetic and i % 40 == 0:
                m = self.mag_frame(i, heading=0 if heading is None else heading(i))
                self.on.submit_mag(m)
            f = self.frame(i, gz=gz, **kwargs)
            flags, values = self.on.update(f)
            off_flags, off_values = self.off.update(f)
        return flags, values, off_flags, off_values

    def ready(self):
        flags, *_ = self.run_ticks()
        self.assertEqual(flags & 0xa800, 0xa800)
        self.assertEqual(self.on.magnetic_diagnostics()['references'], 1)

    def test_initial_valid_waits_for_magnetic_reference(self):
        flags, *_ = self.run_ticks(stop=2250)
        self.assertTrue(flags & 1024)
        self.assertFalse(flags & 2048)
        flags, values, *_ = self.run_ticks(2250, 3400)
        self.assertEqual(flags & 0xa800, 0xa800)
        self.assertAlmostEqual(values[6], 0., delta=.001)

    def test_no_magnetic_data_has_bounded_six_axis_fallback(self):
        flags, *_ = self.run_ticks(stop=7100, magnetic=False)
        self.assertFalse(flags & 2048)
        flags, *_ = self.run_ticks(7100, 7300, magnetic=False)
        self.assertTrue(flags & 2048)
        self.assertFalse(flags & 32768)
        # A late reference must not erase genuine rotation after fallback.
        self.run_ticks(7300, 8300, gz=-328, magnetic=False)
        flags, values, *_ = self.run_ticks(8300, 9600, heading=lambda i: 20.)
        self.assertTrue(flags & 32768)
        self.assertGreater(values[6], 19.)

    def test_heading_zero_preserves_bias_reference_and_new_zero(self):
        self.ready()
        self.run_ticks(3400, 4400, gz=-328, heading=lambda i: 20*(i-3400)/1000)
        before = self.on.magnetic_diagnostics()
        bias = self.run_ticks(4400, 4500, heading=lambda i: 20.)[1][7:]
        self.assertEqual(self.on.lib.replay_heading_zero(), 0)
        flags, values, *_ = self.run_ticks(4500, 14500, heading=lambda i: 20.)
        self.assertAlmostEqual(values[6], 0., delta=.1)
        self.assertEqual(values[7:], bias)
        self.assertEqual(self.on.magnetic_diagnostics()['references'], before['references'])
        self.assertEqual(flags & 0xa800, 0xa800)

    def test_magnetic_wait_restarts_on_motion_bad_gyro_or_gap(self):
        for changes in ({"gz": 200}, {"gyro_age_us": 1001}, {"i": 2252}):
            with self.subTest(changes=changes):
                self.setUp()
                self.run_ticks(stop=2250)
                self.assertEqual(self.on.lib.replay_calibration_phase(), 1)
                values = dict(changes)
                i = values.pop("i", 2250)
                flags, _ = self.on.update(self.frame(i, **values))
                self.assertFalse(flags & 2048)
                self.assertEqual(self.on.lib.replay_calibration_phase(), 0)

    def test_sixty_second_residual_bias_drift_on_vs_off(self):
        self.ready()
        flags, values, off_flags, off_values = self.run_ticks(3400, 63400, gz=-8)
        self.assertTrue(flags & 8192)
        self.assertLess(abs(values[6]), 4.)
        self.assertGreater(off_values[6], 25.)
        self.assertEqual(off_flags & 0xe000, 0)
        self.assertAlmostEqual(sum(v*v for v in values[:4]), 1., delta=1e-6)
        self.assertEqual(values[7:], off_values[7:])  # magnetic aid never changes gyro bias

    def test_real_positive_and_negative_turns_are_not_damped(self):
        for sign in (1, -1):
            with self.subTest(sign=sign):
                self.setUp(); self.ready()
                rate = sign*655*2000/32768
                flags, values, _, off_values = self.run_ticks(3400, 6400, gz=-sign*655,
                    heading=lambda i: rate*(i-3400+1)/1000)
                self.assertTrue(flags & 8192)
                self.assertAlmostEqual(values[6], rate*3, delta=.3)
                self.assertAlmostEqual(values[6], off_values[6], delta=.3)
                self.assertAlmostEqual(values[4], 0., delta=.001)
                self.assertAlmostEqual(values[5], 0., delta=.001)

    def test_slow_tilt_through_vertical_then_world_z_turn(self):
        self.ready()
        scale = 32768/24
        gy = 82  # raw gyro quantization: 5.00488 deg/s, gravity window stays quiet
        rate = gy*2000/32768
        for i in range(3400, 23400):
            pitch = rate*(i-3400+1)/1000
            p = math.radians(pitch)
            f = self.frame(i, ax=round(-math.sin(p)*scale), az=round(math.cos(p)*scale), gy=gy)
            if i % 40 == 0:
                self.on.submit_mag(self.mag_frame(i, pitch=pitch))
            flags, values = self.on.update(f)
            self.off.update(f)
            if 80 <= pitch <= 100:
                self.assertEqual(flags & 0xa800, 0xa800)  # no Euler pitch gate
        # Stop at >90 degrees, then rotate about WORLD down, not body Z.
        for i in range(23400, 24400):
            if i % 40 == 0: self.on.submit_mag(self.mag_frame(i, pitch=pitch))
            f = self.frame(i, ax=round(-math.sin(p)*scale), az=round(math.cos(p)*scale))
            self.on.update(f); self.off.update(f)
        for i in range(24400, 26400):
            yaw = 10*(i-24400+1)/1000
            f = self.frame(i, ax=round(-math.sin(p)*scale), az=round(math.cos(p)*scale),
                           gx=round(math.sin(p)*10*32768/2000),
                           gz=round(-math.cos(p)*10*32768/2000))
            if i % 40 == 0: self.on.submit_mag(self.mag_frame(i, heading=yaw, pitch=pitch))
            flags, values = self.on.update(f)
            _, off_values = self.off.update(f)
            self.assertEqual(flags & 0xa800, 0xa800)
        expected = gate.MagneticHeadingTests.quaternion(0, pitch, yaw)
        for result in (values, off_values):
            dot = min(1, abs(sum(a*b for a, b in zip(result[:4], expected))))
            self.assertLess(math.degrees(2*math.acos(dot)), .2)
        self.assertEqual(self.on.magnetic_diagnostics()['references'], 1)

    def tilt_ticks(self, start, stop, gy=492, gyro_error=0):
        # ~30 deg/s: unit gravity, but the 200 ms per-axis accel std exceeds
        # 0.02 g. This is valid tilting, not stationary acceleration history.
        scale = 32768/24
        rate = gy*2000/32768
        for i in range(start, stop):
            pitch = rate*(i-start+1)/1000
            p = math.radians(pitch)
            if i % 40 == 0:
                self.on.submit_mag(self.mag_frame(i, pitch=pitch))
            f = self.frame(i, ax=round(-math.sin(p)*scale),
                           az=round(math.cos(p)*scale), gy=gy+gyro_error)
            flags, values = self.on.update(f)
            off_flags, off_values = self.off.update(f)
            yield i, pitch, flags, values, off_flags, off_values

    def test_runtime_tilt_keeps_magnetic_aiding_without_stationary_accel(self):
        for sign in (1, -1):
            with self.subTest(sign=sign):
                self.setUp(); self.ready()
                before = self.on.magnetic_diagnostics()
                for i, pitch, flags, values, off_flags, off_values in self.tilt_ticks(
                        3400, 7400, gy=sign*492):
                    self.assertEqual(flags & 0xa800, 0xa800, f'tick {i}, pitch {pitch}')
                    self.assertEqual(off_flags & 0xe000, 0)
                # Compare quaternions, not singular Euler yaw near vertical.
                expected = gate.MagneticHeadingTests.quaternion(0, pitch, 0)
                for result in (values, off_values):
                    dot = min(1, abs(sum(a*b for a, b in zip(result[:4], expected))))
                    self.assertLess(math.degrees(2*math.acos(dot)), .2)
                self.assertEqual(values[7:], off_values[7:])
                after = self.on.magnetic_diagnostics()
                self.assertEqual(after['references'], before['references'])
                self.assertEqual(after['reference_offset_deg'], before['reference_offset_deg'])

    def test_six_axis_accel_correction_stays_active_during_runtime_tilt(self):
        for sign in (1, -1):
            with self.subTest(sign=sign):
                self.setUp(); self.ready()
                # Add ~1 deg/s gyro error AFTER bias calibration. Gravity must
                # correct it even while the per-axis accel history is moving.
                for _, pitch, flags, values, off_flags, off_values in self.tilt_ticks(
                        3400, 7400, gy=sign*492, gyro_error=sign*16):
                    self.assertEqual(flags & 0xa800, 0xa800)
                    self.assertTrue(off_flags & 2048)
                expected = gate.MagneticHeadingTests.quaternion(0, pitch, 0)
                for result in (values, off_values):
                    dot = min(1, abs(sum(a*b for a, b in zip(result[:4], expected))))
                    self.assertLess(math.degrees(2*math.acos(dot)), .6)
                self.assertEqual(values[7:], off_values[7:])
                self.assertEqual(values[7:], [0., 0., 0.])

    def test_six_axis_accel_correction_still_rejects_invalid_accel(self):
        scale = 32768/24
        # A false 30-degree tilt must not pull the quaternion when data is
        # stale, flagged bad, or outside the retained 0.9--1.1 g norm gate.
        cases = ((.85, {}), (1.15, {}),
                 (1., {'accel_age_us': 1001}), (1., {'flags': 2}))
        for magnitude, quality in cases:
            with self.subTest(magnitude=magnitude, quality=quality):
                self.setUp(); self.ready()
                _, before, _, off_before = self.run_ticks(3400, 3401)
                _, values, off_flags, off_values = self.run_ticks(
                    3401, 4401, ax=round(-.5*magnitude*scale),
                    az=round(math.cos(math.radians(30))*magnitude*scale),
                    **quality)
                self.assertTrue(off_flags & 2048)
                for initial, final in ((before, values), (off_before, off_values)):
                    for actual, expected in zip(final[:4], initial[:4]):
                        self.assertAlmostEqual(actual, expected, delta=1e-6)

    def test_rejected_aiding_can_recover_while_tilting(self):
        self.ready()
        before = self.on.magnetic_diagnostics()
        flags, *_ = self.run_ticks(3400, 3401, accel_age_us=1001)
        self.assertEqual(flags & 0xe000, 0xc000)
        for i, _, flags, *_ in self.tilt_ticks(3401, 5401):
            if i < 4200:  # Still require the full 800 ms recovery window.
                self.assertFalse(flags & 8192)
            if i >= 4401:
                self.assertEqual(flags & 0xa800, 0xa800)
        after = self.on.magnetic_diagnostics()
        self.assertEqual(after['references'], before['references'])
        self.assertEqual(after['reference_offset_deg'], before['reference_offset_deg'])

    def test_runtime_motion_gate_still_rejects_bad_accel_and_excess_rate(self):
        cases = ({'az': round(.85*32768/24)},
                 {'az': round(1.15*32768/24)},
                 {'accel_age_us': 1001}, {'flags': 2},
                 {'gy': 1475})  # >90 deg/s after reference; no threshold change.
        for kwargs in cases:
            with self.subTest(kwargs=kwargs):
                self.setUp(); self.ready()
                before = self.on.magnetic_diagnostics()
                flags, *_ = self.run_ticks(3400, 3401, **kwargs)
                self.assertTrue(flags & 2048)  # Six-axis attitude stays valid.
                self.assertEqual(flags & 0xe000, 0xc000)
                after = self.on.magnetic_diagnostics()
                self.assertEqual(after['reason'], 'motion')
                self.assertEqual(after['references'], before['references'])
                self.assertEqual(after['reference_offset_deg'], before['reference_offset_deg'])

    def test_startup_and_magnetic_wait_still_require_stationary_board(self):
        for _, _, flags, *_ in self.tilt_ticks(0, 2600):
            self.assertTrue(flags & 1024)
            self.assertFalse(flags & (2048 | 8192 | 32768))
        self.setUp()
        flags, *_ = self.run_ticks(stop=2250)
        self.assertTrue(flags & 1024)  # Six-axis init done, magnetic wait active.
        self.assertFalse(flags & (2048 | 32768))
        resets = self.on.lib.replay_resets()
        self.run_ticks(2250, 2251, gy=492)
        self.assertGreater(self.on.lib.replay_resets(), resets)
        flags, *_ = self.run_ticks(2251, 4251)
        self.assertTrue(flags & 1024)  # Must repeat the full stationary startup.
        self.assertFalse(flags & (2048 | 8192 | 32768))

    def test_bad_gyro_stale_magnetic_and_recovery_preserve_reference(self):
        self.ready()
        offset = self.on.magnetic_diagnostics()['reference_offset_deg']
        flags, *_ = self.run_ticks(3400, 3401, gyro_age_us=1001)
        self.assertFalse(flags & 2048)
        self.assertFalse(flags & 8192)
        self.assertTrue(flags & 32768)
        flags, *_ = self.run_ticks(3401, 4500)
        self.assertTrue(flags & 8192)
        flags, *_ = self.run_ticks(4500, 4700, magnetic=False)
        self.assertEqual(flags & 0xe000, 0xc000)
        self.assertEqual(self.on.magnetic_diagnostics()['reason'], 'stale')
        flags, *_ = self.run_ticks(4700, 5800)
        self.assertTrue(flags & 8192)
        self.assertEqual(self.on.magnetic_diagnostics()['references'], 1)
        self.assertEqual(self.on.magnetic_diagnostics()['reference_offset_deg'], offset)

    def test_manual_recalibration_and_long_imu_gap_clear_reference(self):
        self.ready()
        self.on.lib.replay_recalibrate()
        flags, _ = self.on.update(self.frame(3400))
        self.assertTrue(flags & 1024)
        self.assertFalse(flags & 32768)
        self.run_ticks(3401, 6800)
        self.assertTrue(self.on.magnetic_diagnostics()['reference_ready'])
        flags, _ = self.on.update(self.frame(6811))
        self.assertTrue(flags & 4096)
        self.assertFalse(flags & 32768)

    def test_sequence_wrap_and_magnetic_overflow_only_suspend_aiding(self):
        self.ready()
        # A sequence discontinuity is rejected, then uint32 wrap is contiguous.
        for n in range(30):
            i = 3400+n*40
            self.on.submit_mag(self.mag_frame(i, sequence=(0xfffffff0+n) & 0xffffffff))
            for j in range(i, i+40):
                flags, _ = self.on.update(self.frame(j))
        self.assertTrue(flags & 8192)
        for n in range(9):
            self.on.submit_mag(self.mag_frame(4600+n, sequence=30+n))
        flags, _ = self.on.update(self.frame(4600))
        self.assertTrue(flags & 2048)
        self.assertEqual(flags & 0xe000, 0xc000)
        self.assertEqual(self.on.magnetic_diagnostics()['overflows'], 1)

    def test_mixed_wire_replay_chunking_crc_and_reference(self):
        records = []
        for i in range(3400):
            if i % 40 == 0: records.append(self.mag_frame(i))
            records.append(self.frame(i, protocol_version=2 if i % 2 else 3))
        bad = bytearray(records[9].encode()); bad[22] ^= 1
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp)/'usb.bin'
            path.write_bytes(b'noise'+bytes(bad)+b''.join(f.encode() for f in records)+b'IM')
            results = [replay.replay(path, self.on, chunk_size=size) for size in (1, 17, 65536)]
        self.assertEqual(results[0], results[1])
        self.assertEqual(results[0], results[2])
        self.assertEqual(results[0]['parser']['crc_errors'], 1)
        self.assertEqual(results[0]['paired_frames'], 3400)
        self.assertEqual(results[0]['magnetic']['records'], 85)
        self.assertGreater(results[0]['magnetic']['aiding_frames'], 300)


class MagneticStreamIntegrationTests(SyntheticField, unittest.TestCase):
    magnetic = True
    setUpClass = classmethod(stream.FirmwareNativeTests.setUpClass.__func__)
    tearDownClass = classmethod(stream.FirmwareNativeTests.tearDownClass.__func__)
    tearDown = stream.FirmwareNativeTests.tearDown
    sample = stream.FirmwareNativeTests.sample
    activate = stream.FirmwareNativeTests.activate
    usb_pump = stream.FirmwareNativeTests.usb_pump
    wire = stream.FirmwareNativeTests.wire

    def setUp(self):
        stream.FirmwareNativeTests.setUp(self)
        self.lib.mock_submit_mag.argtypes = [ctypes.c_uint64, ctypes.c_uint32,
            ctypes.POINTER(ctypes.c_float), ctypes.c_uint16, ctypes.c_uint8]
        self.lib.mock_rx_data.argtypes = [ctypes.POINTER(ctypes.c_uint8), ctypes.c_uint32]
        self.i = 0

    def last(self):
        data = bytes((ctypes.c_uint8*76).in_dll(self.lib, 'g_last_imu_frame'))
        return stream.imu.StreamParser().feed(data)[0]

    def tick(self, process=True, gz=0):
        i = self.i
        t = self.frame(i).timestamp_us
        self.now = t
        if i % 40 == 0:
            m = self.mag_frame(i)
            self.lib.mock_submit_mag(t, m.sequence, (ctypes.c_float*3)(m.x_ut,m.y_ut,m.z_ut), 15, 0)
        self.sample(1, t, (0,0,1365))
        self.sample(2, t, (0,0,gz))
        self.lib.stream_output_tick(t, t, 0)
        if process: self.lib.mock_process_at(t)
        self.i += 1

    def ready(self):
        for _ in range(3400): self.tick()
        self.assertEqual(self.last().flags & 0xa800, 0xa800)

    def test_usb_disconnected_boot_and_reconnect_never_reset_yaw_or_reference(self):
        self.ready()  # No connected USB, yet valid and magnetic aiding.
        self.assertEqual(self.wire(), b'')
        # Small residual bias creates a nonzero yaw to detect accidental rezero.
        for _ in range(500): self.tick(gz=-8)
        before = self.last()
        self.assertGreater(before.yaw_deg, .1)
        self.activate()
        self.tick(gz=-8); self.usb_pump(increment=0)
        connected = self.last()
        self.assertTrue(connected.flags & 16)
        self.assertEqual(connected.flags & 0xa800, 0xa800)
        self.assertAlmostEqual(before.yaw_deg, connected.yaw_deg, delta=.01)
        self.lib.mock_line_state(False)
        self.tick(gz=-8)
        self.lib.mock_line_state(True)
        self.usb_pump(increment=0)
        self.tick(gz=-8); self.usb_pump(increment=0)
        after = self.last()
        self.assertEqual(after.flags & 0xa800, 0xa800)
        self.assertFalse(after.flags & (1024|4096))
        self.assertAlmostEqual(before.yaw_deg, after.yaw_deg, delta=.01)

    def test_actual_stream_control_reset_clears_reference(self):
        self.ready(); self.activate()
        data = control_command(731)
        self.lib.mock_rx_data((ctypes.c_uint8*len(data)).from_buffer_copy(data), len(data))
        self.usb_pump(increment=0)
        # Activation already advanced the mock USB clock by one tick.
        # Advance past the 1 ms flush boundary to observe the queued ACK.
        self.tick(); self.usb_pump(increment=1000)
        f = self.last()
        self.assertTrue(f.flags & 1024)
        self.assertFalse(f.flags & (2048|32768))
        parser = stream.imu.StreamParser(); parser.feed(self.wire())
        replies = [f for f in parser.records if isinstance(f, stream.imu.ControlReply)]
        self.assertEqual(len(replies), 1)
        self.assertEqual(replies[0].request_id, 731)

    def test_actual_stream_imu_queue_overflow_and_idle_clear_reference(self):
        self.ready()
        for _ in range(65): self.tick(process=False)
        self.lib.mock_process_at(self.now)
        self.tick()
        self.assertFalse(self.last().flags & 32768)
        self.assertTrue(self.last().flags & 1024)
        for _ in range(3400): self.tick()
        self.assertTrue(self.last().flags & 32768)
        self.now += 11000
        self.lib.mock_process_at(self.now)
        self.i += 11
        self.tick()
        self.assertFalse(self.last().flags & 32768)
        self.assertTrue(self.last().flags & 1024)


if __name__ == '__main__': unittest.main()
