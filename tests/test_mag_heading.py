"""Synthetic tests of the real C magnetic gate (no hardware or fitting)."""
import ctypes
import json
import hashlib
import re
import math
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
if os.name == "nt":
    os.environ["PATH"] = r"D:\msys64\ucrt64\bin" + os.pathsep + os.environ["PATH"]


class MagneticHeadingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory(prefix="mag-heading-")
        cls.library = Path(cls.temp.name)/"heading.dll"
        subprocess.run([shutil.which("cc"), "-std=c11", "-O2", "-Wall", "-Wextra", "-Werror",
            "-shared", "-fPIC", "-I", str(ROOT/"firmware/include"),
            str(ROOT/"tests/native/mag_heading_harness.c"),
            str(ROOT/"firmware/src/mag_heading.c"), "-lm", "-o", str(cls.library)],
            check=True, capture_output=True, text=True)
        cls.lib = ctypes.CDLL(str(cls.library))
        cls.lib.test_submit.argtypes = [ctypes.c_uint64, ctypes.c_uint32, ctypes.POINTER(ctypes.c_float),
                                        ctypes.c_uint16, ctypes.c_uint8]
        cls.lib.test_step.argtypes = [ctypes.c_uint64, ctypes.c_float, ctypes.c_float, ctypes.c_float,
                                     ctypes.c_float, ctypes.c_bool, ctypes.c_bool]
        cls.lib.test_step.restype = ctypes.c_float
        cls.lib.test_step_q.argtypes = [ctypes.c_uint64, ctypes.POINTER(ctypes.c_float),
            ctypes.c_float, ctypes.c_float, ctypes.c_bool, ctypes.c_bool]
        cls.lib.test_innovation.restype = ctypes.c_float
        cls.lib.test_reason.restype = ctypes.c_uint
        cls.lib.test_value.argtypes = [ctypes.c_uint]
        cls.lib.test_value.restype = ctypes.c_float
        cls.lib.test_flags.restype = ctypes.c_uint
        cls.candidate = json.loads((ROOT/"config/bmm150-calibration-candidate.json").read_text())

    @classmethod
    def tearDownClass(cls):
        if os.name == "nt":
            import _ctypes
            _ctypes.FreeLibrary(cls.lib._handle)
        cls.temp.cleanup()

    def setUp(self):
        self.lib.test_reset()
        self.t = 1000000
        self.seq = 0

    def chip(self, heading=0, roll=0, pitch=0, dip=55, norm=47.8896005862):
        # NED field; body vector R_x(-r) R_y(-p) R_z(-heading) field.
        h, r, p, d = map(math.radians, (heading, roll, pitch, dip))
        x, y, z = norm*math.cos(d)*math.cos(h), -norm*math.cos(d)*math.sin(h), norm*math.sin(d)
        x, z = math.cos(p)*x-math.sin(p)*z, math.sin(p)*x+math.cos(p)*z
        y, z = math.cos(r)*y+math.sin(r)*z, -math.sin(r)*y+math.cos(r)*z
        # Inverse of fixed M and experimental R. Solve without NumPy.
        m = self.candidate["correction_matrix"]
        b = self.candidate["bias_ut"]
        a = [list(m[i])+[(-x, -y, z)[i]] for i in range(3)]
        for i in range(3):
            v = a[i][i]
            a[i] = [v2/v for v2 in a[i]]
            for j in range(3):
                if j != i:
                    v = a[j][i]
                    a[j] = [a[j][k]-v*a[i][k] for k in range(4)]
        return [a[i][3]+b[i] for i in range(3)]

    def sample(self, heading=0, roll=0, pitch=0, dip=55, norm=47.8896005862,
               yaw=0, rate=0, valid=True, gravity=True, flags=15, error=0):
        self.t += 40000
        self.lib.test_submit(self.t, self.seq, (ctypes.c_float*3)(*self.chip(heading, roll, pitch, dip, norm)), flags, error)
        self.seq += 1
        return self.lib.test_step(self.t, roll, pitch, yaw, rate, valid, gravity)

    def ready(self, **kwargs):
        for _ in range(30):
            self.sample(**kwargs)
        self.assertEqual(self.lib.test_flags() & 0xa000, 0xa000)

    def test_fixed_candidate_unchanged_and_calibration_math(self):
        header = (ROOT/"firmware/include/mag_calibration.h").read_text()
        digest = hashlib.sha256((ROOT/"config/bmm150-calibration-candidate.json").read_bytes()).hexdigest()
        self.assertIn('"'+digest+'"', header)
        bias = header.split('mag_bias_ut[3] = {')[1].split('}')[0]
        matrix = header.split('mag_correction[3][3] = {')[1].split('};')[0]
        floats = lambda s: [float(v) for v in re.findall(r'([-+]?\d+\.\d+(?:e[-+]?\d+)?)f', s)]
        for actual, expected in zip(floats(bias)+floats(matrix),
                                    self.candidate['bias_ut']+sum(self.candidate['correction_matrix'], [])):
            self.assertAlmostEqual(actual, expected, delta=1e-7)
        self.assertEqual(len(floats(bias)), 3)
        self.assertEqual(len(floats(matrix)), 9)
        self.assertAlmostEqual(float(re.search(r'MAG_REFERENCE_NORM_UT (.*)f', header)[1]),
                               self.candidate['reference_norm_ut'], delta=1e-7)
        self.assertFalse(self.candidate["firmware_applied"])
        self.assertFalse(self.candidate["mounting_rotation_verified"])
        self.ready(heading=37, roll=25, pitch=-30)
        self.assertAlmostEqual(self.lib.test_value(0), 37, delta=.001)  # frozen offset
        self.assertAlmostEqual(self.lib.test_value(1), 55, delta=.001)  # frozen dip

    def test_relative_zero_and_delayed_reference_does_not_erase_real_yaw(self):
        self.ready(heading=73, yaw=28)
        self.assertAlmostEqual(self.lib.test_value(0), 45, delta=.001)
        correction = self.sample(heading=78, yaw=32)
        self.assertGreater(correction, 0)
        self.assertLess(correction, .001)  # bounded low bandwidth, not a snap

    def test_tilt_compensation_wrap_and_yaw_only_rotation(self):
        self.ready(heading=175, roll=35, pitch=-40)
        correction = self.sample(heading=-175, roll=35, pitch=-40, yaw=9)
        self.assertGreater(correction, 0)
        self.assertAlmostEqual(self.lib.test_value(2), 35, delta=.001)
        self.assertAlmostEqual(self.lib.test_value(3), -40, delta=.001)
        self.assertLess(abs(self.lib.test_value(4)-1), 1e-6)

    def test_reference_is_observable_at_vertical_pitch(self):
        for pitch in (80, 90, -90, 100, -100):
            with self.subTest(pitch=pitch):
                self.setUp()
                self.ready(heading=37, roll=25, pitch=pitch, yaw=12)
                self.assertAlmostEqual(self.lib.test_value(0), 25, delta=.001)
                self.assertAlmostEqual(self.lib.test_value(1), 55, delta=.001)

    @staticmethod
    def quaternion(roll, pitch, yaw):
        r, p, y = (math.radians(v)/2 for v in (roll, pitch, yaw))
        cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
        return [cr*cp*cy+sr*sp*sy, sr*cp*cy-cr*sp*sy,
                cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy]

    @staticmethod
    def world_z_rotate(q, degrees):
        c, s = math.cos(math.radians(degrees)/2), math.sin(math.radians(degrees)/2)
        w, x, y, z = q
        return [c*w-s*z, c*x-s*y, c*y+s*x, c*z+s*w]

    @staticmethod
    def body_gravity(q):
        w, x, y, z = q
        return [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]

    def submit_q(self, q, *, heading=37, roll=25, pitch=90, dip=55,
                 norm=47.8896005862, dt=.001, rate=0):
        self.t += 40000
        self.lib.test_submit(self.t, self.seq,
            (ctypes.c_float*3)(*self.chip(heading, roll, pitch, dip, norm)), 15, 0)
        self.seq += 1
        out = (ctypes.c_float*4)(*q)
        self.lib.test_step_q(self.t, out, dt, rate, True, True)
        return list(out)

    def test_world_z_correction_at_vertical_has_both_signs_and_preserves_gravity(self):
        for pitch in (-180, -100, -90, -89.999, 0, 89.999, 90, 100, 180):
            for error in (-20, -5, 5, 20):
                with self.subTest(pitch=pitch, error=error):
                    self.setUp()
                    self.ready(heading=37, roll=25, pitch=pitch, yaw=12)
                    q = self.quaternion(25, pitch, 12+error)
                    out = self.submit_q(q, pitch=pitch)
                    self.assertTrue(self.lib.test_flags() & 0x2000)
                    correction = max(-1, min(1, -.15*error))*.001
                    expected = self.world_z_rotate(q, correction)
                    for a, b in zip(out, expected): self.assertAlmostEqual(a, b, delta=2e-7)
                    for a, b in zip(self.body_gravity(out), self.body_gravity(q)):
                        self.assertAlmostEqual(a, b, delta=6e-7)
                    self.assertAlmostEqual(sum(v*v for v in out), 1, delta=4e-7)

    def test_frozen_reference_survives_crossing_vertical_and_quaternion_sign(self):
        self.ready(heading=37, roll=25, yaw=12)
        for roll, pitch, yaw in ((25, 80, 12), (25, 90, 12), (25, 100, 12),
                                 (120, 180, 50), (-140, -90, -170), (0, -100, 177)):
            for sign in (-1, 1):
                q = [sign*v for v in self.quaternion(roll, pitch, yaw)]
                out = self.submit_q(q, heading=yaw+25, roll=roll, pitch=pitch, rate=60)
                self.assertTrue(self.lib.test_flags() & 0x2000)
                self.assertAlmostEqual(self.lib.test_innovation(), 0, delta=.0001)
                for a, b in zip(out, q): self.assertAlmostEqual(a, b, delta=2e-7)
        self.assertAlmostEqual(self.lib.test_value(0), 25, delta=.001)

    def test_vertical_still_rejects_bad_horizontal_dip_norm_and_innovation(self):
        for bad, reason in ((dict(dip=89), 10), (dict(dip=75), 12),
                            (dict(norm=70), 9), (dict(heading=70), 13)):
            with self.subTest(bad=bad):
                self.setUp(); self.ready(heading=37, roll=25, pitch=90, yaw=12)
                q = self.quaternion(25, 90, 12)
                out = self.submit_q(q, **bad)
                self.assertFalse(self.lib.test_flags() & 0x2000)
                self.assertTrue(self.lib.test_flags() & 0x8000)
                self.assertEqual(self.lib.test_reason(), reason)
                for a, b in zip(out, q): self.assertAlmostEqual(a, b, delta=1e-7)
                self.ready(heading=37, roll=25, pitch=90, yaw=12)
                self.assertAlmostEqual(self.lib.test_value(0), 25, delta=.001)

    def test_invalid_quaternions_stop_aiding_without_mutating_input(self):
        for q in ([0]*4, [float('nan'), 0, 0, 0], [float('inf'), 0, 0, 0],
                  [1e30]*4, [.1, 0, 0, 0], [2, 0, 0, 0]):
            with self.subTest(q=q):
                self.setUp(); self.ready()
                out = self.submit_q(q)
                self.assertEqual(self.lib.test_reason(), 2)  # IMU, not magnetic quality
                self.assertFalse(self.lib.test_flags() & 0x2000)
                for a, b in zip(out, q):
                    if math.isnan(b): self.assertTrue(math.isnan(a))
                    else: self.assertAlmostEqual(a, b, delta=abs(b)*1e-6)

    def test_held_residual_does_not_damp_vertical_true_rotation(self):
        self.ready(heading=37, roll=25, pitch=90, yaw=12)
        q = self.quaternion(25, 90, 17)  # five degree world-Z estimation error
        out = self.submit_q(q)
        innovation = self.lib.test_innovation()
        # No new 25 Hz measurement: a real gyro turn of 2 degrees must survive.
        turned = self.world_z_rotate(out, 2)
        out = (ctypes.c_float*4)(*turned)
        self.lib.test_step_q(self.t+1000, out, .001, 50, True, True)
        expected = self.world_z_rotate(turned, .15*innovation*.001)
        for a, b in zip(out, expected): self.assertAlmostEqual(a, b, delta=2e-7)
        # Invalid integration intervals cannot apply a magnetic correction.
        for dt in (0, -.001, .002, float('nan'), float('inf')):
            q = list(out)
            self.lib.test_step_q(self.t+2000, out, dt, 0, True, True)
            self.assertEqual(list(out), q)

    def test_reject_fields_and_recover_without_relearning(self):
        for bad in [dict(norm=70), dict(heading=60), dict(dip=75), dict(flags=14),
                    dict(error=1), dict(dip=89), dict(rate=100),
                    dict(gravity=False), dict(valid=False)]:
            with self.subTest(bad=bad):
                self.setUp()
                self.ready(heading=15)
                offset = self.lib.test_value(0)
                self.assertEqual(self.sample(heading=15, **bad) if "heading" not in bad else self.sample(**bad), 0)
                self.assertFalse(self.lib.test_flags() & 0x2000)
                self.assertTrue(self.lib.test_flags() & 0x8000)
                for _ in range(19): self.sample(heading=15)
                self.assertFalse(self.lib.test_flags() & 0x2000)
                for _ in range(4): self.sample(heading=15)
                self.assertTrue(self.lib.test_flags() & 0x2000)
                self.assertEqual(self.lib.test_value(0), offset)

    def test_stale_future_duplicate_gap_overflow_nonfinite(self):
        self.ready()
        # Future sample must not affect the past.
        self.lib.test_submit(self.t+40000, self.seq, (ctypes.c_float*3)(*self.chip(heading=70)), 15, 0)
        self.seq += 1
        self.lib.test_step(self.t+1000, 0, 0, 0, 0, True, True)
        self.assertTrue(self.lib.test_flags() & 0x2000)
        self.lib.test_step(self.t+40000, 0, 0, 0, 0, True, True)
        self.assertFalse(self.lib.test_flags() & 0x2000)
        self.setUp(); self.ready()
        self.lib.test_step(self.t+100001, 0, 0, 0, 0, True, True)
        self.assertFalse(self.lib.test_flags() & 0x2000)
        for kind in ("duplicate", "gap", "overflow", "nan", "outorder"):
            with self.subTest(kind=kind):
                self.setUp(); self.ready()
                if kind == "duplicate": self.seq -= 1
                if kind == "gap": self.seq += 1
                if kind == "outorder": self.t -= 80000
                if kind == "overflow":
                    for i in range(9):
                        self.lib.test_submit(self.t+100+i, self.seq+i, (ctypes.c_float*3)(*self.chip()), 15, 0)
                elif kind == "nan":
                    self.t += 40000
                    self.lib.test_submit(self.t, self.seq, (ctypes.c_float*3)(float("nan"), 0, 0), 15, 0)
                else:
                    self.sample()
                self.lib.test_step(self.t+1000, 0, 0, 0, 0, True, True)
                self.assertFalse(self.lib.test_flags() & 0x2000)

    def test_initial_movement_delays_reference_and_bad_field_never_learns(self):
        for _ in range(100): self.sample(rate=6)
        self.assertFalse(self.lib.test_flags() & 0x8000)
        self.ready()
        for _ in range(100): self.sample(heading=70)
        self.assertFalse(self.lib.test_flags() & 0x2000)
        self.assertAlmostEqual(self.lib.test_value(0), 0, delta=.001)
        self.lib.test_reset()
        self.assertEqual(self.lib.test_flags() & 0x8000, 0)


if __name__ == "__main__": unittest.main()
