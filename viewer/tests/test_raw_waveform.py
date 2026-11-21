"""Regression: protocol status flags must not hide valid raw six-axis waveforms."""
import os
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
import time
import unittest
from unittest.mock import patch
import numpy as np
from PySide6 import QtWidgets
from imu_viewer.capture import Config, LiveConnection
from imu_viewer.gui import Window
from imu_viewer.history import render_history
from imu_viewer.protocol import Frame


def raw_frame(i, flags=0):
    return Frame(i, 1_000_000+i*1000, 10, -20, 1365, -40, 50, -60,
                 20, 10, flags=flags, protocol_version=3,
                 qw=1., qx=0., qy=0., qz=0.,
                 roll_deg=0., pitch_deg=0., yaw_deg=0.)


def receive(connection, start, count, flags, batch=0):
    raw = b"".join(raw_frame(i, flags).encode() for i in range(start, start+count))
    return connection._receive_usb(batch, raw, batch*1_000_000, batch*1_000_000+100)


class RawWaveformTests(unittest.TestCase):
    def test_cdc_status_bits_preserve_all_raw_axes(self):
        for flags in (0, 1, 16, 1024, 2048, 4096, 8192, 16384, 32768,
                      2048 | 8192 | 32768, 2048 | 16384 | 32768):
            with self.subTest(flags=hex(flags)):
                connection = LiveConnection(Config(port="synthetic"))
                self.assertTrue(receive(connection, 0, 20, flags))
                self.assertEqual(connection.history.info().samples, 20)
                _, curves = render_history(connection.history, 0, .019)
                expected = np.array([10, -20, 1365, -40, 50, -60])*connection.history.scales
                for axis, (x, y) in enumerate(curves):
                    finite = np.isfinite(x)
                    np.testing.assert_allclose(x[finite], np.arange(20)/1000)
                    np.testing.assert_allclose(y[finite], expected[axis])

    def test_heading_status_keeps_early_peak_history_and_recent_samples(self):
        connection = LiveConnection(Config(port="synthetic"))
        for batch, start in enumerate(range(0, 13000, 1000)):
            self.assertTrue(receive(connection, start, 1000, 2048 | 8192 | 32768, batch))
        self.assertGreater(connection.history.info().raw_start_us, 1_000_000)
        for lo, hi in ((0, 1), (12, 12.999)):
            _, curves = render_history(connection.history, lo, hi)
            for x, y in curves:
                self.assertGreater(len(x), 0)
                self.assertTrue(np.isfinite(y).all())
                self.assertGreaterEqual(x[0], lo)
                self.assertLessEqual(x[-1], hi)

    def test_real_raw_quality_errors_still_break_lines(self):
        for error_bit in (2, 4, 8, 32, 64, 128, 256, 512):
            with self.subTest(error_bit=error_bit):
                connection = LiveConnection(Config(port="synthetic"))
                frames = [raw_frame(i, (error_bit if i == 2 else 0) | 8192) for i in range(5)]
                raw = b"".join(f.encode() for f in frames)
                self.assertTrue(connection._receive_usb(0, raw, 0, 100))
                _, curves = render_history(connection.history, 0, .004)
                for x, y in curves:
                    self.assertNotIn(.002, x)
                    self.assertIn(.001, x)
                    self.assertIn(.003, x)
                    a, b = np.flatnonzero(x == .001)[0], np.flatnonzero(x == .003)[0]
                    self.assertTrue(np.isnan(y[a:b]).any())


class RawWaveformWidgetTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def test_cdc_to_main_window_curves_after_attitude_tab_switch(self):
        with patch.object(Window, "refresh_devices"):
            window = Window()
        connection = LiveConnection(Config(port="synthetic"))
        try:
            self.assertTrue(receive(connection, 0, 100, 2048 | 8192 | 32768))
            # Only emulate readiness; never start/open either hardware device.
            connection.metrics["camera_frames"] = 1
            window.connection = connection
            window.resize(1440, 960)
            window.show()
            window.tick()
            self.assertIs(window.waveforms.history, connection.history)
            window.telemetry_tabs.setCurrentIndex(1)
            self.app.processEvents()
            window.telemetry_tabs.setCurrentIndex(0)
            window.waveforms.live()
            curves = window.accel_chart.curves + window.gyro_chart.curves
            def has_points():
                return all(c.getData()[0] is not None and len(c.getData()[0]) > 0 for c in curves)
            deadline = time.perf_counter()+2
            while not has_points() and time.perf_counter() < deadline:
                self.app.processEvents()
                time.sleep(.002)
            self.assertTrue(has_points(), "Raw six-axis plots stayed empty after valid CDC data")
            expected = np.array([10, -20, 1365, -40, 50, -60])*connection.history.scales
            for curve, value in zip(curves, expected):
                np.testing.assert_allclose(curve.getData()[1], value)
        finally:
            window.connection = None
            window.close()
            self.app.processEvents()


if __name__ == "__main__":
    unittest.main()
