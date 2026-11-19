"""Offscreen integration: preview + six-axis paint + device-free joint recording."""
import os
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
import tempfile
import time
import unittest
import serial
from pathlib import Path
from unittest.mock import patch
from PySide6 import QtWidgets
from imu_viewer.gui import Window
from test_capture import FakeSerial, FakeCamera

class GuiTests(unittest.TestCase):
    def test_connect_preview_record_stop_preview_disconnect(self):
        app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        parent = Path(__file__).resolve().parents[1]/"test-results"
        parent.mkdir(exist_ok=True)
        identity = dict(port="FAKE", serial_number="UNIT", vid=0xCAFE, pid=0x4011,
                        h743=True, description="Synthetic H743")
        FakeCamera.failure_after = FakeSerial.failure_after = None
        FakeCamera.missing_pts = False
        with tempfile.TemporaryDirectory(dir=parent) as temp, \
             patch("imu_viewer.gui.discover_ports", return_value=[identity]), \
             patch("imu_viewer.gui.camera_inventory", return_value=[]), \
             patch("imu_viewer.capture.select_port", return_value=identity), \
             patch("imu_viewer.capture.serial.Serial", FakeSerial), \
             patch("imu_viewer.capture.cv2.VideoCapture", FakeCamera):
            window = Window()
            window.show()
            try:
                deadline = time.perf_counter()+3
                while not window.ports.count() and time.perf_counter()<deadline:
                    app.processEvents()
                    time.sleep(.01)
                self.assertEqual(window.ports.count(), 1)
                window.resolution.addItem("16×8")
                window.resolution.setCurrentText("16×8")
                window.output.setText(temp)
                window.connect_devices()
                connection = window.connection
                self.assertIsNotNone(connection)
                deadline = time.perf_counter()+3
                while not connection.metrics["camera_frames"] and time.perf_counter()<deadline:
                    app.processEvents()
                    time.sleep(.01)
                for _ in range(15):
                    app.processEvents()
                    time.sleep(.02)
                self.assertIsNone(window.recorder)
                self.assertEqual(list(Path(temp).iterdir()), [])
                self.assertFalse(window.preview_label.pixmap().isNull())
                self.assertGreater(len(window.accel_chart.samples), 0)
                self.assertTrue(window.start_button.isEnabled())
                history = window.waveforms.history
                self.assertIs(history, connection.history)
                samples_before = history.info().samples
                window.waveforms.toggle_expand(window.accel_chart)
                expanded = window.waveforms.expanded[window.accel_chart]
                window.start_recording()
                recorder = window.recorder
                self.assertIsNotNone(recorder)
                for _ in range(35):
                    app.processEvents()
                    time.sleep(.02)
                self.assertFalse(window.preview_label.pixmap().isNull())
                self.assertGreater(len(window.accel_chart.samples), 0)
                self.assertGreater(len(window.gyro_chart.samples), 0)
                self.assertFalse(window.start_button.isEnabled())
                self.assertTrue(window.stop_button.isEnabled())
                self.assertTrue(window.grab().save(str(parent/"gui-synthetic.png")))
                self.assertGreater(history.info().samples, samples_before)
                self.assertIs(window.waveforms.history, history)
                self.assertGreater(recorder.metrics["camera_frames"], 0)
                expanded.showMaximized()
                app.processEvents()
                self.assertTrue(expanded.isMaximized())
                expanded.close()
                app.processEvents()
                self.assertEqual(window.waveforms.expanded, {})
                window.stop_recording()
                deadline = time.perf_counter()+5
                while window.recorder and time.perf_counter()<deadline:
                    app.processEvents()
                    time.sleep(.01)
                self.assertIsNone(window.recorder)
                self.assertIsNone(recorder.error)
                self.assertGreater(recorder.writer.images, 0)
                self.assertEqual(recorder.writer.samples, recorder.metrics["imu_frames"])
                self.assertTrue(window.start_button.isEnabled())
                before = connection.metrics["camera_frames"]
                for _ in range(15):
                    app.processEvents()
                    time.sleep(.02)
                self.assertGreater(connection.metrics["camera_frames"], before)
                self.assertFalse(connection.done.is_set())
                window.disconnect_devices()
                deadline = time.perf_counter()+5
                while window.connection and time.perf_counter()<deadline:
                    app.processEvents()
                    time.sleep(.01)
                self.assertIsNone(window.connection)
                self.assertIs(window.waveforms.history, history)
                saved_count = history.info().samples
                self.assertEqual(saved_count, connection.metrics["imu_frames"])
                # A failed attempt must not clear the disconnected history.
                with patch("imu_viewer.capture.serial.Serial", side_effect=serial.SerialException("busy")):
                    window.connect_devices()
                    deadline = time.perf_counter()+5
                    while window.connection and time.perf_counter()<deadline:
                        app.processEvents()
                        time.sleep(.01)
                self.assertIsNone(window.connection)
                self.assertIs(window.waveforms.history, history)
                self.assertEqual(history.info().samples, saved_count)
                # Only a successful new connection switches to a fresh history.
                window.connect_devices()
                new_connection = window.connection
                deadline = time.perf_counter()+5
                while window.waveforms.history is history and time.perf_counter()<deadline:
                    app.processEvents()
                    time.sleep(.01)
                self.assertIs(window.waveforms.history, new_connection.history)
                self.assertIsNot(window.waveforms.history, history)
                window.disconnect_devices()
                deadline = time.perf_counter()+5
                while window.connection and time.perf_counter()<deadline:
                    app.processEvents()
                    time.sleep(.01)
                self.assertTrue(all(c.closed for c in FakeCamera.instances))
                self.assertTrue(all(s.closed for s in FakeSerial.instances))
            finally:
                if window.connection:
                    window.connection.stop()
                    window.connection.wait(5)
                    window.connection = None
                window.recorder = None
                window.close()
                app.processEvents()

    def test_close_window_drains_recording_and_releases_connection(self):
        app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        identity = dict(port="FAKE", serial_number="UNIT", h743=True,
                        description="Synthetic H743")
        FakeCamera.instances.clear()
        FakeSerial.instances.clear()
        FakeCamera.failure_after = FakeSerial.failure_after = None
        FakeCamera.missing_pts = False
        parent = Path(__file__).resolve().parents[1]/"test-results"
        parent.mkdir(exist_ok=True)
        with tempfile.TemporaryDirectory(dir=parent) as temp, \
             patch("imu_viewer.gui.discover_ports", return_value=[identity]), \
             patch("imu_viewer.gui.camera_inventory", return_value=[]), \
             patch("imu_viewer.capture.select_port", return_value=identity), \
             patch("imu_viewer.capture.serial.Serial", FakeSerial), \
             patch("imu_viewer.capture.cv2.VideoCapture", FakeCamera):
            window = Window()
            window.show()
            connection = None
            try:
                deadline = time.perf_counter()+3
                while not window.ports.count() and time.perf_counter()<deadline:
                    app.processEvents()
                    time.sleep(.01)
                window.resolution.addItem("16×8")
                window.resolution.setCurrentText("16×8")
                window.output.setText(temp)
                window.connect_devices()
                connection = window.connection
                deadline = time.perf_counter()+3
                while not connection.metrics["camera_frames"] and time.perf_counter()<deadline:
                    app.processEvents()
                    time.sleep(.01)
                window.start_recording()
                recorder = window.recorder
                self.assertIsNotNone(recorder)
                for _ in range(15):
                    app.processEvents()
                    time.sleep(.02)
                window.waveforms.toggle_expand(window.gyro_chart)
                self.assertIn(window.gyro_chart, window.waveforms.expanded)
                window.close()
                self.assertTrue(window.closing)
                deadline = time.perf_counter()+6
                while window.isVisible() and time.perf_counter()<deadline:
                    app.processEvents()
                    time.sleep(.01)
                self.assertFalse(window.isVisible())
                self.assertEqual(window.waveforms.expanded, {})
                self.assertFalse(window.waveforms.worker.thread.is_alive())
                self.assertTrue(connection.done.is_set())
                self.assertTrue(recorder.done.is_set())
                self.assertTrue(recorder.writer.closed)
                self.assertIsNone(recorder.error)
                self.assertTrue(all(c.closed for c in FakeCamera.instances))
                self.assertTrue(all(s.closed for s in FakeSerial.instances))
            finally:
                if connection:
                    connection.stop()
                    connection.wait(6)
                window.connection = window.recorder = None
                window.close()
                app.processEvents()
