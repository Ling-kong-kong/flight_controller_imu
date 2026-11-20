"""Automatic recording/export workflow, without hardware."""
import os
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
import io
import tempfile
import threading
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch, Mock
import numpy as np
from PySide6 import QtCore, QtGui, QtWidgets
from imu_viewer.capture import Config
from imu_viewer.protocol import Frame
from imu_viewer.storage import SessionWriter, finalize, read_csv
from imu_viewer.workflow import validate_output_root, resolve_raw_session, next_export_destination, export_recording
from imu_viewer.gui import Window
from imu_viewer.__main__ import main


class WorkflowTests(unittest.TestCase):
    def setUp(self):
        parent = Path(__file__).resolve().parents[1]/"test-results"
        parent.mkdir(exist_ok=True)
        self.temp = tempfile.TemporaryDirectory(dir=parent)
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)

    def session(self, finalized=True):
        path = self.root/"recordings"/"session_test"/"raw"
        writer = SessionWriter(path, Config(port="FAKE", width=8, height=4))
        image = np.zeros((4,8,3), np.uint8)
        for i in range(10):
            frame = Frame(i, 1000000+i*1000, 1,2,3,4,5,6,20,10,16 if i==0 else 0)
            host = 5000000000+i*1000000
            writer.usb(i, frame.encode(), host-10000, host, [frame])
            if i in (0,9):
                writer.image(i//9, float(i), i*1000000, host-100000, host, image)
        writer.close()
        if finalized:
            finalize(path, dict(error=None, timeline_anomaly=False))
        return path

    def test_single_root_and_container_resolution(self):
        root = validate_output_root(self.root/"a"/".."/"recordings")
        self.assertEqual(root, self.root/"recordings")
        with self.assertRaises(ValueError):
            validate_output_root(" ")
        source = self.session()
        self.assertEqual(resolve_raw_session(source.parent), source)
        self.assertEqual(resolve_raw_session(source), source)
        self.assertEqual(next_export_destination(source.parent), source.parent/"export")

    def test_legacy_export_stays_outside_raw_source(self):
        legacy = self.root/"session_old"
        self.assertEqual(resolve_raw_session(legacy), legacy)
        self.assertEqual(next_export_destination(legacy), self.root/"session_old_euroc")

    def test_unique_destination_keeps_completed_and_incomplete(self):
        source = self.root/"recordings"/"session_test"/"raw"
        exports = source.parent
        base = next_export_destination(source)
        base.mkdir(parents=True)
        (base/"export.json").write_text("original")
        incomplete = exports/(base.name+"_001")
        incomplete.mkdir()
        (incomplete/"EXPORT_INCOMPLETE").touch()
        self.assertEqual(next_export_destination(source).name, base.name+"_002")
        self.assertEqual((base/"export.json").read_text(), "original")
        self.assertTrue((incomplete/"EXPORT_INCOMPLETE").exists())

    def test_export_preserves_raw_and_all_imu_rows(self):
        source = self.session()
        before = {p.relative_to(source):p.read_bytes() for p in source.rglob("*") if p.is_file()}
        result = export_recording(source)
        destination = Path(result["destination"])
        self.assertEqual(result["imu_frames"],10)
        self.assertEqual(len(list(read_csv(destination/"imu_image.csv"))),10)
        self.assertEqual(result["source"],str(source))
        self.assertEqual(destination.parent,source.parent)
        self.assertFalse((destination/"EXPORT_INCOMPLETE").exists())
        self.assertEqual(before, {p.relative_to(source):p.read_bytes() for p in source.rglob("*") if p.is_file()})
        second = export_recording(source)
        self.assertEqual(Path(second["destination"]).name, destination.name+"_001")

    def test_unfinalized_is_not_exported_as_success(self):
        source = self.session(False)
        with self.assertRaisesRegex(RuntimeError, "导出失败"):
            export_recording(source)
        self.assertTrue((source/"usb.bin").exists())
        self.assertFalse(any(source.parent.rglob("export.json")))

    def test_conversion_failure_keeps_raw_and_incomplete_marker(self):
        source = self.session()
        raw = (source/"usb.bin").read_bytes()
        with patch("imu_viewer.storage.write_png", side_effect=OSError("disk error")):
            with self.assertRaisesRegex(RuntimeError, "重新导出"):
                export_recording(source)
        self.assertEqual((source/"usb.bin").read_bytes(),raw)
        target = source.parent/"export"
        self.assertTrue((target/"EXPORT_INCOMPLETE").exists())
        self.assertNotEqual(next_export_destination(source),target)


class AutoExportGuiTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        # Windows offscreen starts without fonts; load one before constructing plots.
        font = Path("C:/Windows/Fonts/msyh.ttc")
        if font.exists():
            QtGui.QFontDatabase.addApplicationFont(str(font))
            cls.app.setFont(QtGui.QFont("Microsoft YaHei", 9))

    def window(self):
        with patch.object(Window,"refresh_devices"):
            window = Window()
        self.addCleanup(window.close)
        return window

    def pump_until(self, predicate):
        deadline = time.perf_counter()+5
        while not predicate() and time.perf_counter()<deadline:
            self.app.processEvents()
            time.sleep(.01)
        self.assertTrue(predicate())

    def test_finalize_once_after_done_even_disconnected_and_close_waits(self):
        window = self.window()
        window.show()
        done, entered, release = threading.Event(), threading.Event(), threading.Event()
        source = Path(__file__).resolve().parent/"synthetic_session"/"raw"
        destination = source.parent/"export"
        sink = SimpleNamespace(done=done, path=source, error=None, report={})
        window.recorder = sink
        def export(source_arg):
            self.assertTrue(done.is_set())
            self.assertEqual(source_arg, source)
            entered.set()
            if not release.wait(5):
                raise RuntimeError("test timed out")
            return dict(source=str(source),destination=str(destination))
        with patch("imu_viewer.gui.export_recording",side_effect=export) as exporter:
            try:
                window.tick()
                exporter.assert_not_called()
                done.set()
                window.tick()
                self.pump_until(entered.is_set)
                self.assertTrue(window.busy)
                self.assertFalse(window.output.isEnabled())
                self.assertFalse(window.start_button.isEnabled())
                for _ in range(5):
                    window.tick()
                self.assertEqual(exporter.call_count,1)
                window.close()
                self.assertTrue(window.isVisible())
                self.assertTrue(window.closing)
                release.set()
                self.pump_until(lambda:not window.isVisible())
                self.assertFalse(window.busy)
                self.assertEqual(window.last_export,destination)
            finally:
                release.set()
                self.pump_until(lambda:not window.busy)

    def test_finish_button_shows_saving_then_export_and_auto_closes(self):
        window = self.window()
        window.timer.stop()
        window.show()
        self.assertEqual(window.stop_button.text(), "完成录制")
        done, entered, release = threading.Event(), threading.Event(), threading.Event()
        source = Path("session/raw").resolve()
        stop = Mock()
        window.connection = SimpleNamespace(stop_recording=stop)
        window.recorder = SimpleNamespace(done=done, path=source, error=None, report={})
        with patch.object(window, "set_recording_controls"):
            window.stop_recording()
            stop.assert_called_once()
        dialog = window.export_dialog
        finished = []
        dialog.finished.connect(finished.append)
        self.assertTrue(dialog.isVisible())
        self.assertFalse(dialog.isModal())
        self.assertIn("保存", dialog.stage.text())
        self.assertEqual(dialog.progress.maximum(), 0)
        # Closing the popup or pressing Esc must not hide an active operation.
        dialog.close()
        dialog.reject()
        self.assertTrue(dialog.isVisible())
        window.connection = None
        def export(_):
            entered.set()
            if not release.wait(5):
                raise RuntimeError("test timed out")
            return dict(source=str(source), destination=str(source.parent/"export"))
        with patch("imu_viewer.gui.export_recording", side_effect=export):
            try:
                done.set()
                window.finish_recording()
                self.pump_until(entered.is_set)
                self.assertIs(window.export_dialog, dialog)
                self.assertIn("正在导出", dialog.stage.text())
                # The Qt event loop stays responsive throughout background export.
                fired = []
                QtCore.QTimer.singleShot(0, lambda: fired.append(True))
                self.pump_until(lambda: bool(fired))
            finally:
                release.set()
                self.pump_until(lambda: not window.busy)
        self.assertEqual(finished, [QtWidgets.QDialog.Accepted])
        self.assertIsNone(window.export_dialog)

    def test_failed_export_popup_stays_visible_and_retry_replaces_it(self):
        window = self.window()
        window.show()
        with patch("imu_viewer.gui.export_recording", side_effect=OSError("disk full")):
            window.begin_export(Path("session/raw"))
            self.pump_until(lambda: not window.busy)
        failed = window.export_dialog
        self.assertTrue(failed.isVisible())
        self.assertFalse(failed.running)
        self.assertIn("导出失败", failed.stage.text())
        self.assertIn("disk full", failed.details.text())
        self.assertTrue(failed.close_button.isVisible())
        failed.close()
        self.assertFalse(failed.isVisible())
        with patch("imu_viewer.gui.export_recording", return_value=dict(source="raw", destination="export")):
            window.begin_export(Path("session/raw"))
            self.assertIsNot(window.export_dialog, failed)
            self.assertTrue(window.export_dialog.isVisible())
            self.pump_until(lambda: not window.busy)
        self.assertIsNone(window.export_dialog)

    def test_export_failure_visible_and_retry_clears_only_export_error(self):
        window = self.window()
        window.show()
        with patch("imu_viewer.gui.export_recording",side_effect=OSError("space full")):
            window.begin_export(Path("session/raw"))
            self.pump_until(lambda:not window.busy)
        self.assertIsNone(window.last_export)
        self.assertTrue(window.error_banner.isVisible())
        self.assertIn("space full",window.export_status.text())
        self.assertTrue(window.export_button.isEnabled())
        with patch("imu_viewer.gui.export_recording",return_value=dict(source="raw",destination="exports/result")):
            window.begin_export(Path("session/raw"))
            self.assertFalse(window.error_banner.isVisible())
            self.pump_until(lambda:not window.busy)
        window.error_banner.setText("recording failed")
        window.error_banner.show()
        with patch("imu_viewer.gui.export_recording",return_value=dict(source="raw",destination="exports/result_001")):
            window.begin_export(Path("session/raw"))
            self.pump_until(lambda:not window.busy)
        self.assertTrue(window.error_banner.isVisible())
        self.assertEqual(window.error_banner.text(),"recording failed")


class AutoExportCliTests(unittest.TestCase):
    def test_cli_waits_before_export_and_passes_session(self):
        source = Path("recordings/session/raw").resolve()
        events = []
        recorder = Mock()
        recorder.start.return_value = source
        recorder.done.wait.return_value = True
        recorder.wait.side_effect = lambda timeout: events.append("wait") or True
        recorder.stop.side_effect = lambda: events.append("stop")
        recorder.report = {}
        recorder.snapshot.return_value = {}
        recorder.error = None
        def export(path):
            self.assertEqual(events,["stop","wait"])
            self.assertEqual(path,source)
            events.append("export")
            return {}
        with patch("sys.argv",["imu_viewer","record","--output","raw"]), \
             patch("imu_viewer.__main__.select_port",return_value=dict(port="FAKE",serial_number="UNIT")), \
             patch("imu_viewer.__main__.camera_inventory",return_value=[]), \
             patch("imu_viewer.__main__.Recorder",return_value=recorder) as factory, \
             patch("imu_viewer.__main__.export_recording",side_effect=export), \
             patch("sys.stdout",new_callable=io.StringIO):
            self.assertEqual(main(),0)
        self.assertEqual(events,["stop","wait","export"])
        self.assertIsInstance(factory.call_args.args[0].output,str)
        self.assertEqual(factory.call_args.args[0].output,str(Path("raw").resolve()))

    def test_cli_rejects_empty_root_before_opening_hardware(self):
        with patch("sys.argv",["imu_viewer","record","--output"," "]), \
             patch("imu_viewer.__main__.select_port") as select, \
             patch("sys.stderr",new_callable=io.StringIO):
            self.assertEqual(main(),2)
            select.assert_not_called()

    def test_cli_export_failure_returns_error(self):
        recorder = Mock()
        recorder.start.return_value = Path("raw/session")
        recorder.done.wait.return_value = recorder.wait.return_value = True
        with patch("sys.argv",["imu_viewer","record","--output","raw"]), \
             patch("imu_viewer.__main__.select_port",return_value=dict(port="FAKE",serial_number="UNIT")), \
             patch("imu_viewer.__main__.camera_inventory",return_value=[]), \
             patch("imu_viewer.__main__.Recorder",return_value=recorder), \
             patch("imu_viewer.__main__.export_recording",side_effect=OSError("disk full")), \
             patch("sys.stdout",new_callable=io.StringIO), patch("sys.stderr",new_callable=io.StringIO):
            self.assertEqual(main(),2)


if __name__ == "__main__":
    unittest.main()
