"""Save-only workflow: complete IMU-led associations without conversion or hardware."""
import os
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
import csv
import io
import json
import tempfile
import threading
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch, Mock
import numpy as np
from PySide6 import QtCore, QtGui, QtWidgets
from imu_viewer.capture import Config
from imu_viewer.protocol import Frame, ATTITUDE_FIELDS
from imu_viewer.storage import SessionWriter, finalize, read_csv
from imu_viewer.workflow import validate_output_root, resolve_raw_session
from imu_viewer.gui import Window
from imu_viewer.__main__ import main


class SavedAssociationTests(unittest.TestCase):
    def setUp(self):
        parent = Path(__file__).resolve().parents[1]/"test-results"
        parent.mkdir(exist_ok=True)
        temp = tempfile.TemporaryDirectory(dir=parent)
        self.addCleanup(temp.cleanup)
        self.root = Path(temp.name)

    def session(self, images=True, count=10):
        path = self.root/"session_test"/"raw"
        writer = SessionWriter(path, Config(port="FAKE", width=8, height=4))
        for i in range(count):
            valid = i % 3 == 0
            frame = Frame(i, 1000000+i*1000, -i, 2, 3, 4, 5, i, 20, 10,
                          flags=2048 if valid else 0, protocol_version=3 if i % 3 != 2 else 2,
                          qw=1, qx=0, qy=0, qz=0, roll_deg=1, pitch_deg=2, yaw_deg=3)
            host = 5_000_000_000+i*1_000_000
            writer.usb(i, frame.encode(), host-10000, host, [frame])
            if images and i in (0, 9):
                writer.image(i//9, float(i), i*1_000_000, host-100000, host,
                             np.zeros((4, 8, 3), np.uint8))
        if images:
            writer.image(2, 100., 100_000_000, 5_100_000_000, 5_100_010_000,
                         np.zeros((4, 8, 3), np.uint8))
        writer.close()
        return path

    def test_save_all_rows_attitude_blanks_and_original_images(self):
        path = self.session()
        originals = {p.relative_to(path):p.read_bytes() for p in path.rglob("*")
                     if p.is_file() and p.name != "session.json"}
        with patch("imu_viewer.storage.write_png", side_effect=AssertionError("no re-encoding")), \
             patch("imu_viewer.storage.cv2.imdecode", side_effect=AssertionError("no image reads")), \
             patch("imu_viewer.storage.export_euroc", side_effect=AssertionError("no export")):
            finalize(path, dict(error=None))
        rows = list(read_csv(path/"imu_image.csv"))
        raw = list(read_csv(path/"imu.csv"))
        self.assertEqual(len(rows), 10)
        for a, b in zip(rows, raw):
            self.assertEqual({k:a[k] for k in b}, b)
            if b["attitude_valid"] == "0":
                self.assertTrue(all(a[k] == "" for k in ATTITUDE_FIELDS))
        self.assertEqual([r["image_frame_id"] for r in rows], ["0"]+[""]*8+["1"])
        self.assertEqual(rows[3]["qw"], "1")  # valid attitude without image
        self.assertEqual(rows[3]["cam0"], "")
        for row in (rows[0], rows[9]):
            self.assertEqual(row["image_minus_imu_ns"], "0")
            self.assertTrue((path/row["cam0"]).is_file())
            self.assertTrue((path/row["cam1"]).is_file())
        self.assertEqual(originals, {k:(path/k).read_bytes() for k in originals})
        metadata = json.loads((path/"association.json").read_text(encoding="utf8"))
        self.assertEqual([metadata[k] for k in ("imu_rows", "matched_imu_rows", "unmatched_imu_rows",
                                                "image_frames", "unmatched_image_frames")], [10,2,8,3,1])
        self.assertEqual(metadata["match_tolerance_ns"], 20_000_000)
        self.assertFalse((path.parent/"export").exists())
        self.assertTrue((path/"cam0/000000002.png").is_file())  # unmatched image retained
        self.assertTrue(json.loads((path/"session.json").read_text(encoding="utf8"))["finalized"])
        before = (path/"imu_image.csv").read_bytes()
        finalize(path)
        self.assertEqual((path/"imu_image.csv").read_bytes(), before)

    def test_no_images_keeps_all_imu(self):
        path = self.session(images=False)
        finalize(path, dict(error=None))
        rows = list(read_csv(path/"imu_image.csv"))
        self.assertEqual(len(rows), 10)
        self.assertTrue(all(r[k] == "" for r in rows for k in
                            ("image_frame_id","image_timestamp_ns","image_minus_imu_ns","cam0","cam1")))

    def test_no_imu_has_header_and_retains_images(self):
        path = self.session(count=0)
        finalize(path, dict(error=None))
        self.assertEqual(list(read_csv(path/"imu_image.csv")), [])
        with (path/"imu_image.csv").open(newline="", encoding="utf8") as f:
            self.assertIn("yaw_deg", next(csv.reader(f)))
        self.assertTrue((path/"cam1/000000002.png").is_file())

    def test_atomic_failure_is_not_finalized_and_does_not_replace_table(self):
        path = self.session()
        finalize(path, dict(error=None))
        old = (path/"imu_image.csv").read_bytes()
        replace = Path.replace
        def fail_table(source, target):
            if Path(target).name == "imu_image.csv":
                raise OSError("disk full saving associations")
            return replace(source, target)
        with patch.object(Path, "replace", fail_table), self.assertRaisesRegex(OSError,"disk full"):
            finalize(path)
        self.assertFalse(json.loads((path/"session.json").read_text(encoding="utf8"))["finalized"])
        self.assertEqual((path/"imu_image.csv").read_bytes(), old)
        self.assertTrue((path/"usb.bin").is_file())
        finalize(path)
        self.assertTrue(json.loads((path/"session.json").read_text(encoding="utf8"))["finalized"])

    def test_single_root_and_container_resolution(self):
        self.assertEqual(validate_output_root(self.root/"a"/".."), self.root)
        with self.assertRaises(ValueError):
            validate_output_root(" ")
        source = self.session()
        self.assertEqual(resolve_raw_session(source.parent), source)
        self.assertEqual(resolve_raw_session(source), source)


class SaveGuiTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        font = Path("C:/Windows/Fonts/msyh.ttc")
        if font.exists():
            QtGui.QFontDatabase.addApplicationFont(str(font))
            cls.app.setFont(QtGui.QFont("Microsoft YaHei", 9))

    def window(self):
        with patch.object(Window, "refresh_devices"):
            window = Window()
        window.timer.stop()
        def cleanup():
            window.recorder = window.connection = None
            window.close()
            window.deleteLater()
            QtCore.QCoreApplication.sendPostedEvents(None, QtCore.QEvent.DeferredDelete)
        self.addCleanup(cleanup)
        window.show()
        return window

    def test_save_popup_waits_then_closes_once_without_export_job(self):
        window = self.window()
        done = threading.Event()
        sink = SimpleNamespace(done=done, path=Path("session/raw"), error=None, report={})
        window.recorder = sink
        window.show_save_progress("正在保存录制数据…", sink.path)
        dialog = window.save_dialog
        dialog.reject()
        self.assertTrue(dialog.isVisible())
        with patch.object(window, "task") as task:
            window.finish_recording()
            self.assertIs(window.recorder, sink)
            done.set()
            window.finish_recording()
            window.finish_recording()
            task.assert_not_called()
        self.assertIsNone(window.recorder)
        self.assertIsNone(window.save_dialog)
        self.assertFalse(dialog.isVisible())
        self.assertIn("保存完成", window.save_status.text())
        self.assertIn("imu_image.csv", window.save_status.text())
        self.assertFalse(hasattr(window, "export_button"))
        self.assertFalse(hasattr(window, "begin_export"))

    def test_failure_explicit_closable_and_not_false_success(self):
        window = self.window()
        done = threading.Event()
        done.set()
        window.recorder = SimpleNamespace(done=done, path=Path("session/raw"),
                                          error="Finalization: disk full", report={})
        window.finish_recording()
        dialog = window.save_dialog
        self.assertTrue(dialog.isVisible())
        self.assertEqual(dialog.windowTitle(), "保存失败")
        self.assertIn("disk full", dialog.details.text())
        self.assertIn("可能不完整", dialog.hint.text())
        self.assertIn("保存失败", window.save_status.text())
        self.assertTrue(dialog.close_button.isEnabled())
        dialog.close_button.click()
        self.assertFalse(dialog.isVisible())

    def test_complete_button_stops_recording_but_keeps_connection(self):
        window = self.window()
        done = threading.Event()
        window.recorder = SimpleNamespace(done=done, path=Path("session/raw"), stop_event=threading.Event())
        connection = Mock()
        window.connection = connection
        with patch.object(window, "set_recording_controls"):
            window.stop_recording()
        connection.stop_recording.assert_called_once()
        connection.stop.assert_not_called()
        self.assertTrue(window.save_dialog.isVisible())
        window.save_dialog.complete("test cleanup")


class SaveCliTests(unittest.TestCase):
    def run_record(self, error=None, wait=True):
        events = []
        recorder = Mock()
        recorder.start.return_value = Path("recordings/session/raw").resolve()
        recorder.done.wait.return_value = True
        recorder.wait.side_effect = lambda timeout: events.append("wait") or wait
        recorder.stop.side_effect = lambda: events.append("stop")
        recorder.report = {}
        recorder.snapshot.return_value = {}
        recorder.error = error
        with patch("sys.argv", ["imu_viewer","record","--output","viewer/recordings"]), \
             patch("imu_viewer.__main__.select_port", return_value=dict(port="FAKE", serial_number="UNIT")), \
             patch("imu_viewer.__main__.camera_inventory", return_value=[]), \
             patch("imu_viewer.__main__.Recorder", return_value=recorder), \
             patch("imu_viewer.storage.export_euroc", side_effect=AssertionError("no export")), \
             patch("sys.stdout", new_callable=io.StringIO) as output, \
             patch("sys.stderr", new_callable=io.StringIO) as errors:
            result = main()
        self.assertEqual(events, ["stop","wait"])
        return result, output.getvalue(), errors.getvalue()

    def test_record_save_only_and_failure_exit_status(self):
        result, output, errors = self.run_record()
        self.assertEqual(result, 0)
        self.assertIn("imu_image.csv", output)
        self.assertNotIn("export", output.lower())
        result, output, errors = self.run_record(error="Writer: disk full")
        self.assertEqual(result, 2)
        self.assertIn("保存失败", errors)
        result, output, errors = self.run_record(wait=False)
        self.assertEqual(result, 2)
        self.assertIn("timed out", errors)

    def test_cli_export_removed_and_empty_root_rejected_before_hardware(self):
        with patch("sys.argv", ["imu_viewer","export","a","b"]), \
             patch("sys.stderr", new_callable=io.StringIO), self.assertRaises(SystemExit) as error:
            main()
        self.assertEqual(error.exception.code, 2)
        with patch("sys.argv", ["imu_viewer","record","--output"," "]), \
             patch("imu_viewer.__main__.select_port") as select, \
             patch("sys.stderr", new_callable=io.StringIO):
            self.assertEqual(main(), 2)
            select.assert_not_called()


if __name__ == "__main__":
    unittest.main()
