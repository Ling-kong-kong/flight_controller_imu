"""Calibration progress/retry GUI regressions; synthetic USB, no hardware I/O."""
import os
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
import unittest
from unittest.mock import patch
from PySide6 import QtCore, QtTest, QtWidgets
from imu_viewer.capture import Config, LiveConnection
from imu_viewer.gui import Window
from imu_viewer.protocol import ControlReply
from test_attitude import frame
from test_heading_control import diagnostic


class CalibrationDialogTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def setUp(self):
        with patch.object(Window, "refresh_devices"):
            self.window = Window()
        self.window.timer.stop()
        self.connection = self.make_connection()
        self.window.connection = self.connection
        self.window.update_calibration()

    def tearDown(self):
        self.window.connection = None
        self.window.close()
        self.window.deleteLater()
        self.app.sendPostedEvents(None, QtCore.QEvent.DeferredDelete)
        self.app.processEvents()

    def make_connection(self):
        connection = LiveConnection(Config(port="FAKE", width=8, height=4))
        connection._receive_usb(0, frame().encode(), 100, 110)
        return connection

    def start(self):
        with patch.object(QtWidgets.QMessageBox, "question", return_value=QtWidgets.QMessageBox.Yes):
            self.window.recalibrate_imu()
        return self.window.calibration_dialog

    def timeout(self):
        self.connection.recalibration.command(10)
        self.connection.recalibration.tick(13)
        self.window.update_calibration()

    def complete(self):
        request = self.connection.calibration_snapshot()["request_id"]
        self.connection.recalibration.command(10)
        raw = ControlReply(request, 0, 1001000, 1).encode() + diagnostic(request, timestamp_us=1002000, sequence=2).encode() + frame(2).encode()
        self.connection._receive_usb(1, raw, 200, 210)
        self.window.update_calibration()

    def test_confirmation_no_does_not_show_dialog_or_send_request(self):
        with patch.object(QtWidgets.QMessageBox, "question", return_value=QtWidgets.QMessageBox.No):
            self.window.recalibrate_imu()
        self.assertIsNone(self.window.calibration_dialog)
        self.assertEqual(self.connection.calibration_snapshot()["state"], "idle")

    def test_nonmodal_progress_and_ack_alone_never_mean_complete(self):
        dialog = self.start()
        self.assertTrue(dialog.isVisible())
        self.assertEqual(dialog.windowModality(), QtCore.Qt.NonModal)
        self.assertEqual(dialog.state, "queued")
        self.assertFalse(dialog.retry_button.isVisible())
        self.assertFalse(self.window.recalibrate_button.isEnabled())
        request = dialog.request_id
        self.connection.recalibration.command(10)
        self.window.update_calibration()
        self.assertEqual(dialog.state, "awaiting_ack")
        self.connection._receive_usb(1, ControlReply(request, 0, 1001000, 1).encode(), 200, 210)
        self.window.update_calibration()
        self.assertEqual(dialog.state, "calibrating")
        self.assertFalse(dialog.close_timer.isActive())
        self.connection._receive_usb(2, frame(2, flags=1024).encode(), 300, 310)
        self.window.update_calibration()
        self.assertEqual(self.connection.history.info().samples, 2)
        self.assertEqual(dialog.state, "calibrating")

    def test_complete_stays_one_second_and_polling_does_not_restart_timer(self):
        dialog = self.start()
        self.complete()
        self.assertEqual(dialog.stage.text(), "校准完成")
        self.assertTrue(dialog.isVisible())
        self.assertTrue(dialog.close_timer.isActive())
        QtTest.QTest.qWait(600)
        self.window.update_calibration()
        self.assertLess(dialog.close_timer.remainingTime(), 600)
        self.assertTrue(dialog.isVisible())
        QtTest.QTest.qWait(500)
        self.assertFalse(dialog.isVisible())
        self.assertFalse(dialog.close_timer.isActive())

    def test_heading_zero_does_not_supersede_completed_popup(self):
        dialog = self.start()
        self.complete()
        self.assertEqual(dialog.state, "completed")
        with patch.object(QtWidgets.QMessageBox, "question", return_value=QtWidgets.QMessageBox.Yes):
            self.window.zero_heading()
        self.assertEqual(self.connection.calibration_snapshot()["operation"], 2)
        self.assertEqual(dialog.state, "completed")
        self.assertTrue(dialog.close_timer.isActive())

    def test_timeout_retains_failure_and_manual_retry_resets_request(self):
        dialog = self.start()
        request = dialog.request_id
        self.timeout()
        self.assertEqual(dialog.state, "ack_timeout")
        self.assertIn("结果未知", dialog.details.text())
        self.assertIn("放稳", dialog.hint.text())
        self.assertTrue(dialog.retry_button.isVisible())
        self.assertTrue(dialog.retry_button.isEnabled())
        QtTest.QTest.qWait(1100)
        self.assertTrue(dialog.isVisible())
        self.assertEqual(dialog.request_id, request)  # No automatic resend.
        dialog.retry_button.click()
        self.assertEqual(dialog.state, "queued")
        self.assertNotEqual(dialog.request_id, request)
        self.assertFalse(dialog.retry_button.isVisible())
        self.assertFalse(dialog.close_timer.isActive())

    def test_disconnect_requires_reconnect_and_old_completion_cannot_close_new_attempt(self):
        dialog = self.start()
        old = self.connection
        old.stop_event.set()
        old._commands_disconnected()
        self.window.update_calibration()
        self.assertEqual(dialog.state, "disconnected")
        self.assertFalse(dialog.retry_button.isEnabled())
        self.window.connection = None
        self.window.update_calibration()
        self.assertTrue(dialog.isVisible())
        self.connection = self.make_connection()
        self.window.connection = self.connection
        self.window.update_calibration()
        self.assertTrue(dialog.retry_button.isEnabled())
        dialog.retry_button.click()
        self.assertIs(dialog.connection, self.connection)
        old.recalibration.change("completed")
        self.window.update_calibration()
        self.assertEqual(dialog.state, "queued")
        self.assertFalse(dialog.close_timer.isActive())

    def test_request_exception_is_visible_retryable_and_can_recover(self):
        with patch.object(self.connection, "request_recalibration", side_effect=RuntimeError("synthetic failure")):
            dialog = self.start()
        self.assertEqual(dialog.state, "request_failed")
        self.assertIn("synthetic failure", dialog.details.text())
        self.assertIn("放稳", dialog.hint.text())
        self.assertTrue(dialog.retry_button.isEnabled())
        self.window.update_calibration()
        self.assertEqual(dialog.state, "request_failed")
        dialog.retry_button.click()
        self.assertEqual(dialog.state, "queued")
        self.complete()
        self.assertEqual(dialog.state, "completed")

    def test_waiting_stationary_is_pending_and_late_ack_recovers_timeout(self):
        dialog = self.start()
        self.timeout()
        self.connection.recalibration.accept(ControlReply(dialog.request_id, 0, 1001000, 1), 20)
        self.connection.recalibration.tick(51)
        self.window.update_calibration()
        self.assertEqual(dialog.state, "waiting_stationary")
        self.assertIn("静置", dialog.details.text())
        self.assertFalse(dialog.retry_button.isVisible())
        self.assertFalse(dialog.close_timer.isActive())
        self.connection._receive_usb(1, diagnostic(dialog.request_id, timestamp_us=1002000, sequence=2).encode()+frame(2).encode(), 200, 210)
        self.window.update_calibration()
        self.assertEqual(dialog.state, "completed")

    def test_dismiss_does_not_cancel_and_new_attempt_stops_old_close_timer(self):
        dialog = self.start()
        dialog.close()
        self.assertTrue(self.connection.calibration_snapshot()["busy"])
        self.assertFalse(dialog.isVisible())
        self.complete()
        with patch.object(QtWidgets.QMessageBox, "question", return_value=QtWidgets.QMessageBox.Yes):
            self.window.recalibrate_imu()
        self.assertTrue(dialog.isVisible())
        self.assertEqual(dialog.state, "queued")
        QtTest.QTest.qWait(1100)
        self.assertTrue(dialog.isVisible())
        self.assertFalse(dialog.close_timer.isActive())


if __name__ == "__main__":
    unittest.main()
