"""Manual recalibration regressions; no device I/O or physical validation."""
from dataclasses import replace
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
import numpy as np
from PySide6 import QtWidgets
from imu_viewer.capture import Config, LiveConnection
from imu_viewer.gui import Window
from imu_viewer.protocol import ControlReply, Parser, recalibrate_command, ATTITUDE_FIELDS
from imu_viewer.recalibration import Recalibration
from imu_viewer.storage import SessionWriter, finalize, export_euroc, read_csv
from imu_viewer.verify import verify_session
from test_attitude import frame


class LifecycleTests(unittest.TestCase):
    def pending(self):
        state = Recalibration()
        request = state.request()
        self.assertEqual(state.command(10), recalibrate_command(request))
        self.assertIsNone(state.command(11))
        return state, request

    def test_ack_is_execution_not_completion_and_old_backlog_cannot_complete(self):
        state, request = self.pending()
        with self.assertRaises(RuntimeError):
            state.request()
        state.accept(ControlReply(request+1, 0, 1002000, 2), 11)
        self.assertEqual(state.state, "awaiting_ack")
        state.accept(ControlReply(request, 1, 1002000, 2), 11)
        self.assertEqual(state.state, "awaiting_ack")
        state.accept(ControlReply(request, 0, 1002000, 2), 11)
        self.assertEqual(state.state, "calibrating")
        state.accept(frame(1), 12)
        state.accept(frame(2, flags=1024|4096), 12)
        self.assertTrue(state.masked)
        state.accept(frame(3), 12)
        self.assertEqual(state.state, "completed")
        self.assertFalse(state.masked)
        self.assertFalse(state.busy)
        self.assertEqual([e["state"] for e in state.drain_events()],
                         ["queued", "awaiting_ack", "calibrating", "completed"])
        self.assertEqual(state.drain_events(), [])

    def test_timeout_masks_until_late_ack_and_valid_post_boundary_sample(self):
        state, request = self.pending()
        state.tick(13)
        self.assertEqual(state.state, "ack_timeout")
        self.assertTrue(state.masked)
        state.accept(frame(100), 14)
        self.assertEqual(state.state, "ack_timeout")
        self.assertIsNone(state.command(14))  # Never retry an unknown outcome.
        state.accept(ControlReply(request, 0, 1100000, 100), 15)
        state.tick(46)
        self.assertEqual(state.state, "waiting_stationary")
        self.assertTrue(state.busy)
        state.accept(frame(101, flags=1024), 47)
        state.accept(frame(102), 48)
        self.assertEqual(state.state, "completed")

    def test_disconnect_clears_stale_completed_label_and_does_not_duplicate_event(self):
        state, request = self.pending()
        state.accept(ControlReply(request, 0, 1000000, 0), 11)
        state.accept(frame(), 12)
        self.assertEqual(state.state, "completed")
        state.disconnect()
        state.disconnect()
        self.assertEqual(state.state, "disconnected")
        self.assertEqual([e["state"] for e in state.drain_events()].count("disconnected"), 1)

    def test_explicit_new_request_ignores_old_ack(self):
        state, request = self.pending()
        state.tick(14)
        other = state.request()
        self.assertNotEqual(request, other)
        state.command(15)
        state.accept(ControlReply(request, 0, 1000000, 0), 16)
        self.assertEqual(state.state, "awaiting_ack")


class ControlParserTests(unittest.TestCase):
    def test_mixed_records_all_splits_and_single_bytes(self):
        records = [frame(protocol_version=2, **dict.fromkeys(ATTITUDE_FIELDS)), ControlReply(42, 0, 1002000, 2),
                   frame(1), frame(2, flags=1024|4096), frame(3)]
        raw = b"".join(f.encode() for f in records)
        for size in [1, *range(1, len(raw)+1)]:
            parser = Parser()
            found, sensors = [], []
            for start in range(0, len(raw), size):
                sensors += parser.feed(raw[start:start+size])
                found += parser.records
            self.assertEqual(found, records)
            self.assertEqual(sensors, [r for r in records if not isinstance(r, ControlReply)])
            self.assertEqual(parser.stats["control_replies"], 1)
        for split in range(len(raw)+1):
            parser = Parser()
            parser.feed(raw[:split]); found = list(parser.records)
            parser.feed(raw[split:]); found += parser.records
            self.assertEqual(found, records)

    def test_bad_reply_resynchronizes_without_creating_sensor(self):
        ack = ControlReply(42, 0, 1002000, 2)
        corrupt = bytearray(ack.encode()); corrupt[9] ^= 1
        parser = Parser()
        self.assertEqual(parser.feed(b"junkIMU2\x01\x11\xff\xff"+corrupt+ack.encode()), [])
        self.assertEqual(parser.records, [ack])
        self.assertGreater(parser.stats["crc_errors"], 0)
        self.assertGreater(parser.stats["header_errors"], 0)
        self.assertLess(len(parser.buffer), 32)


class ConnectionTests(unittest.TestCase):
    def connection(self):
        connection = LiveConnection(Config(port="FAKE", width=8, height=4))
        connection._receive_usb(0, frame().encode(), 100, 110)
        return connection

    def test_imu_only_request_queued_then_written_once_by_owner(self):
        connection = self.connection()
        self.assertEqual(connection.metrics["camera_frames"], 0)
        port = Mock(); port.write.side_effect = lambda data: len(data)
        request = connection.request_recalibration()
        port.write.assert_not_called()
        connection._service_commands(port)
        port.write.assert_called_once_with(recalibrate_command(request))
        connection._service_commands(port)
        self.assertEqual(port.write.call_count, 1)
        self.assertEqual(connection.calibration_snapshot()["state"], "awaiting_ack")

    def test_idle_never_writes_and_partial_write_fails_without_retry(self):
        connection = self.connection()
        port = Mock(); port.write.return_value = 12
        connection._service_commands(port)
        port.write.assert_not_called()
        connection.request_recalibration()
        with self.assertRaisesRegex(OSError, "outcome unknown"):
            connection._service_commands(port)
        connection._service_commands(port)
        self.assertEqual(port.write.call_count, 1)
        connection._commands_disconnected()
        self.assertEqual(connection.calibration_snapshot()["state"], "disconnected")

    def test_display_mask_does_not_change_raw_history_or_original_frames(self):
        connection = self.connection()
        request = connection.request_recalibration()
        port = Mock(); port.write.side_effect = len
        connection._service_commands(port)
        data = ControlReply(request, 0, 1003000, 3).encode()+frame(1).encode()+frame(2).encode()
        connection._receive_usb(1, data, 200, 210)
        self.assertEqual(connection.calibration_snapshot()["state"], "calibrating")
        self.assertFalse(connection.attitude_history.latest.attitude_valid)
        self.assertTrue(all(connection.history.raw.active[:connection.history.raw.used]["valid"]))
        self.assertTrue(all(f.attitude_valid for f in connection.parser.feed(frame(2).encode())))
        connection._receive_usb(2, frame(3, flags=1024|4096).encode(), 300, 310)
        connection._receive_usb(3, frame(4).encode(), 400, 410)
        self.assertEqual(connection.calibration_snapshot()["state"], "completed")
        self.assertTrue(connection.attitude_history.latest.attitude_valid)
        rows = connection.attitude_history.snapshot()[2]
        self.assertTrue(np.isnan(rows[:, 1]).any())


class CalibrationGuiTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def test_confirmation_default_no_then_yes_immediately_hides_old_values(self):
        with patch.object(Window, "refresh_devices"):
            window = Window()
        window.timer.stop()
        connection = LiveConnection(Config(port="FAKE", width=8, height=4))
        connection._receive_usb(0, frame().encode(), 100, 110)
        window.connection = connection
        try:
            window.update_calibration()
            self.assertTrue(window.recalibrate_button.isEnabled())  # No camera required.
            window.attitude_panel.refresh(connection.attitude_history, True)
            self.assertIn("0.00", window.attitude_panel.values.text())
            with patch.object(QtWidgets.QMessageBox, "question", return_value=QtWidgets.QMessageBox.No) as ask:
                window.recalibrate_imu()
                self.assertEqual(ask.call_args.args[-1], QtWidgets.QMessageBox.No)
            self.assertEqual(connection.calibration_snapshot()["state"], "idle")
            with patch.object(QtWidgets.QMessageBox, "question", return_value=QtWidgets.QMessageBox.Yes):
                window.recalibrate_imu()
            self.assertIn("—", window.attitude_panel.values.text())
            self.assertIn("—", window.attitude_panel.quaternion.text())
            self.assertFalse(window.recalibrate_button.isEnabled())
            connection.stop_event.set()
            connection._commands_disconnected()
            window.update_calibration()
            self.assertIn("中断", window.calibration_status.text())
        finally:
            window.connection = None
            window.close()


class CalibrationStorageTests(unittest.TestCase):
    def test_ack_and_raw_csv_continuity_redecode_export_and_yaw_epoch_dedup(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp)/"session"
            writer = SessionWriter(path, Config(port="FAKE", width=8, height=4))
            ack = ControlReply(42, 0, 1002000, 2)
            # ACK can precede older frames already in the device TX queue.
            records = [frame(), ack, frame(1), frame(2, flags=1024|4096), frame(3), ack, frame(4)]
            raw = b"".join(r.encode() for r in records)
            parser = Parser()
            for i, offset in enumerate(range(0, len(raw), 17)):
                part = raw[offset:offset+17]
                end = 5000000000+i*1000000
                writer.usb(i, part, end-10000, end, parser.feed(part))
            writer.image(0, 1000., 1000000000, 4999990000, 5000000000,
                         np.zeros((4,8,3), np.uint8))
            writer.close()
            finalize(path, dict(error=None, timeline_anomaly=False))
            self.assertEqual((path/"usb.bin").read_bytes(), raw)
            self.assertEqual(verify_session(path, decode_all=True)["imu_frames"], 5)
            rows = list(read_csv(path/"imu.csv"))
            self.assertEqual([r["sequence"] for r in rows], list(map(str, range(5))))
            self.assertEqual([r["timestamp_us"] for r in rows], [str(1000000+i*1000) for i in range(5)])
            self.assertEqual([rows[2][k] for k in ATTITUDE_FIELDS], [""]*7)
            self.assertEqual(rows[1]["attitude_valid"], "1")  # Board truth, not UI mask.
            events = [json.loads(line) for line in (path/"events.jsonl").read_text().splitlines()]
            self.assertEqual([e["kind"] for e in events].count("imu_recalibration_executed"), 1)
            valid = [e for e in events if e["kind"] == "imu_yaw_reference_valid"]
            self.assertEqual(len(valid), 1)
            self.assertEqual(valid[0]["message"]["first_valid_sequence"], 3)
            target = Path(temp)/"euroc"
            export_euroc(path, target)
            self.assertEqual(len(list(read_csv(target/"mav0/imu0/data.csv"))[0]), 7)
            self.assertEqual(len(list(read_csv(target/"imu_image.csv"))), 5)


if __name__ == "__main__":
    unittest.main()
