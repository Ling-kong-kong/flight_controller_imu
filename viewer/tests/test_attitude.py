"""IMU2 v3 end-to-end regression tests; no devices or physical-axis claims."""
import csv
from dataclasses import replace
import json
import os
from pathlib import Path
import tempfile
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
import numpy as np
from PySide6 import QtGui, QtWidgets
from imu_viewer.attitude import AttitudePanel, AttitudeDetailsDialog
from imu_viewer.attitude_history import AttitudeHistory
from imu_viewer.capture import Config
from imu_viewer.history import WaveformHistory
from imu_viewer.protocol import ATTITUDE_FIELDS, Frame, Parser
from imu_viewer.storage import SessionWriter, finalize, export_euroc, read_csv
from imu_viewer.verify import verify_session


def frame(i=0, **overrides):
    f = Frame(i, 1000000+i*1000, 1, 2, 1365, 4, 5, 6, 20, 10,
              flags=2048, protocol_version=3, qw=1., qx=0., qy=0., qz=0.,
              roll_deg=0., pitch_deg=0., yaw_deg=0.)
    return replace(f, **overrides)


class AttitudeProtocolTests(unittest.TestCase):
    def test_mixed_versions_all_splits_and_single_bytes(self):
        frames = [frame(protocol_version=2), frame(1), frame(2, flags=1024), frame(3)]
        raw = b"".join(f.encode() for f in frames)
        expected = Parser().feed(raw)
        self.assertEqual([f.protocol_version for f in expected], [2, 3, 3, 3])
        for split in range(len(raw)+1):
            parser = Parser()
            self.assertEqual(parser.feed(raw[:split])+parser.feed(raw[split:]), expected)
        parser = Parser()
        result = []
        for byte in raw:
            result.extend(parser.feed(bytes([byte])))
        self.assertEqual(result, expected)
        self.assertEqual(parser.stats["bytes_discarded"], 0)

    def test_resynchronizes_corruption_header_and_truncation(self):
        good = frame(1).encode()
        bad = bytearray(good)
        bad[44] ^= 32
        data = b"noiseIMU2\x03\x03\xff\xff"+bytes(bad)+good[:21]+good
        parser = Parser()
        self.assertEqual(parser.feed(data), [frame(1)])
        self.assertGreaterEqual(parser.stats["crc_errors"], 2)
        self.assertGreaterEqual(parser.stats["header_errors"], 1)
        self.assertEqual(parser.stats["bytes_discarded"], len(data)-76)

    def test_parser_memory_bounded_with_large_feed_and_partial_magic(self):
        parser = Parser()
        self.assertEqual(parser.feed(b"x"*1000000+b"IMU"), [])
        self.assertLess(len(parser.buffer), 4096)
        self.assertEqual(parser.feed(frame().encode()[3:]), [frame()])
        self.assertEqual(parser.feed(frame(1).encode()*15000), [frame(1)]*15000)
        self.assertLess(len(parser.buffer), 4096)

    def test_invalid_values_and_conflicting_flags_never_record_as_attitude(self):
        invalid = [frame(protocol_version=2), frame(flags=0), frame(flags=2048|1024),
                   frame(flags=2048|4096), frame(flags=2048|4), frame(qw=float("nan")),
                   frame(yaw_deg=float("inf")), frame(qw=2.), frame(qz=None),
                   frame(gyro_age_us=1001), frame(accel_age_us=1000001)]
        for f in invalid:
            with self.subTest(f=f):
                self.assertFalse(f.attitude_valid)
                self.assertEqual([f.row()[k] for k in ATTITUDE_FIELDS], [""]*7)
        self.assertTrue(frame().attitude_valid)
        # Bad accelerometer may intentionally use gyro-only propagation.
        self.assertTrue(frame(flags=2048|2|32, accel_age_us=2000).attitude_valid)
        self.assertEqual(frame().row()["attitude_valid"], 1)

    def test_reference_vector_crc_layout(self):
        raw = bytes.fromhex(
            "494d553203034c0078563412f0debc9a785634120080ff7fffff7b0038fe1503"
            "70020000f3010000000841010000803f000000000000000000000000"
            "00000000000000000000b442187ba85c")
        f, = Parser().feed(raw)
        self.assertEqual(len(raw), 76)
        self.assertEqual(f.sequence, 0x12345678)
        self.assertEqual((f.qw, f.qx, f.qy, f.qz, f.yaw_deg), (1, 0, 0, 0, 90))
        self.assertEqual(f.encode(), raw)


class AttitudeHistoryTests(unittest.TestCase):
    def test_capacity_and_liveness(self):
        history = AttitudeHistory()
        history.append_frames([frame(i) for i in range(25000)])
        self.assertEqual(len(history.rows), 10000)
        _, live, rows = history.snapshot(now=history.received_at+.05)
        self.assertTrue(live)
        self.assertEqual(rows.shape, (10000, 4))
        self.assertFalse(history.snapshot(now=history.received_at+.101)[1])
        self.assertFalse(history.snapshot(connected=False, now=history.received_at)[1])

    def test_breaks_calibration_loss_session_regression_and_yaw_wrap(self):
        history = AttitudeHistory()
        history.append_frames([frame(), frame(1, flags=1024), frame(2)])
        self.assertTrue(np.isnan(history.snapshot()[2][1, 1:]).all())
        for f in [frame(4), frame(5, timestamp_us=1020000), frame(6, flags=2048|16),
                  frame(7, timestamp_us=1001000), frame(8, yaw_deg=179), frame(9, yaw_deg=-179)]:
            old = len(history.rows)
            history.append_frames([f])
            # The first 179 degree change need not break, the wrap must.
            if f.sequence != 8:
                self.assertEqual(len(history.rows), old+2)
                self.assertTrue(np.isnan(history.rows[-2][1:]).all())

    def test_raw_quality_independent_of_attitude_flags(self):
        history = WaveformHistory()
        history.append_frames([frame(0, flags=1024|4096), frame(1, flags=2048),
                               frame(2, flags=0), frame(3, flags=4)])
        rows = history.raw.active[:history.raw.used]
        self.assertEqual(rows["valid"].tolist(), [True, True, True, False])


class AttitudeWidgetTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        font = Path("C:/Windows/Fonts/msyh.ttc")
        if font.exists():
            QtGui.QFontDatabase.addApplicationFont(str(font))
            cls.app.setFont(QtGui.QFont("Microsoft YaHei", 9))

    def test_live_valid_calibrating_legacy_disconnect(self):
        panel, history = AttitudePanel(), AttitudeHistory()
        try:
            history.append_frames([frame(roll_deg=12.5, pitch_deg=-7.25, yaw_deg=30)])
            panel.refresh(history, True)
            self.assertIn("12.50", panel.values.text())
            self.assertIn("1.000000", panel.quaternion.text())
            self.assertIn("有效", panel.state.text())
            for f, state in [(frame(1, flags=1024), "校准中"),
                             (frame(2, flags=0), "无效"),
                             (frame(3, protocol_version=2), "旧 v2")]:
                history.append_frames([f])
                panel.refresh(history, True)
                self.assertIn(state, panel.state.text())
                self.assertIn("—", panel.values.text())
                self.assertNotIn("1.000000", panel.quaternion.text())
            panel.refresh(history, False)
            self.assertIn("中断", panel.state.text())
        finally:
            panel.close()

    def test_recording_details_pagination_and_invalid_hiding(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp)
            with (path/"imu.csv").open("w", encoding="utf8", newline="") as handle:
                writer = csv.DictWriter(handle, AttitudeDetailsDialog.FIELDS)
                writer.writeheader()
                for i in range(600):
                    row = {k: "99" for k in ATTITUDE_FIELDS}
                    row.update(sample_id=i, protocol_version=3, attitude_valid=int(i%2 == 0))
                    writer.writerow(row)
            dialog = AttitudeDetailsDialog(path)
            try:
                self.assertEqual(dialog.table.rowCount(), 256)
                self.assertEqual(dialog.table.item(0, 3).text(), "99")
                self.assertEqual(dialog.table.item(1, 3).text(), "—")
                dialog.next_page()
                self.assertEqual(dialog.table.rowCount(), 256)
                self.assertEqual(dialog.table.item(0, 0).text(), "256")
                dialog.next_page()
                self.assertEqual(dialog.table.rowCount(), 88)
                self.assertFalse(dialog.next_button.isEnabled())
            finally:
                dialog.close()
                dialog.handle.close()


class AttitudeStorageTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)

    def tearDown(self):
        self.temp.cleanup()

    def session(self):
        path = self.root/"session"
        writer = SessionWriter(path, Config(port="FAKE", width=8, height=4))
        for i in range(6):
            f = frame(i, protocol_version=2 if i == 0 else 3,
                      flags=16 if i == 0 else 1024 if i == 1 else 2048)
            raw = f.encode()
            decoded = Parser().feed(raw)
            end = 5000000000+i*1000000
            writer.usb(i, raw, end-10000, end, decoded)
            if i == 2:
                writer.image(0, 1000., 1000000000, end-50000, end,
                             np.zeros((4, 8, 3), np.uint8))
        writer.close()
        finalize(path, dict(error=None, timeline_anomaly=False))
        return path

    def test_record_redecode_verify_and_euroc_compatibility(self):
        path = self.session()
        report = verify_session(path, decode_all=True)
        self.assertEqual(report["imu_frames"], 6)
        rows = list(read_csv(path/"imu.csv"))
        for row in rows[:2]:
            self.assertEqual(row["attitude_valid"], "0")
            self.assertEqual([row[k] for k in ATTITUDE_FIELDS], [""]*7)
        self.assertEqual(rows[2]["qw"], "1.0")
        metadata = json.loads((path/"session.json").read_text(encoding="utf8"))
        self.assertEqual(metadata["schema_version"], 3)
        self.assertEqual(metadata["attitude_coordinates"]["body"], "FRD")
        self.assertIn("relative", metadata["attitude_heading"])
        target = self.root/"euroc"
        export_euroc(path, target)
        imu = list(read_csv(target/"mav0/imu0/data.csv"))
        self.assertEqual(len(imu), 6)
        self.assertEqual(len(imu[0]), 7)
        extended = list(read_csv(target/"imu_image.csv"))
        self.assertTrue(set(ATTITUDE_FIELDS).issubset(extended[0]))
        self.assertEqual(extended[0]["qw"], "")
        self.assertEqual(extended[2]["qw"], "1.0")

    def test_modified_attitude_csv_rejected(self):
        path = self.session()
        text = (path/"imu.csv").read_text(encoding="utf8")
        rows = list(read_csv(path/"imu.csv"))
        rows[2]["qw"] = "0.5"
        with (path/"imu.csv").open("w", encoding="utf8", newline="") as handle:
            writer = csv.DictWriter(handle, rows[0].keys())
            writer.writeheader()
            writer.writerows(rows)
        with self.assertRaisesRegex(ValueError, "differs from raw"):
            verify_session(path)
        self.assertNotEqual((path/"imu.csv").read_text(encoding="utf8"), text)

    def test_old_schema_v2_without_new_columns_still_verifies(self):
        path = self.root/"legacy"
        writer = SessionWriter(path, Config(port="FAKE", width=8, height=4))
        f = frame(protocol_version=2, flags=16)
        raw = f.encode()
        writer.usb(0, raw, 4999990000, 5000000000, Parser().feed(raw))
        writer.image(0, 1000., 1000000000, 4999990000, 5000000000,
                     np.zeros((4, 8, 3), np.uint8))
        writer.close()
        finalize(path, dict(error=None, timeline_anomaly=False))
        rows = list(read_csv(path/"imu.csv"))
        fields = [k for k in rows[0] if k not in (*ATTITUDE_FIELDS, "protocol_version", "attitude_valid")]
        with (path/"imu.csv").open("w", encoding="utf8", newline="") as handle:
            writer = csv.DictWriter(handle, fields, extrasaction="ignore")
            writer.writeheader()
            writer.writerows(rows)
        metadata = json.loads((path/"session.json").read_text(encoding="utf8"))
        metadata["schema_version"] = 2
        (path/"session.json").write_text(json.dumps(metadata), encoding="utf8")
        self.assertEqual(verify_session(path)["imu_frames"], 1)
