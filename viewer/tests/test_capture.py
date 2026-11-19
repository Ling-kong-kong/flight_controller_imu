"""Hardware-free lifecycle/fault injection. Never opens a real USB/camera device."""
import errno
import json
from pathlib import Path
import tempfile
import time
import unittest
from unittest.mock import patch
import cv2
import numpy as np
import serial
from imu_viewer.capture import Recorder, Config
from imu_viewer.protocol import Frame
from imu_viewer.storage import SessionWriter, read_csv

class FakeSerial:
    instances = []
    failure_after = None
    period = .01

    def __init__(self, *args, **kwargs):
        self.closed = False
        self.sequence = 0
        self.dtr = False
        self.calls = 0
        self.instances.append(self)

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.closed = True

    @property
    def in_waiting(self):
        return 480

    def read(self, n):
        time.sleep(self.period)
        self.calls += 1
        if self.failure_after is not None and self.calls > self.failure_after:
            raise OSError("injected USB disconnect")
        result = []
        for _ in range(10):
            i = self.sequence
            result.append(Frame(i, 1000000+i*1000, 1,2,3,4,5,6,20,10,
                                16 if i == 0 else 0).encode())
            self.sequence += 1
        return b"".join(result)

    def write(self, data):
        raise AssertionError("Viewer must never transmit commands")

class FakeCamera:
    instances = []
    failure_after = None
    missing_pts = False

    def __init__(self, *args):
        self.closed = False
        self.count = 0
        self.instances.append(self)

    def isOpened(self):
        return True

    def set(self, *args):
        return True

    def getBackendName(self):
        return "FAKE_MSMF"

    def get(self, prop):
        if prop == cv2.CAP_PROP_POS_MSEC:
            return 0.0 if self.missing_pts else self.count * (1000/30)
        return 30.0

    def read(self):
        time.sleep(1/30)
        self.count += 1
        if self.failure_after is not None and self.count > self.failure_after:
            return False, None
        return True, np.zeros((8,16,3), np.uint8)

    def release(self):
        self.closed = True

class CaptureTests(unittest.TestCase):
    def setUp(self):
        parent = Path(__file__).resolve().parents[1]/"test-results"
        parent.mkdir(exist_ok=True)
        self.temp = tempfile.TemporaryDirectory(dir=parent)
        FakeSerial.instances.clear()
        FakeCamera.instances.clear()
        FakeSerial.failure_after = FakeCamera.failure_after = None
        FakeSerial.period = .01
        FakeCamera.missing_pts = False
        self.identity = dict(port="FAKE", serial_number="UNIT", vid=0xCAFE, pid=0x4011)
        self.patches = [
            patch("imu_viewer.capture.select_port", return_value=self.identity),
            patch("imu_viewer.capture.serial.Serial", FakeSerial),
            patch("imu_viewer.capture.cv2.VideoCapture", FakeCamera)]
        for p in self.patches:
            p.start()
        self.recorders = []

    def tearDown(self):
        for r in self.recorders:
            r.stop()
            self.assertTrue(r.wait(5))
        for p in reversed(self.patches):
            p.stop()
        self.temp.cleanup()

    def recorder(self, **kwargs):
        r = Recorder(Config(port="FAKE", width=16, height=8,
                            output=self.temp.name, min_free_bytes=0, **kwargs))
        self.recorders.append(r)
        r.start()
        return r

    def check_closed(self, r):
        self.assertTrue(r.wait(5), "lifecycle did not complete")
        self.assertTrue(all(not t.is_alive() for t in r.threads))
        self.assertFalse(r.writer_thread.is_alive())
        self.assertTrue(all(s.closed for s in FakeSerial.instances))
        self.assertTrue(all(c.closed for c in FakeCamera.instances))
        self.assertTrue(r.writer.closed)
        quality = json.loads((r.path/"quality.json").read_text(encoding="utf8"))
        self.assertEqual(quality["error"], r.error)
        if r.error:
            self.assertEqual(json.loads((r.path/"session.json").read_text(encoding="utf8"))["state"],
                             "stopped_error", (r.path/"failure.json").read_text(encoding="utf8") if (r.path/"failure.json").exists() else str(r.report))
            self.assertIn(r.error, (r.path/"events.jsonl").read_text(encoding="utf8"))

    def test_normal_stop_drains_queues(self):
        r = self.recorder()
        time.sleep(.25)
        r.stop()
        self.check_closed(r)
        self.assertIsNone(r.error)
        self.assertEqual(r.metrics["imu_frames"], r.writer.samples)
        self.assertEqual(r.metrics["camera_frames"], r.writer.images)
        self.assertEqual(r.writer.raw_bytes, (r.path/"usb.bin").stat().st_size)
        self.assertEqual(r.usb_q.qsize()+r.cam_q.qsize(), 0)

    def test_usb_disconnect_preserves_written_data(self):
        FakeSerial.failure_after = 12
        r = self.recorder()
        self.check_closed(r)
        self.assertIn("USB", r.error)
        self.assertGreater(r.writer.samples, 0)
        self.assertTrue(list(read_csv(r.path/"imu.csv")))

    def test_camera_read_failure_releases_devices(self):
        FakeCamera.failure_after = 3
        r = self.recorder()
        self.check_closed(r)
        self.assertIn("Camera read failed", r.error)
        self.assertEqual(r.writer.images, 3)

    def test_repeated_camera_pts_is_diagnostic_not_unified_time(self):
        FakeCamera.missing_pts = True
        r = self.recorder()
        time.sleep(.25)
        r.stop()
        self.check_closed(r)
        self.assertFalse(r.timeline_anomaly)
        self.assertIsNone(r.error)
        self.assertFalse(r.report["models_valid"])
        self.assertTrue(r.report["timestamps_valid"])
        self.assertGreater(r.metrics["camera_media_timestamp_anomalies"], 0)
        self.assertGreater(r.writer.images, 2)

    def test_port_busy_is_visible(self):
        with patch("imu_viewer.capture.serial.Serial",
                   side_effect=serial.SerialException("port already open")):
            r = self.recorder()
            self.check_closed(r)
            self.assertIn("port already open", r.error)

    def test_write_enospc_is_visible(self):
        with patch.object(SessionWriter, "usb", side_effect=OSError(errno.ENOSPC, "injected disk full")):
            r = self.recorder()
            self.check_closed(r)
            self.assertIn("Writer", r.error)
            self.assertIn("disk full", r.error)

    def test_image_write_failure_retains_explicit_incomplete_count(self):
        with patch.object(SessionWriter, "save_image",
                          side_effect=OSError("injected PNG failure")):
            r = self.recorder()
            self.check_closed(r)
            self.assertIn("PNG failure", r.error)
            self.assertEqual(r.writer.images, 0)

    def test_queue_overflow_is_not_silent(self):
        original = SessionWriter.usb
        def slow(writer, *args):
            time.sleep(.04)
            return original(writer, *args)
        with patch.object(SessionWriter, "usb", slow):
            r = self.recorder(usb_queue=1)
            self.check_closed(r)
            self.assertIn("queue overflow", r.error)
            self.assertEqual(r.metrics["rejected_usb_batches"], 1)
            self.assertEqual(r.usb_q.qsize(), 0)

    def test_camera_queue_overflow_explicit(self):
        original = SessionWriter.save_image
        def slow(writer, *args):
            time.sleep(.3)
            return original(writer, *args)
        with patch.object(SessionWriter, "save_image", slow):
            r = self.recorder(camera_queue=1, image_workers=1, image_inflight=1)
            self.check_closed(r)
            self.assertIn("queue overflow", r.error)
            self.assertEqual(r.metrics["rejected_camera_frames"], 1)

    def test_disk_reserve_stops_after_start(self):
        # The real disk is not filled; inject free-space report only.
        from collections import namedtuple
        Usage = namedtuple("Usage", "total used free")
        calls = [0]
        def disk(path):
            calls[0] += 1
            return Usage(10**12,0,10**12 if calls[0] == 1 else 0)
        with patch("imu_viewer.capture.shutil.disk_usage", side_effect=disk):
            r = Recorder(Config(port="FAKE",width=16,height=8,
                                output=self.temp.name,min_free_bytes=1))
            self.recorders.append(r)
            r.start()
            self.check_closed(r)
            self.assertIn("safety reserve", r.error)

    def test_startup_no_camera_frames_is_an_error(self):
        def delayed_read(cap):
            time.sleep(.6)
            return True, np.zeros((8,16,3),np.uint8)
        with patch.object(FakeCamera, "read", delayed_read):
            r = self.recorder(camera_start_timeout_s=.05)
            self.check_closed(r)
            self.assertIn("startup timeout", r.error)

    def test_initial_low_disk_does_not_open_devices(self):
        from collections import namedtuple
        Usage = namedtuple("Usage", "total used free")
        with patch("imu_viewer.capture.shutil.disk_usage", return_value=Usage(1,1,0)):
            r = Recorder(Config(port="FAKE",output=self.temp.name,min_free_bytes=1))
            with self.assertRaisesRegex(OSError, "Insufficient"):
                r.start()
        self.assertEqual(FakeSerial.instances, [])
        self.assertEqual(FakeCamera.instances, [])

if __name__ == "__main__":
    unittest.main()
