"""Continuous connection and independent recording sink regression tests."""
import json
import tempfile
import threading
import time
import unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np
import serial
from imu_viewer.capture import Config, LiveConnection, Recorder
from imu_viewer.protocol import Frame
from imu_viewer.history import WaveformHistory
from imu_viewer.storage import SessionWriter, read_csv
from imu_viewer.verify import verify_session
from test_capture import FakeSerial, FakeCamera


class LiveTests(unittest.TestCase):
    def setUp(self):
        parent = Path(__file__).resolve().parents[1]/"test-results"
        parent.mkdir(exist_ok=True)
        self.temp = tempfile.TemporaryDirectory(dir=parent)
        self.output = Path(self.temp.name)/"not-created-until-recording"
        self.identity = dict(port="FAKE", serial_number="UNIT", vid=0xCAFE, pid=0x4011)
        FakeSerial.instances.clear()
        FakeCamera.instances.clear()
        FakeSerial.failure_after = FakeCamera.failure_after = None
        FakeSerial.period = .01
        FakeCamera.missing_pts = False
        self.patches = [patch("imu_viewer.capture.select_port", return_value=self.identity),
                        patch("imu_viewer.capture.serial.Serial", FakeSerial),
                        patch("imu_viewer.capture.cv2.VideoCapture", FakeCamera)]
        for p in self.patches:
            p.start()
        self.connection = LiveConnection(Config(port="FAKE", width=16, height=8,
                output=str(self.output), min_free_bytes=0))

    def tearDown(self):
        if self.connection.controller:
            self.connection.stop()
            self.assertTrue(self.connection.wait(6))
        elif self.connection.recording:
            self.connection.stop_recording()
            self.assertTrue(self.connection.recording.wait(6))
        for p in reversed(self.patches):
            p.stop()
        self.temp.cleanup()

    def ready(self):
        self.connection.start()
        deadline = time.perf_counter()+3
        while not self.connection.metrics["camera_frames"] and time.perf_counter()<deadline:
            time.sleep(.01)
        self.assertGreater(self.connection.metrics["imu_frames"], 0)
        self.assertGreater(self.connection.metrics["camera_frames"], 0)

    def test_preview_never_creates_files_or_checks_disk(self):
        with patch("imu_viewer.capture.shutil.disk_usage", side_effect=AssertionError("preview touched disk")):
            self.ready()
            time.sleep(.2)
            c = self.connection
            self.assertIsNotNone(c.preview()[0])
            self.assertGreater(len(c.preview()[1]), 0)
            self.assertFalse(self.output.exists())
            self.assertIsNone(c.writer)
            self.assertEqual(c.usb_q.qsize()+c.cam_q.qsize(), 0)
            c.stop()
            self.assertTrue(c.wait(5))
        self.assertTrue(all(s.closed for s in FakeSerial.instances))
        self.assertTrue(all(c.closed for c in FakeCamera.instances))

    def test_repeated_recordings_keep_devices_open_and_indices_independent(self):
        self.ready()
        c = self.connection
        sessions = []
        for _ in range(2):
            sink = c.start_recording()
            sessions.append(sink.path)
            time.sleep(.25)
            c.stop_recording()
            self.assertTrue(sink.wait(5))
            before = c.metrics["camera_frames"]
            time.sleep(.1)
            self.assertGreater(c.metrics["camera_frames"], before)
            self.assertIsNone(sink.error)
            self.assertFalse(c.done.is_set())
            self.assertEqual(sink.writer.samples, sink.metrics["imu_frames"])
            self.assertEqual(sink.writer.images, sink.metrics["camera_frames"])
            verify_session(sink.path, decode_all=True)
        self.assertNotEqual(*sessions)
        self.assertEqual(len(FakeSerial.instances), 1)
        self.assertEqual(len(FakeCamera.instances), 1)
        self.assertFalse(FakeSerial.instances[0].closed)
        self.assertFalse(FakeCamera.instances[0].closed)

    def test_split_packet_at_recording_boundary_remains_reproducible(self):
        c = self.connection
        c.identity = self.identity
        packet = lambda i: Frame(i, 1000000+i*1000,1,2,3,4,5,6,20,10,0).encode()
        now = time.perf_counter_ns()
        image = np.zeros((8,16,3), np.uint8)
        c._receive_usb(0, packet(0)+packet(1)[:17], now, now+10)
        c._receive_camera((0,1.,1000000,now,now+10,image))
        self.assertEqual(len(c.parser.buffer), 17)
        sink = c.start_recording()
        c._receive_usb(1, packet(1)[17:]+packet(2), now+20, now+30)
        c._receive_camera((1,2.,2000000,now+20,now+30,image))
        c._receive_usb(2, packet(3), now+40, now+50)
        c._receive_camera((2,3.,3000000,now+40,now+50,image))
        c.stop_recording()
        self.assertTrue(sink.wait(5))
        self.assertIsNone(sink.error)
        self.assertEqual((sink.path/"usb.bin").read_bytes(), packet(1)[17:]+packet(2)+packet(3))
        self.assertEqual([int(r["sequence"]) for r in read_csv(sink.path/"imu.csv")], [2,3])
        self.assertEqual(sink.parser.stats["bytes_discarded"], 31)
        verify_session(sink.path, decode_all=True)

    def test_recording_write_error_does_not_stop_preview(self):
        self.ready()
        with patch.object(SessionWriter, "usb", side_effect=OSError("injected disk full")):
            sink = self.connection.start_recording()
            self.assertTrue(sink.wait(5))
        self.assertIn("disk full", sink.error)
        before = self.connection.metrics["camera_frames"]
        time.sleep(.15)
        self.assertGreater(self.connection.metrics["camera_frames"], before)
        self.assertIsNone(self.connection.error)
        self.assertTrue((sink.path/"quality.json").exists())

    def test_disconnect_closes_devices_and_finishes_recording(self):
        self.ready()
        sink = self.connection.start_recording()
        time.sleep(.2)
        self.connection.stop()
        self.assertTrue(self.connection.wait(5))
        self.assertTrue(sink.done.is_set())
        self.assertTrue(sink.writer.closed)
        self.assertIsNone(sink.error)
        verify_session(sink.path, decode_all=True)
        self.assertTrue(all(s.closed for s in FakeSerial.instances))
        self.assertTrue(all(c.closed for c in FakeCamera.instances))

    def test_device_failure_stops_sink_with_visible_error(self):
        self.ready()
        sink = self.connection.start_recording()
        FakeSerial.failure_after = FakeSerial.instances[0].calls+10
        self.assertTrue(self.connection.wait(5))
        self.assertIn("USB", self.connection.error)
        self.assertIn("USB", sink.error)
        quality = json.loads((sink.path/"quality.json").read_text(encoding="utf8"))
        self.assertIn("USB", quality["error"])
        self.assertTrue(sink.writer.closed)

    def test_busy_port_fails_connection_without_creating_session(self):
        with patch("imu_viewer.capture.serial.Serial", side_effect=serial.SerialException("port busy")):
            self.connection.start()
            self.assertTrue(self.connection.wait(5))
        self.assertIn("port busy", self.connection.error)
        self.assertFalse(self.output.exists())

    def test_recording_queue_overflow_does_not_stop_preview(self):
        self.connection.config.usb_queue = 1
        self.ready()
        gate = threading.Event()
        original_write = Recorder._write

        def delayed_write(sink):
            gate.wait(5)
            original_write(sink)

        with patch.object(Recorder, "_write", delayed_write):
            sink = self.connection.start_recording()
            try:
                self.assertTrue(sink.stop_event.wait(3))
            finally:
                gate.set()
            self.assertTrue(sink.wait(5))
        self.assertIn("queue overflow", sink.error)
        self.assertGreater(sink.metrics["rejected_usb_batches"], 0)
        before = self.connection.metrics["camera_frames"]
        time.sleep(.15)
        self.assertGreater(self.connection.metrics["camera_frames"], before)
        self.assertIsNone(self.connection.error)
        quality = json.loads((sink.path/"quality.json").read_text(encoding="utf8"))
        self.assertIn("queue overflow", quality["error"])


    def test_history_covers_every_batch_without_gui_and_recording_does_not_reset(self):
        self.ready()
        c = self.connection
        history = c.history
        before = history.info().samples
        sink = c.start_recording()
        time.sleep(.15)
        c.stop_recording()
        self.assertTrue(sink.wait(5))
        time.sleep(.1)
        self.assertIs(c.history, history)
        self.assertGreater(history.info().samples, before)
        c.stop()
        self.assertTrue(c.wait(5))
        count = history.info().samples
        self.assertEqual(count, c.metrics["imu_frames"])
        time.sleep(.03)
        self.assertEqual(history.info().samples, count)
        self.assertIs(c.history, history)
        self.assertGreater(sink.writer.samples, 0)
        self.assertLess(sink.writer.samples, count)

    def test_history_capacity_and_allocation_warning_do_not_stop_raw_recording(self):
        # Force the first summary-chunk allocation over budget. The recent ring
        # remains available and every sink USB byte is still independently parsed.
        self.connection.history = WaveformHistory(max_bytes=128*1024)
        self.ready()
        c = self.connection
        sink = c.start_recording()
        time.sleep(.3)
        self.assertIsNotNone(c.history.info().warning)
        latest = c.history.info().latest_us
        c.stop_recording()
        self.assertTrue(sink.wait(5))
        self.assertIsNone(sink.error)
        self.assertEqual(sink.writer.samples, sink.metrics["imu_frames"])
        self.assertGreater(sink.writer.samples, 100)
        verify_session(sink.path, decode_all=True)
        time.sleep(.1)
        self.assertGreater(c.history.info().latest_us, latest)
        self.assertIsNone(c.error)

    def test_history_memory_error_does_not_stop_raw_recording(self):
        self.ready()
        c = self.connection
        sink = c.start_recording()
        with patch.object(c.history.summary, "append", side_effect=MemoryError("injected")):
            time.sleep(.2)
        self.assertIn("内存分配失败", c.history.info().warning)
        latest = c.history.info().latest_us
        time.sleep(.1)
        self.assertGreater(c.history.info().latest_us, latest)
        c.stop_recording()
        self.assertTrue(sink.wait(5))
        self.assertIsNone(sink.error)
        self.assertIsNone(c.error)
        self.assertEqual(sink.writer.samples, sink.metrics["imu_frames"])
        verify_session(sink.path, decode_all=True)
