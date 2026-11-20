"""Single-owner device producers, bounded writer queues, and explicit failure lifecycle."""
from collections import deque
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass, asdict, replace
from datetime import datetime
import json
from pathlib import Path
import queue
import shutil
import subprocess
import threading
import time
import traceback
import cv2
import psutil
import serial
from serial.tools import list_ports
from .clock import fit_clock
from .timing import HOST_WARNING as WARNING, TIME_BASIS
from .protocol import Parser, Continuity, Frame
from .storage import SessionWriter, finalize, json_write

BASE = Path(__file__).resolve().parents[1]

@dataclass
class Config:
    port: str = ""
    serial_number: str | None = None
    camera_index: int = 0
    backend: str = "MSMF"
    width: int = 2560
    height: int = 720
    fps: float = 30.0
    fourcc: str = "MJPG"
    output: str = str(BASE / "recordings")
    camera_queue: int = 64
    usb_queue: int = 1024
    image_workers: int = 8
    image_inflight: int = 32
    camera_start_timeout_s: float = 15.0
    min_free_bytes: int = 1024 * 1024 * 1024
    camera_inventory: list | None = None

    def validate(self):
        if not self.port:
            raise ValueError("Select an H743 CDC port")
        if self.width < 2 or self.width % 2 or self.height < 1:
            raise ValueError("Stereo width must be positive and even")
        if self.backend not in ("MSMF", "DSHOW", "ANY"):
            raise ValueError("Unknown camera backend")
        if self.fps <= 0 or self.camera_index < 0:
            raise ValueError("Invalid frame rate or camera index")
        if len(self.fourcc) != 4 or self.camera_queue < 1 or self.usb_queue < 1:
            raise ValueError("Invalid FOURCC or queue capacity")
        if not 1 <= self.image_workers <= 16 or not 1 <= self.image_inflight <= 64:
            raise ValueError("Invalid bounded image pool settings")
        if self.camera_start_timeout_s <= 0:
            raise ValueError("Invalid camera startup timeout")
        if self.min_free_bytes < 0:
            raise ValueError("Invalid disk reserve")

def discover_ports():
    return [dict(port=p.device, description=p.description, vid=p.vid, pid=p.pid,
                 serial_number=p.serial_number, h743=p.vid == 0xCAFE and p.pid == 0x4011)
            for p in list_ports.comports()]

def camera_inventory():
    """PnP identity inventory only; OpenCV index mapping is not asserted."""
    if __import__("os").name != "nt":
        return []
    try:
        result = subprocess.run(
            ["powershell", "-NoProfile", "-NonInteractive", "-Command",
             "Get-PnpDevice -PresentOnly | Where-Object {"
             "$_.Class -in @('Camera','Image') -and $_.Status -eq 'OK'} | "
             "Select-Object FriendlyName,InstanceId,Status | ConvertTo-Json -Compress"],
            capture_output=True, text=True, timeout=15,
            creationflags=subprocess.CREATE_NO_WINDOW)
        if result.returncode or not result.stdout.strip():
            return []
        data = json.loads(result.stdout)
        return data if isinstance(data, list) else [data]
    except (OSError, ValueError, subprocess.TimeoutExpired):
        return []

def select_port(port=None, serial_number=None):
    ports = discover_ports()
    if port:
        # Explicit selection must still be the existing H743 USB identity.
        matches = [p for p in ports if p["port"].lower() == port.lower()]
        if not matches:
            raise ValueError(f"Serial device not present: {port}")
        if not matches[0]["h743"]:
            raise ValueError("Selected device is not H743 CAFE:4011")
        if serial_number and matches[0]["serial_number"] != serial_number:
            raise ValueError("Serial number does not match selected port")
        return matches[0]
    matches = [p for p in ports if p["h743"] and
               (serial_number is None or p["serial_number"] == serial_number)]
    if len(matches) != 1:
        raise ValueError("Select exactly one CAFE:4011 device (or supply --port)")
    return matches[0]

class Recorder:
    def __init__(self, config):
        config.validate()
        self.config = config
        self.stop_event = threading.Event()
        self.producers_done = threading.Event()
        self.done = threading.Event()
        self.accepting = True
        self.usb_q = queue.Queue(config.usb_queue)
        self.cam_q = queue.Queue(config.camera_queue)
        self.lock = threading.RLock()
        self.input_lock = self.lock
        self.error = None
        self.timeline_anomaly = False
        self.parser = Parser()
        self.continuity = Continuity()
        self.latest_image = None
        self.plot_samples = deque(maxlen=2000)
        self.imu_obs = deque(maxlen=10000)
        self.cam_obs = deque(maxlen=4000)
        self.metrics = dict(usb_batches=0, usb_bytes=0, imu_frames=0, camera_frames=0,
                            usb_queue_peak=0, camera_queue_peak=0, rss_peak_bytes=0,
                            rejected_usb_batches=0, rejected_camera_frames=0,
                            written_imu=0, written_images=0, free_bytes=0,
                            maximum_write_iteration_ms=0.0, camera_gap_max_ms=0.0,
                            image_inflight=0, image_inflight_peak=0, camera_media_timestamp_anomalies=0)
        self.start_ns = time.perf_counter_ns()
        self.end_ns = None
        self.actual_camera = {}
        self.report = None
        self.writer = None
        self.threads = []
        self.controller = None
        self.path = None
        self.telemetry = deque(maxlen=36000)  # up to 10 h at 1 Hz, bounded
        self._live_clock_cache = {}
        self.camera_last_ns = None
        self.usb_last_ns = None
        self.camera_previous_media_ns = None
        self.imu_span = [None, None, 0]
        self.camera_span = [None, None, 0]

    def fail(self, message, timeline=False):
        with self.lock:
            if self.error is None:
                self.error = message
            self.timeline_anomaly |= timeline
        self.stop_event.set()

    def enqueue(self, q, item):
        if not self.accepting:
            return False
        try:
            q.put_nowait(item)
            return True
        except queue.Full:
            with self.lock:
                self.metrics["rejected_usb_batches" if q is self.usb_q else "rejected_camera_frames"] += 1
            self.fail("Bounded recording queue overflow; current item was NOT saved")
            return False

    def _prepare_writer(self, identity):
        self.config.serial_number = identity["serial_number"]
        output = Path(self.config.output).resolve()
        output.mkdir(parents=True, exist_ok=True)
        if shutil.disk_usage(output).free < self.config.min_free_bytes:
            raise OSError("Insufficient free disk space before recording")
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        container = output / f"session_{stamp}"
        container.mkdir(exist_ok=False)  # Reserve the whole session; never mix recordings.
        self.path = container / "raw"
        self.writer = SessionWriter(self.path, self.config)
        try:
            json_write(self.path / "devices.json", dict(usb=identity,
                       camera_inventory=self.config.camera_inventory,
                       opencv_version=cv2.__version__, camera_index_mapping_verified=False,
                       msmf_hardware_transforms=__import__("os").environ.get("OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS")))
        except BaseException:
            self.writer.close()
            raise
        self.start_ns = time.perf_counter_ns()
        self.writer_thread = threading.Thread(target=self._write, name="session-writer", daemon=True)

    def start(self):
        identity = select_port(self.config.port, self.config.serial_number)
        self._prepare_writer(identity)
        self.threads = [threading.Thread(target=self._usb, name="usb-cdc", daemon=True),
                        threading.Thread(target=self._camera, name="stereo-camera", daemon=True)]
        self.writer_thread.start()
        for thread in self.threads:
            thread.start()
        self.controller = threading.Thread(target=self._supervise, name="recorder-lifecycle", daemon=True)
        self.controller.start()
        return self.path

    def stop(self):
        self.stop_event.set()

    def wait(self, timeout=None):
        return self.done.wait(timeout)

    def _usb(self):
        try:
            # This thread is the only serial owner. No input flush or MCU reset.
            with serial.Serial(self.config.port, baudrate=115200, timeout=0.2,
                               write_timeout=0.2, rtscts=False, dsrdtr=False) as port:
                port.dtr = True
                last_data = time.perf_counter()
                batch_id = 0
                while not self.stop_event.is_set():
                    self._service_commands(port)
                    start = time.perf_counter_ns()
                    raw = port.read(max(1, min(port.in_waiting, 4096)))
                    end = time.perf_counter_ns()
                    if not raw:
                        if time.perf_counter() - last_data > 3:
                            raise TimeoutError("No USB data for 3 seconds")
                        continue
                    last_data = time.perf_counter()
                    if not self._receive_usb(batch_id, raw, start, end):
                        break
                    batch_id += 1
        except Exception as exc:
            self.fail(f"USB: {type(exc).__name__}: {exc}")
        finally:
            self._commands_disconnected()

    def _service_commands(self, port):
        pass

    def _commands_disconnected(self):
        pass

    def _receive_usb(self, batch_id, raw, start, end):
        if start < 0 or end < start or (self.usb_last_ns is not None and start < self.usb_last_ns):
            self.fail("Invalid USB host read interval", timeline=True)
            return False
        self.usb_last_ns = end
        frames = self.parser.feed(raw)
        fatal = None
        for frame in frames:
            anomaly = self.continuity.accept(frame)
            fatal = fatal or anomaly
        if not self.enqueue(self.usb_q, (batch_id, raw, start, end, frames)):
            return False
        with self.lock:
            self.metrics["usb_batches"] += 1
            self.metrics["usb_bytes"] += len(raw)
            self.metrics["imu_frames"] += len(frames)
            self.metrics["usb_queue_peak"] = max(self.metrics["usb_queue_peak"], self.usb_q.qsize())
            if frames:
                if self.imu_span[0] is None:
                    self.imu_span[0] = end
                self.imu_span[1] = end
                self.imu_span[2] += len(frames)
                self.imu_obs.append((frames[-1].timestamp_us * 1000, end))
                self.plot_samples.extend((f.timestamp_us, f.ax_raw * 24/32768,
                    f.ay_raw * 24/32768, f.az_raw * 24/32768,
                    f.gx_raw * 2000/32768, f.gy_raw * 2000/32768,
                    f.gz_raw * 2000/32768) for f in frames)
        self._waveform_frames(frames)
        if fatal:
            self.fail(fatal, timeline=True)
        return True

    def _waveform_frames(self, frames):
        # Recording sinks / CLI do not allocate connection-wide visualization history.
        pass

    def _camera(self):
        cap = None
        try:
            backend = dict(MSMF=cv2.CAP_MSMF, DSHOW=cv2.CAP_DSHOW, ANY=cv2.CAP_ANY)[self.config.backend]
            cap = cv2.VideoCapture(self.config.camera_index, backend)
            if not cap.isOpened():
                raise OSError("Cannot open camera")
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.config.fourcc))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.config.width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.config.height)
            cap.set(cv2.CAP_PROP_FPS, self.config.fps)
            self.actual_camera = dict(backend=cap.getBackendName(),
                reported_width=cap.get(cv2.CAP_PROP_FRAME_WIDTH),
                reported_height=cap.get(cv2.CAP_PROP_FRAME_HEIGHT),
                reported_fps=cap.get(cv2.CAP_PROP_FPS),
                reported_fourcc=int(cap.get(cv2.CAP_PROP_FOURCC)),
                rate_is_measured_separately=True)
            frame_id = 0
            while not self.stop_event.is_set():
                start = time.perf_counter_ns()
                ok, image = cap.read()
                end = time.perf_counter_ns()
                if not ok or image is None:
                    raise OSError("Camera read failed or disconnected")
                media_ms = float(cap.get(cv2.CAP_PROP_POS_MSEC))
                import math
                media_ns = round(media_ms * 1e6) if math.isfinite(media_ms) and media_ms >= 0 else None
                h, w = image.shape[:2]
                self.actual_camera["measured_frame_shape"] = [h, w, image.shape[2] if image.ndim == 3 else 1]
                if (w, h) != (self.config.width, self.config.height) or w % 2:
                    raise OSError(f"Actual frame {w}x{h} differs from requested stereo geometry")
                # Media PTS is raw metadata, not the unified delivery-time source.
                item = (frame_id, media_ms if math.isfinite(media_ms) else "",
                        media_ns, start, end, image)
                if not self._receive_camera(item):
                    break
                frame_id += 1
        except Exception as exc:
            self.fail(f"Camera: {type(exc).__name__}: {exc}")
        finally:
            if cap is not None:
                cap.release()

    def _receive_camera(self, item):
        frame_id, media_ms, media_ns, start, end, image = item
        if start < 0 or end < start or (self.camera_last_ns is not None and start < self.camera_last_ns):
            self.fail("Invalid camera host read interval", timeline=True)
            return False
        if not self.enqueue(self.cam_q, item):
            return False
        with self.lock:
            if media_ns is None or (self.camera_previous_media_ns is not None
                                    and media_ns <= self.camera_previous_media_ns):
                self.metrics["camera_media_timestamp_anomalies"] += 1
            self.camera_previous_media_ns = media_ns
            self.latest_image = image
            previous_end = self.camera_last_ns
            self.camera_last_ns = end
            if self.camera_span[0] is None:
                self.camera_span[0] = end
            self.camera_span[1] = end
            self.camera_span[2] += 1
            self.metrics["camera_frames"] += 1
            self.metrics["camera_queue_peak"] = max(self.metrics["camera_queue_peak"], self.cam_q.qsize())
            if previous_end:
                self.metrics["camera_gap_max_ms"] = max(
                    self.metrics["camera_gap_max_ms"], (end - previous_end) / 1e6)
            if media_ns is not None:
                self.cam_obs.append((media_ns, end))
        return True

    def _write(self):
        last_flush = last_disk = time.perf_counter()
        pending = deque()
        pool = ThreadPoolExecutor(max_workers=self.config.image_workers,
                                  thread_name_prefix="png-writer")
        try:
            while (not self.producers_done.is_set() or not self.usb_q.empty()
                   or not self.cam_q.empty() or pending):
                begin = time.perf_counter()
                work = False
                # CSV/raw writes never wait on PNG encoding. Futures are explicitly bounded.
                for _ in range(256):
                    try:
                        item = self.usb_q.get_nowait()
                    except queue.Empty:
                        break
                    if item[0] == "event":
                        self.writer.event(item[1], item[2])
                    else:
                        self.writer.usb(*item)
                    work = True
                while pending and pending[0].done():
                    self.writer.commit_image(pending.popleft().result())
                    work = True
                while len(pending) < self.config.image_inflight:
                    try:
                        item = self.cam_q.get_nowait()
                    except queue.Empty:
                        break
                    pending.append(pool.submit(self.writer.save_image, *item))
                    work = True
                now = time.perf_counter()
                if now - last_flush >= 1:
                    self.writer.flush()
                    last_flush = now
                if now - last_disk >= 1:
                    free = shutil.disk_usage(self.path).free
                    rss = psutil.Process().memory_info().rss
                    with self.lock:
                        self.metrics["free_bytes"] = free
                        self.metrics["rss_peak_bytes"] = max(self.metrics["rss_peak_bytes"], rss)
                        self.telemetry.append(dict(elapsed_s=(time.perf_counter_ns()-self.start_ns)/1e9,
                            rss_bytes=rss, free_bytes=free, usb_queue=self.usb_q.qsize(),
                            camera_queue=self.cam_q.qsize(), image_inflight=len(pending),
                            written_imu=self.writer.samples, written_images=self.writer.images))
                    if free < self.config.min_free_bytes:
                        self.fail("Disk space below safety reserve; recording stopped")
                    last_disk = now
                with self.lock:
                    self.metrics["written_imu"] = self.writer.samples
                    self.metrics["written_images"] = self.writer.images
                    self.metrics["image_inflight"] = len(pending)
                    self.metrics["image_inflight_peak"] = max(self.metrics["image_inflight_peak"], len(pending))
                    self.metrics["maximum_write_iteration_ms"] = max(
                        self.metrics["maximum_write_iteration_ms"], (now - begin) * 1000)
                if not work:
                    # Event.wait becomes a busy spin once producers stop: sleep while draining.
                    time.sleep(0.001)
        except Exception as exc:
            self.fail(f"Writer: {type(exc).__name__}: {exc}; queued data may be incomplete")
        finally:
            # Complete existing unique PNG writes before closing indexes or finalizing.
            pool.shutdown(wait=True, cancel_futures=True)
            with self.lock:
                self.metrics["uncommitted_image_jobs"] = len(pending)
            try:
                if self.error:
                    self.writer.event("error", self.error)
                self.writer.close()
            except Exception as exc:
                self.fail(f"Closing session: {exc}")

    def _supervise(self):
        try:
            while not self.stop_event.wait(0.25):
                now_ns = time.perf_counter_ns()
                last = self.camera_last_ns
                if last is None and (now_ns-self.start_ns)/1e9 > self.config.camera_start_timeout_s:
                    self.fail("Camera produced no frames before startup timeout")
                elif last is not None and now_ns-last > 3_000_000_000:
                    self.fail("Camera produced no frames for 3 seconds; backend stalled")
            for thread in self.threads:
                thread.join(timeout=10)
                if thread.is_alive():
                    self.fail(f"{thread.name} did not close within 10 s; backend stalled")
            with self.input_lock:
                self.accepting = False
                self.producers_done.set()
            self.writer_thread.join(timeout=60)
            if self.writer_thread.is_alive():
                self.fail("Writer did not finish within 60 s; raw session remains incomplete")
                return
            self.end_ns = time.perf_counter_ns()
            if not self.metrics["camera_frames"]:
                self.fail("No camera frames captured; joint recording is incomplete")
            if not self.metrics["imu_frames"]:
                self.fail("No valid IMU frames captured; joint recording is incomplete")
            quality = self.snapshot()
            quality.update(parser=dict(self.parser.stats, trailing_bytes=len(self.parser.buffer)),
                           continuity=self.continuity.stats, timeline_anomaly=self.timeline_anomaly,
                           actual_camera=self.actual_camera, telemetry=list(self.telemetry),
                           unsaved_usb_batches=self.usb_q.qsize(), unsaved_camera_frames=self.cam_q.qsize())
            # Written first: even finalization errors must leave an explicit failure record.
            json_write(self.path / "quality.json", quality)
            self.report = finalize(self.path, quality)
        except Exception as exc:
            self.fail(f"Finalization: {type(exc).__name__}: {exc}")
            try:
                json_write(self.path / "failure.json", dict(error=self.error, traceback=traceback.format_exc()))
            except OSError:
                pass
        finally:
            self.done.set()

    def snapshot(self, live_clocks=False):
        with self.lock:
            data = dict(self.metrics, error=self.error, path=str(self.path),
                        elapsed_s=((self.end_ns or time.perf_counter_ns()) - self.start_ns) / 1e9,
                        usb_queue=self.usb_q.qsize(), camera_queue=self.cam_q.qsize(),
                        crc_errors=self.parser.stats["crc_errors"],
                        missing_sequences=self.continuity.stats["missing_sequences"],
                        flags=self.continuity.stats["flag_counts"].copy(),
                        done=self.done.is_set(), warning=WARNING)
            imu, cam = list(self.imu_obs), list(self.cam_obs)
            spans = dict(imu=self.imu_span.copy(), camera=self.camera_span.copy())
        data["imu_host_hz"] = ((len(imu)-1)*1e9/(imu[-1][1]-imu[0][1])
                               if len(imu)>1 and imu[-1][1]>imu[0][1] else None)
        # USB above measures batches, not samples; explicitly label and report true span rates separately.
        data["usb_batch_hz"] = data.pop("imu_host_hz")
        data["camera_host_hz"] = ((len(cam)-1)*1e9/(cam[-1][1]-cam[0][1])
                                  if len(cam)>1 and cam[-1][1]>cam[0][1] else None)
        for name, (first, last, count) in spans.items():
            data[name + "_full_span_host_hz"] = ((count-1)*1e9/(last-first)
                if count > 1 and last > first else None)
            data[name + "_host_span_s"] = (last-first)/1e9 if count else 0
        data["imu_average_host_hz"] = data["imu_full_span_host_hz"]
        data["camera_host_hz"] = data["camera_full_span_host_hz"]
        data["timestamp_basis"] = TIME_BASIS
        data["timestamp_source"] = "host_read_end_ns"
        data["clock_fitting_used_for_output"] = False
        if live_clocks:
            for name, obs in (("imu", imu), ("camera", cam)):
                try:
                    data[name + "_live_clock"] = fit_clock(obs).report()
                except ValueError as exc:
                    data[name + "_live_clock"] = dict(error=str(exc))
        return data

    def preview(self):
        with self.lock:
            return self.latest_image, list(self.plot_samples)


class LiveConnection(Recorder):
    """One device connection with an optional independently drained sink.

    Preview retains only bounded in-memory histories. Attaching/detaching a recording
    never reopens devices, resets their clocks, flushes USB, or sends commands.
    """
    def __init__(self, config):
        super().__init__(config)
        from .history import WaveformHistory
        self.history = WaveformHistory()
        from .attitude_history import AttitudeHistory
        self.attitude_history = AttitudeHistory()
        self.routing_lock = threading.RLock()
        self.recording = None
        self.identity = None
        from .recalibration import Recalibration
        self.recalibration = Recalibration()

    def _waveform_frames(self, frames):
        self.history.append_frames(frames)
        display = []
        for record in self.parser.records:
            self.recalibration.accept(record)
            if isinstance(record, Frame):
                display.append(replace(record, flags=record.flags & ~2048)
                               if self.recalibration.masked else record)
        self.attitude_history.append_frames(display)
        self._calibration_events()

    def _calibration_events(self):
        sink = self.recording
        for event in self.recalibration.drain_events():
            if sink is not None and sink.accepting and not sink.stop_event.is_set():
                sink.enqueue(sink.usb_q, ("event", "imu_recalibration", event))

    def request_recalibration(self):
        with self.routing_lock:
            if self.stop_event.is_set() or not self.metrics["imu_frames"]:
                raise RuntimeError("Wait for live IMU data")
            request_id = self.recalibration.request()
            self._calibration_events()
            return request_id

    def calibration_snapshot(self):
        with self.routing_lock:
            return self.recalibration.snapshot()

    def _service_commands(self, port):
        with self.routing_lock:
            self.recalibration.tick()
            command = self.recalibration.command()
            self._calibration_events()
        if command is not None and port.write(command) != len(command):
            raise OSError("Incomplete IMU recalibration command; outcome unknown")

    def _commands_disconnected(self):
        with self.routing_lock:
            self.recalibration.disconnect()
            self._calibration_events()

    def start(self):
        self.identity = select_port(self.config.port, self.config.serial_number)
        self.config.serial_number = self.identity["serial_number"]
        self.start_ns = time.perf_counter_ns()
        self.threads = [threading.Thread(target=self._usb, name="usb-cdc", daemon=True),
                        threading.Thread(target=self._camera, name="stereo-camera", daemon=True)]
        for thread in self.threads:
            thread.start()
        self.controller = threading.Thread(target=self._supervise, name="preview-lifecycle", daemon=True)
        self.controller.start()

    def enqueue(self, q, item):
        # Device loops still perform identical validation and preview metrics, but
        # preview itself has no disk queues. Only an attached sink queues data.
        return self.accepting

    def start_recording(self, output=None):
        with self.routing_lock:
            if self.stop_event.is_set() or self.done.is_set():
                raise RuntimeError("Device connection is closed or stopping")
            if self.recording is not None and not self.recording.done.is_set():
                raise RuntimeError("Previous recording is still active or finalizing")
            if not self.metrics["camera_frames"] or not self.metrics["imu_frames"]:
                raise RuntimeError("Wait for live camera and IMU data before recording")
            sink = Recorder(replace(self.config, output=output or self.config.output))
            sink._prepare_writer(self.identity)
            sink.actual_camera = self.actual_camera.copy()
            sink.input_lock = self.routing_lock
            # No device-owning threads: incoming batches belong only to this sink.
            sink.writer_thread.start()
            sink.controller = threading.Thread(target=sink._supervise,
                                               name="recording-sink-lifecycle", daemon=True)
            sink.controller.start()
            self.recording = sink
            sink.enqueue(sink.usb_q, ("event", "imu_recalibration_context",
                                     self.recalibration.snapshot()))
            return sink

    def stop_recording(self):
        with self.routing_lock:
            sink = self.recording
            if sink is not None:
                sink.accepting = False
                sink.stop()
            return sink

    def _receive_usb(self, batch_id, raw, start, end):
        with self.routing_lock:
            accepted = super()._receive_usb(batch_id, raw, start, end)
            sink = self.recording
            if accepted and sink is not None and not sink.stop_event.is_set():
                # Reparse only recorded raw bytes. A recording may begin halfway
                # through a packet; its prefix is discarded/accounted by Parser,
                # not borrowed from the live parser with a forged arrival time.
                sink._receive_usb(sink.metrics["usb_batches"], raw, start, end)
            return accepted

    def _receive_camera(self, item):
        with self.routing_lock:
            accepted = super()._receive_camera(item)
            sink = self.recording
            if accepted and sink is not None and not sink.stop_event.is_set():
                sink._receive_camera((sink.metrics["camera_frames"], *item[1:]))
            return accepted

    def _supervise(self):
        try:
            while not self.stop_event.wait(.25):
                now = time.perf_counter_ns()
                last = self.camera_last_ns
                if last is None and (now-self.start_ns)/1e9 > self.config.camera_start_timeout_s:
                    self.fail("Camera produced no frames before startup timeout")
                elif last is not None and now-last > 3_000_000_000:
                    self.fail("Camera produced no frames for 3 seconds; backend stalled")
            for thread in self.threads:
                thread.join(timeout=10)
                if thread.is_alive():
                    self.fail(f"{thread.name} did not close within 10 s; backend stalled")
            with self.routing_lock:
                self.accepting = False
                sink = self.recording
                if sink is not None and not sink.done.is_set():
                    if self.error:
                        sink.fail(self.error, timeline=self.timeline_anomaly)
                    self.stop_recording()
            if sink is not None:
                sink.wait()  # Keep connection pending until the sink is safely closed.
            self.end_ns = time.perf_counter_ns()
        finally:
            self.done.set()
