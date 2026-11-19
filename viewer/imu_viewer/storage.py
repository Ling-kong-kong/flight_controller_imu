"""Append-only raw session storage and reproducible finalization/export."""
import csv
from dataclasses import asdict
import json
import os
from pathlib import Path
import shutil
import time
import cv2
from .protocol import Frame
from .clock import WARNING, fit_clock, align_camera
from .timing import HOST_WARNING, TIME_BASIS, iter_host_imu, match_images, image_filename

IMU_FIELDS = ["sample_id", "batch_id", "unified_timestamp_ns"] + list(Frame(0, 0, 0, 0, 0, 0, 0, 0, 0, 0).row())
BATCH_FIELDS = ["batch_id", "raw_offset", "byte_count", "host_read_start_ns",
                "host_read_end_ns", "frame_count", "last_mcu_ns"]
CAM_FIELDS = ["frame_id", "camera_media_ms", "camera_media_ns", "host_read_start_ns",
              "host_read_end_ns", "unified_timestamp_ns", "width", "height", "cam0", "cam1"]
ALIGN_FIELDS = ["frame_id", "camera_media_ns", "estimated_mcu_ns", "status"]

def json_write(path, data):
    path = Path(path)
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=2, allow_nan=False)
        f.flush()
        os.fsync(f.fileno())
    temporary.replace(path)

def read_csv(path):
    with Path(path).open(newline="", encoding="utf-8") as f:
        yield from csv.DictReader(f)

def write_png(path, pixels):
    # imencode/tofile supports non-ASCII Windows paths.
    ok, data = cv2.imencode(".png", pixels, [cv2.IMWRITE_PNG_COMPRESSION, 1,
        cv2.IMWRITE_PNG_STRATEGY, cv2.IMWRITE_PNG_STRATEGY_RLE])
    if not ok:
        raise OSError("PNG encoding failed")
    with Path(path).open("xb") as f:
        f.write(data.tobytes())

class SessionWriter:
    def __init__(self, path, config):
        self.path = Path(path)
        self.path.mkdir(parents=True, exist_ok=False)
        self.handles = []
        self.samples = self.batches = self.images = self.raw_bytes = 0
        self.closed = False
        try:
            for name in ("cam0", "cam1"):
                (self.path / name).mkdir()
            json_write(self.path / "session.json", dict(
                schema_version=2, config=asdict(config), state="recording",
                timestamp_basis=TIME_BASIS, unified_timestamp_source="host_read_end_ns",
                warning=HOST_WARNING, camera_clock_domain="unverified backend media clock",
                camera_media_timestamp_semantics="not verified to be exposure time",
                stereo_exposure_verified=False, calibration=None,
                wall_time_used_for_clock_alignment=False,
                imu_timestamp_ns_conversion="integer microseconds * 1000",
                usb_observation="all decoded frames share the completing USB read batch end time"))
            self.raw = self._open("usb.bin", "wb")
            self.imu = self._csv("imu.csv", IMU_FIELDS)
            self.batch = self._csv("usb_batches.csv", BATCH_FIELDS)
            self.camera = self._csv("frames.csv", CAM_FIELDS)
            self.events = self._open("events.jsonl", "w")
        except BaseException:
            self.close()
            raise

    def _open(self, name, mode):
        f = (self.path / name).open(mode, **({} if "b" in mode else
                                            dict(encoding="utf-8", newline="")))
        self.handles.append(f)
        return f

    def _csv(self, name, fields):
        writer = csv.DictWriter(self._open(name, "w"), fields)
        writer.writeheader()
        return writer

    def event(self, kind, message):
        self.events.write(json.dumps(dict(host_ns=time.perf_counter_ns(), kind=kind,
                                         message=message), ensure_ascii=False) + "\n")
        self.events.flush()

    def usb(self, batch_id, raw, start, end, frames):
        self.raw.write(raw)
        self.batch.writerow(dict(zip(BATCH_FIELDS, (
            batch_id, self.raw_bytes, len(raw), start, end, len(frames),
            frames[-1].timestamp_us * 1000 if frames else ""))))
        self.raw_bytes += len(raw)
        self.batches += 1
        for frame in frames:
            self.imu.writerow(dict(sample_id=self.samples, batch_id=batch_id, unified_timestamp_ns=end, **frame.row()))
            self.samples += 1

    def save_image(self, frame_id, media_ms, media_ns, start, end, image):
        """Worker-safe: write unique PNGs only; return ordered index row."""
        height, width = image.shape[:2]
        left, right = f"cam0/{frame_id:09d}.png", f"cam1/{frame_id:09d}.png"
        write_png(self.path / left, image[:, :width // 2])
        write_png(self.path / right, image[:, width // 2:])
        # Return only once both PNG files exist; CSV handles remain single-threaded.
        return dict(zip(CAM_FIELDS, (frame_id, media_ms,
            media_ns if media_ns is not None else "", start, end, end, width, height, left, right)))

    def commit_image(self, row):
        self.camera.writerow(row)
        self.images += 1

    def image(self, *item):
        self.commit_image(self.save_image(*item))

    def flush(self):
        for f in self.handles:
            f.flush()

    def close(self):
        if self.closed:
            return
        self.closed = True
        errors = []
        for f in self.handles:
            try:
                f.flush()
                os.fsync(f.fileno())
            except OSError as exc:
                errors.append(str(exc))
            finally:
                f.close()
        if errors:
            raise OSError("; ".join(errors))

def finalize(path, quality=None):
    """Finalize persisted delivery times (or recompute legacy affine models)."""
    path = Path(path)
    session = json.loads((path / "session.json").read_text(encoding="utf-8"))
    if quality is None and session.get("state") == "recording":
        raise ValueError("Session is active or was interrupted; do not finalize an open recording")
    if session.get("timestamp_basis") == TIME_BASIS:
        return _finalize_host(path, session, quality)
    persisted_quality = quality
    if persisted_quality is None and (path / "quality.json").exists():
        persisted_quality = json.loads((path / "quality.json").read_text(encoding="utf-8"))
    imu_obs = [(int(r["last_mcu_ns"]), int(r["host_read_end_ns"]))
               for r in read_csv(path / "usb_batches.csv") if r["last_mcu_ns"]]
    frames = list(read_csv(path / "frames.csv"))
    report = dict(warning=WARNING, fixed_delay_identifiable=False,
                  exposure_sync_accuracy_ns=None, camera_clock_domain_verified=False,
                  imu_source="common output grid, not simultaneous physical conversions",
                  camera_source="backend media PTS, exposure semantics unverified",
                  models_valid=False, errors=[], aligned_frames=0,
                  excluded_frames=0, observations=dict(usb_batches=len(imu_obs), camera=len(frames)))
    models = {}
    try:
        if (persisted_quality or {}).get("timeline_anomaly"):
            raise ValueError("Capture detected a timeline anomaly; raw data only")
        if any(not r["camera_media_ns"] for r in frames):
            raise ValueError("Camera media timestamp missing or invalid; no host-time fallback")
        camera_obs = [(int(r["camera_media_ns"]), int(r["host_read_end_ns"])) for r in frames]
        im = fit_clock(imu_obs)
        cm = fit_clock(camera_obs)
        models = dict(imu_to_host=im.report(), camera_to_host=cm.report())
        report["models_valid"] = True
    except ValueError as exc:
        report["errors"].append(str(exc))
    with (path / "aligned_frames.csv").open("w", newline="", encoding="utf-8") as f:
        out = csv.DictWriter(f, ALIGN_FIELDS)
        out.writeheader()
        last = None
        for r in frames:
            t, status = "", "invalid_clock_model"
            if report["models_valid"]:
                try:
                    t = align_camera(int(r["camera_media_ns"]), cm, im)
                    if last is not None and t <= last:
                        raise ValueError("Aligned camera timestamp not increasing")
                    last = t
                    status = "software_estimate"
                except ValueError as exc:
                    t = ""
                    status = str(exc)
            report["aligned_frames" if t != "" else "excluded_frames"] += 1
            out.writerow(dict(zip(ALIGN_FIELDS, (r["frame_id"], r["camera_media_ns"], t, status))))
    # Residuals remain delivery diagnostics; never interpret them as exposure error.
    with (path / "clock_residuals.csv").open("w", newline="", encoding="utf-8") as f:
        out = csv.writer(f)
        out.writerow(["source", "observation_id", "device_ns", "host_ns", "estimated_host_ns", "delivery_residual_ns"])
        if report["models_valid"]:
            for name, model, obs in (("usb_endpoint", im, imu_obs), ("camera", cm, camera_obs)):
                for i, (device, host) in enumerate(obs):
                    estimate = model.to_host(device)
                    out.writerow([name, i, device, host, estimate, host-estimate])
    json_write(path / "clock_models.json", dict(**report, models=models))
    # A forensic re-finalization does not erase the original capture quality report.
    if quality is not None:
        json_write(path / "quality.json", quality)
    session = json.loads((path / "session.json").read_text(encoding="utf-8"))
    session["state"] = ("stopped_error" if quality.get("error") else "stopped") if quality is not None else session.get("state", "recovered")
    session["finalized"] = True
    json_write(path / "session.json", session)
    return report

def _finalize_host(path, session, quality):
    """Persist direct delivery timestamps; no clock fitting or PTS prerequisites."""
    frames = list(read_csv(path / "frames.csv"))
    # A streaming join also checks every IMU's recorded unified time against its batch.
    imu_count = sum(1 for _ in iter_host_imu(path))
    last = None
    for row in frames:
        start, end = int(row["host_read_start_ns"]), int(row["host_read_end_ns"])
        if start < 0 or start > end or (last is not None and start < last):
            raise ValueError("Invalid camera host read interval")
        if int(row["unified_timestamp_ns"]) != end:
            raise ValueError("Camera unified time differs from read end")
        last = end
    with (path / "aligned_frames.csv").open("w", newline="", encoding="utf-8") as f:
        out = csv.DictWriter(f, ["frame_id", "camera_media_ns", "unified_timestamp_ns", "status"])
        out.writeheader()
        for row in frames:
            out.writerow(dict(frame_id=row["frame_id"], camera_media_ns=row["camera_media_ns"],
                              unified_timestamp_ns=row["host_read_end_ns"], status="host_receive_time"))
    report = dict(warning=HOST_WARNING, timestamp_basis=TIME_BASIS,
                  timestamp_source="host_read_end_ns", timestamps_valid=True,
                  models_valid=False, models={}, errors=[], aligned_frames=len(frames), excluded_frames=0,
                  imu_frames=imu_count, fixed_delay_identifiable=False, exposure_sync_accuracy_ns=None,
                  duplicate_imu_timestamps="preserved: one real arrival time per USB read batch",
                  camera_media_clock_required=False, clock_fitting_used=False)
    json_write(path / "clock_models.json", report)
    with (path / "clock_residuals.csv").open("w", newline="", encoding="utf-8") as f:
        csv.writer(f).writerow(["source", "observation_id", "device_ns", "host_ns",
                               "estimated_host_ns", "delivery_residual_ns"])
    if quality is not None:
        json_write(path / "quality.json", quality)
        session["state"] = "stopped_error" if quality.get("error") else "stopped"
    session["finalized"] = True
    json_write(path / "session.json", session)
    return report


def export_euroc(source, destination, match_tolerance_ns=20_000_000):
    """EuRoC-shaped delivery-time export + lossless IMU-led sparse association table.

    Duplicate arrival times are deliberately preserved, not made strictly increasing.
    This format may not be accepted by consumers requiring distinct sensor times.
    """
    source, destination = Path(source), Path(destination)
    if destination.exists():
        raise FileExistsError("Export destination already exists; refusing to overwrite")
    if not isinstance(match_tolerance_ns, int) or match_tolerance_ns < 0:
        raise ValueError("Matching tolerance must be a non-negative integer in ns")
    session = json.loads((source / "session.json").read_text(encoding="utf-8"))
    if not session.get("finalized"):
        raise ValueError("Session has not been finalized; do not export an active recording")
    # Audit legacy affine indexes as legacy, but export both schemas using raw host times.
    from .verify import verify_session
    audit = verify_session(source, write_report=False)
    frames = list(read_csv(source / "frames.csv"))
    associations = match_images(iter_host_imu(source), frames, match_tolerance_ns)
    destination.mkdir(parents=True, exist_ok=False)
    marker = destination / "EXPORT_INCOMPLETE"
    marker.write_text("Export in progress; do not use until this marker disappears", encoding="utf-8")
    for cam in ("cam0", "cam1"):
        folder = destination / "mav0" / cam
        (folder / "data").mkdir(parents=True)
        with (folder / "data.csv").open("w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(["#timestamp [ns]", "filename"])
            for row in frames:
                image = cv2.imdecode(__import__("numpy").fromfile(source / row[cam], dtype="uint8"),
                                     cv2.IMREAD_GRAYSCALE)
                if image is None:
                    raise OSError(f"Cannot read {row[cam]}")
                name = image_filename(row)
                write_png(folder / "data" / name, image)
                writer.writerow([row["host_read_end_ns"], name])
    folder = destination / "mav0" / "imu0"
    folder.mkdir()
    image_fields = ["image_frame_id", "image_timestamp_ns", "image_minus_imu_ns", "cam0", "cam1"]
    imu_count = duplicate_count = 0
    previous = None
    with (folder / "data.csv").open("w", newline="", encoding="utf-8") as f, \
         (destination / "imu_image.csv").open("w", newline="", encoding="utf-8") as joined:
        writer = csv.writer(f)
        writer.writerow(["#timestamp [ns]", "w_RS_S_x [rad s^-1]", "w_RS_S_y [rad s^-1]",
                         "w_RS_S_z [rad s^-1]", "a_RS_S_x [m s^-2]", "a_RS_S_y [m s^-2]", "a_RS_S_z [m s^-2]"])
        # Raw sensor columns stay unchanged, alongside the explicit unified host time.
        fields = list(next(iter(read_csv(source / "imu.csv")), {}).keys())
        if not fields:
            fields = IMU_FIELDS
        if "unified_timestamp_ns" not in fields:
            fields = fields + ["unified_timestamp_ns"]
        out = csv.DictWriter(joined, fields + image_fields)
        out.writeheader()
        for row in iter_host_imu(source):
            stamp = int(row["unified_timestamp_ns"])
            duplicate_count += previous == stamp
            previous = stamp
            writer.writerow([stamp] + [row["g"+a+"_rad_s"] for a in "xyz"] +
                                      [row["a"+a+"_m_s2"] for a in "xyz"])
            image = associations.get(int(row["sample_id"]))
            if image is not None:
                row = dict(row, image_frame_id=image["frame_id"],
                           image_timestamp_ns=image["host_read_end_ns"],
                           image_minus_imu_ns=int(image["host_read_end_ns"])-stamp,
                           **{c: f"mav0/{c}/data/{image_filename(image)}" for c in ("cam0", "cam1")})
            out.writerow(row)  # DictWriter leaves unmatched image fields empty.
            imu_count += 1
    camera_duplicates = sum(int(b["host_read_end_ns"]) == int(a["host_read_end_ns"])
                            for a, b in zip(frames, frames[1:]))
    trace = destination / "provenance"
    trace.mkdir()
    for name in ("session.json", "clock_models.json", "frames.csv", "aligned_frames.csv",
                 "usb_batches.csv", "imu.csv", "quality.json", "clock_residuals.csv", "devices.json", "events.jsonl"):
        if (source / name).exists():
            shutil.copy2(source / name, trace / name)
    json_write(destination / "export.json", dict(
        warning=HOST_WARNING, timestamp_basis=TIME_BASIS, timestamp_source="host_read_end_ns",
        source=str(source.resolve()), imu_frames=imu_count, stereo_pairs=len(frames),
        matched_images=len(associations), unmatched_imu_rows=imu_count-len(associations),
        unmatched_images=len(frames)-len(associations), match_tolerance_ns=match_tolerance_ns,
        association_policy="camera order; nearest remaining IMU arrival within tolerance; one-to-one; earliest sample on ties",
        duplicate_imu_timestamps=duplicate_count, duplicate_camera_timestamps=camera_duplicates,
        strict_euroc_timestamp_compatibility=duplicate_count == 0 and camera_duplicates == 0,
        excluded_camera_frames=0, calibration_provided=False, stereo_exposure_verified=False,
        source_sha256=audit["sha256"], imu_policy="all CRC-valid combined frames; no resampling or flag filtering",
        camera_policy="all indexed stereo pairs; no fitted-interval filtering; unmatched images retained"))
    (destination / "README.txt").write_text(
        HOST_WARNING + "\nHost perf_counter_ns read-end times; NOT exposure/sample times.\n"
        "IMU rows in a USB batch share a timestamp; duplicates are not altered.\n"
        "EuRoC-shaped only: consumers requiring strictly increasing times may reject this data.\n"
        "imu_image.csv includes EVERY IMU row; unmatched image fields are blank.\n"
        "Association is a time-ordered nearest-remaining one-to-one match within the documented tolerance, not synchronization.\n"
        "No calibration supplied; axes unchanged; rad/s and m/s^2. Archive source with provenance.\n", encoding="utf-8")
    marker.unlink()
    return dict(imu_frames=imu_count, stereo_pairs=len(frames), matched_images=len(associations),
                unmatched_imu_rows=imu_count-len(associations), destination=str(destination))
