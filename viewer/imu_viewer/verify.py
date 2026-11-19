"""Streaming integrity audit. Does not access devices or modify original indexes."""
import hashlib
import json
from pathlib import Path
from .protocol import Parser, Continuity, ATTITUDE_FIELDS
from .storage import read_csv, json_write
from .clock import ClockModel, align_camera
from .timing import TIME_BASIS

def digest(path):
    h = hashlib.sha256()
    with Path(path).open("rb") as f:
        for chunk in iter(lambda: f.read(1024*1024), b""):
            h.update(chunk)
    return h.hexdigest()

def verify_session(path, decode_all=False, write_report=True):
    path = Path(path)
    session = json.loads((path/"session.json").read_text(encoding="utf8"))
    host_mode = session.get("timestamp_basis") == TIME_BASIS
    if session.get("schema_version", 1) >= 2 and not host_mode:
        raise ValueError("Unsupported session timestamp basis")
    parser, continuity = Parser(), Continuity()
    imu = iter(read_csv(path/"imu.csv"))
    batches = samples = offset = 0
    first_host = last_host = last_read_end = None
    with (path/"usb.bin").open("rb") as raw:
        for b in read_csv(path/"usb_batches.csv"):
            if int(b["batch_id"]) != batches or int(b["raw_offset"]) != offset:
                raise ValueError("USB batch ID/byte offset mismatch")
            n = int(b["byte_count"])
            if n <= 0 or n > 4096:
                raise ValueError("Invalid USB batch byte count")
            data = raw.read(n)
            if len(data) != n:
                raise ValueError("Raw USB file truncated")
            frames = parser.feed(data)
            if len(frames) != int(b["frame_count"]):
                raise ValueError("USB parser frame count differs from index")
            endpoint = str(frames[-1].timestamp_us*1000) if frames else ""
            if endpoint != b["last_mcu_ns"]:
                raise ValueError("USB batch clock endpoint differs")
            start, end = int(b["host_read_start_ns"]), int(b["host_read_end_ns"])
            if start < 0 or end < start or (last_read_end is not None and start < last_read_end):
                raise ValueError("Invalid USB host read interval")
            last_read_end = end
            for f in frames:
                row = next(imu, None)
                if row is None or int(row["sample_id"]) != samples or int(row["batch_id"]) != batches:
                    raise ValueError("IMU row/batch correspondence mismatch")
                # Only old schemas may omit the newly introduced attitude fields.
                new_fields = set(ATTITUDE_FIELDS) | {"protocol_version", "attitude_valid"}
                expected = f.row()
                if session.get("schema_version", 1) < 3 and f.protocol_version == 2:
                    expected = {k:v for k,v in expected.items() if k in row or k not in new_fields}
                if any(row.get(k) != str(v) for k,v in expected.items()):
                    raise ValueError("IMU CSV differs from raw USB decoding")
                if (host_mode and "unified_timestamp_ns" not in row) or (
                        "unified_timestamp_ns" in row and row["unified_timestamp_ns"] != str(end)):
                    raise ValueError("IMU unified time differs from USB read end")
                anomaly = continuity.accept(f)
                if anomaly:
                    raise ValueError(anomaly)
                samples += 1
            if frames and first_host is None:
                first_host = end
            if frames:
                last_host = end
            offset += n
            batches += 1
        if raw.read(1) or next(imu, None) is not None:
            raise ValueError("Unindexed raw USB bytes or IMU rows")
    rows = list(read_csv(path/"frames.csv"))
    aligned = list(read_csv(path/"aligned_frames.csv"))
    if len(aligned) != len(rows):
        raise ValueError("Frame/alignment index lengths differ")
    clock = json.loads((path/"clock_models.json").read_text(encoding="utf8"))
    if host_mode != (clock.get("timestamp_basis") == TIME_BASIS):
        raise ValueError("Session/clock timestamp basis mismatch")
    if host_mode and (not clock.get("timestamps_valid") or clock.get("clock_fitting_used") is not False
                      or clock.get("models_valid") or clock.get("models")):
        raise ValueError("Invalid host-time metadata")
    models = {}
    if clock["models_valid"]:
        for key, report in clock["models"].items():
            models[key] = ClockModel(**{k:report[k] for k in ClockModel.__dataclass_fields__})
    previous = None
    previous_media = None
    decoded = 0
    aligned_count = 0
    total_png_bytes = 0
    last_camera_host = None
    for i,(r,a) in enumerate(zip(rows,aligned)):
        if (int(r["frame_id"]) != i or r["frame_id"] != a["frame_id"]
                or r["camera_media_ns"] != a["camera_media_ns"]):
            raise ValueError("Camera frame IDs differ")
        host_start,host_end = int(r["host_read_start_ns"]), int(r["host_read_end_ns"])
        if host_start < 0 or host_end < host_start or (last_camera_host is not None and host_start < last_camera_host):
            raise ValueError("Invalid camera host read interval")
        last_camera_host = host_end
        if host_mode:
            if (r.get("unified_timestamp_ns") != str(host_end)
                    or a.get("unified_timestamp_ns") != str(host_end)
                    or a.get("status") != "host_receive_time"):
                raise ValueError("Missing or inconsistent aligned image timestamp")
            aligned_count += 1
        else:
            media = int(r["camera_media_ns"]) if r["camera_media_ns"] else None
            if clock["models_valid"] and (media is None or (previous_media is not None and media <= previous_media)):
                raise ValueError("Invalid media clock")
            previous_media = media
            expected = None
            if clock["models_valid"]:
                try:
                    expected = align_camera(media, models["camera_to_host"], models["imu_to_host"])
                except ValueError:
                    pass
            if (a["estimated_mcu_ns"] != (str(expected) if expected is not None else "")):
                raise ValueError("Missing or inconsistent aligned image timestamp")
            if a["estimated_mcu_ns"]:
                t = int(a["estimated_mcu_ns"])
                if previous is not None and t <= previous:
                    raise ValueError("Non-increasing aligned image timestamps")
                if not clock["models_valid"] or t != align_camera(media,models["camera_to_host"],models["imu_to_host"]):
                    raise ValueError("Saved alignment cannot be recomputed")
                previous = t
                aligned_count += 1
        for cam in ("cam0","cam1"):
            image_path = path/r[cam]
            total_png_bytes += image_path.stat().st_size
            if decode_all or i in (0,len(rows)//2,len(rows)-1):
                import cv2
                import numpy as np
                image = cv2.imdecode(np.fromfile(image_path,dtype=np.uint8),cv2.IMREAD_UNCHANGED)
                expected = (int(r["height"]),int(r["width"])//2,3)
                if image is None or image.shape != expected:
                    raise ValueError("Unreadable/non-color/wrong-sized stereo PNG")
                decoded += 1
    if aligned_count != clock["aligned_frames"]:
        raise ValueError("Aligned image count differs from clock report")
    span = (int(rows[-1]["host_read_end_ns"])-int(rows[0]["host_read_end_ns"]))/1e9 if len(rows)>1 else 0
    hashes = {n:digest(path/n) for n in ("session.json","usb.bin","imu.csv","usb_batches.csv","frames.csv",
                                       "aligned_frames.csv","clock_models.json","quality.json") if (path/n).exists()}
    report = dict(status="passed", imu_frames=samples, usb_batches=batches, usb_bytes=offset,
                  parser=parser.stats, trailing_usb_bytes=len(parser.buffer), continuity=continuity.stats,
                  stereo_pairs=len(rows), aligned_pairs=aligned_count, camera_host_span_s=span,
                  camera_measured_host_fps=(len(rows)-1)/span if span else None,
                  imu_measured_host_hz=(samples-1)*1e9/(last_host-first_host) if samples>1 and last_host>first_host else None,
                  png_files_checked=2*len(rows), png_files_decoded=decoded, png_bytes=total_png_bytes,
                  decode_policy="all" if decode_all else "first/middle/last pair; existence/size for all",
                  sha256=hashes, warning=clock["warning"], exposure_accuracy_verified=False)
    if write_report:
        json_write(path/"verification.json",report)
    return report
