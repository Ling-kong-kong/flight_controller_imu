"""Direct host-delivery time axis and sparse, IMU-preserving image association.

These timestamps describe read completion, never sensor sampling or exposure.
"""
import csv
from collections import deque
from pathlib import Path

TIME_BASIS = "host_receive"
HOST_WARNING = "电脑接收时间统一，曝光同步误差未验证"


def _rows(path):
    with Path(path).open(newline="", encoding="utf-8") as f:
        yield from csv.DictReader(f)


def iter_host_imu(path):
    """Streaming batch join. Preserve original device fields and exact batch end.

    A frame split over reads belongs to the batch that completes its decoding.
    Empty decode batches are checked too. No timestamp interpolation is used.
    """
    path = Path(path)
    imu = iter(_rows(path / "imu.csv"))
    sample_id = 0
    previous_end = None
    for batch_id, b in enumerate(_rows(path / "usb_batches.csv")):
        if int(b["batch_id"]) != batch_id:
            raise ValueError("USB batch ID mismatch")
        start, end = int(b["host_read_start_ns"]), int(b["host_read_end_ns"])
        if start < 0 or end < start or (previous_end is not None and start < previous_end):
            raise ValueError("Invalid USB host read interval")
        previous_end = end
        count = int(b["frame_count"])
        if count < 0:
            raise ValueError("Invalid USB frame count")
        last_mcu = ""
        for _ in range(count):
            row = next(imu, None)
            if row is None or int(row["sample_id"]) != sample_id or int(row["batch_id"]) != batch_id:
                raise ValueError("IMU row/batch correspondence mismatch")
            if int(row["timestamp_ns"]) != int(row["timestamp_us"]) * 1000:
                raise ValueError("IMU device timestamp conversion mismatch")
            if "unified_timestamp_ns" in row and row["unified_timestamp_ns"] != str(end):
                raise ValueError("IMU unified time differs from USB read end")
            row["unified_timestamp_ns"] = str(end)
            last_mcu = row["timestamp_ns"]
            sample_id += 1
            yield row
        if b["last_mcu_ns"] != last_mcu:
            raise ValueError("USB batch clock endpoint differs")
    if next(imu, None) is not None:
        raise ValueError("Unindexed IMU rows")


def image_filename(frame):
    # Frame ID is needed: two camera reads can have the same recorded host time.
    return f'{int(frame["host_read_end_ns"])}_{int(frame["frame_id"]):09d}.png'


def match_images(imu_rows, frames, tolerance_ns):
    """Camera-ordered greedy nearest-remaining match, ties to earliest sample.

    One-to-one, not a global optimum or evidence of synchronization. IMU rows
    remain in the output even when they are skipped here. Streams are traversed
    once; unmatched past groups within the tolerance and associations are retained.
    """
    if not isinstance(tolerance_ns, int) or tolerance_ns < 0:
        raise ValueError("Matching tolerance must be a non-negative integer in ns")
    stream = iter(imu_rows)
    upcoming = next(stream, None)
    # Retain older eligible groups: after a closer group is exhausted, an older
    # sample can still be the nearest remaining match for the next camera frame.
    before = deque()
    result = {}
    previous_image = None
    for frame in frames:
        t = int(frame["host_read_end_ns"])
        if previous_image is not None and t < previous_image:
            raise ValueError("Camera host timestamps regressed")
        previous_image = t
        while upcoming is not None and int(upcoming["unified_timestamp_ns"]) <= t:
            stamp = int(upcoming["unified_timestamp_ns"])
            if stamp >= t - tolerance_ns:
                if not before or before[-1][0] != stamp:
                    before.append((stamp, deque()))
                before[-1][1].append(upcoming)
            upcoming = next(stream, None)
        while before and before[0][0] < t - tolerance_ns:
            before.popleft()
        candidate = before[-1][1][0] if before else None
        distance = t - before[-1][0] if before else None
        use_future = upcoming is not None and (distance is None or
                     int(upcoming["unified_timestamp_ns"]) - t < distance)
        if use_future:
            candidate = upcoming
            distance = int(candidate["unified_timestamp_ns"]) - t
        if candidate is not None and distance <= tolerance_ns:
            result[int(candidate["sample_id"])] = frame
            if use_future:
                upcoming = next(stream, None)
            else:
                before[-1][1].popleft()
                if not before[-1][1]:
                    before.pop()
    return result
