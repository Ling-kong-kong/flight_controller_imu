"""Independent, bounded IMU2 v2/v3 decoder. Data frames plus CRC-protected calibration control."""
from dataclasses import dataclass, asdict
import math
import struct
import zlib

PACKET = struct.Struct("<4sBBHIQhhhhhhIIHHI")
PACKET_V3 = struct.Struct("<4sBBHIQhhhhhhIIHH7fI")
MAG_PACKET = struct.Struct("<4sBBHIQhhhHfffHBBBBBBIIIIII")
CONTROL = struct.Struct("<4sBBHIIQII")
FUSION_DIAGNOSTIC = struct.Struct("<4sBBHQIIIIIffIII")
assert FUSION_DIAGNOSTIC.size == 56
MAG_REASONS = ("waiting", "ok", "imu", "motion", "quality", "time", "sequence",
               "stale", "overflow", "norm", "horizontal", "tilt_reserved", "dip",
               "innovation", "unstable")
ATTITUDE_FIELDS = ("qw", "qx", "qy", "qz", "roll_deg", "pitch_deg", "yaw_deg")
FLAGS = {1: "output_timing", 2: "accel_spi_error", 4: "gyro_spi_error",
         8: "ring_overflow", 16: "session_start", 32: "accel_stale",
         64: "gyro_stale", 128: "accel_drdy_anomaly",
         256: "gyro_drdy_anomaly", 512: "history_missing", 1024: "calibrating",
         2048: "attitude_valid", 4096: "fusion_reset",
         8192: "mag_heading_aiding", 16384: "mag_heading_rejected",
         32768: "mag_heading_reference"}

# Only raw sensor/acquisition errors invalidate the six-axis display. Calibration,
# fusion and magnetic-heading status flags describe attitude, not raw sensor data.
RAW_IMU_INVALID_FLAGS = (2 | 4 | 8 | 32 | 64 | 128 | 256 | 512)

@dataclass(frozen=True)
class Frame:
    sequence: int
    timestamp_us: int
    ax_raw: int
    ay_raw: int
    az_raw: int
    gx_raw: int
    gy_raw: int
    gz_raw: int
    accel_age_us: int
    gyro_age_us: int
    flags: int = 0
    tick_late_us: int = 0

    protocol_version: int = 2
    qw: float | None = None
    qx: float | None = None
    qy: float | None = None
    qz: float | None = None
    roll_deg: float | None = None
    pitch_deg: float | None = None
    yaw_deg: float | None = None

    @property
    def attitude_valid(self):
        values = [getattr(self, k) for k in ATTITUDE_FIELDS]
        return (self.protocol_version == 3 and bool(self.flags & 2048)
                and not self.flags & (1024 | 4096)
                and max(self.accel_age_us, self.gyro_age_us) <= self.timestamp_us
                and self.gyro_age_us <= 1000 and not self.flags & 4
                and all(v is not None and math.isfinite(v) for v in values)
                and abs(sum(v*v for v in values[:4])-1) < 0.01)

    def row(self):
        r = asdict(self)
        r["attitude_valid"] = int(self.attitude_valid)
        if not self.attitude_valid:
            r.update({k: "" for k in ATTITUDE_FIELDS})
        r.update(timestamp_ns=self.timestamp_us * 1000,
                 accel_time_us=self.timestamp_us - self.accel_age_us,
                 gyro_time_us=self.timestamp_us - self.gyro_age_us)
        for axis in "xyz":
            a = getattr(self, "a" + axis + "_raw") * 24 / 32768
            g = getattr(self, "g" + axis + "_raw") * 2000 / 32768
            r["a" + axis + "_g"] = a
            r["g" + axis + "_deg_s"] = g
            r["a" + axis + "_m_s2"] = a * 9.80665
            r["g" + axis + "_rad_s"] = g * math.pi / 180
        return r

    def encode(self):
        """Offline fixture helper only; never transmitted."""
        values = [getattr(self, k) for k in (
            "sequence", "timestamp_us", "ax_raw", "ay_raw", "az_raw",
            "gx_raw", "gy_raw", "gz_raw", "accel_age_us", "gyro_age_us", "flags", "tick_late_us")]
        packet = PACKET_V3 if self.protocol_version == 3 else PACKET
        if self.protocol_version == 3:
            values += [getattr(self, k) if getattr(self, k) is not None else 0.0 for k in ATTITUDE_FIELDS]
        b = packet.pack(b"IMU2", self.protocol_version, 3, packet.size, *values, 0)
        return b[:-4] + struct.pack("<I", zlib.crc32(b[:-4]))

@dataclass(frozen=True)
class ControlReply:
    request_id: int
    status: int
    timestamp_us: int
    sequence: int

    operation: int = 1

    def encode(self):
        return _control_packet(0x13 if self.operation == 2 else 0x11, self.request_id, self.status, self.timestamp_us, self.sequence)



@dataclass(frozen=True)
class FusionDiagnostic:
    timestamp_us: int
    sequence: int
    phase: int
    reason: int
    recovery_count: int
    recovery_elapsed_ms: int
    innovation_deg: float
    correction_rate_dps: float
    flags: int
    request_id: int

    @property
    def reason_name(self):
        return MAG_REASONS[self.reason] if 0 <= self.reason < len(MAG_REASONS) else "unknown"

    def encode(self):
        values = [getattr(self, key) for key in self.__dataclass_fields__]
        raw = FUSION_DIAGNOSTIC.pack(b"IMU2", 1, 0x12, 56, *values, 0)
        return raw[:-4] + struct.pack("<I", zlib.crc32(raw[:-4]))


def _control_packet(kind, request_id, value, timestamp_us=0, sequence=0):
    raw = CONTROL.pack(b"IMU2", 1, kind, CONTROL.size, request_id, value,
                       timestamp_us, sequence, 0)
    return raw[:-4] + struct.pack("<I", zlib.crc32(raw[:-4]))


def recalibrate_command(request_id):
    if not 0 < request_id <= 0xffffffff:
        raise ValueError("Nonzero uint32 request ID required")
    return _control_packet(0x10, request_id, 1)


def heading_zero_command(request_id):
    if not 0 < request_id <= 0xffffffff:
        raise ValueError("Nonzero uint32 request ID required")
    return _control_packet(0x10, request_id, 2)


class Parser:
    def __init__(self):
        self.buffer = bytearray()
        self.records = []
        self.stats = dict(bytes_received=0, bytes_discarded=0,
                          crc_errors=0, header_errors=0, frames=0, control_replies=0, mag_frames=0, fusion_diagnostics=0)

    def feed(self, data):
        self.stats["bytes_received"] += len(data)
        result = []
        self.records = []
        # Drain per chunk: storage remains below 4096 bytes even on huge input.
        for start in range(0, len(data), 4000):
            self.buffer.extend(data[start:start + 4000])
            while True:
                i = self.buffer.find(b"IMU2")
                if i < 0:
                    self._discard(max(0, len(self.buffer) - 3))
                    break
                self._discard(i)
                if len(self.buffer) < 8:
                    break
                if bytes(self.buffer[4:8]) not in (b"\x02\x03\x30\x00", b"\x03\x03\x4c\x00", b"\x01\x11\x20\x00", b"\x01\x13\x20\x00", b"\x01\x12\x38\x00", b"\x01\x04\x48\x00"):
                    self.stats["header_errors"] += 1
                    self._discard(1)
                    continue
                codec = CONTROL if self.buffer[4] == 1 else (PACKET_V3 if self.buffer[4] == 3 else PACKET)
                if self.buffer[5] == 4:
                    codec = MAG_PACKET
                if self.buffer[5] == 0x12:
                    codec = FUSION_DIAGNOSTIC
                if len(self.buffer) < codec.size:
                    break
                packet = bytes(self.buffer[:codec.size])
                fields = codec.unpack(packet)
                if zlib.crc32(packet[:-4]) != fields[-1]:
                    self.stats["crc_errors"] += 1
                    self._discard(1)
                    continue
                del self.buffer[:codec.size]
                if codec is MAG_PACKET:
                    # Independent diagnostic stream; never pass to IMU recording/control.
                    self.stats["mag_frames"] += 1
                    continue
                if codec is FUSION_DIAGNOSTIC:
                    self.records.append(FusionDiagnostic(*fields[4:-1]))
                    self.stats["fusion_diagnostics"] += 1
                    continue
                if codec is CONTROL:
                    self.records.append(ControlReply(*fields[4:-1], operation=2 if fields[2] == 0x13 else 1))
                    self.stats["control_replies"] += 1
                    continue
                result.append(Frame(*fields[4:16], protocol_version=fields[1],
                                    **(dict(zip(ATTITUDE_FIELDS, fields[16:-1])) if fields[1] == 3 else {})))
                self.records.append(result[-1])
                self.stats["frames"] += 1
        return result

    def _discard(self, n):
        self.stats["bytes_discarded"] += n
        del self.buffer[:n]

class Continuity:
    def __init__(self):
        self.previous = None
        self.stats = dict(samples=0, missing_sequences=0, duplicate_sequences=0,
                          sequence_regressions=0, timestamp_regressions=0,
                          sessions_observed=0, invalid_source_times=0,
                          flag_counts={name: 0 for name in FLAGS.values()})

    def accept(self, frame):
        """Returns fatal timeline anomaly; packet/quality loss remains explicit."""
        s = self.stats
        s["samples"] += 1
        for mask, name in FLAGS.items():
            if frame.flags & mask:
                s["flag_counts"][name] += 1
        if max(frame.accel_age_us, frame.gyro_age_us) > frame.timestamp_us:
            s["invalid_source_times"] += 1
        fatal = None
        p = self.previous
        if p is None or frame.flags & 16:
            s["sessions_observed"] += 1
            if p is not None:
                fatal = "MCU session changed; start a new recording"
        if p is not None:
            if frame.timestamp_us <= p.timestamp_us:
                s["timestamp_regressions"] += 1
                fatal = "MCU timestamp did not increase (reset or duplicate)"
            if not frame.flags & 16:
                step = (frame.sequence - p.sequence) & 0xFFFFFFFF
                if step == 0:
                    s["duplicate_sequences"] += 1
                    fatal = "Duplicate IMU sequence"
                elif step >= 0x80000000:
                    s["sequence_regressions"] += 1
                    fatal = "IMU sequence regressed"
                else:
                    s["missing_sequences"] += step - 1
        self.previous = frame
        return fatal
