"""Independent, bounded IMU2 v2 decoder. No firmware imports or writes."""
from dataclasses import dataclass, asdict
import math
import struct
import zlib

PACKET = struct.Struct("<4sBBHIQhhhhhhIIHHI")
FLAGS = {1: "output_timing", 2: "accel_spi_error", 4: "gyro_spi_error",
         8: "ring_overflow", 16: "session_start", 32: "accel_stale",
         64: "gyro_stale", 128: "accel_drdy_anomaly",
         256: "gyro_drdy_anomaly", 512: "history_missing"}

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

    def row(self):
        r = asdict(self)
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
        b = PACKET.pack(b"IMU2", 2, 3, 48, *asdict(self).values(), 0)
        return b[:44] + struct.pack("<I", zlib.crc32(b[:44]))

class Parser:
    def __init__(self):
        self.buffer = bytearray()
        self.stats = dict(bytes_received=0, bytes_discarded=0,
                          crc_errors=0, header_errors=0, frames=0)

    def feed(self, data):
        self.stats["bytes_received"] += len(data)
        result = []
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
                if self.buffer[4:8] != b"\x02\x03\x30\x00":
                    self.stats["header_errors"] += 1
                    self._discard(1)
                    continue
                if len(self.buffer) < 48:
                    break
                packet = bytes(self.buffer[:48])
                fields = PACKET.unpack(packet)
                if zlib.crc32(packet[:44]) != fields[-1]:
                    self.stats["crc_errors"] += 1
                    self._discard(1)
                    continue
                del self.buffer[:48]
                result.append(Frame(*fields[4:-1]))
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
