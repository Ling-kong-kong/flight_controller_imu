"""Connection-local attitude history, independent of recording and GUI refresh."""
from collections import deque
import threading
import time
import numpy as np
from .history import WaveformHistory


class AttitudeHistory:
    def __init__(self, capacity=10000, max_bytes=128*1024**2):
        # Bounded legacy snapshot API; the plots use the chunked full history.
        self.rows = deque(maxlen=capacity)
        self.lock = threading.Lock()
        self.latest = self.previous = None
        self.received_at = None
        self.origin = None
        self.waveforms = WaveformHistory(channels=7, value_dtype="<f8", scales=[1.]*7,
                                         max_bytes=max_bytes)

    def append_frames(self, frames):
        if not frames:
            return
        try:
            self._append_frames(frames)
        except MemoryError:
            # Visualization failure must never stop the raw recording sink.
            with self.waveforms.lock:
                self.waveforms._freeze("姿态波形内存分配失败")
                self.waveforms.last_valid = False

    def _append_frames(self, frames):
        values, valid, breaks = [], [], []
        with self.lock:
            for f in frames:
                if self.origin is None:
                    self.origin = f.timestamp_us
                t = (f.timestamp_us-self.origin)/1e6
                p = self.previous
                gap = (p is not None and (((f.sequence-p.sequence) & 0xffffffff) != 1
                       or f.timestamp_us-p.timestamp_us > 10000
                       or f.timestamp_us <= p.timestamp_us or f.flags & 16
                       or any(abs((getattr(f, axis) or 0)-(getattr(p, axis) or 0)) > 180
                              for axis in ("roll_deg", "pitch_deg", "yaw_deg"))))
                attitude_valid = f.attitude_valid
                # q and -q encode the same pose. Preserve transmitted components,
                # but do not draw a spurious line through a representation flip.
                if p is not None and attitude_valid and p.attitude_valid:
                    gap |= sum(getattr(f, k)*getattr(p, k)
                               for k in ("qw", "qx", "qy", "qz")) < 0
                if gap:
                    self.rows.append((t, float("nan"), float("nan"), float("nan")))
                self.rows.append((t, f.roll_deg, f.pitch_deg, f.yaw_deg) if attitude_valid
                                 else (t, float("nan"), float("nan"), float("nan")))
                values.append((f.roll_deg, f.pitch_deg, f.yaw_deg, f.qw, f.qx, f.qy, f.qz)
                              if attitude_valid else (float("nan"),)*7)
                valid.append(attitude_valid)
                breaks.append(gap)
                self.previous = self.latest = f
            self.received_at = time.monotonic()
        # No label/status lock held during summary computation or rendering.
        self.waveforms.append_arrays(
            np.fromiter((f.timestamp_us for f in frames), np.uint64), values,
            np.fromiter((f.sequence for f in frames), np.uint32),
            np.fromiter((f.flags for f in frames), np.uint16),
            validity=valid, discontinuity=breaks)

    def _status(self, connected, now):
        now = time.monotonic() if now is None else now
        live = (connected and self.received_at is not None and now-self.received_at <= .1)
        return self.latest, bool(live)

    def status(self, connected=True, now=None):
        """Small live-value snapshot; does not copy any waveform history."""
        with self.lock:
            return self._status(connected, now)

    def snapshot(self, connected=True, now=None):
        """Compatibility/debug snapshot of the bounded recent Euler queue only."""
        with self.lock:
            return (*self._status(connected, now),
                    np.array(self.rows, dtype=float).reshape(-1, 4))
