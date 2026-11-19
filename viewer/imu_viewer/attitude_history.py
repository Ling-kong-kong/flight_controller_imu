"""Bounded, connection-local attitude history; never substitutes raw measurements."""
from collections import deque
import threading
import time
import numpy as np


class AttitudeHistory:
    def __init__(self, capacity=10000):
        self.rows = deque(maxlen=capacity)
        self.lock = threading.Lock()
        self.latest = self.previous = None
        self.received_at = None
        self.origin = None

    def append_frames(self, frames):
        with self.lock:
            for f in frames:
                if self.origin is None:
                    self.origin = f.timestamp_us
                t = (f.timestamp_us-self.origin)/1e6
                p = self.previous
                gap = (p is not None and (((f.sequence-p.sequence) & 0xffffffff) != 1
                       or f.timestamp_us-p.timestamp_us > 10000
                       or f.timestamp_us <= p.timestamp_us or f.flags & 16
                       or abs((f.yaw_deg or 0)-(p.yaw_deg or 0)) > 180))
                if gap:
                    self.rows.append((t, float("nan"), float("nan"), float("nan")))
                self.rows.append((t, f.roll_deg, f.pitch_deg, f.yaw_deg) if f.attitude_valid
                                 else (t, float("nan"), float("nan"), float("nan")))
                self.previous = self.latest = f
            if frames:
                self.received_at = time.monotonic()

    def snapshot(self, connected=True, now=None):
        with self.lock:
            now = time.monotonic() if now is None else now
            live = (connected and self.received_at is not None and now-self.received_at <= .1)
            return self.latest, bool(live), np.array(self.rows, dtype=float).reshape(-1, 4)
