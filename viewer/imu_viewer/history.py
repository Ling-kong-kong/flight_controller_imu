"""Connection-local waveform history, independent of recording and camera clocks.

Recent samples are exact raw integers. Older data is a per-axis first/min/max/last
summary on a 30 Hz MCU-time grid, NOT synchronized image samples. Completed chunks
are immutable; readers only copy partial chunks under the short history lock.
"""
from bisect import bisect_left
from dataclasses import dataclass
import threading
import numpy as np

RAW = np.dtype([("t", "<u8"), ("v", "<i2", (6,)),
                ("segment", "<u8"), ("valid", "?")])
SUMMARY = np.dtype([("start", "<u8"), ("end", "<u8"),
                    ("t", "<u8", (4, 6)), ("v", "<i2", (4, 6)),
                    ("segment", "<u8")])


class Chunks:
    """Append-only arrays; sealed arrays can safely outlive pruning/snapshots."""
    def __init__(self, dtype, size):
        self.dtype, self.size = dtype, size
        self.chunks, self.ends = [], []
        self.active = None
        self.used = 0

    @property
    def allocated(self):
        return (len(self.chunks) + (self.active is not None)) * self.size * self.dtype.itemsize

    def append(self, rows):
        offset = 0
        while offset < len(rows):
            if self.active is None:
                self.active = np.empty(self.size, self.dtype)
            n = min(len(rows)-offset, self.size-self.used)
            self.active[self.used:self.used+n] = rows[offset:offset+n]
            self.used += n
            offset += n
            if self.used == self.size:
                self.active.setflags(write=False)
                self.chunks.append(self.active)
                key = "t" if self.dtype == RAW else "end"
                self.ends.append(int(self.active[key][-1]))
                self.active, self.used = None, 0

    def prune(self, cutoff):
        n = bisect_left(self.ends, cutoff)
        if n:
            del self.chunks[:n]
            del self.ends[:n]

    def snapshot(self, lo, hi):
        key = "t" if self.dtype == RAW else "start"
        i = bisect_left(self.ends, lo)
        result = []
        # Do not copy the entire tail of the index for a narrow historical view.
        for j in range(i, len(self.chunks)):
            chunk = self.chunks[j]
            if int(chunk[key][0]) > hi:
                break
            result.append(chunk)
        if self.used and int(self.active[key][0]) <= hi:
            result.append(self.active[:self.used].copy())
        return result


@dataclass(frozen=True)
class HistoryInfo:
    origin_us: int | None
    latest_us: int | None
    summary_end_us: int | None
    raw_start_us: int | None
    samples: int
    allocated_bytes: int
    warning: str | None

    @property
    def duration(self):
        return 0.0 if self.origin_us is None else (self.latest_us-self.origin_us)/1e6


class WaveformHistory:
    def __init__(self, recent_s=10, max_bytes=128*1024**2):
        self.lock = threading.RLock()
        self.recent_us = int(recent_s*1e6)
        self.max_bytes = max_bytes
        self.raw = Chunks(RAW, 1024)
        self.summary = Chunks(SUMMARY, 256)
        self.current = None
        self.current_bucket = None
        self.origin = self.latest = self.last_sequence = None
        self.last_valid = False
        self.segment = 0
        self.samples = 0
        self.summary_end = None
        self.warning = None
        # A fixed recent-buffer reserve bounds pathological rates as well.
        self.raw_reserve = min(4*1024**2, max(2*1024*RAW.itemsize, max_bytes//4))
        self.raw_warning = None

    def append_frames(self, frames):
        if not frames:
            return
        try:
            self.append_arrays(
                np.fromiter((f.timestamp_us for f in frames), np.uint64),
                np.array([(f.ax_raw, f.ay_raw, f.az_raw, f.gx_raw, f.gy_raw, f.gz_raw)
                          for f in frames], dtype=np.int16),
                np.fromiter((f.sequence for f in frames), np.uint32),
                np.fromiter((f.flags for f in frames), np.uint16),
                np.fromiter((max(f.accel_age_us, f.gyro_age_us) > f.timestamp_us
                             for f in frames), bool))
        except MemoryError:
            # Never stop the independent recording sink on a visualization failure.
            with self.lock:
                self._freeze("波形内存分配失败")
                self.last_valid = False

    def _allocated(self):
        # Include conservative per-chunk Python/index overhead and active summary.
        return (self.raw.allocated + self.summary.allocated + SUMMARY.itemsize
                + (len(self.raw.chunks)+len(self.summary.chunks)+4)*512)

    def _freeze(self, text):
        if self.warning is None:
            self.warning = text + "；全程历史已冻结，实时窗口及原始录制继续"

    def _commit(self):
        if self.current is None or self.warning:
            return
        extra = self.summary.size*SUMMARY.itemsize if self.summary.active is None else 0
        if self._allocated()+extra+self.raw_reserve > self.max_bytes:
            self._freeze("波形历史达到内存安全上限")
            return
        self.summary.append(np.array([self.current], dtype=SUMMARY))
        self.current = None

    def append_arrays(self, times, values, sequences, flags=None, invalid_source=None):
        """Batch API shared by acquisition and deterministic performance fixtures."""
        times = np.asarray(times, dtype=np.uint64)
        n = len(times)
        if not n:
            return
        values = np.asarray(values, dtype=np.int16).reshape(n, 6)
        sequences = np.asarray(sequences, dtype=np.uint32)
        flags = np.zeros(n, np.uint16) if flags is None else np.asarray(flags, np.uint16)
        invalid_source = np.zeros(n, bool) if invalid_source is None else np.asarray(invalid_source, bool)
        with self.lock:
            # Do not fabricate a monotonic axis after reset/duplicates.
            bad = np.flatnonzero(times[1:] <= times[:-1]) + 1
            if self.latest is not None and int(times[0]) <= self.latest:
                bad = np.r_[0, bad]
            timeline_bad = len(bad) > 0
            if timeline_bad:
                n = int(bad[0])
                times, values, sequences = times[:n], values[:n], sequences[:n]
                flags, invalid_source = flags[:n], invalid_source[:n]
            if not n:
                self._freeze("H743 时间戳异常")
                self.last_valid = False
                return
            if self.origin is None:
                self.origin = int(times[0])
            # Attitude/calibration bits must not suppress raw six-axis measurements.
            valid = ((flags & 0xE3EE) == 0) & ~invalid_source
            steps = (sequences.astype(np.uint64) - np.r_[
                np.uint64(self.last_sequence if self.last_sequence is not None else (int(sequences[0])-1)&0xFFFFFFFF),
                sequences[:-1].astype(np.uint64)]) & np.uint64(0xFFFFFFFF)
            breaks = (steps != 1) | ~valid | ~np.r_[self.last_valid, valid[:-1]] | ((flags & 16) != 0)
            segments = self.segment + np.cumsum(breaks, dtype=np.uint64)
            rows = np.empty(n, RAW)
            rows["t"], rows["v"], rows["valid"], rows["segment"] = times, values, valid, segments
            # Append in bounded portions so even an unusually large input batch is bounded.
            for offset in range(0, n, 1024):
                part = rows[offset:offset+1024]
                self.raw.prune(int(part["t"][-1])-self.recent_us)
                while self.raw.chunks and self.raw.allocated+1024*RAW.itemsize > self.raw_reserve:
                    del self.raw.chunks[0]
                    del self.raw.ends[0]
                    self.raw_warning = "输入速率过高：最近完整样本窗口不足 10 秒"
                self.raw.append(part)
            self.latest, self.last_sequence = int(times[-1]), int(sequences[-1])
            self.segment, self.last_valid = int(segments[-1]), bool(valid[-1])
            self.samples += n
            if not self.warning:
                buckets = ((times - np.uint64(self.origin))*30)//1_000_000
                starts = np.r_[0, np.flatnonzero((buckets[1:] != buckets[:-1]) |
                                               (segments[1:] != segments[:-1]))+1]
                ends = np.r_[starts[1:], n]
                try:
                    for a, b in zip(starts, ends):
                        a, b = int(a), int(b)
                        if not valid[a]:
                            self._commit()
                            continue
                        bucket, segment = int(buckets[a]), int(segments[a])
                        if self.current is not None and (bucket != self.current_bucket or
                                                       segment != int(self.current["segment"])):
                            self._commit()
                        if self.warning:
                            break
                        v = values[a:b]
                        mins, maxs = v.argmin(axis=0)+a, v.argmax(axis=0)+a
                        ix = np.stack((np.full(6, a), mins, maxs, np.full(6, b-1)))
                        ts, vs = times[ix], values[ix, np.arange(6)]
                        if self.current is None:
                            self.current = np.zeros((), SUMMARY)
                            self.current["start"], self.current["segment"] = times[a], segment
                            self.current["t"], self.current["v"] = ts, vs
                            self.current_bucket = bucket
                        else:
                            for row, cmp in ((1, np.less), (2, np.greater)):
                                mask = cmp(vs[row], self.current["v"][row])
                                self.current["v"][row, mask] = vs[row, mask]
                                self.current["t"][row, mask] = ts[row, mask]
                            self.current["t"][3], self.current["v"][3] = ts[3], vs[3]
                        self.current["end"] = times[b-1]
                        self.summary_end = int(times[b-1])
                except MemoryError:
                    self._freeze("波形内存分配失败")
            if timeline_bad:
                self._freeze("H743 时间戳异常")
                self.last_valid = False

    def info(self):
        with self.lock:
            raw_start = (int(self.raw.chunks[0]["t"][0]) if self.raw.chunks else
                         int(self.raw.active["t"][0]) if self.raw.used else None)
            if raw_start is not None:
                raw_start = max(raw_start, self.latest-self.recent_us)
            return HistoryInfo(self.origin, self.latest, self.summary_end, raw_start,
                               self.samples, self._allocated(), self.warning or self.raw_warning)

    def snapshot(self, lo_s, hi_s):
        with self.lock:
            info = self.info()
            if self.origin is None:
                return info, [], []
            lo, hi = self.origin+int(lo_s*1e6), self.origin+int(hi_s*1e6)
            summaries = self.summary.snapshot(lo, hi)
            if self.current is not None and int(self.current["end"]) >= lo and int(self.current["start"]) <= hi:
                summaries.append(np.array([self.current], dtype=SUMMARY))
            raw = self.raw.snapshot(lo, hi)
            return info, summaries, raw


def envelope(t, v, segment, lo, hi, pixels):
    """First/min/max/last per pixel AND continuity segment, with NaN gap markers.

    Vectorized selection retains the *actual* time of each axis's extreme. There
    is no synthesized six-axis sample at an envelope extremum.
    """
    if not len(t):
        return np.empty(0), np.empty(0)
    keep = (t >= lo) & (t <= hi)
    t, v, segment = t[keep], v[keep], segment[keep]
    if not len(t):
        return np.empty(0), np.empty(0)
    bins = np.floor((t-lo)/max(hi-lo, 1e-6)*max(1, pixels)).astype(np.int64)
    starts = np.r_[0, np.flatnonzero((bins[1:] != bins[:-1]) | (segment[1:] != segment[:-1]))+1]
    ends = np.r_[starts[1:], len(t)]
    groups = np.repeat(np.arange(len(starts)), ends-starts)
    low, high = np.minimum.reduceat(v, starts), np.maximum.reduceat(v, starts)
    indices = np.arange(len(t))
    imin, imax = np.full(len(starts), len(t)), np.full(len(starts), len(t))
    np.minimum.at(imin, groups, np.where(v == low[groups], indices, len(t)))
    np.minimum.at(imax, groups, np.where(v == high[groups], indices, len(t)))
    chosen = np.sort(np.stack((starts, imin, imax, ends-1)), axis=0).T.ravel()
    # Keep order; remove duplicate point indices only (not equal-valued samples).
    chosen = chosen[np.r_[True, chosen[1:] != chosen[:-1]]]
    x, y, seg = t[chosen], v[chosen], segment[chosen]
    gaps = np.flatnonzero(seg[1:] != seg[:-1])+1
    if len(gaps):
        x, y = np.insert(x, gaps, np.nan), np.insert(y.astype(float), gaps, np.nan)
    return x, y


def render_history(history, lo_s, hi_s, pixels=1000):
    """CPU work for the renderer thread, never performed under acquisition locks."""
    info, summaries, raw = history.snapshot(lo_s, hi_s)
    if info.origin_us is None:
        return info, [(np.empty(0), np.empty(0)) for _ in range(6)]
    summary = np.concatenate(summaries) if summaries else np.empty(0, SUMMARY)
    exact = np.concatenate(raw) if raw else np.empty(0, RAW)
    cutoff = info.raw_start_us if info.raw_start_us is not None else info.latest_us+1
    summary = summary[summary["start"] < cutoff]
    exact = exact[(exact["t"] >= cutoff) & exact["valid"]]
    series = []
    for axis in range(6):
        ts = summary["t"][:, :, axis]
        vs = summary["v"][:, :, axis]
        order = np.argsort(ts, axis=1, kind="stable")
        st = np.take_along_axis(ts, order, axis=1).ravel()
        sv = np.take_along_axis(vs, order, axis=1).ravel()
        ss = np.repeat(summary["segment"], 4)
        older = st < cutoff
        st, sv, ss = st[older], sv[older], ss[older]
        # A frozen interval must not be bridged to the current recent ring.
        rawseg = exact["segment"].copy()
        if info.warning and len(rawseg) and len(ss):
            rawseg += int(ss.max())+1
        t = np.r_[st, exact["t"]]
        v = np.r_[sv, exact["v"][:, axis]].astype(np.float64)
        seg = np.r_[ss, rawseg]
        # Subtract integer origin BEFORE float conversion.
        relative = (t.astype(np.int64)-info.origin_us)/1e6
        x, y = envelope(relative, v, seg, lo_s, hi_s, pixels)
        series.append((x, y*(24/32768 if axis < 3 else 2000/32768)))
    return info, series
