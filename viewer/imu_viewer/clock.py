"""One-way lower-envelope clock estimates, NOT exposure synchronization."""
from dataclasses import dataclass, asdict
import numpy as np

WARNING = "软件估算对齐，曝光同步误差未验证"

@dataclass(frozen=True)
class ClockModel:
    device_origin_ns: int
    host_origin_ns: int
    offset_ns: float
    slope: float
    device_min_ns: int
    device_max_ns: int
    observations: int
    residual_ns_p05: float
    residual_ns_p50: float
    residual_ns_p95: float
    envelope_points: int

    def to_host(self, device_ns):
        return self.host_origin_ns + round(
            self.offset_ns + self.slope * (int(device_ns) - self.device_origin_ns))

    def from_host(self, host_ns):
        return self.device_origin_ns + round(
            (int(host_ns) - self.host_origin_ns - self.offset_ns) / self.slope)

    def contains(self, device_ns):
        return self.device_min_ns <= device_ns <= self.device_max_ns

    def report(self):
        return dict(asdict(self), drift_ppm=(self.slope - 1) * 1e6,
                    method="one-way binned lower-envelope affine fit",
                    host_clock="perf_counter_ns",
                    fixed_delay_identifiable=False,
                    exposure_accuracy_verified=False,
                    residual_is_sync_accuracy=False)

def fit_clock(observations):
    """Each pair is (device_ns, host read-complete ns); batch endpoints for USB.

    Integer anchors avoid catastrophic subtraction for large uint64 MCU times.
    Low residual points in time bins approximate minimum delivery latency.
    This cannot identify fixed latency, sensor delay, or clock-domain identity.
    """
    pairs = list(observations)
    if len(pairs) < 20:
        raise ValueError("Need at least 20 clock observations")
    d0, h0 = map(int, pairs[0])
    ds = [int(p[0]) for p in pairs]
    hs = [int(p[1]) for p in pairs]
    if any(b <= a for a, b in zip(ds, ds[1:])):
        raise ValueError("Device timestamps must strictly increase")
    if any(b < a for a, b in zip(hs, hs[1:])):
        raise ValueError("Host clock regressed")
    x = np.array([d - d0 for d in ds], dtype=np.float64)
    y = np.array([h - h0 for h in hs], dtype=np.float64)
    if x[-1] < 2e9:
        raise ValueError("Need at least two seconds of clock span")
    # 8..120 bins; minimum of detrended delivery delay, not minimum timestamp.
    bin_count = min(120, max(8, len(x) // 20))
    bins = np.minimum((x / x[-1] * bin_count).astype(int), bin_count - 1)
    slope = float((y[-1] - y[0]) / (x[-1] - x[0]))
    chosen = []
    for _ in range(4):
        residual = y - slope * x
        chosen = [idx[np.argmin(residual[idx])]
                  for b in np.unique(bins) if len(idx := np.flatnonzero(bins == b))]
        xx, yy = x[chosen], y[chosen]
        xc, yc = xx.mean(), yy.mean()
        denom = np.sum((xx - xc) ** 2)
        if denom == 0:
            raise ValueError("Insufficient clock baseline")
        slope = float(np.sum((xx - xc) * (yy - yc)) / denom)
        offset = float(yc - slope * xc)
    if not np.isfinite(slope) or not 0.98 < slope < 1.02:
        raise ValueError("Clock slope outside conservative ±2% validity range")
    residual = y - (offset + slope * x)
    q = np.percentile(residual, [5, 50, 95])
    return ClockModel(d0, h0, offset, slope, ds[0], ds[-1], len(ds),
                      *map(float, q), len(chosen))

def align_camera(camera_ns, camera_model, imu_model):
    if not camera_model.contains(camera_ns):
        raise ValueError("Camera time outside fitted interval")
    imu_ns = imu_model.from_host(camera_model.to_host(camera_ns))
    if not imu_model.contains(imu_ns):
        raise ValueError("Outside common fitted clock interval")
    return imu_ns
