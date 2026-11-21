"""Reproducible synthetic history/Qt benchmark; no devices or recording required.

Run from viewer: .venv\\Scripts\\python.exe tests/benchmark_history.py
Results are synthetic, NOT a one-hour hardware acquisition acceptance.
"""
import os
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
import argparse
import json
from pathlib import Path
import platform
import sys
import time
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import numpy as np
import psutil
from PySide6 import QtCore, QtGui, QtWidgets
from imu_viewer.history import WaveformHistory, render_history
from imu_viewer.attitude_history import AttitudeHistory
from imu_viewer.protocol import Frame
from imu_viewer.waveform import WaveformPanel


def stats(values):
    a = np.asarray(values)*1000
    return {"count": len(a), "median_ms": float(np.median(a)),
            "p95_ms": float(np.percentile(a, 95)), "max_ms": float(a.max())}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--samples", type=int, default=3_600_000)
    parser.add_argument("--batch", type=int, default=86)
    parser.add_argument("--attitude", action="store_true", help="Benchmark seven attitude components")
    parser.add_argument("--output", default=None)
    args = parser.parse_args()
    process = psutil.Process()
    rss_initial = process.memory_info().rss
    rss_peak = rss_initial
    history = (WaveformHistory(channels=7, value_dtype="<f8", scales=[1.]*7)
               if args.attitude else WaveformHistory())
    latencies = []
    started = time.perf_counter()
    for start in range(0, args.samples, args.batch):
        ix = np.arange(start, min(start+args.batch, args.samples), dtype=np.uint64)
        t = 1_000_000+ix*1000
        if args.attitude:
            phase = ix.astype(float)*.001
            v = np.column_stack((20*np.sin(phase), 10*np.cos(phase), 40*np.sin(phase*.1),
                                 np.cos(phase*.5), np.zeros(len(ix)), np.zeros(len(ix)),
                                 np.sin(phase*.5)))
        else:
            v = (((ix[:, None]*np.array([13, 17, 19, 23, 29, 31], np.uint64))
                  % 60001).astype(np.int32)-30000).astype(np.int16)
        before = time.perf_counter()
        history.append_arrays(t, v, ix.astype(np.uint32),
                              validity=np.ones(len(ix), bool) if args.attitude else None)
        latencies.append(time.perf_counter()-before)
        if start % (args.batch*100) == 0:
            rss_peak = max(rss_peak, process.memory_info().rss)
    ingestion_s = time.perf_counter()-started
    info = history.info()
    assert info.samples == args.samples and not info.warning, info
    assert info.allocated_bytes < 128*1024**2
    render = {}
    for name, lo, hi in (("whole_hour", 0, info.duration),
                         ("recent_10s", max(0, info.duration-10), info.duration),
                         ("early_10s", 1, min(11, info.duration))):
        durations = []
        for _ in range(3):
            before = time.perf_counter()
            _, series = render_history(history, lo, hi, 1200)
            durations.append(time.perf_counter()-before)
            rss_peak = max(rss_peak, process.memory_info().rss)
        render[name] = {**stats(durations), "points_per_axis": [len(x) for x, y in series]}
    # Include conversion from actual parsed Frame objects at the USB-batch size.
    small = AttitudeHistory() if args.attitude else WaveformHistory()
    frame_latencies = []
    for start in range(0, 86_000, 86):
        attitude = (dict(flags=2048, protocol_version=3, qw=1., qx=0., qy=0., qz=0.,
                         roll_deg=0., pitch_deg=0., yaw_deg=0.) if args.attitude else {})
        frames = [Frame(i, 1_000_000+i*1000, 1, 2, 3, 4, 5, 6, 10, 10, **attitude)
                  for i in range(start, start+86)]
        before = time.perf_counter()
        small.append_frames(frames)
        frame_latencies.append(time.perf_counter()-before)

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    panel = WaveformPanel(chart_specs=(
        ("Euler", "deg", (1, 2, 3), ("roll", "pitch", "yaw")),
        ("Quaternion", "", (4, 5, 6, 7), ("qw", "qx", "qy", "qz")))) if args.attitude else WaveformPanel()
    panel.resize(1360, 520)
    panel.show()
    panel.set_history(history)
    app.processEvents()
    # Navigate whole-history repeatedly while wheel events and a 5ms GUI timer
    # run. Includes real queued render-result delivery + Qt plot painting, not
    # just the time taken to enqueue a background calculation.
    gaps, wheel_times = [], []
    last = time.perf_counter()
    ticks = 0
    def tick():
        nonlocal last, ticks, rss_peak
        now = time.perf_counter()
        gaps.append(now-last)
        last = now
        ticks += 1
        if ticks % 10 == 0:
            chart = panel.primary_chart
            pos = chart.plot.mapFromScene(chart.view.sceneBoundingRect().center())
            point = QtCore.QPointF(pos)
            event = QtGui.QWheelEvent(point, QtCore.QPointF(chart.plot.mapToGlobal(pos)),
                QtCore.QPoint(), QtCore.QPoint(0, 120 if ticks % 20 else -120),
                QtCore.Qt.NoButton, QtCore.Qt.NoModifier,
                QtCore.Qt.NoScrollPhase, False)
            before = time.perf_counter()
            QtWidgets.QApplication.sendEvent(chart.plot.viewport(), event)
            wheel_times.append(time.perf_counter()-before)
        if ticks % 20 == 0:
            panel.all_history()
        rss_peak = max(rss_peak, process.memory_info().rss)
    timer = QtCore.QTimer()
    timer.timeout.connect(tick)
    timer.start(5)
    panel.all_history()
    QtCore.QTimer.singleShot(4000, app.quit)
    app.exec()
    timer.stop()
    panel.shutdown()
    panel.close()
    report = {
        "kind": "synthetic_not_hardware_acceptance", "platform": platform.platform(),
        "python": platform.python_version(), "numpy": np.__version__,
        "mode": "attitude_7_components" if args.attitude else "raw_six_axis",
        "samples": args.samples, "rate_hz": 1000, "duration_s": info.duration,
        "batch_size": args.batch, "ingestion_wall_s": ingestion_s,
        "append_arrays": stats(latencies), "append_86_frames": stats(frame_latencies),
        "history_allocated_bytes_including_index_estimate": info.allocated_bytes,
        "rss_initial_bytes": rss_initial, "rss_observed_peak_bytes": rss_peak,
        "render": render, "qt_5ms_timer_delivery_gap": stats(gaps),
        "qt_synthetic_wheel_handler": stats(wheel_times),
        "qt_timer_max_gap_under_100ms": max(gaps) <= .1,
        "notes": ["offscreen Qt; timer gaps include event-loop work and painting",
                  "not a physical mouse or display-latency measurement",
                  "history budget excludes transient rendering arrays and Qt/process RSS",
                  "timings depend on machine load; no long hardware acceptance claimed"]}
    path = Path(args.output or ("test-results/attitude-waveform-benchmark.json" if args.attitude
                                else "test-results/waveform-benchmark.json"))
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf8")
    print(json.dumps(report, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
