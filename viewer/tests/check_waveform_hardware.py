"""Opt-in short real-device check. Opens camera/CDC and writes a short recording.

Run from viewer: .venv\\Scripts\\python.exe tests/check_waveform_hardware.py --run
Qt runs offscreen with real hardware; this is not a physical mouse/display test.
"""
import argparse
import dataclasses
import json
import os
from pathlib import Path
import sys
import time
from unittest.mock import patch
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from PySide6 import QtCore, QtGui, QtWidgets, QtTest
from imu_viewer.gui import Window
from imu_viewer.verify import verify_session


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", action="store_true", help="open real devices and record")
    args = parser.parse_args()
    if not args.run:
        parser.error("Requires --run; opens real devices and saves a short recording")
    root = Path(__file__).resolve().parents[1]/"test-results"
    output = root/("waveform-hardware-"+time.strftime("%Y%m%d-%H%M%S"))
    output.mkdir(parents=True)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    app.setStyle("Fusion")
    window = Window()
    window.show()
    report = dict(kind="short_real_hardware_offscreen_QTest_not_long_acceptance",
                  output=str(output), checks=[])
    connection = recorder = None

    def pump(seconds):
        deadline = time.perf_counter()+seconds
        while time.perf_counter() < deadline:
            app.processEvents()
            time.sleep(.005)

    def until(predicate, timeout, message):
        deadline = time.perf_counter()+timeout
        while not predicate() and time.perf_counter() < deadline:
            pump(.02)
        if not predicate():
            raise AssertionError(message)

    def check(condition, label):
        if not condition:
            raise AssertionError(label)
        report["checks"].append(label)

    def message_error(*args):
        raise RuntimeError(str(args[-1]))

    def doubleclick(chart):
        pos = chart.plot.mapFromScene(chart.view.sceneBoundingRect().center())
        QtTest.QTest.mouseDClick(chart.plot.viewport(), QtCore.Qt.LeftButton, pos=pos)
        pump(.2)

    try:
        with patch.object(QtWidgets.QMessageBox,"critical", side_effect=message_error), \
             patch.object(QtWidgets.QMessageBox,"warning", side_effect=message_error):
            until(lambda: window.ports.count()>0, 10, "no devices enumerated")
            index = next((i for i in range(window.ports.count())
                          if window.ports.itemData(i)["h743"]), None)
            check(index is not None, "H743 identified by USB identity, not fixed COM number")
            window.ports.setCurrentIndex(index)
            report["device"] = window.ports.currentData()
            window.output.setText(str(output))
            window.connect_devices()
            connection = window.connection
            until(lambda: connection.error or (window.waveforms.history is connection.history
                  and len(window.accel_chart.samples)>0), 20, "no preview/curves")
            check(connection.error is None, "real camera and IMU started")
            check(not window.preview_label.pixmap().isNull(), "camera preview without recording")
            check(not list(output.iterdir()), "preview created no files")
            pump(12)
            history = connection.history
            check(history.info().duration>10, "unrecorded connection history exceeds exact 10-second window")
            window.waveforms.all_history()
            pump(.4)
            check(not window.waveforms.follow, "all-history pauses following, not acquisition")
            check(window.accel_chart.samples[0]<1, "early pre-recording waveform remains visible")
            window.grab().save(str(output/"history.png"))
            window.start_recording()
            recorder = window.recorder
            check(recorder is not None, "real recording started")
            count = history.info().samples
            images = connection.metrics["camera_frames"]
            doubleclick(window.accel_chart)
            expanded = window.waveforms.expanded[window.accel_chart]
            check(expanded.isFullScreen(), "double-click moved chart into full-screen window")
            for right in (False, True):
                expanded.half(right)
                pump(.4)
                check(abs(expanded.width()-expanded.screen().availableGeometry().width()/2)<3,
                      "right half-screen" if right else "left half-screen")
            chart = window.accel_chart
            pos = chart.plot.mapFromScene(chart.view.sceneBoundingRect().center())
            before = chart.view.viewRange()[0].copy()
            wheel = QtGui.QWheelEvent(QtCore.QPointF(pos),
                QtCore.QPointF(chart.plot.viewport().mapToGlobal(pos)), QtCore.QPoint(),
                QtCore.QPoint(0,120), QtCore.Qt.NoButton, QtCore.Qt.NoModifier,
                QtCore.Qt.NoScrollPhase,False)
            QtWidgets.QApplication.sendEvent(chart.plot.viewport(),wheel)
            pump(.2)
            after = chart.view.viewRange()[0]
            check(after[1]-after[0]<before[1]-before[0], "wheel zoom in expanded chart")
            check(all(abs(a-b)<1e-8 for a,b in zip(after,window.gyro_chart.view.viewRange()[0])),
                  "expanded and main charts share X range")
            expanded.grab().save(str(output/"expanded.png"))
            pump(3)
            check(history.info().samples>count and connection.metrics["camera_frames"]>images,
                  "IMU and camera continue during expanded-window interaction")
            check(recorder.metrics["camera_frames"]>0 and recorder.metrics["imu_frames"]>0,
                  "recording receives both streams while chart expanded")
            QtTest.QTest.keyClick(expanded,QtCore.Qt.Key_Escape)
            pump(.3)
            check(not window.waveforms.expanded, "Esc restores chart")
            doubleclick(window.gyro_chart)
            doubleclick(window.gyro_chart)
            check(not window.waveforms.expanded, "double-click again restores chart")
            doubleclick(window.accel_chart)
            window.waveforms.expanded[window.accel_chart].close()
            pump(.2)
            check(not window.waveforms.expanded, "closing expanded window restores chart")
            window.waveforms.live()
            pump(.3)
            check(window.waveforms.follow, "return to live resumes following")
            window.stop_recording()
            until(lambda: window.recorder is None, 30, "recording did not drain")
            check(recorder.error is None, "recording drained without error")
            check(window.waveforms.history is history, "record start/stop preserves connection history")
            report["recording"] = recorder.snapshot()
            before = history.info().samples
            pump(.5)
            check(history.info().samples>before, "preview and history continue after recording stops")
            window.disconnect_devices()
            until(lambda: window.connection is None, 20, "connection did not close")
            check(window.waveforms.history is history, "disconnect retains history")
            report["connection"] = connection.snapshot()
            report["history"] = dataclasses.asdict(history.info())
            report["verification"] = verify_session(recorder.path,decode_all=True)
            window.grab().save(str(output/"disconnected.png"))
            report["passed"] = True
    except Exception as exc:
        report["passed"] = False
        report["error"] = repr(exc)
    finally:
        if connection is not None:
            connection.stop_recording()
            connection.stop()
            connection.wait(30)
        window.close()
        pump(.5)
        (output/"report.json").write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding="utf8")
        print(json.dumps(report,ensure_ascii=False,indent=2))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
