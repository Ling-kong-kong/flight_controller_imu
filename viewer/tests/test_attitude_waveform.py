"""Attitude uses the shared interactive, full-connection waveform implementation."""
import os
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
import time
import unittest
import numpy as np
from PySide6 import QtCore, QtGui, QtWidgets, QtTest
from imu_viewer.attitude import AttitudePanel
from imu_viewer.attitude_history import AttitudeHistory
from imu_viewer.history import render_history
from test_attitude import frame


class AttitudeWaveformHistoryTests(unittest.TestCase):
    def test_full_connection_float_history_and_real_extreme_times(self):
        h = AttitudeHistory()
        for start in range(0, 22000, 1000):
            h.append_frames([frame(i, roll_deg=12.345 if i == 11 else -2.125,
                                   pitch_deg=-7.625 if i == 12 else .125,
                                   yaw_deg=.03125, qw=.5, qx=-.5, qy=.5, qz=-.5)
                             for i in range(start, start+1000)])
        info, series = render_history(h.waveforms, 0, 1, 1000)
        self.assertEqual(info.samples, 22000)
        self.assertAlmostEqual(info.duration, 21.999)
        self.assertEqual(len(series), 7)
        x, y = series[0]
        self.assertEqual(x[np.argmax(y)], .011)
        self.assertEqual(y.max(), 12.345)
        x, y = series[1]
        self.assertEqual(x[np.argmin(y)], .012)
        self.assertEqual(y.min(), -7.625)
        for axis, value in enumerate([.03125, .5, -.5, .5, -.5], 2):
            np.testing.assert_allclose(series[axis][1], value)
        self.assertLess(info.allocated_bytes, 128*1024**2)

    def test_invalid_attitude_quality_gaps_and_wrap_are_not_joined(self):
        h = AttitudeHistory()
        h.append_frames([frame(0, flags=2048|2|32), frame(1, flags=1024),
                         frame(2), frame(4), frame(5, yaw_deg=179), frame(6, yaw_deg=-179),
                         frame(7, qw=-1), frame(8, protocol_version=2)])
        _, series = render_history(h.waveforms, 0, .02)
        for x, y in series:
            self.assertTrue(np.isnan(y).any())
            self.assertNotIn(.001, x)
            self.assertNotIn(.008, x)
        rows = h.waveforms.raw.active[:h.waveforms.raw.used]
        self.assertEqual(rows['valid'].tolist(), [True, False, True, True, True, True, True, False])
        self.assertNotEqual(rows['segment'][4], rows['segment'][5])
        self.assertNotEqual(rows['segment'][5], rows['segment'][6])

    def test_freeze_warns_without_losing_recent_attitude(self):
        h = AttitudeHistory(max_bytes=150000)
        for start in range(0, 20000, 1000):
            h.append_frames([frame(i, roll_deg=.125) for i in range(start, start+1000)])
        self.assertIsNotNone(h.waveforms.info().warning)
        self.assertEqual(h.waveforms.info().samples, 20000)
        _, series = render_history(h.waveforms, 19, 20)
        self.assertAlmostEqual(series[0][0][-1], 19.999)
        np.testing.assert_allclose(series[0][1], .125)


class AttitudeWaveformWidgetTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def setUp(self):
        self.panel = AttitudePanel()
        self.panel.resize(1200, 760)
        self.panel.show()
        self.history = AttitudeHistory()
        self.history.append_frames([frame(i, roll_deg=12.25, qw=.5, qx=-.5, qy=.5, qz=-.5)
                                    for i in range(22000)])
        self.panel.refresh(self.history, True)
        self.deck = self.panel.waveforms
        self.wait_for(lambda: self.deck.charts[0].curves[0].getData()[0] is not None)

    def tearDown(self):
        self.panel.close()
        self.app.processEvents()
        self.assertFalse(self.deck.worker.thread.is_alive())

    def pump(self, seconds=.08):
        until = time.perf_counter()+seconds
        while time.perf_counter() < until:
            self.app.processEvents()
            time.sleep(.002)

    def wait_for(self, predicate, timeout=2.):
        # Rendering is intentionally asynchronous. Wait for the observed result,
        # not a fixed sleep that can expire under full-suite machine load.
        until = time.perf_counter()+timeout
        while not predicate() and time.perf_counter() < until:
            self.pump(.01)
        self.assertTrue(predicate(), "Timed out waiting for waveform render")

    def double_click(self, chart):
        pos = chart.plot.mapFromScene(chart.view.sceneBoundingRect().center())
        QtTest.QTest.mouseDClick(chart.plot.viewport(), QtCore.Qt.LeftButton, pos=pos)
        self.pump()

    def wheel(self, chart, shift=False):
        pos = chart.plot.mapFromScene(chart.view.sceneBoundingRect().center())
        viewport = chart.plot.viewport()
        event = QtGui.QWheelEvent(QtCore.QPointF(pos), QtCore.QPointF(viewport.mapToGlobal(pos)),
            QtCore.QPoint(), QtCore.QPoint(0, 120), QtCore.Qt.NoButton,
            QtCore.Qt.ShiftModifier if shift else QtCore.Qt.NoModifier, QtCore.Qt.NoScrollPhase, False)
        QtWidgets.QApplication.sendEvent(viewport, event)
        self.pump()

    def test_seven_curves_real_units_and_common_controls(self):
        a, q = self.deck.charts
        self.assertEqual(len(a.curves), 3)
        self.assertEqual(len(q.curves), 4)
        self.assertEqual([c.name() for c in a.curves], ['roll', 'pitch', 'yaw'])
        self.assertEqual([c.name() for c in q.curves], ['qw', 'qx', 'qy', 'qz'])
        np.testing.assert_allclose(a.curves[0].getData()[1], 12.25)
        np.testing.assert_allclose(q.curves[3].getData()[1], -.5)
        buttons = {b.text() for b in self.deck.findChildren(QtWidgets.QPushButton)}
        self.assertTrue({'回到实时', '查看全程', '纵轴自动', '纵轴范围…'}.issubset(buttons))

    def test_history_navigation_links_ranges_and_refresh_preserves_manual_view(self):
        a, q = self.deck.charts
        self.deck.all_history()
        self.deck.set_range(1, 3)
        self.deck.time_span_spin.setValue(.2)
        np.testing.assert_allclose(a.view.viewRange()[0], [1.9, 2.1])
        self.wheel(q)
        self.assertFalse(self.deck.follow)
        before = a.view.viewRange()[0].copy()
        self.history.append_frames([frame(22000)])
        self.panel.refresh(self.history, True)
        self.pump(.15)
        np.testing.assert_allclose(a.view.viewRange()[0], before)
        np.testing.assert_allclose(q.view.viewRange()[0], before)
        self.assertIn('历史查看', self.deck.state.text())
        self.wheel(q, shift=True)
        self.assertEqual(q.y_mode.text(), '纵轴手动')
        self.assertEqual(a.y_mode.text(), '纵轴自动')
        self.deck.live()
        self.assertEqual(q.y_mode.text(), '纵轴自动')
        np.testing.assert_allclose(a.view.viewRange()[0], [21.8, 22.])
        self.deck.set_y_ranges([(-20, 20), (-1.1, 1.1)])
        self.deck.all_history()
        self.deck.live()
        np.testing.assert_allclose(q.view.viewRange()[1], [-1.1, 1.1])
        self.deck.auto_y()
        self.assertEqual(q.y_mode.text(), '纵轴自动')
        self.deck.all_history()
        self.pump()
        self.assertIn('分段峰值', self.deck.state.text())
        self.wait_for(lambda: a.curves[0].getData()[0] is not None
                      and len(a.curves[0].getData()[0]) > 0
                      and a.curves[0].getData()[0][0] <= .001)

    def test_double_click_normal_windows_actions_escape_restore_and_shutdown(self):
        a, q = self.deck.charts
        self.deck.all_history()
        self.deck.set_range(1, 2)
        for chart in (a, q):
            self.double_click(chart)
            window = self.deck.expanded[chart]
            self.assertIs(window.centralWidget(), chart)
            self.assertFalse(window.isFullScreen())
            self.assertFalse(window.isMaximized())
            self.assertTrue(window.windowFlags() & QtCore.Qt.WindowMinimizeButtonHint)
            self.assertTrue(window.windowFlags() & QtCore.Qt.WindowMaximizeButtonHint)
            toolbar = window.findChild(QtWidgets.QToolBar)
            self.assertEqual([act.text() for act in toolbar.actions() if act.text()],
                             ['返回主窗口', '回到实时', '查看全程', '纵轴自动', '纵轴范围…'])
            window.time_span_spin.setValue(.5)
            self.assertEqual(self.deck.time_span_spin.value(), .5)
            QtTest.QTest.keyClick(window, QtCore.Qt.Key_Escape)
            self.pump()
            self.assertNotIn(chart, self.deck.expanded)
            np.testing.assert_allclose(chart.view.viewRange()[0], [1.25, 1.75])
        self.double_click(q)
        self.double_click(q)
        self.assertNotIn(q, self.deck.expanded)
        self.double_click(a)
        self.deck.expanded[a].close()
        self.pump()
        self.assertNotIn(a, self.deck.expanded)
        self.double_click(q)
        self.panel.shutdown()
        self.assertFalse(self.deck.expanded)

    def test_shutdown_disposes_gui_graphs_before_background_collection(self):
        from shiboken6 import isValid
        self.deck.show_ranges()
        self.double_click(self.deck.charts[0])
        self.panel.shutdown()
        self.panel.shutdown()  # Main window and child close may both call it.
        self.app.sendPostedEvents(None, QtCore.QEvent.DeferredDelete)
        self.app.processEvents()
        self.assertFalse(self.deck.worker.thread.is_alive())
        for chart in self.deck.charts:
            self.assertFalse(isValid(chart))
            self.assertFalse(isValid(chart.plot))
            self.assertFalse(isValid(chart.view))
        # In-flight queued render results must not touch the deleted graphs.
        self.deck.rendered((self.deck.generation, self.deck.version, None,
                            [(np.array([0.]), np.array([1.]))]*7, None))
        self.deck.refresh()

    def test_disconnect_keeps_history_new_connection_resets(self):
        self.deck.all_history()
        self.deck.set_range(1, 2)
        generation = self.deck.generation
        self.panel.refresh(connected=False)
        self.assertIs(self.deck.history, self.history.waveforms)
        self.assertEqual(self.deck.generation, generation)
        self.assertFalse(self.deck.follow)
        new = AttitudeHistory()
        new.append_frames([frame()])
        self.panel.refresh(new, True)
        self.assertIs(self.deck.history, new.waveforms)
        self.assertTrue(self.deck.follow)
        self.assertEqual(len(self.deck.charts[1].curves), 4)
        self.assertEqual(self.deck.history.info().samples, 1)


if __name__ == '__main__':
    unittest.main()
