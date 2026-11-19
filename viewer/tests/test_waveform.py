"""Offscreen Qt input, linked views and expanded-window lifecycle tests."""
import os
os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
import time
import unittest
from unittest.mock import patch
import numpy as np
from PySide6 import QtCore, QtGui, QtWidgets, QtTest
from imu_viewer.history import WaveformHistory
from imu_viewer.waveform import WaveformPanel, RenderWorker
from test_history import feed


class WaveformTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def setUp(self):
        self.panel = WaveformPanel()
        self.panel.resize(1200,600)
        self.panel.show()
        self.history = WaveformHistory()
        feed(self.history,0,31000)
        self.panel.set_history(self.history)
        self.pump(.15)

    def tearDown(self):
        self.panel.shutdown()
        self.panel.close()
        self.app.processEvents()

    def pump(self,seconds=.05):
        end = time.perf_counter()+seconds
        while time.perf_counter() < end:
            self.app.processEvents()
            time.sleep(.002)

    def wheel(self,chart,shift=False,axis=False):
        view = chart.view
        rect = view.sceneBoundingRect()
        scene = QtCore.QPointF(rect.left()-15 if axis else rect.left()+rect.width()*.25, rect.center().y())
        pos = chart.plot.mapFromScene(scene)
        viewport = chart.plot.viewport()
        event = QtGui.QWheelEvent(QtCore.QPointF(pos),QtCore.QPointF(viewport.mapToGlobal(pos)),
            QtCore.QPoint(),QtCore.QPoint(0,120),QtCore.Qt.NoButton,
            QtCore.Qt.ShiftModifier if shift else QtCore.Qt.NoModifier,QtCore.Qt.NoScrollPhase,False)
        QtWidgets.QApplication.sendEvent(viewport,event)
        self.app.processEvents()

    def test_custom_time_span_in_live_and_history(self):
        spin = self.panel.time_span_spin
        spin.setValue(.1)
        self.assertTrue(self.panel.follow)
        for chart in self.panel.charts:
            np.testing.assert_allclose(chart.view.viewRange()[0], [30.899, 30.999])
        feed(self.history, 31000, 1000)
        self.panel.refresh()
        np.testing.assert_allclose(self.panel.accel_chart.view.viewRange()[0], [31.899, 31.999])
        self.panel.all_history()
        self.panel.set_range(3, 7)
        spin.setValue(.2)
        self.assertFalse(self.panel.follow)
        for chart in self.panel.charts:
            np.testing.assert_allclose(chart.view.viewRange()[0], [4.9, 5.1])
        self.wheel(self.panel.accel_chart)
        self.assertAlmostEqual(spin.value(), .2)  # Navigation does not change live preset.
        self.panel.live()
        np.testing.assert_allclose(self.panel.accel_chart.view.viewRange()[0], [31.799, 31.999])

    def test_fixed_y_ranges_survive_live_and_history_until_auto_or_reconnect(self):
        a, g = self.panel.charts
        self.panel.set_y_ranges([(.98, 1.02), (-.5, .5)])
        self.assertTrue(self.panel.follow)
        for chart, bounds in zip((a, g), ((.98, 1.02), (-.5, .5))):
            np.testing.assert_allclose(chart.view.viewRange()[1], bounds)
            self.assertEqual(chart.y_mode.text(), "纵轴手动（固定）")
        self.panel.all_history()
        self.panel.live()
        feed(self.history, 31000, 1000)
        self.panel.refresh()
        self.pump(.15)
        np.testing.assert_allclose(a.view.viewRange()[1], [.98, 1.02])
        np.testing.assert_allclose(g.view.viewRange()[1], [-.5, .5])
        self.assertTrue(self.panel.follow)
        with self.assertRaises(ValueError):
            self.panel.set_y_ranges([(0, 2), (1, 1)])
        np.testing.assert_allclose(a.view.viewRange()[1], [.98, 1.02])
        self.panel.auto_y()
        for chart in (a, g):
            self.assertEqual(chart.y_mode.text(), "纵轴自动")
        self.panel.set_y_ranges([(-1, 1), None])
        self.panel.set_history(WaveformHistory())
        self.assertEqual(a.y_mode.text(), "纵轴自动")

    def test_range_controls_in_expanded_window_and_dialog_validation(self):
        a, g = self.panel.charts
        self.double_click(g)
        window = self.panel.expanded[g]
        window.time_span_spin.setValue(.5)
        self.assertEqual(self.panel.time_span_spin.value(), .5)
        self.assertTrue(self.panel.follow)
        action = next(action for action in window.findChildren(QtGui.QAction)
                      if action.text() == "纵轴范围…")
        action.trigger()
        dialog = self.panel.range_dialog
        self.assertTrue(dialog.isVisible())
        self.assertFalse(dialog.isModal())
        for fixed, low, high in dialog.fields:
            fixed.setChecked(True)
            low.setValue(-.25)
            high.setValue(.25)
        dialog.fields[1][2].setValue(-.5)
        apply = dialog.buttons.button(QtWidgets.QDialogButtonBox.Apply)
        apply.click()
        self.assertIn("下限必须小于上限", dialog.error.text())
        self.assertEqual(a.y_mode.text(), "纵轴自动")
        self.assertTrue(self.panel.follow)
        dialog.fields[1][2].setValue(.5)
        apply.click()
        self.assertEqual(dialog.error.text(), "")
        np.testing.assert_allclose(a.view.viewRange()[1], [-.25, .25])
        np.testing.assert_allclose(g.view.viewRange()[1], [-.25, .5])
        dialog.close()
        window.close()
        self.panel.live()
        self.assertEqual(a.y_mode.text(), "纵轴手动（固定）")
        self.panel.time_span_spin.setValue(.1)
        self.double_click(g)
        self.assertEqual(self.panel.expanded[g].time_span_spin.value(), .1)
        # Main-window range control also opens the same settings safely.
        self.panel.expanded[g].close()
        button = next(b for b in self.panel.findChildren(QtWidgets.QPushButton)
                      if b.text() == "纵轴范围…")
        button.click()
        self.assertTrue(dialog.isVisible())
        dialog.fields[0][0].setChecked(False)
        apply.click()
        self.assertEqual(a.y_mode.text(), "纵轴自动")
        self.assertEqual(g.y_mode.text(), "纵轴手动（固定）")
        self.panel.shutdown()
        self.assertFalse(dialog.isVisible())

    def test_cursor_time_zoom_link_independent_y_and_live_all(self):
        a,g = self.panel.charts
        oldx = np.array(a.view.viewRange()[0])
        oldy = np.array(g.view.viewRange()[1])
        anchor = oldx[0]+np.diff(oldx)[0]*.25
        self.wheel(a)
        newx = a.view.viewRange()[0]
        self.assertFalse(self.panel.follow)
        self.assertLess(newx[1]-newx[0],np.diff(oldx)[0])
        self.assertAlmostEqual(newx[0]+(newx[1]-newx[0])*.25,anchor,delta=.05)
        np.testing.assert_allclose(newx,g.view.viewRange()[0])
        before = np.array(a.view.viewRange()[1])
        self.wheel(a,shift=True)
        self.assertLess(np.diff(a.view.viewRange()[1])[0],np.diff(before)[0])
        np.testing.assert_allclose(oldy,g.view.viewRange()[1])
        self.assertFalse(a.view.state['autoRange'][1])
        self.panel.auto_y()
        self.assertTrue(a.view.state['autoRange'][1])
        self.panel.all_history()
        np.testing.assert_allclose(a.view.viewRange()[0],[0,30.999])
        self.assertFalse(self.panel.follow)
        self.assertIn('分段峰值',self.panel.state.text())
        self.panel.live()
        np.testing.assert_allclose(a.view.viewRange()[0],[20.999,30.999])
        self.assertTrue(self.panel.follow)

    def test_time_wheel_keeps_y_auto_and_fits_later_gyro_peaks(self):
        # Exercise the real viewport wheel route, then live acquisition/rendering.
        # A time-only zoom must not freeze the gyro's current amplitude range.
        a, g = self.panel.charts
        initial = np.array(g.view.viewRange()[1])
        self.wheel(g)
        self.panel.live()
        values = np.zeros((1000, 6), dtype=np.int16)
        values[:, 2] = 1365  # Approximately 1 g, independent of gyro magnitude.
        values[:, 3:] = np.where(np.arange(1000)[:, None] % 2, 1000, -1000)
        feed(self.history, 31000, 1000, values=values)
        self.panel.refresh()
        deadline = time.perf_counter() + 2
        while time.perf_counter() < deadline:
            self.pump(.02)
            if len(g.samples) and g.samples[-1] >= 31.999:
                break
        self.assertGreaterEqual(g.samples[-1], 31.999)
        # setData marks auto-range dirty; fitting happens on the next Qt paint,
        # not synchronously with the background result's arrival.
        self.pump(.1)
        low, high = g.view.viewRange()[1]
        peak = 1000 * 2000 / 32768
        self.assertLessEqual(low, -peak, f"clipped gyro: old={initial}, now={[low, high]}")
        self.assertGreaterEqual(high, peak)
        self.assertTrue(g.view.state['autoRange'][1])
        self.assertTrue(a.view.state['autoRange'][1])
        # This is display-only: the data keeps its nominal degrees/s conversion.
        for curve in g.curves:
            y = curve.getData()[1]
            self.assertAlmostEqual(np.nanmax(y), peak)
            self.assertAlmostEqual(np.nanmin(y), -peak)

    def test_time_wheel_preserves_each_charts_y_mode(self):
        a, g = self.panel.charts
        for chart in (a, g):
            with self.subTest(chart=chart.title, mode='auto'):
                self.panel.auto_y()
                self.wheel(chart)
                self.assertTrue(chart.view.state['autoRange'][1])
            with self.subTest(chart=chart.title, mode='manual'):
                self.wheel(chart, shift=True)
                before = np.array(chart.view.viewRange()[1])
                self.wheel(chart)
                self.assertFalse(chart.view.state['autoRange'][1])
                np.testing.assert_allclose(chart.view.viewRange()[1], before)

    def test_live_restores_both_y_axes_and_visible_mode_labels(self):
        a, g = self.panel.charts
        for chart in (a, g):
            self.assertEqual(chart.y_mode.text(), "纵轴自动")
            self.wheel(chart, shift=True)
            self.assertEqual(chart.y_mode.text(), "纵轴手动")
        live = next(b for b in self.panel.findChildren(QtWidgets.QPushButton)
                    if b.text() == "回到实时")
        live.click()
        self.assertTrue(self.panel.follow)
        for chart in (a, g):
            self.assertTrue(chart.view.state['autoRange'][1])
            self.assertEqual(chart.y_mode.text(), "纵轴自动")

    def test_history_y_auto_fits_visible_data_until_explicit_y_adjustment(self):
        self.panel.all_history()
        a, g = self.panel.charts
        # The displayed series includes an off-screen peak: only visible values
        # should determine the automatic Y range when examining history.
        x = np.array([0., 1., 2., 3., 4.])
        series = [(x, np.array([1000., 0., 1., 0., 1000.])) for _ in range(6)]
        self.panel.set_range(1, 3)
        for chart in (a, g):
            chart.set_series(series)
            chart.view.updateAutoRange()
            self.assertTrue(chart.view.state['autoRange'][1])
            self.assertLess(chart.view.viewRange()[1][1], 10)
            self.assertEqual(chart.y_mode.text(), "纵轴自动")
        self.wheel(g, shift=True)
        manual_y = g.view.viewRange()[1].copy()
        self.wheel(a)
        self.panel.all_history()
        self.assertFalse(self.panel.follow)
        self.assertTrue(a.view.state['autoRange'][1])
        self.assertFalse(g.view.state['autoRange'][1])
        np.testing.assert_allclose(g.view.viewRange()[1], manual_y)
        self.assertEqual(g.y_mode.text(), "纵轴手动")
        self.panel.auto_y()
        self.assertFalse(self.panel.follow)
        self.assertEqual(g.y_mode.text(), "纵轴自动")

    def test_expanded_live_action_restores_y_modes_and_reconnect_resets_them(self):
        a, g = self.panel.charts
        self.double_click(g)
        for chart in (a, g):
            self.wheel(chart, shift=True)
        self.assertEqual(g.y_mode.text(), "纵轴手动")
        window = self.panel.expanded[g]
        action = next(a for a in window.findChildren(QtGui.QAction)
                      if a.text() == "回到实时")
        action.trigger()
        self.assertTrue(self.panel.follow)
        for chart in (a, g):
            self.assertTrue(chart.view.state['autoRange'][1])
            self.assertEqual(chart.y_mode.text(), "纵轴自动")
        window.close()
        self.wheel(a, shift=True)
        self.panel.set_history(WaveformHistory())
        self.assertTrue(self.panel.follow)
        self.assertEqual(a.y_mode.text(), "纵轴自动")

    def test_y_axis_wheel_and_left_drag(self):
        a,g = self.panel.charts
        beforex = np.array(a.view.viewRange()[0])
        beforey = np.array(a.view.viewRange()[1])
        self.wheel(a,axis=True)
        np.testing.assert_allclose(beforex,a.view.viewRange()[0])
        self.assertLess(np.diff(a.view.viewRange()[1])[0],np.diff(beforey)[0])
        self.panel.live()
        viewport = a.plot.viewport()
        center = a.plot.mapFromScene(a.view.sceneBoundingRect().center())
        QtTest.QTest.mousePress(viewport,QtCore.Qt.LeftButton,pos=center)
        QtTest.QTest.mouseMove(viewport,center+QtCore.QPoint(80,0),delay=20)
        QtTest.QTest.mouseMove(viewport,center+QtCore.QPoint(120,0),delay=20)
        QtTest.QTest.mouseRelease(viewport,QtCore.Qt.LeftButton,pos=center+QtCore.QPoint(120,0))
        self.pump()
        self.assertFalse(self.panel.follow)
        self.assertLess(a.view.viewRange()[0][0],beforex[0])
        for chart in (a, g):
            self.assertTrue(chart.view.state['autoRange'][1])
            self.assertEqual(chart.y_mode.text(), "纵轴自动")
        np.testing.assert_allclose(a.view.viewRange()[0],g.view.viewRange()[0])

    def double_click(self,chart):
        pos = chart.plot.mapFromScene(chart.view.sceneBoundingRect().center())
        QtTest.QTest.mouseDClick(chart.plot.viewport(),QtCore.Qt.LeftButton,pos=pos)
        self.pump()

    def test_expand_half_restore_doubleclick_escape_close_and_shutdown(self):
        a,g = self.panel.charts
        self.panel.all_history()
        before = a.view.viewRange()[0].copy()
        self.double_click(a)
        w = self.panel.expanded[a]
        self.assertIs(w.centralWidget(),a)
        self.assertTrue(w.isFullScreen())
        for right in (False,True):
            w.half(right)
            self.pump()
            rect = w.screen().availableGeometry()
            self.assertAlmostEqual(w.width(),rect.width()/2,delta=2)
            self.assertAlmostEqual(w.x(),rect.x()+(rect.width()//2 if right else 0),delta=3)
        self.wheel(a)
        np.testing.assert_allclose(a.view.viewRange()[0],g.view.viewRange()[0])
        preserved = a.view.viewRange()[0].copy()
        QtTest.QTest.keyClick(w,QtCore.Qt.Key_Escape)
        self.pump()
        self.assertNotIn(a,self.panel.expanded)
        self.assertIs(a.parentWidget(),self.panel.places[a].parentWidget())
        np.testing.assert_allclose(preserved,a.view.viewRange()[0])
        self.double_click(a)
        self.double_click(a)
        self.assertNotIn(a,self.panel.expanded)
        self.double_click(g)
        self.panel.expanded[g].close()
        self.pump()
        self.assertNotIn(g,self.panel.expanded)
        self.double_click(a)
        self.panel.shutdown()
        self.assertFalse(self.panel.expanded)
        self.assertFalse(self.panel.worker.thread.is_alive())

    def test_stale_results_ignored_and_old_view_not_recomputed_each_tick(self):
        self.panel.all_history()
        self.panel.set_range(0,1)
        self.panel.refresh()
        with patch.object(self.panel.worker,'request') as request:
            feed(self.history,31000,1000)
            self.panel.refresh()
            request.assert_not_called()
        generation = self.panel.generation
        self.panel.set_history(WaveformHistory())
        fake = [(np.array([99.]),np.array([99.])) for _ in range(6)]
        self.panel.rendered((generation,self.panel.version-1,None,fake,None))
        self.assertEqual(len(self.panel.accel_chart.samples),0)


if __name__ == '__main__':
    unittest.main()
