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
