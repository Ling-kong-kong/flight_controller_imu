"""Responsive layout checks, without hardware or an interactive desktop."""
import os
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
from pathlib import Path
import unittest
import threading
from types import SimpleNamespace
from unittest.mock import patch
from PySide6 import QtCore, QtGui, QtWidgets
from imu_viewer.attitude_history import AttitudeHistory
from imu_viewer.gui import Window, StereoPreview, APP_STYLESHEET
import numpy as np


class LayoutTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        # The offscreen Qt plugin on Windows starts with an empty font database.
        font = Path('C:/Windows/Fonts/msyh.ttc')
        if font.exists():
            QtGui.QFontDatabase.addApplicationFont(str(font))
            cls.app.setFont(QtGui.QFont('Microsoft YaHei', 9))
        cls.app.setStyle('Fusion')
        cls.app.setStyleSheet(APP_STYLESHEET)

    def test_preview_fits_both_eyes_and_resizes_without_new_frame(self):
        preview = StereoPreview()
        image = QtGui.QPixmap(2560, 720)
        image.fill(QtGui.QColor('#36a69d'))
        preview.setPixmap(image)
        key = preview.pixmap().cacheKey()
        for stacked in (True, False):
            preview.set_stacked(stacked)
            for size in ((480, 650), (1200, 150), (200, 120)):
                preview.resize(*size)
                preview.show()
                self.app.processEvents()
                rects = preview.image_rects()
                self.assertEqual(len(rects), 2)
                self.assertEqual(sum(source.width() for _, source, _ in rects), 2560)
                for dest, source, cell in rects:
                    self.assertAlmostEqual(dest.width()/dest.height(), 1280/720)
                    self.assertEqual(source.height(), 720)
                    self.assertTrue(cell.contains(dest))
                    self.assertTrue(QtCore.QRectF(preview.rect()).contains(dest))
                self.assertEqual(preview.pixmap().cacheKey(), key)
        preview.setText('disconnected')
        self.assertTrue(preview.pixmap().isNull())
        preview.close()

    def test_responsive_layout_controls_and_collapsible_details(self):
        with patch.object(Window, 'refresh_devices'):
            window = Window()
        try:
            window.ports.addItem('COM12 · Synthetic long device description ' * 5)
            window.camera_label.setText('PnP: ' + 'Long camera name ' * 12)
            window.waveforms.manual()
            window.waveforms.set_range(2, 7)
            window.show()
            for width, height, wide in ((1440, 960, True), (1024, 720, False), (1360, 900, True)):
                window.resize(width, height)
                for _ in range(3):
                    self.app.processEvents()
                self.assertEqual((window.width(), window.height()), (width, height))
                self.assertEqual(window._wide_layout, wide)
                self.assertFalse(window.waveforms.follow)
                for chart in window.waveforms.charts:
                    np.testing.assert_allclose(chart.view.viewRange()[0], (2, 7), atol=1e-8)
                self.assertEqual(window.preview_label.stacked, wide)
                self.assertEqual(window.waveforms.chart_splitter.orientation(),
                                 QtCore.Qt.Vertical if wide else QtCore.Qt.Horizontal)
                for control in (window.ports, window.connect_button, window.disconnect_button,
                                window.start_button, window.stop_button, window.export_button,
                                window.preview_label, window.waveforms):
                    rect = QtCore.QRect(control.mapTo(window.centralWidget(), QtCore.QPoint()), control.size())
                    self.assertTrue(window.centralWidget().rect().contains(rect), (control, rect))
                self.assertFalse(window.details_panel.isVisible())
            window.settings_button.click()
            self.app.processEvents()
            self.assertFalse(window.settings_panel.isVisible())
            window.details_button.click()
            self.app.processEvents()
            self.assertTrue(window.details_panel.isVisible())
            window.layout_mode.setCurrentIndex(2)
            self.app.processEvents()
            self.assertFalse(window.preview_label.stacked)
            window.layout_mode.setCurrentIndex(1)
            self.app.processEvents()
            self.assertTrue(window.preview_label.stacked)
            self.assertEqual(window.waveforms.chart_splitter.orientation(), QtCore.Qt.Vertical)
        finally:
            window.close()
            self.app.processEvents()

    def test_error_visible_after_disconnect_with_details_collapsed(self):
        with patch.object(Window, 'refresh_devices'):
            window = Window()
        try:
            snapshot = dict(camera_frames=0, imu_frames=0, elapsed_s=1, flags={},
                            crc_errors=0, missing_sequences=0, error='USB disconnected')
            done = threading.Event()
            done.set()
            window.connection = SimpleNamespace(
                snapshot=lambda **kw: snapshot, lock=threading.Lock(),
                latest_image=None, done=done, attitude_history=AttitudeHistory(),
                stop_event=done, error=snapshot['error'])
            window.next_diag = float('inf')
            window.show()
            window.tick()
            self.app.processEvents()
            self.assertIsNone(window.connection)
            self.assertTrue(window.error_banner.isVisible())
            self.assertIn('USB disconnected', window.error_banner.text())
            self.assertFalse(window.details_panel.isVisible())
        finally:
            window.connection = None
            window.close()
            self.app.processEvents()

    def test_synthetic_layout_screenshots(self):
        with patch.object(Window, 'refresh_devices'):
            window = Window()
        try:
            # Checkerboard and labels make distortion/cropping visible in saved evidence.
            image = QtGui.QPixmap(2560, 720)
            image.fill(QtGui.QColor('#153e51'))
            painter = QtGui.QPainter(image)
            for eye, color in enumerate(('#21647c', '#245b4f')):
                painter.fillRect(eye*1280, 0, 1280, 720, QtGui.QColor(color))
                painter.setPen(QtGui.QColor('#62aba7'))
                for x in range(0, 1281, 80):
                    painter.drawLine(eye*1280+x, 0, eye*1280+x, 720)
                for y in range(0, 721, 80):
                    painter.drawLine(eye*1280, y, (eye+1)*1280, y)
                painter.setPen(QtGui.QColor('white'))
                painter.setFont(QtGui.QFont('Microsoft YaHei', 36))
                painter.drawText(QtCore.QRect(eye*1280, 0, 1280, 720), QtCore.Qt.AlignCenter,
                                 f'CAM {eye}  /  1280 x 720')
            painter.end()
            window.preview_label.setPixmap(image)
            t = np.linspace(0, 10, 1000)
            for chart in window.waveforms.charts:
                for axis, curve in enumerate(chart.curves):
                    curve.setData(t, .03*np.sin(t*(axis+1)*4)+(1 if axis == 2 else 0))
            window.waveforms.state.setText('实时跟随 | 当前 0.000–10.000 s | 总时长 10.000 s\n10,000 样本 | 完整样本 | 历史内存 0.6 MiB')
            window.status.setText('实时预览 · IMU 1000 Hz / 相机 30 fps | CRC 0 / 缺号 0\n质量标志 session_start:1')
            window.ports.addItem('COM12 · Synthetic H743')
            window.show()
            parent = Path(__file__).resolve().parents[1]/'test-results'
            parent.mkdir(exist_ok=True)
            for width, height in ((1360, 900), (1024, 720)):
                window.resize(width, height)
                for _ in range(4):
                    self.app.processEvents()
                self.assertTrue(window.grab().save(str(parent/f'layout-{width}x{height}.png')))
        finally:
            window.close()
            self.app.processEvents()
