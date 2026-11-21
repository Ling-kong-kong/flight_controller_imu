"""Qt front end; rendering stays independent of acquisition and saving."""
from pathlib import Path
import json
import sys
import threading
import time
import cv2
from PySide6 import QtCore, QtGui, QtWidgets
from .capture import Config, LiveConnection, BASE, discover_ports, camera_inventory
from .timing import HOST_WARNING as WARNING
from .waveform import WaveformPanel
from .attitude import AttitudePanel, AttitudeDetailsDialog
from .calibration_dialog import CalibrationProgressDialog
from .workflow import validate_output_root, resolve_raw_session

APP_STYLESHEET = """
        QWidget { background:#182536; color:#dce5f2; font-size:13px; }
        QLineEdit,QComboBox,QSpinBox,QDoubleSpinBox,QPlainTextEdit {
            background:#101b2b; border:1px solid #3b4a60; border-radius:4px; padding:5px;
        }
        QPushButton { background:#30465b; border:0; border-radius:4px; padding:7px 10px; }
        QPushButton:disabled { color:#68778b; background:#233246; }
        QPushButton:hover { background:#3d637a; }
    """

class Bridge(QtCore.QObject):
    result = QtCore.Signal(str, object, bool)

class ElidedLabel(QtWidgets.QLabel):
    """Keep device inventories on one line; the tooltip retains the full text."""
    def __init__(self, text):
        super().__init__(text)
        self.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Fixed)
        self.setToolTip(text)

    def setText(self, text):
        super().setText(text)
        self.setToolTip(text)

    def paintEvent(self, event):
        painter = QtGui.QPainter(self)
        painter.setPen(self.palette().color(QtGui.QPalette.WindowText))
        text = self.fontMetrics().elidedText(self.text().replace("\n", " | "), QtCore.Qt.ElideRight, self.contentsRect().width())
        painter.drawText(self.contentsRect(), QtCore.Qt.AlignVCenter | QtCore.Qt.AlignLeft, text)


class SaveProgressDialog(QtWidgets.QDialog):
    """Non-modal writer-drain indicator; no invented progress percentage."""
    def __init__(self, parent):
        super().__init__(parent)
        self.source = None
        self.running = True
        self.setWindowTitle("正在保存录制数据")
        self.setWindowModality(QtCore.Qt.NonModal)
        self.setWindowFlag(QtCore.Qt.WindowContextHelpButtonHint, False)
        self.setFixedWidth(460)
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(20, 18, 20, 18)
        layout.setSpacing(12)
        self.stage = QtWidgets.QLabel()
        self.stage.setStyleSheet("font-size:16px;font-weight:700;color:#75d9cd")
        self.stage.setWordWrap(True)
        layout.addWidget(self.stage)
        self.progress = QtWidgets.QProgressBar()
        self.progress.setRange(0, 0)
        self.progress.setTextVisible(False)
        layout.addWidget(self.progress)
        self.details = QtWidgets.QLabel()
        self.details.setTextFormat(QtCore.Qt.PlainText)
        self.details.setWordWrap(True)
        self.details.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        layout.addWidget(self.details)
        self.hint = QtWidgets.QLabel("正在排空写盘队列、保存时间索引与六轴关联表。请勿强制退出；设备预览可继续。")
        self.hint.setWordWrap(True)
        layout.addWidget(self.hint)
        self.close_button = QtWidgets.QPushButton("关闭")
        self.close_button.setAutoDefault(False)
        self.close_button.clicked.connect(self.accept)
        self.close_button.hide()
        layout.addWidget(self.close_button)

    def set_stage(self, text, details):
        self.stage.setText(text)
        self.details.setText(details)
        self.adjustSize()

    def complete(self, error=None):
        self.running = False
        if error is None:
            self.accept()
            return
        self.setWindowTitle("保存失败")
        self.stage.setStyleSheet("font-size:16px;font-weight:700;color:#ffb3ad")
        self.set_stage("保存失败", f"已写入文件：{self.source}\n原因：{error}")
        self.progress.hide()
        self.hint.setText("已写入文件保留，但数据或关联索引可能不完整。请检查磁盘空间、文件占用及故障记录；关闭提示不会删除文件。")
        self.close_button.show()
        self.close_button.setEnabled(True)
        self.adjustSize()

    def accept(self):
        if not self.running:
            super().accept()

    def reject(self):
        if not self.running:
            super().reject()

    def closeEvent(self, event):
        if self.running:
            event.ignore()
        else:
            super().closeEvent(event)


class StereoPreview(QtWidgets.QWidget):
    """Aspect-fit painting independent of image size hints and capture refreshes."""
    def __init__(self):
        super().__init__()
        self._pixmap = QtGui.QPixmap()
        self._text = "双目预览 · 连接后即可显示，不需要录制"
        self.stacked = False
        self.setMinimumSize(160, 100)
        self.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Ignored)

    def sizeHint(self):
        return QtCore.QSize(480, 300)

    def pixmap(self):
        return self._pixmap

    def setPixmap(self, pixmap):
        self._pixmap = pixmap
        self.update()

    def setText(self, text):
        self._text = text
        self._pixmap = QtGui.QPixmap()
        self.update()

    def set_stacked(self, stacked):
        self.stacked = stacked
        self.update()

    def image_rects(self):
        """Return destination/source pairs; both eyes retain their native aspect."""
        if self._pixmap.isNull():
            return []
        area = QtCore.QRectF(self.rect()).adjusted(6, 6, -6, -6)
        gap, caption = 6, 20
        result = []
        half = self._pixmap.width() // 2
        for eye in range(2):
            if self.stacked:
                cell = QtCore.QRectF(area.x(), area.y()+eye*(area.height()+gap)/2,
                                     area.width(), max(1, (area.height()-gap)/2))
            else:
                cell = QtCore.QRectF(area.x()+eye*(area.width()+gap)/2, area.y(),
                                     max(1, (area.width()-gap)/2), area.height())
            source = QtCore.QRectF(eye*half, 0, half if eye == 0 else self._pixmap.width()-half,
                                  self._pixmap.height())
            available = cell.adjusted(0, caption, 0, 0)
            scale = min(available.width()/source.width(), available.height()/source.height())
            size = QtCore.QSizeF(source.width()*scale, source.height()*scale)
            dest = QtCore.QRectF(available.center().x()-size.width()/2,
                                 available.center().y()-size.height()/2, size.width(), size.height())
            result.append((dest, source, cell))
        return result

    def paintEvent(self, event):
        painter = QtGui.QPainter(self)
        painter.fillRect(self.rect(), QtGui.QColor("#0a1220"))
        painter.setPen(QtGui.QColor("#8c9caf"))
        if self._pixmap.isNull():
            painter.drawText(self.rect().adjusted(12, 12, -12, -12),
                             QtCore.Qt.AlignCenter | QtCore.Qt.TextWordWrap, self._text)
            return
        painter.setRenderHint(QtGui.QPainter.SmoothPixmapTransform)
        for eye, (dest, source, cell) in enumerate(self.image_rects()):
            painter.drawPixmap(dest, self._pixmap, source)
            painter.drawText(cell.adjusted(4, 0, -4, 0), QtCore.Qt.AlignTop | QtCore.Qt.AlignLeft,
                             f"cam{eye} · {'左' if eye == 0 else '右'}半幅")


class Window(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("H743 · 双目 / IMU 数据采集")
        self.resize(1360, 950)
        self.connection = None
        self.recorder = None
        self.closing = False
        self.last_session = None
        self.save_dialog = None
        self.inventory = []
        self.busy = False
        self.last_rates = None
        self.next_diag = 0
        self._calibration_state = None
        self.calibration_dialog = None
        self.bridge = Bridge()
        self.bridge.result.connect(self.job_result)
        self._last_preview_image = None
        self._wide_layout = None
        main = QtWidgets.QWidget()
        self.setCentralWidget(main)
        layout = QtWidgets.QVBoxLayout(main)
        layout.setContentsMargins(10, 6, 10, 6)
        layout.setSpacing(2)
        header = QtWidgets.QHBoxLayout()
        title = QtWidgets.QLabel("H743 / STEREO + IMU")
        title.setStyleSheet("font-size:19px;font-weight:700;color:#75d9cd")
        header.addWidget(title)
        header.addStretch()
        self.layout_mode = QtWidgets.QComboBox()
        self.layout_mode.addItems(["自适应布局", "左右分区", "上下分区"])
        self.layout_mode.currentIndexChanged.connect(self.update_workspace_layout)
        header.addWidget(self.layout_mode)
        self.settings_button = QtWidgets.QPushButton("设备 / 保存设置")
        self.settings_button.setCheckable(True)
        self.settings_button.setChecked(True)
        header.addWidget(self.settings_button)
        self.details_button = QtWidgets.QPushButton("诊断 / 日志")
        self.details_button.setCheckable(True)
        header.addWidget(self.details_button)
        layout.addLayout(header)
        warning = QtWidgets.QLabel(WARNING + " · 无标定 / 不保证左右目同步曝光")
        warning.setWordWrap(True)
        warning.setStyleSheet("background:#43371d;color:#ffe0a1;padding:5px;border-radius:5px")
        layout.addWidget(warning)
        self.ports = QtWidgets.QComboBox()
        self.ports.setMinimumWidth(160)
        self.ports.setMinimumContentsLength(14)
        self.ports.setSizeAdjustPolicy(QtWidgets.QComboBox.AdjustToMinimumContentsLengthWithIcon)
        self.ports.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
        self.refresh = QtWidgets.QPushButton("刷新设备")
        self.refresh.clicked.connect(self.refresh_devices)
        self.camera = QtWidgets.QSpinBox()
        self.camera.setRange(0, 15)
        self.camera.setPrefix("索引 ")
        self.backend = QtWidgets.QComboBox()
        self.backend.addItems(["MSMF", "DSHOW", "ANY"])
        self.resolution = QtWidgets.QComboBox()
        self.resolution.addItems(["2560×720", "1280×480", "1280×720", "3840×1080"])
        self.fps = QtWidgets.QDoubleSpinBox()
        self.fps.setRange(1, 120)
        self.fps.setValue(30)
        self.fps.setSuffix(" fps（请求）")
        self.connect_button = QtWidgets.QPushButton("连接预览")
        self.connect_button.clicked.connect(self.connect_devices)
        self.disconnect_button = QtWidgets.QPushButton("断开连接")
        self.disconnect_button.clicked.connect(self.disconnect_devices)
        self.start_button = QtWidgets.QPushButton("开始录制")
        self.start_button.setStyleSheet("background:#117c72;font-weight:700")
        self.start_button.clicked.connect(self.start_recording)
        self.stop_button = QtWidgets.QPushButton("完成录制")
        self.stop_button.setToolTip("完成录制后弹窗显示保存状态，生成完整六轴关联表；设备保持连接、预览继续")
        self.stop_button.setEnabled(False)
        self.stop_button.clicked.connect(self.stop_recording)
        actions = QtWidgets.QHBoxLayout()
        for w in (self.ports, self.connect_button, self.disconnect_button, self.start_button,
                  self.stop_button):
            actions.addWidget(w)
        layout.addLayout(actions)
        calibration_row = QtWidgets.QHBoxLayout()
        self.recalibrate_button = QtWidgets.QPushButton("完整重校准 IMU")
        self.recalibrate_button.setToolTip("不重启设备；重新估计陀螺零偏 / 重力倾角，并将相对 yaw 归零。原始录制继续。")
        self.recalibrate_button.clicked.connect(self.recalibrate_imu)
        self.heading_zero_button = QtWidgets.QPushButton("仅航向归零")
        self.heading_zero_button.setToolTip("保留陀螺零偏、磁参考与门控状态；只更换相对航向零点。近竖直时拒绝。")
        self.heading_zero_button.clicked.connect(self.zero_heading)
        self.calibration_status = QtWidgets.QLabel("重新校准：未连接")
        self.calibration_status.setWordWrap(True)
        calibration_row.addWidget(self.recalibrate_button)
        calibration_row.addWidget(self.heading_zero_button)
        calibration_row.addWidget(self.calibration_status, 1)
        layout.addLayout(calibration_row)
        self.settings_panel = QtWidgets.QWidget()
        settings = QtWidgets.QGridLayout(self.settings_panel)
        settings.setContentsMargins(0, 0, 0, 0)
        settings.addWidget(self.refresh, 0, 0)
        for col, widget in enumerate((self.camera, self.backend, self.resolution, self.fps), 1):
            settings.addWidget(widget, 0, col)
        self.output = QtWidgets.QLineEdit(str(BASE/"recordings"))
        self.output.setMinimumWidth(100)
        self.output.setToolTip("每次录制新建一个会话文件夹；raw/ 内含原始数据、双目图片和 imu_image.csv 关联表")
        self.browse = QtWidgets.QPushButton("保存位置…")
        self.browse.clicked.connect(self.choose_directory)
        paths = QtWidgets.QHBoxLayout()
        paths.addWidget(QtWidgets.QLabel("保存位置"))
        paths.addWidget(self.output, 1)
        paths.addWidget(self.browse)
        settings.addLayout(paths, 1, 0, 1, 5)
        self.camera_label = ElidedLabel("正在枚举摄像头；索引需以预览实测确认")
        settings.addWidget(self.camera_label, 2, 0, 1, 5)
        layout.addWidget(self.settings_panel)
        self.settings_button.toggled.connect(self.settings_panel.setVisible)
        self.preview_label = StereoPreview()
        self.preview_label.setToolTip("cam0 / cam1 对应输入左右半幅；物理目序和曝光同步未标定。预览缩放不改变录制分辨率。")
        self.waveforms = WaveformPanel()
        self.accel_chart = self.waveforms.accel_chart
        self.gyro_chart = self.waveforms.gyro_chart
        self.preview_splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        self.preview_splitter.setChildrenCollapsible(False)
        self.preview_splitter.setHandleWidth(7)
        self.preview_splitter.addWidget(self.preview_label)
        self.attitude_panel = AttitudePanel()
        self.telemetry_tabs = QtWidgets.QTabWidget()
        self.telemetry_tabs.addTab(self.waveforms, "原始六轴")
        self.telemetry_tabs.addTab(self.attitude_panel, "姿态 / 四元数")
        self.preview_splitter.addWidget(self.telemetry_tabs)
        layout.addWidget(self.preview_splitter, 1)
        self.status = QtWidgets.QLabel("待机 · 连接预览不写盘；开始录制后保存原始数据")
        self.status.setWordWrap(True)
        self.status.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        layout.addWidget(self.status)
        self.save_status = ElidedLabel("录制流程：开始录制 → 完成录制 → 保存完成")
        layout.addWidget(self.save_status)
        self.error_banner = QtWidgets.QLabel()
        self.error_banner.setWordWrap(True)
        self.error_banner.setStyleSheet("background:#672c32;color:#ffe2df;padding:6px")
        self.error_banner.hide()
        layout.addWidget(self.error_banner)
        self.details_panel = QtWidgets.QTabWidget()
        self.details_panel.setMaximumHeight(160)
        self.diagnostics = QtWidgets.QLabel("统一时间：电脑读取结束时间；不使用时钟拟合；不是曝光 / 采样时间")
        self.diagnostics.setWordWrap(True)
        self.diagnostics.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        self.telemetry = QtWidgets.QLabel("未录制")
        self.telemetry.setWordWrap(True)
        detail_page = QtWidgets.QWidget()
        detail_layout = QtWidgets.QVBoxLayout(detail_page)
        detail_layout.addWidget(self.telemetry)
        detail_layout.addWidget(self.diagnostics)
        self.recorded_attitude_button = QtWidgets.QPushButton("查看已完成录制的姿态 / 四元数…")
        self.recorded_attitude_button.clicked.connect(self.show_recorded_attitude)
        detail_layout.addWidget(self.recorded_attitude_button)
        detail_layout.addStretch()
        detail_scroll = QtWidgets.QScrollArea()
        detail_scroll.setWidgetResizable(True)
        detail_scroll.setWidget(detail_page)
        self.details_panel.addTab(detail_scroll, "设备 / 写盘 / 接收时间")
        self.log = QtWidgets.QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(500)
        self.details_panel.addTab(self.log, "事件日志")
        self.details_panel.hide()
        self.details_button.toggled.connect(self.details_panel.setVisible)
        layout.addWidget(self.details_panel)
        self.update_workspace_layout()
        screen = self.screen() or QtGui.QGuiApplication.primaryScreen()
        available = screen.availableGeometry()
        self.resize(min(1440, available.width()-24), min(960, available.height()-50))
        self.controls = [self.ports,self.refresh,self.camera,self.backend,self.resolution,
                         self.fps,self.output,self.browse,
                         self.start_button]
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(100)
        self.refresh_devices()
        self.set_recording_controls(False)

    def update_workspace_layout(self, *_):
        if not hasattr(self, "preview_splitter"):
            return
        mode = self.layout_mode.currentIndex()
        wide = mode == 1 or (mode == 0 and self.width() >= 1250 and self.height() >= 780)
        if wide == self._wide_layout:
            return
        self._wide_layout = wide
        self.preview_splitter.setOrientation(QtCore.Qt.Horizontal if wide else QtCore.Qt.Vertical)
        self.preview_label.set_stacked(wide)
        self.waveforms.set_stacked(wide)
        self.attitude_panel.waveforms.set_stacked(wide)
        self.preview_splitter.setStretchFactor(0, 2)
        self.preview_splitter.setStretchFactor(1, 3)
        self.preview_splitter.setSizes([480, 800] if wide else [160, 340])

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.update_workspace_layout()

    def task(self, name, function):
        def job():
            try:
                self.bridge.result.emit(name, function(), False)
            except Exception as exc:
                self.bridge.result.emit(name, f"{type(exc).__name__}: {exc}", True)
        threading.Thread(target=job, daemon=True, name="gui-"+name).start()

    def refresh_devices(self):
        self.refresh.setEnabled(False)
        self.task("devices", lambda: (discover_ports(), camera_inventory()))

    @QtCore.Slot(str, object, bool)
    def job_result(self, name, value, error):
        if name == "devices":
            self.refresh.setEnabled(self.connection is None and not self.busy and not self.closing)
            if error:
                self.log.appendPlainText(str(value))
                return
            ports, self.inventory = value
            self.ports.clear()
            for port in sorted(ports, key=lambda p:not p["h743"]):
                self.ports.addItem(f'{port["port"]} · {port["description"]} · {port["serial_number"] or "无序列号"}', port)
            names = ", ".join(p.get("FriendlyName","?") for p in self.inventory) or "未发现活动 Camera/Image 类设备"
            self.camera_label.setText("PnP: " + names + " · 此列表不是 OpenCV 索引映射")
    def choose_directory(self):
        directory = QtWidgets.QFileDialog.getExistingDirectory(self,"选择录制目录",self.output.text())
        if directory:
            self.output.setText(directory)

    def set_recording_controls(self, recording=False):
        connected = self.connection is not None
        stopping = connected and self.connection.stop_event.is_set()
        saving = self.recorder is not None
        for widget in (self.ports, self.refresh, self.camera, self.backend, self.resolution, self.fps):
            widget.setEnabled(not connected and not self.busy and not self.closing)
        for widget in (self.output, self.browse):
            widget.setEnabled(not saving and not self.busy and not self.closing)
        self.connect_button.setEnabled(not connected and not self.busy and not self.closing)
        self.disconnect_button.setEnabled(connected and not stopping and not self.closing)
        ready = connected and self.connection.metrics["camera_frames"] > 0 and self.connection.metrics["imu_frames"] > 0
        self.start_button.setEnabled(bool(ready and not saving and not stopping and not self.busy and not self.closing))
        self.stop_button.setEnabled(saving and not self.recorder.stop_event.is_set() and not self.closing)
        self.update_calibration()

    def update_calibration(self):
        connection = self.connection
        snapshot = connection.calibration_snapshot() if connection else dict(state="disconnected", busy=False, masked=False)
        state = snapshot["state"]
        labels = {
            "idle": "待命",
            "queued": "请求已排队 · 等待发送",
            "awaiting_ack": "已发送 · 等待板端执行确认（不是校准完成）",
            "calibrating": "板端已执行 · 请持续静置约 2.2 秒，等待姿态有效",
            "waiting_stationary": "仍在等待有效姿态 · 运动会延迟校准，请保持静置",
            "magnetic_wait": "零偏/倾角已估计 · 请继续静置，等待磁参考（额外最多 5 秒）",
            "completed": "已完成 · 姿态有效，相对 yaw 已重新归零",
            "degraded": "六轴已就绪 · 磁参考未就绪，已明确降级；yaw 可能漂移",
            "rejected": "板端拒绝请求（状态码 %s），未执行航向归零" % snapshot.get("status"),
            "ack_timeout": "未收到执行确认 · 旧固件可能不支持，结果未知；不自动重试，请检查固件 / 连接",
            "disconnected": "未连接 / 已中断 · 不代表校准完成",
        }
        text = labels.get(state, state)
        diagnostic = snapshot.get("diagnostic")
        if snapshot.get("operation") == 2:
            if state == "completed":
                text = "航向已归零 · 保留零偏、磁参考及门控状态"
            elif state == "calibrating":
                text = "航向归零已执行 · 等待边界后有效姿态"
            elif state == "rejected":
                text = ("近竖直姿态航向不确定，请离开竖直后再归零" if snapshot.get("status") == 2
                        else "姿态尚未有效，航向归零被拒绝")
        elif state == "completed" and diagnostic is None:
            text = "六轴姿态有效 · 磁状态未知（未收到新鲜诊断），不能认定磁校准成功"
        if diagnostic is None:
            text += "\n磁辅助：状态未知 / 诊断缺失或过期"
        else:
            reasons = {"waiting":"等待观测", "ok":"观测通过", "imu":"IMU 无效",
                       "motion":"运动过快", "quality":"加速度质量", "time":"时间异常",
                       "sequence":"磁序号中断", "stale":"磁数据过期", "overflow":"磁队列溢出",
                       "norm":"磁模长异常", "horizontal":"水平磁分量不足", "dip":"磁倾角异常",
                       "innovation":"航向创新异常", "unstable":"恢复窗口不稳定"}
            reason = diagnostic["reason_name"]
            aiding = bool(diagnostic["flags"] & 8192)
            text += ("\n磁辅助：%s · 板端原因 %s · 恢复 %d 次 / %d ms（需 ≥20 次且 ≥800 ms）"
                     " · 创新 %s° · 修正 %s°/s" %
                     ("作用中" if aiding else "暂停 / 恢复中", reasons.get(reason, reason),
                      diagnostic["recovery_count"], diagnostic["recovery_elapsed_ms"],
                      diagnostic["innovation_deg"], diagnostic["correction_rate_dps"]))
        self.calibration_status.setText("IMU 控制：" + text)
        key = (id(connection), state, snapshot.get("request_id"))
        if key != self._calibration_state:
            if state not in ("idle", "disconnected"):
                self.log.appendPlainText("重新校准 IMU：" + text)
            self._calibration_state = key
        ready = connection is not None and connection.metrics["imu_frames"] > 0
        self.recalibrate_button.setEnabled(bool(ready and not connection.stop_event.is_set()
                                               and not snapshot["busy"] and not self.closing))
        self.heading_zero_button.setEnabled(self.recalibrate_button.isEnabled())
        dialog = self.calibration_dialog
        if dialog is not None:
            if dialog.request_id is not None:
                if (dialog.connection is connection and dialog.request_id == snapshot.get("request_id")
                        and snapshot.get("operation", 1) == 1):
                    dialog.set_state(state, text)
                elif dialog.connection is connection and dialog.state != "completed":
                    dialog.set_state("superseded", "已被新控制取代；本次校准未确认完成，请查看当前控制状态。")
                elif dialog.state != "completed":
                    dialog.set_state("disconnected", "连接已中断；本次校准未确认完成。请连接设备后重试。")
            dialog.set_retry_available(self.recalibrate_button.isEnabled())
        return snapshot

    def recalibrate_imu(self):
        connection = self.connection
        if not self.recalibrate_button.isEnabled() or connection is None:
            return
        answer = QtWidgets.QMessageBox.question(self, "重新校准 IMU",
            "请先将板卡放稳。确认后重新估计陀螺零偏和重力倾角，连续静置约 2.2 秒估计零偏/倾角，再等待磁参考；"
            "磁参考额外最多等 5 秒，未就绪则明确六轴降级。运动会重新累计静置。"
            "相对 yaw 将归零，不能与之前的航向基准直接拼接。\n"
            "不重启 MCU，不中断原始六轴采集或录制。是否继续？",
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No, QtWidgets.QMessageBox.No)
        if answer != QtWidgets.QMessageBox.Yes:
            return
        self.start_imu_calibration()

    def zero_heading(self):
        connection = self.connection
        if connection is None or not self.heading_zero_button.isEnabled():
            return
        answer = QtWidgets.QMessageBox.question(self, "仅航向归零",
            "只更换相对航向零点，保留陀螺零偏、磁参考和门控/恢复状态，不重新标定。"
            "近竖直或姿态无效时板端会拒绝；不会强行开启磁辅助。\n"
            "航向曲线将在边界处断开，不能与之前零点直接拼接。是否继续？",
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No, QtWidgets.QMessageBox.No)
        if answer != QtWidgets.QMessageBox.Yes:
            return
        try:
            connection.request_heading_zero()
            snapshot = self.update_calibration()
            self.attitude_panel.refresh(connection.attitude_history,
                                        not connection.stop_event.is_set(), snapshot["masked"])
        except Exception as exc:
            self.log.appendPlainText("航向归零请求失败：" + str(exc))

    def start_imu_calibration(self):
        connection = self.connection
        if connection is None or not self.recalibrate_button.isEnabled():
            return
        if self.calibration_dialog is None:
            self.calibration_dialog = CalibrationProgressDialog(self)
            self.calibration_dialog.retry_requested.connect(self.retry_imu_calibration)
        dialog = self.calibration_dialog
        dialog.begin(connection)
        try:
            dialog.request_id = connection.request_recalibration()
            snapshot = self.update_calibration()
            self.attitude_panel.refresh(connection.attitude_history,
                                        not connection.stop_event.is_set(), snapshot["masked"])
        except Exception as exc:
            dialog.request_id = None
            dialog.set_state("request_failed", "校准请求失败：" + str(exc))
            self.log.appendPlainText("重新校准 IMU 请求失败：" + str(exc))
            self.update_calibration()

    def retry_imu_calibration(self):
        dialog = self.calibration_dialog
        if dialog is None or dialog.state not in dialog.FAILURE_STATES:
            return
        # Re-evaluate live connection/busy state; never send an automatic retry.
        self.update_calibration()
        if dialog.retry_button.isEnabled():
            self.start_imu_calibration()

    def connect_devices(self):
        if self.connection is not None or self.busy or self.closing:
            return
        port = self.ports.currentData()
        if not port:
            QtWidgets.QMessageBox.warning(self,"未选择设备","请先连接 H743 并刷新设备")
            return
        try:
            width,height = map(int,self.resolution.currentText().split("×"))
            connection = LiveConnection(Config(port=port["port"], serial_number=port["serial_number"],
                camera_index=self.camera.value(), backend=self.backend.currentText(),
                width=width,height=height,fps=self.fps.value(),output=self.output.text(),
                camera_inventory=self.inventory))
            connection.start()
            self.connection = connection
            self.last_rates = None
            self._last_preview_image = None
            self.error_banner.hide()
            self.next_diag = 0
            self.preview_label.setText("正在连接，等待相机图像…（仅预览，不写盘）")
            self.log.appendPlainText("正在连接设备；图像和六轴实时预览不创建录制文件。")
            self.set_recording_controls()
        except Exception as exc:
            QtWidgets.QMessageBox.critical(self,"无法连接",str(exc))

    def disconnect_devices(self):
        if self.connection is not None:
            self.connection.stop_recording()
            self.connection.stop()
            self.log.appendPlainText("正在断开设备；如有录制，先排空写盘并完成会话…")
            self.set_recording_controls()

    def start_recording(self):
        if self.connection is None or self.recorder is not None or self.busy or self.closing:
            return
        try:
            output_root = validate_output_root(self.output.text())
            recorder = self.connection.start_recording(str(output_root))
            self.recorder = recorder
            self.last_session = recorder.path
            self.error_banner.hide()
            self.set_recording_controls(True)
            self.save_status.setText(f"正在录制 · 保存位置: {recorder.path}\n完成录制后生成: {recorder.path / 'imu_image.csv'}")
            self.log.appendPlainText("录制开始: "+str(recorder.path))
        except Exception as exc:
            QtWidgets.QMessageBox.critical(self,"无法开始录制",str(exc))

    def stop_recording(self):
        if self.connection is not None and self.recorder is not None:
            self.connection.stop_recording()
            self.show_save_progress("正在保存录制数据…", self.recorder.path)
            self.save_status.setText("正在保存写盘队列、时间索引与六轴关联表…")
            self.set_recording_controls()
            self.log.appendPlainText("录制已停止，正在排空写盘并保存接收时间索引与六轴关联表；设备保持连接、预览继续。")

    def show_recorded_attitude(self):
        source = QtWidgets.QFileDialog.getExistingDirectory(
            self, "选择已完成的原始会话", str(self.last_session or BASE/"recordings"))
        if not source:
            return
        try:
            path = resolve_raw_session(source)
            session = json.loads((path/"session.json").read_text(encoding="utf8"))
            if not session.get("finalized"):
                raise ValueError("请先完成录制并等待写盘完成")
            dialog = AttitudeDetailsDialog(path, self)
            try:
                dialog.exec()
            finally:
                dialog.handle.close()
        except Exception as exc:
            QtWidgets.QMessageBox.warning(self, "无法读取姿态详情", str(exc))

    def show_save_progress(self, stage, source):
        if self.save_dialog is not None and not self.save_dialog.running:
            self.save_dialog.close()
            self.save_dialog.deleteLater()
            self.save_dialog = None
        if self.save_dialog is None:
            self.save_dialog = SaveProgressDialog(self)
        self.save_dialog.source = Path(source)
        self.save_dialog.set_stage(stage, f"保存位置：{source}")
        self.save_dialog.show()
        self.save_dialog.raise_()

    def finish_recording(self):
        sink = self.recorder
        if sink is None or not sink.done.is_set():
            return
        # Detach exactly once. done means all writers and finalization ended.
        self.recorder = None
        if sink.error:
            message = f"保存失败: {sink.error}；已写入文件保留在 {sink.path}，可能不完整。"
            self.save_status.setText(message)
            self.error_banner.setText(message)
            self.error_banner.show()
            self.log.appendPlainText(message)
            self.show_save_progress("保存失败", sink.path)
            self.save_dialog.complete(sink.error)
        else:
            message = f"保存完成 · {sink.path}\n六轴 / 图像 / 姿态关联表: {sink.path / 'imu_image.csv'}"
            self.save_status.setText(message)
            self.log.appendPlainText(message)
            if self.save_dialog is not None:
                self.save_dialog.complete()
                self.save_dialog.deleteLater()
                self.save_dialog = None
        self.log.appendPlainText("接收时间结果: "+json.dumps(sink.report, ensure_ascii=False))
        self.set_recording_controls()

    def tick(self):
        self.finish_recording()
        r = self.connection
        if r is None:
            self.attitude_panel.refresh(connected=False)
            if self.closing and not self.recorder and not self.busy:
                self.close()
            return
        now = time.perf_counter()
        diag = now >= self.next_diag
        if diag:
            self.next_diag = now + 2
        s = r.snapshot()
        calibration = self.update_calibration()
        self.attitude_panel.refresh(r.attitude_history, not r.stop_event.is_set(), calibration["masked"])
        # Keep the old connection history if a new device attempt fails before
        # producing both streams. Once ready, this connection owns the history.
        if s["camera_frames"] and s["imu_frames"] and self.waveforms.history is not r.history:
            self.waveforms.set_history(r.history)
        with r.lock:
            image = r.latest_image
        if image is not None and image is not self._last_preview_image:
            # Cache native pixels; painting handles resizes even without a new frame.
            rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            qimage = QtGui.QImage(rgb.data, rgb.shape[1], rgb.shape[0], rgb.strides[0],
                                 QtGui.QImage.Format_RGB888).copy()
            self.preview_label.setPixmap(QtGui.QPixmap.fromImage(qimage))
            self._last_preview_image = image
        disk = self.recorder.snapshot() if self.recorder else None
        saving_text = ("录制收尾中 / 预览继续" if self.recorder and self.recorder.stop_event.is_set()
                       else "正在录制" if self.recorder
                       else "仅实时预览（不写盘）")
        disk_text = (f'已写 IMU {disk["written_imu"]:,} / 双目 {disk["written_images"]:,} | '
                     f'队列 USB {disk["usb_queue"]} · 相机 {disk["camera_queue"]} | '
                     f'剩余 {disk["free_bytes"]/1024**3:.1f} GiB / 峰值 RSS {disk["rss_peak_bytes"]/1024**2:.0f} MiB'
                     + (f'\n录制错误: {disk["error"]}' if disk["error"] else "")
                     if disk else "不保存图像或 USB 数据；点击开始录制才写盘")
        if self.last_rates is not None and now-self.last_rates[0] >= 1:
            old_t,old_imu,old_cam = self.last_rates
            self._rates = ((s["imu_frames"]-old_imu)/(now-old_t),
                           (s["camera_frames"]-old_cam)/(now-old_t))
            self.last_rates = (now,s["imu_frames"],s["camera_frames"])
        elif self.last_rates is None:
            self._rates = (0,0)
            self.last_rates = (now,s["imu_frames"],s["camera_frames"])
        ihz,chz = self._rates
        flags = ", ".join(f"{k}:{v}" for k,v in s["flags"].items() if v) or "无"
        self.status.setText(
            f'{"连接异常 / 正在断开" if s["error"] else saving_text} · {s["elapsed_s"]:.1f}s | '
            f'IMU {ihz:.1f} Hz / 相机 {chz:.1f} fps | CRC {s["crc_errors"]} / 缺号 {s["missing_sequences"]}'
            + (f' | USB 队列 {disk["usb_queue"]} / 图像队列 {disk["camera_queue"]} | 剩余 {disk["free_bytes"]/1024**3:.1f} GiB' if disk else "")
            + f'\n质量标志 {flags}')
        self.telemetry.setText(disk_text + f'\n质量标志 {flags}')
        error = s["error"] or (disk["error"] if disk else None)
        if error:
            self.error_banner.setText("异常：" + str(error))
            self.error_banner.show()  # Remains visible after disconnect, even with details hidden.
        if diag:
            self.diagnostics.setText(
                "统一时间：电脑 perf_counter_ns() 读取结束时间；不使用时钟拟合。\n"
                "同一 USB 批次共享时间戳；接收时间不是采样 / 曝光时间。\n"
                f'相机媒体时间异常 {s["camera_media_timestamp_anomalies"]} 次（保留原值，不影响接收时间轴）；'
                "波形横轴仍为 H743 相对时间。")
        if r.done.is_set():
            self.log.appendPlainText(("连接错误: "+r.error) if r.error else "设备已断开。")
            self.connection = None
            self.attitude_panel.refresh(connected=False)
            self.preview_label.setText("设备已断开 · 点击「连接预览」重新显示")
            self.status.setText("连接错误: "+r.error if r.error else "已断开（未采集）")
        self.set_recording_controls()
        if self.closing and self.connection is None and self.recorder is None and not self.busy:
            self.close()

    def closeEvent(self,event):
        if self.connection is not None or self.recorder is not None or self.busy:
            self.closing = True
            self.disconnect_devices()
            self.set_recording_controls()
            self.log.appendPlainText("正在安全关闭，请等待设备释放及后台任务完成…")
            event.ignore()
        else:
            if self.save_dialog is not None:
                self.save_dialog.close()
                self.save_dialog.deleteLater()
                self.save_dialog = None
            if self.calibration_dialog is not None:
                self.calibration_dialog.close()
                self.calibration_dialog.deleteLater()
                self.calibration_dialog = None
            self.waveforms.shutdown()
            self.attitude_panel.shutdown()
            event.accept()


def run(smoke_seconds=0):
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv[:1])
    app.setStyle("Fusion")
    app.setStyleSheet(APP_STYLESHEET)
    window = Window()
    window.show()
    if smoke_seconds:
        QtCore.QTimer.singleShot(int(smoke_seconds*1000),window.close)
    return app.exec()
