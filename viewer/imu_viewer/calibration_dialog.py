"""Non-blocking manual IMU recalibration feedback; no device I/O."""
from PySide6 import QtCore, QtWidgets


class CalibrationProgressDialog(QtWidgets.QDialog):
    retry_requested = QtCore.Signal()
    FAILURE_STATES = {"ack_timeout", "disconnected", "request_failed", "rejected", "degraded", "superseded"}

    def __init__(self, parent):
        super().__init__(parent)
        self.connection = None
        self.request_id = None
        self.state = None
        self.setWindowTitle("IMU 校准中")
        self.setWindowModality(QtCore.Qt.NonModal)
        self.setWindowFlag(QtCore.Qt.WindowContextHelpButtonHint, False)
        self.setMinimumWidth(440)
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(20, 18, 20, 18)
        layout.setSpacing(12)
        self.stage = QtWidgets.QLabel()
        self.stage.setStyleSheet("font-size:18px;font-weight:700")
        self.stage.setWordWrap(True)
        layout.addWidget(self.stage)
        self.progress = QtWidgets.QProgressBar()
        self.progress.setRange(0, 0)
        self.progress.setTextVisible(False)
        layout.addWidget(self.progress)
        self.details = QtWidgets.QLabel()
        self.details.setTextFormat(QtCore.Qt.PlainText)
        self.details.setWordWrap(True)
        layout.addWidget(self.details)
        self.hint = QtWidgets.QLabel()
        self.hint.setWordWrap(True)
        layout.addWidget(self.hint)
        actions = QtWidgets.QHBoxLayout()
        self.close_button = QtWidgets.QPushButton("关闭提示")
        self.close_button.setAutoDefault(False)
        self.close_button.clicked.connect(self.reject)
        actions.addWidget(self.close_button)
        self.retry_button = QtWidgets.QPushButton("重试校准")
        self.retry_button.setMinimumHeight(40)
        self.retry_button.setStyleSheet("font-size:15px;font-weight:700;background:#3d637a")
        self.retry_button.clicked.connect(self.retry_requested.emit)
        actions.addWidget(self.retry_button)
        layout.addLayout(actions)
        self.close_timer = QtCore.QTimer(self)
        self.close_timer.setSingleShot(True)
        self.close_timer.setTimerType(QtCore.Qt.PreciseTimer)
        self.close_timer.setInterval(1000)
        self.close_timer.timeout.connect(self.accept)

    def begin(self, connection):
        self.close_timer.stop()
        self.connection = connection
        self.request_id = None
        self.set_state("queued", "请求已排队 · 等待发送")
        self.show()
        self.raise_()

    def set_state(self, state, details):
        changed = state != self.state
        self.state = state
        self.details.setText(details)
        failed = state in self.FAILURE_STATES
        complete = state == "completed"
        self.retry_button.setVisible(failed)
        if not failed:
            self.retry_button.setEnabled(False)
        self.progress.setVisible(not failed and not complete)
        if complete:
            self.setWindowTitle("IMU 校准完成")
            self.stage.setText("校准完成")
            self.hint.setText("已收到板端有效姿态；磁状态以诊断详情为准，1 秒后自动关闭。")
            if changed:
                self.close_timer.start()
        else:
            self.close_timer.stop()
            self.setWindowTitle("IMU 校准未完成" if failed else "IMU 校准中")
            self.stage.setText("六轴已就绪，磁参考未就绪" if state == "degraded" else
                               "正在等待磁参考，请继续静置" if state == "magnetic_wait" else
                               "校准未完成，请检查后重试" if failed else "校准中，请保持静置")
            self.hint.setText(
                "请将板卡放稳；检查连接和固件后点击「重试校准」。未连接或仍有请求执行时无法重试。"
                if failed else "请将板卡放稳，不要移动。关闭提示不会取消板端校准，原始采集和录制继续。")
        self.adjustSize()

    def set_retry_available(self, ready):
        self.retry_button.setEnabled(bool(ready and self.state in self.FAILURE_STATES))

    def done(self, result):
        self.close_timer.stop()
        super().done(result)
