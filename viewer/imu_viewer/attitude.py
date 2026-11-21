"""Live attitude display; history is independent of the GUI runtime."""
from PySide6 import QtCore, QtWidgets
from .waveform import WaveformPanel


class AttitudePanel(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.history = None
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(4)
        self.state = QtWidgets.QLabel("姿态：未连接")
        self.values = QtWidgets.QLabel("roll / pitch / yaw：— / — / — °")
        self.quaternion = QtWidgets.QLabel("四元数 qw,qx,qy,qz：—")
        self.quaternion.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        status_row = QtWidgets.QHBoxLayout()
        status_row.addWidget(self.state)
        status_row.addWidget(self.values, 1)
        layout.addLayout(status_row)
        layout.addWidget(self.quaternion)
        note = QtWidgets.QLabel("FRD → NED · 相对航向（实验磁辅助状态见上，非磁北）· 安装方向待实板核验")
        note.setWordWrap(True)
        note.setToolTip("Euler 3-2-1；yaw 静置校准归零。安装旋转暂定 X/Z 取反，必须实板核验。"
                        "连接历史最近 10 秒完整，早期为各分量独立峰值，不是有效的合并四元数。")
        for label in (self.state, self.values, self.quaternion):
            label.setWordWrap(True)
        layout.addWidget(note)
        self.waveforms = WaveformPanel(chart_specs=(
            ("欧拉角", "°", (1, 2, 3), ("roll", "pitch", "yaw")),
            ("四元数", "", (4, 5, 6, 7), ("qw", "qx", "qy", "qz"))))
        self.euler_chart, self.quaternion_chart = self.waveforms.charts
        # Compatibility aliases for the old Euler plot, not a separate renderer.
        self.plot, self.curves = self.euler_chart.plot, self.euler_chart.curves
        layout.addWidget(self.waveforms, 1)

    def refresh(self, history=None, connected=False, recalibrating=False):
        if history is not None and history is not self.history:
            self.history = history
            self.waveforms.set_history(history.waveforms)
        if self.history is None:
            return
        f, live = self.history.status(connected)
        valid = live and f is not None and f.attitude_valid and not recalibrating
        if valid:
            # A missing bit is not proof that magnetic firmware was disabled.
            mag = f.flags & 0xe000
            magnetic = ("磁标志异常" if mag not in (0, 16384, 16384 | 32768, 8192 | 32768)
                        else "磁辅助中" if mag & 8192
                        else "磁拒绝 / 六轴积分" if mag & 16384 and mag & 32768
                        else "磁等待 / 六轴积分" if mag & 16384
                        else "未报告磁状态")
            self.state.setText("姿态：有效 · 相对航向 · " + magnetic)
            self.values.setText(f"roll / pitch / yaw：{f.roll_deg:.2f} / {f.pitch_deg:.2f} / {f.yaw_deg:.2f} °")
            self.quaternion.setText("四元数 qw,qx,qy,qz：" + ", ".join(f"{v:.6f}" for v in (f.qw,f.qx,f.qy,f.qz)))
        else:
            state = ("中断 / 未连接" if not live else "重新校准待确认 / 进行中" if recalibrating
                     else "无数据" if f is None else "旧 v2 无姿态" if f.protocol_version != 3
                     else "校准中（请静置）" if f.flags & 1024 else "无效")
            self.state.setText("姿态：" + state)
            self.values.setText("roll / pitch / yaw：— / — / — °")
            self.quaternion.setText("四元数 qw,qx,qy,qz：—")

    def shutdown(self):
        self.waveforms.shutdown()

    def closeEvent(self, event):
        self.shutdown()
        super().closeEvent(event)


class AttitudeDetailsDialog(QtWidgets.QDialog):
    """Read a completed recording in bounded 256-row pages, not a full replay."""
    FIELDS = ("sample_id", "protocol_version", "attitude_valid", "qw", "qx", "qy", "qz",
              "roll_deg", "pitch_deg", "yaw_deg")
    PAGE_SIZE = 256

    def __init__(self, path, parent=None):
        super().__init__(parent)
        import csv
        self.setWindowTitle("录制姿态详情 · 四元数 / Euler")
        self.resize(900, 480)
        self.count = 0
        layout = QtWidgets.QVBoxLayout(self)
        note = QtWidgets.QLabel("FRD → NED，qw,qx,qy,qz；yaw 为相对航向。无效 / v2 姿态显示 —。\n"
                               "每页最多 256 行；仅查看 CSV，完整性请运行离线 verify。")
        note.setWordWrap(True)
        layout.addWidget(note)
        self.table = QtWidgets.QTableWidget(0, len(self.FIELDS))
        self.table.setHorizontalHeaderLabels(self.FIELDS)
        self.table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        layout.addWidget(self.table)
        self.next_button = QtWidgets.QPushButton("下一页")
        self.next_button.clicked.connect(self.next_page)
        layout.addWidget(self.next_button)
        self.handle = (path / "imu.csv").open(encoding="utf8", newline="")
        self.reader = csv.DictReader(self.handle)
        self.finished.connect(lambda _: self.handle.close())
        try:
            self.next_page()
        except Exception:
            self.handle.close()
            raise

    def next_page(self):
        from itertools import islice
        rows = list(islice(self.reader, self.PAGE_SIZE))
        self.table.setRowCount(len(rows))
        for i, row in enumerate(rows):
            valid = row.get("attitude_valid") == "1" and row.get("protocol_version") == "3"
            for j, key in enumerate(self.FIELDS):
                value = row.get(key, "") if j < 3 or valid else "—"
                self.table.setItem(i, j, QtWidgets.QTableWidgetItem(value))
        self.count += len(rows)
        self.next_button.setText(f"下一页（已读 {self.count} 行）")
        self.next_button.setEnabled(len(rows) == self.PAGE_SIZE)
