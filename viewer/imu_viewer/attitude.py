"""Live attitude display; history is independent of the GUI runtime."""
from PySide6 import QtCore, QtWidgets
import pyqtgraph as pg


class AttitudePanel(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.history = None
        layout = QtWidgets.QVBoxLayout(self)
        self.state = QtWidgets.QLabel("姿态：未连接")
        self.values = QtWidgets.QLabel("roll / pitch / yaw：— / — / — °")
        self.quaternion = QtWidgets.QLabel("四元数 qw,qx,qy,qz：—")
        self.quaternion.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        for label in (self.state, self.values, self.quaternion):
            label.setWordWrap(True)
            layout.addWidget(label)
        note = QtWidgets.QLabel("FRD → NED · Euler 3-2-1 · yaw 静置校准归零、允许漂移（无磁力计）\n安装旋转暂定 X/Z 取反，必须实板核验。趋势仅保留最近 10,000 点。")
        note.setWordWrap(True)
        layout.addWidget(note)
        self.plot = pg.PlotWidget()
        self.plot.setLabel("left", "角度", units="°")
        self.plot.setLabel("bottom", "H743 相对时间", units="s")
        self.plot.addLegend()
        self.curves = [self.plot.plot(pen=pg.mkPen(c, width=1.5), name=n, connect="finite")
                       for c,n in zip(("#ffb454", "#7edbaf", "#78b9ff"), ("roll", "pitch", "yaw"))]
        layout.addWidget(self.plot, 1)

    def refresh(self, history=None, connected=False, recalibrating=False):
        if history is not None:
            self.history = history
        if self.history is None:
            return
        f, live, rows = self.history.snapshot(connected)
        valid = live and f is not None and f.attitude_valid and not recalibrating
        if valid:
            self.state.setText("姿态：有效 · 相对航向")
            self.values.setText(f"roll / pitch / yaw：{f.roll_deg:.2f} / {f.pitch_deg:.2f} / {f.yaw_deg:.2f} °")
            self.quaternion.setText("四元数 qw,qx,qy,qz：" + ", ".join(f"{v:.6f}" for v in (f.qw,f.qx,f.qy,f.qz)))
        else:
            state = ("中断 / 未连接" if not live else "重新校准待确认 / 进行中" if recalibrating
                     else "无数据" if f is None else "旧 v2 无姿态" if f.protocol_version != 3
                     else "校准中（请静置）" if f.flags & 1024 else "无效")
            self.state.setText("姿态：" + state)
            self.values.setText("roll / pitch / yaw：— / — / — °")
            self.quaternion.setText("四元数 qw,qx,qy,qz：—")
        for i, curve in enumerate(self.curves):
            curve.setData(rows[:,0], rows[:,i+1])
        if len(rows):
            self.plot.setXRange(max(0, rows[-1,0]-10), max(10, rows[-1,0]), padding=0)


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
