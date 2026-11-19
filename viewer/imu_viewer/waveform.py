"""Interactive linked waveform views; no hardware/disk work on the GUI thread."""
import threading
import time
import numpy as np
from PySide6 import QtCore, QtGui, QtWidgets
import pyqtgraph as pg
from .history import render_history


class WaveViewBox(pg.ViewBox):
    manual = QtCore.Signal()
    expanded = QtCore.Signal()

    def __init__(self):
        super().__init__(enableMenu=False)
        self.setMouseMode(self.PanMode)
        self.setMouseEnabled(x=True, y=False)

    def wheelEvent(self, event, axis=None):
        self.manual.emit()
        vertical = axis == 1 or bool(event.modifiers() & QtCore.Qt.ShiftModifier)
        # AxisItem forwards the event in axis-local coordinates; scenePos works
        # for both plot-area and axis events.
        center = self.mapToView(self.mapFromScene(event.scenePos()))
        factor = 1.002 ** (-event.delta())
        if vertical:
            self.enableAutoRange(y=False)
            self.scaleBy((1, factor), center=center)
        else:
            self.scaleBy((factor, 1), center=center)
        event.accept()

    def mouseDragEvent(self, event, axis=None):
        self.manual.emit()
        # Axis-specific dragging remains available; plot-area drag pans time only.
        if axis == 1:
            self.setMouseEnabled(y=True)
            try:
                super().mouseDragEvent(event, axis=axis)
            finally:
                self.setMouseEnabled(y=False)
        else:
            super().mouseDragEvent(event, axis=axis)

    def mouseDoubleClickEvent(self, event):
        if event.button() == QtCore.Qt.LeftButton:
            self.expanded.emit()
            event.accept()
        else:
            event.ignore()


class Chart(QtWidgets.QWidget):
    def __init__(self, title, unit, columns):
        super().__init__()
        self.title, self.unit, self.columns = title, unit, columns
        self.samples = np.empty(0)  # Small rendered series only, never full history.
        self.view = WaveViewBox()
        self.plot = pg.PlotWidget(viewBox=self.view, background="#101c2c")
        self.plot.setTitle(title + " · 双击放大", color="#d4deea")
        self.plot.setLabel("left", title, units=unit)
        self.plot.setLabel("bottom", "本次连接 H743 相对时间", units="s")
        self.plot.showGrid(x=True, y=True, alpha=.18)
        self.plot.addLegend(offset=(10, 10))
        self.plot.hideButtons()
        self.curves = [self.plot.plot(pen=pg.mkPen(color, width=1.2), name=axis,
                                      connect="finite", skipFiniteCheck=False)
                       for axis, color in zip("XYZ", ("#ff737e", "#59dfba", "#6db6ff"))]
        self.view.setXRange(0, 10, padding=0)
        self.view.enableAutoRange(y=True)
        self.setMinimumSize(240, 160)
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self.plot)

    def set_series(self, series):
        for curve, column in zip(self.curves, self.columns):
            x, y = series[column-1]
            curve.setData(x, y, connect="finite")
        self.samples = series[self.columns[0]-1][0]


class RenderWorker(QtCore.QObject):
    result = QtCore.Signal(object)

    def __init__(self):
        super().__init__()
        self.condition = threading.Condition()
        self.pending = None
        self.stopping = False
        self.thread = threading.Thread(target=self.run, name="waveform-render", daemon=True)
        self.thread.start()

    def request(self, request):
        with self.condition:
            self.pending = request  # Only newest pending view; no unbounded work queue.
            self.condition.notify()

    def run(self):
        while True:
            with self.condition:
                self.condition.wait_for(lambda: self.pending is not None or self.stopping)
                if self.stopping:
                    return
                request, self.pending = self.pending, None
            history, generation, version, lo, hi, pixels = request
            try:
                info, series = render_history(history, lo, hi, pixels)
                self.result.emit((generation, version, info, series, None))
            except Exception as exc:
                self.result.emit((generation, version, None, None,
                                  f"波形绘制失败（不影响录制）: {type(exc).__name__}: {exc}"))

    def shutdown(self):
        with self.condition:
            self.stopping = True
            self.pending = None
            self.condition.notify()
        self.thread.join(timeout=2)


class ExpandedChart(QtWidgets.QMainWindow):
    def __init__(self, deck, chart):
        super().__init__(deck.window())
        self.deck, self.chart = deck, chart
        self.setWindowTitle(chart.title + " · H743 波形（Esc 返回）")
        toolbar = self.addToolBar("波形窗口")
        toolbar.setMovable(False)
        for text, callback in (("全屏", self.showFullScreen),
                               ("左半屏", lambda: self.half(False)),
                               ("右半屏", lambda: self.half(True)),
                               ("回到实时", deck.live), ("查看全程", deck.all_history),
                               ("纵轴自动", deck.auto_y), ("返回主窗口", self.close)):
            action = toolbar.addAction(text)
            action.triggered.connect(callback)
        self.statusBar().showMessage("滚轮缩放时间 · 拖动回看 · Shift+滚轮缩放幅值 · 早期数据为分段峰值")
        self.setCentralWidget(chart)
        escape = QtGui.QShortcut(QtGui.QKeySequence("Escape"), self)
        escape.activated.connect(self.close)

    def half(self, right):
        screen = self.screen() or QtGui.QGuiApplication.primaryScreen()
        rect = screen.availableGeometry()
        middle = rect.left()+rect.width()//2
        self.showNormal()
        self.setGeometry(middle if right else rect.left(), rect.top(),
                         rect.right()-middle+1 if right else rect.width()//2, rect.height())

    def closeEvent(self, event):
        self.deck.restore_chart(self)
        event.accept()


class WaveformPanel(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.history = None
        self.generation = self.version = 0
        self.follow = True
        self.changing = False
        self.last_request = None
        self.last_submit = 0.0
        self.expanded = {}
        self.worker = RenderWorker()
        self.worker.result.connect(self.rendered)
        self.last_error = None
        self.accel_chart = Chart("加速度", "g", (1, 2, 3))
        self.gyro_chart = Chart("角速度", "°/s", (4, 5, 6))
        self.charts = (self.accel_chart, self.gyro_chart)
        self.places = {}
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        controls = QtWidgets.QHBoxLayout()
        for text, callback in (("回到实时", self.live), ("查看全程", self.all_history),
                               ("纵轴自动", self.auto_y)):
            button = QtWidgets.QPushButton(text)
            button.clicked.connect(callback)
            controls.addWidget(button)
        controls.addStretch()
        help_label = QtWidgets.QLabel("滚轮缩放 · 拖动回看 · 双击放大")
        help_label.setWordWrap(True)
        help_label.setToolTip("滚轮：以鼠标为中心缩放时间；拖动：回看；Shift+滚轮或纵轴区滚轮：缩放幅值。")
        layout.addLayout(controls)
        layout.addWidget(help_label)
        split = self.chart_splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        split.setChildrenCollapsible(False)
        split.setHandleWidth(6)
        for chart in self.charts:
            holder = QtWidgets.QWidget()
            holder_layout = QtWidgets.QVBoxLayout(holder)
            holder_layout.setContentsMargins(0, 0, 0, 0)
            holder_layout.addWidget(chart)
            self.places[chart] = holder_layout
            split.addWidget(holder)
            chart.view.manual.connect(self.manual)
            chart.view.sigXRangeChanged.connect(lambda view, value, c=chart: self.range_changed(c, value))
            chart.view.expanded.connect(lambda c=chart: self.toggle_expand(c))
        layout.addWidget(split, 1)
        self.state = QtWidgets.QLabel("等待 IMU 数据；波形历史不等于录制文件")
        self.state.setWordWrap(True)
        self.state.setToolTip("早期历史仅保留各轴独立峰值，不能恢复原始细节，也不是同一时刻的六轴样本；波形历史不等于录制文件。")
        self.state.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        layout.addWidget(self.state)
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(100)

    def set_stacked(self, stacked):
        self.chart_splitter.setOrientation(QtCore.Qt.Vertical if stacked else QtCore.Qt.Horizontal)
        self.chart_splitter.setSizes([500, 500])

    def set_history(self, history):
        self.history = history
        self.generation += 1
        self.version += 1
        self.last_request = None
        self.last_error = None
        self.follow = True
        empty = [(np.empty(0), np.empty(0)) for _ in range(6)]
        for chart in self.charts:
            chart.set_series(empty)
        self.set_range(0, 10)
        self.refresh()

    def manual(self):
        self.follow = False
        self.version += 1
        self.last_request = None

    def range_changed(self, chart, value):
        if self.changing:
            return
        # This path also handles programmatic range changes in tests. Explicit
        # mouse/wheel handlers own the transition out of live-follow mode.
        self.changing = True
        try:
            for other in self.charts:
                if other is not chart:
                    other.view.setXRange(*value, padding=0)
        finally:
            self.changing = False
        self.last_request = None

    def set_range(self, lo, hi):
        self.changing = True
        try:
            for chart in self.charts:
                chart.view.setXRange(lo, max(hi, lo+.0001), padding=0)
        finally:
            self.changing = False

    def live(self):
        self.follow = True
        self.version += 1
        self.last_request = None
        self.refresh()

    def all_history(self):
        self.manual()
        if self.history is not None:
            self.set_range(0, max(.01, self.history.info().duration))
        self.refresh()

    def auto_y(self):
        for chart in self.charts:
            chart.view.enableAutoRange(y=True)

    def refresh(self):
        if self.history is None:
            return
        info = self.history.info()
        if info.origin_us is None:
            return
        if self.follow:
            self.set_range(max(0, info.duration-10), max(10, info.duration))
        lo, hi = self.accel_chart.view.viewRange()[0]
        exact_start = (info.raw_start_us-info.origin_us)/1e6 if info.raw_start_us is not None else info.duration
        fidelity = "完整样本" if lo >= exact_start else "分段峰值（早期细节不可恢复）" + (" + 最近完整样本" if hi >= exact_start else "")
        warning = ""
        if info.warning:
            end = ((info.summary_end_us-info.origin_us)/1e6 if info.summary_end_us is not None else 0)
            warning = f"\n警告：{info.warning}；全程摘要截止 {end:.3f} s"
        self.state.setText(f'{"实时跟随" if self.follow else "历史查看"} | 当前 {lo:.3f}–{hi:.3f} s | '
                           f'总时长 {info.duration:.3f} s\n{info.samples:,} 样本 | {fidelity} | '
                           f'历史内存 {info.allocated_bytes/1024**2:.1f} MiB'
                           + warning + ("\n"+self.last_error if self.last_error else ""))
        # Geometry changes, new samples and navigation all invalidate the request.
        pixels = max(100, max(int(c.view.width()) for c in self.charts))
        # Once a historical view lies wholly before the exact-window cutoff,
        # its immutable summaries no longer change. Do not recopy/recompute them
        # for every newly arriving sample. Wide live views are rate-limited;
        # navigation/resize always submits immediately and work is coalesced.
        stable = hi < exact_start - 1/30
        content = 0 if stable else info.samples
        key = (self.generation, self.version, content, lo, hi, pixels)
        now = time.monotonic()
        same_view = self.last_request is not None and self.last_request[:2] == key[:2] and self.last_request[3:] == key[3:]
        if key != self.last_request and not (same_view and hi-lo > 60 and now-self.last_submit < 1):
            self.last_request = key
            self.last_submit = now
            self.worker.request((self.history, self.generation, self.version, lo, hi, pixels))

    @QtCore.Slot(object)
    def rendered(self, result):
        generation, version, info, series, error = result
        if generation != self.generation or version != self.version:
            return  # Result from old connection or navigation, never overwrite new view.
        self.last_error = error
        if series is not None:
            for chart in self.charts:
                chart.set_series(series)

    def toggle_expand(self, chart):
        if chart in self.expanded:
            self.expanded[chart].close()
            return
        holder = self.places[chart]
        holder.removeWidget(chart)
        placeholder = QtWidgets.QPushButton(chart.title + "已在独立窗口显示 · 点击返回")
        placeholder.clicked.connect(lambda: self.expanded[chart].close())
        holder.addWidget(placeholder)
        window = ExpandedChart(self, chart)
        window.placeholder = placeholder
        self.expanded[chart] = window
        window.showFullScreen()

    def restore_chart(self, window):
        chart = window.chart
        if chart not in self.expanded:
            return
        del self.expanded[chart]
        window.takeCentralWidget()
        window.placeholder.deleteLater()
        self.places[chart].addWidget(chart)
        chart.show()
        window.deleteLater()

    def shutdown(self):
        self.timer.stop()
        for window in list(self.expanded.values()):
            window.close()
        self.worker.shutdown()
