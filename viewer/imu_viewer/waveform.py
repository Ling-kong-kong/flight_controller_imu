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
            self.scaleBy(y=factor, center=center)
        else:
            # Passing (factor, 1) still explicitly sets BOTH axes in
            # pyqtgraph and disables Y auto-range, even though Y did not move.
            # Affect only X so live amplitude fitting keeps working.
            self.scaleBy(x=factor, center=center)
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
        self.fixed_y = False
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
        self.view.setAutoVisible(y=True)
        self.view.enableAutoRange(y=True)
        self.y_mode = QtWidgets.QLabel()
        self.y_mode.setToolTip("自动：适配可见时间区间；手动：回到实时恢复自动；手动（固定）：“纵轴范围…”的显式固定值，回到实时仍保留，纵轴自动可解除。")
        self.view.sigStateChanged.connect(self.update_y_mode)
        self.update_y_mode()
        self.setMinimumSize(240, 160)
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self.y_mode)
        layout.addWidget(self.plot)

    def update_y_mode(self, *_):
        text = "纵轴自动" if self.view.autoRangeEnabled()[1] else "纵轴手动"
        if self.fixed_y and not self.view.autoRangeEnabled()[1]:
            text += "（固定）"
        if self.y_mode.text() != text:
            self.y_mode.setText(text)

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


class YRangeDialog(QtWidgets.QDialog):
    """Display-only settings; modeless so acquisition/preview stay interactive."""

    def __init__(self, deck):
        super().__init__(deck)
        self.deck = deck
        self.setWindowTitle("波形纵轴范围")
        self.fields = []
        layout = QtWidgets.QVBoxLayout(self)
        hint = QtWidgets.QLabel("纵轴为幅值，不是时间。固定范围可观察细微变化，但范围外波形会被裁切。\n只影响显示，不改变采集、历史压缩或录制数据。")
        hint.setWordWrap(True)
        layout.addWidget(hint)
        for chart in deck.charts:
            group = QtWidgets.QGroupBox(f"{chart.title}（{chart.unit}）")
            form = QtWidgets.QFormLayout(group)
            fixed = QtWidgets.QCheckBox("固定纵轴（实时／历史均保留）")
            low, high = QtWidgets.QDoubleSpinBox(), QtWidgets.QDoubleSpinBox()
            for spin in (low, high):
                spin.setDecimals(6)
                spin.setRange(-1e9, 1e9)
                spin.setSingleStep(.01 if chart.unit == "g" else .1)
                spin.setSuffix(" " + chart.unit)
                spin.setKeyboardTracking(False)
            form.addRow(fixed)
            form.addRow("下限", low)
            form.addRow("上限", high)
            fixed.toggled.connect(low.setEnabled)
            fixed.toggled.connect(high.setEnabled)
            self.fields.append((fixed, low, high))
            layout.addWidget(group)
        self.error = QtWidgets.QLabel()
        self.error.setStyleSheet("color: #f06464")
        self.error.setWordWrap(True)
        layout.addWidget(self.error)
        self.buttons = QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.Apply |
                                                  QtWidgets.QDialogButtonBox.Close)
        self.buttons.button(QtWidgets.QDialogButtonBox.Apply).setText("应用")
        self.buttons.button(QtWidgets.QDialogButtonBox.Close).setText("关闭")
        self.buttons.button(QtWidgets.QDialogButtonBox.Apply).clicked.connect(self.apply)
        self.buttons.rejected.connect(self.close)
        layout.addWidget(self.buttons)

    def load_ranges(self):
        self.error.clear()
        for chart, (fixed, low, high) in zip(self.deck.charts, self.fields):
            bounds = chart.view.viewRange()[1]
            low.setValue(bounds[0])
            high.setValue(bounds[1])
            fixed.setChecked(chart.fixed_y)
            low.setEnabled(chart.fixed_y)
            high.setEnabled(chart.fixed_y)

    def apply(self):
        ranges = [(low.value(), high.value()) if fixed.isChecked() else None
                  for fixed, low, high in self.fields]
        try:
            self.deck.set_y_ranges(ranges)
        except ValueError as exc:
            self.error.setText(str(exc))
            return
        self.error.clear()


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
                               ("纵轴自动", deck.auto_y), ("纵轴范围…", deck.show_ranges),
                               ("返回主窗口", self.close)):
            action = toolbar.addAction(text)
            action.triggered.connect(callback)
        time_controls, self.time_span_spin = deck.make_time_controls()
        toolbar.addWidget(time_controls)
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
    time_span_changed = QtCore.Signal(float)

    def __init__(self):
        super().__init__()
        self.history = None
        self.generation = self.version = 0
        self.follow = True
        self.time_span = 10.0
        self.changing = False
        self.last_request = None
        self.last_submit = 0.0
        self.expanded = {}
        self.range_dialog = None
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
                               ("纵轴自动", self.auto_y), ("纵轴范围…", self.show_ranges)):
            button = QtWidgets.QPushButton(text)
            button.clicked.connect(callback)
            controls.addWidget(button)
        time_controls, self.time_span_spin = self.make_time_controls()
        controls.addWidget(time_controls)
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

    def show_ranges(self):
        if self.range_dialog is None:
            self.range_dialog = YRangeDialog(self)
        if not self.range_dialog.isVisible():
            self.range_dialog.load_ranges()
        self.range_dialog.show()
        self.range_dialog.raise_()
        self.range_dialog.activateWindow()

    def make_time_controls(self):
        widget = QtWidgets.QWidget()
        layout = QtWidgets.QHBoxLayout(widget)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(QtWidgets.QLabel("时间窗"))
        spin = QtWidgets.QDoubleSpinBox()
        spin.setObjectName("timeSpan")
        spin.setDecimals(3)
        spin.setRange(.001, 86400)
        spin.setSingleStep(.1)
        spin.setSuffix(" s")
        spin.setKeyboardTracking(False)
        spin.setValue(self.time_span)
        spin.setToolTip("横轴显示时长；实时跟随最新数据，历史查看以当前中心调整。不会改变采样或最近 10 秒完整历史策略。")
        spin.valueChanged.connect(self.set_time_span)
        self.time_span_changed.connect(spin.setValue)
        layout.addWidget(spin)
        return widget, spin

    def set_time_span(self, seconds):
        seconds = float(seconds)
        if not np.isfinite(seconds) or not .001 <= seconds <= 86400:
            raise ValueError("时间窗必须在 0.001–86400 秒之间")
        if seconds == self.time_span:
            return
        self.time_span = seconds
        self.time_span_changed.emit(seconds)
        self.version += 1
        self.last_request = None
        if not self.follow:
            lo, hi = self.accel_chart.view.viewRange()[0]
            center = (lo+hi)/2
            self.set_range(center-seconds/2, center+seconds/2)
        elif self.history is None or self.history.info().origin_us is None:
            self.set_range(0, seconds)
        self.refresh()

    def set_stacked(self, stacked):
        self.chart_splitter.setOrientation(QtCore.Qt.Vertical if stacked else QtCore.Qt.Horizontal)
        self.chart_splitter.setSizes([500, 500])

    def set_history(self, history):
        if self.range_dialog is not None:
            self.range_dialog.close()
        self.history = history
        self.generation += 1
        self.version += 1
        self.last_request = None
        self.last_error = None
        self.follow = True
        empty = [(np.empty(0), np.empty(0)) for _ in range(6)]
        for chart in self.charts:
            chart.set_series(empty)
        self.set_range(0, self.time_span)
        self.auto_y()
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
        for chart in self.charts:
            if not chart.fixed_y:
                chart.view.enableAutoRange(y=True)
        self.follow = True
        self.version += 1
        self.last_request = None
        self.refresh()

    def all_history(self):
        self.manual()
        if self.history is not None:
            self.set_range(0, max(.01, self.history.info().duration))
        self.refresh()

    def set_y_ranges(self, ranges):
        # Validate everything before applying: a bad second range must not
        # partially change the first chart. None requests visible-data auto Y.
        if len(ranges) != len(self.charts):
            raise ValueError("需要分别提供加速度和角速度范围")
        for bounds in ranges:
            if bounds is not None and (len(bounds) != 2 or
                    not all(np.isfinite(v) for v in bounds) or bounds[0] >= bounds[1]):
                raise ValueError("纵轴下限必须小于上限，且必须为有限数值")
        for chart, bounds in zip(self.charts, ranges):
            chart.fixed_y = bounds is not None
            if bounds is None:
                chart.view.enableAutoRange(y=True)
            else:
                chart.view.setYRange(*bounds, padding=0)
            chart.update_y_mode()

    def auto_y(self):
        self.set_y_ranges([None, None])

    def refresh(self):
        if self.history is None:
            return
        info = self.history.info()
        if info.origin_us is None:
            return
        if self.follow:
            self.set_range(max(0, info.duration-self.time_span), max(self.time_span, info.duration))
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
        if self.range_dialog is not None:
            self.range_dialog.close()
        for window in list(self.expanded.values()):
            window.close()
        self.worker.shutdown()
