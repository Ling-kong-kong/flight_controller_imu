"""Serialized, I/O-free calibration/heading control with board diagnostics."""
from dataclasses import asdict
import math
import secrets
import time
from .protocol import (ControlReply, Frame, FusionDiagnostic,
                       recalibrate_command, heading_zero_command)


def finite_json(value):
    """Diagnostics are untrusted float32 values; never emit JSON NaN/Infinity."""
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {k: finite_json(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [finite_json(v) for v in value]
    return value


class Recalibration:
    ACTIVE = ('queued', 'awaiting_ack', 'calibrating', 'waiting_stationary', 'magnetic_wait')

    def __init__(self):
        self.state = 'idle'
        self.request_id = secrets.randbelow(0xffffffff) + 1
        self.operation = 1
        self.status = None
        self.boundary = None
        self.deadline = None
        self.events = []
        self.diagnostic = None
        self.diagnostic_received = None
        self.diagnostics_supported = False
        self.latest_timestamp = None
        self.valid_received = None
        self.valid_timestamp = None
        self.break_pending = False

    @property
    def busy(self):
        return self.state in self.ACTIVE

    @property
    def masked(self):
        return self.busy or self.state == 'ack_timeout'

    def change(self, state):
        if self.state == state:
            return
        self.state = state
        self.events.append(dict(state=state, request_id=self.request_id, operation=self.operation,
                                status=self.status, boundary=self.boundary))

    def request(self, operation=1):
        if self.busy:
            raise RuntimeError('IMU control already pending')
        if operation not in (1, 2):
            raise ValueError('Unsupported IMU operation')
        self.request_id = (self.request_id + 1) & 0xffffffff or secrets.randbelow(0xffffffff)+1
        self.operation = operation
        self.status = None
        self.boundary = None
        self.valid_received = self.valid_timestamp = None
        self.break_pending = False
        self.change('queued')
        return self.request_id

    def command(self, now=None):
        if self.state != 'queued':
            return None
        self.deadline = (time.monotonic() if now is None else now) + 3
        self.change('awaiting_ack')
        return (heading_zero_command if self.operation == 2 else recalibrate_command)(self.request_id)

    def fresh_diagnostic(self, now):
        d = self.diagnostic
        if (d is None or self.diagnostic_received is None or
                not 0 <= now-self.diagnostic_received <= 1 or
                (self.latest_timestamp is not None and
                 abs(self.latest_timestamp-d.timestamp_us) > 1000000)):
            return None
        return d

    def _complete(self, now):
        if self.state not in ('calibrating', 'waiting_stationary', 'magnetic_wait'):
            return
        d = self.fresh_diagnostic(now)
        if self.operation == 1 and d is not None and d.request_id == self.request_id and self._after(d):
            if d.phase == 1:
                self.change('magnetic_wait')
            elif d.phase == 0:
                self.valid_received = self.valid_timestamp = None
                self.change('calibrating')
            elif d.phase in (2, 3) and self.valid_timestamp is not None and self.valid_timestamp >= d.timestamp_us:
                self.change('completed' if d.phase == 2 else 'degraded')
            return
        if self.valid_received is None:
            return
        if self.operation == 2:
            self.break_pending = True
            self.change('completed')
        elif not self.diagnostics_supported and now-self.valid_received >= .5:
            # Legacy firmware can prove six-axis validity, NOT magnetic completion.
            self.change('completed')

    def _after(self, record):
        return (self.boundary is not None and
                record.timestamp_us >= self.boundary['timestamp_us'] and
                ((record.sequence-self.boundary['sequence']) & 0xffffffff) < 0x80000000)

    def tick(self, now=None):
        now = time.monotonic() if now is None else now
        if self.state == 'awaiting_ack' and now >= self.deadline:
            self.change('ack_timeout')
        elif self.state == 'calibrating' and now >= self.deadline:
            self.change('waiting_stationary')
        self._complete(now)

    def accept(self, record, now=None):
        now = time.monotonic() if now is None else now
        if isinstance(record, ControlReply):
            if (record.request_id == self.request_id and record.operation == self.operation and
                    self.state in ('awaiting_ack', 'ack_timeout')):
                self.status = record.status
                if record.status != 0:
                    self.change('rejected')
                    return
                self.boundary = dict(timestamp_us=record.timestamp_us, sequence=record.sequence)
                self.deadline = now + 30
                self.change('calibrating')
        elif isinstance(record, FusionDiagnostic):
            self.diagnostics_supported = True
            # Do not replace a newer board observation with older backlog.
            if self.diagnostic is None or record.timestamp_us >= self.diagnostic.timestamp_us:
                self.diagnostic = record
                self.diagnostic_received = now
        elif isinstance(record, Frame):
            self.latest_timestamp = record.timestamp_us
            if self.state in ('calibrating', 'waiting_stationary', 'magnetic_wait') and self._after(record):
                if record.attitude_valid:
                    if self.valid_received is None:
                        self.valid_received = now
                    self.valid_timestamp = record.timestamp_us
                else:
                    self.valid_received = self.valid_timestamp = None
        self._complete(now)

    def consume_break(self):
        result, self.break_pending = self.break_pending, False
        return result

    def disconnect(self):
        self.diagnostic = self.diagnostic_received = None
        self.change('disconnected')

    def snapshot(self, now=None):
        now = time.monotonic() if now is None else now
        d = self.fresh_diagnostic(now)
        diagnostic = None if d is None else finite_json(dict(asdict(d), reason_name=d.reason_name))
        return dict(state=self.state, busy=self.busy, masked=self.masked, operation=self.operation,
                    status=self.status, request_id=self.request_id, boundary=self.boundary,
                    diagnostics_supported=self.diagnostics_supported, diagnostic=diagnostic)

    def drain_events(self):
        events, self.events = self.events, []
        return events
