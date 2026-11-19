"""Pure manual-calibration lifecycle. Caller serializes access; no device I/O."""
import secrets
import time
from .protocol import ControlReply, recalibrate_command


class Recalibration:
    def __init__(self):
        self.state = "idle"
        self.request_id = secrets.randbelow(0xffffffff) + 1
        self.boundary = None
        self.deadline = None
        self.events = []

    @property
    def busy(self):
        return self.state in ("queued", "awaiting_ack", "calibrating", "waiting_stationary")

    @property
    def masked(self):
        return self.busy or self.state == "ack_timeout"

    def change(self, state):
        self.state = state
        self.events.append(dict(state=state, request_id=self.request_id, boundary=self.boundary))

    def request(self):
        if self.busy:
            raise RuntimeError("IMU recalibration already pending")
        self.request_id = (self.request_id + 1) & 0xffffffff or secrets.randbelow(0xffffffff)+1
        self.boundary = None
        self.change("queued")
        return self.request_id

    def command(self, now=None):
        if self.state != "queued":
            return None
        self.deadline = (time.monotonic() if now is None else now) + 3
        self.change("awaiting_ack")
        return recalibrate_command(self.request_id)

    def tick(self, now=None):
        now = time.monotonic() if now is None else now
        if self.state == "awaiting_ack" and now >= self.deadline:
            self.change("ack_timeout")
        elif self.state == "calibrating" and now >= self.deadline:
            # Moving board can legitimately remain uncalibrated indefinitely.
            self.change("waiting_stationary")

    def accept(self, record, now=None):
        now = time.monotonic() if now is None else now
        if isinstance(record, ControlReply):
            if (record.request_id == self.request_id and self.state in
                    ("awaiting_ack", "ack_timeout") and record.status == 0):
                self.boundary = dict(timestamp_us=record.timestamp_us, sequence=record.sequence)
                self.deadline = now + 30
                self.change("calibrating")
        elif self.state in ("calibrating", "waiting_stationary"):
            if record.timestamp_us >= self.boundary["timestamp_us"] and record.attitude_valid:
                self.change("completed")

    def disconnect(self):
        if self.state != "disconnected":
            self.change("disconnected")

    def snapshot(self):
        return dict(state=self.state, busy=self.busy, masked=self.masked,
                    request_id=self.request_id, boundary=self.boundary)

    def drain_events(self):
        events, self.events = self.events, []
        return events
