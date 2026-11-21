import unittest
from imu_viewer.protocol import ControlReply, FusionDiagnostic, heading_zero_command
from imu_viewer.recalibration import Recalibration
from test_attitude import frame


def diagnostic(request, phase=2, **kwargs):
    values = dict(timestamp_us=1003000, sequence=3, phase=phase, reason=1,
                  recovery_count=20, recovery_elapsed_ms=800, innovation_deg=0.,
                  correction_rate_dps=0., flags=8192|32768, request_id=request)
    values.update(kwargs)
    return FusionDiagnostic(**values)


class HeadingLifecycleTests(unittest.TestCase):
    def pending(self, operation=1):
        state = Recalibration()
        request = state.request(operation)
        state.command(10)
        state.accept(ControlReply(request, 0, 1002000, 2, operation), 11)
        return state, request

    def test_magnetic_wait_is_not_six_axis_completion(self):
        state, request = self.pending()
        state.accept(diagnostic(request, 1), 12)
        state.accept(frame(3), 12)
        self.assertEqual(state.state, 'magnetic_wait')
        self.assertTrue(state.masked)
        state.accept(diagnostic(request, 3), 12.1)
        self.assertEqual(state.state, 'degraded')
        self.assertFalse(state.masked)

    def test_valid_frame_before_diagnostic_does_not_report_magnetic_success(self):
        state, request = self.pending()
        state.accept(frame(3), 12)
        self.assertNotEqual(state.state, 'completed')
        state.accept(diagnostic(request, 3), 12.1)
        self.assertEqual(state.state, 'degraded')

    def test_fresh_matching_diag_and_valid_boundary_required(self):
        state, request = self.pending()
        state.accept(diagnostic(request+1), 12)
        state.accept(frame(1), 12)
        self.assertNotEqual(state.state, 'completed')
        state.accept(diagnostic(request, timestamp_us=1001000), 12)
        state.accept(frame(3, flags=1024), 12)
        self.assertNotEqual(state.state, 'completed')
        state.accept(diagnostic(request), 12.1)
        state.accept(frame(4), 12.2)
        self.assertEqual(state.state, 'completed')
        self.assertEqual(state.snapshot(12.2)['diagnostic']['reason_name'], 'ok')
        self.assertIsNone(state.snapshot(13.3)['diagnostic'])

    def test_stale_device_diagnostic_does_not_complete(self):
        state, request = self.pending()
        state.accept(diagnostic(request, 1), 12)
        state.accept(frame(2000), 12.1)
        state.accept(diagnostic(request), 12.2)
        state.tick(14)
        self.assertNotEqual(state.state, 'completed')

    def test_zero_operation_must_match_ack_and_break_display_once(self):
        state = Recalibration()
        request = state.request(2)
        self.assertEqual(state.command(10), heading_zero_command(request))
        state.accept(ControlReply(request, 0, 1002000, 2), 11)
        self.assertEqual(state.state, 'awaiting_ack')
        state.accept(ControlReply(request, 0, 1002000, 2, 2), 11)
        state.accept(frame(1), 12)
        self.assertTrue(state.masked)
        state.accept(frame(3), 12)
        self.assertEqual(state.state, 'completed')
        self.assertTrue(state.consume_break())
        self.assertFalse(state.consume_break())

    def test_rejected_zero_never_retries_or_runs_full_reset(self):
        state = Recalibration()
        request = state.request(2)
        state.command(10)
        state.accept(ControlReply(request, 2, 1002000, 2, 2), 11)
        self.assertEqual(state.state, 'rejected')
        self.assertEqual(state.snapshot(11)['status'], 2)
        self.assertFalse(state.masked)
        self.assertIsNone(state.command(12))

    def test_legacy_completion_is_explicitly_unknown(self):
        state, request = self.pending()
        state.accept(frame(3), 12)
        state.tick(12.6)
        self.assertEqual(state.state, 'completed')
        self.assertIsNone(state.snapshot(12.6)['diagnostic'])


if __name__ == '__main__':
    unittest.main()
