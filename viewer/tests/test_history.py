"""Waveform history correctness, independent of hardware, GUI and recording."""
import unittest
from unittest.mock import patch
import numpy as np
from imu_viewer.history import WaveformHistory, render_history, RAW, SUMMARY
from imu_viewer.protocol import Frame


def feed(history, start, count, values=None, step=1000, **kwargs):
    ix = np.arange(start, start+count, dtype=np.uint64)
    if values is None:
        values = np.tile((ix % 101).astype(np.int16)[:, None], (1, 6))
    history.append_arrays(1_000_000+ix*step, values, ix.astype(np.uint32), **kwargs)


def summaries(history):
    return np.concatenate(history.snapshot(0, 1e9)[1])


class HistoryTests(unittest.TestCase):
    def test_long_history_exact_recent_and_visible_only_snapshot(self):
        h = WaveformHistory()
        for start in range(0, 31000, 1000):
            feed(h, start, 1000)
        self.assertEqual(h.info().samples, 31000)
        self.assertEqual(h.info().duration, 30.999)
        info, old = render_history(h, 0, 1, 1200)
        self.assertGreater(len(old[0][0]), 30)
        _, recent = render_history(h, 30, 30.999, 2000)
        np.testing.assert_allclose(recent[0][0], np.arange(30000, 31000)/1000)
        selected = h.snapshot(30, 30.999)[1]
        self.assertLess(sum(map(len, selected)), 520)  # not the entire session
        raw = h.snapshot(0, 100)[2]
        self.assertLessEqual(sum(map(len, raw)), 12048)
        self.assertEqual(raw[0].dtype, RAW)
        self.assertEqual(selected[0].dtype, SUMMARY)

    def test_bucket_boundary_spikes_real_times_independent_axes_and_batches(self):
        h = WaveformHistory(recent_s=.001)
        ts = np.array([0, 1000, 11000, 21000, 33333, 33334, 60000, 66666, 66667], np.uint64)
        values = np.full((len(ts), 6), 7, np.int16)
        for axis in range(6):
            values[axis % 3+1, axis] = -100-axis
            values[(axis+1) % 3+1, axis] = 200+axis
        # Split a bucket across multiple USB deliveries.
        for a,b in ((0,2),(2,4),(4,6),(6,9)):
            h.append_arrays(1_000_000+ts[a:b], values[a:b], np.arange(a,b,dtype=np.uint32))
        result = summaries(h)
        self.assertEqual(list(result['start']), [1000000,1033334,1066667])
        self.assertEqual(list(result['end']), [1033333,1066666,1066667])
        for row, inds in zip(result, (np.arange(5), np.arange(5,8), np.arange(8,9))):
            for axis in range(6):
                low = inds[values[inds,axis].argmin()]
                high = inds[values[inds,axis].argmax()]
                self.assertEqual(row['t'][1,axis], 1_000_000+ts[low])
                self.assertEqual(row['t'][2,axis], 1_000_000+ts[high])
                self.assertEqual(row['v'][1,axis], values[low,axis])
                self.assertEqual(row['v'][2,axis], values[high,axis])
        # Pixel envelopes must retain positive AND negative peaks on every axis.
        _, curves = render_history(h, 0, .033333, 1)
        for axis,(x,y) in enumerate(curves):
            scale = 24/32768 if axis < 3 else 2000/32768
            self.assertAlmostEqual(y.min()/scale, -100-axis)
            self.assertAlmostEqual(y.max()/scale, 200+axis)
            self.assertIn(ts[values[:5,axis].argmin()]/1e6, x)

    def test_constant_values_and_large_integer_origin(self):
        h = WaveformHistory(recent_s=.1)
        origin = 2**55+123
        h.append_arrays(origin+np.arange(500,dtype=np.uint64)*1000,
                        np.full((500,6), 42, np.int16), np.arange(500,dtype=np.uint32))
        _, curves = render_history(h, 0, .499, 1200)
        self.assertTrue(np.all(np.diff(curves[0][0]) >= 0))
        self.assertIn(.401, curves[0][0])  # integer subtraction before float
        self.assertTrue(np.all(curves[0][1] == 42*24/32768))
        self.assertLess(h.info().allocated_bytes, h.max_bytes)

    def test_wrap_gap_quality_and_invalid_source_break_lines(self):
        h = WaveformHistory(recent_s=.001)
        ts = np.arange(9, dtype=np.uint64)*1000+1_000_000
        seq = np.array([0xfffffffe,0xffffffff,0,2,3,4,5,6,7],np.uint32)
        flags = np.array([16,1,0,0,2,0,0,64,0], np.uint16)
        invalid = np.array([False]*6+[True,False,False])
        h.append_arrays(ts, np.tile(np.arange(9,dtype=np.int16)[:,None],(1,6)), seq,flags,invalid)
        _, curves = render_history(h, 0, .008, 100)
        x,y = curves[0]
        self.assertTrue(np.isnan(x).any())
        for invalid_t in (.004,.006,.007):
            self.assertNotIn(invalid_t,x)
        self.assertTrue(np.isfinite(y[x==.001]).all())  # timing flag allowed
        i0 = np.flatnonzero(x==0)[0]
        i2 = np.flatnonzero(x==.002)[0]
        self.assertFalse(np.isnan(x[i0:i2]).any())  # sequence wrap not a gap
        self.assertTrue(np.isnan(x[i2:np.flatnonzero(x==.003)[0]]).any())

    def test_timestamp_anomaly_never_fabricates_monotonic_time(self):
        h = WaveformHistory()
        h.append_arrays([1000000,1001000,1000500,1003000],np.zeros((4,6),np.int16),[0,1,2,3])
        self.assertEqual(h.info().samples,2)
        self.assertIn('时间戳异常', h.info().warning)
        h.append_arrays([1004000],np.zeros((1,6),np.int16),[2])
        _, curves = render_history(h,0,.01)
        self.assertNotIn(.0005,curves[0][0])
        self.assertTrue(np.isnan(curves[0][0]).any())

    def test_capacity_freezes_summary_preserves_recent_and_old_history(self):
        h = WaveformHistory(recent_s=.1,max_bytes=256*1024)
        for start in range(0,20000,100):
            feed(h,start,100)
        self.assertIsNotNone(h.info().warning)
        cutoff = h.info().summary_end_us
        before = summaries(h).copy()
        for start in range(20000,22000,100):
            feed(h,start,100)
        self.assertEqual(h.info().summary_end_us,cutoff)
        np.testing.assert_array_equal(before,summaries(h))
        self.assertEqual(h.info().samples,22000)
        self.assertLessEqual(h.info().allocated_bytes,h.max_bytes)
        _, curves = render_history(h,0,22)
        self.assertTrue(np.isnan(curves[0][0]).any())
        self.assertAlmostEqual(curves[0][0][-1],21.999)
        self.assertTrue(np.all(curves[0][0][np.isfinite(curves[0][0])] <= 21.999))

    def test_summary_allocation_failure_recent_continues(self):
        h = WaveformHistory()
        with patch.object(h.summary,'append',side_effect=MemoryError):
            feed(h,0,100)
        self.assertIn('内存分配失败',h.info().warning)
        feed(h,100,100)
        self.assertEqual(h.info().samples,200)
        _, curves = render_history(h,0,1)
        self.assertAlmostEqual(curves[0][0][-1],.199)

    def test_preparation_allocation_failure_is_visible_and_does_not_escape(self):
        h = WaveformHistory()
        frames = [Frame(0,1000000,1,2,3,4,5,6,20,10)]
        with patch('imu_viewer.history.np.fromiter',side_effect=MemoryError):
            h.append_frames(frames)
        self.assertIn('内存分配失败',h.info().warning)
        h.append_frames(frames)
        self.assertEqual(h.info().samples,1)


if __name__ == '__main__':
    unittest.main()
