import unittest
import numpy as np
from imu_viewer.clock import fit_clock, align_camera

class ClockTests(unittest.TestCase):
    def observations(self, delay=0, slope=1.000075):
        rng = np.random.default_rng(912)
        d0 = 0x123456789ABCDEF0 * 1000
        h0 = 50000000000000
        x = range(0,60000000000,10000000)
        # Delivery completion cannot precede an earlier completion, even with jitter.
        host = np.maximum.accumulate([h0+round(t*slope)+delay+int(jitter)
                for t,jitter in zip(x,rng.exponential(2e6,len(x)))])
        return [(d0+t,int(h)) for t,h in zip(x,host)]

    def test_drift_jitter_and_fixed_latency_unidentifiable(self):
        pairs = self.observations(delay=10000000)
        m = fit_clock(pairs)
        self.assertLess(abs(m.slope-1.000075),5e-6)
        self.assertLess(abs(m.host_origin_ns+m.offset_ns-50000000000000-10000000),5e5)
        self.assertFalse(m.report()["fixed_delay_identifiable"])
        self.assertFalse(m.report()["residual_is_sync_accuracy"])
        # Constant latency is inseparable from clock offset.
        m2 = fit_clock(self.observations(delay=50000000))
        self.assertAlmostEqual(m2.host_origin_ns-m.host_origin_ns+m2.offset_ns-m.offset_ns,40000000,delta=2)
        self.assertAlmostEqual(m2.slope,m.slope,places=10)
        self.assertLess(abs(m.from_host(m.to_host(pairs[1234][0]))-pairs[1234][0]),2)

    def test_repeatability_and_integer_anchors(self):
        pairs = self.observations()
        self.assertEqual(fit_clock(pairs).report(),fit_clock(pairs).report())
        self.assertIsInstance(fit_clock(pairs).device_origin_ns,int)

    def test_reject_bad_clocks(self):
        for pairs in ([], [(0,0)]*30, [(i*1_000_000,i*1_000_000) for i in range(30)],
                      [(i*1_000_000_000,i*2_000_000_000) for i in range(30)]):
            with self.assertRaises(ValueError):
                fit_clock(pairs)
        pairs = self.observations()
        pairs[2] = (pairs[1][0],pairs[2][1])
        with self.assertRaises(ValueError):
            fit_clock(pairs)

    def test_no_snapping_and_domain_guard(self):
        p = [(i*100_000_000,i*100_000_000+1_000_000) for i in range(40)]
        m = fit_clock(p)
        t = align_camera(2345678901,m,m)
        self.assertEqual(t,2345678901)
        self.assertNotEqual(t%1_000_000,0)
        with self.assertRaises(ValueError):
            align_camera(-1,m,m)

if __name__ == "__main__":
    unittest.main()
