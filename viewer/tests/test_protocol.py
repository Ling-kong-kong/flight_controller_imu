import unittest
from imu_viewer.protocol import Frame, Parser, Continuity

VECTOR = bytes.fromhex(
    "49 4D 55 32 02 03 30 00 78 56 34 12 F0 DE BC 9A "
    "78 56 34 12 00 80 FF 7F FF FF 7B 00 38 FE 15 03 "
    "70 02 00 00 F3 01 00 00 1F 03 41 01 69 BB 9C 26")

def frame(sequence=0, timestamp=1000000, flags=0):
    return Frame(sequence, timestamp, -32768, 32767, -1, 123, -456, 789, 624, 499, flags, 321)

class ProtocolTests(unittest.TestCase):
    def test_reference_vector(self):
        p = Parser()
        f, = p.feed(VECTOR)
        self.assertEqual(f.sequence, 0x12345678)
        self.assertEqual(f.timestamp_us, 0x123456789ABCDEF0)
        self.assertEqual(f.encode(), VECTOR)
        self.assertEqual(f.row()["timestamp_ns"], f.timestamp_us * 1000)
        self.assertEqual(f.ax_raw, -32768)
        self.assertEqual(f.flags, 0x031F)

    def test_all_split_positions_and_gluing(self):
        for split in range(49):
            p = Parser()
            result = p.feed(VECTOR[:split]) + p.feed(VECTOR[split:] + VECTOR)
            self.assertEqual(len(result), 2)

    def test_noise_crc_header_and_bounded_buffer(self):
        p = Parser()
        corrupt = bytearray(VECTOR)
        corrupt[21] ^= 1
        result = p.feed(b"x"*200000 + b"IMU2\x01\x03\x30\x00" + corrupt + VECTOR + b"IM")
        self.assertEqual(len(result), 1)
        self.assertEqual(p.stats["crc_errors"], 1)
        self.assertEqual(p.stats["header_errors"], 1)
        self.assertLess(len(p.buffer), 48)
        self.assertGreater(p.stats["bytes_discarded"], 200000)

    def test_wrap_gap_duplicate_regression(self):
        c = Continuity()
        self.assertIsNone(c.accept(frame(0xFFFFFFFF,1000000,16)))
        self.assertIsNone(c.accept(frame(0,1001000)))
        self.assertIsNone(c.accept(frame(3,1004000)))
        self.assertEqual(c.stats["missing_sequences"],2)
        self.assertIn("Duplicate",c.accept(frame(3,1005000)))
        self.assertIn("regressed",c.accept(frame(1,1006000)))

    def test_session_and_time_reset(self):
        c = Continuity()
        c.accept(frame())
        self.assertIn("session", c.accept(frame(0,2000000,16)))
        self.assertEqual(c.stats["sessions_observed"],2)
        self.assertIn("timestamp",c.accept(frame(1,10)))
        self.assertEqual(c.stats["timestamp_regressions"],1)

    def test_nominal_units_source_times_no_bias(self):
        f = frame()
        r = f.row()
        self.assertEqual(r["ax_g"], -24)
        self.assertAlmostEqual(r["ax_m_s2"],-24*9.80665)
        self.assertAlmostEqual(r["gy_deg_s"],-456*2000/32768)
        self.assertEqual(r["gyro_time_us"],f.timestamp_us-499)
        self.assertEqual(r["ax_raw"],-32768)

if __name__ == "__main__":
    unittest.main()
