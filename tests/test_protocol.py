"""Host tests: python3 -m unittest discover -s tests -v"""

import csv
from dataclasses import replace
import importlib.util
import io
import json
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import unittest
import zlib

MODULE_PATH = Path(__file__).resolve().parents[1] / "tools" / "imu_capture.py"
SPEC = importlib.util.spec_from_file_location("imu_capture", MODULE_PATH)
imu = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = imu
SPEC.loader.exec_module(imu)


class ProtocolTests(unittest.TestCase):
    def setUp(self):
        self.first = imu.Frame(1, 0x12345678, 0x123456789ABCDEF0, -32768, 32767, -1, 8)
        self.second = imu.Frame(2, 42, 9123, 123, -456, 789, 0)

    def test_wire_layout_crc_and_signed_axes(self):
        packet = self.first.encode()
        self.assertEqual(len(packet), 32)
        self.assertEqual(packet[:12], b"IMU1\x01\x01\x20\x00\x78\x56\x34\x12")
        self.assertEqual(packet[12:20], bytes.fromhex("f0debc9a78563412"))
        self.assertEqual(packet[20:28], bytes.fromhex("0080ff7fffff0800"))
        self.assertEqual(packet, bytes.fromhex(
            "494d55310101200078563412f0debc9a785634120080ff7fffff08001b00cfb4"))
        self.assertEqual(zlib.crc32(b"123456789"), 0xCBF43926)
        self.assertEqual(struct.unpack_from("<I", packet, 28)[0], zlib.crc32(packet[:28]))
        self.assertEqual(imu.StreamParser().feed(packet), [self.first])

    def test_every_split_and_one_byte_input(self):
        data = self.first.encode() + self.second.encode()
        for boundary in range(len(data) + 1):
            with self.subTest(boundary=boundary):
                parser = imu.StreamParser()
                decoded = parser.feed(data[:boundary]) + parser.feed(data[boundary:])
                self.assertEqual(decoded, [self.first, self.second])
                self.assertEqual(parser.bytes_discarded, 0)
        parser = imu.StreamParser()
        decoded = []
        for value in data:
            decoded += parser.feed(bytes([value]))
        self.assertEqual(decoded, [self.first, self.second])

    def test_noise_corruption_and_truncated_frame_resync(self):
        corrupt = bytearray(self.first.encode())
        corrupt[20] ^= 0x40
        data = b"noiseIM" + bytes(corrupt) + self.first.encode()[:17] + self.second.encode()
        parser = imu.StreamParser()
        self.assertEqual(parser.feed(data), [self.second])
        self.assertGreaterEqual(parser.crc_errors, 2)
        self.assertEqual(parser.bytes_discarded, len(data) - 32)

    def test_unknown_version_type_and_length_resync(self):
        for index, value in [(4, 2), (5, 0), (5, 3), (6, 31), (7, 1)]:
            with self.subTest(index=index, value=value):
                packet = bytearray(self.first.encode())
                packet[index] = value
                struct.pack_into("<I", packet, 28, zlib.crc32(packet[:28]))
                parser = imu.StreamParser()
                self.assertEqual(parser.feed(bytes(packet) + self.second.encode()), [self.second])
                self.assertEqual(parser.header_errors, 1)

    def test_parser_bounded_and_partial_magic_preserved(self):
        parser = imu.StreamParser()
        self.assertEqual(parser.feed(b"x" * 100_000 + b"IMU"), [])
        self.assertLess(len(parser.buffer), 32)
        self.assertEqual(parser.feed(self.second.encode()[3:]), [self.second])
        self.assertEqual(parser.bytes_received, 100_000 + 32)

    def test_sequence_wrap_rates_gaps_and_independent_sensors(self):
        capture = imu.Capture()
        frames = [imu.Frame(1, 0xFFFFFFFF, 1000, 1, 2, 3, 8),
                  imu.Frame(2, 12, 1100, 4, 5, 6, 8),
                  imu.Frame(1, 0, 1625, 1, 2, 3),
                  imu.Frame(2, 13, 1600, 4, 5, 6),
                  imu.Frame(1, 2, 2875, 1, 2, 3, 4)]
        capture.feed(b"".join(frame.encode() for frame in frames))
        accel, gyro = capture.summary()["legacy"]["accel"], capture.summary()["legacy"]["gyro"]
        self.assertAlmostEqual(accel["sequence_hz"], 1600)
        self.assertAlmostEqual(accel["received_hz"], 2e6 / 1875)
        self.assertEqual(accel["missing_sequences"], 1)
        self.assertEqual(accel["sequence_regressions"], 0)
        self.assertEqual(accel["period_us_mean"], 625)
        self.assertEqual(accel["flag_counts"]["ring_overflow"], 1)
        self.assertEqual(gyro["received_hz"], 2000)
        self.assertEqual(gyro["samples"], 2)

    def test_session_restart_and_unmarked_regressions(self):
        stats = imu.SensorStats()
        for frame in [imu.Frame(1, 100, 100000, 0, 0, 0, 8),
                      imu.Frame(1, 101, 100625, 0, 0, 0),
                      imu.Frame(1, 200, 5, 0, 0, 0, 8),
                      imu.Frame(1, 201, 630, 0, 0, 0)]:
            stats.add(frame)
        summary = stats.summary()
        self.assertEqual(summary["sessions_observed"], 2)
        self.assertEqual(summary["missing_sequences"], 0)
        self.assertEqual(summary["timestamp_regressions"], 0)
        self.assertEqual(summary["received_hz"], 1600)
        stats.add(imu.Frame(1, 201, 1255, 0, 0, 0))
        stats.add(imu.Frame(1, 199, 1880, 0, 0, 0))
        stats.add(imu.Frame(1, 200, 100, 0, 0, 0))
        self.assertEqual(stats.duplicate_sequences, 1)
        self.assertEqual(stats.sequence_regressions, 1)
        self.assertEqual(stats.timestamp_regressions, 1)

    def test_jitter_population_std_and_flags(self):
        stats = imu.SensorStats()
        for seq, timestamp in enumerate([0, 499, 1000, 1500]):
            stats.add(imu.Frame(2, seq, timestamp, 0, 0, 0, 15 if seq == 0 else 1))
        summary = stats.summary()
        self.assertEqual(summary["period_us_min"], 499)
        self.assertEqual(summary["period_us_max"], 501)
        self.assertEqual(summary["period_us_mean"], 500)
        self.assertAlmostEqual(summary["period_us_std"], (2 / 3) ** 0.5)
        self.assertEqual(summary["flag_counts"]["drdy_interval"], 4)
        self.assertEqual(summary["flag_counts"]["spi_error"], 1)

    def test_raw_capture_preserves_corrupt_bytes_and_csv_units(self):
        raw, output = io.BytesIO(), io.StringIO()
        frame = imu.Frame(1, 7, 999, 16384, -32768, 0, 8)
        data = b"junk" + frame.encode() + b"trailing"
        capture = imu.Capture(raw, output, units=True)
        capture.feed(data)
        self.assertEqual(raw.getvalue(), data)
        rows = list(csv.DictReader(io.StringIO(output.getvalue())))
        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0]["ax_raw"], "16384")
        self.assertEqual(rows[0]["legacy_sensor"], "accel")
        self.assertEqual(rows[0]["gx_raw"], "")
        self.assertEqual(float(rows[0]["ax_g"]), 12.0)
        self.assertEqual(float(rows[0]["ay_g"]), -24.0)

    def test_offline_cli_and_same_file_protection(self):
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / "in.bin"
            output = Path(directory) / "out.csv"
            original = self.second.encode()
            source.write_bytes(original)
            result = subprocess.run([sys.executable, str(MODULE_PATH), "--input", str(source),
                                     "--csv", str(output), "--stats"], capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertEqual(json.loads(result.stdout)["legacy"]["gyro"]["samples"], 1)
            self.assertTrue(output.exists())
            result = subprocess.run([sys.executable, str(MODULE_PATH), "--input", str(source),
                                     "--raw", str(source)], capture_output=True, text=True)
            self.assertNotEqual(result.returncode, 0)
            self.assertEqual(source.read_bytes(), original)


class PairedProtocolTests(unittest.TestCase):
    def setUp(self):
        self.first = imu.PairedFrame(0x12345678, 0x123456789ABCDEF0,
                                     -32768, 32767, -1, 123, -456, 789,
                                     624, 499, 0x031F, 321)
        self.second = imu.PairedFrame(42, 100000, 1, 2, 3, -4, -5, -6, 123, 345)

    def test_paired_wire_golden_vector_and_all_fields(self):
        packet = self.first.encode()
        self.assertEqual(len(packet), 48)
        self.assertEqual(packet, bytes.fromhex(
            "494d55320203300078563412f0debc9a785634120080ff7fffff7b0038fe1503"
            "70020000f30100001f03410169bb9c26"))
        self.assertEqual(packet[:12], b"IMU2\x02\x03\x30\x00\x78\x56\x34\x12")
        self.assertEqual(struct.unpack_from("<hhh", packet, 20), (-32768, 32767, -1))
        self.assertEqual(struct.unpack_from("<hhh", packet, 26), (123, -456, 789))
        self.assertEqual(struct.unpack_from("<IIHH", packet, 32), (624, 499, 0x031F, 321))
        self.assertEqual(struct.unpack_from("<I", packet, 44)[0], zlib.crc32(packet[:44]))
        self.assertEqual(imu.StreamParser().feed(packet), [self.first])
        self.assertEqual(self.second.accel_time_us, 99877)
        self.assertEqual(self.second.gyro_time_us, 99655)

    def test_paired_32bit_ages_and_late_saturation_field(self):
        frame = replace(self.first, accel_age_us=0xFFFFFFFF, gyro_age_us=0xFEDCBA98,
                        flags=0x03FF, tick_late_us=65535)
        self.assertEqual(imu.StreamParser().feed(frame.encode()), [frame])

    def test_paired_every_split_bytefeeds_and_mixed_legacy_stream(self):
        legacy = imu.Frame(1, 12, 60000, 1, 2, 3, 8)
        frames = [self.first, legacy, self.second]
        data = b"".join(frame.encode() for frame in frames)
        for boundary in range(len(data) + 1):
            with self.subTest(boundary=boundary):
                parser = imu.StreamParser()
                self.assertEqual(parser.feed(data[:boundary]) + parser.feed(data[boundary:]), frames)
                self.assertEqual(parser.bytes_discarded, 0)
        parser = imu.StreamParser()
        decoded = []
        for byte in data:
            decoded += parser.feed(bytes([byte]))
        self.assertEqual(decoded, frames)

    def test_paired_corruption_noise_and_truncation_recover(self):
        bad = bytearray(self.first.encode())
        bad[39] ^= 1
        data = b"noiseIMU" + bytes(bad) + self.first.encode()[:31] + self.second.encode()
        parser = imu.StreamParser()
        self.assertEqual(parser.feed(data), [self.second])
        self.assertGreaterEqual(parser.crc_errors, 2)
        self.assertEqual(parser.bytes_discarded, len(data) - 48)
        self.assertEqual(parser.summary()["valid_frames"], 1)

    def test_paired_invalid_header_and_bounded_partial_magic(self):
        for index, value in [(4, 1), (5, 1), (5, 2), (6, 32), (7, 1)]:
            with self.subTest(index=index, value=value):
                bad = bytearray(self.first.encode())
                bad[index] = value
                struct.pack_into("<I", bad, 44, zlib.crc32(bad[:44]))
                parser = imu.StreamParser()
                self.assertEqual(parser.feed(bytes(bad) + self.second.encode()), [self.second])
                self.assertEqual(parser.header_errors, 1)
        parser = imu.StreamParser()
        parser.feed(b"x" * 100000 + b"IMU")
        self.assertLess(len(parser.buffer), 48)
        self.assertEqual(parser.feed(self.second.encode()[3:]), [self.second])

    def test_paired_sequence_wrap_gaps_and_1000hz_grid(self):
        frames = [replace(self.second, sequence=0xFFFFFFFF, timestamp_us=100000, flags=16),
                  replace(self.second, sequence=0, timestamp_us=101000),
                  replace(self.second, sequence=2, timestamp_us=103000, flags=8),
                  replace(self.second, sequence=3, timestamp_us=104000, flags=0)]
        capture = imu.Capture()
        capture.feed(b"".join(frame.encode() for frame in frames))
        stats = capture.summary()["imu"]
        self.assertEqual(stats["sequence_hz"], 1000)
        self.assertEqual(stats["received_hz"], 750)
        self.assertEqual(stats["missing_sequences"], 1)
        self.assertEqual(stats["sequence_regressions"], 0)
        self.assertEqual(stats["period_us_mean"], 1000)
        self.assertEqual(stats["period_us_std"], 0)
        self.assertEqual(stats["flag_counts"]["ring_overflow"], 1)
        self.assertEqual(stats["sessions_observed"], 1)  # bit3 now means overflow, not session.
        self.assertEqual(capture.summary()["legacy"]["accel"]["samples"], 0)

    def test_paired_session_marker_and_timestamp_discontinuities(self):
        stats = imu.PairedStats()
        for frame in [replace(self.second, sequence=20, timestamp_us=100000, flags=16),
                      replace(self.second, sequence=21, timestamp_us=101000),
                      replace(self.second, sequence=0, timestamp_us=2000, flags=16),
                      replace(self.second, sequence=1, timestamp_us=3000)]:
            stats.add(frame)
        summary = stats.summary()
        self.assertEqual(summary["sessions_observed"], 2)
        self.assertEqual(summary["timestamp_regressions"], 0)
        self.assertEqual(summary["missing_sequences"], 0)
        self.assertEqual(summary["received_hz"], 1000)
        self.assertEqual(summary["source_regressions"], {"accel": 0, "gyro": 0})
        stats.add(replace(self.second, sequence=1, timestamp_us=4000))
        stats.add(replace(self.second, sequence=0, timestamp_us=5000))
        stats.add(replace(self.second, sequence=1, timestamp_us=4500))
        self.assertEqual(stats.summary()["duplicate_sequences"], 1)
        self.assertEqual(stats.summary()["sequence_regressions"], 1)
        self.assertEqual(stats.summary()["timestamp_regressions"], 1)

    def test_paired_age_skew_lateness_stats_separate_from_grid(self):
        stats = imu.PairedStats()
        for seq, (accel_age, gyro_age, late) in enumerate([(600, 100, 0), (200, 100, 100),
                                                         (0, 400, 200)]):
            stats.add(replace(self.second, sequence=seq, timestamp_us=100000 + 1000 * seq,
                              accel_age_us=accel_age, gyro_age_us=gyro_age,
                              tick_late_us=late, flags=0x03FF if seq == 0 else 0))
        summary = stats.summary()
        self.assertEqual(summary["period_us_std"], 0)
        self.assertEqual(summary["accel_age_us"]["min"], 0)
        self.assertEqual(summary["accel_age_us"]["max"], 600)
        self.assertAlmostEqual(summary["accel_age_us"]["mean"], 800 / 3)
        self.assertEqual(summary["source_skew_us"]["max"], 500)
        self.assertEqual(summary["tick_late_us"]["mean"], 100)
        self.assertAlmostEqual(summary["tick_late_us"]["std"], (20000 / 3) ** 0.5)
        self.assertEqual(set(summary["flag_counts"].values()), {0, 1})

    def test_paired_source_repeat_regression_and_invalid_age_visible(self):
        stats = imu.PairedStats()
        for seq, (timestamp, accel_age, gyro_age) in enumerate(
                [(100000, 100, 100), (101000, 1100, 100), (102000, 2200, 100)]):
            stats.add(replace(self.second, sequence=seq, timestamp_us=timestamp,
                              accel_age_us=accel_age, gyro_age_us=gyro_age))
        self.assertEqual(stats.summary()["source_repeats"], {"accel": 1, "gyro": 0})
        self.assertEqual(stats.summary()["source_regressions"], {"accel": 1, "gyro": 0})
        stats.add(replace(self.second, timestamp_us=1, accel_age_us=2, gyro_age_us=0))
        self.assertEqual(stats.summary()["invalid_source_times"], 1)

    def test_paired_raw_csv_units_and_offline_cli(self):
        raw, output = io.BytesIO(), io.StringIO()
        frame = replace(self.second, ax=16384, ay=-32768, gx=16384, gy=-32768, flags=16)
        data = b"junk" + frame.encode() + b"tail"
        capture = imu.Capture(raw, output, units=True)
        capture.feed(data)
        self.assertEqual(raw.getvalue(), data)
        row = list(csv.DictReader(io.StringIO(output.getvalue())))[0]
        self.assertEqual(row["ax_raw"], "16384")
        self.assertEqual(row["gyro_time_us"], "99655")
        self.assertEqual(row["accel_time_us"], "99877")
        self.assertEqual(row["protocol_version"], "2")
        self.assertEqual(row["legacy_sensor"], "")
        self.assertEqual(float(row["ax_g"]), 12)
        self.assertEqual(float(row["ay_g"]), -24)
        self.assertEqual(float(row["gx_deg_s"]), 1000)
        self.assertEqual(float(row["gy_deg_s"]), -2000)
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / "paired.bin"
            source.write_bytes(data)
            result = subprocess.run([sys.executable, str(MODULE_PATH), "--input", str(source),
                                     "--stats"], capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertEqual(json.loads(result.stdout)["imu"]["samples"], 1)


class AttitudeProtocolTests(unittest.TestCase):
    def setUp(self):
        self.frame = imu.PairedFrame(7, 100000, 1, 2, 1365, 0, 0, 0, 100, 200,
                                    flags=2048, protocol_version=3,
                                    qw=1., qx=0., qy=0., qz=0.,
                                    roll_deg=0., pitch_deg=0., yaw_deg=0.)

    def test_v3_golden_vector(self):
        f = replace(self.frame, sequence=0x12345678, timestamp_us=0x123456789abcdef0,
                    ax=-32768, ay=32767, az=-1, gx=123, gy=-456, gz=789,
                    accel_age_us=624, gyro_age_us=499, tick_late_us=321, yaw_deg=90.)
        self.assertEqual(f.encode(), bytes.fromhex(
            "494d553203034c0078563412f0debc9a785634120080ff7fffff7b0038fe1503"
            "70020000f3010000000841010000803f000000000000000000000000"
            "00000000000000000000b442187ba85c"))
        self.assertEqual(len(f.encode()), 76)
        self.assertEqual(imu.StreamParser().feed(f.encode()), [f])

    def test_mixed_three_versions_splits_corruption_and_bounds(self):
        old = replace(self.frame, protocol_version=2, qw=None, qx=None, qy=None,
                      qz=None, roll_deg=None, pitch_deg=None, yaw_deg=None)
        legacy = imu.Frame(1, 3, 4000, 1, 2, 3)
        frames = [legacy, self.frame, old]
        raw = b"".join(f.encode() for f in frames)
        for cut in range(len(raw)+1):
            p = imu.StreamParser()
            self.assertEqual(p.feed(raw[:cut])+p.feed(raw[cut:]), frames)
        p = imu.StreamParser()
        self.assertEqual([f for b in raw for f in p.feed(bytes([b]))], frames)
        bad = bytearray(self.frame.encode()); bad[48] ^= 1
        p = imu.StreamParser()
        self.assertEqual(p.feed(b"noise"+bad+self.frame.encode()[:19]+raw), frames)
        self.assertGreaterEqual(p.crc_errors, 2)
        p.feed(b"x"*100000+b"IMU")
        self.assertLess(len(p.buffer), p.BUFFER_LIMIT)

    def test_v3_csv_invalid_and_legacy_attitudes_blank(self):
        output = io.StringIO()
        frames = [self.frame, replace(self.frame, flags=1024),
                  replace(self.frame, flags=2048|4096), replace(self.frame, gyro_age_us=1001),
                  replace(self.frame, qw=float('nan')),
                  replace(self.frame, protocol_version=2), imu.Frame(1, 3, 4000, 1, 2, 3)]
        capture = imu.Capture(csv_file=output)
        capture.feed(b"".join(f.encode() for f in frames))
        rows = list(csv.DictReader(io.StringIO(output.getvalue())))
        self.assertEqual(rows[0]['attitude_valid'], '1')
        self.assertEqual(float(rows[0]['qw']), 1.)
        for row in rows[1:]:
            self.assertIn(row['attitude_valid'], ('0', ''))
            self.assertTrue(all(row[k] == '' for k in imu.ATTITUDE_FIELDS))


if __name__ == "__main__":
    unittest.main()
