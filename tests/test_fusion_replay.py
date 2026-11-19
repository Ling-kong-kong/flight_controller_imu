"""Offline replay exercises the same C fusion, without serial or ST-Link."""
import csv
from dataclasses import replace
import io
from pathlib import Path
import tempfile
import unittest

from tools import imu_fusion_replay as replay
from tools.imu_capture import ATTITUDE_FIELDS, Frame, PairedFrame


class FusionReplayTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.native = replay.NativeFusion(replay.ROOT/'firmware/src/fusion.c',
                                         replay.ROOT/'firmware/include')

    @classmethod
    def tearDownClass(cls):
        cls.native.close()

    def setUp(self):
        self.native.lib.replay_reset()
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.source = Path(self.temp.name)/'usb.bin'

    def frame(self, index, **overrides):
        # Raw chip z is +1g under the provisional PITCH_180 installation.
        f = PairedFrame(index, 100000+index*1000, 0, 0, 1365,
                        16 if index % 2 else -16, -5, 2, 0, 0,
                        protocol_version=3)
        return replace(f, **overrides)

    def write_frames(self, frames):
        data = b''.join(f.encode() for f in frames)
        self.source.write_bytes(data)
        return data

    def test_each_replay_starts_fresh_and_leaves_input_unchanged(self):
        original = self.write_frames(self.frame(i) for i in range(2300))
        first = replay.replay(self.source, self.native)
        second = replay.replay(self.source, self.native)
        self.assertEqual(first['first_valid_elapsed_s'], 2.199)
        self.assertEqual(second['first_valid_elapsed_s'], 2.199)
        self.assertEqual(first['valid_frames'], second['valid_frames'])
        self.assertEqual(second['fusion_resets_including_initial'], 1)
        self.assertEqual(self.source.read_bytes(), original)

    def test_mixed_versions_bad_crc_and_arbitrary_chunking_match(self):
        frames = [self.frame(i, protocol_version=2 if i % 2 else 3,
                             flags=2048, qw=1, qx=0, qy=0, qz=0,
                             roll_deg=0, pitch_deg=0, yaw_deg=0)
                  for i in range(2300)]
        corrupt = bytearray(self.frame(77).encode())
        corrupt[25] ^= 0x40
        # Corrupt extra bytes are not accepted as a sample.
        self.source.write_bytes(b'noise' + b''.join(f.encode() for f in frames[:80])
                                + corrupt + b''.join(f.encode() for f in frames[80:]) + b'I')
        results = [replay.replay(self.source, self.native, chunk_size=size)
                   for size in (1, 17, 65536)]
        self.assertEqual(results[0], results[1])
        self.assertEqual(results[0], results[2])
        result = results[0]
        self.assertEqual(result['paired_frames'], 2300)
        self.assertEqual(result['recorded_valid_attitude_frames'], 1150)
        self.assertEqual(result['first_valid_elapsed_s'], 2.199)
        self.assertEqual(result['valid_frames'], 101)
        self.assertEqual(result['input_sequence_missing'], 0)
        self.assertEqual(result['parser']['crc_errors'], 1)
        self.assertEqual(result['parser']['trailing_bytes'], 1)

    def test_cli_csv_invalid_attitude_blank_and_report_provenance(self):
        import contextlib
        import json
        self.write_frames(self.frame(i) for i in range(2300))
        output = Path(self.temp.name)/'replay.csv'
        report = Path(self.temp.name)/'report.json'
        with contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(replay.main([str(self.source), '--report', str(report),
                                          '--csv', str(output)]), 0)
        rows = list(csv.DictReader(io.StringIO(output.read_text(encoding='utf-8'))))
        self.assertEqual(len(rows), 2300)
        self.assertTrue(all(row[k] == '' for row in rows[:2199] for k in ATTITUDE_FIELDS))
        self.assertEqual(rows[2199]['attitude_valid'], '1')
        self.assertAlmostEqual(sum(float(rows[2199][k])**2 for k in ATTITUDE_FIELDS[:4]), 1, places=6)
        self.assertEqual(rows[2199]['timestamp_us'], str(100000+2199*1000))
        summary = json.loads(report.read_text(encoding='utf-8'))
        self.assertEqual(summary['input_sha256'], replay.sha256(self.source))
        self.assertEqual(summary['native_build']['fusion_source_sha256'],
                         replay.sha256(replay.ROOT/'firmware/src/fusion.c'))
        self.assertEqual(summary['native_build']['replay_script_sha256'],
                         replay.sha256(Path(replay.__file__)))

    def test_timestamp_gap_resets_but_does_not_fill_missing_samples(self):
        frames = [self.frame(i) for i in range(2300)]
        frames += [self.frame(i) for i in range(2311, 4611)]
        self.write_frames(frames)
        result = replay.replay(self.source, self.native)
        self.assertEqual(result['paired_frames'], 4600)
        self.assertEqual(result['input_sequence_missing'], 11)
        self.assertEqual(result['fusion_resets_including_initial'], 2)
        self.assertEqual(result['valid_frames'], 202)
        self.assertEqual([x['frame_index'] for x in result['validity_transitions']],
                         [0, 2199, 2300, 4499])

    def test_usb_new_session_does_not_reset_attitude(self):
        frames = [self.frame(i) for i in range(2300)]
        frames += [self.frame(i, sequence=i-2300, flags=16 if i == 2300 else 0)
                   for i in range(2300, 2400)]
        self.write_frames(frames)
        result = replay.replay(self.source, self.native)
        self.assertEqual(result['fusion_resets_including_initial'], 1)
        self.assertEqual(result['valid_frames'], 201)
        self.assertEqual(result['input_timing']['sessions_observed'], 2)
        self.assertEqual(result['input_sequence_regressions'], 0)
        self.assertEqual(result['validity_transition_count'], 2)

    def test_unpaired_legacy_input_fails_without_fabricating_pairs(self):
        self.source.write_bytes(Frame(1, 1, 100000, 0, 0, 1365).encode())
        with self.assertRaisesRegex(ValueError, 'No paired IMU2'):
            replay.replay(self.source, self.native)

    def test_cli_cannot_overwrite_input_or_implementation(self):
        import contextlib
        original = self.write_frames([self.frame(0)])
        for output in (self.source, replay.ROOT/'tools/native/fusion_replay.c',
                       replay.ROOT/'tools/imu_capture.py'):
            with self.subTest(output=output), contextlib.redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit) as error:
                    replay.main([str(self.source), '--report', str(output)])
                self.assertEqual(error.exception.code, 2)
        self.assertEqual(self.source.read_bytes(), original)


if __name__ == '__main__':
    unittest.main()

\n