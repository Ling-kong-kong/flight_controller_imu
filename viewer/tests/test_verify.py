"""Integrity and rejection paths; no real devices are used."""
import json
from unittest.mock import patch
import test_storage
from imu_viewer.storage import finalize, export_euroc, read_csv
from imu_viewer.verify import verify_session
from imu_viewer.capture import select_port

# Inherit the existing end-to-end source factory without duplicating its tests.
import unittest
class VerifyTests(unittest.TestCase):
    setUp = test_storage.StorageTests.setUp
    tearDown = test_storage.StorageTests.tearDown
    session = test_storage.StorageTests.session

    def finalized(self):
        source = self.session()
        finalize(source, dict(error=None, timeline_anomaly=False))
        return source

    def test_full_decode_recompute_and_hash(self):
        source = self.finalized()
        result = verify_session(source, decode_all=True)
        self.assertEqual(result['imu_frames'], 4000)
        self.assertEqual(result['png_files_decoded'], 200)
        self.assertEqual(result['aligned_pairs'], 100)
        self.assertEqual(len(result['sha256']['usb.bin']), 64)
        self.assertTrue((source/'verification.json').exists())

    def test_corrupt_imu_rejected_before_export(self):
        source = self.finalized()
        file = source/'imu.csv'
        file.write_text(file.read_text(encoding='utf8').replace(',0,0,1000000,', ',0,9,1000000,', 1), encoding='utf8')
        # Make a deterministic raw/CSV mismatch regardless of CSV field ordering.
        with (source/'usb.bin').open('r+b') as f:
            f.seek(24)
            f.write(b'\xff')
        target = self.root/'bad_export'
        with self.assertRaises(ValueError):
            export_euroc(source, target)
        self.assertFalse(target.exists())

    def test_alignment_tampering_rejected(self):
        source = self.finalized()
        file = source/'aligned_frames.csv'
        rows = list(read_csv(file))
        original = rows[0]['unified_timestamp_ns']
        file.write_text(file.read_text(encoding='utf8').replace(original, str(int(original)+1), 1), encoding='utf8')
        with self.assertRaises(ValueError):
            verify_session(source)

    def test_missing_stereo_file_rejected(self):
        source = self.finalized()
        (source/'cam1/000000000.png').unlink()
        with self.assertRaises(FileNotFoundError):
            verify_session(source)

    def test_export_failure_keeps_incomplete_marker(self):
        source = self.finalized()
        target = self.root/'incomplete'
        with patch('imu_viewer.storage.write_png', side_effect=OSError('injected ENOSPC')):
            with self.assertRaises(OSError):
                export_euroc(source, target)
        self.assertTrue((target/'EXPORT_INCOMPLETE').exists())

    def test_active_finalize_refused(self):
        source = self.session()
        with self.assertRaises(ValueError):
            finalize(source)

    def test_usb_identity_and_serial_selection(self):
        a = dict(port='COM12', serial_number='a', h743=True)
        b = dict(port='COM3', serial_number='b', h743=False)
        with patch('imu_viewer.capture.discover_ports', return_value=[a,b]):
            self.assertEqual(select_port()['port'], 'COM12')
            self.assertEqual(select_port('com12','a'), a)
            with self.assertRaises(ValueError):
                select_port('COM3')
            with self.assertRaises(ValueError):
                select_port('COM12','wrong')
        with patch('imu_viewer.capture.discover_ports', return_value=[a,dict(a,port='COM13')]):
            with self.assertRaises(ValueError):
                select_port()

if __name__ == '__main__':
    unittest.main()
