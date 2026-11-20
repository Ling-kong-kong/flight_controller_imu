"""Host-read timestamps and lossless sparse image association; synthetic only."""
import csv
import json
import random
import unittest
import numpy as np
import test_storage
from audit_export import audit
from imu_viewer.capture import Config, Recorder
from imu_viewer.protocol import Frame, Parser
from imu_viewer.storage import SessionWriter, finalize, export_euroc, read_csv, json_write
from imu_viewer.timing import iter_host_imu, match_images
from imu_viewer.verify import verify_session


def imu_rows(times):
    return [dict(sample_id=str(i), unified_timestamp_ns=str(t)) for i,t in enumerate(times)]


def images(times):
    return [dict(frame_id=str(i), host_read_end_ns=str(t)) for i,t in enumerate(times)]


class MatchingTests(unittest.TestCase):
    def check_reference(self, times, camera, tolerance):
        expected, used = {}, set()
        frames = images(camera)
        for frame, t in zip(frames, camera):
            choices = [i for i,s in enumerate(times) if i not in used and abs(t-s) <= tolerance]
            if choices:
                i = min(choices, key=lambda i:(abs(times[i]-t),i))
                used.add(i)
                expected[i] = frame
        self.assertEqual(match_images(iter(imu_rows(times)),frames,tolerance),expected)

    def test_ties_duplicates_and_exhausted_closer_group(self):
        for times,camera in [([0,9],[10,11]),([0,9,11],[10,10,10]),
                             ([0,0,11],[10,10,10]),([0,0,10,10],[5,5,5,5]),
                             ([10,10,10],[9,10,11,12])]:
            self.check_reference(times,camera,20)

    def test_tolerance_boundary_empty_and_unmatched(self):
        for times,camera in [([],[]),([],[1,2]),([1,2],[]),([0,100],[20,79,120]),
                             ([1,1,1],[0,1,1,1,1,2])]:
            for tolerance in (0,20,100):
                self.check_reference(times,camera,tolerance)
        for value in (-1,1.5,None):
            with self.assertRaises(ValueError):
                match_images([],[],value)
        with self.assertRaises(ValueError):
            match_images([],images([2,1]),5)

    def test_randomized_matches_independent_exhaustive_reference(self):
        rng = random.Random(20261119)
        for _ in range(600):
            self.check_reference(sorted(rng.randrange(100) for _ in range(rng.randrange(60))),
                                 sorted(rng.randrange(120) for _ in range(rng.randrange(40))),
                                 rng.randrange(50))


class HostTimeTests(unittest.TestCase):
    setUp = test_storage.StorageTests.setUp
    tearDown = test_storage.StorageTests.tearDown
    session = test_storage.StorageTests.session

    def small_session(self):
        path = self.root/'host'
        writer = SessionWriter(path,Config(port='FAKE',width=8,height=4))
        parser = Parser()
        raw = b''.join(Frame(i,1000000+i*1000,1,2,3,4,5,6,20,10,16 if i==0 else 0).encode() for i in range(6))
        # Split first frame across batches; all six frames complete in batch 1.
        writer.usb(0,raw[:20],999_999_990,1_000_000_000,parser.feed(raw[:20]))
        writer.usb(1,raw[20:],1_000_000_001,1_010_000_000,parser.feed(raw[20:]))
        pixels = np.zeros((4,8,3),np.uint8)
        # Duplicate host times, broken/missing/reversed media times are raw metadata.
        for i,(t,pts) in enumerate([(1_010_000_000,None),(1_010_000_000,8),
                                    (1_010_000_000,8),(1_010_000_000,2),
                                    (1_100_000_000,None)]):
            writer.image(i, '' if pts is None else pts/1e6,pts,t,t,pixels)
        writer.close()
        return path

    def test_split_batch_time_duplicates_and_all_rows_sparse_export(self):
        source = self.small_session()
        result = finalize(source,dict(error=None,timeline_anomaly=False))
        self.assertTrue(result['timestamps_valid'])
        self.assertFalse(result['clock_fitting_used'])
        self.assertEqual(result['excluded_frames'],0)
        original = list(iter_host_imu(source))
        self.assertEqual([int(r['unified_timestamp_ns']) for r in original],[1_010_000_000]*6)
        self.assertEqual([int(r['batch_id']) for r in original],[1]*6)
        self.assertEqual([int(r['timestamp_ns']) for r in original],[1_000_000_000+i*1_000_000 for i in range(6)])
        output = self.root/'export'
        exported = export_euroc(source,output)
        self.assertEqual(exported['imu_frames'],6)
        self.assertEqual(exported['stereo_pairs'],5)
        self.assertEqual(exported['matched_images'],4)
        self.assertEqual(exported['unmatched_imu_rows'],2)
        joined = list(read_csv(output/'imu_image.csv'))
        self.assertEqual([r['image_frame_id'] for r in joined],['0','1','2','3','',''])
        for row in joined[4:]:
            self.assertTrue(all(row[k] == '' for k in ('image_frame_id','image_timestamp_ns','image_minus_imu_ns','cam0','cam1')))
        index = list(read_csv(output/'mav0/cam0/data.csv'))
        self.assertEqual(len({r['filename'] for r in index}),5)
        self.assertEqual(len(list(read_csv(output/'mav0/imu0/data.csv'))),6)
        report = json.loads((output/'export.json').read_text(encoding='utf8'))
        self.assertEqual(report['duplicate_imu_timestamps'],5)
        self.assertEqual(report['duplicate_camera_timestamps'],3)
        self.assertFalse(report['strict_euroc_timestamp_compatibility'])
        self.assertEqual(audit(source,output)['status'],'passed')

    def test_no_images_preserves_all_imu_rows(self):
        source = self.small_session()
        rows = list(read_csv(source/'frames.csv'))
        with (source/'frames.csv').open('w',newline='',encoding='utf8') as f:
            csv.DictWriter(f,list(rows[0])).writeheader()
        finalize(source,dict(error=None))
        output = self.root/'no_images'
        result = export_euroc(source,output)
        self.assertEqual(result['unmatched_imu_rows'],6)
        self.assertEqual(audit(source,output)['sparse_rows_checked'],6)

    def test_no_imu_retains_all_images(self):
        source = self.small_session()
        for name in ('imu.csv','usb_batches.csv'):
            header = (source/name).read_text(encoding='utf8').splitlines()[0]
            (source/name).write_text(header+'\n',encoding='utf8')
        (source/'usb.bin').write_bytes(b'')
        finalize(source,dict(error=None))
        output = self.root/'no_imu'
        self.assertEqual(export_euroc(source,output)['stereo_pairs'],5)
        self.assertEqual(audit(source,output)['imu_rows_all_checked'],0)
        with (output/'imu_image.csv').open(newline='', encoding='utf8') as handle:
            header = next(csv.reader(handle))
        self.assertEqual(header[:11], ['ax_raw', 'ay_raw', 'az_raw', 'gx_raw', 'gy_raw', 'gz_raw',
                                      'image_frame_id', 'image_timestamp_ns', 'image_minus_imu_ns', 'cam0', 'cam1'])
        self.assertEqual(header[11:19], ['qw', 'qx', 'qy', 'qz', 'roll_deg', 'pitch_deg', 'yaw_deg', 'attitude_valid'])
        report = json.loads((output/'export.json').read_text(encoding='utf8'))
        self.assertEqual(report['imu_image_columns'], header)

    def test_tampered_unified_and_device_timestamp_rejected(self):
        source = self.small_session()
        finalize(source,dict(error=None))
        file = source/'imu.csv'
        original = file.read_bytes()
        for key in ('unified_timestamp_ns','timestamp_ns'):
            rows = list(read_csv(file)); rows[0][key] = str(int(rows[0][key])+1)
            with file.open('w',newline='',encoding='utf8') as f:
                out = csv.DictWriter(f,list(rows[0]));out.writeheader();out.writerows(rows)
            with self.assertRaises(ValueError):
                verify_session(source)
            with self.assertRaises(ValueError):
                export_euroc(source,self.root/'bad')
            self.assertFalse((self.root/'bad').exists())
            file.write_bytes(original)

    def test_invalid_host_interval_stops_capture(self):
        r = Recorder(Config(port='FAKE',output=str(self.root),min_free_bytes=0))
        self.assertFalse(r._receive_usb(0,b'x',10,9))
        self.assertIn('host read interval',r.error)
        self.assertTrue(r.timeline_anomaly)

    def test_legacy_valid_and_invalid_models_export_raw_host_axis(self):
        source = self.session()
        # Reproduce a schema-1 session rather than treating schema-2 as legacy.
        for name in ('imu.csv','frames.csv'):
            rows = list(read_csv(source/name))
            fields = [k for k in rows[0] if k != 'unified_timestamp_ns']
            with (source/name).open('w',newline='',encoding='utf8') as f:
                out = csv.DictWriter(f,fields,extrasaction='ignore');out.writeheader();out.writerows(rows)
        session = json.loads((source/'session.json').read_text(encoding='utf8'))
        session['schema_version'] = 1
        session.pop('timestamp_basis');session.pop('unified_timestamp_source')
        json_write(source/'session.json',session)
        result = finalize(source,dict(error=None,timeline_anomaly=False))
        self.assertTrue(result['models_valid'])
        self.assertEqual(result['aligned_frames'],100)
        output = self.root/'legacy_host'
        self.assertEqual(export_euroc(source,output)['stereo_pairs'],100)
        self.assertEqual(audit(source,output)['imu_rows_all_checked'],4000)
        finalize(source,dict(error='old media failure',timeline_anomaly=True))
        self.assertFalse(json.loads((source/'clock_models.json').read_text(encoding='utf8'))['models_valid'])
        output = self.root/'legacy_invalid_host'
        self.assertEqual(export_euroc(source,output)['stereo_pairs'],100)
        self.assertEqual(audit(source,output)['status'],'passed')

    def test_export_rejects_active_or_invalid_tolerance(self):
        source = self.small_session()
        with self.assertRaises(ValueError):
            export_euroc(source,self.root/'active')
        finalize(source,dict(error=None))
        for value in (-1,3.2,None):
            with self.assertRaises(ValueError):
                export_euroc(source,self.root/'bad',value)


if __name__ == '__main__':
    unittest.main()
