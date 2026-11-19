"""Manual real-data acceptance: python tests/audit_export.py SOURCE EXPORT."""
import argparse
import bisect
import csv
import hashlib
import json
import math
from pathlib import Path
import cv2
import numpy as np


def rows(path):
    with path.open(newline='', encoding='utf8') as f:
        return list(csv.reader(f))[1:]


def audit(source, exported):
    assert not (exported/'EXPORT_INCOMPLETE').exists(), 'Incomplete export'
    report = json.loads((exported/'export.json').read_text(encoding='utf8'))
    with (source/'frames.csv').open(newline='', encoding='utf8') as f:
        frames = {r['frame_id']: r for r in csv.DictReader(f)}
    frames = list(frames.values())
    left = rows(exported/'mav0/cam0/data.csv')
    right = rows(exported/'mav0/cam1/data.csv')
    assert left == right, 'Stereo indexes differ'
    assert len(left) == len(frames) == report['stereo_pairs']
    stamps = [int(r[0]) for r in left]
    assert all(a <= b for a,b in zip(stamps,stamps[1:])), 'Image times regressed'
    checked = 0
    for entry, frame in zip(left, frames):
        stamp = frame['host_read_end_ns']
        name = f"{stamp}_{int(frame['frame_id']):09d}.png"
        assert entry == [stamp, name]
        for cam in ('cam0','cam1'):
            image = cv2.imdecode(np.fromfile(exported/'mav0'/cam/'data'/entry[1],dtype=np.uint8),cv2.IMREAD_UNCHANGED)
            gray = cv2.imdecode(np.fromfile(source/frame[cam],dtype=np.uint8),cv2.IMREAD_GRAYSCALE)
            assert image is not None and image.ndim == 2
            assert image.shape == (int(frame['height']),int(frame['width'])//2)
            assert np.array_equal(image,gray), 'Gray export differs from source'
            checked += 1
    with (source/'imu.csv').open(newline='', encoding='utf8') as f:
        original_imu = list(csv.DictReader(f))
    output_imu = rows(exported/'mav0/imu0/data.csv')
    assert len(output_imu) == len(original_imu) == report['imu_frames'], 'IMU resampled'
    with (source/'usb_batches.csv').open(newline='', encoding='utf8') as f:
        batches = {r['batch_id']: int(r['host_read_end_ns']) for r in csv.DictReader(f)}
    host_times = [batches[r['batch_id']] for r in original_imu]
    previous = None
    for r, original in zip(output_imu,original_imu):
        stamp = int(r[0])
        assert stamp == batches[original['batch_id']]
        assert int(original['timestamp_ns']) == int(original['timestamp_us'])*1000
        assert previous is None or stamp >= previous
        previous = stamp
        expected = [int(original['g'+a+'_raw'])*2000/32768*math.pi/180 for a in 'xyz']
        expected += [int(original['a'+a+'_raw'])*24/32768*9.80665 for a in 'xyz']
        assert all(math.isclose(float(x),y,rel_tol=1e-12,abs_tol=1e-12) for x,y in zip(r[1:],expected)), 'Units or axes differ'
    with (exported/'imu_image.csv').open(newline='', encoding='utf8') as f:
        joined = list(csv.DictReader(f))
    assert len(joined) == len(original_imu), 'Sparse association lost IMU rows'
    # Independent reference: inspect all still-available samples within the
    # tolerance using bisect bounds, not the exporter's streaming matcher.
    tolerance = report['match_tolerance_ns']
    used = set()
    expected_images = {}
    for frame in frames:
        t = int(frame['host_read_end_ns'])
        lo, hi = bisect.bisect_left(host_times, t-tolerance), bisect.bisect_right(host_times, t+tolerance)
        candidates = (i for i in range(lo, hi) if i not in used)
        sample = min(candidates, key=lambda i: (abs(host_times[i]-t), i), default=None)
        if sample is not None:
            used.add(sample)
            expected_images[sample] = frame
    image_fields = ('image_frame_id','image_timestamp_ns','image_minus_imu_ns','cam0','cam1')
    for i, (original, row) in enumerate(zip(original_imu, joined)):
        assert all(row[k] == v for k, v in original.items()), 'Original IMU field overwritten'
        assert int(row['unified_timestamp_ns']) == host_times[i]
        frame = expected_images.get(i)
        if frame is None:
            assert all(row[k] == '' for k in image_fields), 'Unmatched image must be blank'
        else:
            assert row['image_frame_id'] == frame['frame_id']
            assert row['image_timestamp_ns'] == frame['host_read_end_ns']
            assert int(row['image_minus_imu_ns']) == int(frame['host_read_end_ns'])-host_times[i]
            for cam in ('cam0','cam1'):
                expected_path = f"mav0/{cam}/data/{frame['host_read_end_ns']}_{int(frame['frame_id']):09d}.png"
                assert row[cam] == expected_path and (exported/expected_path).exists()
    assert len(used) == report['matched_images']
    assert len(joined)-len(used) == report['unmatched_imu_rows']
    assert len(frames)-len(used) == report['unmatched_images']
    assert report['timestamp_basis'] == 'host_receive'
    for name, expected in report['source_sha256'].items():
        h = hashlib.sha256()
        with (source/name).open('rb') as f:
            for chunk in iter(lambda:f.read(1024*1024),b''):
                h.update(chunk)
        assert h.hexdigest() == expected, 'Source hash changed: '+name
    assert not list(exported.rglob('sensor.yaml')), 'Unexpected calibration'
    assert report['calibration_provided'] is False
    assert report['stereo_exposure_verified'] is False
    assert '曝光同步误差未验证' in report['warning']
    assert (exported/'provenance/imu.csv').read_bytes() == (source/'imu.csv').read_bytes()
    return dict(status='passed',stereo_pairs=len(left),gray_pngs_compared=checked,
                imu_rows_all_checked=len(output_imu),sparse_rows_checked=len(joined),matched_images=len(used),source_hashes_checked=len(report['source_sha256']),
                units='rad/s, m/s^2; original x/y/z',warning=report['warning'])


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('source',type=Path)
    parser.add_argument('exported',type=Path)
    args = parser.parse_args()
    print(json.dumps(audit(args.source,args.exported),ensure_ascii=False,indent=2))
