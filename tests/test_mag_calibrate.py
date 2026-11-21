"""Offline magnetic calibration: synthetic geometry and rejection regressions."""
import contextlib
import csv
import hashlib
import io
import json
from pathlib import Path
import tempfile
import unittest

try:
    import numpy as np
except ImportError:
    np = None


class RetainedCandidateTests(unittest.TestCase):
    """The retained result is data only, never an enabled firmware default."""

    def load_candidate(self):
        path = Path(__file__).resolve().parents[1] / 'config' / 'bmm150-calibration-candidate.json'
        return json.loads(path.read_text(encoding='utf-8'))

    def test_candidate_lifecycle_and_coordinate_contract(self):
        result = self.load_candidate()
        self.assertEqual(result['schema'], 'bmm150-integration-candidate-v1')
        self.assertEqual(result['status'], 'candidate_only')
        self.assertIsNone(result['mounting_rotation_to_frd'])
        for flag in ('mounting_rotation_verified', 'firmware_applied', 'magnetic_heading_fused'):
            self.assertIs(result[flag], False)
        self.assertIn('chip/register axes', result['input_axes'])
        self.assertIn('mounting rotation NOT applied', result['input_axes'])
        self.assertIn('Startup-zeroed relative yaw', result['heading_semantics'])
        self.assertEqual(result['integration_order'], [
            'verify_magnetic_to_imu_axes',
            'add_abnormal_field_and_freshness_gates',
            'validate_tilt_compensation_and_relative_yaw_drift_suppression',
        ])
        self.assertEqual(result['source_candidate_sha256'],
                         '2ef9ac08ca40a6a7d9563343cfda3850f9bee04494a576cdbe1b85c17107061e')
        self.assertEqual(result['bias_ut'],
                         [-20.51479880648215, -14.69928811135179, -4.728956550836864])
        self.assertEqual(result['reference_norm_ut'], 47.889600586206996)

    @unittest.skipIf(np is None, "optional NumPy required for offline magnetic calibration")
    def test_retained_matrix_is_spd_and_uses_chip_axis_formula(self):
        from tools.mag_calibrate import apply
        result = self.load_candidate()
        matrix = np.asarray(result['correction_matrix'])
        self.assertEqual(matrix.shape, (3, 3))
        self.assertTrue(np.isfinite(matrix).all())
        np.testing.assert_allclose(matrix, [
            [1.0079481866451447, -.005386536906908035, .002291617838467617],
            [-.005386536906908059, .9825200371420542, -.010625394151918972],
            [.0022916178384676197, -.010625394151918925, 1.0099145875143967],
        ], rtol=0, atol=1e-15)
        np.testing.assert_allclose(matrix, matrix.T, rtol=0, atol=1e-15)
        self.assertTrue(np.all(np.linalg.eigvalsh(matrix) > 0))
        self.assertAlmostEqual(np.linalg.det(matrix), 1.0, places=12)
        measured = [25., -8., 30.]
        expected = [sum(matrix[i, j] * (measured[j] - result['bias_ut'][j])
                        for j in range(3)) for i in range(3)]
        np.testing.assert_allclose(apply([measured], result)[0], expected, rtol=0, atol=1e-14)
        np.testing.assert_allclose(apply([result['bias_ut']], result), [[0., 0., 0.]], atol=0)


@unittest.skipIf(np is None, "optional NumPy required for offline magnetic calibration")
class MagCalibrationTests(unittest.TestCase):
    def cloud(self, count=2500):
        rng = np.random.default_rng(42)
        unit = rng.normal(size=(count, 3))
        unit /= np.linalg.norm(unit, axis=1)[:, None]
        return unit * 47.0

    def test_recovers_offset_and_symmetric_distortion(self):
        from tools.mag_calibrate import calibrate, apply
        truth = self.cloud()
        distortion = np.array([[1.2, .08, -.03], [.08, .85, .04], [-.03, .04, 1.05]])
        bias = np.array([8., -12., 4.])
        observed = truth @ distortion.T + bias
        result = calibrate(observed)
        self.assertTrue(result['candidate_accepted'], result['rejection_reasons'])
        np.testing.assert_allclose(result['bias_ut'], bias, atol=1e-8)
        corrected = apply(observed, result)
        self.assertLess(np.std(np.linalg.norm(corrected, axis=1)), 1e-8)
        self.assertAlmostEqual(np.linalg.det(result['correction_matrix']), 1., places=8)
        # Norm fit cannot recover absolute field magnitude or an axis rotation.
        self.assertFalse(result['independently_validated'])
        self.assertFalse(result['mounting_rotation_verified'])

    def test_planar_data_rejected(self):
        from tools.mag_calibrate import calibrate, CalibrationError
        angle = np.linspace(0, 2*np.pi, 1000)
        cloud = np.column_stack((40*np.cos(angle), 40*np.sin(angle), np.ones(1000)*20))
        with self.assertRaises(CalibrationError):
            calibrate(cloud)

    def test_partial_sphere_is_not_accepted(self):
        from tools.mag_calibrate import calibrate
        data = self.cloud(12000)
        data = data[data[:, 2] > 10]
        result = calibrate(data)
        self.assertFalse(result['candidate_accepted'])
        self.assertTrue(any('coverage' in reason for reason in result['rejection_reasons']))

    def test_nonfinite_and_insufficient_data_rejected(self):
        from tools.mag_calibrate import calibrate, CalibrationError
        for values in (np.zeros((40, 3)), np.full((600, 3), np.nan), np.zeros((600, 2))):
            with self.subTest(shape=values.shape):
                with self.assertRaises(CalibrationError):
                    calibrate(values)

    def test_noise_has_small_residual(self):
        from tools.mag_calibrate import calibrate
        rng = np.random.default_rng(7)
        data = self.cloud() + rng.normal(scale=.5, size=(2500, 3)) + [5, -3, 9]
        result = calibrate(data)
        self.assertTrue(result['candidate_accepted'], result['rejection_reasons'])
        self.assertLess(result['quality']['corrected_norm_cv'], .02)

    def test_variable_field_fails_residual_gate(self):
        from tools.mag_calibrate import calibrate
        rng = np.random.default_rng(18)
        data = self.cloud() * rng.uniform(.7, 1.3, size=(2500, 1))
        result = calibrate(data)
        self.assertFalse(result['candidate_accepted'])
        self.assertTrue(any('residual' in reason for reason in result['rejection_reasons']))


    def write_csv(self, path, data, times=None, invalid_first=False):
        with path.open('w', newline='', encoding='utf-8') as handle:
            writer = csv.writer(handle)
            writer.writerow(['valid', 'x_ut', 'y_ut', 'z_ut', 'timestamp_us'])
            if invalid_first:
                writer.writerow([0, '', '', '', ''])
            for index, vector in enumerate(data):
                writer.writerow([1, *vector, index*40000 if times is None else times[index]])

    def test_csv_filters_invalid_and_bounds_memory(self):
        from tools.mag_calibrate import read_csv, CalibrationError
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'mag.csv'
            self.write_csv(path, self.cloud(600), invalid_first=True)
            data, source = read_csv(path)
            self.assertEqual(data.shape, (600, 3))
            self.assertEqual(source['invalid_rows'], 1)
            self.assertEqual(source['rows'], 601)
            self.assertAlmostEqual(source['duration_s'], 599*.04)
            with self.assertRaises(CalibrationError):
                read_csv(path, max_samples=599)

    def test_csv_rejects_bad_headers_timestamps_and_values(self):
        from tools.mag_calibrate import read_csv, CalibrationError
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'mag.csv'
            for times in ([0, 0], [40000, 0], [-1, 0], [0, 2**64]):
                self.write_csv(path, self.cloud(2), times=times)
                with self.subTest(times=times), self.assertRaises(CalibrationError):
                    read_csv(path)
            for value in ('nan', 'inf', 'broken', ''):
                path.write_text('valid,x_ut,y_ut,z_ut,timestamp_us\n1,'+value+',0,0,0\n', encoding='utf-8')
                with self.subTest(value=value), self.assertRaises(CalibrationError):
                    read_csv(path)
            path.write_text('x_ut,y_ut,z_ut\n1,2,3\n', encoding='utf-8')
            with self.assertRaises(CalibrationError):
                read_csv(path)

    def test_cli_writes_candidate_not_hardware_approval(self):
        from tools.mag_calibrate import main
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'mag.csv'
            output = Path(directory)/'candidate.json'
            self.write_csv(path, self.cloud(600)+[5, -2, 3])
            with contextlib.redirect_stdout(io.StringIO()):
                code = main(['--input', str(path), '--output', str(output)])
            self.assertEqual(code, 0)
            result = json.loads(output.read_text(encoding='utf-8'))
            self.assertEqual(result['source']['sha256'], hashlib.sha256(path.read_bytes()).hexdigest())
            for flag in ('independently_validated', 'mounting_rotation_verified',
                         'firmware_applied', 'magnetic_heading_fused'):
                self.assertFalse(result[flag])
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as error:
                main(['--input', str(path), '--output', str(path)])
            self.assertEqual(error.exception.code, 2)

    def test_cli_rejected_candidate_is_preserved_with_exit_two(self):
        from tools.mag_calibrate import main
        data = self.cloud(12000)
        data = data[data[:, 2] > 10]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'mag.csv'
            output = Path(directory)/'candidate.json'
            self.write_csv(path, data)
            with contextlib.redirect_stdout(io.StringIO()):
                code = main(['--input', str(path), '--output', str(output)])
            self.assertEqual(code, 2)
            result = json.loads(output.read_text(encoding='utf-8'))
            self.assertFalse(result['candidate_accepted'])
            self.assertTrue(result['rejection_reasons'])


if __name__ == '__main__':
    unittest.main()
