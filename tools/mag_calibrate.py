#!/usr/bin/env python3
"""Offline hard/soft-iron candidate fit; never writes to hardware or changes yaw.

Input is mag_capture.py CSV in BMM150 register axes, in uT. Absolute field scale
and mounting rotation are unobservable from norm-only fitting. A successful fit
is a candidate, not an independent validation or permission to enable fusion.
NumPy is an optional dependency for this tool only.
"""
from __future__ import annotations
import argparse
import csv
import hashlib
import itertools
import json
from pathlib import Path

try:
    import numpy as np
except ImportError as error:
    raise SystemExit('mag_calibrate requires NumPy: python -m pip install -r tools/requirements-mag-calibration.txt') from error


class CalibrationError(ValueError):
    pass


def apply(samples, calibration):
    values = np.asarray(samples, dtype=float)
    return (values - calibration['bias_ut']) @ np.asarray(calibration['correction_matrix']).T


def norm_stats(samples):
    norm = np.linalg.norm(samples, axis=1)
    return dict(min=float(norm.min()), max=float(norm.max()), mean=float(norm.mean()),
                std=float(norm.std()), cv=float(norm.std()/norm.mean()))


def calibrate(samples):
    """Fit one SPD ellipsoid, preserving volume with a determinant-one correction.

    Quality gates are initial engineering safeguards, not sensor specifications.
    No automatic outlier removal: magnetic disturbance must remain visible.
    """
    values = np.asarray(samples, dtype=float)
    if values.ndim != 2 or values.shape[1] != 3 or len(values) < 500:
        raise CalibrationError('need at least 500 three-component samples')
    if not np.isfinite(values).all():
        raise CalibrationError('nonfinite magnetic input')
    origin = values.mean(axis=0)
    scale = float(np.sqrt(np.mean((values-origin)**2)))
    if scale < 1e-6:
        raise CalibrationError('no measurable three-dimensional variation')
    x, y, z = ((values-origin)/scale).T
    design = np.column_stack((x*x, y*y, z*z, 2*x*y, 2*x*z, 2*y*z, x, y, z))
    coeff, _, rank, singular = np.linalg.lstsq(design, np.ones(len(values)), rcond=None)
    condition = float(singular[0]/singular[-1]) if singular[-1] > 0 else float('inf')
    if rank != 9 or condition > 1e4:
        raise CalibrationError('degenerate/ill-conditioned ellipsoid: improve 3D coverage')
    a, b, c, d, e, f = coeff[:6]
    quadratic = np.array([[a, d, e], [d, b, f], [e, f, c]])
    try:
        center = -.5*np.linalg.solve(quadratic, coeff[6:])
    except np.linalg.LinAlgError as error:
        raise CalibrationError('singular ellipsoid') from error
    radius_factor = float(1 + center @ quadratic @ center)
    if abs(radius_factor) < 1e-12:
        raise CalibrationError('degenerate ellipsoid radius')
    eigen, vectors = np.linalg.eigh(quadratic/radius_factor)
    if np.any(eigen <= 0) or not np.isfinite(eigen).all():
        raise CalibrationError('fit is not a positive-definite ellipsoid')
    # A unit-determinant matrix cannot claim the unknown true field strength.
    geometric_radius = float(scale/np.prod(eigen)**(1/6))
    correction = (vectors * np.sqrt(eigen)) @ vectors.T * geometric_radius/scale
    bias = origin + scale*center
    corrected = (values-bias) @ correction.T
    norms = np.linalg.norm(corrected, axis=1)
    if np.any(norms <= 1e-9) or not np.isfinite(corrected).all():
        raise CalibrationError('nonphysical corrected vectors')
    unit = corrected/norms[:, None]
    directional_eigen = np.linalg.eigvalsh(unit.T @ unit/len(unit))
    faces = np.concatenate((unit.max(axis=0), -unit.min(axis=0)))
    directions = np.array([p for p in itertools.product((-1, 0, 1), repeat=3) if any(p)], dtype=float)
    directions /= np.linalg.norm(directions, axis=1)[:, None]
    labels = np.argmax(unit @ directions.T, axis=1)
    bins = np.bincount(labels, minlength=26)
    # Require several observations, not a single sample in a direction.
    occupied = int(np.count_nonzero(bins >= 3))
    relative_residual = np.abs(norms/geometric_radius-1)
    p95 = float(np.quantile(relative_residual, .95))
    cv = float(norms.std()/norms.mean())
    distortion_condition = float(np.sqrt(eigen.max()/eigen.min()))
    reasons = []
    if min(faces) < .75 or directional_eigen[0] < .10 or occupied < 18:
        reasons.append('coverage insufficient: need both signs of all axes and more spatial directions')
    if cv > .05 or p95 > .10:
        reasons.append('norm residual too large: possible disturbance, noise or invalid ellipsoid model')
    if distortion_condition > 5:
        reasons.append('soft-iron condition too large: candidate may be nonphysical')
    return dict(schema='bmm150-offline-calibration-candidate-v1',
                input_axes='BMM150 chip/register axes; mounting rotation NOT applied',
                formula='corrected_uT = correction_matrix @ (measured_uT - bias_ut)',
                scale_semantics='determinant-one correction; absolute field strength not established',
                sample_count=len(values), bias_ut=bias.tolist(),
                correction_matrix=correction.tolist(), reference_norm_ut=geometric_radius,
                candidate_accepted=not reasons, rejection_reasons=reasons,
                independently_validated=False, mounting_rotation_verified=False,
                firmware_applied=False, magnetic_heading_fused=False,
                quality=dict(raw_norm=norm_stats(values), corrected_norm=norm_stats(corrected),
                             corrected_norm_cv=cv, relative_residual_p95=p95,
                             directional_second_moment_eigenvalues=directional_eigen.tolist(),
                             signed_face_support=faces.tolist(), occupied_direction_bins=occupied,
                             direction_bin_counts=bins.tolist(), design_condition=condition,
                             soft_iron_condition=distortion_condition))


def read_csv(path, max_samples=50000):
    samples, times = [], []
    rows = invalid = 0
    with Path(path).open(newline='', encoding='utf-8-sig') as handle:
        reader = csv.DictReader(handle)
        required = {'valid', 'x_ut', 'y_ut', 'z_ut', 'timestamp_us'}
        if not required.issubset(reader.fieldnames or []):
            raise CalibrationError('input must be mag_capture.py CSV with validity and timestamps')
        for row in reader:
            rows += 1
            if row['valid'] != '1':
                invalid += 1
                continue
            try:
                values = [float(row[key]) for key in ('x_ut', 'y_ut', 'z_ut')]
                timestamp = int(row['timestamp_us'])
            except (ValueError, TypeError) as error:
                raise CalibrationError(f'malformed valid sample at CSV row {rows+1}') from error
            if not np.isfinite(values).all():
                raise CalibrationError(f'nonfinite valid sample at CSV row {rows+1}')
            if not 0 <= timestamp < 2**64:
                raise CalibrationError('timestamp outside uint64 range')
            if times and timestamp <= times[-1]:
                raise CalibrationError('timestamps must strictly increase in one acquisition')
            samples.append(values)
            times.append(timestamp)
            if len(samples) > max_samples:
                raise CalibrationError(f'input exceeds bounded offline limit of {max_samples} samples')
    return np.asarray(samples), dict(rows=rows, invalid_rows=invalid,
                                    duration_s=(times[-1]-times[0])/1e6 if len(times)>1 else 0)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args(argv)
    if args.input.resolve() == args.output.resolve():
        parser.error('input and output must differ')
    try:
        samples, source = read_csv(args.input)
        result = calibrate(samples)
        digest = hashlib.sha256()
        with args.input.open('rb') as handle:
            for block in iter(lambda: handle.read(65536), b''):
                digest.update(block)
        result['source'] = dict(path=str(args.input.resolve()),
                              sha256=digest.hexdigest(), **source)
        args.output.write_text(json.dumps(result, indent=2, ensure_ascii=False, allow_nan=False)+'\n', encoding='utf-8')
        print(json.dumps(result, indent=2, ensure_ascii=False, allow_nan=False))
        return 0 if result['candidate_accepted'] else 2
    except (OSError, ValueError, np.linalg.LinAlgError) as error:
        parser.exit(1, f'mag_calibrate: {error}\n')


if __name__ == '__main__':
    raise SystemExit(main())
