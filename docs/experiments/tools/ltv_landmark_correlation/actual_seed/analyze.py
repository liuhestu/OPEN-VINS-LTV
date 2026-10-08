#!/usr/bin/env python3
"""Audit actual C++ seed perturbations; no claim of real landmark calibration."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'reference'))
from run import covariance_intervals


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def analyze(samples, output):
    root = Path(__file__).resolve().parents[5]
    contract_path = root / 'docs/ltv/landmark_correlation/actual_seed/experiment_contract.json'
    contract = json.loads(contract_path.read_text())
    data = np.genfromtxt(samples, delimiter=',', names=True)
    predicted_path = Path(str(samples) + '.predicted.csv')
    q = np.loadtxt(predicted_path, delimiter=',')
    alpha = .01 / (len(contract['amplitudes']) * 9 ** 2)
    result = {'contract_sha256': sha(contract_path), 'samples_path': str(samples),
              'samples_sha256': sha(samples), 'prediction_sha256': sha(predicted_path),
              'real_history_covariance': 'BLOCKED_UNKNOWN_POSE_BEARING_POSTERIOR_CROSS',
              'real_landmark_statistical_calibration': 'NOT_EVALUATED', 'amplitudes': []}
    for amplitude in contract['amplitudes']:
        chosen = data[np.isclose(data['amplitude'], amplitude, rtol=0, atol=1e-15)]
        assert len(chosen) == contract['independent_draws_per_amplitude']
        assert np.array_equal(chosen['draw'], np.arange(len(chosen)))
        linear = np.column_stack([chosen[f'linear_{i}'] for i in range(9)]) / amplitude
        actual = np.column_stack([chosen[f'actual_{i}'] for i in range(9)]) / amplitude
        intervals = covariance_intervals(linear, q, alpha)
        empirical = np.cov(actual, rowvar=False)
        delta = actual - linear
        result['amplitudes'].append({
            'amplitude': amplitude, 'independent_draws': len(chosen),
            'linear_reference_status': 'PASS' if all(row['passes'] for row in intervals) else 'FAIL',
            'linear_simultaneous_intervals': intervals,
            'actual_sample_joint_covariance_normalized': empirical.tolist(),
            'actual_relative_covariance_deviation': float(np.linalg.norm(empirical - q) / np.linalg.norm(q)),
            'actual_cross_relative_deviation': float(np.linalg.norm(empirical[:6, 6:] - q[:6, 6:]) / np.linalg.norm(q[:6, 6:])),
            'actual_mean_error_normalized': np.mean(actual, axis=0).tolist(),
            'actual_minus_linear_rms_normalized': np.sqrt(np.mean(delta ** 2, axis=0)).tolist(),
            'actual_non_gaussian_confidence': 'NOT_CLAIMED; descriptive linearization diagnostic only'})
    result['predicted_joint_covariance_per_unit_amplitude'] = q.tolist()
    output.write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps([{key: row[key] for key in ('amplitude', 'linear_reference_status', 'actual_relative_covariance_deviation',
                                                'actual_cross_relative_deviation')} for row in result['amplitudes']], indent=2))
    return 0 if all(row['linear_reference_status'] == 'PASS' for row in result['amplitudes']) else 1


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('samples', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    raise SystemExit(analyze(args.samples, args.output))
