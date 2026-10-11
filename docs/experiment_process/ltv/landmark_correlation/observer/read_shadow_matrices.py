#!/usr/bin/env python3
"""Independently decode native C++ shadow matrices and verify local identities.

This checks deterministic source sensitivity maps, NOT statistical calibration.
Binary encoding is native little-endian on the recorded x86_64 Linux environment.
"""
import argparse
import json
from pathlib import Path
import numpy as np


def decode(stream, description):
    rows, cols = description['rows'], description['cols']
    stream.seek(description['offset'])
    if description['encoding'] == 'native_float64_column_major':
        payload = stream.read(rows * cols * 8)
        if len(payload) != rows * cols * 8:
            raise ValueError('truncated dense matrix')
        return np.frombuffer(payload, dtype='<f8').reshape((rows, cols), order='F')
    count = description['nonzero']
    payload = stream.read(count * 16)
    if len(payload) != count * 16:
        raise ValueError('truncated sparse matrix')
    entries = np.frombuffer(payload, dtype=np.dtype([('row', '<u4'), ('col', '<u4'), ('value', '<f8')]))
    matrix = np.zeros((rows, cols))
    matrix[entries['row'], entries['col']] = entries['value']
    return matrix


def audit(path):
    checks, maximum, counts = 0, 0.0, {}
    saved_anchors, camera_points = {}, {}
    lag_checks, nonzero_cross_time = 0, 0
    def compare(lhs, rhs):
        nonlocal checks, maximum
        assert lhs.shape == rhs.shape
        error = float(np.linalg.norm(lhs - rhs) / max(1.0, np.linalg.norm(rhs)))
        maximum = max(maximum, error)
        checks += 1
        if error > 1e-11:
            raise ValueError(f'joint identity error {error}')
    with Path(str(path) + '.matrices.bin').open('rb') as binary:
        for line in Path(str(path) + '.matrices.jsonl').open():
            event = json.loads(line)
            assert event['valid_unconditional'] is False
            counts[event['event']] = counts.get(event['event'], 0) + 1
            matrices = {key: decode(binary, desc) for key, desc in event['matrices'].items()}
            assert all(np.isfinite(matrix).all() for matrix in matrices.values())
            if event['event'] == 'reset':
                saved_anchors.clear()
                camera_points.clear()
            if event['event'] == 'lifecycle':
                retained = set(event['retained_local_ids'])
                for key in list(saved_anchors):
                    if key[0] == event['epoch'] and key[1] not in retained:
                        del saved_anchors[key]
            if event['event'] == 'anchor_replace':
                replacement = matrices['conditional_anchor_source_map']
                times = {item['local_id']: item for item in event['anchor_times']}
                for slot, local_id in enumerate(event['retained_local_ids']):
                    key = (event['epoch'], local_id)
                    assert times[local_id]['time'] == event['time']
                    assert times[local_id]['lag_available']
                    row = replacement[3 * slot:3 * slot + 3]
                    compare(row, camera_points[(event['epoch'], event['time'], local_id)])
                    saved_anchors[key] = (event['time'], row.copy())
            if event['event'] == 'imu'  and matrices.get('state_before', np.zeros((0, 0))).size:
                x, dt = matrices['state_before'].ravel(), event['dt']
                predicted_b = np.zeros((len(x), 6))
                predicted_b[-6:-3, :3] = dt * np.eye(3)
                for offset in range(0, len(x), 3):
                    a, b, c = x[offset:offset + 3]
                    predicted_b[offset:offset + 3, 3:] = dt * np.array([[0, -c, b], [c, 0, -a], [-b, a, 0]])
                compare(matrices['B_corrected_acc_gyro'], predicted_b)
            if event['event'] == 'camera':
                b = matrices['B_same_bearing']
                compare(matrices['conditional_unit_source_gram'], b @ b.T)
                a, t = matrices['conditional_anchor_source_map'], matrices['conditional_current_point_source_map']
                compare(matrices['conditional_unit_Sigma_aa'], a @ a.T)
                compare(matrices['conditional_unit_Sigma_tt_crosspoint'], t @ t.T)
                compare(matrices['conditional_unit_Sigma_at'], a @ t.T)
                compare(t, matrices['persistent_input_offset'][:t.shape[0]])
                times = {item['local_id']: item for item in event['anchor_times']}
                lag_cross_norm = 0.0
                for slot, local_id in enumerate(event['retained_local_ids']):
                    key = (event['epoch'], local_id)
                    current_row = t[3 * slot:3 * slot + 3]
                    camera_points[(event['epoch'], event['time'], local_id)] = current_row.copy()
                    if times[local_id]['lag_available']:
                        assert key in saved_anchors
                        prior_time, prior_row = saved_anchors[key]
                        assert times[local_id]['time'] == prior_time
                        assert prior_time < event['time']
                        compare(a[3 * slot:3 * slot + 3], prior_row)
                        lag_checks += 1
                        lag_cross_norm += float(np.linalg.norm(matrices['conditional_unit_Sigma_at'][3 * slot:3 * slot + 3]))
                if lag_cross_norm > 0:
                    nonzero_cross_time += 1
    return {'scope': 'fixed_seed_and_gain_unit_source_sensitivity_only', 'valid_unconditional': False,
            'events': counts, 'matrix_identity_checks': checks, 'max_relative_identity_error': maximum,
            'prior_snapshot_identity_checks': lag_checks, 'nonzero_cross_time_camera_events': nonzero_cross_time,
            'binary_bytes': Path(str(path) + '.matrices.bin').stat().st_size}


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('csv_path', type=Path)
    args = parser.parse_args()
    print(json.dumps(audit(args.csv_path), indent=2))
