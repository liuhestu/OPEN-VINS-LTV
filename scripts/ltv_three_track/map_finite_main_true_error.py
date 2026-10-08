#!/usr/bin/env python3
"""Offline true-error chart mapping; never a landmark-truth calibration.

The frozen SE(3) gauge is fitted on the complete supplied OFF trajectory.
No candidate-specific alignment/scale or parameter selection is performed.
Source covariances remain conditional hypotheses even after this exact chart map.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation, Slerp


def skew(x):
    return np.array([[0., -x[2], x[1]], [x[2], 0., -x[0]], [-x[1], x[0], 0.]])


def jpl_product(q, p):
    z = np.r_[q[3] * p[:3] + p[3] * q[:3] - np.cross(q[:3], p[:3]), q[3] * p[3] - q[:3] @ p[:3]]
    z /= np.linalg.norm(z)
    return z if z[3] >= 0 else -z


def read_matrix(stream, item):
    stream.seek(item['offset'])
    return np.frombuffer(stream.read(8 * item['rows'] * item['cols']), dtype=np.float64).reshape((item['rows'], item['cols']), order='F').copy()


def main():
    p = argparse.ArgumentParser()
    p.add_argument('run', type=Path)
    p.add_argument('gt_csv', type=Path)
    p.add_argument('off_trajectory', type=Path)
    p.add_argument('output', type=Path)
    p.add_argument('--finite-name', default='finite.csv')
    p.add_argument('--camera-imu-offset', type=float, required=True)
    a = p.parse_args()
    if a.output.exists():
        raise RuntimeError('refusing output overwrite')
    a.output.mkdir(parents=True)
    gt = np.loadtxt(a.gt_csv, delimiter=',', skiprows=1)
    times = gt[:, 0] * 1e-9
    order = np.argsort(times)
    times, gt = times[order], gt[order]
    unique = np.r_[True, np.diff(times) > 0]
    times, gt = times[unique], gt[unique]
    # EuRoC q_RS is Hamilton body-to-world [w,x,y,z]; its xyzw coefficients
    # also describe the corresponding JPL world-to-body rotation.
    quaternions = gt[:, [5, 6, 7, 4]]
    slerp = Slerp(times, Rotation.from_quat(quaternions))
    off = np.genfromtxt(a.off_trajectory, delimiter=',', names=True)
    ref_time = np.asarray(off['timestamp']) + a.camera_imu_offset
    valid = (ref_time >= times[0]) & (ref_time <= times[-1])
    x = np.column_stack([off[k] for k in ('px', 'py', 'pz')])[valid]
    y = np.column_stack([np.interp(ref_time[valid], times, gt[:, i]) for i in (1, 2, 3)])
    if len(x) < 20:
        raise RuntimeError('insufficient OFF/GT gauge coverage')
    xc, yc = x.mean(0), y.mean(0)
    u, _, vt = np.linalg.svd((x - xc).T @ (y - yc))
    adjust = np.diag([1., 1., np.linalg.det(vt.T @ u.T)])
    align = vt.T @ adjust @ u.T
    translation = yc - align @ xc
    identity = {'off_trajectory': str(a.off_trajectory), 'off_trajectory_sha256': hashlib.sha256(a.off_trajectory.read_bytes()).hexdigest(),
                'gt_csv': str(a.gt_csv), 'gt_sha256': hashlib.sha256(a.gt_csv.read_bytes()).hexdigest(), 'camera_imu_offset': a.camera_imu_offset,
                'gauge': 'SE3 complete supplied OFF, no scale, reused unchanged for all emitted windows', 'rotation_est_to_gt': align.tolist(),
                'translation_est_to_gt': translation.tolist(), 'fit_samples': len(x), 'off_unmatched_gt': int(np.sum(~valid)),
                'real_landmark_calibration': 'NOT_EVALUATED', 'source_law_calibration': 'NOT_PROVEN',
                'scope': 'first-order actual main quaternion error chart map of conditional program covariance; all prior/source/H/K assumptions retained'}
    (a.output / 'identity.json').write_text(json.dumps(identity, indent=2) + '\n')
    index = a.run / (a.finite_name + '.matrices.jsonl')
    stream_path = a.run / (a.finite_name + '.matrices.bin')
    rows, missing = [], []
    with stream_path.open('rb') as stream, (a.output / 'mapped_covariances.bin').open('wb') as out, (a.output / 'mapped_covariances.jsonl').open('w') as idx:
        for ordinal, line in enumerate(index.read_text().splitlines()):
            event = json.loads(line)
            matrices = event['matrices']
            layout = read_matrix(stream, matrices['nominal_poses_time_orientation_row_position_row_qGtoI_xyzw_pWorld'])
            sigma = read_matrix(stream, matrices['Sigma_main_local_program'])
            stored = read_matrix(stream, matrices['P_stored_main'])
            if np.any(layout[:, 0] < times[0]) or np.any(layout[:, 0] > times[-1]):
                missing.append({'ordinal': ordinal, 'time': event['time'], 'status': 'NOT_EVALUATED_GT_OUT_OF_RANGE'})
                continue
            mapping = np.eye(len(sigma))
            mean_errors = []
            for pose in layout:
                t, row = pose[0], int(pose[1])
                r_body_to_gt = slerp([t]).as_matrix()[0]
                r_body_to_est = align.T @ r_body_to_gt
                q_true = Rotation.from_matrix(r_body_to_est).as_quat()
                q_est = pose[3:7]
                inverse = q_est.copy()
                inverse[:3] *= -1
                error = jpl_product(q_true, inverse)
                if abs(error[3]) < .1:
                    raise RuntimeError('true-error rational quaternion chart near singularity')
                e0 = 2 * error[:3] / error[3]
                mapping[row:row+3, row:row+3] = np.eye(3) - .5 * skew(e0) + .25 * np.outer(e0, e0)
                mean_errors.append(float(np.linalg.norm(e0)))
            mapped = mapping @ sigma @ mapping.T
            values = {'Sigma_main_true_error_under_declared_source_law': mapped,
                      'C_main_true_error_observer_under_declared_source_law': mapping @ read_matrix(stream, matrices['C_main_local_program_observer'])}
            for name, item in matrices.items():
                if name.startswith('C_main_local_program_anchor_'):
                    values[name.replace('C_main_local_program', 'C_main_true_error_under_declared_source_law')] = mapping @ read_matrix(stream, item)
            saved = {}
            for name, value in values.items():
                offset = out.tell()
                out.write(value.astype(np.float64).tobytes(order='F'))
                saved[name] = {'offset': offset, 'rows': value.shape[0], 'cols': value.shape[1], 'encoding': 'native_float64_column_major'}
            idx.write(json.dumps({'time': event['time'], 'source_event_ordinal': ordinal, 'matrices': saved, 'statistical_consistency_pass': False}) + '\n')
            rows.append({'time': event['time'], 'physical_main_trace_under_law': float(np.trace(mapped)), 'program_main_trace': float(np.trace(sigma)),
                         'physical_minus_program_norm': float(np.linalg.norm(mapped-sigma)), 'physical_minus_stored_norm': float(np.linalg.norm(mapped-stored)),
                         'max_main_clone_attitude_error_chart_rad': max(mean_errors)})
    if rows:
        with (a.output / 'summary.csv').open('w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)
    (a.output / 'coverage.json').write_text(json.dumps({'mapped': len(rows), 'unmatched': missing,
                                                       'statistical_consistency': 'NOT_PROVEN', 'landmark_truth': 'UNAVAILABLE'}) + '\n')
    print(json.dumps({'mapped': len(rows), 'unmatched': len(missing), 'real_calibration': 'NOT_EVALUATED'}))


if __name__ == '__main__':
    main()
