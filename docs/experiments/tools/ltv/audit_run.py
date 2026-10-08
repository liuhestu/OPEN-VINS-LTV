#!/usr/bin/env python3
"""Audit replay integrity and LTV diagnostics without consulting trajectory accuracy."""
import argparse
import collections
import csv
import json
from pathlib import Path
import numpy as np


def audit(path):
    path = Path(path)
    manifest = json.loads((path / 'manifest.json').read_text())
    replay = json.loads((path / 'replay.json').read_text())
    assert manifest['status'] == 'complete' and manifest['exit_code'] == 0
    if manifest['full']:
        assert replay['complete']
        assert replay['camera_packets'] == replay['input_camera_packets']
        assert replay['imu_consumed'] == replay['input_imu_samples']
    trajectory = np.loadtxt(path / 'trajectory.csv', delimiter=',', skiprows=1, ndmin=2)
    assert len(trajectory) == replay['output_rows'] and np.isfinite(trajectory).all()
    assert np.all(np.diff(trajectory[:, 0]) > 0)
    result = dict(path=str(path), mode=manifest['mode'], replay=replay,
                  runtime_seconds=manifest['runtime_seconds'], trajectory_finite=True)
    if manifest['mode'] == 'B':
        assert not (path / 'ltv.csv').exists()
        return result
    rows = list(csv.DictReader((path / 'ltv.csv').open()))
    assert len(rows) == replay['camera_packets'] and all(None not in row for row in rows)
    strings = {'reason', 'G_reason', 'V_reason', 'submit_reason', 'core_reset'}
    fields = {k: np.array([float(row[k]) for row in rows]) for k in rows[0] if k not in strings}
    assert all(np.isfinite(value).all() for value in fields.values())
    assert np.all((fields['EKF_calls'] >= 0) & (fields['EKF_calls'] <= 1))
    tokens = [(r['epoch'], r['attempt_id']) for r in rows if r['used_once'] == '1']
    assert len(tokens) == len(set(tokens)), 'snapshot consumed more than once'
    result.update(camera_rows=len(rows), available=int(fields['available'].sum()),
                  core_valid=int(fields['valid'].sum()),
                  submit_reasons=dict(collections.Counter(row['submit_reason'] for row in rows)),
                  pause_reasons=dict(collections.Counter(row['reason'] for row in rows)),
                  core_reset_reasons=dict(collections.Counter(row['core_reset'] for row in rows)),
                  max_ekf_calls=int(fields['EKF_calls'].max()), max_update_norm=float(fields['update_norm'].max()),
                  max_P_symmetry=float(fields['P_symmetry'].max()),
                  min_P_eigenvalue=float(fields['P_min_eigenvalue'][fields['P_checked'] > 0].min()) if np.any(fields['P_checked']) else None,
                  min_PL_eigenvalue=float(fields['PL_min_eigenvalue'][fields['PL_checked'] > 0].min()) if np.any(fields['PL_checked']) else None,
                  epoch_count=len(set(row['epoch'] for row in rows if row['available'] == '1')),
                  unique_attempts=len(tokens))
    joint = np.array([row['submit_reason'] == 'joint_applied' for row in rows])
    assert all(row['submit_reason'] in {'none', 'joint_applied'} for row in rows), 'unexpected submission rejection'
    for branch in ['G', 'V']:
        active = branch in manifest['mode']
        used = joint & (fields[branch + '_rows'] > 0)
        nis = fields[branch + '_nis']; measured = nis >= 0
        if not active:
            assert not np.any(fields[branch + '_rows'])
        result[branch] = dict(updates=int(used.sum()), active=active, no_effect=bool(active and not used.any()),
                              coverage=float(used.sum() / max(1, replay['output_rows'])),
                              coverage_all_camera=float(used.mean()),
                              eligible_count=int(fields[branch + '_eligible'].sum()),
                              coverage_eligible=float(used.sum() / max(1, fields[branch + '_eligible'].sum())),
                              reasons=dict(collections.Counter(row[branch + '_reason'] for row in rows)))
        for key in ['nis', 'residual', 'gain_norm', 'update_norm']:
            values = fields[branch + '_' + key][measured if key in ['nis', 'residual'] else used]
            result[branch][key] = dict(count=len(values), mean=float(values.mean()), max=float(values.max()),
                                      p95=float(np.quantile(values, .95))) if len(values) else None
    result['GV_joint_updates'] = int((joint & (fields['G_rows'] > 0) & (fields['V_rows'] > 0)).sum())
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = audit(args.run)
    args.output.write_text(json.dumps(result, indent=2))
    print(json.dumps(result, indent=2))
