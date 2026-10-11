"""Reconstruct actual diagnostic native landmark K*r blocks, with declared units.

Input is real joint prior/H/S/residual/layout capture, not Riccati covariance.
Numerical reconstruction validates existing native receipts, not statistics.
"""
import argparse
import csv
import json
from pathlib import Path
import numpy as np


def reconstruct(path):
    with (path / 'landmark_approx.csv').open() as stream:
        frames = list(csv.DictReader(stream))
    applied = {round(float(f['camera_time']) * 1e9): f for f in frames
               if int(f['rows']) > 0 and f['consumed'] == '1' and f['submit_reason'] == 'joint_applied'}
    assert applied
    results = []
    for line in (path / 'matrices.jsonl').open():
        r = json.loads(line)
        time = int(r['camera_ns'])
        matched = min(applied, key=lambda t: abs(t-time))
        if abs(matched-time) > 1000:
            continue
        f = applied[matched]
        P, H, S = [np.asarray(r[k], float) for k in ['prior_P', 'joint_H', 'joint_S']]
        residual = np.asarray(r['joint_res'], float).reshape(-1)
        assert P.ndim == H.ndim == S.ndim == 2 and P.shape[0] == P.shape[1]
        Hd = np.zeros((H.shape[0], P.shape[0]))
        offset = 0
        for index, width in zip(r['joint_ids'], r['joint_sizes']):
            Hd[:, index:index+width] = H[:, offset:offset+width]
            offset += width
        assert offset == H.shape[1]
        # Same upper-selfadjoint innovation as the recorded native implementation.
        assert np.allclose(S, S.T, rtol=0, atol=1e-10)
        K = np.linalg.solve(S, Hd @ P).T
        rows = int(f['rows'])
        delta = K[:, -rows:] @ residual[-rows:]
        assert np.isfinite(delta).all()
        assert abs(np.linalg.norm(delta) - float(f['landmark_delta_norm'])) <= 1e-10
        assert abs(np.linalg.norm(delta[9:12]) - float(f['landmark_bg_delta_norm'])) <= 1e-10
        assert abs(np.linalg.norm(delta[12:15]) - float(f['landmark_ba_delta_norm'])) <= 1e-10
        results.append(dict(camera_ns=time, landmark_rows=rows, native_prior_dimension=P.shape[0],
                            theta_rad=delta[0:3].tolist(), position_G_m=delta[3:6].tolist(),
                            velocity_G_m_s=delta[6:9].tolist(), gyro_bias_I_rad_s=delta[9:12].tolist(),
                            accel_bias_I_m_s2=delta[12:15].tolist(), full_error_state_norm_mixed_units=float(np.linalg.norm(delta))))
    assert len(results) == len(applied), (len(results), len(applied))
    fields = ['theta_rad', 'position_G_m', 'velocity_G_m_s', 'gyro_bias_I_rad_s', 'accel_bias_I_m_s2']
    maxima = {k: float(max(np.linalg.norm(r[k]) for r in results)) for k in fields}
    return {'status': 'PASS_NATIVE_COMPONENT_RECONSTRUCTION', 'scope': 'actual joint matrix capture and receipt components',
            'mode': 'G/V OFF; approximation shared-source cross blocks omitted', 'calibration': 'NOT_CLAIMED',
            'frames': len(results), 'maximum_component_norms': maxima, 'records': results}


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('diagnostic_run', type=Path)
    p.add_argument('output', type=Path)
    a = p.parse_args()
    x = reconstruct(a.diagnostic_run)
    with a.output.open('x') as stream:
        json.dump(x, stream, indent=2)
        stream.write('\n')
    print(json.dumps({k: v for k, v in x.items() if k != 'records'}, indent=2))
