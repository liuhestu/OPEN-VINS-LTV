"""Verify actual native ON exercise and safety; never promotes a prefix to qualification."""
import argparse
import csv
import json
import math
from pathlib import Path


def read(path):
    with path.open() as stream:
        return list(csv.DictReader(stream))


def audit(off, on):
    options = [json.loads((p / 'effective_options.json').read_text()) for p in [off, on]]
    assert not options[0]['ltv_enable_landmark_approx'] and options[1]['ltv_enable_landmark_approx']
    for x in options:
        assert not x['ltv_enable_gravity'] and not x['ltv_enable_velocity'] and x['max_slam_features'] == 0
        assert x['ltv_landmark_approx_max_points'] == 2
        assert x['ltv_landmark_approx_sigma_floor_m'] == .5 and x['ltv_landmark_approx_information_cap'] == .01
    a, b = [read(p / 'audit.csv') for p in [off, on]]
    assert len(a) == len(b) and a
    assert all(x['camera_ns'] == y['camera_ns'] and x['initialized'] == y['initialized'] for x, y in zip(a, b))
    times = [int(x['camera_ns']) for x in b]
    frames = read(on / 'landmark_approx.csv')
    selected, applied, nonzero = [], [], []
    for f in frames:
        assert int(f['selected']) <= 2
        for field in ['residual_norm', 'variance_min_m2', 'variance_max_m2', 'information_budget_used',
                      'landmark_delta_norm', 'landmark_bg_delta_norm', 'landmark_ba_delta_norm', 'p_min_eigenvalue']:
            assert math.isfinite(float(f[field])), (f['camera_time'], field)
        assert float(f['information_budget_used']) <= .01 + 1e-12
        if int(f['selected']):
            selected.append(f)
            assert float(f['variance_min_m2']) > 0
        if int(f['rows']) > 0 and f['consumed'] == '1' and f['submit_reason'] == 'joint_applied':
            applied.append(f)
            assert int(f['ekf_calls']) == 1 and float(f['p_min_eigenvalue']) >= -1e-10
            if float(f['landmark_delta_norm']) > 0:
                nonzero.append(f)
    assert applied and nonzero, 'ON was not actually exercised; investigate supply, never claim ON pass'
    point_rows = read(on / 'landmark_approx.csv.points.csv')
    actual_points = [r for r in point_rows if r['applied'] == '1']
    assert actual_points and len(actual_points) == sum(int(f['rows']) // 3 for f in applied)
    for r in actual_points:
        assert float(r['variance_m2']) > 0 and 0 <= float(r['information_after']) <= .005 + 1e-12
        assert all(math.isfinite(float(r[k])) for k in ['rx_m', 'ry_m', 'rz_m'])
    first_applied_time = float(applied[0]['camera_time'])
    changed = [int(y['camera_ns']) for x, y in zip(a, b) if x['state_digest'] != y['state_digest']]
    assert changed, 'no actual main state/P difference'
    first = min(times, key=lambda t: abs(t * 1e-9 - first_applied_time))
    assert abs(first * 1e-9 - first_applied_time) <= 1e-6
    assert changed[0] >= first, 'state/P changed before first true landmark application'
    ta, tb = [read(p / 'trajectory.csv') for p in [off, on]]
    assert len(ta) == len(tb) and all(x['timestamp'] == y['timestamp'] for x, y in zip(ta, tb))
    mean_changes = []
    fields = [k for k in ta[0] if k != 'timestamp']
    for x, y in zip(ta, tb):
        assert all(math.isfinite(float(y[k])) for k in fields)
        if any(x[k] != y[k] for k in fields):
            mean_changes.append(float(y['timestamp']))
    replay = [json.loads((p / 'replay.json').read_text()) for p in [off, on]]
    assert replay[0]['camera_packets'] == replay[1]['camera_packets'] == len(a)
    assert replay[0]['imu_consumed'] == replay[1]['imu_consumed']
    return {'status': 'PASS_ACTUAL_EXERCISE_NUMERICS_WITHIN_SCOPE',
            'scope': 'FULL_SEQUENCE' if replay[1]['complete'] else 'REGISTERED_40S_PREFIX_ONLY',
            'frames': len(a), 'selected_frames': len(selected), 'actual_applied_frames': len(applied),
            'actual_applied_points': len(actual_points), 'nonzero_component_frames': len(nonzero),
            'first_applied_camera_time': first_applied_time, 'first_full_main_state_P_difference_ns': changed[0],
            'current_IMU_mean_changed': bool(mean_changes), 'first_current_mean_change_s': mean_changes[0] if mean_changes else None,
            'maximum_landmark_component_norm_mixed_units': max(float(f['landmark_delta_norm']) for f in applied),
            'maximum_gyro_bias_component_rad_s': max(float(f['landmark_bg_delta_norm']) for f in applied),
            'maximum_accel_bias_component_m_s2': max(float(f['landmark_ba_delta_norm']) for f in applied),
            'approximation': 'shared-source cross blocks omitted, not proved zero; native covariance chart limitation',
            'engineering_full_non_degradation': 'NOT_EVALUATED_BY_THIS_SAFETY_AUDIT',
            'accuracy_benefit': 'NOT_EVALUATED_BY_THIS_SAFETY_AUDIT', 'statistical_calibration': 'NOT_CLAIMED',
            'off_path': str(off), 'on_path': str(on)}


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('off', type=Path)
    p.add_argument('on', type=Path)
    p.add_argument('output', type=Path)
    a = p.parse_args()
    result = audit(a.off, a.on)
    with a.output.open('x') as stream:
        json.dump(result, stream, indent=2)
        stream.write('\n')
    print(json.dumps(result, indent=2))
