"""Independent exact passive-output audit. Does not load GT or tune a selector."""
import argparse
import csv
import json
import math
from pathlib import Path


def rows(path):
    with Path(path).open(newline='') as f:
        return list(csv.DictReader(f))


def inspect(directory):
    p = Path(directory)
    metadata = json.loads((p / 'replay.json').read_text())
    config = json.loads((p / 'effective_options.json').read_text())
    audit = rows(p / 'audit.csv')
    trajectory = rows(p / 'trajectory.csv')
    for key in ('ltv_enable_gravity', 'ltv_enable_velocity'):
        if config[key]:
            raise ValueError(f'injection configured: {key}')
    if not config['ltv_passive_audit_enabled']:
        raise ValueError('passive audit disabled')
    mode = metadata['mode']
    if mode not in ('B', 'P_OLD', 'P_NEW'):
        raise ValueError('unknown mode')
    if bool(config['ltv_enabled']) != (mode != 'B') or bool(config['ltv_feature_readiness_enabled']) != (mode == 'P_NEW'):
        raise ValueError('effective mode mismatch')
    if len(audit) != metadata['camera_packets'] or len(trajectory) != metadata['output_rows']:
        raise ValueError('missing audit or trajectory rows')
    times = [int(x['camera_ns']) for x in audit]
    if not times or any(b <= a for a, b in zip(times, times[1:])):
        raise ValueError('empty/nonmonotonic camera audit')
    for row in audit:
        for key in ('auxiliary_receipts', 'gravity_submissions', 'velocity_submissions'):
            if int(row[key]) != 0:
                raise ValueError(f'actual injection: {key}')
        for key in ('state_digest', 'tracker_digest', 'visual_digest'):
            digest = row[key]
            if len(digest) != 40 or any(c not in '0123456789abcdef' for c in digest):
                raise ValueError(f'invalid digest: {key}')
    if any(not math.isfinite(float(value)) for row in trajectory for value in row.values()):
        raise ValueError('nonfinite main trajectory')
    if metadata['actual_G_submissions'] or metadata['actual_V_submissions']:
        raise ValueError('nonzero aggregate injection')
    if metadata['complete']:
        if metadata['camera_packets'] != metadata['input_camera_packets'] or metadata['imu_consumed'] != metadata['input_imu_samples']:
            raise ValueError('incomplete formal sensor consumption')
        if not trajectory:
            raise ValueError('full replay never initialized')
    elif not 0 < metadata['short_limit_seconds'] <= 10 or not 0 <= metadata['input_span_seconds'] <= metadata['short_limit_seconds'] + 1e-9:
        raise ValueError('short input exceeded ten seconds')
    return dict(mode=mode, camera_packets=len(audit), trajectory_rows=len(trajectory),
                complete=metadata['complete'], zero_injection=True, status='PASS')


def compare(directories):
    paths = [Path(p) for p in directories]
    if len(paths) < 2:
        raise ValueError('comparison requires at least two runs')
    evidence = [inspect(p) for p in paths]
    for name in ('audit.csv', 'trajectory.csv', 'unmatched_camera.csv'):
        expected = (paths[0] / name).read_bytes()
        for path in paths[1:]:
            if (path / name).read_bytes() != expected:
                raise ValueError(f'passive mismatch: {name}: {paths[0]} != {path}')
    return dict(status='PASS', exact_files=['audit.csv', 'trajectory.csv', 'unmatched_camera.csv'], runs=evidence)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directories', nargs='+', type=Path)
    args = parser.parse_args()
    result = inspect(args.directories[0]) if len(args.directories) == 1 else compare(args.directories)
    print(json.dumps(result, indent=2))
