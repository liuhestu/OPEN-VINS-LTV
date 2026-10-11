#!/usr/bin/env python3
"""Exact state/P, Observer, lifecycle, readiness and input parity; wall time excluded."""
import argparse
import csv
import hashlib
import json
from pathlib import Path


def sha(path):
    h = hashlib.sha256()
    with path.open('rb') as f:
        for block in iter(lambda: f.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def rows(path):
    return [json.loads(x) for x in path.read_text().splitlines() if x]


def compare(diagnostic, scalar):
    checks = {}
    for filename in ('trajectory.csv', 'unmatched_camera.csv', 'lifecycle_identity.jsonl', 'algorithm_visual_identity.jsonl'):
        checks[filename] = {'diagnostic_sha256': sha(diagnostic / filename), 'scalar_sha256': sha(scalar / filename)}
        assert checks[filename]['diagnostic_sha256'] == checks[filename]['scalar_sha256'], filename
    with (diagnostic / 'audit.csv').open() as a, (scalar / 'audit.csv').open() as b:
        left, right = list(csv.DictReader(a)), list(csv.DictReader(b))
    assert len(left) == len(right) and left, 'audit cardinality'
    for index, (a, b) in enumerate(zip(left, right)):
        for key in ('camera_ns', 'initialized', 'state_time', 'clones', 'state_digest', 'tracker_digest'):
            assert a[key] == b[key], (index, key)
    checks['audit_exact_full_state_P_tracker'] = len(left)
    checks['algorithm_visual_exact'] = len(left)
    checks['mixed_visual_digest'] = 'NOT_COMPARABLE: contains intentionally disabled passive-only audit containers; original failed attempt retained'
    a = rows(diagnostic / 'fusion.jsonl')
    b = rows(scalar / 'observer_identity.jsonl')
    assert len(a) == len(b) == len(left)
    for index, (x, y) in enumerate(zip(a, b)):
        assert x['camera_ns'] == y['camera_ns'] and x['observer_digest'] == y['observer_digest'], ('observer', index)
    checks['observer_exact_full_x_P_R'] = len(a)
    for path in (diagnostic, scalar):
        r = json.loads((path / 'replay.json').read_text())
        assert r['complete'] and r['short_limit_seconds'] == 0
        assert r['camera_packets'] == r['input_camera_packets'] == len(left)
        assert r['imu_consumed'] == r['input_imu_samples']
        checks[str(path) + '_replay'] = r
    return {'status': 'PASS_WITHIN_SCOPE', 'scope': 'full deterministic synchronous ASL input; no ROS transport qualification',
            'comparison': 'exact_zero_tolerance', 'checks': checks}


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('diagnostic', type=Path)
    p.add_argument('scalar', type=Path)
    p.add_argument('output', type=Path)
    args = p.parse_args()
    result = compare(args.diagnostic, args.scalar)
    with args.output.open('x') as f:
        json.dump(result, f, indent=2, allow_nan=False)
        f.write('\n')
