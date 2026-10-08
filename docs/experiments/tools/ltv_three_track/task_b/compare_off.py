"""Zero-tolerance original/scalar/same-diagnostic landmark-OFF checks.

Wall time and intentionally disabled audit-only storage are excluded explicitly.
A prefix is labelled a prefix; it does not grant complete-sequence qualification.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def rows(path):
    return [json.loads(x) for x in path.open()]


def compare(left, right):
    checks = {}
    for name in ['trajectory.csv', 'unmatched_camera.csv', 'lifecycle_identity.jsonl', 'algorithm_visual_identity.jsonl']:
        a, b = sha(left / name), sha(right / name)
        assert a == b, name
        checks[name] = a
    with (left / 'audit.csv').open() as a, (right / 'audit.csv').open() as b:
        ar, br = list(csv.DictReader(a)), list(csv.DictReader(b))
    assert len(ar) == len(br) and ar
    for i, (a, b) in enumerate(zip(ar, br)):
        for key in ['camera_ns', 'initialized', 'state_time', 'clones', 'state_digest', 'tracker_digest']:
            assert a[key] == b[key], (i, key)
    observers = []
    for path in [left, right]:
        name = 'observer_identity.jsonl' if (path / 'observer_identity.jsonl').exists() else 'fusion.jsonl'
        observers.append(rows(path / name))
    assert len(observers[0]) == len(observers[1]) == len(ar)
    for i, (a, b) in enumerate(zip(*observers)):
        assert a['camera_ns'] == b['camera_ns'] and a['observer_digest'] == b['observer_digest'], ('observer', i)
    replay = [json.loads((p / 'replay.json').read_text()) for p in [left, right]]
    for key in ['camera_packets', 'imu_consumed', 'complete', 'short_limit_seconds']:
        assert replay[0][key] == replay[1][key], ('replay', key)
    checks.update(frames=len(ar), full_state_P_tracker=True, observer_full_x_P=True,
                  mixed_audit_visual_digest='NOT_COMPARABLE; pure algorithm visual output is exact', replay=replay)
    return dict(status='PASS_EXACT_WITHIN_SCOPE', scope='FULL_SEQUENCE' if replay[0]['complete'] else 'REGISTERED_PREFIX_ONLY',
                left=str(left), right=str(right), checks=checks)


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('left', type=Path)
    p.add_argument('right', type=Path)
    p.add_argument('output', type=Path)
    a = p.parse_args()
    result = compare(a.left, a.right)
    with a.output.open('x') as stream:
        json.dump(result, stream, indent=2)
        stream.write('\n')
    print(result['status'], result['scope'])
