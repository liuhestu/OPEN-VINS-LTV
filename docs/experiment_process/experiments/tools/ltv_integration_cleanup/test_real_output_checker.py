"""Tampering tests for the exact comparator; no estimator or simulation runs."""
import json
import struct
import tempfile
from pathlib import Path

from check_real_outputs import compare


def fixture(root, timing):
    root.mkdir()
    (root / 'audit.csv').write_text('camera_ns,state_digest\n1,state-and-full-covariance-digest\n')
    (root / 'trajectory.csv').write_text('timestamp,vx\n1,0.125\n')
    (root / 'unmatched_camera.csv').write_text('camera_id,timestamp_ns,filename\n')
    event = {'camera_ns': 1, 'v': [0.125, 0.0, 0.0], 'compute_time_ms': timing,
             'nested': {'compute_time_ms': 7}, 'ready_V': True}
    (root / 'features.jsonl').write_text(json.dumps(event) + '\n')
    metadata = {'complete': True, 'short_limit_seconds': 0, 'camera_packets': 1,
                'input_camera_packets': 1, 'imu_consumed': 1, 'input_imu_samples': 1,
                'actual_G_submissions': 0, 'actual_V_submissions': 0}
    (root / 'replay.json').write_text(json.dumps(metadata))
    (root / 'effective_options.json').write_text(json.dumps({'ltv_enable_gravity': False, 'ltv_enable_velocity': False}))
    with (root / 'cache.bin').open('wb') as cache:
        cache.write(b'LTVPCACHE001\0')
        for kind, payload in ((1, struct.pack('<d', 1)),
                              (2, struct.pack('<d', 1) + b'x/P-and-ACTIVE_CONSISTENCY_V1'), (0, b'')):
            cache.write(struct.pack('<BQ', kind, len(payload)) + payload)


def reject(before, after, edit):
    original = {p: p.read_bytes() for p in after.iterdir()}
    edit(after)
    try:
        compare(before, after)
    except AssertionError:
        pass
    else:
        raise AssertionError('Tampering unexpectedly accepted')
    finally:
        for path, data in original.items():
            path.write_bytes(data)


def edit_json(path, field, value):
    data = json.loads(path.read_text())
    data[field] = value
    path.write_text(json.dumps(data) + '\n')


if __name__ == '__main__':
    with tempfile.TemporaryDirectory() as temp:
        before, after = Path(temp) / 'before', Path(temp) / 'after'
        fixture(before, 1)
        fixture(after, 999)
        assert compare(before, after)['status'] == 'PASS_EXACT_NATIVE_PASSIVE'
        reject(before, after, lambda root: edit_json(root / 'features.jsonl', 'v', [0.126, 0.0, 0.0]))
        reject(before, after, lambda root: edit_json(root / 'features.jsonl', 'v', [0.125, -0.0, 0.0]))
        reject(before, after, lambda root: edit_json(root / 'features.jsonl', 'nested', {'compute_time_ms': 8}))
        reject(before, after, lambda root: (root / 'audit.csv').write_text('camera_ns,state_digest\n1,changed-covariance\n'))
        reject(before, after, lambda root: (root / 'cache.bin').write_bytes(
            (root / 'cache.bin').read_bytes().replace(b'ACTIVE_CONSISTENCY_V1', b'ACTIVE_CONSISTENCY_X1')))
        reject(before, after, lambda root: (root / 'cache.bin').write_bytes((root / 'cache.bin').read_bytes()[:-1]))
        # Invalid evidence must be rejected even when both sides agree exactly.
        edit_json(before / 'replay.json', 'complete', False)
        edit_json(after / 'replay.json', 'complete', False)
        try:
            compare(before, after)
        except AssertionError:
            pass
        else:
            raise AssertionError('Incomplete identical runs accepted')
    print('PASS: root timing-only difference accepted; state, covariance, nested fields, cache suffix, truncation and incomplete runs rejected')
