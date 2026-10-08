"""Exact native Passive regression, including the complete serialized LTV cache.

No numerical tolerance. Only features.jsonl's top-level compute_time_ms is
removed. Full cache bytes include x/P, input, slots and active-consistency events;
this deliberately does not use the older decoder that ignores cache suffixes.
"""
import argparse
import csv
import hashlib
import json
import struct
from itertools import zip_longest
from pathlib import Path


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1 << 20), b''):
            digest.update(block)
    return digest.hexdigest()


def first_difference(a, b, prefix=''):
    if type(a) is not type(b):
        return prefix, a, b
    if isinstance(a, dict):
        if a.keys() != b.keys():
            return prefix + '.keys', sorted(a), sorted(b)
        for key in a:
            difference = first_difference(a[key], b[key], prefix + '.' + key)
            if difference:
                return difference
    elif isinstance(a, list):
        if len(a) != len(b):
            return prefix + '.length', len(a), len(b)
        for i, (x, y) in enumerate(zip(a, b)):
            difference = first_difference(x, y, prefix + f'[{i}]')
            if difference:
                return difference
    elif isinstance(a, float):
        if struct.pack('<d', a) != struct.pack('<d', b):
            return prefix, a, b
    elif a != b:
        return prefix, a, b
    return None


def csv_exact(a, b):
    with a.open() as left, b.open() as right:
        rows = 0
        for i, (x, y) in enumerate(zip_longest(csv.reader(left), csv.reader(right))):
            if x != y:
                raise AssertionError(dict(file=a.name, row=i, left=x, right=y))
            rows += 1
    return rows - 1


def jsonl_exact(a, b):
    digest = hashlib.sha256()
    with a.open() as left, b.open() as right:
        rows = 0
        for i, (x, y) in enumerate(zip_longest(left, right)):
            if x is None or y is None:
                raise AssertionError(dict(file=a.name, row=i, reason='length mismatch'))
            x, y = json.loads(x), json.loads(y)
            timestamp = x.get('camera_ns')
            x.pop('compute_time_ms', None)
            y.pop('compute_time_ms', None)
            difference = first_difference(x, y)
            if difference:
                raise AssertionError(dict(file=a.name, row=i, camera_ns=timestamp, difference=difference))
            canonical = json.dumps(x, sort_keys=True, separators=(',', ':'), allow_nan=False)
            if canonical != json.dumps(y, sort_keys=True, separators=(',', ':'), allow_nan=False):
                raise AssertionError(dict(file=a.name, row=i, camera_ns=timestamp, reason='canonical JSON mismatch'))
            digest.update(canonical.encode())
            rows += 1
    return dict(rows=rows, canonical_sha=digest.hexdigest())


def cache_exact(a, b):
    counts = dict(imu=0, process=0, pause=0)
    with a.open('rb') as left, b.open('rb') as right:
        magic = b'LTVPCACHE001\0'
        if left.read(len(magic)) != magic or right.read(len(magic)) != magic:
            raise AssertionError('Invalid cache magic')
        index = 0
        while True:
            headers = [s.read(9) for s in (left, right)]
            if headers[0] != headers[1] or len(headers[0]) != 9:
                raise AssertionError(dict(file=a.name, record=index, reason='event type/length or truncated header'))
            kind, length = struct.unpack('<BQ', headers[0])
            if length > 256 * 1024 * 1024:
                raise AssertionError('Invalid cache record size')
            x, y = left.read(length), right.read(length)
            if len(x) != length or len(y) != length:
                raise AssertionError('Truncated cache record')
            if x != y:
                offset = next(i for i, (u, v) in enumerate(zip(x, y)) if u != v)
                timestamp = struct.unpack('<d', x[:8])[0] if len(x) >= 8 else None
                raise AssertionError(dict(file=a.name, record=index, kind=kind, timestamp=timestamp, payload_byte=offset))
            if kind == 0:
                if length or left.read(1) or right.read(1):
                    raise AssertionError('Trailing cache bytes')
                break
            if kind not in (1, 2, 3):
                raise AssertionError('Invalid cache event')
            counts[{1: 'imu', 2: 'process', 3: 'pause'}[kind]] += 1
            index += 1
    return dict(counts=counts, sha256=sha(a), full_serialized_payload_exact=True)


def compare(before, after):
    before, after = Path(before), Path(after)
    result = dict(status='PASS_EXACT_NATIVE_PASSIVE', before=str(before), after=str(after),
                  excluded_fields=['features.jsonl.compute_time_ms'], tolerance=0)
    result['audit_rows'] = csv_exact(before / 'audit.csv', after / 'audit.csv')
    result['trajectory_rows'] = csv_exact(before / 'trajectory.csv', after / 'trajectory.csv')
    result['unmatched_camera_rows'] = csv_exact(before / 'unmatched_camera.csv', after / 'unmatched_camera.csv')
    result['features'] = jsonl_exact(before / 'features.jsonl', after / 'features.jsonl')
    result['cache'] = cache_exact(before / 'cache.bin', after / 'cache.bin')
    for filename in ('effective_options.json', 'replay.json'):
        difference = first_difference(json.loads((before / filename).read_text()), json.loads((after / filename).read_text()))
        if difference:
            raise AssertionError(dict(file=filename, difference=difference))
    metadata = json.loads((before / 'replay.json').read_text())
    options = json.loads((before / 'effective_options.json').read_text())
    if not metadata['complete'] or metadata['short_limit_seconds'] != 0:
        raise AssertionError('Complete real input required')
    if metadata['camera_packets'] != metadata['input_camera_packets'] or metadata['imu_consumed'] != metadata['input_imu_samples']:
        raise AssertionError('Incomplete sensor consumption')
    if any(metadata[key] for key in ('actual_G_submissions', 'actual_V_submissions')) or any(options[key] for key in ('ltv_enable_gravity', 'ltv_enable_velocity')):
        raise AssertionError('G/V injection must be disabled')
    if result['audit_rows'] <= 0 or result['features']['rows'] != result['audit_rows'] or result['audit_rows'] != metadata['camera_packets']:
        raise AssertionError('Camera timeline mismatch or empty run')
    if result['cache']['counts']['process'] + result['cache']['counts']['pause'] != metadata['camera_packets']:
        raise AssertionError('Missing full camera cache evidence')
    if result['cache']['counts']['imu'] != metadata['input_imu_samples']:
        raise AssertionError('Missing full IMU cache evidence')
    result['files'] = {name: dict(before_sha=sha(before / name), after_sha=sha(after / name))
                       for name in ('audit.csv', 'trajectory.csv', 'cache.bin', 'features.jsonl', 'effective_options.json', 'replay.json', 'unmatched_camera.csv')}
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('before', type=Path)
    parser.add_argument('after', type=Path)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    if args.out.exists():
        raise FileExistsError(args.out)
    try:
        result = compare(args.before, args.after)
    except Exception as error:
        args.out.write_text(json.dumps(dict(status='FAIL_EXACT_NATIVE_PASSIVE', error=str(error)), indent=2) + '\n')
        raise
    args.out.write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))
