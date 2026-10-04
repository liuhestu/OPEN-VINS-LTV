"""Exact before/after adapter comparison on identical existing CSR sensor inputs.

Uses the production C02 input-only wrapper and runner; no truth, tuning or new
observer implementation. Mode 4 is active-consistency OFF, mode 5 is frozen C02.
"""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    'ltv_c02_input_runner', ROOT / 'scripts/ltv_active_consistency_r4/synthetic_runner.py')
RUNNER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(RUNNER)


def collect(library, input_dir):
    """One library per spawned process: equal native SONAMEs cannot be reused."""
    with np.load(Path(input_dir) / 'inputs.npz', allow_pickle=False) as archive:
        data = {key: archive[key] for key in archive.files}
    results = []
    for mode in (0, 4, 5):
        adapter = RUNNER.Adapter(library, mode, 'HYBRID', float(data['bearing_sigma_rad']))
        digest = hashlib.sha256()
        records = []
        births = skips = retires = 0
        imu_cursor = 0
        try:
            for k, time in enumerate(data['camera_times']):
                while imu_cursor < len(data['imu_sample_times']) and data['imu_sample_times'][imu_cursor] <= time + .005 + 1e-12:
                    adapter.imu(data['imu_sample_times'][imu_cursor], data['imu_samples'][imu_cursor])
                    imu_cursor += 1
                event = adapter.frame(data, k)
                event.pop('compute_time_ms', None)
                encoded = json.dumps(event, sort_keys=True, allow_nan=True)
                state = adapter.state(True)
                # Include the full padded arrays, including actual core x/P and slots.
                # Padding is generated deterministically by the unchanged runner.
                record = {'event': encoded, 'dimension': state[3]}
                digest.update(encoded.encode())
                for value, name in zip(state[:3], ('x', 'P', 'slots')):
                    raw = value.tobytes()
                    record[name] = hashlib.sha256(raw).hexdigest()
                    digest.update(raw)
                records.append(record)
                births += sum(bool(seed['mean_written']) for seed in event['seeds'])
                skips += event.get('consistency_skip_count', 0)
                retires += event.get('consistency_retire_count', 0)
        finally:
            adapter.close()
        results.append({'mode': mode, 'frames': len(records), 'imu_samples': imu_cursor,
                        'mean_writes': births, 'consistency_skips': skips,
                        'consistency_retires': retires, 'numeric_sha': digest.hexdigest(),
                        'records': records, 'status': 'PASS_EXACT'})
    return results


def compare(before, after, input_dir, output):
    from concurrent.futures import ProcessPoolExecutor
    from multiprocessing import get_context

    input_dir = Path(input_dir).resolve()
    source = input_dir / 'inputs.npz'
    identity = json.loads((input_dir / 'identity.json').read_text())
    input_sha = hashlib.sha256(source.read_bytes()).hexdigest()
    if input_sha != identity['input_sha']:
        raise ValueError('Input identity mismatch')
    libraries = [str(Path(path).resolve()) for path in (before, after)]
    result = {'baseline': 'f3e055ab2dc185d3a5910c9fa60c505dd912671d',
              'input_sha': input_sha, 'comparison': 'exact_full_state_P_slots_and_non_wallclock_event_JSON',
              'excluded_fields': ['compute_time_ms'],
              'process_isolation': 'spawned_process_per_library_no_shared_SONAME_reuse',
              'libraries': {name: hashlib.sha256(Path(path).read_bytes()).hexdigest()
                            for name, path in zip(('before', 'after'), libraries)}, 'modes': []}
    with ProcessPoolExecutor(max_workers=2, mp_context=get_context('spawn')) as executor:
        futures = [executor.submit(collect, path, str(input_dir)) for path in libraries]
        runs = [future.result() for future in futures]
    for old, new in zip(*runs):
        if old != new:
            for k, (a, b) in enumerate(zip(old['records'], new['records'])):
                for field in a:
                    if a[field] != b[field]:
                        raise AssertionError(f"mode {old['mode']}, camera {k}: {field} differs")
            raise AssertionError(f"mode {old['mode']}: counts or numeric digest differs")
        old.pop('records')
        result['modes'].append(old)
    result['status'] = 'PASS_EXACT_TESTED_INPUT'
    Path(output).write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--before', required=True)
    parser.add_argument('--after', required=True)
    parser.add_argument('--input-dir', required=True)
    parser.add_argument('--output', required=True)
    compare(**vars(parser.parse_args()))
