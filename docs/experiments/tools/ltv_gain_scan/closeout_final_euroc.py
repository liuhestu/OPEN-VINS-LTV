#!/usr/bin/env python3
"""Recheck a completed frozen experiment without writing to its raw directory."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path

import numpy as np

import final_euroc as experiment


def read(path):
    return json.loads(path.read_text())


def compare(actual, expected, label):
    if isinstance(expected, dict):
        assert actual.keys() == expected.keys(), label + ': keys'
        for key in expected:
            compare(actual[key], expected[key], label + '/' + key)
    elif isinstance(expected, list):
        assert len(actual) == len(expected), label + ': length'
        for i, value in enumerate(expected):
            compare(actual[i], value, label + '/' + str(i))
    elif isinstance(expected, float):
        assert np.isclose(actual, expected, rtol=1e-11, atol=1e-12), label
    else:
        assert actual == expected, label


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('coord', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    obj = experiment.Final(args.coord)
    dest = experiment.DEST
    recorded = read(dest / 'sha256.json')
    for name, digest in recorded.items():
        assert experiment.sha(dest / name) == digest, 'published hash: ' + name
    captured = {}
    original_write = experiment.write
    experiment.write = lambda path, value: captured.update({Path(path).name: value})
    try:
        obj.audit()
        obj.evaluate_metrics()
    finally:
        experiment.write = original_write
    assert captured['independent_audit.json']['status'] == 'PASS'
    for name in ['metrics.json', 'local_windows.json']:
        compare(captured[name], read(dest / name), name)
        compare(captured[name], read(args.coord / name), 'raw/' + name)
    performance = read(args.coord / 'performance_index.json')
    expected_pairs = {(seq, mode) for seq in experiment.EUROC for mode in experiment.MODES}
    assert len(performance) == 50
    assert {(r['sequence'], r['mode']) for r in performance} == expected_pairs
    compare(performance, read(dest / 'performance_index.json'), 'performance_index')
    published = {(r['sequence'], r['mode']): r for r in read(dest / 'results.json')}
    assert len(published) == 50 and set(published) == expected_pairs
    input_files = 0
    for seq, item in obj.inputs.items():
        assert experiment.sha(item['gt']) == item['gt_sha256'], 'GT: ' + seq
        for name, digest in read(args.coord / 'support' / (seq + '_input_hashes.json')).items():
            assert experiment.sha(Path(item['root']) / name) == digest, 'input: ' + seq + '/' + name
            input_files += 1
        print('input hashes verified: ' + seq, flush=True)
    observer = None
    for row in obj.index + performance:
        assert row['exit_code'] == 0 and row['status'] == 'VALID_FULL_MATCHED'
        out = Path(row['output_dir'])
        options = read(out / 'effective_options.json')
        assert read(out / 'threading.json')['actual_opencv_threads'] == 1
        assert options['max_slam_features'] == 0
        g, v, l = experiment.PROFILES[experiment.scene(row['sequence'])][row['mode']]
        wanted = dict(ltv_sigma_gravity_deg=10 / np.sqrt(g or 1),
                      ltv_sigma_velocity_mps=1 / np.sqrt(v or 1),
                      ltv_landmark_approx_sigma_floor_m=.5 / np.sqrt(l or 1),
                      ltv_landmark_approx_information_cap=.01 * (l or 1),
                      ltv_landmark_approx_max_points=2)
        for key, value in wanted.items():
            assert np.isclose(options[key], value, rtol=1e-14, atol=0), key
        if row['mode'] == 'OFF':
            assert not options['ltv_enabled']
        else:
            current = {k: value for k, value in options.items() if k.startswith('ltv_observer_')}
            if observer is None:
                observer = current
            assert current == observer, 'Observer parameters changed'
        if row['phase'] == 'accuracy':
            p = dict(published[(row['sequence'], row['mode'])])
            delta = p.pop('relative_off_percent')
            compare(row, p, 'published accuracy')
            off = obj.lookup_mode(row['sequence'], 'OFF')['ate_rmse_m']
            assert np.isclose(delta, 100 * (row['ate_rmse_m'] / off - 1), atol=1e-12)
        else:
            first = Path(obj.lookup_mode(row['sequence'], row['mode'])['output_dir'])
            assert (out / 'trajectory.csv').read_bytes() == (first / 'trajectory.csv').read_bytes()
            timing = np.loadtxt(out / 'frame_processing.csv', delimiter=',', skiprows=1, ndmin=2)[:, 1]
            resources = read(out / 'resources.json')
            cost = dict(camera_api_p50_ms=float(np.quantile(timing, .5)),
                        camera_api_p95_ms=float(np.quantile(timing, .95)),
                        camera_api_p99_ms=float(np.quantile(timing, .99)),
                        camera_api_max_ms=float(timing.max()), full_wall_s=resources['elapsed_s'],
                        full_cpu_s=resources['user_s'] + resources['system_s'],
                        peak_rss_kb=resources['max_rss_kb'], timing_samples=len(timing))
            compare(cost, row['cost'], 'raw cost')
    summary = read(dest / 'analysis_summary.json')
    for item in summary['scene_ate']:
        changes = [r['relative_off_percent'] for r in published.values()
                   if r['scene'] == item['scene'] and r['mode'] == item['mode']]
        assert item['sequences'] == len(changes)
        assert item['lower'] == sum(v < 0 for v in changes)
        assert item['equal'] == sum(v == 0 for v in changes)
        assert item['higher'] == sum(v > 0 for v in changes)
        for name, reducer in [('minimum_percent', min), ('median_percent', np.median), ('maximum_percent', max)]:
            compare(float(reducer(changes)), item[name], 'scene summary/' + name)
    costs = {(r['sequence'], r['mode']): r['cost'] for r in performance}
    for item in summary['paired_cost']:
        pairs = []
        for seq in experiment.EUROC:
            off, on = costs[(seq, 'OFF')], costs[(seq, item['mode'])]
            pairs.append(dict(sequence=seq, **{k: 100 * (on[k] / off[k] - 1)
                                              for k in item['median_percent']}))
        compare(pairs, item['pairs'], 'cost summary/pairs')
        for name, reducer in [('minimum_percent', min), ('median_percent', np.median), ('maximum_percent', max)]:
            for key in item[name]:
                compare(float(reducer([p[key] for p in pairs])), item[name][key], 'cost summary/' + key)
        assert item['camera_p95_over_historical_5pct'] == sum(p['camera_api_p95_ms'] > 5 for p in pairs)
        assert item['rss_over_historical_10pct'] == sum(p['peak_rss_kb'] > 10 for p in pairs)
    windows = captured['local_windows.json']
    baseline = {(r['sequence'], r['bin']): r['ate_rmse_m'] for r in windows if r['mode'] == 'OFF'}
    for item in summary['local_diagnostics']:
        changes = []
        for row in windows:
            if row['mode'] != item['mode']:
                continue
            off = baseline[(row['sequence'], row['bin'])]
            changes.append(dict(sequence=row['sequence'], bin=row['bin'],
                                increase_percent=100 * (row['ate_rmse_m'] / off - 1),
                                increase_m=row['ate_rmse_m'] - off))
        for name, key in [('largest_relative_increase', 'increase_percent'), ('largest_absolute_increase', 'increase_m')]:
            compare(max(changes, key=lambda r: r[key]), item[name], 'local summary/' + name)
    result = dict(timestamp_utc=datetime.now(timezone.utc).isoformat(), status='PASS',
                  accuracy_cases=50, serial_cases=50, trajectory_byte_identical=50,
                  published_files_rehashed=len(recorded), dataset_files_rehashed=input_files,
                  recomputed_metrics=len(captured['metrics.json']),
                  recomputed_local_windows=len(captured['local_windows.json']),
                  effective_profiles_verified=100, opencv_single_thread_verified=100,
                  analysis_summary_recomputed=True,
                  raw_audit=captured['independent_audit.json'],
                  process_check_scope='current /proc namespace only; historical completion manifests also checked',
                  raw_directory=str(args.coord.resolve()), replay_needed=False,
                  qualification='Experiment integrity PASS; measured cost FAIL; full engineering NOT_EVALUATED')
    args.output.write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
