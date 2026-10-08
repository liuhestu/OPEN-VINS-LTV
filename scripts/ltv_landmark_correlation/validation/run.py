#!/usr/bin/env python3
"""Append-only integrated checks and sequential matched real OFF/shadow replays."""
import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[3]
DOC = ROOT / 'docs/ltv/nondegradation_validation'
ENV = {k: v for k, v in os.environ.items() if k not in ('LD_LIBRARY_PATH', 'PYTHONPATH', 'AMENT_PREFIX_PATH', 'CMAKE_PREFIX_PATH', 'COLCON_PREFIX_PATH')}
ENV.update(OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1', MKL_NUM_THREADS='1', NUMEXPR_NUM_THREADS='1')


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def write(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')


def command(out, name, arguments):
    return ['bash', '--noprofile', '--norc', '-c',
            'source /opt/ros/humble/setup.bash && source "$1/setup.bash" && exec "$2" "${@:3}"',
            'correlation', str(out / 'install'), str(out / 'install/ov_msckf/lib/ov_msckf' / name), *map(str, arguments)]


def run(out, label, cmd, cwd=ROOT, extra_env=None):
    ledger = out / 'attempts.jsonl'
    entry = {'event': 'start', 'label': label, 'command': cmd, 'cwd': str(cwd), 'started': time.time(),
             'commit': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
             'extra_env': extra_env or {}}
    with ledger.open('a') as f:
        f.write(json.dumps(entry) + '\n')
    resources = out / f'{label}_resources.json'
    timed = ['/usr/bin/time', '-q', '-f', '{"max_rss_kb":%M,"elapsed_s":%e,"user_s":%U,"system_s":%S}', '-o', str(resources), *cmd]
    with (out / f'{label}.log').open('w') as f:
        result = subprocess.run(timed, cwd=cwd, env={**ENV, **(extra_env or {})}, stdout=f, stderr=subprocess.STDOUT)
    entry.update(event='finish', exit_code=result.returncode, seconds=time.time() - entry['started'], resources=json.loads(resources.read_text()))
    with ledger.open('a') as f:
        f.write(json.dumps(entry) + '\n')
    print(label, result.returncode, entry['seconds'], flush=True)
    if result.returncode:
        raise RuntimeError(f'{label} failed; evidence retained')
    return entry


def tests(out):
    historical = json.loads((ROOT / 'docs/ltv/landmark_validation/tests.json').read_text())
    records = []
    for entry in historical:
        name = entry['name']
        # Legacy command tail is authoritative for fixtures and arguments.
        args = entry['command'][8:]
        records.append(run(out, name, command(out, name, args)))
    for name in ('test_ltv_error_shadow', 'test_ltv_seed_joint', 'test_ltv_landmark_shadow'):
        args = [out / 'integrated_actual_seed_samples.csv'] if name == 'test_ltv_seed_joint' else []
        records.append(run(out, name, command(out, name, args)))
    fixture = ROOT / 'docs/ltv_handover/parity_fixture'
    records.append(run(out, 'core_golden', command(out, 'test_ltv_core_parity',
                       [fixture / 'inputs.jsonl', out / 'core_golden_matrices.jsonl'])))
    for actual, expected, label in [(out / 'core_golden.log', fixture / 'expected_outputs.jsonl', 'golden_means'),
                                    (out / 'core_golden_matrices.jsonl', fixture / 'expected_matrices.jsonl', 'golden_full_matrices')]:
        records.append(run(out, label, ['python3', str(fixture / 'compare.py'), str(actual),
                           '--expected', str(expected), '--atol', '0', '--rtol', '0']))
    write(DOC / 'tests.json', records)


def replay(out):
    import numpy as np
    frozen = json.loads((ROOT / 'docs/ltv/landmark_validation/freeze.json').read_text())
    sys.path.insert(0, str(ROOT / 'scripts/ltv_integration_cleanup'))
    import check_real_outputs as exact
    records = []
    for seq, inp in frozen['inputs'].items():
        for name, digest in inp['config_sha'].items():
            assert sha(Path(inp['config']).parent / name) == digest
        paired = []
        for mode in ('OFF', 'SHADOW_OFF'):
            destination = out / 'runs' / f'{seq}_{mode}_01'
            destination.mkdir(parents=True, exist_ok=False)
            extra = {'LTV_JOINT_SHADOW_PATH': str(destination / 'joint_shadow.csv')} if mode == 'SHADOW_OFF' else {}
            record = run(out, seq + '_' + mode + '_01', command(out, 'run_ltv_gv_evaluation',
                         [inp['config'], inp['root'], destination, 'OFF']), cwd=destination, extra_env=extra)
            parity = exact.compare(inp['off_path'], destination)
            parity['ltv_rows'] = exact.csv_exact(Path(inp['off_path']) / 'ltv.csv', destination / 'ltv.csv')
            hashes = {}
            for name, digest in inp['off_evidence_sha'].items():
                hashes[name] = sha(destination / name)
                # Only features.compute_time_ms is excluded by exact structured comparison.
                if name != 'features.jsonl':
                    assert hashes[name] == digest, (seq, name, hashes[name], digest)
            metadata = json.loads((destination / 'replay.json').read_text())
            options = json.loads((destination / 'effective_options.json').read_text())
            assert options['max_slam_features'] == 0
            assert options['ltv_observer_warmup_camera_updates'] == 20
            assert not options['ltv_enable_gravity'] and not options['ltv_enable_velocity']
            timing = np.genfromtxt(destination / 'frame_processing.csv', delimiter=',', names=True)
            assert len(timing) == metadata['camera_packets']
            assert np.all(np.isfinite(timing['feed_camera_ms']))
            summary = {'sequence': seq, 'mode': mode, 'run': record, 'parity': parity, 'output_sha256': hashes,
                       'replay': metadata, 'frame_feed_camera_ms_p95': float(np.percentile(timing['feed_camera_ms'], 95)),
                       'frame_feed_camera_ms_max': float(np.max(timing['feed_camera_ms']))}
            if mode == 'SHADOW_OFF':
                with (destination / 'joint_shadow.csv').open() as f:
                    rows = list(csv.DictReader(f))
                camera = [row for row in rows if row['event'] == 'camera']
                assert camera and all(row['valid_unconditional'] == 'false' for row in rows)
                summary.update(shadow_rows=len(rows), camera_shadow_rows=len(camera), real_statistical_valid_fraction=0,
                               missing_blocks=sorted(set(row['missing_blocks'] for row in rows)),
                               events={event: sum(row['event'] == event for row in rows) for event in sorted(set(row['event'] for row in rows))},
                               shadow_path=str(destination / 'joint_shadow.csv'), shadow_sha256=sha(destination / 'joint_shadow.csv'))
            write(DOC / f'{seq}_{mode}.json', summary)
            paired.append(summary)
            records.append(summary)
        shadow, baseline = paired[1], paired[0]
        resources = {'sequence': seq, 'scope': 'heavy diagnostic SHADOW vs same-binary OFF, feed_camera includes synchronous shadow I/O',
                     'p95_relative_growth': shadow['frame_feed_camera_ms_p95'] / baseline['frame_feed_camera_ms_p95'] - 1,
                     'peak_rss_relative_growth': shadow['run']['resources']['max_rss_kb'] / baseline['run']['resources']['max_rss_kb'] - 1,
                     'production_GV_performance': 'NOT_EVALUATED', 'ROS_transport_queues': 'NOT_EVALUATED; direct deterministic ASL runner'}
        write(DOC / f'{seq}_shadow_overhead.json', resources)
    write(DOC / 'full_shadow_replays.json', records)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('out', type=Path)
    parser.add_argument('action', choices=['tests', 'replay'])
    args = parser.parse_args()
    {'tests': tests, 'replay': replay}[args.action](args.out.resolve())
