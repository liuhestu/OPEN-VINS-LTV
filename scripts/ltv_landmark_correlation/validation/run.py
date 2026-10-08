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
    write(DOC / 'tests.json', records)


def replay(out):
    frozen = json.loads((ROOT / 'docs/ltv/landmark_validation/freeze.json').read_text())
    sys.path.insert(0, str(ROOT / 'scripts/ltv_integration_cleanup'))
    import check_real_outputs as exact
    records = []
    for seq, inp in frozen['inputs'].items():
        for name, digest in inp['config_sha'].items():
            assert sha(Path(inp['config']).parent / name) == digest
        destination = out / 'runs' / f'{seq}_SHADOW_OFF_01'
        destination.mkdir(parents=True, exist_ok=False)
        record = run(out, seq + '_SHADOW_OFF_01', command(out, 'run_ltv_gv_evaluation',
                     [inp['config'], inp['root'], destination, 'OFF']), cwd=destination,
                     extra_env={'LTV_JOINT_SHADOW_PATH': str(destination / 'joint_shadow.csv')})
        parity = exact.compare(inp['off_path'], destination)
        parity['ltv_rows'] = exact.csv_exact(Path(inp['off_path']) / 'ltv.csv', destination / 'ltv.csv')
        hashes = {}
        for name, digest in inp['off_evidence_sha'].items():
            hashes[name] = sha(destination / name)
            # Exact structured comparison above excludes compute_time_ms only.
            if name != 'features.jsonl':
                assert hashes[name] == digest, (seq, name, hashes[name], digest)
        metadata = json.loads((destination / 'replay.json').read_text())
        with (destination / 'joint_shadow.csv').open() as f:
            rows = list(csv.DictReader(f))
        camera = [row for row in rows if row['event'] == 'camera']
        assert camera and all(row['valid_unconditional'] == 'false' for row in rows)
        summary = {'sequence': seq, 'run': record, 'parity': parity, 'output_sha256': hashes, 'replay': metadata,
                   'shadow_rows': len(rows), 'camera_shadow_rows': len(camera), 'real_statistical_valid_fraction': 0,
                   'missing_blocks': sorted(set(row['missing_blocks'] for row in rows)),
                   'events': {event: sum(row['event'] == event for row in rows) for event in sorted(set(row['event'] for row in rows))},
                   'shadow_path': str(destination / 'joint_shadow.csv'), 'shadow_sha256': sha(destination / 'joint_shadow.csv')}
        write(DOC / f'{seq}_shadow.json', summary)
        records.append(summary)
    write(DOC / 'full_shadow_replays.json', records)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('out', type=Path)
    parser.add_argument('action', choices=['tests', 'replay'])
    args = parser.parse_args()
    {'tests': tests, 'replay': replay}[args.action](args.out.resolve())
