#!/usr/bin/env python3
"""Independent completion and immutable-input audit; retains findings on failure."""
import argparse
from datetime import datetime
from decimal import Decimal, ROUND_HALF_EVEN
import json
from pathlib import Path
import numpy as np
from study import ROOT, UZH, UZH_TUNE, EUROC, MODES, REP, Study, gate, sha, write


def audit(coord):
    study = Study(coord)
    errors = []
    expected = {(s, m, a) for s in UZH + EUROC for m in MODES for a in ([1] if m == 'OFF' else [1, 2, 4])}
    observed = [(r['sequence'], r['mode'], r['alpha']) for r in study.index]
    if len(observed) != len(set(observed)): errors.append('duplicate configuration')
    if set(observed) != expected: errors.append('missing or unexpected configuration')
    paths = [r['output_dir'] for r in study.index if 'output_dir' in r]
    if len(paths) != len(set(paths)): errors.append('overlapping output directories')
    sessions = json.loads((coord / 'session.json').read_text())['tasks']
    cpus = [cpu for task in sessions.values() for cpu in task['cpu_affinity']]
    if len(cpus) != 16 or len(set(cpus)) != 16: errors.append('overlapping worker CPUs')
    if len({v['ros_domain_id'] for v in sessions.values()}) != 4: errors.append('ROS domains not isolated')
    build = json.loads((Path(study.identity['artifact']) / 'runner_build_identity.json').read_text())
    for library in build['libraries'].values():
        if sha(library['new_path']) != library['sha256']: errors.append('dependency library changed ' + library['new_path'])
    for entry in build['source_closure']:
        if sha(Path(study.identity['source']) / entry['path']) != entry['sha256']: errors.append('frozen source changed ' + entry['path'])
    for seq, identity in study.inputs.items():
        if sha(identity['gt']) != identity['gt_sha256']: errors.append('GT changed ' + seq)
        manifest = Path(identity['sensor_hash_manifest'])
        if sha(manifest) != identity['sensor_hash_manifest_sha256']: errors.append('input manifest changed ' + seq)
        for relative, digest in json.loads(manifest.read_text()).items():
            if sha(Path(identity['root']) / relative) != digest: errors.append('input changed ' + seq + '/' + relative)
    for seq in UZH:
        item = study.inputs[seq]
        original, sensor = Path(item['original']), Path(item['root'])
        for camera, side in enumerate(('left', 'right')):
            expected_rows = []
            for line in (original / f'{side}_images.txt').read_text().splitlines():
                if not line or line.startswith('#'): continue
                _, stamp, name = line.split()
                ns = int((Decimal(stamp) * 10**9).to_integral_value(rounding=ROUND_HALF_EVEN))
                expected_rows.append(f'{ns},{Path(name).name}')
            actual_rows = [line for line in (sensor / f'cam{camera}/data.csv').read_text().splitlines() if line and not line.startswith('#')]
            if actual_rows != expected_rows: errors.append('camera conversion mismatch ' + seq)
        expected_rows = []
        for line in (original / 'imu.txt').read_text().splitlines():
            if not line or line.startswith('#'): continue
            _, stamp, *values = line.split()
            ns = int((Decimal(stamp) * 10**9).to_integral_value(rounding=ROUND_HALF_EVEN))
            expected_rows.append(','.join([str(ns), *values]))
        actual_rows = [line for line in (sensor / 'imu0/data.csv').read_text().splitlines() if line and not line.startswith('#')]
        if actual_rows != expected_rows: errors.append('IMU conversion mismatch ' + seq)
    for row in study.index:
        if 'output_dir' not in row: continue
        output = Path(row['output_dir'])
        manifest = json.loads((output / 'manifest_finish.json').read_text())
        if manifest['source_mutation'] or manifest['binary_mutation']: errors.append('artifact mutation ' + row['run_id'])
        for category in ['binary_paths', 'config_paths']:
            for path, item in manifest[category].items():
                if sha(path) != item['sha256']: errors.append(category + ' changed ' + path)
        if row['status'] == 'VALID_FULL_MATCHED':
            check = study.evaluate(row)
            if check['status'] != row['status'] or not np.isclose(check['ate_rmse_m'], row['ate_rmse_m'], rtol=1e-12, atol=0):
                errors.append('evaluation mismatch ' + row['run_id'])
    for screen in study.screening:
        rows = []
        for seq in REP:
            row = dict(study.lookup(seq, screen['mode'], screen['alpha']))
            row['off_ate'] = (study.lookup(seq, 'OFF') or {}).get('ate_rmse_m')
            row['reference_ate'] = (study.lookup(seq, screen['mode']) or {}).get('ate_rmse_m')
            rows.append(row)
        if gate(rows)['status'] != screen['status']: errors.append('screen mismatch')
        for seq in [s for s in EUROC + UZH if s not in REP]:
            row = study.lookup(seq, screen['mode'], screen['alpha'])
            if screen['status'] == 'BLOCKED' and (row['status'] != 'BLOCKED' or 'run_id' in row): errors.append('blocked configuration launched')
    manifests = [json.loads(path.read_text()) for path in coord.glob('w*/results/*/manifest_finish.json')]
    replay = [m for m in manifests if m['kind'] == 'replay']
    timeline = sorted([(m['started_at'], 1) for m in replay] + [(m['finished_at'], -1) for m in replay])
    active, peak = 0, 0
    for _, change in timeline:
        active += change
        peak = max(peak, active)
    if peak > 4: errors.append('more than four concurrent replays')
    for build in [m for m in manifests if m['kind'] == 'build']:
        if any(r['started_at'] < build['finished_at'] and build['started_at'] < r['finished_at'] for r in replay):
            errors.append('build overlapped replay')
    amendment = json.loads((coord / 'scope_amendment.json').read_text()) if (coord / 'scope_amendment.json').exists() else {}
    excluded = set(amendment.get('excluded_from_tuning', sorted(set(UZH) - set(UZH_TUNE))))
    if excluded:
        cutoff = datetime.fromisoformat(amendment['recorded_at_utc']).timestamp() if amendment else 0
        for item in replay:
            if item.get('configuration', {}).get('sequence') in excluded and item['started_at'] >= cutoff:
                errors.append('excluded sequence launched after user amendment')
        for row in study.index:
            if row['sequence'] in excluded and any(row.get('audit', {}).get('actual_' + branch, 0) for branch in ('G', 'V', 'L')):
                errors.append('exclusion no-effect premise does not hold')
    status_counts = {s: sum(r['status'] == s for r in study.index) for s in sorted({r['status'] for r in study.index})}
    result = dict(status='PASS' if not errors else 'FAIL', errors=errors, planned=len(expected), recorded=len(observed),
                  status_counts=status_counts, input_hashes_reverified=True, sensor_conversion_reverified=True, attempts=len(manifests),
                  excluded_from_tuning=sorted(excluded), active_configurations=sum(r['sequence'] not in excluded for r in study.index),
                  maximum_concurrent_replays=peak, failed_attempts=[dict(run_id=m['run_id'], exit_code=m['exit_code']) for m in manifests if m['exit_code']],
                  maximum_horn_svd_difference_m=max(((r.get('svd_horn_difference_m') or 0) for r in study.index), default=0),
                  report_scope='screening only; historical engineering conclusions unchanged')
    write(coord / 'independent_audit.json', result)
    print(json.dumps(result, ensure_ascii=False))
    if errors: raise SystemExit(1)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('coord', type=Path)
    audit(parser.parse_args().coord)
