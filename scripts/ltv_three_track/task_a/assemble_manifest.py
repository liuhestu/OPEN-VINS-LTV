#!/usr/bin/env python3
"""Export every task attempt, including preflight failures and historical scopes."""
import argparse
import csv
import hashlib
import json
from pathlib import Path

p = argparse.ArgumentParser()
p.add_argument('coord', type=Path)
p.add_argument('output', type=Path)
a = p.parse_args()
records = [json.loads(s) for s in (a.coord / 'registry/task_a.jsonl').read_text().splitlines() if s]
groups = {}
for r in records:
    groups.setdefault(r['run_id'], []).append(r)
rows = []
for run_id, events in groups.items():
    first = events[0]
    terminal = [r for r in events if r.get('event') in ('FINISH', 'LAUNCHER_FAILURE')]
    finish = terminal[-1] if terminal else {}
    starts = [r for r in events if r.get('event') == 'START']
    start = starts[-1] if starts else {}
    source = finish if finish.get('source_oid') else start
    if not source:
        spec_path = Path(first.get('spec_path', ''))
        if spec_path.is_file() and hashlib.sha256(spec_path.read_bytes()).hexdigest() == first.get('spec_sha256'):
            source = json.loads(spec_path.read_text())
    path = a.coord / 'tasks/task_a/results' / run_id
    files = {}
    if path.exists():
        for filename in ('manifest_start.json', 'manifest_finish.json', 'environment.json', 'replay.json', 'resources.json',
                         'instrumentation.json', 'effective_options.json', 'trajectory.csv', 'audit.csv', 'algorithm_visual_identity.jsonl', 'lifecycle_identity.jsonl',
                         'observer_identity.jsonl', 'ltv.csv', 'frame_processing.csv', 'checks.json', 'exact.json'):
            f = path / filename
            if f.exists():files[filename] = {'bytes': f.stat().st_size, 'sha256': hashlib.sha256(f.read_bytes()).hexdigest()}
    rows.append({'task': 'a', 'run_id': run_id, 'label': first.get('label'), 'kind': first.get('kind'),
                 'source_oid': source.get('source_oid'), 'source_snapshot': source.get('source_snapshot'),
                 'parent_baseline_oid': '2417d036002fbfd542ae71d96d0fc734cd6d47e8',
                 'binary_and_library_identity': json.dumps(source.get('binary_paths', {})),
                 'config_identity': json.dumps(source.get('config_paths', {})),
                 'source_files_sha256': json.dumps(source.get('source_files_sha256', {})),
                 'contract_sha256': json.dumps(source.get('contract_sha256', {})),
                 'cache_schema': source.get('cache_schema'),
                 'worktree_head': source.get('worktree_head'), 'dirty_state': source.get('dirty_state'),
                 'dirty_patch_sha256': source.get('dirty_patch_sha256'), 'spec_path': first.get('spec_path'),
                 'spec_sha256': first.get('spec_sha256'), 'diagnostic_level': source.get('diagnostic_level'),
                 'performance': first.get('performance'), 'heavy': first.get('heavy'),
                 'pid': start.get('pid'), 'pgid': start.get('pgid'), 'start_identity': json.dumps(start.get('start_identity')),
                 'started_at': start.get('started_at'), 'finished_at': finish.get('finished_at', finish.get('time')),
                 'exit_code': finish.get('exit_code'), 'validity': finish.get('validity', finish.get('event', 'NOT_TERMINAL')),
                 'error': finish.get('error'), 'output_dir': str(path),
                 'configuration': json.dumps(source.get('configuration', {})), 'data': json.dumps(source.get('data', {})),
                 'resources': json.dumps(finish.get('resources')), 'artifacts': json.dumps(files),
                 'environment_identity_path': str(path / 'environment.json') if (path / 'environment.json').exists() else None,
                 'effective_options_path': str(path / 'effective_options.json') if (path / 'effective_options.json').exists() else None,
                 'replay_identity_path': str(path / 'replay.json') if (path / 'replay.json').exists() else None,
                 'determinism_evidence': 'task_a/evidence/repeat_determinism_v203.json; hard exact run artifacts indexed above',
                 'metrics_evidence': 'task_a/evidence/metrics_first_round.csv; per-run mode/sequence in effective/replay metadata',
                 'historical_reuse': False, 'qualification': 'See task report; EXIT_ZERO is execution only'})
a.output.mkdir(parents=True, exist_ok=True)
with (a.output / 'run_manifest.csv').open('w') as f:
    w = csv.DictWriter(f, fieldnames=rows[0].keys())
    w.writeheader()
    w.writerows(rows)
(a.output / 'attempts_index.json').write_text(json.dumps({'runs': len(rows), 'rows': rows}, indent=2) + '\n')
