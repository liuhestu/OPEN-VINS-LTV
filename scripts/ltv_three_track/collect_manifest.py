#!/usr/bin/env python3
"""Consolidate authoritative launcher events without turning waits into passes."""
import argparse
import csv
import hashlib
import json
from pathlib import Path


def collect(coord, destination):
    rows = []
    session = json.loads((coord / 'session.json').read_text())
    for task in ('a', 'b', 'c'):
        path = coord / 'registry' / f'task_{task}.jsonl'
        state = {}
        for line in path.read_text().splitlines():
            event = json.loads(line)
            if event['event'] in ('WAITING_RESOURCE', 'START', 'FINISH', 'LAUNCHER_FAILURE', 'CANCELLED_SOURCE_UPDATE'):
                prior = state.get(event['run_id'], {})
                state[event['run_id']] = {**prior, **event}
        for run_id, event in state.items():
            live = False
            if event['event'] == 'START' and event.get('pid'):
                proc = Path(f"/proc/{event['pid']}/stat")
                try:
                    fields = proc.read_text().rsplit(') ', 1)[1].split()
                    identity = event.get('start_identity') or {}
                    live = (int(fields[2]) == event['pgid'] and fields[0] != 'Z' and
                            fields[19] == identity.get('start_ticks'))
                except (OSError, IndexError, ValueError):
                    pass
            row = {k: event.get(k, '') for k in ('label', 'source_oid', 'worktree_head', 'dirty_state',
                   'dirty_patch_sha256', 'pid', 'pgid', 'started_at', 'finished_at', 'exit_code', 'validity',
                   'output_dir', 'diagnostic_level', 'cache_schema', 'spec_path', 'spec_sha256', 'launcher_sha256_at_start')}
            row.update(task=task, run_id=run_id, status=event['event'], verified_live=live,
                       historical_reuse=event.get('historical_reuse', False))
            for key in ('command_expanded', 'binary_paths', 'config_paths', 'data', 'configuration', 'resources',
                        'source_mutation', 'binary_mutation', 'locks', 'start_identity', 'contract_sha256',
                        'source_files_sha256'):
                row[key] = json.dumps(event.get(key), ensure_ascii=False)
            row['baseline_oid'] = session['baseline_oid']
            row['performance'] = event.get('performance', False)
            row['kind'] = event.get('kind', '')
            row['attempt_identity'] = run_id
            row['scientific_verdict'] = 'SEE_TASK_REPORT; EXIT_ZERO_IS_NOT_PASS'
            # Only terminal output is immutable. Do not hash a live estimator's
            # files or imply that waiting commands have executed.
            provenance = {}
            if event['event'] in ('FINISH', 'LAUNCHER_FAILURE', 'CANCELLED_SOURCE_UPDATE') and event.get('output_dir'):
                output = Path(event['output_dir'])
                for name in ('environment.json', 'replay.json', 'instrumentation.json', 'effective_options.json'):
                    artifact = output / name
                    if artifact.is_file():
                        data = artifact.read_bytes()
                        provenance[name] = {'path': str(artifact), 'sha256': hashlib.sha256(data).hexdigest(),
                                            'content': json.loads(data)}
                for artifact in sorted(output.glob('*.ldd.txt')):
                    provenance[artifact.name] = {'path': str(artifact),
                                                'sha256': hashlib.sha256(artifact.read_bytes()).hexdigest()}
            row['terminal_provenance'] = json.dumps(provenance, ensure_ascii=False)
            rows.append(row)
    destination.mkdir(parents=True, exist_ok=True)
    with (destination / 'run_manifest.csv').open('w', newline='') as stream:
        fields = ['task', 'run_id', 'status', *[key for key in rows[0] if key not in ('task', 'run_id', 'status')]]
        writer = csv.DictWriter(stream, fields, lineterminator='\n')
        writer.writeheader()
        writer.writerows(rows)
    summary = {'coord_root': str(coord), 'runs': len(rows), 'states': {key: sum(r['status'] == key for r in rows)
               for key in sorted(set(r['status'] for r in rows))},
               'registry_sha256': {task: hashlib.sha256((coord / 'registry' / f'task_{task}.jsonl').read_bytes()).hexdigest()
                                   for task in ('a', 'b', 'c')},
               'scope': 'execution provenance only; exit zero is not numerical/statistical qualification'}
    (destination / 'execution_summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('coord', type=Path)
    parser.add_argument('destination', type=Path)
    args = parser.parse_args()
    collect(args.coord.resolve(), args.destination.resolve())
