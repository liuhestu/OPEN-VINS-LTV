#!/usr/bin/env python3
"""Resume the frozen 11 x 5 matrix, serially. Stop at the first failed run for diagnosis."""
import json
import subprocess
import sys
from run_sequence import OUT, ROOT, execution_identity, write


def main():
    experiment = json.loads((OUT / 'experiment_manifest.json').read_text())
    assert experiment['frozen'] and experiment['execution_identity'] == execution_identity()
    for sequence in experiment['final_sequences']:
        for mode in experiment['modes']:
            latest = {r['run_id']: r for r in map(json.loads, (OUT / 'runs.jsonl').read_text().splitlines())}
            matches = [r for r in latest.values() if r['stage'] == 'final' and r['sequence'] == sequence and r['mode'] == mode]
            complete = [r for r in matches if r['status'] == 'complete' and r.get('frozen') and
                        r['binary_sha256'] == experiment['execution_identity']['binary_sha256'] and
                        r['source_sha256'] == experiment['execution_identity']['source_sha256']]
            if complete:
                record = complete[-1]
                print('reuse', sequence, mode, record['run_id'], flush=True)
            else:
                if matches:
                    raise SystemExit('Prior incomplete/failed final run needs diagnosis: ' + matches[-1]['path'])
                log = OUT / ('final_' + sequence + '_' + mode + '_runner.log')
                command = [sys.executable, str(ROOT / 'docs/experiments/tools/ltv/run_sequence.py'), sequence, mode, '--stage', 'final']
                with log.open('w') as output:
                    result = subprocess.run(command, stdout=output, stderr=subprocess.STDOUT)
                if result.returncode:
                    raise SystemExit('Replay failed; retained ' + str(log))
                record = json.loads(log.read_text().splitlines()[-1])
            audit_command = [sys.executable, str(ROOT / 'docs/experiments/tools/ltv/audit_run.py'), record['path'],
                             '--output', str(OUT / ('final_' + sequence + '_' + mode + '_audit.json'))]
            checked = subprocess.run(audit_command, stdout=subprocess.DEVNULL)
            if checked.returncode:
                raise SystemExit('Diagnostics failed: ' + record['path'])
            print('complete', sequence, mode, record['run_id'], flush=True)
    latest = {r['run_id']: r for r in map(json.loads, (OUT / 'runs.jsonl').read_text().splitlines())}
    paths = {}
    for sequence in experiment['final_sequences']:
        paths[sequence] = {mode: next(r['path'] for r in reversed(list(latest.values()))
                                     if r['stage'] == 'final' and r['sequence'] == sequence and r['mode'] == mode and r['status'] == 'complete')
                           for mode in experiment['modes']}
    write(OUT / 'final_paths.json', paths)
    print('Final 55-run matrix complete; evaluate common support next.', flush=True)


if __name__ == '__main__':
    main()
