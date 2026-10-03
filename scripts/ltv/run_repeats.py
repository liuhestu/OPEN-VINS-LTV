#!/usr/bin/env python3
"""One required frozen repeat per selected sequence and B/G/V/GV, preserving every run."""
import json
import subprocess
import sys
from run_sequence import OUT, ROOT, execution_identity, write


def main():
    experiment = json.loads((OUT / 'experiment_manifest.json').read_text())
    assert experiment['frozen'] and experiment['execution_identity'] == execution_identity()
    paths = {}
    for sequence in json.loads((OUT / 'repeat_sequences.json').read_text()):
        paths[sequence] = {}
        for mode in ['B', 'G', 'V', 'GV']:
            latest = {r['run_id']: r for r in map(json.loads, (OUT / 'runs.jsonl').read_text().splitlines())}
            records = [r for r in latest.values() if r['sequence'] == sequence and r['mode'] == mode and r['stage'] == 'repeat']
            if records:
                record = records[-1]
                assert record['status'] == 'complete' and record['source_sha256'] == experiment['execution_identity']['source_sha256']
            else:
                log = OUT / ('repeat_' + sequence + '_' + mode + '_runner.log')
                with log.open('w') as output:
                    process = subprocess.run([sys.executable, str(ROOT / 'scripts/ltv/run_sequence.py'), sequence, mode, '--stage', 'repeat'],
                                             stdout=output, stderr=subprocess.STDOUT)
                assert process.returncode == 0, str(log)
                record = json.loads(log.read_text().splitlines()[-1])
            paths[sequence][mode] = record['path']
            audit = subprocess.run([sys.executable, str(ROOT / 'scripts/ltv/audit_run.py'), record['path'], '--output',
                                    str(OUT / ('repeat_' + sequence + '_' + mode + '_audit.json'))], stdout=subprocess.DEVNULL)
            assert audit.returncode == 0
            print('repeat complete', sequence, mode, flush=True)
        write(OUT / 'repeat_paths.json', paths)


if __name__ == '__main__':
    main()
