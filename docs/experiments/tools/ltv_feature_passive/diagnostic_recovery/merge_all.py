"""Offline merge of six exact diagnostic recoveries, with immutable old metrics."""
from pathlib import Path
import json
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import budget
from artifacts import sha
import freeze
from recover_all import RUNS


def main():
    _, digest = freeze.check()
    source = budget.OUT / 'final_manifest.json'
    manifest = json.loads(source.read_text())
    full = budget.OUT / 'diagnostic_recovery/full'
    replays = json.loads((full / 'replays.json').read_text())
    if replays['status'] != 'PASS' or len(replays['records']) != 6:
        raise ValueError('Requires all six complete exact replays')
    records = []
    for sequence, name in RUNS.items():
        directory = full / sequence
        output = directory / 'overlay'
        command = [sys.executable, str(Path(__file__).with_name('merge_overlay.py')),
                   '--run', str(budget.OUT / 'real_runs' / name),
                   '--metrics', manifest['real_metrics'][sequence],
                   '--recovered', str(directory / 'diagnostics.jsonl'),
                   '--provenance', str(directory / 'exact_report.json'),
                   '--binding', str(directory / 'binding.json'), '--out', str(output)]
        result = budget.run('analysis', command, 'offline_management_overlay_' + sequence)
        records.append(dict(sequence=sequence, attempt=result, output=str(output)))
        print(json.dumps(records[-1]), flush=True)
        if result['exit_code']:
            raise RuntimeError('Overlay failed; no implicit overwrite or retry')
        manifest['real_metrics'][sequence] = str(output / 'metrics.json')
    manifest['diagnostic_recovery'] = {
        'original_manifest': str(source), 'original_manifest_sha256': sha(source),
        'replays': str(full / 'replays.json'), 'replays_sha256': sha(full / 'replays.json'),
        'overlay_attempts': records, 'numerical_freeze_unchanged': digest,
        'scientific_raw_ready_reference_errors_preserved': True}
    target = budget.OUT / 'final_manifest_recovered.json'
    if target.exists():
        raise FileExistsError(target)
    target.write_text(json.dumps(manifest, indent=2) + '\n')

if __name__ == '__main__':
    main()
