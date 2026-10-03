"""Six budgeted diagnostic replays only; frozen numerical runtime never rebuilt.

This dispatcher has no automatic retry or overwrite. Each replay counts as a full
real attempt, including failure. Original runs and metrics are read-only.
"""
import argparse
from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import budget
from artifacts import sha
import freeze

RUNS = {
    'V1_01_easy': 'V1_01_easy_P_NEW_2b579f65d564',
    'V2_02_medium': 'V2_02_medium_P_NEW_a19cf59091be',
    'indoor_forward_3': 'indoor_forward_3_P_NEW_a6f5a9d8bc53',
    'V1_03_difficult': 'V1_03_difficult_P_NEW_0ff98f1261ce',
    'V2_03_difficult': 'V2_03_difficult_P_NEW_37b2c47edc88',
    'indoor_forward_6': 'indoor_forward_6_P_NEW_dddf6774f4e2',
}

def main(binary):
    frozen, digest = freeze.check()
    binary = Path(binary).resolve()
    buildproof = binary.parent / ('build_' + binary.name.removeprefix('recover_') + '.json')
    build = json.loads(buildproof.read_text())
    if build['binary_sha256'] != sha(binary) or build['freeze_sha256'] != digest:
        raise ValueError('Recovery binary/build identity mismatch')
    base = budget.OUT / 'diagnostic_recovery' / 'full'
    if base.exists():
        raise FileExistsError('Explicit per-attempt reconciliation required; no automatic restart')
    if budget.usage()['real'] != 30:
        raise ValueError('Expected exactly six remaining full-real slots')
    manifest = json.loads((budget.OUT / 'final_manifest.json').read_text())
    jobs = []
    for seq, directory in RUNS.items():
        run = budget.OUT / 'real_runs' / directory
        identity = json.loads((run / 'identity.json').read_text())
        metric = json.loads(Path(manifest['real_metrics'][seq]).read_text())
        if identity != metric['seed_source_identity'] or identity['runtime'] != frozen['runtime']:
            raise ValueError('Original identity does not match frozen selected runtime')
        family = 'ltv_euroc' if seq.startswith('V') else 'uzhfpv_indoor'
        config = budget.OUT / 'mode_configs/STEREO_THEN_TEMPORAL' / family / 'P_NEW/estimator_config.yaml'
        for name, expected in identity['config_tree_sha'].items():
            if sha(config.parent / name) != expected:
                raise ValueError('Original configuration changed')
        jobs.append((seq, run, config))
    base.mkdir()
    def execute(job):
        seq, run, config = job
        directory = base / seq
        directory.mkdir()
        command = [str(binary), str(config), str(run / 'cache.bin'),
                   str(directory / 'diagnostics.jsonl'), str(directory / 'exact_report.json')]
        identity = {'task': 'diagnostic_recovery_full_cache', 'sequence': seq,
                    'freeze_sha': digest, 'original_run': str(run),
                    'binary_sha': sha(binary), 'runtime': frozen['runtime'],
                    'original_identity_sha': sha(run / 'identity.json')}
        result = budget.run('real', command, json.dumps(identity, sort_keys=True))
        record = dict(identity, attempt=result, command=command)
        if result['exit_code'] == 0:
            record['artifacts_sha'] = {p.name: sha(p) for p in directory.iterdir() if p.is_file()}
            binding = dict(identity, attempt=result, recovered_sha256=sha(directory / 'diagnostics.jsonl'),
                cache_sha256=sha(run / 'cache.bin'), report_sha256=sha(directory / 'exact_report.json'),
                buildproof_sha256=sha(buildproof), buildproof=str(buildproof))
            (directory / 'binding.json').write_text(json.dumps(binding, indent=2) + '\n')
        (directory / 'provenance.json').write_text(json.dumps(record, indent=2) + '\n')
        print(json.dumps({'sequence': seq, **result}), flush=True)
        return record
    with ThreadPoolExecutor(max_workers=2) as pool:
        records = list(pool.map(execute, jobs))
    success = all(r['attempt']['exit_code'] == 0 for r in records)
    (base / 'replays.json').write_text(json.dumps({'status': 'PASS' if success else 'FAIL',
        'freeze_sha': digest, 'records': records, 'budget': budget.usage()}, indent=2) + '\n')
    return 0 if success else 1

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary', required=True)
    raise SystemExit(main(parser.parse_args().binary))
