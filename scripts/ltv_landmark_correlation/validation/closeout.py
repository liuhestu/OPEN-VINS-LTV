#!/usr/bin/env python3
"""Consolidate only completed, checked evidence; refuse premature closeout."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[3]
DOC = ROOT / 'docs/ltv/nondegradation_validation'


def sha(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def write_csv(path, rows):
    fields = list(dict.fromkeys(k for row in rows for k in row))
    with path.open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fields)
        writer.writeheader()
        writer.writerows(rows)


def closeout(out):
    tests = json.loads((DOC / 'tests.json').read_text())
    assert len(tests) == 31 and all(row['exit_code'] == 0 for row in tests)
    replays = json.loads((DOC / 'full_shadow_replays.json').read_text())
    assert len(replays) == 6
    assert all(row['run']['exit_code'] == 0 and row['replay']['complete'] and
               row['parity']['status'] == 'PASS_EXACT_NATIVE_PASSIVE' for row in replays)
    shadow_audits = {}
    for seq in ('V2_02_medium', 'V2_03_difficult'):
        path = DOC / f'{seq}_matrix_audit.json'
        audit = json.loads(path.read_text())
        assert audit['matrix_reference']['max_relative_identity_error'] <= 1e-11
        assert audit['matrix_reference']['nonzero_cross_time_camera_events'] > 0
        assert audit['full_model_valid_constraints'] == 0
        shadow_audits[seq] = audit
    matrix_rows = []
    for key, value in json.loads((ROOT / 'docs/ltv/landmark_correlation/gv/completeness.json').read_text()).items():
        seq, mode = key.split('/')
        matrix_rows.append({'sequence': seq, 'mode': mode, 'kind': 'historical_real_reuse', 'status': 'VERIFIED_COMPLETE',
                            'commit': '16f02e30a4ba8308e112eb2c96d30046672007ec', 'camera_packets': value['camera_packets'],
                            'source': 'docs/ltv/landmark_correlation/gv/completeness.json'})
    ledger = [json.loads(line) for line in (out / 'attempts.jsonl').read_text().splitlines()]
    for row in ledger:
        if row['event'] == 'finish':
            matrix_rows.append({'sequence': row['label'].split('_SHADOW')[0].split('_OFF')[0] if row['label'].startswith('V2_') else '',
                                'mode': row['label'], 'kind': 'new_execution', 'status': 'EXIT_ZERO' if row['exit_code'] == 0 else 'FAILED_RETAINED',
                                'exit_code': row['exit_code'], 'seconds': row['seconds'], 'commit': row['commit'],
                                'source': row.get('log_path', str(out / (row['label'] + '.log')))})
    for label in ('initial', 'integrated', 'final', 'clean'):
        path = out / f'build_{label}.log'
        assert path.exists()
        text = path.read_text()
        assert 'Summary:' in text and 'finished' in text.split('Summary:')[-1]
        assert 'failed' not in text.split('Summary:')[-1].lower()
        matrix_rows.append({'sequence': '', 'mode': f'build_{label}', 'kind': 'build', 'status': 'COMPLETED',
                            'source': str(path), 'sha256': sha(path), 'scope': 'clean is final accepted package build; initial/incremental artifacts excluded'})
    write_csv(DOC / 'run_matrix.csv', matrix_rows)
    index = []
    for row in json.loads((ROOT / 'docs/ltv/landmark_correlation/gv/raw_identity.json').read_text()):
        index.append({'kind': 'gv_history', 'path': row['path'], 'sha256': row['actual']['sha256'], 'status': row['status']})
    with (ROOT / 'docs/ltv/landmark_correlation/holdout/evidence_index.csv').open() as stream:
        for row in csv.DictReader(stream):
            index.append({'kind': 'holdout_history', **row})
    for row in replays:
        for name, digest in row['output_sha256'].items():
            index.append({'kind': 'new_real', 'path': str(Path(row['run']['cwd']) / name),
                          'sha256': digest, 'status': 'EXACT_FIELDS_ONLY' if name == 'features.jsonl' else 'EXACT_BYTES'})
        index.append({'kind': 'new_real_canonical_features', 'path': str(Path(row['run']['cwd']) / 'features.jsonl'),
                      'sha256': row['parity']['features']['canonical_sha'], 'status': 'EXACT_CANONICAL_JSON',
                      'hash_semantics': 'sorted JSON; only compute_time_ms excluded'})
    for path in sorted(out.glob('*.log')):
        index.append({'kind': 'new_log', 'path': str(path), 'sha256': sha(path), 'status': 'RETAINED'})
    for path in sorted(out.iterdir()):
        if path.is_file() and path.suffix != '.log':
            index.append({'kind': 'new_raw_or_driver', 'path': str(path), 'sha256': sha(path), 'status': 'RETAINED'})
    for directory in (out / 'integrated_observer_mc', out / 'reference_recheck', out / 'failed_mixed_artifacts'):
        for path in sorted(directory.iterdir()):
            index.append({'kind': directory.name, 'path': str(path), 'sha256': sha(path), 'status': 'RETAINED'})
    for directory in (out / 'runs').iterdir():
        for suffix in ('joint_shadow.csv', 'joint_shadow.csv.matrices.jsonl', 'joint_shadow.csv.matrices.bin',
                       'shadow_receipts.jsonl', 'frame_processing.csv'):
            path = directory / suffix
            if path.exists():
                index.append({'kind': 'new_shadow_source', 'path': str(path), 'sha256': sha(path), 'status': 'RECORDED'})
    write_csv(DOC / 'evidence_index.csv', index)
    shutil.copyfile(ROOT / 'docs/ltv/landmark_correlation/gv/metrics.csv', DOC / 'metrics.csv')
    manifest = json.loads((DOC / 'manifest.json').read_text())
    for name, expected in manifest['source_identity_before_clean_build']['source_sha256'].items():
        assert sha(ROOT / name) == expected, ('post-freeze source changed', name)
    manifest['tested_commit'] = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip()
    manifest['clean_build'] = {'command': (out / 'build_clean.sh').read_text(), 'log': str(out / 'build_clean.log'),
                               'log_sha256': sha(out / 'build_clean.log'), 'exit_code': 0,
                               'compile_commands_sha256': sha(out / 'build/ov_msckf/compile_commands.json')}
    binaries = [out / 'install/ov_msckf/lib/libov_msckf_lib.so', out / 'install/ov_msckf/lib/ov_msckf/run_ltv_gv_evaluation',
                out / 'install/ov_msckf/lib/ov_msckf/test_ltv_error_shadow', out / 'install/ov_msckf/lib/ov_msckf/test_ltv_seed_joint',
                out / 'install/ov_msckf/lib/ov_msckf/test_ltv_observer_mc']
    manifest['binary_sha256'] = {str(p): sha(p) for p in binaries}
    manifest['evaluator_sha256'] = {str(p.relative_to(ROOT)): sha(p) for p in (ROOT / 'scripts/ltv_landmark_correlation').rglob('*.py')}
    manifest['raw_retention'] = 'large logs/matrices/CSVs external output or ignored results; indexes versioned'
    (DOC / 'manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + '\n')
    failures = json.loads((DOC / 'development_failures.json').read_text())
    failures['verification'] = 'clean build and 31 integrated executable/comparison records all EXIT_ZERO; full two-sequence OFF/shadow exact regression'
    (DOC / 'development_failures.json').write_text(json.dumps(failures, indent=2) + '\n')
    print(json.dumps({'integrated_records': len(tests), 'complete_real_runs': len(replays), 'evidence_entries': len(index)}, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('out', type=Path)
    args = parser.parse_args()
    closeout(args.out.resolve())
