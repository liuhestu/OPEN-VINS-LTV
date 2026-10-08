"""Re-audit frozen causal prediction evidence; never runs or proposes an update."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(ROOT / 'docs/experiments/tools/ltv_landmark_validation'))
from analyze import paired_group, labels, match_times
import numpy as np


def sha(path):
    h = hashlib.sha256()
    with path.open('rb') as f:
        for block in iter(lambda: f.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def audit(source, destination):
    contract_path = destination / 'contract.json'
    contract = json.loads(contract_path.read_text())
    frozen = json.loads((source / 'freeze.json').read_text())
    assert contract['historical_thresholds'] == {
        k: frozen['protocol'][k] for k in contract['historical_thresholds']}
    index = []

    def record(path, expected=None):
        actual = sha(path)
        if expected is not None:
            assert actual == expected, str(path)
        index.append(dict(path=str(path.resolve()), sha256=actual,
                          expected_sha256=expected, status='VERIFIED'))

    result = dict(status='INCONCLUSIVE', causal_prediction='PASS',
                  incremental_information='NOT_DEMONSTRATED',
                  conditional_information_rank='BLOCKED_REAL_JOINT_BLOCKS_UNAVAILABLE',
                  real_landmark_statistical_calibration='NOT_EVALUATED_NO_INDEPENDENT_LANDMARK_GT',
                  fusion_executed=False, sequences={})
    record(contract_path)
    for name in ['freeze.json', 'summary.json', 'correlation_audit.json',
                 'correlation_audit_protocol.json', 'delivery.json', 'compiled_source.json']:
        record(source / name)
    previous = json.loads((source / 'correlation_audit.json').read_text())
    for seq, inp in frozen['inputs'].items():
        old = Path(inp['off_path'])
        for name, expected in inp['off_evidence_sha'].items():
            record(old / name, expected)
        for name, expected in inp['config_sha'].items():
            record(Path(inp['config']).parent / name, expected)
        for item in inp['sensors'].values():
            record(Path(item['path']), item['sha256'])
        features = [json.loads(line) for line in (old / 'features.jsonl').open()]
        phase = labels(features)
        path = source / 'past_geometry_audit' / f'{seq}_predictions.csv'
        record(path, previous['sequences'][seq]['predictions_sha'])
        record(source / f'{seq}_predictions.csv')
        record(source / 'past_geometry_audit' / f'{seq}_exact.json')
        with path.open() as f, (source / f'{seq}_predictions.csv').open() as g:
            rows = list(csv.DictReader(f))
            original = csv.DictReader(g)
            for i, a in enumerate(original):
                assert i < len(rows) and all(a[k] == rows[i][k] for k in a)
            assert i + 1 == len(rows)
        raw = sorted({int(r['camera_ns']) for r in rows})
        mapping = dict(zip(raw, map(int, match_times(raw, sorted(phase)))))
        min_margin = float('inf')
        identities = set()
        for r in rows:
            identity = tuple(r[k] for k in ['camera_ns', 'camera_id', 'feature_id', 'epoch', 'core_id', 'entered'])
            assert identity not in identities, identity
            identities.add(identity)
            r['phase'] = phase[mapping[int(r['camera_ns'])]]
            for key in ['anchor_time', 'past_fit_latest_time']:
                if np.isfinite(float(r[key])):
                    margin = float(r['imu_time']) - float(r[key])
                    assert margin > 1e-9
                    min_margin = min(min_margin, margin)
        mature = np.array([r['past_mature'] == '1' for r in rows])
        phases = np.array([r['phase'] for r in rows])
        # Freeze all three valid on exactly the same rows, preserving candidates
        # outside support in separate coverage output, never silently filtering.
        common = np.array([r['ltv_valid'] == r['main_valid'] == r['past_fit_valid'] == '1'
                           and np.isfinite(float(r['anchor_time'])) for r in rows])
        fit_rows = [dict(r, main_valid=r['past_fit_valid'], main_angle_rad=r['past_fit_angle_rad']) for r in rows]
        groups = {'primary_mature': mature}
        groups.update({p: mature & (phases == p) for p in ['normal', 'recovery', 'startup']})
        metrics = {}
        for group, mask in groups.items():
            metrics[group] = dict(candidates=int(mask.sum()), common_all_three=int((mask & common).sum()),
                                 ltv_vs_historical_geometry_original_support=paired_group(fit_rows, mask),
                                 ltv_vs_historical_geometry_common_three=paired_group(fit_rows, mask & common),
                                 ltv_vs_anchor_common_three=paired_group(rows, mask & common))
        result['sequences'][seq] = dict(rows=len(rows), unique_source_ids=len(identities),
                                        primary_fields_exact=True, minimum_past_margin_s=min_margin,
                                        prediction_split='rolling_one_step_ahead_before_consumption',
                                        prediction_holdout_permanent=False, groups=metrics)
    for name in ['ov_msckf/src/ltv/diagnostics/LtvLandmarkShadow.h',
                 'ov_msckf/src/ltv/observer/LtvAdapter.cpp', 'ov_msckf/src/ltv/observer/LtvAdapterHardened.cpp',
                 'ov_msckf/src/core/VioManager.cpp',
                 'ov_msckf/src/ltv/diagnostics/LtvPassiveCache.cpp',
                 'ov_msckf/tests/ltv/test_ltv_landmark_shadow.cpp',
                 'docs/experiments/tools/ltv_landmark_validation/analyze.py', 'docs/experiments/tools/ltv_landmark_validation/audit.py',
                 'docs/experiments/tools/ltv_landmark_correlation/holdout/run.py']:
        compiled = json.loads((source / 'compiled_source.json').read_text())
        record(ROOT / name, compiled.get(name))
    destination.mkdir(parents=True, exist_ok=True)
    (destination / 'summary.json').write_text(json.dumps(result, indent=2) + '\n')
    with (destination / 'evidence_index.csv').open('w') as f:
        writer = csv.DictWriter(f, fieldnames=['path', 'sha256', 'expected_sha256', 'status'])
        writer.writeheader()
        writer.writerows(index)
    print(json.dumps({seq: {g: dict(n=m['common_all_three'], gain=m['ltv_vs_historical_geometry_common_three']['frame_rms_gain'], ci=m['ltv_vs_historical_geometry_common_three']['gain_ci95']) for g,m in s['groups'].items()} for seq,s in result['sequences'].items()}, indent=2))


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--source', type=Path, default=Path('/home/he/output/ltv_landmark_validation_20261005'))
    p.add_argument('--destination', type=Path, default=ROOT / 'docs/ltv/landmark_correlation/holdout')
    args = p.parse_args()
    audit(args.source.resolve(), args.destination.resolve())
