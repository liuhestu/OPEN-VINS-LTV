"""Frozen causal gates against the strongest historical-bearing geometry baseline.

Run through the session launcher. Sources remain read-only. No GT is consumed.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import sys
import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'scripts/ltv_landmark_validation'))
from analyze import paired_group, labels, match_times, angular_stats


def sha(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for b in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(b)
    return h.hexdigest()


def assess(source, contract_path, destination):
    contract = json.loads(contract_path.read_text())
    destination.mkdir(parents=True, exist_ok=False)
    freeze = json.loads((source / 'freeze.json').read_text())
    historical = json.loads((ROOT / 'docs/ltv/landmark_validation/correlation_audit.json').read_text())
    results = {'source_kind': 'HISTORICAL_REUSED', 'contract_sha256': sha(contract_path),
               'causality': 'before_current_bearing_consumption; no GT; past readiness and maturity',
               'correlation_model': 'approximate_not_implemented', 'fusion_executed': False,
               'analysis_script_sha256': sha(Path(__file__)), 'inputs': [], 'sequences': {}}
    compiled = json.loads((source / 'compiled_source.json').read_text())
    for rel in ['ov_msckf/src/ltv/diagnostics/LtvLandmarkShadow.h', 'ov_msckf/src/ltv/observer/LtvAdapterHardened.cpp', 'ov_msckf/src/core/VioManager.cpp']:
        assert sha(ROOT / rel) == compiled[rel], 'prediction source mismatch: ' + rel
        results['inputs'].append(dict(path=str(ROOT / rel), sha256=sha(ROOT / rel)))
    for seq, inp in freeze['inputs'].items():
        path = source / 'past_geometry_audit' / (seq + '_predictions.csv')
        expected = historical['sequences'][seq]['predictions_sha']
        actual = sha(path)
        assert actual == expected, (path, actual, expected)
        feature = Path(inp['off_path']) / 'features.jsonl'
        assert sha(feature) == inp['off_evidence_sha']['features.jsonl']
        cache = Path(inp['off_path']) / 'cache.bin'
        assert sha(cache) == inp['off_evidence_sha']['cache.bin']
        results['inputs'].append(dict(path=str(cache), sha256=sha(cache)))
        results['inputs'].extend([dict(path=str(path), sha256=actual),
                                 dict(path=str(feature), sha256=sha(feature))])
        phases = labels([json.loads(x) for x in feature.open()])
        with path.open() as stream:
            rows = list(csv.DictReader(stream))
        times = sorted({int(r['camera_ns']) for r in rows})
        mapping = dict(zip(times, map(int, match_times(times, sorted(phases)))))
        identities = [tuple(r[k] for k in contract['primary_paired_fields']) for r in rows]
        assert len(identities) == len(set(identities)), 'duplicate prediction identity'
        for r in rows:
            if np.isfinite(float(r['anchor_time'])):
                assert float(r['anchor_time']) < float(r['imu_time']) - 1e-9
            if np.isfinite(float(r['past_fit_latest_time'])):
                assert float(r['past_fit_latest_time']) < float(r['imu_time']) - 1e-9
        phase = np.array([phases[mapping[int(r['camera_ns'])]] for r in rows])
        mature = np.array([r['past_mature'] == '1' for r in rows])
        ready = np.array([r['past_ready_V'] == '1' for r in rows])
        span = np.array([float(r['anchor_span']) for r in rows])
        ltv = np.array([[float(r['ltv_' + d]) for d in 'xyz'] for r in rows])
        main = np.array([[float(r['main_' + d]) for d in 'xyz'] for r in rows])
        disagreement = np.linalg.norm(ltv - main, axis=1) / np.maximum(np.linalg.norm(main, axis=1), .1)
        common = np.array([r['ltv_valid'] == r['main_valid'] == r['past_fit_valid'] == '1' for r in rows])
        strong = [dict(r, main_valid=r['past_fit_valid'], main_angle_rad=r['past_fit_angle_rad']) for r in rows]
        gates = {'B0': mature & ready,
                 'B1': mature & ready & (span >= .2 - 1e-9) & (span <= .5 + 1e-9),
                 'B2': mature & ready & (span >= .2 - 1e-9) & (span <= .5 + 1e-9) & (disagreement <= .02)}
        assert list(gates) == contract['candidate_ids']
        seq_out = {'rows': len(rows), 'ungated_all_candidates': paired_group(strong, np.ones(len(rows), bool)),
                   'ungated_mature': paired_group(strong, mature), 'gates': {}, 'nominal_mature_wait_s': freeze['protocol'].get('maturity_seconds', .1)}
        survival = {}
        for r in rows:
            if r['core_id'] != '-1':
                key = tuple(r[k] for k in ['feature_id', 'epoch', 'core_id', 'entered'])
                t = float(r['imu_time'])
                lo, hi = survival.get(key, (t, t))
                survival[key] = (min(lo, t), max(hi, t))
        seq_out['observable_identity_span_s'] = angular_stats([hi-lo for lo, hi in survival.values()])
        seq_out['observable_identity_span_s'].pop('above_0_02_rad', None)
        seq_out['observable_identity_span_scope'] = 'offline descriptive; truncated observation span, never gate input'
        for gid, mask in gates.items():
            metrics = paired_group(strong, mask)
            triple_metrics = paired_group(strong, mask & common)
            candidate_n = int(mask.sum())
            mature_n = int(mature.sum())
            paired_n = metrics['paired']
            coverage = candidate_n / mature_n if mature_n else 0
            common_coverage = paired_n / candidate_n if candidate_n else 0
            all_available = {k: angular_stats([float(r[field]) for r, keep in zip(rows, mask)
                                              if keep and r[valid] == '1'])
                             for k, field, valid in [('ltv', 'ltv_angle_rad', 'ltv_valid'),
                                                     ('anchor', 'main_angle_rad', 'main_valid'),
                                                     ('geometry', 'past_fit_angle_rad', 'past_fit_valid')]}
            checks = {'reliable_advantage': metrics['reliable_advantage'],
                      'mature_coverage': coverage >= contract['min_mature_coverage'],
                      'common_coverage': common_coverage >= contract['min_common_coverage']}
            if paired_n:
                checks['bad_fraction_nonworse'] = (metrics['ltv']['above_0_02_rad'] <=
                                                   contract['tail_ratio'] * metrics['main']['above_0_02_rad'])
            else:
                checks['bad_fraction_nonworse'] = False
            subgroups = {}
            for p in ['normal', 'recovery', 'startup']:
                v = paired_group(strong, mask & (phase == p))
                subgroups[p] = v
                if p in contract['required_tail_subgroups']:
                    sufficient = v.get('paired_frames', 0) >= 30 and v.get('time_blocks', 0) >= 10
                    checks[p + '_tail_nonworse'] = sufficient and v.get('tail_nonworse', False)
                    checks[p + '_bad_fraction_nonworse'] = sufficient and (v['ltv']['above_0_02_rad'] <= contract['tail_ratio'] * v['main']['above_0_02_rad'])
            seq_out['gates'][gid] = {'candidate_denominator': candidate_n, 'common_denominator': paired_n,
                                     'mature_denominator': mature_n, 'mature_coverage': coverage,
                                     'common_coverage': common_coverage, 'common_gated': metrics,
                                     'common_all_three_gated': triple_metrics, 'all_available_gated': all_available, 'subgroups': subgroups,
                                     'checks': checks, 'status': 'PASS' if all(checks.values()) else 'FAIL'}
        results['sequences'][seq] = seq_out
    results['eligible_candidates'] = [gid for gid in contract['candidate_ids']
                                      if all(s['gates'][gid]['status'] == 'PASS' for s in results['sequences'].values())]
    results['decision'] = 'ADVANCE_TO_FROZEN_ON' if results['eligible_candidates'] else 'STOP_PREDICTION_ADMISSION'
    (destination / 'summary.json').write_text(json.dumps(results, indent=2) + '\n')
    print(json.dumps({'decision': results['decision'], 'sequences': {
        s: {g: {'coverage': v['mature_coverage'], 'gain': v['common_gated']['frame_rms_gain'],
                'ci': v['common_gated']['gain_ci95'], 'checks': v['checks']}
            for g, v in data['gates'].items()} for s, data in results['sequences'].items()}}, indent=2))


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--source', type=Path, required=True)
    p.add_argument('--contract', type=Path, required=True)
    p.add_argument('--destination', type=Path, required=True)
    args = p.parse_args()
    assess(args.source, args.contract, args.destination)
