"""Explain frozen readiness interruptions from recorded causal diagnostics."""
import argparse
from collections import Counter
import csv
import json
from pathlib import Path
import numpy as np
from analyze import SEQUENCES, intervals, readlines, stats


def causes(f, b):
    result = []
    if not f['raw_current']:
        result.append('no_current_output')
    if f['observed_features'] < 15:
        result.append('observed_below_15')
    if f['mature_features'] < 15:
        result.append('mature_below_15')
    if not f['pool_ready']:
        result.append('pool_not_ready')
    if not f['correction_diagnostics_valid']:
        result.append('invalid_correction_diagnostics')
    def exceeds(key, value, label):
        x = f[key]
        if x is None or not np.isfinite(x) or x < 0 or x > value:
            result.append(label)
    exceeds('prediction_angle_p95_rad', .02, 'prediction_angle_above_0.02')
    exceeds('gravity_correction_rate', 1., 'gravity_rate_above_1')
    if b == 'V':
        exceeds('velocity_correction_rate', .5, 'velocity_rate_above_0.5')
    g = f['eta_body']
    if g is None or not 8. <= np.linalg.norm(g) <= 11.5:
        result.append('gravity_norm_outside_8_11.5')
    if f['actual_corrections'] < 20:
        result.append('corrections_below_20')
    if not result:
        if abs(f['last_strict_good_'+b]-f['imu_time']) <= 1e-6:
            result.append('strict_good_recorded_confirmation_below_20')
        else:
            result.append('healthy_snapshot_but_other_branch_context')
    return result


def main():
    p = argparse.ArgumentParser()
    p.add_argument('evidence', type=Path)
    p.add_argument('out', type=Path)
    args = p.parse_args()
    manifest = json.loads((args.out/'read_manifest.json').read_text())
    summary = {}
    for seq in SEQUENCES:
        path = args.evidence/'final_runs'/f'{seq}_OFF_01/features.jsonl'
        expected = manifest[str(path.resolve())]['sha256']
        f = list(readlines(path, manifest))
        assert expected == manifest[str(path.resolve())]['sha256']
        with (args.out/(seq+'_curves.csv')).open() as stream:
            curves = [r for r in csv.DictReader(stream) if r['mode']=='OFF']
        summary[seq] = {}
        t = np.array([x['target_imu_time'] for x in f])
        ids = {x['camera_ns']:i for i,x in enumerate(f)}
        for b in ('G', 'V'):
            rows = [r for r in curves if r['branch']==b]
            counts = Counter()
            reason = Counter()
            transitions = []
            seen = False
            for i, x in enumerate(f):
                if x['ready_'+b]:
                    seen = True
                elif seen and x['initialized']:
                    counts.update(causes(x, b))
                    reason[x['reason']] += 1
                    if i and f[i-1]['ready_'+b]:
                        transitions.append(dict(elapsed_s=float(t[i]-t[0]), causes=causes(x, b), reason=x['reason'],
                            observed=x['observed_features'], mature=x['mature_features'],
                            angle=x['prediction_angle_p95_rad'], velocity_rate=x['velocity_correction_rate'],
                            gravity_rate=x['gravity_correction_rate'], gap_s=float(t[i]-t[i-1])))
            better_missing = [r for r in rows if r['stratum']=='interruption' and r['rising_high_error']=='1' and
                              float(r['ltv_error']) < float(r['main_error'])]
            missed_causes = Counter()
            for r in better_missing:
                missed_causes.update(causes(f[ids[int(r['camera_ns'])]], b))
            mask = np.zeros(len(f), bool)
            for r in better_missing:
                mask[ids[int(r['camera_ns'])]] = True
            blocks = []
            for block in intervals(mask, t):
                subset = [r for r in better_missing if block['start_s']-1e-6 <= float(r['elapsed_s']) <= block['end_s']+1e-6]
                block.update(main=stats([float(r['main_error']) for r in subset]),
                             ltv=stats([float(r['ltv_error']) for r in subset]))
                blocks.append(block)
            summary[seq][b] = dict(interruption_causes_nonexclusive=dict(counts), reasons=dict(reason),
                ready_to_not_ready=transitions, missed_better_rising_frames=len(better_missing),
                missed_causes_nonexclusive=dict(missed_causes), missed_intervals=blocks,
                physical_fault_count_max=max(x['physical_fault_count'] for x in f),
                bootstrap_count_max=max(x['bootstrap_count'] for x in f))
    (args.out/'supply.json').write_text(json.dumps(summary, indent=2, allow_nan=False)+'\n')
    print(json.dumps({s:{b:dict(missed=x['missed_better_rising_frames'],causes=x['missed_causes_nonexclusive'],
        reasons=x['reasons']) for b,x in r.items()} for s,r in summary.items()}, indent=2))


if __name__ == '__main__':
    main()
