#!/usr/bin/env python3
"""Apply frozen engineering metric thresholds without dropping sparse/empty bins."""
import argparse
import csv
import json
from pathlib import Path


def assess(rows, contract):
    refs = {(r['sequence'], r['region'], r['metric']): float(r['value'])
            for r in rows if r['mode'] == 'OFF' and r['value'] not in ('', None)}
    global_keys = {'ATE_m': 'ATE_translation_m', 'RPE_1s_m': 'RPE_translation_m_1s_5s',
                   'RPE_5s_m': 'RPE_translation_m_1s_5s', 'RPE_1s_deg': 'RPE_rotation_deg_1s_5s',
                   'RPE_5s_deg': 'RPE_rotation_deg_1s_5s', 'velocity_mps': 'velocity_mps'}
    result = []
    for original in rows:
        r = dict(original)
        base = refs.get((r['sequence'], r['region'], r['metric']))
        r['off'] = base
        if r['value'] in ('', None) or base is None:
            r.update(delta='', allowed_worsening='', status='NOT_RUN_EMPTY_SUPPORT')
        else:
            spec = contract['attitude_deg'] if r['metric'] == 'attitude_deg' else contract['global_metrics'][global_keys[r['metric']]]
            local = r['region'] != 'full'
            fraction = contract['local_metrics']['relative'] if local else spec['relative']
            absolute = spec['absolute'] * (contract['local_metrics']['absolute_multiplier'] if local else 1)
            delta, tolerance = float(r['value']) - base, max(fraction * base, absolute)
            r.update(delta=delta, allowed_worsening=tolerance, status='PASS_WITHIN_SCOPE' if delta <= tolerance else 'FAIL')
        result.append(r)
    return result


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('metrics', type=Path)
    p.add_argument('contract', type=Path)
    p.add_argument('output', type=Path)
    a = p.parse_args()
    with a.metrics.open() as f: rows = list(csv.DictReader(f))
    result = assess(rows, json.loads(a.contract.read_text()))
    with a.output.open('x') as f:
        w = csv.DictWriter(f, fieldnames=result[0].keys())
        w.writeheader()
        w.writerows(result)
    print(json.dumps({'rows': len(result), 'failures': sum(r['status'] == 'FAIL' for r in result),
                      'empty': sum(r['status'] == 'NOT_RUN_EMPTY_SUPPORT' for r in result),
                      'scope': 'metrics only; execution/performance/determinism/statistics separately judged'}))
