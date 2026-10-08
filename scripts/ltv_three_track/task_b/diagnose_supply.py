"""Descriptive source audit after B gate v1; no additional gate qualification."""
import argparse
import csv
import json
import math
from pathlib import Path


def main(source, destination):
    destination.mkdir(parents=True, exist_ok=False)
    results = {'purpose': 'distinguish a fixed-window supply design error from prediction-model failure',
               'status': 'DESCRIPTIVE_NOT_ADMISSION', 'sequences': {}}
    for seq in ['V2_02_medium', 'V2_03_difficult']:
        bins = {}
        rows = 0
        with (source / 'past_geometry_audit' / (seq + '_predictions.csv')).open() as stream:
            for r in csv.DictReader(stream):
                if r['past_mature'] != '1' or r['past_ready_V'] != '1':
                    continue
                rows += 1
                span = float(r['anchor_span'])
                if not math.isfinite(span):
                    label = 'nonfinite'
                elif abs(span - .5) <= 1e-6:
                    label = '0.5s_plusminus_1us'
                elif abs(span - .55) <= 1e-6:
                    label = '0.55s_plusminus_1us'
                elif span < .2 - 1e-6:
                    label = 'below_0.2s'
                elif span < .5 - 1e-6:
                    label = '0.2_to_0.5s'
                elif span < .6:
                    label = '0.5_to_0.6s_other'
                else:
                    label = 'above_0.6s'
                b = bins.setdefault(label, dict(n=0, min_span_s=span, max_span_s=span,
                                                v1_age_accepted=0, disagreement_accepted=0,
                                                paired=0, ltv_squared=0., geometry_squared=0.,
                                                ltv_bad=0, geometry_bad=0))
                b['n'] += 1
                if math.isfinite(span):
                    b['min_span_s'] = min(b['min_span_s'], span)
                    b['max_span_s'] = max(b['max_span_s'], span)
                b['v1_age_accepted'] += .2 - 1e-9 <= span <= .5 + 1e-9
                l = [float(r['ltv_' + d]) for d in 'xyz']
                m = [float(r['main_' + d]) for d in 'xyz']
                relative = math.sqrt(sum((a - c) ** 2 for a, c in zip(l, m))) / max(math.sqrt(sum(c*c for c in m)), .1)
                b['disagreement_accepted'] += relative <= .02
                if r['ltv_valid'] == r['past_fit_valid'] == '1':
                    le, ge = float(r['ltv_angle_rad']), float(r['past_fit_angle_rad'])
                    b['paired'] += 1
                    b['ltv_squared'] += le * le
                    b['geometry_squared'] += ge * ge
                    b['ltv_bad'] += le > .02
                    b['geometry_bad'] += ge > .02
        for b in bins.values():
            n = b['paired']
            b['ltv_point_rms_rad'] = math.sqrt(b.pop('ltv_squared') / n) if n else None
            b['geometry_point_rms_rad'] = math.sqrt(b.pop('geometry_squared') / n) if n else None
        results['sequences'][seq] = {'ready_mature_candidates': rows, 'bins': bins}
    (destination / 'supply.json').write_text(json.dumps(results, indent=2) + '\n')
    print(json.dumps(results, indent=2))


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--source', type=Path, required=True)
    p.add_argument('--destination', type=Path, required=True)
    a = p.parse_args()
    main(a.source, a.destination)
