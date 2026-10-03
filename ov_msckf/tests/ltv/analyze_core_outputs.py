#!/usr/bin/env python3
"""Report source/target changes and spectra after immutable-golden comparisons."""
import argparse
import json
from pathlib import Path
import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path, help='Directory containing source/target_outputs.stdout.log')
    args = parser.parse_args()
    rows = {name: [json.loads(line) for line in (args.output / (name + '_outputs.stdout.log')).read_text().splitlines()]
            for name in ['source', 'target']}
    if len(rows['source']) != 174 or len(rows['target']) != 174:
        raise SystemExit('Expected 174 events')
    summary = {}
    for key in ['state', 'P']:
        maximum, event, ratio = 0.0, None, 0.0
        for old, new in zip(rows['source'], rows['target']):
            a, b = np.array(old[key]), np.array(new[key])
            difference = np.abs(a - b)
            if difference.max() > maximum:
                maximum, event = float(difference.max()), old['event']
            ratio = max(ratio, float(np.max(difference / (1e-8 + 1e-10 * np.abs(a)))))
        summary[key] = dict(max_absolute_difference=maximum, event=event, max_tolerance_ratio=ratio)
    passed = True
    for name, events in rows.items():
        asymmetry, negative, scale, normalized_negative = 0.0, 0.0, 1.0, 0.0
        for row in events:
            p = np.array(row['P'])
            if not np.isfinite(p).all():
                raise SystemExit('Non-finite P')
            asymmetry = max(asymmetry, float(np.abs(p - p.T).max()))
            eigenvalues = np.linalg.eigvalsh((p + p.T) / 2)
            local_scale = max(1.0, float(np.max(np.abs(eigenvalues))))
            negative = min(negative, float(eigenvalues.min()))
            normalized_negative = max(normalized_negative, max(0.0, -float(eigenvalues.min())) / local_scale)
            scale = max(scale, local_scale)
        summary[name] = dict(max_asymmetry=asymmetry, min_eigenvalue_or_zero=negative,
                             max_spectral_scale=scale, max_normalized_negative_eigenvalue=normalized_negative)
        if name == 'target':
            passed = asymmetry == 0.0 and normalized_negative <= 1e-10
    summary['target_spectral_pass'] = passed
    (args.output / 'numerical_differences.json').write_text(json.dumps(summary, indent=2))
    print(json.dumps(summary, indent=2))
    if not passed:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
