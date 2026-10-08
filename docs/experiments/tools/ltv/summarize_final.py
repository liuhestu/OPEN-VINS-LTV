#!/usr/bin/env python3
"""Evaluate the frozen matrix and select repeats without changing any estimator setting."""
import json
from pathlib import Path
import numpy as np
from evaluate_sequence import evaluate
from run_sequence import OUT, ROOT, sha, write


def main():
    protocol = json.loads((OUT / 'evaluator_protocol.json').read_text())
    assert protocol['evaluator_sha256'] == sha(ROOT / 'docs/experiments/tools/ltv/evaluate_sequence.py')
    paths = json.loads((OUT / 'final_paths.json').read_text())
    results, rows, passive = {}, [], {}
    for sequence, modes in paths.items():
        result = evaluate(sequence, modes)
        write(OUT / ('metrics_' + sequence + '.json'), result)
        results[sequence] = result
        row = {'sequence': sequence}
        for mode in ['B', 'G', 'V', 'GV']:
            rec = result['modes'][mode]
            row[mode] = rec.get('ate_rmse_m') if rec['status'] == 'VALID' else None
        for mode in ['G', 'V', 'GV']:
            row['delta_' + mode + '_percent'] = 100 * (row['B'] - row[mode]) / row['B'] if row['B'] and row[mode] is not None else None
        rows.append(row)
        passive[sequence] = {name: (Path(modes['B']) / name).read_bytes() == (Path(modes['P']) / name).read_bytes()
                             for name in ['trajectory.csv', 'audit.csv']}
        assert all(passive[sequence].values()), 'OFF/Passive differs on ' + sequence
    groups = {}
    for group, subset in [('Easy/Medium', [r for r in rows if 'difficult' not in r['sequence']]),
                          ('Difficult', [r for r in rows if 'difficult' in r['sequence']]), ('All', rows)]:
        valid = [r for r in subset if all(r[m] is not None for m in ['B', 'G', 'V', 'GV'])]
        means = {mode: float(np.mean([r[mode] for r in valid])) if valid else None for mode in ['B', 'G', 'V', 'GV']}
        groups[group] = dict(sequences=[r['sequence'] for r in valid], valid_count=len(valid), total_count=len(subset), means=means)
    ranked = sorted((r for r in rows if r['delta_GV_percent'] is not None), key=lambda r: r['delta_GV_percent'], reverse=True)
    repeats = ['V1_01_easy']
    for candidates in [ranked, list(reversed(ranked))]:
        for row in candidates:
            if row['sequence'] not in repeats:
                repeats.append(row['sequence'])
                break
    write(OUT / 'final_summary.json', dict(rows=rows, groups=groups, passive_exact=passive,
                                         delta_definition='100*(B-mode)/B; positive is improvement', repeat_sequences=repeats))
    write(OUT / 'repeat_sequences.json', repeats)
    print(json.dumps(dict(rows=rows, groups=groups, repeats=repeats), indent=2))


if __name__ == '__main__':
    main()
