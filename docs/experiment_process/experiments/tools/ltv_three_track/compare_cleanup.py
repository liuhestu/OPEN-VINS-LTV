#!/usr/bin/env python3
"""Compare full cleanup replays, excluding timers/output paths only."""
import argparse
import csv
import hashlib
import json
from pathlib import Path

p = argparse.ArgumentParser()
p.add_argument('index', type=Path)
p.add_argument('output', type=Path)
a = p.parse_args()
rows = json.loads(a.index.read_text())
cases = {(r['task'], r['sequence'], r['mode']): Path(r['output_dir']) for r in rows}
results = []
byte_files = ['trajectory.csv', 'audit.csv', 'algorithm_visual_identity.jsonl', 'observer_identity.jsonl',
              'lifecycle_identity.jsonl', 'shadow_receipts.jsonl', 'fusion.jsonl', 'unmatched_camera.csv',
              'instrumentation.json', 'replay.json', 'ltv.csv', 'landmark_approx.csv', 'landmark_approx.csv.points.csv']
for seq in ['V2_02_medium', 'V2_03_difficult']:
    for mode in ['OFF', 'G', 'V', 'GV', 'L_OFF', 'L_ON']:
        left, right = cases['experiment', seq, mode], cases['main', seq, mode]
        evidence = {}
        for name in byte_files:
            x, y = left / name, right / name
            assert x.exists() == y.exists(), (seq, mode, name)
            if x.exists():
                assert x.read_bytes() == y.read_bytes(), (seq, mode, name)
                evidence[name] = hashlib.sha256(x.read_bytes()).hexdigest()
        def features(path):
            entries = [json.loads(l) for l in path.read_text().splitlines()]
            for row in entries:
                row.pop('compute_time_ms', None)
            return entries
        assert features(left / 'features.jsonl') == features(right / 'features.jsonl'), (seq, mode, 'features')
        def options(path):
            x = json.loads(path.read_text())
            for key in list(x):
                if key in ('ltv_log_path', 'ltv_passive_cache_path', 'ltv_value_diagnostics_path', 'ltv_landmark_approx_log_path'):
                    del x[key]
            return x
        assert options(left / 'effective_options.json') == options(right / 'effective_options.json'), (seq, mode, 'options')
        assert json.loads((right / 'replay.json').read_text())['complete']
        results.append(dict(sequence=seq, mode=mode, status='PASS_EXACT', files=evidence,
                            before=str(left), after=str(right)))
a.output.write_text(json.dumps({'status': 'PASS_EXACT_ALL_12_FULL_CASES', 'cases': results,
                                'excluded': ['features compute_time_ms', 'explicit diagnostic output path values'],
                                'scope': 'cleanup equivalence, no new accuracy/benefit qualification'}, indent=2) + '\n')
print('PASS_EXACT_ALL_12_FULL_CASES')
