#!/usr/bin/env python3
"""Verify original baseline raw bytes; missing history is never synthesized."""
import argparse
import hashlib
import json
from pathlib import Path


def sha(path):
    h = hashlib.sha256()
    with path.open('rb') as f:
        for block in iter(lambda: f.read(4 * 1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()

p = argparse.ArgumentParser()
p.add_argument('source', type=Path)
p.add_argument('output', type=Path)
a = p.parse_args()
freeze = json.loads((a.source / 'docs/ltv/landmark_validation/freeze.json').read_text())
rows = []
for sequence, entry in freeze['inputs'].items():
    baseline = Path(entry['off_path'])
    for filename, expected in entry['off_evidence_sha'].items():
        path = baseline / filename
        try:
            actual = sha(path)
            status = 'PASS' if actual == expected else 'HISTORICAL_SHA_MISMATCH'
            error = None
        except OSError as e:
            actual, status, error = None, 'HISTORICAL_UNAVAILABLE', repr(e)
        rows.append({'sequence': sequence, 'path': str(path), 'expected_sha256': expected,
                     'actual_sha256': actual, 'status': status, 'error': error})
        print(sequence, filename, status, flush=True)
result = {'status': 'PASS' if all(r['status'] == 'PASS' for r in rows) else 'HISTORICAL_UNAVAILABLE',
          'rows': rows, 'scope': 'old full OFF raw evidence, not new replay'}
with (a.output / 'historical_baseline_identity.json').open('x') as f:
    json.dump(result, f, indent=2)
    f.write('\n')
# Missing baseline does not invalidate newly frozen sensor data; publisher reports both separately.
