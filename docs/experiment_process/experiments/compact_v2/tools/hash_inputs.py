#!/usr/bin/env python3
"""Verify all original ASL inputs against the preserved frozen manifests."""
import hashlib
import json
from pathlib import Path
import time
ROOT = Path(__file__).resolve().parents[5]
DEST = Path(__file__).resolve().parents[1]
ARCHIVE = ROOT / 'docs/experiment_process/euroc_final_results_data'
sequences = json.loads((ARCHIVE / 'protocol.json').read_text())['sequences']
rows = []
for seq in sequences:
    start = time.time()
    path = ARCHIVE / 'support' / (seq + '_input_hashes.json')
    manifest = json.loads(path.read_text())
    data = Path('/home/he/datasets/euroc/ASL') / seq / 'mav0'
    mismatches = []
    for name, expected in manifest.items():
        try:
            actual = hashlib.sha256((data / name).read_bytes()).hexdigest()
        except OSError as error:
            actual = str(error)
        if actual != expected:
            mismatches.append(name)
    rows.append(dict(sequence=seq, files=len(manifest), errors=mismatches, start_unix=start, end_unix=time.time(),
                     frozen_manifest_sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
    (DEST / 'data_verification.json').write_text(json.dumps(rows, indent=2) + '\n')
    print(seq, len(manifest), len(mismatches), flush=True)
    if mismatches:
        raise SystemExit('Input identity failed; stop expansion')
