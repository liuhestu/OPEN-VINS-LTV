#!/usr/bin/env python3
"""Join attempted shadow events with authoritative post-frame source receipts.

Conditional spectra are unit-source sensitivity diagnostics, not calibrated S.
"""
import argparse
import importlib.util
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
SPEC = importlib.util.spec_from_file_location('matrix_reader', ROOT / 'docs/ltv/landmark_correlation/observer/read_shadow_matrices.py')
READER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(READER)


def audit(directory):
    path = directory / 'joint_shadow.csv'
    reference = READER.audit(path)
    receipts = [json.loads(line) for line in (directory / 'shadow_receipts.jsonl').open()]
    by_epoch = {}
    for receipt in receipts:
        if receipt['available']:
            by_epoch.setdefault(receipt['epoch'], []).append(receipt)
    times = {epoch: np.array([r['time'] for r in rows]) for epoch, rows in by_epoch.items()}
    candidates, matched, ambiguous, unmatched = 0, 0, 0, 0
    spectra = []
    identities = set()
    with Path(str(path) + '.matrices.bin').open('rb') as binary:
        for line in Path(str(path) + '.matrices.jsonl').open():
            event = json.loads(line)
            if event['event'] != 'camera':
                continue
            candidates += 1
            epoch, time = event['epoch'], event['time']
            rtime = times.get(epoch, np.array([]))
            indices = np.where(np.abs(rtime - time) <= 1e-6)[0]
            matching = []
            for index in indices:
                r = by_epoch[epoch][index]
                local = [entry['local_id'] for entry in sorted(r['core_ids'], key=lambda item: item['slot'])]
                if local == event['retained_local_ids']:
                    matching.append(r)
            if len(matching) == 1:
                key = (epoch, matching[0]['sequence'], matching[0]['camera_ns'])
                if key in identities:
                    ambiguous += 1
                else:
                    matched += 1
                    identities.add(key)
            elif not matching:
                unmatched += 1
            else:
                ambiguous += 1
            m = {key: READER.decode(binary, desc) for key, desc in event['matrices'].items()}
            point = m['conditional_current_point_source_map']
            singular = np.linalg.svd(point, compute_uv=False)
            eigenvalues = singular ** 2
            threshold = 1e-11 * max(float(eigenvalues.max()) if eigenvalues.size else 0., 1.)
            # Rank of known conditional source contribution only. Missing noise,
            # main C and visual cross prevent candidate R/N/S/K construction.
            spectra.append({'epoch': epoch, 'time': time, 'dimension': point.shape[0],
                            'nonzero_spectrum_candidate': eigenvalues.tolist(),
                            'effective_rank': int(np.sum(eigenvalues > threshold)), 'rank_threshold': threshold,
                            'unconditional_valid': False})
    return {'matrix_reference': reference, 'source_receipt_join': {'attempted_camera_events': candidates,
            'unique_receipt_time_epoch_slot_matches': matched, 'ambiguous_or_duplicate_matches': ambiguous,
            'unmatched_attempts': unmatched, 'interpretation': 'post-frame receipt identity support only; full x/P equality established by cache regression, attempted matrices do not prove committed conditional-map correctness'},
            'conditional_point_spectra': spectra, 'full_model_valid_constraints': 0,
            'full_model_R_N_visual_S_K': 'BLOCKED_UNKNOWN_JOINT_SOURCE_BLOCKS',
            'landmark_statistical_calibration': 'NOT_EVALUATED_NO_INDEPENDENT_TRUTH'}


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('directory', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    result = audit(args.directory)
    args.output.write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps({k: v for k, v in result.items() if k != 'conditional_point_spectra'}, indent=2))
