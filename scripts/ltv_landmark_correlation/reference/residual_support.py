#!/usr/bin/env python3
"""Supplement LC05: covariance PSD does not prove a residual is in its support."""
import csv
from pathlib import Path

import numpy as np
from run import X, ANCHOR, CURRENT, VISUAL, block_model, source_map


def main():
    rows = []
    residual = np.array([.1, -.08, .05])  # Same frozen offline diagnostic residual.
    for scene in ('zero_noise', 'independent_sources', 'shared_imu_bias', 'shared_bearing',
                  'seed_feedback', 'duplicate_information', 'near_rank_deficiency'):
        m = source_map(scene)
        full = m @ m.T
        zero = np.zeros_like(full)
        for block in (X, ANCHOR, CURRENT, VISUAL):
            zero[block, block] = full[block, block]
        for label, q in (('full', full), ('zero_all_consumer_cross_blocks_DIAGNOSTIC_ONLY', zero)):
            model = block_model(q)
            values, vectors = np.linalg.eigh((model['S'] + model['S'].T) / 2)
            selected = values > 1e-11 * max(float(np.max(np.abs(values))), 1.)
            projection = vectors[:, selected] @ vectors[:, selected].T
            outside = float(np.linalg.norm(residual - projection @ residual))
            compatible = outside <= 1e-10 * max(float(np.linalg.norm(residual)), 1.)
            rows.append({'scene': scene, 'model': label, 'residual_rank': model['rank'],
                         'covariance_compatible': True, 'residual_support_compatible': compatible,
                         'outside_support_norm': outside, 'support_tolerance': 1e-10,
                         'Kr_interpretation': 'formal covariance diagnostic only; residual invalid' if not compatible else 'offline diagnostic only',
                         'fusion_authorization': 'NONE'})
    assert sum(not row['residual_support_compatible'] for row in rows) == 4
    assert next(r for r in rows if r['scene'] == 'duplicate_information' and r['model'] == 'full')['residual_support_compatible'] is False
    destination = Path(__file__).resolve().parents[3] / 'docs/ltv/landmark_correlation/reference/residual_support.csv'
    with destination.open('w', newline='') as stream:
        writer = csv.DictWriter(stream, rows[0].keys(), lineterminator='\n')
        writer.writeheader()
        writer.writerows(rows)
    print('PASS: four residual-support incompatibilities preserved; no updates submitted')


if __name__ == '__main__':
    main()
