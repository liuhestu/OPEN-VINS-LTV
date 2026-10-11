#!/usr/bin/env python3
"""Compare the final production replay with the archived frozen experiment."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import sys

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[4]
TOOLS = ROOT / 'docs/experiment_process/experiments/tools/ltv_gain_scan'
sys.path.insert(0, str(TOOLS))
from run_manual_euroc import resolve, prepare, execute


def read(path):
    return json.loads(path.read_text())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('install', type=Path)
    parser.add_argument('--output', type=Path, default=Path(__file__).with_name('regression.json'))
    args = parser.parse_args()
    archive = ROOT / 'docs/experiment_process/euroc_final_results_data'
    frozen = {(r['sequence'], r['mode']): r for r in read(archive / 'results.json')}
    yaml_path = ROOT / 'config/euroc_final/manual_run.yaml'
    base = yaml.safe_load(yaml_path.read_text())
    rows = []
    for seq in ['MH_05_difficult', 'V2_02_medium']:
        for mode in ['OFF', 'G', 'V', 'L', 'GVL']:
            config = dict(base, sequence=seq, mode=mode, weights={'G': None, 'V': None, 'L': None},
                          install_root=str(args.install.resolve()), short_seconds=0,
                          output_root='/home/he/output/ltv_finalization_regression')
            selection = resolve(config)
            output, target = prepare(yaml_path, config, selection)
            # Archive the actual selection used by this verification, not the default YAML.
            (output / 'manual_run.yaml').write_text(yaml.safe_dump(config))
            print(f'RUN {seq} {mode}: {output}', flush=True)
            execute(output, target, selection)
            previous = Path(frozen[(seq, mode)]['output_dir'])
            actual, expected = output / 'trajectory.csv', previous / 'trajectory.csv'
            identical = actual.read_bytes() == expected.read_bytes()
            x = np.loadtxt(actual, delimiter=',', skiprows=1, ndmin=2)
            y = np.loadtxt(expected, delimiter=',', skiprows=1, ndmin=2)
            same_shape = x.shape == y.shape
            maximum_difference = float(np.max(np.abs(x - y))) if same_shape else None
            actual_run, old_run = read(output / 'replay.json'), read(previous / 'replay.json')
            input_keys = ['input_left_images', 'input_right_images', 'camera_packets', 'input_camera_packets',
                          'imu_consumed', 'input_imu_samples', 'output_rows', 'actual_G_submissions',
                          'actual_V_submissions', 'complete']
            replay_matches = all(actual_run[k] == old_run[k] for k in input_keys)
            validation = read(output / 'manual_validation.json')
            applications = validation['actual_applications']
            archived = frozen[(seq, mode)]['audit']
            applications_match = applications == dict(G=archived['actual_G'], V=archived['actual_V'], L=archived['actual_L'])
            receipt_matches = (output / 'fusion.jsonl').read_bytes() == (previous / 'fusion.jsonl').read_bytes()
            observer_matches = (output / 'observer_identity.jsonl').read_bytes() == (previous / 'observer_identity.jsonl').read_bytes()
            main_audit_matches = (output / 'audit.csv').read_bytes() == (previous / 'audit.csv').read_bytes()
            row = dict(sequence=seq, mode=mode, output_dir=str(output), frozen_output_dir=str(previous),
                       trajectory_byte_identical=identical, maximum_absolute_numeric_difference=maximum_difference,
                       replay_counts_match=replay_matches, actual_applications=applications,
                       applications_match=applications_match, fusion_receipts_byte_identical=receipt_matches,
                       observer_digests_byte_identical=observer_matches, main_state_covariance_audit_byte_identical=main_audit_matches,
                       binary_sha256=selection['binary_sha256'])
            row['status'] = 'PASS' if all([identical, replay_matches, applications_match, receipt_matches, observer_matches, main_audit_matches]) else 'FAIL'
            rows.append(row)
            record = dict(timestamp_utc=datetime.now(timezone.utc).isoformat(),
                          status='IN_PROGRESS' if len(rows) < 10 else ('PASS' if all(r['status'] == 'PASS' for r in rows) else 'FAIL'),
                          completed=len(rows), planned=10, cases=rows,
                          scope='Two full sequences, all five modes; exact trajectories, main state/P audits, scalar fusion receipts and Observer x/P digests. Not independent accuracy validation or full lifecycle engineering acceptance.')
            args.output.write_text(json.dumps(record, indent=2) + '\n')
            print(f"{row['status']} {seq} {mode}: max difference {maximum_difference}", flush=True)
    if any(r['status'] != 'PASS' for r in rows):
        raise SystemExit(1)


if __name__ == '__main__':
    main()
