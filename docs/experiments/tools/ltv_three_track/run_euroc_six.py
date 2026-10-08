#!/usr/bin/env python3
"""Run the canonical six-mode full EuRoC matrix through the isolated launcher."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import subprocess

MODES = ['OFF', 'G', 'V', 'GV', 'L', 'L_GV']
FLAGS = {'OFF': [0, 0, 0, 0], 'G': [1, 1, 0, 0], 'V': [1, 0, 1, 0],
         'GV': [1, 1, 1, 0], 'L': [1, 0, 0, 1], 'L_GV': [1, 1, 1, 1]}
SEQUENCES = ['MH_01_easy', 'MH_02_easy', 'MH_03_medium', 'MH_04_difficult', 'MH_05_difficult',
             'V1_01_easy', 'V1_02_medium', 'V1_03_difficult', 'V2_01_easy', 'V2_02_medium', 'V2_03_difficult']


def verify_output(path, mode, full):
    errors = []
    try:
        options = json.loads((path / 'effective_options.json').read_text())
        keys = ['ltv_enabled', 'ltv_enable_gravity', 'ltv_enable_velocity', 'ltv_enable_landmark_approx']
        if [int(options[k]) for k in keys] != FLAGS[mode]:
            errors.append('effective_flags_mismatch')
        if options['experiment_mode'] != mode or options['experiment_mode_schema'] != 2:
            errors.append('mode_schema_mismatch')
        if options['max_slam_features'] != 0:
            errors.append('max_slam_not_zero')
        replay = json.loads((path / 'replay.json').read_text())
        if replay['complete'] != full:
            errors.append('input_interval_mismatch')
        if replay['camera_packets'] != replay['input_camera_packets'] or replay['imu_consumed'] != replay['input_imu_samples']:
            errors.append('incomplete_input_consumption')
        if mode == 'OFF' and (replay['actual_G_submissions'] or replay['actual_V_submissions']):
            errors.append('OFF_auxiliary_applied')
        landmark = path / 'landmark_approx.csv'
        applied = 0
        if landmark.exists():
            rows = list(csv.DictReader(landmark.open()))
            active = [r for r in rows if int(r['rows']) > 0 and r['consumed'] == '1']
            applied = len(active)
            if any(r['submit_reason'] != 'joint_applied' or int(r['ekf_calls']) != 1 for r in active):
                errors.append('landmark_receipt_mismatch')
            if any(int(r['selected']) > 2 for r in rows):
                errors.append('landmark_supply_limit')
        if not FLAGS[mode][3] and applied:
            errors.append('disabled_landmark_applied')
        return dict(errors=errors, actual_G=replay['actual_G_submissions'], actual_V=replay['actual_V_submissions'],
                    actual_L=applied, output_rows=replay['output_rows'], camera_packets=replay['camera_packets'])
    except (OSError, KeyError, ValueError) as e:
        return dict(errors=['missing_or_invalid_output: ' + str(e)])


def main():
    p = argparse.ArgumentParser()
    p.add_argument('coord', type=Path)
    p.add_argument('--stage', choices=['smoke', 'full'], required=True)
    p.add_argument('--data', type=Path, default=Path('/home/he/datasets/euroc/ASL'))
    a = p.parse_args()
    identity = json.loads((a.coord / 'identity.json').read_text())
    install = Path(identity['artifact']) / 'install'
    config = Path(identity['source']) / 'docs/experiments/configs/euroc_c0/estimator_config.yaml'
    index_path = a.coord / (a.stage + '_index.json')
    results = json.loads(index_path.read_text()) if index_path.exists() else []
    done = {(r['sequence'], r['mode']) for r in results}
    sequences = ['V2_02_medium', 'V2_03_difficult'] if a.stage == 'smoke' else SEQUENCES
    for seq in sequences:
        for mode in MODES:
            if (seq, mode) in done:
                continue
            command = [str(a.coord / 'tools/env_exec.sh'), str(install),
                       str(install / 'ov_msckf/lib/ov_msckf/run_ltv_gv_production'), str(config),
                       str(a.data / seq / 'mav0'), '{RUN_DIR}', mode]
            if a.stage == 'smoke':
                command.append('40')
            spec = dict(coord_root=str(a.coord.resolve()), task='six', label=a.stage + '_' + seq + '_' + mode,
                        kind='replay', heavy=True, source_oid=identity['source_oid'], source_snapshot=identity['source'],
                        command=command, binary_paths=[str(install / 'ov_msckf/lib/ov_msckf/run_ltv_gv_production')],
                        config_paths=[str(config)], configuration=dict(stage=a.stage, sequence=seq, mode=mode, schema=2, threads=1),
                        data=dict(root=str(a.data / seq / 'mav0'), identity=str(a.coord / 'data_identity.json')),
                        diagnostic_level='scalar_ltv_csv')
            sp = a.coord / (a.stage + '_' + seq + '_' + mode + '.json')
            sp.write_text(json.dumps(spec, indent=2) + '\n')
            result = subprocess.run(['python3', str(a.coord / 'tools/launcher.py'), str(sp)])
            events = [json.loads(l) for l in (a.coord / 'registry/task_six.jsonl').read_text().splitlines()]
            event = next(e for e in reversed(events) if e.get('spec_path') == str(sp) and e['event'] in ('FINISH', 'LAUNCHER_FAILURE'))
            output = Path(event.get('output_dir', ''))
            audit = verify_output(output, mode, a.stage == 'full') if event.get('exit_code') == 0 else {'errors': ['runtime_failure']}
            row = dict(sequence=seq, mode=mode, run_id=event['run_id'], output_dir=str(output),
                       exit_code=event.get('exit_code'), audit=audit)
            results.append(row)
            index_path.write_text(json.dumps(results, indent=2) + '\n')
            print(json.dumps(row), flush=True)
    if a.stage == 'smoke':
        # Gate implementation correctness, not whether a sequence has usable observations.
        if any(r['audit']['errors'] for r in results):
            raise SystemExit('smoke interface failed; retain all results and inspect runner before full batch')
        for seq in sequences:
            triple = next(r for r in results if r['sequence'] == seq and r['mode'] == 'L_GV')
            if not all(triple['audit'][k] > 0 for k in ['actual_G', 'actual_V', 'actual_L']):
                raise SystemExit('combined smoke has no actual update in a branch; inspect before batch')
    assert len(results) == len(sequences) * 6


if __name__ == '__main__':
    main()
