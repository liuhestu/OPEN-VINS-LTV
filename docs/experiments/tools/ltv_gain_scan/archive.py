#!/usr/bin/env python3
"""Archive lossless position evidence and independently re-evaluate without raw inputs."""
import argparse
import json
from pathlib import Path
import numpy as np
from study import ROOT, UZH, FLAGS, Study, load_trajectory, nearest, aligned_rmse, sha, write


def archive(coord):
    study = Study(coord)
    destinations = [ROOT / 'docs/euroc_tune_results', ROOT / 'docs/uzhfpv_results_data']
    manifests = [[], []]
    for row in study.index:
        if row.get('status') != 'VALID_FULL_MATCHED': continue
        output = Path(row['output_dir'])
        x = load_trajectory(output / 'trajectory.csv')
        name = f"{row['sequence']}_{row['mode']}_{row['alpha']}.npz"
        hashes = {p.name: sha(p) for p in output.iterdir() if p.is_file() and p.name in (
            'trajectory.csv', 'replay.json', 'effective_options.json', 'ltv.csv', 'landmark_approx.csv',
            'landmark_points.csv', 'audit.csv', 'manifest_finish.json')}
        for number, dest in enumerate(destinations):
            if number and not (row['sequence'] in UZH and row['alpha'] == 1): continue
            folder = dest / 'evaluation_positions'
            folder.mkdir(exist_ok=True)
            target = folder / name
            if not target.exists(): np.savez_compressed(target, times=x[:, 0], positions=x[:, 5:8])
            data = np.load(target)
            assert np.array_equal(data['times'], x[:, 0]) and np.array_equal(data['positions'], x[:, 5:8])
            manifests[number].append(dict(sequence=row['sequence'], mode=row['mode'], alpha=row['alpha'], run_id=row['run_id'],
                archive=str(target.relative_to(dest)), sha256=sha(target), original_output_dir=str(output), original_hashes=hashes,
                columns='complete estimated timestamps and xyz; lossless float64 compression; no GT attitude',
                ate_rmse_m=row['ate_rmse_m'], replay=json.loads((output / 'replay.json').read_text()),
                effective_options=json.loads((output / 'effective_options.json').read_text()), audit=row['audit']))
    for dest, records in zip(destinations, manifests):
        write(dest / 'evaluation_archive.json', records)
        verify(dest)


def verify(dest):
    manifest = json.loads((dest / 'evaluation_archive.json').read_text())
    maximum = 0.
    for record in manifest:
        path = dest / record['archive']
        assert sha(path) == record['sha256']
        x = np.load(path)
        replay, options = record['replay'], record['effective_options']
        assert replay['complete'] and replay['camera_packets'] == replay['input_camera_packets'] and replay['imu_consumed'] == replay['input_imu_samples']
        assert len(x['times']) == replay['output_rows'] and np.all(np.diff(x['times']) > 0) and np.isfinite(x['positions']).all()
        keys = ['ltv_enabled', 'ltv_enable_gravity', 'ltv_enable_velocity', 'ltv_enable_landmark_approx']
        assert [int(options[k]) for k in keys] == FLAGS[record['mode']]
        assert options['experiment_mode_schema'] == 2 and options['experiment_mode'] == record['mode'] and options['max_slam_features'] == 0
        g, v, landmark = FLAGS[record['mode']][1:]
        alpha = record['alpha']
        wanted = dict(ltv_sigma_gravity_deg=10/np.sqrt(alpha if g else 1), ltv_sigma_velocity_mps=1/np.sqrt(alpha if v else 1),
                      ltv_landmark_approx_sigma_floor_m=.5/np.sqrt(alpha if landmark else 1),
                      ltv_landmark_approx_information_cap=.01*(alpha if landmark else 1), ltv_landmark_approx_max_points=2)
        assert all(np.isclose(options[k], value, rtol=1e-14, atol=0) for k, value in wanted.items())
        frozen = np.load(dest / 'support' / (record['sequence'] + '.npz'))
        assert abs(x['times'][0] - float(frozen['initialization'])) <= 1e-6
        indices = nearest(x['times'], frozen['times'])
        assert np.all(abs(x['times'][indices] - frozen['times']) <= 1e-6)
        ate, horn = aligned_rmse(x['positions'][indices], frozen['truth'])
        difference = abs(ate - record['ate_rmse_m'])
        assert difference <= 1e-12 * max(1., ate)
        maximum = max(maximum, difference, abs(ate - horn))
    result = dict(status='PASS', independently_recomputed=len(manifest), maximum_difference_m=maximum,
                  needs_original_inputs_or_temporary_directory=False, alignment='position SE(3), no scale; Horn/SVD')
    write(dest / 'archive_audit.json', result)
    print(json.dumps(result))

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('action', choices=['archive', 'verify'])
    parser.add_argument('path', type=Path)
    args = parser.parse_args()
    if args.action == 'archive': archive(args.path)
    else: verify(args.path)
