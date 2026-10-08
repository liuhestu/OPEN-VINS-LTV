"""Verify frozen identities and independent physical-reference/trajectory agreement."""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import numpy as np
from analyze import SEQUENCES, nearest, reference


def main():
    p = argparse.ArgumentParser()
    p.add_argument('evidence', type=Path)
    p.add_argument('out', type=Path)
    args = p.parse_args()
    root = Path(__file__).resolve().parents[4]
    frozen = json.loads((root/'docs/ltv/gv_fusion_evaluation/evidence/external_manifest.json').read_text())
    read = json.loads((args.out/'read_manifest.json').read_text())
    count = 0
    for path, identity in read.items():
        raw = Path(path)
        if raw.is_relative_to(args.evidence):
            expected = frozen['files'][str(raw.relative_to(args.evidence))]
            assert identity == expected, raw
            count += 1
    sys.path.insert(0, str(root/'docs/experiments/tools/ltv_feature_passive'))
    spec = importlib.util.spec_from_file_location('frozen_reference', root/'docs/experiments/tools/ltv_feature_passive/real_evaluate.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    result = dict(raw_files_matching_frozen_manifest=count, sequences={}, online_source_changes=False, new_replays=0)
    summary = json.loads((args.out/'summary.json').read_text())
    frozen_metrics = json.loads((root/'docs/ltv/gv_fusion_evaluation/metrics.json').read_text())
    for seq in SEQUENCES:
        compact = json.loads((args.out/(seq+'_OFF_01_compact.json')).read_text())
        t = np.array([x['target_imu_time'] for x in compact['features']])
        freeze = json.loads((args.evidence/'freeze.json').read_text())
        gtpath = Path(freeze['inputs'][seq]['root'])/'state_groundtruth_estimate0/data.csv'
        assert read[str(gtpath)]['sha256'] == next(
            m['gt_sha'] for m in frozen_metrics if m['sequence']==seq and m['mode']=='OFF')
        v, g, valid = reference(np.loadtxt(gtpath, delimiter=',', comments='#'), t)
        independent = module.physical_reference(module.load_gt(seq, gtpath), t)
        np.testing.assert_array_equal(valid, independent['pose_valid'])
        np.testing.assert_allclose(v, independent['v'], atol=1e-14, rtol=0, equal_nan=True)
        np.testing.assert_allclose(g, independent['eta']/9.81, atol=1e-14, rtol=0, equal_nan=True)
        post = np.load(args.out/(seq+'_OFF_01.npz'))['post']
        trajectory = np.loadtxt(args.evidence/'final_runs'/(seq+'_OFF_01')/'trajectory.csv', delimiter=',', skiprows=1)
        ix = nearest(t, trajectory[:, 0])
        assert np.max(abs(t[ix]-trajectory[:, 0])) <= 1e-6
        np.testing.assert_array_equal(post[ix], trajectory[:, 1:])
        s = summary[seq]
        for b in ('G', 'V'):
            masks = s['branches'][b]['masks']
            assert sum(masks[k] for k in ('startup', 'normal', 'weak_visual', 'interruption')) == s['common_gt_frames']
            for mode, z in s['modes'].items():
                a = z[b]['all']
                assert a['actual_frames'] <= a['ready_frames']
                assert a['actual_frames'] <= a['attempts']
                if mode == 'OFF' or mode == ('V' if b=='G' else 'G'):
                    assert a['actual_frames'] == 0
                for values in a['corrections'].values():
                    assert values['n'] == a['actual_frames']
        result['sequences'][seq] = dict(physical_reference_matches_existing=True,
            OFF_matrix_states_equal_trajectory=True, trajectory_samples_checked=len(trajectory),
            common_GT_frames=s['common_gt_frames'], mask_partition_and_actual_event_counts=True)
    assert count == 24
    result['protocol_sha256'] = hashlib.sha256((root/'docs/ltv/gv_no_benefit_diagnosis/protocol.md').read_bytes()).hexdigest()
    assert result['protocol_sha256'] == json.loads((args.out/'freeze.json').read_text())['protocol_sha256']
    result['status'] = 'PASS'
    (args.out/'verification.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
