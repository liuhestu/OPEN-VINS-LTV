"""Read-only diagnosis of frozen G/V evidence; never supplies GT to replay."""
import argparse
import csv
import hashlib
import json
from itertools import zip_longest
from pathlib import Path

import numpy as np
from scipy.linalg import cho_factor, cho_solve
from scipy.spatial.transform import Rotation, Slerp

SEQUENCES = ('V2_02_medium', 'V2_03_difficult')
BLOCKS = {'attitude_deg': slice(0, 3), 'position_m': slice(3, 6), 'velocity_mps': slice(6, 9)}


def nearest(t, u):
    hi = np.searchsorted(t, u).clip(0, len(t)-1)
    lo = np.maximum(hi-1, 0)
    return np.where(abs(t[lo]-u) <= abs(t[hi]-u), lo, hi)


def angle(a, b):
    denominator = np.linalg.norm(a, axis=-1)*np.linalg.norm(b, axis=-1)
    dot = np.divide(np.sum(a*b, axis=-1), denominator,
                    out=np.full_like(denominator, np.nan), where=denominator > 0)
    return np.degrees(np.arccos(np.clip(dot, -1, 1)))


def body(x):
    result_v = np.full((len(x), 3), np.nan)
    result_g = result_v.copy()
    valid = np.isfinite(x).all(axis=1) & (np.linalg.norm(x[:, :4], axis=1) > 0)
    if not valid.any():
        return result_v, result_g
    rot = Rotation.from_quat(x[valid, :4]).as_matrix()
    result_v[valid] = np.einsum('nji,nj->ni', rot, x[valid, 7:10])
    result_g[valid] = np.einsum('nji,j->ni', rot, [0., 0., -1.])
    return result_v, result_g


def reference(raw, t):
    gt = raw[:, 0]*1e-9
    ix = np.searchsorted(gt, t).clip(1, len(gt)-1)
    near = nearest(gt, t)
    valid = ((t >= gt[0]) & (t <= gt[-1]) & (abs(gt[near]-t) <= .02) &
             (gt[ix]-gt[ix-1] <= .010000001))
    v = np.full((len(t), 3), np.nan)
    g = v.copy()
    if not valid.any():
        return v, g, valid
    rot = Slerp(gt-gt[0], Rotation.from_quat(raw[:, [5, 6, 7, 4]]))(t[valid]-gt[0]).as_matrix()
    vel = np.column_stack([np.interp(t[valid], gt, raw[:, j]) for j in range(8, 11)])
    v[valid] = np.einsum('nji,nj->ni', rot, vel)
    g[valid] = np.einsum('nji,j->ni', rot, [0., 0., -1.])
    return v, g, valid


def stats(x):
    x = np.asarray(x, float)
    x = x[np.isfinite(x)]
    if not len(x):
        return dict(n=0, mean=None, rms=None, p50=None, p95=None, maximum=None)
    return dict(n=len(x), mean=float(x.mean()), rms=float(np.sqrt(np.mean(x*x))),
                p50=float(np.median(x)), p95=float(np.quantile(x, .95)), maximum=float(x.max()))


def joint_gain(raw):
    P, H, S = [np.asarray(raw[k], float) for k in ('prior_P', 'joint_H', 'joint_S')]
    dense = np.zeros((H.shape[0], len(P)))
    offset = 0
    for id_, size in zip(raw['joint_ids'], raw['joint_sizes']):
        dense[:, id_:id_+size] = H[:, offset:offset+size]
        offset += size
    assert offset == H.shape[1]
    return cho_solve(cho_factor(S, lower=False), (P@dense.T).T).T


def inject(prior, delta):
    """Native JPL IMU injection; auxiliary branch contribution only, not an EKF."""
    x = prior.copy()
    valid = np.isfinite(prior).all(axis=1) & np.isfinite(delta).all(axis=1)
    dq = np.column_stack([.5*delta[valid, :3], np.ones(valid.sum())])
    dq /= np.linalg.norm(dq, axis=1)[:, None]
    q = prior[valid, :4]
    new = np.column_stack([dq[:, 3, None]*q[:, :3]-np.cross(dq[:, :3], q[:, :3])+dq[:, :3]*q[:, 3, None],
                           dq[:, 3]*q[:, 3]-np.sum(dq[:, :3]*q[:, :3], axis=1)])
    new[new[:, 3]<0] *= -1
    new /= np.linalg.norm(new, axis=1)[:, None]
    x[valid, :4] = new
    x[valid, 4:10] += delta[valid, 3:9]
    x[~valid] = np.nan
    return x


def readlines(path, manifest):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for line in stream:
            h.update(line)
            yield json.loads(line)
    manifest[str(path.resolve())] = dict(sha256=h.hexdigest(), bytes=path.stat().st_size)


def extract(run, out, manifest):
    fields = ('camera_ns', 'initialized', 'target_imu_time', 'imu_time', 'raw_current', 'available',
              'ready_G', 'ready_V', 'observed_features', 'mature_features', 'eligible_seeds',
              'actual_corrections', 'physical_fault_count', 'bootstrap_count', 'availability_state',
              'reason', 'eta_body', 'v_body', 'grace_G', 'grace_V')
    features = [{k: r.get(k) for k in fields} for r in readlines(run/'features.jsonl', manifest)]
    events = list(readlines(run/'fusion.jsonl', manifest))
    n = len(events)
    assert len(features) == n
    prior, post = np.full((n, 16), np.nan), np.full((n, 16), np.nan)
    delta = {b: np.full((n, 9), np.nan) for b in ('G', 'V')}
    gains = {b: np.full((n, 3), np.nan) for b in ('G', 'V')}
    for i, (event, raw) in enumerate(zip_longest(events, readlines(run/'matrices.jsonl', manifest))):
        assert event is not None and raw is not None
        assert event['camera_ns'] == raw['camera_ns'] == features[i]['camera_ns']
        post[i] = np.asarray(raw['post_imu']).ravel()
        if raw['prior_imu']:
            prior[i] = np.asarray(raw['prior_imu']).ravel()
        if event['submit_reason'] != 'joint_applied':
            continue
        assert event['consumed'] and event['ekf_calls'] == 1
        K = joint_gain(raw)
        res = np.asarray(raw['joint_res']).ravel()
        start = event['visual_rows']
        for b in ('G', 'V'):
            count = event[b+'_rows']
            if count:
                block = K[:9, start:start+count]
                delta[b][i] = block@res[start:start+count]
                for j, (name, s) in enumerate(BLOCKS.items()):
                    # Gain attitude stays radians / dimensionless G or m/s V residual.
                    gains[b][i, j] = np.linalg.norm(block[s], 'fro')
            start += count
        np.testing.assert_allclose(sum(np.nan_to_num(delta[b][i]) for b in ('G', 'V')),
                                   K[:9, event['visual_rows']:]@res[event['visual_rows']:], atol=1e-12)
    arrays = dict(prior=prior, post=post)
    for b in ('G', 'V'):
        arrays['delta_'+b], arrays['gain_'+b] = delta[b], gains[b]
    np.savez_compressed(out/(run.name+'.npz'), **arrays)
    (out/(run.name+'_compact.json')).write_text(json.dumps(dict(features=features, events=events)))
    print('extracted', run.name, flush=True)
    return features, events, arrays


def intervals(mask, t):
    ix = np.flatnonzero(mask)
    if not len(ix):
        return []
    cadence = np.median(np.diff(t))
    groups = np.split(ix, np.flatnonzero((np.diff(ix) != 1) | (np.diff(t[ix]) > 1.5*cadence))+1)
    return [dict(start_s=float(t[g[0]]-t[0]), end_s=float(t[g[-1]]-t[0]), frames=len(g)) for g in groups]


def labels(ready, weak):
    start = np.flatnonzero(ready)
    before = np.arange(len(ready)) < (start[0] if len(start) else len(ready))
    return np.where(before, 'startup', np.where(~ready, 'interruption', np.where(weak, 'weak_visual', 'normal')))


def compare(main, ltv, mask, ready, applied):
    valid = mask & np.isfinite(main) & np.isfinite(ltv)
    better = valid & (ltv < main)
    return dict(target_frames=int(mask.sum()), paired_frames=int(valid.sum()), missing_pairs=int((mask & ~valid).sum()),
                main=stats(main[valid]), ltv=stats(ltv[valid]), ltv_minus_main=stats((ltv-main)[valid]),
                ltv_better_frames=int(better.sum()), ltv_better_fraction=float(better.sum()/valid.sum()) if valid.any() else None,
                ready_pairs=int((valid & ready).sum()), ready_better=int((better & ready).sum()),
                actual_pairs=int((valid & applied).sum()), actual_better=int((better & applied).sum()),
                correlation=float(np.corrcoef(main[valid], ltv[valid])[0, 1]) if valid.sum()>2 and
                np.std(main[valid])>0 and np.std(ltv[valid])>0 else None)


def summarize(seq, data, gt, out):
    off_f, off_e, off_a = data['OFF']
    time = np.array([f['target_imu_time'] for f in off_f])
    init = np.array([bool(f['initialized']) for f in off_f])
    truth_v, truth_g, gt_valid = reference(gt, time)
    support = init & gt_valid
    visual = np.array([e['visual_rows'] for e in off_e])
    threshold = float(np.quantile(visual[support], .2))
    weak = visual <= threshold
    off_v, off_g = body(off_a['post'])
    base_error = dict(G=angle(off_g, truth_g), V=np.linalg.norm(off_v-truth_v, axis=1))
    masks = {'all': support, 'weak_visual_overlap': support & weak}
    fixed_labels, off_ready = {}, {}
    result = dict(target_initialized_frames=int(init.sum()), common_gt_frames=int(support.sum()),
                  missing_gt_frames=int((init & ~gt_valid).sum()), weak_visual_threshold=threshold, branches={}, modes={})
    before = nearest(time, time-1.)
    for b in ('G', 'V'):
        ready = np.array([bool(f['ready_'+b]) for f in off_f])
        off_ready[b] = ready
        lab = labels(ready & init, weak)
        fixed_labels[b] = lab
        high = support & (base_error[b] >= np.quantile(base_error[b][support], .75))
        rising = high & (abs(time[before]-(time-1.)) <= .025) & init[before] & (base_error[b] > base_error[b][before])
        branch_masks = dict(all=support, weak_visual_overlap=support & weak, high_error=high, rising_high_error=rising)
        branch_masks.update(high_error_ready_OFF=high & ready,
                            high_error_interruption_OFF=high & (lab == 'interruption'),
                            rising_high_error_ready_OFF=rising & ready,
                            rising_high_error_interruption_OFF=rising & (lab == 'interruption'))
        branch_masks.update(ready_OFF=support & ready, post_startup=support & (lab != 'startup'),
                            weak_visual_interruption=support & weak & (lab == 'interruption'))
        branch_masks.update({s: support & (lab == s) for s in ('startup', 'interruption', 'weak_visual', 'normal')})
        result['branches'][b] = dict(high_error_threshold=float(np.quantile(base_error[b][support], .75)),
            readiness_interruptions=intervals(init & (lab == 'interruption'), time),
            rising_high_error_intervals=intervals(rising, time), masks={k:int(v.sum()) for k,v in branch_masks.items()})
        masks[b] = branch_masks
    curve_rows = []
    for mode, (f, e, a) in data.items():
        assert [x['camera_ns'] for x in f] == [x['camera_ns'] for x in off_f]
        assert np.max(abs(np.array([x['target_imu_time'] for x in f])-time)) <= 1e-6
        lv = np.array([x['v_body'] if x['v_body'] is not None else [np.nan]*3 for x in f], float)
        lg = np.array([x['eta_body'] if x['eta_body'] is not None else [np.nan]*3 for x in f], float)
        current = np.array([bool(x['raw_current']) and abs(x['imu_time']-time[i])<=1e-6 for i,x in enumerate(f)])
        lv[~current] = np.nan
        lg[~current] = np.nan
        mv, mg = body(a['post'])
        pv, pg = body(a['prior'])
        errors = dict(G=(angle(mg, truth_g), angle(lg, truth_g), angle(pg, truth_g)),
                      V=(np.linalg.norm(mv-truth_v, axis=1), np.linalg.norm(lv-truth_v, axis=1),
                         np.linalg.norm(pv-truth_v, axis=1)))
        mode_result = {}
        for b in ('G', 'V'):
            main, ltv, prior = errors[b]
            ready = np.array([bool(x['ready_'+b]) for x in f])
            actual = np.array([bool(x['consumed']) and x[b+'_rows']>0 and
                               x['submit_reason']=='joint_applied' and x['ekf_calls']==1 for x in e])
            attempted = np.array([x[b+'_nis'] >= 0 for x in e])
            cv, cg = body(inject(a['prior'], a['delta_'+b]))
            contribution_error = angle(cg, truth_g) if b=='G' else np.linalg.norm(cv-truth_v, axis=1)
            residual = np.array([np.linalg.norm(np.asarray(x[b+'_residual'])) if x[b+'_residual'] else np.nan for x in e])
            nis = np.array([x[b+'_nis'] if x[b+'_nis']>=0 else np.nan for x in e])
            disag = angle(lg, pg) if b=='G' else np.linalg.norm(lv-pv, axis=1)
            if mode == 'OFF':
                disag = angle(lg, mg) if b=='G' else np.linalg.norm(lv-mv, axis=1)
            mode_result[b] = {}
            for name, mask in masks[b].items():
                mode_result[b][name] = dict(post_comparison=compare(main, ltv, mask, ready, actual),
                    prior_comparison=compare(prior, ltv, mask, ready, actual), ready_frames=int((mask & ready).sum()),
                    actual_prior_comparison=compare(prior, ltv, mask & actual, ready, actual),
                    actual_frames=int((mask & actual).sum()), attempts=int((mask & attempted).sum()),
                    residual_attempted=stats(residual[mask & attempted]), residual_actual=stats(residual[mask & actual]),
                    nis_attempted=stats(nis[mask & attempted]), nis_actual=stats(nis[mask & actual]),
                    ltv_main_disagreement=stats(disag[mask]), corrections={}, gains={})
                direction_mask = mask & actual & np.isfinite(prior) & np.isfinite(contribution_error)
                change = contribution_error-prior
                mode_result[b][name]['branch_only_error_change'] = stats(change[direction_mask])
                mode_result[b][name]['branch_only_improves_frames'] = int((direction_mask & (change < 0)).sum())
                for j, (block, s) in enumerate(BLOCKS.items()):
                    value = np.linalg.norm(a['delta_'+b][:, s], axis=1)
                    if block == 'attitude_deg':
                        value *= 180/np.pi
                    mode_result[b][name]['corrections'][block] = stats(value[mask & actual])
                    mode_result[b][name]['gains'][block.replace('attitude_deg', 'attitude_rad')] = stats(a['gain_'+b][mask & actual, j])
            for i in np.flatnonzero(support):
                row = dict(mode=mode, branch=b, camera_ns=f[i]['camera_ns'], imu_time=time[i], elapsed_s=time[i]-time[0],
                    stratum=fixed_labels[b][i], weak_visual=int(weak[i]), visual_rows=int(visual[i]),
                    high_error=int(masks[b]['high_error'][i]), rising_high_error=int(masks[b]['rising_high_error'][i]),
                    main_error=main[i], ltv_error=ltv[i], prior_error=prior[i], ready=int(ready[i]), actual=int(actual[i]),
                    residual_norm=residual[i], nis=nis[i], observed_features=f[i]['observed_features'],
                    branch_only_error_change=contribution_error[i]-prior[i],
                    eligible_seeds=f[i]['eligible_seeds'], reason=f[i]['reason'], availability_state=f[i]['availability_state'])
                for j, (block, s) in enumerate(BLOCKS.items()):
                    vector = a['delta_'+b][i, s]*(180/np.pi if block=='attitude_deg' else 1.)
                    row['correction_'+block] = np.linalg.norm(vector)
                    row['gain_'+block.replace('attitude_deg', 'attitude_rad')] = a['gain_'+b][i, j]
                    for axis, value in zip('xyz', vector):
                        row['correction_'+block+'_'+axis] = value
                rawres = np.asarray(e[i][b+'_residual']).ravel()
                for j in range(3):
                    row['residual_'+str(j)] = rawres[j] if j<len(rawres) else np.nan
                curve_rows.append(row)
        result['modes'][mode] = mode_result
    with (out/(seq+'_curves.csv')).open('w') as stream:
        w = csv.DictWriter(stream, fieldnames=list(curve_rows[0]))
        w.writeheader()
        w.writerows(curve_rows)
    return result


def main():
    p = argparse.ArgumentParser()
    p.add_argument('evidence', type=Path)
    p.add_argument('out', type=Path)
    p.add_argument('--reuse-derived', action='store_true')
    args = p.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    manifest_path = args.out/'read_manifest.json'
    manifest = json.loads(manifest_path.read_text()) if args.reuse_derived else {}
    freeze = json.loads((args.evidence/'freeze.json').read_text())
    results = {}
    for seq in SEQUENCES:
        data = {}
        for mode in ('OFF', 'G', 'V', 'GV'):
            run = args.evidence/'final_runs'/f'{seq}_{mode}_01'
            if args.reuse_derived and (args.out/(run.name+'_compact.json')).exists() and (args.out/(run.name+'.npz')).exists():
                compact = json.loads((args.out/(run.name+'_compact.json')).read_text())
                data[mode] = compact['features'], compact['events'], dict(np.load(args.out/(run.name+'.npz')))
            else:
                data[mode] = extract(run, args.out, manifest)
                manifest_path.write_text(json.dumps(manifest, indent=2)+'\n')
        gtpath = Path(freeze['inputs'][seq]['root'])/'state_groundtruth_estimate0/data.csv'
        manifest[str(gtpath)] = dict(sha256=hashlib.sha256(gtpath.read_bytes()).hexdigest(), bytes=gtpath.stat().st_size)
        results[seq] = summarize(seq, data, np.loadtxt(gtpath, delimiter=',', comments='#'), args.out)
    (args.out/'summary.json').write_text(json.dumps(results, indent=2, allow_nan=False)+'\n')
    manifest_path.write_text(json.dumps(manifest, indent=2)+'\n')
    print('summary complete', flush=True)


if __name__ == '__main__':
    main()
