"""Frozen V20/V10 evaluation: admission, recovery, physical quality and trajectories."""
import argparse
from collections import Counter
import csv
from itertools import zip_longest
import json
from pathlib import Path
import sys

import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT/'docs/experiments/tools/ltv_gv_diagnosis'))
import analyze as diag
sys.path.insert(0, str(ROOT/'docs/experiments/tools/ltv_gv_evaluation'))
import evaluate as ev
from run_matrix import write, exact


def applied(e):
    return bool(e['consumed'] and e['V_rows']==3 and e['submit_reason']=='joint_applied' and e['ekf_calls']==1)


def partition(ready, time):
    first = np.flatnonzero(ready)
    labels = np.full(len(ready), 'startup', dtype='U8')
    episodes = []
    if not len(first):
        return labels, episodes
    labels[first[0]:] = 'normal'
    start = None
    for i in range(first[0]+1, len(ready)):
        if not ready[i]:
            if start is None:
                start = i
            labels[i] = 'recovery'
        elif start is not None:
            labels[start:np.searchsorted(time, time[i]+1., side='right')] = 'recovery'
            episodes.append((start, i))
            start = None
    if start is not None:
        labels[start:] = 'recovery'
        episodes.append((start, None))
    return labels, episodes


def toggles(ready, time, init):
    ix = np.flatnonzero(init)
    r, t = ready[ix], time[ix]
    change = np.diff(np.r_[False, r, False].astype(int))
    starts, ends = np.flatnonzero(change==1), np.flatnonzero(change==-1)
    cadence = np.median(np.diff(time))
    duration = np.array([t[b-1]-t[a]+cadence for a,b in zip(starts, ends)])
    return dict(ready_frames=int(r.sum()), ready_on_transitions=len(starts),
                ready_off_transitions=int(np.sum(r[:-1] & ~r[1:])),
                transitions_per_minute=float(np.sum(r[1:] != r[:-1])*60/(t[-1]-t[0])),
                ready_segments_under_1s=int(np.sum(duration<1.)), ready_segment_duration_s=diag.stats(duration),
                first_ready_elapsed_s=float(t[starts[0]]-time[0]) if len(starts) else None)


def quality(errors, mask):
    valid = mask & np.isfinite(errors)
    values = errors[valid]
    return dict(target=int(mask.sum()), evaluated=int(valid.sum()), missing=int((mask & ~valid).sum()),
                error_mps=diag.stats(values), tails={str(v):dict(count=int(np.sum(values>v)),
                fraction=float(np.mean(values>v)) if len(values) else None) for v in (.1,.2,1.)})


def deterioration(base, additional):
    b, a = base['error_mps'], additional['error_mps']
    return dict(rmse_25pct_and_0_01=a['rms'] is not None and a['rms']>1.25*b['rms'] and a['rms']-b['rms']>.01,
                p95_plus_0_05=a['p95'] is not None and a['p95']-b['p95']>.05,
                tail_0_2_plus_5pp=additional['tails']['0.2']['fraction'] is not None and
                additional['tails']['0.2']['fraction']-base['tails']['0.2']['fraction']>.05)


def trajectory_regions(out, seq, mode, inp, labels, times):
    full, curve = ev.metrics(out, seq, mode, inp)
    if curve is None:
        return {'all': full}, None, np.empty(0, dtype='U8')
    run = out/'final_runs'/f'{seq}_{mode}_01'
    trajectory = ev.trajectory(run/'trajectory.csv')
    t = curve['time']
    idx = diag.nearest(times, t)
    assert np.max(abs(times[idx]-t)) <= 1e-6
    region = labels[idx]
    ji = diag.nearest(trajectory[:,0], t)
    raw = np.loadtxt(Path(inp['root'])/'state_groundtruth_estimate0/data.csv', delimiter=',', comments='#')
    gi = diag.nearest(raw[:,0]*1e-9, t)
    gt, gtq = raw[gi,1:4], raw[gi][:,[5,6,7,4]]
    est = trajectory[ji]
    re, rg = Rotation.from_quat(est[:,1:5]), Rotation.from_quat(gtq)
    result = {'all': full}
    discontinuities = np.r_[0, np.cumsum(region[1:] != region[:-1])]
    for name in ('normal','recovery','startup'):
        mask = region==name
        r = dict(samples=int(mask.sum()), ate_rmse_m=diag.stats(curve['error'][mask])['rms'],
                 rotation_rmse_deg=diag.stats(curve['rotation'][mask])['rms'], rpe={})
        for dt in (1.,5.):
            end = diag.nearest(t, t+dt)
            ok = (abs(t[end]-(t+dt))<=.025) & mask & (region[end]==name) & (discontinuities[end]==discontinuities)
            a, b = np.flatnonzero(ok), end[ok]
            de = re[a].inv().apply(est[b,5:8]-est[a,5:8]) if len(a) else np.empty((0,3))
            dg = rg[a].inv().apply(gt[b]-gt[a]) if len(a) else np.empty((0,3))
            dr = ((rg[a].inv()*rg[b]).inv()*(re[a].inv()*re[b])).magnitude() if len(a) else np.empty(0)
            r['rpe'][str(dt)] = dict(pairs=len(a), translation_rmse_m=diag.stats(np.linalg.norm(de-dg,axis=1))['rms'],
                                      rotation_rmse_deg=diag.stats(np.degrees(dr))['rms'])
        result[name] = r
    return result, curve, region


def check_health(run, manifest):
    checked = grace = 0
    for f in diag.readlines(run/'features.jsonl', manifest):
        if not f['ready_V']:
            continue
        assert f['raw_current'] and f['pool_ready'] and f['observer_valid']
        assert f['observed_features']>=15 and f['mature_features']>=15 and f['actual_corrections']>=20
        assert f['correction_diagnostics_valid'] and abs(f['imu_time']-f['target_imu_time'])<=1e-9
        assert 8.<=np.linalg.norm(f['eta_body'])<=11.5
        factor = 2. if f['grace_V'] else 1.
        assert 0<=f['prediction_angle_p95_rad']<=factor*.02
        assert 0<=f['velocity_correction_rate']<=factor*.5 and 0<=f['gravity_correction_rate']<=factor*1.
        if f['grace_V']:
            grace += 1
            assert f['soft_failure_frames_V']<=2 and f['imu_time']-f['last_strict_good_V']<=.10+1e-6
        checked += 1
    return dict(ready_frames_checked=checked, existing_grace_frames=grace, all_original_hard_health_bounds_pass=True)


def main():
    p = argparse.ArgumentParser()
    p.add_argument('out', type=Path)
    p.add_argument('--reuse-derived', action='store_true')
    args = p.parse_args()
    out = args.out.resolve()
    previous = Path('/home/he/output/ltv_gv_no_benefit_20261005')
    frozen = json.loads((out/'freeze.json').read_text())
    manifest = json.loads((out/'analysis_manifest.json').read_text()) if args.reuse_derived else {}
    summaries = {}
    for seq, inp in frozen['inputs'].items():
        name = seq+'_V_01'
        previous_delivery = json.loads((previous/'delivery.json').read_text())['derived_external_files']
        for filename in (name+'_compact.json', name+'.npz'):
            path = previous/filename
            assert exact.sha(path)==previous_delivery[filename]['sha256']
            manifest[str(path)] = dict(sha256=exact.sha(path), bytes=path.stat().st_size)
        old = json.loads((previous/(name+'_compact.json')).read_text())
        f20, e20, a20 = old['features'], old['events'], dict(np.load(previous/(name+'.npz')))
        run = out/'final_runs'/f'{seq}_V10_01'
        if args.reuse_derived:
            short = json.loads((out/(run.name+'_compact.json')).read_text())
            f10, e10, a10 = short['features'], short['events'], dict(np.load(out/(run.name+'.npz')))
        else:
            f10, e10, a10 = diag.extract(run, out, manifest)
        assert len(f20)==len(f10) and [f['camera_ns'] for f in f20]==[f['camera_ns'] for f in f10]
        assert all(not e['requested_G'] and not e['G_rows'] for e in e10+e20)
        t = np.array([f['target_imu_time'] for f in f20])
        assert np.max(abs(t-np.array([f['target_imu_time'] for f in f10])))<=1e-6
        init = np.array([bool(f['initialized']) for f in f20])
        ready20, ready10 = [np.array([bool(f['ready_V']) for f in fs]) for fs in (f20,f10)]
        actual20, actual10 = [np.array([applied(e) for e in es]) for es in (e20,e10)]
        labels, episodes = partition(ready20, t)
        added, lost = actual10 & ~actual20, actual20 & ~actual10
        gtpath = Path(inp['root'])/'state_groundtruth_estimate0/data.csv'
        truth_v, truth_g, gt_valid = diag.reference(np.loadtxt(gtpath, delimiter=',', comments='#'), t)
        manifest[str(gtpath)] = dict(sha256=exact.sha(gtpath), bytes=gtpath.stat().st_size)
        error, prior_error, change = {}, {}, {}
        for mode, fs, arrays in [('V20',f20,a20),('V10',f10,a10)]:
            lv = np.array([f['v_body'] if f['v_body'] is not None else [np.nan]*3 for f in fs])
            current = np.array([bool(f['raw_current']) and abs(f['imu_time']-t[i])<=1e-6 for i,f in enumerate(fs)])
            lv[~current] = np.nan
            pv, pg = diag.body(arrays['prior'])
            cv, cg = diag.body(diag.inject(arrays['prior'], arrays['delta_V']))
            error[mode] = np.linalg.norm(lv-truth_v, axis=1)
            prior_error[mode] = np.linalg.norm(pv-truth_v, axis=1)
            change[mode] = np.linalg.norm(cv-truth_v, axis=1)-prior_error[mode]
        admission = dict(original_actual=quality(error['V20'], actual20), short_actual=quality(error['V10'],actual10),
                         added_short=quality(error['V10'],added), added_original_same_time=quality(error['V20'],added),
                         lost_original=quality(error['V20'],lost))
        base, additional = admission['original_actual'], admission['added_short']
        deteriorates = deterioration(base, additional)
        recoveries = []
        for start, end in episodes:
            bound = len(t) if end is None else end+1
            candidates = np.flatnonzero(ready10[start:bound])+start
            early = candidates[0] if len(candidates) else None
            actual_candidates = np.flatnonzero(actual10[start:bound])+start
            early_actual = actual_candidates[0] if len(actual_candidates) else None
            recoveries.append(dict(start_camera_ns=f20[start]['camera_ns'], start_elapsed_s=float(t[start]-t[0]),
                V20_ready_elapsed_s=float(t[end]-t[0]) if end is not None else None,
                V10_first_ready_elapsed_s=float(t[early]-t[0]) if early is not None else None,
                V20_wait_s=float(t[end]-t[start]) if end is not None else None,
                V10_wait_s=float(t[early]-t[start]) if early is not None else None,
                shortened_s=float(t[end]-t[early]) if end is not None and early is not None else None,
                V10_first_actual_elapsed_s=float(t[early_actual]-t[0]) if early_actual is not None else None,
                actual_shortened_s=float(t[end]-t[early_actual]) if end is not None and early_actual is not None else None,
                V10_already_ready_at_anchor=bool(ready10[start]), V20_right_censored=end is None,
                V10_off_transitions_before_V20_return=int(np.sum(ready10[start:bound-1] & ~ready10[start+1:bound]))))
        trajectory, curves = {}, {}
        for mode in ('OFF','V','V10'):
            trajectory[mode], curves[mode], regions = trajectory_regions(out,seq,mode,inp,labels,t)
        direction_mask = added & np.isfinite(change['V10'])
        mode_masks = dict(all=init, normal=init & (labels=='normal'), recovery=init & (labels=='recovery'), startup=init & (labels=='startup'))
        phase_quality = {name:dict(V20=quality(error['V20'], mask & actual20), V10=quality(error['V10'],mask & actual10),
                       added=quality(error['V10'],mask & added)) for name, mask in mode_masks.items()}
        summary = dict(all_camera_frames=len(t), common_GT_frames=int((init & gt_valid).sum()),
            actual_V20=int(actual20.sum()), actual_V10=int(actual10.sum()), added_actual=int(added.sum()), lost_actual=int(lost.sum()),
            added_intervals=diag.intervals(added,t), lost_intervals=diag.intervals(lost,t), admissions=admission,
            phase_quality=phase_quality, V20_toggles=toggles(ready20,t,init), V10_toggles=toggles(ready10,t,init),
            recoveries=recoveries, paired_recovery_shortening_s=diag.stats([r['shortened_s'] for r in recoveries if r['shortened_s'] is not None]),
            paired_actual_recovery_shortening_s=diag.stats([r['actual_shortened_s'] for r in recoveries if r['actual_shortened_s'] is not None]),
            V10_ready_without_actual=dict(Counter(e10[i]['V_reason'] for i in np.flatnonzero(ready10 & ~actual10))),
            added_direction=dict(evaluated=int(direction_mask.sum()), improves=int((direction_mask & (change['V10']<0)).sum()),
                                 error_change_mps=diag.stats(change['V10'][direction_mask]),
                                 main_prior_error_mps=diag.stats(prior_error['V10'][direction_mask]),
                                 ltv_better_than_prior=int((direction_mask & (error['V10']<prior_error['V10'])).sum())),
            deterioration_flags=deteriorates, quality_decision='RETAIN_20' if any(deteriorates.values()) else 'NO_PREDEFINED_QUALITY_DETERIORATION',
            health=check_health(run,manifest), trajectory=trajectory,
            added_corrections={name:diag.stats(np.linalg.norm(a10['delta_V'][added,s],axis=1)*(180/np.pi if name=='attitude_deg' else 1.)) for name,s in diag.BLOCKS.items()})
        first = next((i for i,(a,b) in enumerate(zip(e20,e10)) if a['main_digest']!=b['main_digest'] or a['covariance_digest']!=b['covariance_digest']),None)
        assert first is not None and added[first]
        summary['first_main_divergence'] = dict(camera_ns=f20[first]['camera_ns'], added_actual=True)
        assert not np.any(actual10 & ~ready10)
        rows = []
        for i in np.flatnonzero(init):
            r = dict(camera_ns=f20[i]['camera_ns'], imu_time=t[i], elapsed_s=t[i]-t[0], phase=labels[i],
                ready_V20=int(ready20[i]), ready_V10=int(ready10[i]), actual_V20=int(actual20[i]), actual_V10=int(actual10[i]),
                added=int(added[i]), lost=int(lost[i]), grace_V10=int(f10[i]['grace_V']),
                V20_ltv_error_mps=error['V20'][i], V10_ltv_error_mps=error['V10'][i], V10_prior_error_mps=prior_error['V10'][i],
                V10_branch_error_change_mps=change['V10'][i], observed=f10[i]['observed_features'], mature=f10[i]['mature_features'],
                reason=f10[i]['reason'], V10_NIS=e10[i]['V_nis'])
            for block,slice_ in diag.BLOCKS.items():
                v = a10['delta_V'][i,slice_]*(180/np.pi if block=='attitude_deg' else 1.)
                r['correction_'+block] = np.linalg.norm(v)
                for axis,value in zip('xyz',v):
                    r['correction_'+block+'_'+axis] = value
            rows.append(r)
        with (out/(seq+'_admissions.csv')).open('w') as stream:
            w=csv.DictWriter(stream,fieldnames=list(rows[0]),lineterminator='\n');w.writeheader();w.writerows(rows)
        for mode,curve in curves.items():
            if curve is None:
                continue
            with (out/(seq+'_'+mode+'_trajectory_curve.csv')).open('w') as stream:
                w=csv.writer(stream,lineterminator='\n');w.writerow(['time','phase','translation_error_m','rotation_error_deg'])
                w.writerows(zip(curve['time'],regions,curve['error'],curve['rotation']))
        summaries[seq] = summary
        print(seq, summary['actual_V20'],summary['actual_V10'],summary['added_actual'],summary['quality_decision'],flush=True)
    write(out/'analysis_manifest.json',manifest)
    write(out/'summary.json',summaries)


if __name__ == '__main__':
    main()
