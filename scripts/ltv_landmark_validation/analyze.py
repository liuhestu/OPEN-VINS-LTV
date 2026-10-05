"""Paired pre-correction bearing prediction study; no GT or online selection."""
import argparse
from collections import Counter
import csv
import importlib.util
import json
from pathlib import Path
import sys
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'scripts/ltv_gv_evaluation'))
import evaluate as trajectory_eval
from run import sha, write


def labels(features):
    """Frozen OFF readiness interruption plus 1s after return; startup is explicit."""
    result = {}
    seen, previous, recovery_until = False, False, -np.inf
    for f in features:
        ready = bool(f['ready_V'])
        t = f['camera_ns'] * 1e-9
        if not seen and not ready:
            phase = 'startup'
        else:
            if ready and not previous and seen:
                recovery_until = t + 1.
            phase = 'recovery' if seen and (not ready or t <= recovery_until) else 'normal'
        result[int(f['camera_ns'])] = phase
        seen |= ready
        previous = ready
    return result


def match_times(prediction_ns, reference_ns):
    reference_ns=np.asarray(reference_ns,dtype=np.int64)
    prediction_ns=np.asarray(prediction_ns,dtype=np.int64)
    hi=np.searchsorted(reference_ns,prediction_ns).clip(0,len(reference_ns)-1)
    lo=(hi-1).clip(0)
    index=np.where(abs(reference_ns[lo]-prediction_ns)<=abs(reference_ns[hi]-prediction_ns),lo,hi)
    matched=reference_ns[index]
    if np.any(abs(matched-prediction_ns)>1000) or len(np.unique(matched))!=len(prediction_ns):
        raise ValueError('Prediction/reference timeline mismatch or duplicate match')
    return matched


def angular_stats(x):
    x = np.asarray(x, float)
    x = x[np.isfinite(x)]
    if not len(x):
        return dict(n=0, rms=None, median=None, p95=None, p99=None, maximum=None, above_0_02_rad=None)
    return dict(n=len(x), rms=float(np.sqrt(np.mean(x*x))), median=float(np.median(x)),
                p95=float(np.quantile(x, .95)), p99=float(np.quantile(x, .99)), maximum=float(x.max()),
                above_0_02_rad=int((x > .02).sum()))


def paired_group(rows, mask, bootstrap=2000):
    mask = np.asarray(mask, bool)
    lv = np.array([r['ltv_valid'] == '1' for r in rows])
    mv = np.array([r['main_valid'] == '1' for r in rows])
    # A past shared anchor is required for either predictor to enter this comparison.
    anchor = np.array([np.isfinite(float(r['anchor_time'])) for r in rows])
    lv &= anchor
    mv &= anchor
    paired = mask & lv & mv
    n = int(mask.sum())
    result = dict(denominator=n, ltv_valid=int((mask & lv).sum()), main_valid=int((mask & mv).sum()),
                  paired=int(paired.sum()), ltv_only=int((mask & lv & ~mv).sum()), main_only=int((mask & mv & ~lv).sum()),
                  neither=int((mask & ~lv & ~mv).sum()),
                  reasons=dict(Counter(r['reason'] for r, keep in zip(rows, mask) if keep)))
    result['ltv_coverage'] = result['ltv_valid']/n if n else None
    result['main_coverage'] = result['main_valid']/n if n else None
    indices = np.flatnonzero(paired)
    l = np.array([float(rows[i]['ltv_angle_rad']) for i in indices])
    m = np.array([float(rows[i]['main_angle_rad']) for i in indices])
    result.update(ltv=angular_stats(l), main=angular_stats(m))
    # Equal frame weights prevent feature-rich frames from dominating the primary criterion.
    frame = {}
    for i, le, me in zip(indices, l, m):
        frame.setdefault(int(rows[i]['camera_ns']), []).append((le*le, me*me))
    times = np.array(sorted(frame), dtype=np.int64)
    frame_mse = np.array([np.mean(frame[int(t)], axis=0) for t in times]).reshape(-1, 2)
    result['paired_frames'] = len(times)
    result['gain_ci95'] = None
    result['frame_rms_gain'] = None
    result['reliable_advantage'] = False
    if len(times):
        means = frame_mse.mean(0)
        result['frame_rms_ltv'] = float(np.sqrt(means[0]))
        result['frame_rms_main'] = float(np.sqrt(means[1]))
        result['frame_rms_gain'] = float(1 - np.sqrt(means[0]/means[1])) if means[1] else None
        blocks = (times - times[0]) // 1000000000
        unique = np.unique(blocks)
        result['time_blocks'] = len(unique)
        # Each resampled block retains its number of original frame samples.
        sums = np.array([frame_mse[blocks == b].sum(0) for b in unique])
        counts = np.array([(blocks == b).sum() for b in unique])
        if len(times) >= 30 and len(unique) >= 10 and means[1] > 0:
            rng = np.random.default_rng(20261005)
            samples = rng.integers(0, len(unique), size=(bootstrap, len(unique)))
            draw = sums[samples].sum(1)/counts[samples].sum(1)[:, None]
            gains = 1 - np.sqrt(draw[:, 0]/draw[:, 1])
            result['gain_ci95'] = list(map(float, np.quantile(gains, [.025, .975])))
            tail_ok = result['ltv']['p95'] <= 1.05*result['main']['p95'] and result['ltv']['p99'] <= 1.05*result['main']['p99']
            coverage_ok = result['ltv_valid'] >= result['main_valid']
            result.update(tail_nonworse=bool(tail_ok), coverage_nonworse=coverage_ok)
            result['reliable_advantage'] = bool(result['frame_rms_gain'] >= .05 and result['gain_ci95'][0] > 0 and tail_ok and coverage_ok)
    return result


def assess(out):
    frozen = json.loads((out / 'freeze.json').read_text())
    results = {}
    for seq, inp in frozen['inputs'].items():
        old = Path(inp['off_path'])
        assert sha(old / 'features.jsonl') == inp['off_evidence_sha']['features.jsonl']
        features = [json.loads(s) for s in (old / 'features.jsonl').read_text().splitlines()]
        phase_by_time = labels(features)
        with (out / f'{seq}_predictions.csv').open() as stream:
            rows = list(csv.DictReader(stream))
        raw_times=sorted({int(r['camera_ns']) for r in rows})
        reference=sorted(phase_by_time)
        matched=match_times(raw_times,reference)
        matched_by_raw=dict(zip(raw_times,map(int,matched)))
        for r in rows:
            r['support_camera_ns']=matched_by_raw[int(r['camera_ns'])]
            if np.isfinite(float(r['anchor_time'])):
                assert float(r['anchor_time']) < float(r['imu_time']) - 1e-9
            r['phase'] = phase_by_time[r['support_camera_ns']]
        mature = np.array([r['past_mature'] == '1' for r in rows])
        phase = np.array([r['phase'] for r in rows])
        ready = np.array([r['past_ready_V'] == '1' for r in rows])
        span = np.array([float(r['anchor_span']) for r in rows])
        groups = {'all_candidates': np.ones(len(rows), bool), 'primary_mature': mature}
        for name in ['startup', 'normal', 'recovery']:
            groups[name] = mature & (phase == name)
        groups.update(past_ready=mature & ready, past_not_ready=mature & ~ready,
                      immature=~mature, anchor_short=mature & (span <= .200000001), anchor_long=mature & (span > .200000001))
        group_results = {name: paired_group(rows, mask) for name, mask in groups.items()}
        metrics, curve = trajectory_eval.metrics(Path('/home/he/output/ltv_gv_fusion_evaluation_20261005'), seq, 'OFF', inp)
        captured={r['support_camera_ns'] for r in rows}
        initialized=[f for f in features if f['initialized']]
        missing=[f for f in initialized if int(f['camera_ns']) not in captured]
        result = dict(frame_support=dict(max_clock_mapping_delta_ns=int(max(abs(matched-np.array(raw_times,dtype=np.int64)),default=0)),original_camera_frames=len(features),initialized_frames=len(initialized),frames_with_candidates=len(captured),initialized_frames_without_candidate_rows=len(missing),missing_reasons=dict(Counter(f['reason'] for f in missing))),groups=group_results, rows=len(rows), exact=json.loads((out / f'{seq}_exact.json').read_text()),
                      baseline_trajectory=metrics, actual_landmark_updates=0, fusion_executed=False,
                      covariance_change=0, main_state_change=0, observer_change=0,
                      shadow_csv_sha=sha(out / f'{seq}_predictions.csv'))
        results[seq] = result
        columns = ['camera_ns','phase','candidates','mature','paired','ltv_frame_rms_rad','main_frame_rms_rad']
        frame_rows = {}
        for r in rows:
            frame_rows.setdefault(int(r['camera_ns']), []).append(r)
        with (out / f'{seq}_prediction_curve.csv').open('w') as stream:
            w = csv.writer(stream); w.writerow(columns)
            for ns, rs in sorted(frame_rows.items()):
                ms = [r for r in rs if r['past_mature'] == '1']
                ps = [r for r in ms if r['ltv_valid'] == r['main_valid'] == '1' and np.isfinite(float(r['anchor_time']))]
                vals = [np.sqrt(np.mean([float(r[k])**2 for r in ps])) if ps else '' for k in ['ltv_angle_rad','main_angle_rad']]
                w.writerow([ns, rs[0]['phase'],len(rs),len(ms),len(ps),*vals])
    eligible_groups = ['primary_mature','past_ready','past_not_ready','anchor_short','anchor_long']
    supported = [name for name in eligible_groups if all(r['groups'][name]['reliable_advantage'] for r in results.values())]
    only_local = {seq:[name for name in eligible_groups if r['groups'][name]['reliable_advantage']] for seq,r in results.items()}
    decision = 'CONTINUE_CORRELATION_AUDIT' if supported else ('REVIEW_LOCAL_ADVANTAGE' if any(only_local.values()) else 'STOP_NO_RELIABLE_PREDICTION_ADVANTAGE')
    summary = dict(baseline=frozen['landmark_baseline'], protocol=frozen['protocol'], sequences=results,
                   supported_online_groups=supported, exploratory_local_groups=only_local, decision=decision,
                   correlation='not modeled; Observer P is not asserted to be calibrated error covariance',
                   gt_used_for_prediction=False, gt_used_online=False)
    write(out / 'summary.json', summary)
    return summary


if __name__ == '__main__':
    p=argparse.ArgumentParser();p.add_argument('out',type=Path)
    args=p.parse_args()
    print(assess(args.out.resolve())['decision'])
