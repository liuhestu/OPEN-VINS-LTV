#!/usr/bin/env python3
"""New matched four-mode metrics; fixed OFF support and full-run SE3 alignment."""
import argparse
from collections import Counter
import csv
import hashlib
import json
from pathlib import Path
import sys

import numpy as np
from scipy.spatial.transform import Rotation
ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'scripts/ltv_gv_evaluation'))
from evaluate import align, nearest
sys.path.insert(0, str(ROOT / 'scripts/ltv_gv_diagnosis'))
from analyze import reference, body, labels


def rmse(x):
    return float(np.sqrt(np.mean(np.asarray(x) ** 2))) if len(x) else None


def evaluate(run_paths, data_root, sequence, diagnostic_off=None, bootstrap=False):
    paths = {mode: Path(p) for mode, p in run_paths.items()}
    off = np.loadtxt(paths['OFF'] / 'trajectory.csv', delimiter=',', skiprows=1, ndmin=2)
    gt = np.loadtxt(Path(data_root) / 'state_groundtruth_estimate0/data.csv', delimiter=',', comments='#')
    with (paths['OFF'] / 'audit.csv').open() as f:
        audit = list(csv.DictReader(f))
    offset = json.loads((paths['OFF'] / 'effective_options.json').read_text())['calib_camimu_dt']
    sensor = np.asarray([int(r['camera_ns']) * 1e-9 + offset for r in audit])
    sensor = sensor[sensor >= off[0, 0] - 1e-6]
    gi = nearest(gt[:, 0] * 1e-9, sensor)
    valid = (sensor >= gt[0, 0] * 1e-9) & (sensor <= gt[-1, 0] * 1e-9) & (abs(gt[gi, 0] * 1e-9 - sensor) <= .02)
    times = sensor[valid]
    gp, gq = gt[gi[valid], 1:4], gt[gi[valid]][:, [5, 6, 7, 4]]
    gv, _, velocity_valid = reference(gt, times)
    rg = Rotation.from_quat(gq)
    masks = {'full': np.ones(len(times), dtype=bool)}
    # Fixed 10-second half-open bins by OFF initialization; no ON-dependent interval selection.
    segments = np.floor((times - off[0, 0]) / 10).astype(int)
    for segment in np.unique(segments):
        masks['fixed_10s_' + str(segment)] = segments == segment
    diagnostic = Path(diagnostic_off) if diagnostic_off else paths['OFF']
    features = [json.loads(line) for line in (diagnostic / 'features.jsonl').read_text().splitlines() if line]
    fusion = [json.loads(line) for line in (diagnostic / 'fusion.jsonl').read_text().splitlines() if line]
    assert len(features) == len(fusion) == len(audit), 'complete diagnostic OFF required for frozen masks'
    ft = np.asarray([f['target_imu_time'] for f in features])
    fv, fg, fvalid = reference(gt, ft)
    initialized = np.asarray([bool(f['initialized']) for f in features])
    ix = nearest(off[:, 0], ft)
    fsupport = initialized & fvalid & (abs(off[ix, 0] - ft) <= 1e-6)
    ov, og = body(off[ix, 1:])
    error = {'V': np.linalg.norm(ov - fv, axis=1),
             'G': np.degrees(np.arccos(np.clip(np.sum(og * fg, axis=1), -1, 1)))}
    visual = np.asarray([f['visual_rows'] for f in fusion])
    weak = visual <= np.quantile(visual[fsupport], .2)
    fmasks = {'weak_visual_OFF': fsupport & weak}
    before = nearest(ft, ft - 1.)
    for branch in ('G', 'V'):
        ready = np.asarray([bool(f['ready_' + branch]) for f in fusion])
        label = labels(ready & initialized, weak)
        high = fsupport & (error[branch] >= np.quantile(error[branch][fsupport], .75))
        fmasks[branch + '_rising_high_OFF'] = high & (abs(ft[before] - (ft - 1.)) <= .025) & initialized[before] & (error[branch] > error[branch][before])
        fmasks[branch + '_interruption_OFF'] = fsupport & (label == 'interruption')
    findex = nearest(ft, times)
    for name, mask in fmasks.items():
        masks[name] = mask[findex]
    result, events, full_errors = [], [], {}
    for mode in ('OFF', 'G', 'V', 'GV'):
        path = paths[mode]
        replay = json.loads((path / 'replay.json').read_text())
        assert replay['complete'] and replay['short_limit_seconds'] == 0
        assert replay['camera_packets'] == replay['input_camera_packets'] == len(audit)
        assert replay['imu_consumed'] == replay['input_imu_samples']
        x = np.loadtxt(path / 'trajectory.csv', delimiter=',', skiprows=1, ndmin=2)
        assert x.shape[1] == 17 and np.isfinite(x).all()
        assert len(x) == len(off) and np.array_equal(x[:, 0], off[:, 0]), 'complete matching output required'
        xi = nearest(x[:, 0], times)
        assert np.all(abs(x[xi, 0] - times) <= 1e-6)
        est = x[xi]
        r, trans = align(gp, est[:, 5:8])
        re = Rotation.from_quat(est[:, 1:5])
        pe = np.linalg.norm(est[:, 5:8] @ r.T + trans - gp, axis=1)
        ae = np.degrees((rg.inv() * (Rotation.from_matrix(r) * re)).magnitude())
        ve = np.linalg.norm(re.inv().apply(est[:, 8:11]) - gv, axis=1)
        full_errors[mode] = {'ATE_m': (times, pe), 'attitude_deg': (times, ae),
                             'velocity_mps': (times[velocity_valid], ve[velocity_valid])}
        for region, mask in masks.items():
            metrics = {'ATE_m': pe[mask], 'attitude_deg': ae[mask], 'velocity_mps': ve[mask & velocity_valid]}
            for dt in (1., 5.):
                end = nearest(times, times + dt)
                ok = (abs(times[end] - (times + dt)) <= .025) & mask & mask[end]
                a, b = np.flatnonzero(ok), end[ok]
                if not len(a):
                    metrics[f'RPE_{int(dt)}s_m'] = np.empty(0)
                    metrics[f'RPE_{int(dt)}s_deg'] = np.empty(0)
                    continue
                de = re[a].inv().apply(est[b, 5:8] - est[a, 5:8])
                dg = rg[a].inv().apply(gp[b] - gp[a])
                dr = ((rg[a].inv() * rg[b]).inv() * (re[a].inv() * re[b])).magnitude()
                metrics[f'RPE_{int(dt)}s_m'] = np.linalg.norm(de - dg, axis=1)
                metrics[f'RPE_{int(dt)}s_deg'] = np.degrees(dr)
                if region == 'full':
                    full_errors[mode][f'RPE_{int(dt)}s_m'] = (times[a], metrics[f'RPE_{int(dt)}s_m'])
                    full_errors[mode][f'RPE_{int(dt)}s_deg'] = (times[a], metrics[f'RPE_{int(dt)}s_deg'])
            for metric, values in metrics.items():
                result.append(dict(sequence=sequence, mode=mode, region=region, metric=metric, value=rmse(values), samples=len(values)))
        with (path / 'ltv.csv').open() as f:
            logs = list(csv.DictReader(f))
        events.append(dict(sequence=sequence, mode=mode, G_reasons=dict(Counter(z['G_reason'] for z in logs)),
                           V_reasons=dict(Counter(z['V_reason'] for z in logs)),
                           input=replay, bias_first=x[0, 11:17].tolist(), bias_last=x[-1, 11:17].tolist()))
    if bootstrap:
        rng = np.random.default_rng(20261008)
        effects = []
        for mode in ('G', 'V', 'GV'):
            for metric, (mt, values) in full_errors[mode].items():
                ot, base = full_errors['OFF'][metric]
                assert np.array_equal(mt, ot) and len(base) == len(values)
                blocks = np.floor((mt - off[0, 0]) / 1.).astype(int)
                _, inverse = np.unique(blocks, return_inverse=True)
                counts = np.bincount(inverse)
                sse_off = np.bincount(inverse, weights=base ** 2)
                sse_on = np.bincount(inverse, weights=values ** 2)
                draw = rng.integers(0, len(counts), size=(2000, len(counts)))
                weights = counts[draw].sum(axis=1)
                delta = np.sqrt(sse_on[draw].sum(axis=1) / weights) - np.sqrt(sse_off[draw].sum(axis=1) / weights)
                effects.append({'mode': mode, 'metric': metric, 'delta_rmse': rmse(values) - rmse(base),
                                'ci95': np.quantile(delta, [.025, .975]).tolist(), 'blocks': len(counts), 'draws': 2000,
                                'interpretation': 'descriptive paired 1s time-block resampling; deterministic repeats are not independent samples'})
        events.append({'sequence': sequence, 'benefit_effects': effects, 'seed': 20261008})
    return result, events


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('index', type=Path, help='JSON sequences:{sequence:{data_root,runs:{OFF,G,V,GV}}}')
    p.add_argument('output', type=Path)
    args = p.parse_args()
    rows, events = [], []
    for seq, entry in json.loads(args.index.read_text())['sequences'].items():
        a, b = evaluate(entry['runs'], entry['data_root'], seq, entry.get('diagnostic_off'), bootstrap=True)
        rows.extend(a)
        events.extend(b)
    args.output.mkdir(exist_ok=False)
    with (args.output / 'metrics.csv').open('x') as f:
        w = csv.DictWriter(f, fieldnames=rows[0].keys())
        w.writeheader()
        w.writerows(rows)
    (args.output / 'events.json').write_text(json.dumps(events, indent=2, allow_nan=False) + '\n')
