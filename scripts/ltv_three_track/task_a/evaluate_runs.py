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
from analyze import reference


def rmse(x):
    return float(np.sqrt(np.mean(np.asarray(x) ** 2))) if len(x) else None


def evaluate(run_paths, data_root, sequence):
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
    result, events = [], []
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
        for region, mask in masks.items():
            metrics = {'ATE_m': pe[mask], 'attitude_deg': ae[mask], 'velocity_mps': ve[mask & velocity_valid]}
            for dt in (1., 5.):
                end = nearest(times, times + dt)
                ok = (abs(times[end] - (times + dt)) <= .025) & mask & mask[end]
                a, b = np.flatnonzero(ok), end[ok]
                de = re[a].inv().apply(est[b, 5:8] - est[a, 5:8])
                dg = rg[a].inv().apply(gp[b] - gp[a])
                dr = ((rg[a].inv() * rg[b]).inv() * (re[a].inv() * re[b])).magnitude()
                metrics[f'RPE_{int(dt)}s_m'] = np.linalg.norm(de - dg, axis=1)
                metrics[f'RPE_{int(dt)}s_deg'] = np.degrees(dr)
            for metric, values in metrics.items():
                result.append(dict(sequence=sequence, mode=mode, region=region, metric=metric, value=rmse(values), samples=len(values)))
        with (path / 'ltv.csv').open() as f:
            logs = list(csv.DictReader(f))
        events.append(dict(sequence=sequence, mode=mode, G_reasons=dict(Counter(z['G_reason'] for z in logs)),
                           V_reasons=dict(Counter(z['V_reason'] for z in logs)),
                           input=replay, bias_first=x[0, 11:17].tolist(), bias_last=x[-1, 11:17].tolist()))
    return result, events


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('index', type=Path, help='JSON sequences:{sequence:{data_root,runs:{OFF,G,V,GV}}}')
    p.add_argument('output', type=Path)
    args = p.parse_args()
    rows, events = [], []
    for seq, entry in json.loads(args.index.read_text())['sequences'].items():
        a, b = evaluate(entry['runs'], entry['data_root'], seq)
        rows.extend(a)
        events.extend(b)
    args.output.mkdir(exist_ok=False)
    with (args.output / 'metrics.csv').open('x') as f:
        w = csv.DictWriter(f, fieldnames=rows[0].keys())
        w.writeheader()
        w.writerows(rows)
    (args.output / 'events.json').write_text(json.dumps(events, indent=2, allow_nan=False) + '\n')
