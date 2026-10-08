#!/usr/bin/env python3
"""Frozen OFF support, SE(3) ATE, independent Horn verification, no scale."""
import argparse
import csv
import json
from pathlib import Path
import numpy as np
from run_euroc_six import MODES, SEQUENCES


def nearest(t, u):
    hi = np.searchsorted(t, u).clip(0, len(t) - 1)
    lo = np.maximum(0, hi - 1)
    return np.where(abs(t[lo] - u) <= abs(t[hi] - u), lo, hi)


def aligned_rmse(x, y):
    xc, yc = x.mean(0), y.mean(0)
    s = (x - xc).T @ (y - yc)
    u, _, vt = np.linalg.svd(s)
    d = np.diag([1., 1., np.linalg.det(vt.T @ u.T)])
    r = vt.T @ d @ u.T
    residual = (x - xc) @ r.T - (y - yc)
    svd = float(np.sqrt(np.mean(np.sum(residual * residual, axis=1))))
    tr = np.trace(s)
    n = np.array([[tr, s[1, 2]-s[2, 1], s[2, 0]-s[0, 2], s[0, 1]-s[1, 0]],
                  [s[1, 2]-s[2, 1], s[0, 0]-s[1, 1]-s[2, 2], s[0, 1]+s[1, 0], s[0, 2]+s[2, 0]],
                  [s[2, 0]-s[0, 2], s[0, 1]+s[1, 0], -s[0, 0]+s[1, 1]-s[2, 2], s[1, 2]+s[2, 1]],
                  [s[0, 1]-s[1, 0], s[0, 2]+s[2, 0], s[1, 2]+s[2, 1], -s[0, 0]-s[1, 1]+s[2, 2]]])
    _, vectors = np.linalg.eigh(n)
    w, a, b, c = vectors[:, -1]
    rh = np.array([[1-2*(b*b+c*c), 2*(a*b-w*c), 2*(a*c+w*b)],
                   [2*(a*b+w*c), 1-2*(a*a+c*c), 2*(b*c-w*a)],
                   [2*(a*c-w*b), 2*(b*c+w*a), 1-2*(a*a+b*b)]])
    e = (x-xc) @ rh.T - (y-yc)
    horn = float(np.sqrt(np.mean(np.sum(e*e, axis=1))))
    assert abs(svd-horn) <= 1e-9 * max(1., svd), (svd, horn)
    return svd, horn


def load_trajectory(path):
    x = np.loadtxt(path, delimiter=',', skiprows=1, ndmin=2)
    if x.shape[1] != 17 or len(x) < 3 or not np.isfinite(x).all() or not np.all(np.diff(x[:, 0]) > 0):
        raise ValueError('invalid/insufficient trajectory')
    return x


def evaluate(index, data):
    runs = {(r['sequence'], r['mode']): r for r in index}
    result = []
    for seq in SEQUENCES:
        gt = np.loadtxt(data / seq / 'mav0/state_groundtruth_estimate0/data.csv', delimiter=',', comments='#', ndmin=2)
        try:
            off = load_trajectory(Path(runs[seq, 'OFF']['output_dir']) / 'trajectory.csv')
            gi = nearest(gt[:, 0]*1e-9, off[:, 0])
            ok = abs(gt[gi, 0]*1e-9-off[:, 0]) <= .02
            ok &= (off[:, 0] >= gt[0, 0]*1e-9) & (off[:, 0] <= gt[-1, 0]*1e-9)
            support, truth = off[ok, 0], gt[gi[ok], 1:4]
            baseline_valid = runs[seq, 'OFF']['exit_code'] == 0 and not runs[seq, 'OFF']['audit']['errors']
        except (OSError, ValueError, KeyError):
            off = None
            baseline_valid = False
            support, truth = np.empty(0), np.empty((0, 3))
        for mode in MODES:
            if (seq, mode) not in runs:
                result.append(dict(sequence=seq, mode=mode, ate_rmse_m=None, conditional_ate_rmse_m=None, status='PENDING', samples=0, baseline_support=len(support), actual_G=0, actual_V=0, actual_L=0, reason='not yet completed'))
                continue
            run = runs[seq, mode]
            row = dict(sequence=seq, mode=mode, ate_rmse_m=None, conditional_ate_rmse_m=None,
                       status='FAIL', reason='', samples=0, baseline_support=len(support), coverage=0.,
                       initialization_delta_s=None, run_id=run['run_id'], output_dir=run['output_dir'],
                       actual_G=run['audit'].get('actual_G', 0), actual_V=run['audit'].get('actual_V', 0),
                       actual_L=run['audit'].get('actual_L', 0), svd_horn_difference_m=None)
            try:
                x = load_trajectory(Path(run['output_dir']) / 'trajectory.csv')
                if len(support) < 3:
                    gi = nearest(gt[:, 0]*1e-9, x[:, 0]); ok = abs(gt[gi, 0]*1e-9-x[:, 0]) <= .02
                    if ok.sum() >= 3:
                        row['conditional_ate_rmse_m'] = aligned_rmse(x[ok, 5:8], gt[gi[ok], 1:4])[0]
                    row['reason'] = 'baseline_missing_or_insufficient_GT_support'
                else:
                    xi = nearest(x[:, 0], support); matched = abs(x[xi, 0]-support) <= 1e-6
                    row.update(samples=int(matched.sum()), coverage=float(matched.mean()),
                               initialization_delta_s=float(x[0, 0]-off[0, 0]))
                    if matched.sum() >= 3:
                        rmse, horn = aligned_rmse(x[xi[matched], 5:8], truth[matched])
                        row['conditional_ate_rmse_m'] = rmse
                        row['svd_horn_difference_m'] = abs(rmse-horn)
                    complete = run['exit_code'] == 0 and not run['audit']['errors']
                    if complete and baseline_valid and matched.all() and abs(row['initialization_delta_s']) <= 1e-6:
                        row.update(status='VALID_FULL_MATCHED', ate_rmse_m=row['conditional_ate_rmse_m'], reason='')
                    else:
                        row['reason'] = 'runtime/input/flags/baseline/coverage/initialization mismatch'
                row['estimated_rows'] = len(x)
            except (OSError, ValueError) as e:
                row['reason'] = str(e)
            result.append(row)
    return result


def main():
    p = argparse.ArgumentParser()
    p.add_argument('coord', type=Path)
    p.add_argument('report', type=Path)
    p.add_argument('--data', type=Path, default=Path('/home/he/datasets/euroc/ASL'))
    p.add_argument('--partial', action='store_true', help='publish progress; unexecuted cases are PENDING')
    a = p.parse_args()
    index = json.loads((a.coord / 'full_index.json').read_text())
    assert len({(r['sequence'], r['mode']) for r in index}) == len(index)
    assert len(index) == 66 or a.partial
    rows = evaluate(index, a.data)
    artifact = a.report.parent / 'euroc_results_data'
    artifact.mkdir(exist_ok=True)
    (artifact / 'results.json').write_text(json.dumps(rows, indent=2) + '\n')
    fields = sorted({k for row in rows for k in row})
    with (artifact / 'results.csv').open('w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=fields, lineterminator='\n'); w.writeheader(); w.writerows(rows)
    identity = json.loads((a.coord / 'identity.json').read_text())
    text = ['# EuRoC 六模式 ATE RMSE', '', f'main；11序列×6模式，已完成{len(index)}/66次新完整回放。ATE单位：m。',
            'OFF是纯OpenVINS；全部组max_slam=0、固定C0参数。G/V/L分别开启对应近似融合，L_GV三项同时开启。',
            'SE(3)对齐，无尺度；冻结OFF完整初始化后GT支持，GT最近20ms、输出匹配1µs。独立Horn与SVD交叉核验。',
            '此表不改变历史FAIL/STOP，不等同生产资格、性能或统计一致性。同步ASL原始流，不是ROS transport回放。',
            '', '| 序列 | OFF | G | V | GV | L | L_GV |', '|---|---:|---:|---:|---:|---:|---:|']
    lookup = {(r['sequence'], r['mode']): r for r in rows}
    for seq in SEQUENCES:
        values = [f"{lookup[seq,m]['ate_rmse_m']:.9f}" if lookup[seq,m]['ate_rmse_m'] is not None else ('PENDING' if lookup[seq,m]['status']=='PENDING' else 'FAIL') for m in MODES]
        text.append('| ' + seq + ' | ' + ' | '.join(values) + ' |')
    text += ['', '## 初始化、覆盖与实际融合', '', '| 序列 | 模式 | 状态 | 样本/基准 | G/V/L应用帧 | 条件ATE(m) |', '|---|---|---|---:|---:|---:|']
    for r in rows:
        value = f"{r['conditional_ate_rmse_m']:.9f}" if r['conditional_ate_rmse_m'] is not None else '—'
        text.append(f"| {r['sequence']} | {r['mode']} | {r['status']} | {r['samples']}/{r['baseline_support']} | {r['actual_G']}/{r['actual_V']}/{r['actual_L']} | {value} |")
    text += ['', '条件ATE只描述可用片段；失败或覆盖不足不按完整结果计入。没有实际应用帧的分支明确为未生效，不能仅凭开启标志称融合成功。',
             '', '## 运行身份', '', f"二进制源码：`{identity['source_oid']}`；原main：`{identity['base_main_oid']}`。", f"原始输出：`{a.coord}`（本机路径，不是公开下载）。",
             '[结构化结果](euroc_results_data/results.csv) · [完整结果与失败原因](euroc_results_data/results.json)。']
    notes = artifact / 'report_notes.md'
    if notes.exists() and not a.partial:
        text += ['', notes.read_text().rstrip()]
    a.report.write_text('\n'.join(text) + '\n')
    print(json.dumps({'cases': len(rows), 'valid': sum(r['status']=='VALID_FULL_MATCHED' for r in rows)}))


if __name__ == '__main__':
    main()
