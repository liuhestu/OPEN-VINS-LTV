#!/usr/bin/env python3
"""compact_v2 frozen-weight replay, fixed historical support and serial costs."""
import argparse
from concurrent.futures import ThreadPoolExecutor
from contextlib import ExitStack
import csv
import fcntl
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[5]
DEST = Path(__file__).resolve().parents[1]
TOOLS = ROOT / 'docs/experiment_process/experiments/tools'
sys.path[:0] = [str(TOOLS / 'ltv_gain_scan'), str(TOOLS / 'ltv_three_track')]
import run_manual_euroc as manual
from evaluate_euroc_six import nearest, aligned_rmse, load_trajectory
ARCHIVE = manual.ARCHIVE
PROTOCOL = manual.read(ARCHIVE / 'protocol.json')
MODES = list(manual.BRANCHES)
SEQUENCES = PROTOCOL['sequences']
INSTALL = Path('/home/he/open_vins_ltv_ws/install_compact_v2')


def write(path, value):
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')
    temporary.replace(path)


def run_one(sequence, mode, phase, cpus):
    config = dict(sequence=sequence, mode=mode, weights=dict(G=None, V=None, L=None),
                  dataset_root='/home/he/datasets/euroc/ASL', install_root=str(INSTALL),
                  output_root=str(DEST / 'runs' / phase), cpu_affinity=cpus,
                  short_seconds=40 if phase == 'smoke' else 0)
    selection = manual.resolve(config)
    row_identity = manual.read(DEST / 'identity.json')['installed_binary_sha256']
    # Each generated YAML and frozen input template is recorded by the manual preparer.
    config_path = DEST / 'protocol.json'
    output, target = manual.prepare(config_path, config, selection)
    env = dict(os.environ)
    for key in list(env):
        if key.startswith(('LTV_', 'ROS_', 'AMENT_', 'COLCON_', 'CCACHE_')) or key in ('CMAKE_PREFIX_PATH', 'LD_LIBRARY_PATH', 'PYTHONPATH'):
            env.pop(key, None)
    env.update(OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1', MKL_NUM_THREADS='1', NUMEXPR_NUM_THREADS='1',
               ROS_LOG_DIR=str(output / 'ros_logs'))
    shell = 'set -e; source /opt/ros/humble/setup.bash; source "$1/local_setup.bash"; shift; exec "$@"'
    command = ['taskset', '-c', ','.join(map(str, cpus)), '/usr/bin/time', '-q',
               '-f', '{"max_rss_kb":%M,"elapsed_s":%e,"user_s":%U,"system_s":%S}',
               '-o', str(output / 'resources.json'), 'bash', '-c', shell, 'compact-v2',
               str(INSTALL), selection['binary'], str(target), selection['data'], str(output), selection['native_mode']]
    if phase == 'smoke':
        command.append('40')
    with (output / 'stdout_stderr.log').open('w') as log:
        result = subprocess.run(command, cwd=output, env=env, stdout=log, stderr=subprocess.STDOUT)
    row = dict(sequence=sequence, mode=mode, phase=phase, output_dir=str(output), exit_code=result.returncode,
               cpu_affinity=cpus, weights=selection['weights'], binary_identity=row_identity, errors=[], applications=dict(G=0, V=0, L=0))
    if result.returncode == 0:
        replay = manual.read(output / 'replay.json')
        effective = manual.read(output / 'effective_options.json')
        for key, wanted in selection['parameters'].items():
            actual = effective['max_slam_features' if key == 'max_slam' else key]
            if actual != wanted and not np.isclose(actual, wanted, rtol=1e-12, atol=1e-15):
                row['errors'].append('effective_option:' + key)
        if replay['complete'] != (phase != 'smoke'):
            row['errors'].append('completion_mismatch')
        if phase != 'smoke' and (replay['camera_packets'] != replay['input_camera_packets'] or replay['imu_consumed'] != replay['input_imu_samples']):
            row['errors'].append('incomplete_input')
        if manual.read(output / 'threading.json')['actual_opencv_threads'] != 1:
            row['errors'].append('OpenCV_threads')
        row['applications'].update(G=replay['actual_G_submissions'], V=replay['actual_V_submissions'])
        path = output / 'ltv.csv'
        if path.exists():
            with path.open() as stream:
                logs = list(csv.DictReader(stream))
            row['applications']['L'] = sum(int(r['L_rows']) > 0 and r['submit_reason'] == 'joint_applied' for r in logs)
            row['rejections'] = {}
            for r in logs:
                for branch in ['G', 'V', 'L']:
                    reason = r[branch + '_reason']
                    label = branch + ':' + reason
                    row['rejections'][label] = row['rejections'].get(label, 0) + 1
            row['ekf_calls'] = sum(int(r['EKF_calls']) for r in logs)
            row['nis'] = {}
            for branch in ['G', 'V', 'L', 'joint']:
                values = [float(r[branch + '_nis']) for r in logs if float(r[branch + '_nis']) >= 0]
                row['nis'][branch] = dict(samples=len(values), p50=float(np.median(values)) if values else None,
                                         p95=float(np.quantile(values, .95)) if values else None)
            if any(r['reason'] in ('core_imu_failure', 'core_camera_failure') for r in logs):
                row['errors'].append('observer_basic_failure')
        for branch, count in row['applications'].items():
            if branch not in manual.BRANCHES[mode] and count:
                row['errors'].append('disabled_branch:' + branch)
        try:
            x = load_trajectory(output / 'trajectory.csv')
            row['estimated_rows'] = len(x)
        except (ValueError, OSError) as e:
            row['errors'].append(str(e))
        resource = manual.read(output / 'resources.json')
        timing = np.loadtxt(output / 'frame_processing.csv', delimiter=',', skiprows=1, ndmin=2)[:, 1]
        row['cost'] = dict(camera_api_p50_ms=float(np.quantile(timing, .5)), camera_api_p95_ms=float(np.quantile(timing, .95)),
                           camera_api_p99_ms=float(np.quantile(timing, .99)), full_wall_s=resource['elapsed_s'],
                           full_cpu_s=resource['user_s'] + resource['system_s'], peak_rss_kb=resource['max_rss_kb'])
        row['trajectory_sha256'] = manual.digest(output / 'trajectory.csv')
    row['status'] = 'PASS' if result.returncode == 0 and not row['errors'] else 'FAIL'
    write(output / 'validation.json', row)
    return row


def phase_run(phase):
    path = DEST / (phase + '.json')
    rows = manual.read(path) if path.exists() else []
    completed = {(r['sequence'], r['mode']) for r in rows if r['status'] == 'PASS'}
    sequences = ['MH_05_difficult', 'V2_02_medium'] if phase == 'smoke' else SEQUENCES
    jobs = [(seq, mode) for i, seq in enumerate(sequences) for mode in (MODES if i % 2 == 0 else list(reversed(MODES)))
            if (seq, mode) not in completed]
    with ExitStack() as stack:
        lockroot = Path('/home/he/output/ltv_shared_replay_locks')
        for name, flag in [('build_replay.lock', fcntl.LOCK_SH),
                           ('performance_gate.lock', fcntl.LOCK_EX if phase == 'performance' else fcntl.LOCK_SH)]:
            lock = stack.enter_context((lockroot / name).open('r'))
            fcntl.flock(lock, flag | fcntl.LOCK_NB)
        # Serial costs use fixed [0,1]; accuracy uses four disjoint two-CPU pairs.
        workers = 1 if phase == 'performance' else 4
        with ThreadPoolExecutor(max_workers=workers) as pool:
            for start in range(0, len(jobs), workers):
                batch = jobs[start:start + workers]
                futures = [pool.submit(run_one, seq, mode, phase, [2 * i, 2 * i + 1]) for i, (seq, mode) in enumerate(batch)]
                for future in futures:
                    row = future.result()
                    rows = [r for r in rows if (r['sequence'], r['mode']) != (row['sequence'], row['mode'])]
                    rows.append(row)
                    write(path, rows)
                    print(json.dumps(dict(phase=phase, completed=len(rows), sequence=row['sequence'], mode=row['mode'], status=row['status'])), flush=True)
                if any(r['status'] != 'PASS' for r in rows):
                    raise RuntimeError('basic/runtime/input failure; expansion stopped, see ' + str(path))
    return rows


def analyze():
    accuracy = manual.read(DEST / 'accuracy.json')
    performance = manual.read(DEST / 'performance.json') if (DEST / 'performance.json').exists() else []
    frozen_results = {(r['sequence'], r['mode']): r for r in manual.read(ARCHIVE / 'results.json')}
    costs = {(r['sequence'], r['mode']): r for r in performance}
    metrics, windows = [], []
    for row in accuracy:
        seq, mode = row['sequence'], row['mode']
        x = load_trajectory(Path(row['output_dir']) / 'trajectory.csv')
        support = np.load(ARCHIVE / 'support' / (seq + '.npz'))
        times, truth = support['times'], support['truth']
        xi = nearest(x[:, 0], times)
        matched = abs(x[xi, 0] - times) <= 1e-6
        item = dict(sequence=seq, mode=mode, coverage=float(matched.mean()), samples=int(matched.sum()),
                    baseline_support=len(times), ate_rmse_m=None, status=row['status'], applications=row['applications'])
        if matched.all() and row['status'] == 'PASS':
            estimate = x[xi]
            xyz = estimate[:, 5:8]
            ate, horn = aligned_rmse(xyz, truth)
            u, _, vt = np.linalg.svd((xyz - xyz.mean(0)).T @ (truth - truth.mean(0)))
            r = vt.T @ np.diag([1., 1., np.linalg.det(vt.T @ u.T)]) @ u.T
            error = np.linalg.norm((xyz - xyz.mean(0)) @ r.T + truth.mean(0) - truth, axis=1)
            item.update(ate_rmse_m=ate, horn_difference_m=abs(ate - horn), position_p95_m=float(np.quantile(error, .95)),
                        position_max_m=float(error.max()), tail_last10pct_rmse_m=float(np.sqrt(np.mean(error[-max(1, len(error) // 10):] ** 2))))
            gt = np.loadtxt(Path('/home/he/datasets/euroc/ASL') / seq / 'mav0/state_groundtruth_estimate0/data.csv', delimiter=',', comments='#', ndmin=2)
            gi = nearest(gt[:, 0] * 1e-9, times)
            rg = Rotation.from_quat(gt[gi][:, [5, 6, 7, 4]])
            re = Rotation.from_quat(estimate[:, 1:5])
            item['rotation_truth_status'] = 'UNVERIFIED_V1_01_ORIENTATION' if seq == 'V1_01_easy' else 'existing_frozen_GT'
            for dt in [1, 5]:
                end = nearest(times, times + dt)
                ok = abs(times[end] - times - dt) <= .025
                a, b = np.flatnonzero(ok), end[ok]
                de = re[a].inv().apply(estimate[b, 5:8] - estimate[a, 5:8])
                dg = rg[a].inv().apply(truth[b] - truth[a])
                rr = ((rg[a].inv() * rg[b]).inv() * (re[a].inv() * re[b])).magnitude()
                item[f'rpe_translation_{dt}s_m'] = float(np.sqrt(np.mean(np.linalg.norm(de - dg, axis=1) ** 2)))
                item[f'rpe_rotation_{dt}s_deg'] = float(np.sqrt(np.mean(np.degrees(rr) ** 2)))
            bins = np.floor((times - support['initialization']) / 10).astype(int)
            for bucket in np.unique(bins):
                mask = bins == bucket
                windows.append(dict(sequence=seq, mode=mode, bin=int(bucket), samples=int(mask.sum()),
                                    rmse_m=float(np.sqrt(np.mean(error[mask] ** 2)))))
            historical = frozen_results[seq, mode]
            item['historical_ate_m'] = historical['ate_rmse_m']
            item['historical_change_pct'] = 100 * (ate / historical['ate_rmse_m'] - 1)
        else:
            item['status'] = 'FAIL_COVERAGE_OR_RUNTIME'
        cost = costs.get((seq, mode))
        if cost:
            item['serial_cost'] = cost['cost']
            item['serial_trajectory_identical'] = cost.get('trajectory_sha256') == row.get('trajectory_sha256')
        metrics.append(item)
    write(DEST / 'metrics.json', metrics)
    write(DEST / 'local_windows.json', windows)
    lookup = {(r['sequence'], r['mode']): r for r in metrics}
    text = ['# compact_v2 评估', '', '冻结权重；10条序列×5模式。ATE使用原冻结 OFF 的相同时间支撑，SE(3)对齐、不缩放。',
            '原谱裁剪/Euler实现归档在 prechange；本轮变更不证明统计一致性。', '',
            '| 序列 | OFF ATE(m) | G | V | L | GVL |', '|---|---:|---:|---:|---:|---:|']
    for seq in SEQUENCES:
        text.append('| ' + seq + ' | ' + ' | '.join(f"{lookup[seq, m]['ate_rmse_m']:.6f}" if lookup.get((seq, m), {}).get('ate_rmse_m') is not None else 'FAIL/PENDING' for m in MODES) + ' |')
    text += ['', '相对本轮 OFF 的变化（负数改善，正数退化）：', '', '| 序列 | G | V | L | GVL |', '|---|---:|---:|---:|---:|']
    for seq in SEQUENCES:
        base = lookup.get((seq, 'OFF'), {}).get('ate_rmse_m')
        text.append('| ' + seq + ' | ' + ' | '.join(f"{100 * (lookup[seq, m]['ate_rmse_m'] / base - 1):+.3f}%" if base and lookup.get((seq, m), {}).get('ate_rmse_m') is not None else 'FAIL/PENDING' for m in MODES[1:]) + ' |')
    text += ['', '完整 ATE/RPE、覆盖率、尾段、更新次数见 [metrics.json](metrics.json)，10秒局部窗口见 [local_windows.json](local_windows.json)。',
             '拒绝原因、NIS和执行状态见 [accuracy.json](accuracy.json)；独立串行开销见 [performance.json](performance.json)。',
             '串行使用固定 CPU 0,1；OpenCV/BLAS单线程；每组一次独立运行，未估计计时置信区间。V1_01姿态真值来源限制沿用历史报告。', '']
    (DEST / 'REPORT.md').write_text('\n'.join(text))
    write(DEST / 'audit.json', dict(accuracy_count=len(accuracy), performance_count=len(performance),
          all_full_coverage=all(r['coverage'] == 1 for r in metrics), all_runs_pass=all(r['status'] == 'PASS' for r in accuracy + performance),
          off_ate_max_historical_difference=max((abs(r['ate_rmse_m'] - r['historical_ate_m']) for r in metrics if r['mode'] == 'OFF' and r['ate_rmse_m'] is not None), default=None),
          serial_trajectory_identical=all(r.get('serial_trajectory_identical', False) for r in metrics) if len(performance) == 50 else None))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('action', choices=['smoke', 'accuracy', 'performance', 'analyze', 'all'])
    action = parser.parse_args().action
    if not (DEST / 'tests.json').exists() or any(r['exit_code'] for r in manual.read(DEST / 'tests.json')):
        raise RuntimeError('basic test evidence missing or failed; expansion stopped')
    identity = manual.read(DEST / 'identity.json')
    for name, wanted in identity['installed_binary_sha256'].items():
        if manual.digest(Path(name)) != wanted:
            raise RuntimeError('frozen binary changed: ' + name)
    for name, wanted in identity['source_sha256'].items():
        if manual.digest(ROOT / name) != wanted:
            raise RuntimeError('frozen source changed: ' + name)
    if not (DEST / 'protocol.json').exists():
        write(DEST / 'protocol.json', dict(version='compact_v2', profiles=PROTOCOL['profiles'], sequences=SEQUENCES,
            source_support=str(ARCHIVE / 'support'), smoke_seconds=40, accuracy_workers=4, serial_cpu=[0, 1],
            parameter_scan=False, independent_serial_runs=True))
    if action in ['smoke', 'all']:
        phase_run('smoke')
    if action in ['accuracy', 'performance', 'all']:
        smoke = manual.read(DEST / 'smoke.json')
        if len(smoke) != 10 or any(r['status'] != 'PASS' for r in smoke):
            raise RuntimeError('smoke evidence failed or incomplete')
    if action in ['accuracy', 'all']:
        phase_run('accuracy')
    if action in ['performance', 'all']:
        phase_run('performance')
    if action in ['analyze', 'all']:
        analyze()

if __name__ == '__main__':
    main()
