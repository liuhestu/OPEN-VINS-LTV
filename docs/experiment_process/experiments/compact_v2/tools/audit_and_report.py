#!/usr/bin/env python3
"""Independent artifact audit and readable compact_v2 comparison tables."""
import csv
import hashlib
import json
from pathlib import Path
import sys
import numpy as np
ROOT = Path(__file__).resolve().parents[5]
DEST = Path(__file__).resolve().parents[1]
ARCHIVE = ROOT / 'docs/experiment_process/euroc_final_results_data'
sys.path.insert(0, str(ROOT / 'docs/experiment_process/experiments/tools/ltv_three_track'))
from evaluate_euroc_six import nearest, aligned_rmse, load_trajectory


def read(p):
    return json.loads(p.read_text())


def digest(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    accuracy, performance, metrics = (read(DEST / (name + '.json')) for name in ['accuracy', 'performance', 'metrics'])
    old = {(r['sequence'], r['mode']): r for r in read(ARCHIVE / 'results.json')}
    old_cost = {(r['sequence'], r['mode']): r['cost'] for r in read(ARCHIVE / 'performance_index.json')}
    protocol = read(DEST / 'protocol.json')
    modes = ['OFF', 'G', 'V', 'L', 'GVL']
    expected = {(s, m) for s in protocol['sequences'] for m in modes}
    errors, off_checks = [], []
    for phase, rows in [('accuracy', accuracy), ('performance', performance)]:
        if len(rows) != 50 or {(r['sequence'], r['mode']) for r in rows} != expected:
            errors.append(phase + ':identity_count')
        for row in rows:
            seq, mode = row['sequence'], row['mode']
            label = phase + ':' + seq + ':' + mode
            if row['status'] != 'PASS' or row['exit_code'] or row['errors']:
                errors.append(label + ':failed_run')
            output = Path(row['output_dir'])
            selection = read(output / 'selection.json')
            if digest(output / 'config/estimator_config.yaml') != selection['generated_config_sha256']:
                errors.append(label + ':config_changed')
            if digest(output / 'trajectory.csv') != row['trajectory_sha256']:
                errors.append(label + ':trajectory_changed')
            if not selection['frozen_profile'] or tuple(selection['weights'].values()) != tuple(protocol['profiles']['MH' if seq.startswith('MH') else 'Vicon'][mode]):
                errors.append(label + ':weight_changed')
            x = load_trajectory(output / 'trajectory.csv')
            frozen = np.load(ARCHIVE / 'support' / (seq + '.npz'))
            xi = nearest(x[:, 0], frozen['times'])
            if not (abs(x[xi, 0] - frozen['times']) <= 1e-6).all():
                errors.append(label + ':coverage')
            if abs(x[0, 0] - frozen['initialization']) > 1e-6:
                errors.append(label + ':initialization')
            ate = aligned_rmse(x[xi, 5:8], frozen['truth'])[0]
            metric = next(r for r in metrics if (r['sequence'], r['mode']) == (seq, mode))
            if metric['ate_rmse_m'] is None or abs(ate - metric['ate_rmse_m']) > 1e-12:
                errors.append(label + ':ATE')
            path = output / 'ltv.csv'
            if path.exists():
                with path.open() as stream:
                    reader = csv.DictReader(stream)
                    fields = reader.fieldnames
                    logs = list(reader)
                if any('gain_norm' in name or 'update_norm' in name or 'delta_norm' in name for name in fields):
                    errors.append(label + ':retired_diagnostic_column')
                if any(int(r['EKF_calls']) > 1 for r in logs):
                    errors.append(label + ':multiple_native_updates')
                for branch in ['G', 'V', 'L']:
                    count = sum(int(r[branch + '_rows']) > 0 and r['submit_reason'] == 'joint_applied' and int(r['EKF_calls']) == 1 for r in logs)
                    if count != row['applications'][branch]:
                        errors.append(label + ':application_count:' + branch)
            if phase == 'performance' and row['cpu_affinity'] != [0, 1]:
                errors.append(label + ':serial_affinity')
            if phase == 'accuracy' and mode == 'OFF':
                previous = Path(old[seq, mode]['output_dir'])
                check = dict(sequence=seq)
                for name in ['trajectory.csv', 'audit.csv', 'algorithm_visual_identity.jsonl']:
                    check[name] = (output / name).read_bytes() == (previous / name).read_bytes()
                    if not check[name]:
                        errors.append(label + ':OFF_changed:' + name)
                off_checks.append(check)
    identity = read(DEST / 'identity.json')
    for name, value in identity['source_sha256'].items():
        if digest(ROOT / name) != value:
            errors.append('source_changed:' + name)
    for name, value in identity['installed_binary_sha256'].items():
        if digest(Path(name)) != value:
            errors.append('binary_changed:' + name)
    tests = read(DEST / 'tests.json')
    if len(tests) != 14 or any(r['exit_code'] for r in tests):
        errors.append('basic_test_failure')
    inputs = read(DEST / 'data_verification.json')
    if len(inputs) != 10 or any(r['errors'] for r in inputs):
        errors.append('input_identity')
    if not all(r.get('serial_trajectory_identical') for r in metrics):
        errors.append('serial_accuracy_trajectory_difference')
    audit = dict(status='PASS' if not errors else 'FAIL', errors=errors, basic_tests=len(tests),
                 smoke_runs=len(read(DEST / 'smoke.json')), accuracy_runs=len(accuracy), serial_runs=len(performance),
                 off_exact_checks=off_checks, input_files=sum(r['files'] for r in inputs),
                 full_coverage=all(r['coverage'] == 1 for r in metrics), source_and_binary_preserved=not any('changed' in e for e in errors))
    (DEST / 'independent_audit.json').write_text(json.dumps(audit, indent=2) + '\n')
    lookup = {(r['sequence'], r['mode']): r for r in metrics}
    for r in metrics:
        base = lookup[r['sequence'], 'OFF']['ate_rmse_m']
        r['change_vs_new_OFF_pct'] = 100 * (r['ate_rmse_m'] / base - 1) if r['ate_rmse_m'] is not None and base else None
        prior_cost = old_cost[r['sequence'], r['mode']]
        r['serial_cpu_change_vs_historical_pct'] = 100 * (r['serial_cost']['full_cpu_s'] / prior_cost['full_cpu_s'] - 1)
    fields = sorted(set().union(*(r.keys() for r in metrics)))
    with (DEST / 'results.csv').open('w') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for r in metrics:
            writer.writerow({k: json.dumps(v, sort_keys=True) if isinstance(v, dict) else v for k, v in r.items()})
    # Historical runs used additional branch K/Kr and spectral diagnostics; retain this comparison limitation.
    text = ['# compact_v2 评估结果', '',
            f"{len(tests)}个基础测试、10个40秒短段、50个完整精度回放和50个独立串行回放；审计：**{audit['status']}**。",
            'OFF的十条轨迹、逐帧主State/P及前端审计、视觉输出摘要与旧冻结结果逐字节相同。ON允许离散实现改变，冻结权重未重新扫描。', '',
            'ATE使用原冻结OFF的相同时间支撑，SE(3)对齐、不缩放；所有模式覆盖率为100%，初始化时间不变。', '',
            '| 序列 | OFF ATE(m) | G | V | L | GVL |', '|---|---:|---:|---:|---:|---:|']
    for seq in protocol['sequences']:
        text.append('| ' + seq + ' | ' + ' | '.join(f"{lookup[seq, m]['ate_rmse_m']:.6f}" for m in modes) + ' |')
    text += ['', '相对本轮OFF的ATE变化（负数改善，正数退化）：', '', '| 序列 | G | V | L | GVL |', '|---|---:|---:|---:|---:|']
    for seq in protocol['sequences']:
        text.append('| ' + seq + ' | ' + ' | '.join(f"{lookup[seq, m]['change_vs_new_OFF_pct']:+.3f}%" for m in modes[1:]) + ' |')
    text += ['', '相对旧同模式ATE的变化：', '', '| 序列 | G | V | L | GVL |', '|---|---:|---:|---:|---:|']
    for seq in protocol['sequences']:
        text.append('| ' + seq + ' | ' + ' | '.join(f"{lookup[seq, m]['historical_change_pct']:+.3f}%" for m in modes[1:]) + ' |')
    text += ['', '完整1/5秒平移与旋转RPE、位置P95/最大值、最后10%误差和有效覆盖见 [metrics.json](metrics.json) 与 [results.csv](results.csv)；原OFF起点划分的10秒局部误差见 [local_windows.json](local_windows.json)。V1_01姿态真值来源尚未验证，姿态相关RPE仅作描述。', '',
             '独立串行开销：固定CPU 0,1、OpenCV/BLAS单线程、独占实验性能锁，每组只测一次；下表全部取第二次独立回放，不采用并行精度计时。相机API不包含完整IMU和I/O成本。', '',
             '| 序列 | 模式 | 相机P50(ms) | P95(ms) | 全段CPU(s) | 墙钟(s) | RSS(MiB) |', '|---|---|---:|---:|---:|---:|---:|']
    for seq in protocol['sequences']:
        for mode in modes:
            c = lookup[seq, mode]['serial_cost']
            text.append(f"| {seq} | {mode} | {c['camera_api_p50_ms']:.3f} | {c['camera_api_p95_ms']:.3f} | {c['full_cpu_s']:.2f} | {c['full_wall_s']:.2f} | {c['peak_rss_kb']/1024:.2f} |")
    on = [r for r in metrics if r['mode'] != 'OFF']
    overhead = [100 * (r['serial_cost']['full_cpu_s'] / lookup[r['sequence'], 'OFF']['serial_cost']['full_cpu_s'] - 1) for r in on]
    text += ['', f"ON相对本轮OFF的全段CPU开销增加范围：{min(overhead):.1f}% 至 {max(overhead):.1f}%；仍存在明显额外开销。",
             f"相对旧同模式全段CPU，ON变化中位数为 {np.median([r['serial_cpu_change_vs_historical_pct'] for r in on]):+.1f}%。旧版包含额外K/Kr与谱诊断；该变化同时包含离散数学和诊断删减，不能单独归因于某一项。", '',
             '实际更新次数（同一联合EKF可能同时包含多个分支，各分支次数不可相加当作EKF次数）：', '',
             '| 序列 | 模式 | G | V | L |', '|---|---|---:|---:|---:|']
    for seq in protocol['sequences']:
        for mode in modes:
            a = lookup[seq, mode]['applications']
            text.append(f"| {seq} | {mode} | {a['G']} | {a['V']} | {a['L']} |")
    text += ['', '拒绝原因、NIS与执行状态：[accuracy.json](accuracy.json)；单独串行原始开销：[performance.json](performance.json)。所有串行/精度轨迹相同，单帧辅助提交最多一次原生EKFUpdate。',
             '基础测试覆盖解析相机修正、PSD、增删重排交叉块、首次事件、空观测、LLT失败、G/V/L FEJ Jacobian、混合列布局、空视觉、辅助拒绝与重复提交。初次旧测试接口及缺失标定的测试夹具错误已修正，失败证据保留在tests_initial.json和build日志中。',
             'Observer的P是设计矩阵，辅助R仍为既有启发式噪声；未证明共享输入的统计独立或一致性。MH_04沿用原排除范围，不属于本轮10条序列。', '',
             '重现与源码导读：[README.md](README.md)；冻结身份：[identity.json](identity.json)；输入核对：[data_verification.json](data_verification.json)；独立审计：[independent_audit.json](independent_audit.json)。', '']
    (DEST / 'REPORT.md').write_text('\n'.join(text))
    analysis = dict(script_sha256=digest(Path(__file__)), script=str(Path(__file__)), inputs={name: digest(DEST / name) for name in ['metrics.json', 'accuracy.json', 'performance.json', 'identity.json', 'data_verification.json']})
    (DEST / 'analysis_identity.json').write_text(json.dumps(analysis, indent=2) + '\n')
    print(json.dumps(audit, indent=2))
    if errors:
        raise SystemExit(1)

if __name__ == '__main__':
    main()
