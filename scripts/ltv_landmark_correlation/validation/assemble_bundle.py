#!/usr/bin/env python3
"""Assemble the prescribed LC deliverable paths from validated source evidence."""
import csv
import json
from pathlib import Path
import shutil

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
DOC = ROOT / 'docs/ltv/nondegradation_validation'
SOURCE = ROOT / 'docs/ltv/landmark_correlation'
OUT = DOC / 'landmark_correlation'


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False) + '\n')


def write_csv(path, rows):
    keys = list(dict.fromkeys(key for row in rows for key in row))
    with path.open('w', newline='') as stream:
        writer = csv.DictWriter(stream, keys, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def assemble():
    OUT.mkdir(exist_ok=True)
    replays = json.loads((DOC / 'full_shadow_replays.json').read_text())
    assert len(replays) == 6 and all(r['parity']['status'] == 'PASS_EXACT_NATIVE_PASSIVE' for r in replays)
    matrix = [
        {'LC': 'LC-01', 'status': 'BLOCKED', 'executed_scope': 'actual JPL seed Jacobian, synthetic full-source seed/main joint map, 3x12000 draws', 'dependency': 'posterior historic pose-bearing and calibration/time joint cross', 'evidence': '../../landmark_correlation/actual_seed/integrated_results.json'},
        {'LC': 'LC-02', 'status': 'BLOCKED', 'executed_scope': 'actual Euler F/B; correlated-source joint reference; persistent offset and same-bearing substeps', 'dependency': 'main visual/clone/injection/source cross and full gain sensitivity', 'evidence': '../../landmark_correlation/observer/model_audit.md'},
        {'LC': 'LC-03', 'status': 'BLOCKED', 'executed_scope': 'rolling prior-camera conditional anchors, nonzero cross-time/cross-point, identity events', 'dependency': 'full seed/main/anchor covariance; mature primary anchor linkage', 'evidence': '../V2_02_medium_matrix_audit.json;../V2_03_difficult_matrix_audit.json'},
        {'LC': 'LC-04', 'status': 'PASS', 'executed_scope': 'linear reference elimination/Joseph/direct-source/SVD and exact duplicate counterexamples only', 'dependency': 'real residual R/N/S blocked by LC01-03', 'evidence': '../../landmark_correlation/reference/results/math_and_lifecycle_checks.json'},
        {'LC': 'LC-05', 'status': 'PASS', 'executed_scope': 'offline full/zero-cross comparison only; partial-zero nonPSD rejection', 'dependency': 'real prior joint blocks unavailable, real candidate K forbidden', 'evidence': 'approximation_comparison.csv'},
        {'LC': 'LC-06', 'status': 'PASS', 'executed_scope': 'linear-Gaussian controlled reference calibrated; actual seed/Observer nonlinear fragments descriptive only', 'dependency': 'real statistical calibration NOT_EVALUATED without independent landmark truth', 'evidence': 'monte_carlo_summary.csv;../../landmark_correlation/observer_mc/README.md'},
        {'LC': 'LC-07', 'status': 'BLOCKED', 'executed_scope': '2 sequences complete real OFF+2shadow each, exact OFF and shadow streams; matrix/source receipt audits', 'dependency': 'full_model_valid_constraints=0; source covariances and full main/LTV cross unavailable', 'evidence': 'shadow_summary.csv;../full_shadow_replays.json'},
        {'LC': 'LC-08', 'status': 'INCONCLUSIVE', 'executed_scope': 'rolling pre-consumption prediction, three-way shared samples and historical geometry baseline', 'dependency': 'historical-geometry gain CI cross zero; real conditional information rank unknown', 'evidence': 'incremental_information.md;../../landmark_correlation/holdout/summary.json'}]
    for row in matrix:
        row['fusion_authorization'] = 'NONE'
        row['method_contract'] = 'experiment_contract.yaml'
    write_csv(OUT / 'experiment_matrix.csv', matrix)
    contract = json.loads((SOURCE / 'experiment_contract.yaml').read_text())
    contract['registered_LC_matrix'] = matrix
    contract['component_contracts'] = {'linear_reference': '../../landmark_correlation/reference/experiment_contract.json',
        'actual_seed': '../../landmark_correlation/actual_seed/experiment_contract.json',
        'native_observer_nonlinear': '../../landmark_correlation/observer_mc/contract.json',
        'holdout': '../../landmark_correlation/holdout/contract.json'}
    contract['assembled_after_runs'] = 'status matrix is post-run reporting; frozen thresholds and source contracts retained unchanged'
    write_json(OUT / 'experiment_contract.yaml', contract)
    checks = {'status_by_LC': matrix,
              'linear_reference': json.loads((SOURCE / 'reference/results/math_and_lifecycle_checks.json').read_text()),
              'native_seed': json.loads((SOURCE / 'actual_seed/integrated_results.json').read_text()),
              'native_observer_local': json.loads((SOURCE / 'observer/local_checks.json').read_text()),
              'real_shadow': {seq: {k: v for k, v in json.loads((DOC / f'{seq}_matrix_audit.json').read_text()).items() if k != 'conditional_point_spectra'}
                               for seq in ('V2_02_medium', 'V2_03_difficult')},
              'scope_limit': 'reader independently checks B, Gram, anchor blocks/prior snapshots; not every F or persistent-map recursion; unconditional model remains BLOCKED'}
    write_json(OUT / 'math_and_lifecycle_checks.json', checks)
    with (SOURCE / 'reference/results/monte_carlo_summary.csv').open() as stream:
        monte = [dict(scope='linear or controlled stereo-depth reference', **row) for row in csv.DictReader(stream)]
    with (SOURCE / 'observer_mc/summary.csv').open() as stream:
        monte.extend(dict(scope='actual nonlinear Observer input-output diagnostic, no Gaussian CI or landmark truth',
                          scene='two_Euler_plus_camera', seed=202610081, status='EXECUTED_DIAGNOSTIC_ONLY', **row) for row in csv.DictReader(stream))
    for row in checks['native_seed']['amplitudes']:
        monte.append({'scope': 'actual nonlinear seed descriptive / linearized source simultaneous Gaussian intervals',
                      'scene': 'native_seed', 'amplitude': row['amplitude'], 'draws': row['independent_draws'], 'seed': 20261008,
                      'status': row['linear_reference_status'], 'actual_relative_covariance_deviation': row['actual_relative_covariance_deviation'],
                      'actual_cross_relative_deviation': row['actual_cross_relative_deviation'], 'confidence': 'linearized reference only; full intervals linked in native_seed evidence'})
    write_csv(OUT / 'monte_carlo_summary.csv', monte)
    (OUT / 'approximation_comparison.csv').write_text((SOURCE / 'reference/results/approximation_comparison.csv').read_text())
    intervals = json.loads((SOURCE / 'gv/frozen_intervals.json').read_text())
    shadow = []
    for seq in ('V2_02_medium', 'V2_03_difficult'):
        audit = json.loads((DOC / f'{seq}_matrix_audit.json').read_text())
        run = next(r for r in replays if r['sequence'] == seq and r['mode'] == 'SHADOW_OFF' and r['repeat'] == 1)
        off = next(r for r in replays if r['sequence'] == seq and r['mode'] == 'OFF')
        frozen = intervals[seq]
        # OFF camera grid/masks are frozen; match source time within existing 1us convention.
        grid = np.asarray(frozen['times_s'])
        def camera_index(time):
            insertion = int(np.searchsorted(grid, time))
            candidates = [i for i in (insertion - 1, insertion) if 0 <= i < len(grid)]
            closest = min(candidates, key=lambda i: abs(grid[i] - time))
            return closest if abs(grid[closest] - time) <= 1e-6 else None
        groups = {'all_input': None, **{name: set(indices) for name, indices in frozen['masks'].items()}}
        for region, selected in groups.items():
            spectra = [r for r in audit['conditional_point_spectra'] if selected is None or camera_index(r['time']) in selected]
            ranks = [r['effective_rank'] for r in spectra]
            shadow.append({'sequence': seq, 'region': region, 'scope': 'fixed-seed/fixed-gain unit-source contribution only',
                'conditional_camera_rows': len(spectra), 'full_valid_constraints': 0, 'full_valid_fraction': 0,
                'missing_blocks': ';'.join(run['missing_blocks']), 'conditional_rank_min': min(ranks) if ranks else '',
                'conditional_rank_max': max(ranks) if ranks else '', 'matrix_reference_max_relative_error': audit['matrix_reference']['max_relative_identity_error'],
                'prior_snapshot_identity_checks_all': audit['matrix_reference']['prior_snapshot_identity_checks'],
                'unique_post_receipt_matches_all': audit['source_receipt_join']['unique_receipt_time_epoch_slot_matches'],
                'frame_p95_ms_all': run['frame_feed_camera_ms_p95'], 'matched_off_p95_ms_all': off['frame_feed_camera_ms_p95'],
                'peak_rss_kb_all': run['run']['resources']['max_rss_kb'], 'matched_off_peak_rss_kb_all': off['run']['resources']['max_rss_kb'],
                'real_constraint_model_status': 'BLOCKED', 'real_calibration': 'NOT_EVALUATED'})
    write_csv(OUT / 'shadow_summary.csv', shadow)
    (OUT / 'sources_and_blocks.md').write_text('''# 来源与必要交叉块\n\n主误差 true-minus-estimate，Observer/seed/anchor 为 estimate-minus-true；原生JPL及frame检查见 [seed报告](../../landmark_correlation/actual_seed/report.md)。同一物理来源只登记一次，Observer同帧bearing子步合成B而不重抽。IMU offset六源贯穿IMU/camera；每epoch/local-ID的上一camera anchor保留其物理时间，退休/reset/epoch替换失效。\n\n| 来源/块 | 当前证据 | 统计资格 |\n|---|---|---|\n| actual seed输入pose/camera/bearing索引与J | 原生Jacobian和受控完整源映射 | 真实posterior历史pose-bearing cross UNKNOWN |\n| 主clone完整P | 现有生产边缘及clone交叉保留 | 不提供历史噪声与main/Observer joint |\n| IMU校正acc/gyro单位源 | 逐步F/B、持久offset map | 原始插值端点、bias/calibration噪声cross UNKNOWN |\n| 同帧bearing源 | epoch/localID/time、完整合成B | nominal gain固定；gain敏感度实测差异约2.94% |\n| conditional Sigma_aa/tt/at（含跨点） | 完整可解码源maps及非零跨时刻块 | 仅已知条件贡献，非完整Sigma |\n| C_xa/C_xt、LTV-visual cross | UNKNOWN | 不能置零或构造真实R/N/S/K |\n| Riccati P_R | 原有Euler/谱处理保留、OFF精确 | 不是未经证明的Cov(estimate−true) |\n| 独立landmark真值 | UNAVAILABLE | 真实统计校准NOT_EVALUATED |\n\n主视觉nullspace/compression、EKF校正与JPL注入/reset、clone扩增/边缘化的完整来源映射尚未实现；不得宣称闭合。逐帧来源stream和post-frame global/localID/slot receipt、sha、路径见 [证据索引](../evidence_index.csv)，数学/生命周期范围见 [审计](../../landmark_correlation/observer/model_audit.md)。成功完整runner遇异常即停止；attempt日志通过receipt支持核对，不伪称applied。未来catch-and-continue需transaction lineage与commit/abort marker。\n''')
    (OUT / 'incremental_information.md').write_text('''# 条件信息与未消费观测\n\n[LC08原始复核](../../landmark_correlation/holdout/report.md)核对时间、身份、历史geometry及共同样本：当前bearing在预测时尚未消费，之后允许消费，属于滚动一步留出而非永久holdout。历史geometry严格使用past，主预测原字段精确复现。V2_02三方共同样本86102/87075，geometry对照frame-RMS改善0.484262、CI[-0.035071,0.604226]；V2_03为55584/55908、改善0.126036、CI[-0.171455,0.380810]。正常/恢复组也未形成稳健优势，判INCONCLUSIVE。\n\n线性参考的完全重复信息与非零视觉重复反例均验证给定视觉之后信息秩0。置零跨块会制造假后验收缩，见approximation_comparison.csv。真实main/LTV/visual joint缺失，真实条件信息秩保持BLOCKED；条件unit-source rank不是有效新信息秩。预测改善不能替代ON轨迹收益，本轮没有landmark融合。\n''')
    (OUT / 'report.md').write_text('''# LC01–08执行结果与阶段D\n\n阶段D为STOP_WITH_REASON。逐项状态、已执行范围、剩余依赖见 [experiment_matrix.csv](experiment_matrix.csv)；真实统计校准NOT_EVALUATED，融合收益NOT_DEMONSTRATED。实验执行完成允许得到STOP，不能被写成全部统计/融合验收通过。\n\nLC01–03的实际映射、只读共同源传播、滚动anchor与生命周期已落实；它们没有消除main/历史噪声/visual/gain的未知块。LC04–06在明确参考/受控范围完成；Monte Carlo抽样合同、反例功效与非线性失效边界保留。LC07两序列各一次匹配OFF与两次完整shadow，主状态、完整P、Observer x/P_R、生命周期与旧基线精确一致；新增source/matrix流逐字节重复。独立reader检查285250项B/Gram/anchor恒等式、107346项prior-snapshot身份，max相对差约1.13e-16；它不验证所有完整F递推或物理噪声模型。真实有效完整约束为0，只能判模型/覆盖不足，不能判可融合。\n\n重日志shadow p95增长约7.35%/11.18%，不满足5%预算，单独记录于shadow_summary.csv；这是诊断模式而非候选生产路径。LC08的geometry优势置信区间跨零，真实条件信息仍未知。所有GO前提均需真实源模型闭合和信息增量证据；[STOP理由及最小扩展](../landmark_go_stop.md)给出具体缺失矩阵。\n''')
    (OUT / 'report.md').write_text((OUT / 'report.md').read_text() + '\nLC05补充：固定r的4处有效子空间不兼容见 ../../landmark_correlation/reference/residual_support.csv，Kr仅形式诊断，不作物理候选。\n')
    (OUT / 'sources_and_blocks.md').write_text((OUT / 'sources_and_blocks.md').read_text() + '\n实际相对landmark残差及主状态原生JPL/FEJ Jacobian未实装核对，GO前必须补齐；seed J和小系统H不替代它。\n')
    print('Assembled 9 prescribed LC artifacts')


if __name__ == '__main__':
    assemble()
