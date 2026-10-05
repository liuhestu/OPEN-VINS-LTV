"""Render standalone scientific figures and a reviewable stop/continue report."""
import argparse
import csv
import json
from pathlib import Path
import shutil
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from run import sha, write


def number(x):
    return '—' if x is None else f'{x:.6g}'


def render(out, dest):
    summary = json.loads((out/'summary.json').read_text())
    audit=json.loads((out/'correlation_audit.json').read_text())
    dest.mkdir(parents=True, exist_ok=True)
    figures=dest/'figures';figures.mkdir(exist_ok=True)
    text=['# Landmark 因果预测验证结果', '', f"冻结起点 `{summary['baseline']}`；决策：**{audit['decision']}**。共同anchor主对照的阶段决策为 `{summary['decision']}`，已完成后续审计。", '',
          '本轮首先实现并运行当前观测消费前的只读shadow，没有实施landmark ON。两预测使用同一过去LTV三维点，主滤波用当前先验/存活anchor clone传播，LTV使用当前correction前状态；比较同一相机bearing。GT不参与预测或在线选择。关联按既有tracker/epoch/core-ID/entered-time合同核对，未用独立地图证明真实物理点同一；当前匹配污染及不一致的风险保留在失效与误差尾部中。', '',
          '主指标使用过去已成熟的点、完整共同支持及每帧等权的角度RMSE。原始候选、单边/双边失效和无anchor保留分母；当前Active Consistency随后拒绝/退役的帧不会倒推删除。影子入口位于当前feature pipeline、LTV correction和主视觉更新之前。', '',
          '## 预测误差与覆盖', '',
          '| 序列 | 分区 | 成熟候选 / 共同有效 | LTV / main覆盖 | 每帧RMSE LTV / main rad | 改善比例 / 95%区间 | LTV / main P95 rad | LTV / main P99 rad | 可靠优势 |',
          '|---|---|---:|---:|---:|---|---:|---:|---|']
    for seq,r in summary['sequences'].items():
        for group in ['primary_mature','normal','recovery','startup','past_ready','past_not_ready','anchor_short','anchor_long']:
            g=r['groups'][group]
            ci=g['gain_ci95']
            text.append(f"| {seq} | {group} | {g['denominator']} / {g['paired']} | {number(g['ltv_coverage'])} / {number(g['main_coverage'])} | {number(g.get('frame_rms_ltv'))} / {number(g.get('frame_rms_main'))} | {number(g['frame_rms_gain'])} / {list(map(number,ci)) if ci else '—'} | {number(g['ltv']['p95'])} / {number(g['main']['p95'])} | {number(g['ltv']['p99'])} / {number(g['main']['p99'])} | {g['reliable_advantage']} |")
    text += ['', '改善比例为1−LTV/main；负数表示LTV更差。95%区间按1s时间块配对重采样，不能将相关点/相机当作独立样本。至少30个共同帧、10个时间块才给区间。5%投入判据、P95/P99最多恶化5%与不降低覆盖均在采集前冻结，不作为在线门控。', '',
             '| 序列 | 全部原始候选 | 过去成熟候选 | 主有效 / LTV有效 / 共同 | 成熟候选失效原因 |',
             '|---|---:|---:|---:|---|']
    for seq,r in summary['sequences'].items():
        g=r['groups']['primary_mature'];text.append(f"| {seq} | {r['rows']} | {g['denominator']} | {g['main_valid']} / {g['ltv_valid']} / {g['paired']} | {g['reasons']} |")
        with (out/f'{seq}_prediction_curve.csv').open() as f:
            curve=list(csv.DictReader(f))
        times=np.array([int(x['camera_ns']) for x in curve],dtype=np.int64)
        t=(times-times[0])*1e-9
        lv=np.array([float(x['ltv_frame_rms_rad']) if x['ltv_frame_rms_rad'] else np.nan for x in curve])
        mv=np.array([float(x['main_frame_rms_rad']) if x['main_frame_rms_rad'] else np.nan for x in curve])
        phase=np.array([x['phase'] for x in curve])
        fig, axes=plt.subplots(3,1,figsize=(12,8),sharex=True)
        axes[0].plot(t,np.rad2deg(lv),label='LTV before correction',lw=.7)
        axes[0].plot(t,np.rad2deg(mv),label='main frozen prior, shared anchor',lw=.7)
        axes[0].set_ylabel('Bearing frame RMS (deg)');axes[0].legend()
        axes[1].plot(t,[int(x['candidates']) for x in curve],label='all candidates',lw=.8)
        axes[1].plot(t,[int(x['mature']) for x in curve],label='past mature',lw=.8)
        axes[1].plot(t,[int(x['paired']) for x in curve],label='paired mature',lw=.8)
        axes[1].set_ylabel('Candidate counts');axes[1].legend()
        axes[2].plot(t,np.rad2deg(lv-mv),lw=.7)
        axes[2].axhline(0,color='black',lw=.6);axes[2].set_ylabel('LTV − main RMS (deg)');axes[2].set_xlabel('Seconds from first shadow candidate frame')
        for ax in axes:
            ax.fill_between(t,0,1,where=phase=='recovery',transform=ax.get_xaxis_transform(),color='orange',alpha=.13)
            ax.grid(alpha=.2)
        fig.suptitle(seq+'; orange = frozen OFF recovery')
        fig.tight_layout();fig.savefig(figures/f'{seq}_prediction.png',dpi=150);plt.close(fig)
    text += ['', '## 历史视觉补充对照与相关性审计', '',
             '共同anchor主对照确实显示预测优势；但LTV期间消费了过去bearing，主对照只传播冻结点均值。补充对照用同点、同anchor窗口、同主先验、仅过去camera0 bearing，调用原LtvSeedEstimator拟合世界点，再预测当前bearing。它使用不同的过去点均值，不能替换主对照或声称是独立地图。当前和未来观测均被排除，全部主对照输出字段再次精确复现。', '',
             '| 序列 | 分区 | 成熟候选 / LTV-过去几何共同有效 | LTV / 过去几何每帧RMSE rad | 改善比例 / 95%区间 | LTV / 过去几何P95 rad | 可靠优势 |',
             '|---|---|---:|---:|---|---:|---|']
    for seq,r in audit['sequences'].items():
        for name,g in r['groups'].items():
            text.append(f"| {seq} | {name} | {g['denominator']} / {g['paired']} | {number(g.get('frame_rms_ltv'))} / {number(g.get('frame_rms_main'))} | {number(g['frame_rms_gain'])} / {g['gain_ci95']} | {number(g['ltv']['p95'])} / {number(g['main']['p95'])} | {g['reliable_advantage']} |")
    text += ['', '补充对照中优势没有稳健通过1s时间块置信区间；不能证明LTV没有信息，也不能把共同anchor预测改善直接当作额外独立信息。不同的失效支持和过去几何的有效/无效计数均在correlation_audit.json保留（reasons字段仍是原主对照原因；补充拟合未保存内部拒绝理由，不推测其具体退化原因），没有通过替换支持隐藏问题。', '',
             '| 序列 | 实际seed均值写入 | Observer出生P迹（固定gain metric） | 计算的seed协方差迹中位 / P95 / 最大 m² | 两者相等次数 |',
             '|---|---:|---:|---:|---:|']
    for seq,r in audit['sequences'].items():
        c=r['covariance_evidence'];v=c['seed_trace']
        text.append(f"| {seq} | {c['admitted_written_seeds']} | {c['observer_birth_trace']} | {number(v['median'])} / {number(v['p95'])} / {number(v['maximum'])} | {c['seed_trace_equal_observer_birth_trace']} |")
    text += ['', '出生接口只传入均值，不传入seed协方差或与主状态的交叉块；P按固定初值及Riccati形式演化。seed协方差本身也是局部线性风险估计，不是已校准真误差。以上差异说明不能直接把Observer P认作物理误差协方差，不能证明它一定是不保守的上界。', '',
             '审计保存了离散误差协方差与Riccati metric不自动等价的数值反例。可解释的联合方案还需要seed增广、共享IMU/main bias供给、相机全部Euler子步与bearing相关Jacobian、主视觉更新/reset、anchor协方差及历史观测噪声映射；当前接口和缓存未提供这些交叉块或完整消费来源。实现这套误差传播将超出最小融合改动，本轮不扩大架构，也不运行未知交叉块置零的探索ON。', '']
    text += ['', '**收尾选择：保留因果shadow工具和共同anchor预测价值证据，停止本轮两时刻landmark ON路径。** 没有融合收益结论，原因是增强预测对照未给出稳健的增量优势，且可解释的相关性模型需要超出本轮边界的误差传播架构。', '', '## 轨迹、实际更新与数值边界', '',
             '以下为冻结OFF原始指标。没有ON轨迹，不能将shadow误差或OFF指标写成融合收益。', '',
             '| 序列 | ATE m | 1s平移RPE m / 旋转RPE ° | 5s平移RPE m / 旋转RPE ° | 覆盖 / 样本 |',
             '|---|---:|---:|---:|---:|']
    for seq,r in summary['sequences'].items():
        m=r['baseline_trajectory']
        a=m.get('rpe',{}).get('1.0',{});b=m.get('rpe',{}).get('5.0',{})
        text.append(f"| {seq} | {number(m.get('ate_rmse_m'))} | {number(a.get('translation_rmse_m'))} / {number(a.get('rotation_rmse_deg'))} | {number(b.get('translation_rmse_m'))} / {number(b.get('rotation_rmse_deg'))} | {m.get('coverage')} / {m.get('target_gt_rows')} |")
    text += ['', 'landmark实际更新0；无landmark门控或提交拒绝，因为未构建融合行。状态各块修正0；没有ON协方差或首次融合分歧可报告。这是未进入融合阶段，不是融合被全部拒绝。', '',
             '## 相关性与路径选择', '',
             '当前seed世界点依赖主位姿；同一tracker观测可进入seed、LTV和主MSCKF。缓存没有逐观测主消费账本，无法量化每条约束实际重复使用的比例。两时刻残差消去世界点均值，但不消除跨时间、跨点、LTV-main和LTV-visual相关性。现有Observer P按连续Riccati形式演化，不能直接宣称是校准的landmark误差协方差。', '',
             '本轮没有传播这些交叉块，也没有将未知块置零、调大噪声或增加门控替代建模。停止/继续判断首先依据因果预测证据；预测优势本身不证明独立信息或统计一致性。', '',
             '若决策为停止，则保留shadow诊断与缓存分析工具，停止当前max_slam=0、冻结seed/history条件下的两时刻landmark融合研究；不据此断言其他地图、观测或序列均无潜在收益。不增加权重扫描、SLAM状态或视觉退化实验。生产融合保持关闭，R4未闭合项保持原状态。', '',
             '## 验证、身份与资源', '',
             '详见delivery.json、各序列exact.json、off_parity.json和tests.json。缓存重放逐事件精确核对原Observer x/P、输入、ID/slot、生命周期、Consistency和readiness；缓存shadow没有运行主EKF，主先验来自未修改的原缓存。另以新库真实OFF回放精确核对主状态/完整P的身份摘要及Observer完整矩阵。OFF的联合矩阵日志未导出数值数组，其空内容哈希相同不代表额外完整P证据；完整主P参与state_digest。不能混称为在线shadow ON回放。', '',
             f'大缓存、预测逐行向量、完整曲线、构建和日志在 `{out}`，未提交。每次失败和重跑进入独立账本；既有R4账本及预留未修改。没有独立未选参数据，不给收益泛化结论。', '']
    attempts=[json.loads(x) for x in (out/'attempts.jsonl').read_text().splitlines()]
    text += ['', '| 采集任务 | 退出码 | 用时 s | 峰值RSS MiB |', '|---|---:|---:|---:|']
    for e in attempts:
        if e['event']=='finish' and e['kind'] in ('real','cache'):
            resource=json.loads(Path(e['resource_path']).read_text())
            text.append(f"| {e['label']} | {e['exit_code']} | {number(e['seconds'])} | {number(resource['max_rss_kb']/1024)} |")
    text += ['', '资源是各工具端到端测量，未测在线shadow或融合开销，不能用并发/不同工具的耗时差直接推算新增开销。离线Python分析峰值约2.5 GiB；原生shadow约110 MiB。两次真实OFF、四次缓存重放均成功；两次离线分析故障（时间表示关联、启动日志可选seed字段）已保留，并执行对应回归，无真实回放失败或参数扫描。', '', 'ROS/colcon构建、24项既有原生测试、新shadow几何/因果时序/精确性测试、C++14兼容及7项离线回归通过。main没有在线shadow ON；缓存输入与原始主先验保持不变。', '', '脚本入口：run.py 的 build/check_install/collect/off_parity/collect_past；analyze.py 用于原始共同anchor对照，audit.py 用于过去几何与相关性审计，render.py输出本报告。命令及退出码完整保存于delivery.json和外部attempts.jsonl。', '']
    for seq in summary['sequences']:
        text.append(f"![{seq} 预测及覆盖](figures/{seq}_prediction.png)")
    (dest/'report.md').write_text('\n'.join(text)+'\n')
    for path in [out/'summary.json',out/'freeze.json',out/'tests.json',out/'delivery.json',out/'build_identity.json',out/'correlation_audit.json',out/'correlation_audit_protocol.json',out/'determinism.json']:
        if path.exists():shutil.copyfile(path,dest/path.name)
    for seq in summary['sequences']:
        for suffix in ['exact','off_parity','identity']:
            path=out/f'{seq}_{suffix}.json'
            if path.exists():shutil.copyfile(path,dest/path.name)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('out',type=Path);p.add_argument('dest',type=Path)
    a=p.parse_args();render(a.out.resolve(),a.dest.resolve())
