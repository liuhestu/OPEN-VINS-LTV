"""Render bounded V20/V10 results, unit-separated curves and reviewable evidence."""
import argparse
import csv
import json
from pathlib import Path
import shutil

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def fmt(x):
    return '—' if x is None else f'{x:.6g}'


def main():
    p=argparse.ArgumentParser();p.add_argument('out',type=Path);p.add_argument('doc',type=Path);args=p.parse_args()
    out,doc=args.out,args.doc
    doc.mkdir(parents=True,exist_ok=True);figures=doc/'figures';figures.mkdir(exist_ok=True)
    summary=json.loads((out/'summary.json').read_text())
    bad=any(s['quality_decision']=='RETAIN_20' for s in summary.values())
    lines=['# V-only 20/10 帧确认等待敏感性结果','',
        '**工程结论：'+('保留原20帧策略。新增放行输出触发预定义质量恶化条件，不将更新数量增加视为收益。' if bad else
        '10帧在本组实验中恢复更及时，未触发预定义输出质量恶化条件；V2_03的短ready段与切换明显增加，生产默认仍保留20帧，不宣称轨迹收益。')+'**','',
        '基线为 f22bfbcd82fe9a2b82d85ce9079fcd33d496e3f8，分支 feature/ltv-v-readiness-recovery。唯一算法变化为实验 runner 的 V10：V strict-good 确认改10，G 确认及生产默认仍20。G 融合关闭，噪声、NIS、成熟/观测/有效修正、prediction angle、correction rate、初始化、时间和grace全部冻结。实验字段不从生产YAML解析，仅允许0（默认）或10，不扫描其他长度。','',
        '预先固定的接受口径及区间定义见 [protocol.md](protocol.md)。仅四次完整真实回放：两次V20精确复现与两次V10；OFF及原V基线复用。数据不裁剪，GT仅用于离线评价。小幅ATE/RPE变化不作为收益或一致性证明。','',
        '## 新增实际更新及恢复','',
        '| 序列 | 原V20实际 | V10实际 | 新增 / 减少实际 | 配对恢复episode | 首次恢复缩短均值 / 中位 / P95 s | V20 / V10 ready开启次数 | V20 / V10 <1s ready段 |',
        '|---|---:|---:|---:|---:|---:|---:|---:|']
    for seq,s in summary.items():
        d=s['paired_recovery_shortening_s'];a=s['V20_toggles'];b=s['V10_toggles']
        lines.append(f"| {seq} | {s['actual_V20']} | {s['actual_V10']} | {s['added_actual']} / {s['lost_actual']} | {d['n']} | {fmt(d['mean'])} / {fmt(d['p50'])} / {fmt(d['p95'])} | {a['ready_on_transitions']} / {b['ready_on_transitions']} | {a['ready_segments_under_1s']} / {b['ready_segments_under_1s']} |")
    lines+=['','实际更新要求receipt被消费、V_rows=3、joint_applied、ekf_calls=1，未以非零状态变化替代。恢复episode以原V20的ready→not-ready为共同锚点；首次V10 ready计时早于原返回，并不保证之后一直ready。完整锚点、右删失和提前放行后再次撤销的次数保留在 summary.json；尾段不删除。','',
        '| 序列 | 首次实际更新恢复缩短均值 / 中位 / P95 s | V10 ready但无实际更新 | 原core检查拒绝原因 |',
        '|---|---:|---:|---|']
    for seq,s in summary.items():
        d=s['paired_actual_recovery_shortening_s'];reasons=s['V10_ready_without_actual']
        lines.append(f"| {seq} | {fmt(d['mean'])} / {fmt(d['p50'])} / {fmt(d['p95'])} | {sum(reasons.values())} | {reasons} |")
    lines+=['','这些ready但未更新帧的core healthy_updates仍为10–19 / 12–19，没有达到原core连续20帧warmup；该条件完整保留。因此不能把所有新增ready算作新增有效更新。','',
        '| 序列 | 原 / 短确认首次ready（相对首相机包 s） | 原 / 短确认切换每分钟 | 提前放行后又撤销的episode | 原策略右删失episode |',
        '|---|---:|---:|---:|---:|']
    for seq,s in summary.items():
        a,b=s['V20_toggles'],s['V10_toggles']
        lines.append(f"| {seq} | {fmt(a['first_ready_elapsed_s'])} / {fmt(b['first_ready_elapsed_s'])} | {fmt(a['transitions_per_minute'])} / {fmt(b['transitions_per_minute'])} | {sum(x['V10_off_transitions_before_V20_return']>0 for x in s['recoveries'])} | {sum(x['V20_right_censored'] for x in s['recoveries'])} |")
    lines+=['','新增事件的全部时间位置和减少事件均保存；下面仅列最长的五段新增连续片段（相邻物理时间超过1.5倍相机周期即切段）：','',
        '| 序列 | 起止 s | 新增帧 |','|---|---|---:|']
    for seq,s in summary.items():
        for r in sorted(s['added_intervals'],key=lambda x:x['frames'],reverse=True)[:5]:
            lines.append(f"| {seq} | {r['start_s']:.3f}–{r['end_s']:.3f} | {r['frames']} |")
    lines+=['','## 新放行速度质量及修正方向','',
        '物理body速度GT在同一camera+offset时间进行SLERP姿态和发布世界速度插值（GT间隙≤10ms），无时间/姿态/尺度拟合。基线和短确认使用相同固定支持。下表以全部实际事件为分母，缺少GT的帧显式列出，误差单位m/s。新增V10与原V20同时间原始LTV配对，原策略该帧未实际更新。','',
        '| 序列 | 输出集合 | 事件 / 可评价 / 缺失 | RMSE | 中位 | P95 | 最大 | >0.1 / >0.2 / >1 m/s 帧 |',
        '|---|---|---:|---:|---:|---:|---:|---:|']
    for seq,s in summary.items():
        for k in ('original_actual','short_actual','added_short','added_original_same_time','lost_original'):
            z=s['admissions'][k];e=z['error_mps'];tails=z['tails']
            lines.append(f"| {seq} | {k} | {z['target']} / {z['evaluated']} / {z['missing']} | {fmt(e['rms'])} | {fmt(e['p50'])} | {fmt(e['p95'])} | {fmt(e['maximum'])} | {tails['0.1']['count']} / {tails['0.2']['count']} / {tails['1.0']['count']} |")
    lines+=['','新放行分布与原实际放行分布不同可能反映更早进入较困难窗口，不等同于observer算法本身变差；因此保留同时间原始输出对照。但新的实际注入集合必须满足预定义工程质量口径。未用GT决定任何在线放行。','',
        '| 序列 | 新增可评价修正 | 使主先验body速度误差降低 | LTV比主先验更准 | 误差修正均值 m/s | 姿态修正P95 ° | 速度修正P95 m/s | 位置修正P95 m |',
        '|---|---:|---:|---:|---:|---:|---:|---:|']
    for seq,s in summary.items():
        d=s['added_direction'];c=s['added_corrections']
        lines.append(f"| {seq} | {d['evaluated']} | {d['improves']} | {d['ltv_better_than_prior']} | {fmt(d['error_change_mps']['mean'])} | {fmt(c['attitude_deg']['p95'])} | {fmt(c['velocity_mps']['p95'])} | {fmt(c['position_m']['p95'])} |")
    lines+=['','贡献由保存的完整联合P/H/S重算K后取V列乘残差；body误差方向诊断是对冻结先验注入该分支贡献，不是额外在线EKF或因果轨迹收益。姿态、速度、位置不混成一个范数。新增向量、NIS和先验误差逐帧保存在 *_admissions.csv。','',
        '| 序列 | 分区 | 原 / 短确认实际帧 | 新增实际帧 | 原 / 短确认速度 RMSE m/s |',
        '|---|---|---:|---:|---:|']
    for seq,s in summary.items():
        for phase in ('normal','recovery','startup'):
            z=s['phase_quality'][phase];a,b=z['V20'],z['V10']
            lines.append(f"| {seq} | {phase} | {a['target']} / {b['target']} | {z['added']['target']} | {fmt(a['error_mps']['rms'])} / {fmt(b['error_mps']['rms'])} |")
    lines+=['',
        '质量恶化条件预先固定为新增RMSE>原实际1.25倍且绝对增加>0.01m/s，或P95增加>0.05m/s，或>0.2m/s比例增加>5个百分点；0.1/0.2/1m/s仅为离线尾部指标。','',
        '| 序列 | RMSE条件 | P95条件 | 尾部条件 | 工程选择 | 原硬健康条件逐帧检查 / grace帧 |',
        '|---|---|---|---|---|---:|']
    for seq,s in summary.items():
        z=s['deterioration_flags'];h=s['health']
        lines.append(f"| {seq} | {z['rmse_25pct_and_0_01']} | {z['p95_plus_0_05']} | {z['tail_0_2_plus_5pp']} | {s['quality_decision']} | {h['ready_frames_checked']} / {h['existing_grace_frames']} |")
    lines+=['','所有V10 ready帧仍满足原始current、pool、成熟/观测15、有效修正20、有效诊断、重力物理边界与时间条件。strict或原有soft-grace envelope分别核对，grace仍最多2帧/0.1s，没有延长或放行无效诊断。首次主状态/P分歧必须对应新增实际V更新，检查通过。','',
        '## 全程、正常及恢复段轨迹','',
        '共同支持来自原OFF初始化，GT最近邻≤20ms、输出匹配≤1us、SE(3)无尺度。V列表示原V20。每个模式仅使用全程共同支持的一次拟合，分区不重新对齐。启动保留。正常/恢复从原V20冻结，与GT无关；恢复是V20中断加每次返回ready后1s。分区RPE要求端点和整个采样支持窗口处于同一分区，5s没有有效配对时显式空缺。','',
        '| 序列 | 模式 | 区间 | 样本 | ATE m | 1s平移 m / 旋转 ° / 对数 | 5s平移 m / 旋转 ° / 对数 |',
        '|---|---|---|---:|---:|---:|---:|']
    for seq,s in summary.items():
        for mode,groups in s['trajectory'].items():
            for group,z in groups.items():
                rp=z['rpe'];r1,r5=rp['1.0'],rp['5.0']
                lines.append(f"| {seq} | {mode} | {group} | {z['samples']} | {fmt(z['ate_rmse_m'])} | {fmt(r1['translation_rmse_m'])} / {fmt(r1['rotation_rmse_deg'])} / {r1['pairs']} | {fmt(r5['translation_rmse_m'])} / {fmt(r5['rotation_rmse_deg'])} / {r5['pairs']} |")
    lines+=['','完整coverage、初始化差、定位失败判定和原始指标都在 summary.json。工程选项的结论只涉及本组恢复及输出质量；不将ATE微小变化称为收益，也不宣布相关性、一致性或原R4恢复验收通过。','',
        '## 曲线、身份和验证','']
    for seq in summary:
        raw=np.genfromtxt(out/(seq+'_admissions.csv'),delimiter=',',names=True,dtype=None,encoding='utf-8')
        t=raw['elapsed_s'];added=raw['added'].astype(bool)
        fig,axs=plt.subplots(3,1,figsize=(13,8),sharex=True)
        axs[0].plot(t,raw['V20_ltv_error_mps'],label='V20 LTV',lw=.8)
        axs[0].plot(t,raw['V10_ltv_error_mps'],label='V10 LTV',lw=.8)
        axs[0].scatter(t[added],raw['V10_ltv_error_mps'][added],label='new actual',s=8,color='red')
        axs[0].set_yscale('log');axs[0].set_ylabel('Body speed error (m/s, log)');axs[0].legend(ncol=3)
        for name,y in [('ready_V20',2),('ready_V10',1),('added',0)]:
            mask=raw[name].astype(bool);axs[1].scatter(t[mask],np.full(mask.sum(),y),s=3)
        axs[1].set_yticks([0,1,2],['New actual','V10 ready','V20 ready']);axs[1].set_ylim(-.5,2.5)
        axs[2].plot(t,raw['observed'],label='observed',lw=.8);axs[2].plot(t,raw['mature'],label='mature',lw=.8)
        axs[2].axhline(15,color='gray',ls='--');axs[2].set_ylabel('LTV features');axs[2].legend()
        axs[-1].set_xlabel('Seconds since first frozen camera packet');fig.suptitle(seq+' fixed V20/V10 admission experiment')
        fig.tight_layout();fig.savefig(figures/(seq+'_admissions.png'),dpi=130);plt.close(fig)
        fig,axs=plt.subplots(4,1,figsize=(13,9),sharex=True)
        axs[0].plot(t[added],raw['V10_branch_error_change_mps'][added],'.',ms=3);axs[0].axhline(0,color='gray')
        axs[0].set_ylabel('Branch error change\n(m/s; negative improves)')
        for ax,k,label in zip(axs[1:],['attitude_deg','velocity_mps','position_m'],['Attitude (deg)','Velocity (m/s)','Position (m)']):
            ax.plot(t[added],raw['correction_'+k][added],'.',ms=3);ax.set_ylabel(label)
        axs[-1].set_xlabel('Seconds since first frozen camera packet');fig.suptitle(seq+' new actual V contributions')
        fig.tight_layout();fig.savefig(figures/(seq+'_contributions.png'),dpi=130);plt.close(fig)
        lines.append(f'- [{seq} 输出质量/ready/供给全时间曲线](figures/{seq}_admissions.png)；[新增修正方向与分状态块贡献](figures/{seq}_contributions.png)。')
        fig,ax=plt.subplots(figsize=(13,4))
        origin=raw['imu_time'][0]-raw['elapsed_s'][0]
        for mode,label in [('OFF','OFF'),('V','V20'),('V10','V10')]:
            curve=np.genfromtxt(out/(seq+'_'+mode+'_trajectory_curve.csv'),delimiter=',',names=True,dtype=None,encoding='utf-8')
            ax.plot(curve['time']-origin,curve['translation_error_m'],lw=.8,label=label)
        ax.set_xlabel('Seconds since first frozen camera packet');ax.set_ylabel('Aligned translation error (m)')
        ax.set_title(seq+' full common support; one global SE(3) fit per mode');ax.legend()
        fig.tight_layout();fig.savefig(figures/(seq+'_trajectory.png'),dpi=130);plt.close(fig)
        lines.append(f'- [{seq} 全程共同支持轨迹误差曲线](figures/{seq}_trajectory.png)。')
    lines+=['','外部证据目录：`'+str(out)+'`。包含原始矩阵/cache、逐帧admissions与trajectory曲线、每次identity/退出码、完整compiled/build身份和账本。新增真实回放4，原开始66→70；失败的开发测试和重跑另记入原账本，没有扩展真实矩阵。','',
        'V20精确复现：主状态/完整P、LTV完整x/P、输入cache、ID/slot/lifecycle/consistency/readiness、融合事件与完整矩阵；仅排除既有features顶层compute_time_ms。instrumentation只新增确认长度20元数据；effective_options仍与原基线完全一致。V10唯一配置差别由实验mode及instrumentation的10记录，原输入YAML不变。','',
        '实际ROS Humble/colcon按原选项构建；原24个相关安装测试通过，新增C++14 readiness测试和4项离线评价测试通过。纯离线重算使用同一冻结GT方法；数值派生重复核对。命令和退出码见 delivery.json、tests.json、parity_*.json。','',
        '## 下一阶段','',
        '本轮实验矩阵结束，不继续扫描确认长度或融合噪声，G保持关闭。提交本结果后只进入landmark残差与相关性方案设计：seed世界点依赖主位姿，不是独立地图；max_slam=0。设计需明确状态/锚定/ID/epoch/时间、跨observer/main/visual的相关性和重复观测使用，不直接实施landmark ON。']
    (doc/'report.md').write_text('\n'.join(lines)+'\n')
    for name in ('summary.json','tests.json','build_identity.json','compiled_source.json','analysis_manifest.json','matrix_status.json','budget_before.json','budget_after.json','development_failures.json'):
        if (out/name).exists():shutil.copyfile(out/name,doc/name)
    for path in out.glob('parity_*.json'):shutil.copyfile(path,doc/path.name)


if __name__=='__main__':
    main()
