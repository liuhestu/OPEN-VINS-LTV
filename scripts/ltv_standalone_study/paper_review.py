#!/usr/bin/env python3
"""Offline paper-inspired baseline review. Never schedules an observer simulation."""
from run_study import DEFAULT, DOC, ROOT, config, code_identity, digest, write, frozen_check
import argparse
import csv
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from truth_model import geometry, truth
from evaluate import settle

SCENES = ['REGULAR', 'FAST', 'PAPER']
GRADES = {'wide': (.1, 1., .2, .05), 'strict': (.05, .5, .1, .02)}


def load(path):
    with np.load(path) as z:
        return {k: z[k] for k in z.files}


def merged(trace, camera, horizon):
    t = np.r_[trace['t'], camera['t']]
    side = np.r_[np.full(len(trace['t']), 'common'), camera['side']]
    v = np.r_[trace['values'], camera['values']]
    l = np.r_[trace['landmarks'], camera['landmarks']]
    order = np.argsort(t, kind='stable'); order = order[t[order] <= horizon]
    return t[order], v[order], l[order], side[order]


def settling(trace, camera, horizon, n):
    t, v, l, _ = merged(trace, camera, horizon)
    times, index = np.unique(t, return_index=True)
    result = {}
    for grade, (vm, ga, gm, lm) in GRADES.items():
        V = v[:, 0] <= vm
        G = (v[:, 2] <= ga) & (v[:, 3] <= gm)
        L = l[:, :n, 1] <= lm
        masks = {'t_V': V, 't_G': G, 't_VG': V & G, 't_L': np.all(L, axis=1)}
        result[grade] = {k: settle(times, np.logical_and.reduceat(mask, index)) for k, mask in masks.items()}
    return result


def excitation(scene, hz):
    points, _, pc = geometry(scene)
    t = np.arange(10 * hz + 1) / hz
    centers = np.array([p + R @ pc for R, p, *_ in (truth(tt, scene) for tt in t)])
    delta = points[None] - centers[:, None]
    ranges = np.linalg.norm(delta, axis=2); b = delta / ranges[:, :, None]
    projection = np.eye(3) - b[:, :, :, None] * b[:, :, None, :]
    result = {}
    for T in [4, 5, 10]:
        W = np.trapz(projection[:T * hz + 1], t[:T * hz + 1], axis=0) / T
        result[T] = (W, np.linalg.eigvalsh(W)[:, 0], ranges[:T * hz + 1])
    return result


def savefig(fig, destination, name):
    for ax in fig.axes:
        ax.grid(alpha=.2)
    fig.tight_layout()
    fig.savefig(destination / (name + '.png'), dpi=140)
    path = destination / (name + '.svg')
    fig.savefig(path, metadata={'Date': None})
    path.write_text('\n'.join(line.rstrip() for line in path.read_text().splitlines()) + '\n')
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, default=DEFAULT, help='Existing immutable simulation output')
    parser.add_argument('--destination', type=Path, default=DOC / 'paper_baseline_review')
    args = parser.parse_args(); out, dest = args.out.resolve(), args.destination.resolve()
    ledger_path = out / 'ledger.json'; ledger_sha = digest(ledger_path.read_bytes())
    entries = json.loads(ledger_path.read_text()); selected = []
    for scene in SCENES:
        duration = 20 if scene == 'PAPER' else 60
        for impl, tight in [('CONT_REF', False), ('CORE', False), ('CONT_REF', True)]:
            expected = config(impl, scene, duration=duration, tight=tight)
            matches = [e for e in entries if e['config'] == expected and e['status'] == 'COMPLETED' and e['exit_code'] == 0]
            if len(matches) != 1:
                raise RuntimeError(f'Missing/ambiguous required identity: {expected}')
            selected.append(matches[0])
    expected = config('CONT_REF', duration=120)
    extension = [e for e in entries if e['config'] == expected and e['status'] == 'COMPLETED']
    if len(extension) != 1:
        raise RuntimeError('Missing/ambiguous REGULAR 120 s continuous diagnostic')
    selected += extension
    existing = json.loads((DOC / 'evidence/combined_settling.json').read_text())
    references = json.loads((out / 'reference_checks.json').read_text())
    for scene in SCENES:
        ids = [e['id'] for e in selected if e['config']['scene'] == scene and e['config']['impl'] == 'CONT_REF' and e['config']['duration'] <= 60]
        assert any(r['base'] in ids and r['tight'] in ids and r['pass'] for r in references)
    dest.mkdir(parents=True, exist_ok=True)
    plt.rcParams['svg.hashsalt'] = 'paper-baseline-review'
    sources = {str(ledger_path): ledger_sha}; data = {}; results = {}
    for e in selected:
        rd = Path(e['directory']); c = e['config']; n = len(geometry(c['scene'])[0])
        assert json.loads((rd/'config.json').read_text()) == c
        assert json.loads((rd/'code.json').read_text()) == code_identity()
        tr, cam = load(rd/'trace.npz'), load(rd/'camera.npz')
        assert np.array_equal(tr['t'], np.arange(round(c['duration']*200)+1)/200)
        assert np.all(tr['ids'][:, :n] == np.arange(n))
        assert tr['x'].shape[1] >= 3*n+6
        full = settling(tr, cam, c['duration'], n)
        for grade in GRADES:
            for short, old in [('t_V','t_V_all_samples'),('t_G','t_G_all_samples'),('t_VG','t_VG_all_samples'),('t_L','t_L_all_samples')]:
                assert full[grade][short] == existing[e['id']][grade][old]
        results[e['id']] = {'config':c, 'common_20s':settling(tr, cam, 20, n), 'native_horizon':full}
        data[e['id']] = (tr, cam)
        for name in ['config.json','code.json','trace.npz','camera.npz','metrics.json']:
            p=rd/name; sources[str(p)]=digest(p.read_bytes())
    geometric = {}; rows = []; precision = {}
    for scene in SCENES:
        coarse, fine = excitation(scene, 200), excitation(scene, 400)
        points, _, _ = geometry(scene); R0,p0,*_ = truth(0,scene)
        initial_error = np.linalg.norm(points-p0,axis=1)
        geometric[scene] = {}
        for T in [4,5,10]:
            W, eig, ranges = coarse[T]
            discrepancy = float(np.max(abs(eig-fine[T][1])))
            precision[f'{scene}_{T}'] = {'max_absolute_eigenvalue_difference':discrepancy,'pass':discrepancy<=1e-6}
            geometric[scene][T] = {'minimum':float(eig.min()),'median':float(np.median(eig)),'W_per_point':W.tolist(),'lambda_min_per_point':eig.tolist()}
            for j in range(len(points)):
                rows.append({'scene':scene,'window_s':T,'id':j,'lambda_min':eig[j],'range_min_m':ranges[:,j].min(),'range_median_m':np.median(ranges[:,j]),'initial_landmark_error_m':initial_error[j]})
        expected = {'REGULAR':(.00243,.00399),'FAST':(.02457,.03601),'PAPER':(.09606,.16251)}[scene]
        assert np.allclose([geometric[scene][4]['minimum'],geometric[scene][4]['median']],expected,rtol=0,atol=5e-6)
    with (dest/'excitation_per_point.csv').open('w') as f:
        writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    write(dest/'excitation.json',geometric);write(dest/'quadrature_precision.json',precision)
    # Preserve each failing side, plus the grouped terminal good suffix.
    paper = next(e for e in selected if e['config']['scene']=='PAPER' and e['config']['impl']=='CORE')
    t,v,l,sides=merged(*data[paper['id']],20)
    masks={'velocity':v[:,0]>.05,'gravity_angle':~(v[:,2]<=.5),'gravity_magnitude':v[:,3]>.1,'landmark_worst':~np.all(l[:,:16,1]<=.02,axis=1)}
    failures=[]; blockers={}
    relative=l[:,:16,1]
    worst=np.max(np.where(np.isfinite(relative),relative,-np.inf),axis=1)
    worst[~np.isfinite(worst)]=np.nan
    for name, bad in masks.items():
        values={'velocity':v[:,0],'gravity_angle':v[:,2],'gravity_magnitude':v[:,3],'landmark_worst':worst}[name]
        unique,index=np.unique(t,return_index=True); good=~np.logical_or.reduceat(bad,index); ix=np.flatnonzero(~good)
        start=ix[-1]+1 if len(ix) else 0
        suffix=float(unique[start]) if start<len(unique) else None
        blockers[name]={'last_bad_time':float(t[bad][-1]) if bad.any() else None,'last_bad_sides':sorted(set(sides[bad & (t==(t[bad][-1] if bad.any() else -1))])),'terminal_good_suffix_start':suffix,'remaining_validation_seconds':None if suffix is None else 20-suffix,'failed_samples_after_15s':int(np.sum(bad & (t>=15)))}
        for i in np.flatnonzero(bad):
            failures.append({'metric':name,'time_s':t[i],'side':sides[i],'value':values[i] if np.isfinite(values[i]) else 'undefined'})
    with (dest/'paper_core_strict_failures.csv').open('w') as f:
        writer=csv.DictWriter(f,fieldnames=['metric','time_s','side','value']);writer.writeheader();writer.writerows(failures)
    write(dest/'paper_core_strict_diagnosis.json',blockers)
    figures=[]
    labels=['Velocity error [m/s]','Gravity vector error [m/s²]','Gravity direction [deg]','Gravity magnitude error [m/s²]','Landmark RMS / worst [m]','Landmark relative RMS / worst']
    for horizon in ['common20','native']:
        fig, axes=plt.subplots(6,3,figsize=(16,18))
        for col,scene in enumerate(SCENES):
            for e in selected:
                c=e['config']
                if c['scene']!=scene or c['tight'] or c['duration']>60: continue
                tr,_=data[e['id']]; n=len(geometry(scene)[0]); mask=tr['t']<=20 if horizon=='common20' else np.ones(len(tr['t']),dtype=bool)
                for row in range(4): axes[row,col].plot(tr['t'][mask],tr['values'][mask,row],label=c['impl'],lw=.8)
                for row,index in [(4,0),(5,1)]:
                    errors=tr['landmarks'][mask,:n,index]
                    axes[row,col].plot(tr['t'][mask],np.sqrt(np.mean(errors**2,axis=1)),label=c['impl']+' RMS',lw=.8)
                    axes[row,col].plot(tr['t'][mask],errors.max(axis=1),label=c['impl']+' worst',ls='--',lw=.6)
            axes[0,col].set_title(scene)
            for row in range(6): axes[row,col].set_ylabel(labels[row]);axes[row,col].legend(fontsize=6)
            axes[-1,col].set_xlabel('Time [s]')
        name='errors_'+horizon;savefig(fig,dest,name);figures.append(name)
    e=extension[0];tr,_=data[e['id']];fig,axes=plt.subplots(3,1,figsize=(10,8))
    for ax,col in zip(axes,[0,1]): ax.plot(tr['t'],tr['values'][:,col]);ax.set_ylabel(labels[col])
    axes[2].plot(tr['t'],np.sqrt(np.mean(tr['landmarks'][:,:30,0]**2,axis=1)));axes[2].set_ylabel('Landmark RMS [m]');axes[2].set_xlabel('Time [s]')
    for ax in axes: ax.axvline(60,color='gray',ls='--')
    savefig(fig,dest,'REGULAR_extension120');figures.append('REGULAR_extension120')
    fig,axes=plt.subplots(1,3,figsize=(13,4))
    for ax,T in zip(axes,[4,5,10]):
        ax.boxplot([geometric[s][T]['lambda_min_per_point'] for s in SCENES],labels=SCENES);ax.set_title(f'[0, {T}] s');ax.set_ylabel('Per-landmark minimum eigenvalue')
    savefig(fig,dest,'excitation_distribution');figures.append('excitation_distribution')
    status='COMPLETED' if all(p['pass'] for p in precision.values()) else 'BLOCKED_REFERENCE'
    write(dest/'results.json',{'status':status,'runs_before':len(entries),'runs_after':len(entries),'new_simulations':0,'results':results,'integrity':frozen_check()})
    lines=['# 论文启发场景与几何激励复核','',f'状态：**{status}**。仅离线提取与解析积分，完整仿真仍为 {len(entries)}/64。','',
           '## 同一判据下的达标时间','',
           '每格为宽／严档秒数；“未观测到”只表示该窗口未满足持续至结束且至少 5 s 验证。检查合并 200 Hz 和相机前后样本，时间分辨率 0.005 s。', '',
           '|身份|评价窗口|t_V|t_G|t_VG|t_L 全部点|','|---|---|---|---|---|---|']
    def fmt(x): return f'{x:.3f}' if isinstance(x,(float,int)) else '未观测到'
    for e in selected:
        if e['config']['tight']: continue
        for window in ['common_20s','native_horizon']:
            result=results[e['id']][window]
            lines.append('|'+e['id']+'|'+('0–20 s' if window=='common_20s' else f"0–{e['config']['duration']} s")+'|'+'|'.join(fmt(result['wide'][k])+' / '+fmt(result['strict'][k]) for k in ['t_V','t_G','t_VG','t_L'])+'|')
    lines += ['','## 前缀窗口激励','', 'W_i=(1/T)∫(I−b_i b_iᵀ)dt，b_i 为相机光心到路标的世界单位方向，包含杆臂。下表不是全时域滑动窗口统计。','', '|场景|窗口 s|逐点最小值|逐点中位值|200/400 Hz 最大绝对差|','|---|---:|---:|---:|---:|']
    for scene in SCENES:
        for T in [4,5,10]:
            g=geometric[scene][T];lines.append(f"|{scene}|{T}|{g['minimum']:.8f}|{g['median']:.8f}|{precision[f'{scene}_{T}']['max_absolute_eigenvalue_difference']:.3g}|")
    lines += ['','逐点完整矩阵与特征值见 [excitation.json](excitation.json)，距离、初始零估计误差见 [CSV](excitation_per_point.csv)。400 Hz 只细化解析几何积分，不运行观测器。验收门限固定为绝对差≤1e−6。','',
              '## S_PAPER CORE 严档诊断','', '|指标|最后越界 s|最后越界侧|终末连续合格起点 s|剩余验证 s|15 s 后越界样本数|','|---|---:|---|---:|---:|---:|']
    for name,b in blockers.items(): lines.append(f"|{name}|{b['last_bad_time']}|{','.join(b['last_bad_sides'])}|{b['terminal_good_suffix_start']}|{b['remaining_validation_seconds']}|{b['failed_samples_after_15s']}|")
    lines += ['','逐条越界保存在 [CSV](paper_core_strict_failures.csv)，相同物理时刻的不同侧别分别记录，不把重复侧别计数解释为持续时间。','',
              '## 论文公开量与补充假设','',
              '依据[本地论文 Section V、式 (6)/(15)/(16)](../../../Pose_Velocity_and_Landmark_Position_Estimation_Using_IMU_and_Bearing_Measurements.pdf)。', '',
              '|类别|本轮采用值与边界|','|---|---|',
              '|公开运动|p=2[sin t,sin t cos t,1]，ω=[−cos 2t,1,sin 2t]，R(0)=I；解析 R=Exp(t[−1,3,0]×)Ry(−2t) 满足该角速度|',
              '|公开输入与外参|g=[0,0,−9.81]，a=Rᵀ(p̈−g)，R_BC=I，p_BC=[.02,.06,.01] m|',
              '|公开第一层参数|16 个地面点；v、η、路标估计均为零；Q=1e−4 I，V=1e6 I|',
              '|本轮补充点坐标|x,y∈{−1.5,−.5,.5,1.5}，z=0；不是作者公开的精确坐标|',
              '|本轮补充初始矩阵|P(0)=I，保留完整 Riccati 与交叉块|',
              '|本轮参考求解|DOP853，rtol=1e−9、atol=1e−11、max_step=.005；加严为1e−11、1e−13、.0025|',
              '|本轮离散设定|CORE IMU 200 Hz、相机20 Hz、中点 IMU；t=0建槽、首次校正长度零|',
              '|可见性模型|理想有符号 bearing，不裁剪、不翻轴；约49.63%的真实相机Z为负，不证明真实相机始终可见|',
              '|论文图示与本轮指标|Fig.3含第二层 R̂η̂，本轮为第一层η误差；不等同原图，也不使用第二层k_R=40、k_p=100调第一层|', '',
              '## 归因与下一步','',
              'S_PAPER 的宽档连续 11.450 s、CORE 10.555 s，明显快于 REGULAR，但没有复现一个论文未定义的“4秒联合达标”门限。前4秒激励最小值约为 REGULAR 的39.48倍；这是场景依赖的证据，不是收敛速率倍率，也不是单因素因果证明。路标数量、距离、初始绝对误差和运动同时变化。', '',
              'CORE 严档的瓶颈是速度：最后越界发生在15.200 s相机校正前，合并检查从15.205 s起持续合格至20 s，仅剩4.795 s，未满足5 s验证。因此不是截至20 s仍持续失控，也不能报告已经验收收敛或推断20 s后的行为。重力方向、模长和全部路标分别从10.205、9.355、13.505 s起终末持续合格。相机前越界说明采样/校正两侧差异影响严档验收；没有PAPER SPLIT_REF，不能进一步将其唯一归因于子流积分或分步冻结。参考加严已通过，不能把 REGULAR 的85.975秒外推为原观测器普遍收敛慢。','',
              '下一步优先核对论文未公开的点坐标、P(0)和数值设定，并审阅 S_PAPER 严档离散差异。若后续要隔离几何因果或拆分 S_PAPER 的分步误差，应另立受控实验；本轮不自动消耗剩余预算。种子质量判断保留为短寿命问题的条件性分支，不作为论文正向基准核对之前的首要修改。','',
              '本轮不拟合Fig.3、不证明全时间PE、不启动第二层或ROS/VIO/ATE、不更新生产参数。历史研究的两项执行偏差仍见[原决策日志](../decision_log.md)。','', '## 曲线与追溯','',
              '公共曲线全部为真实机体系误差范数，路标RMS按点数归一化；虚线为最差点。重力方向在零估计时无定义，保留缺失。完整时域单独展示，不用截短结果替换原始判据。','']
    for f in figures: lines.append(f'- [{f} PNG]({f}.png) / [SVG]({f}.svg)')
    lines += ['','复现：`python3 scripts/ltv_standalone_study/paper_review.py`。支持 `--out`（既有数据目录）、`--destination`（新报告目录）。缺失身份直接报错，不触发仿真。']
    (dest/'report.md').write_text('\n'.join(lines)+'\n')
    assert digest(ledger_path.read_bytes())==ledger_sha
    for p in [Path(__file__), ROOT/'scripts/ltv_standalone_study/truth_model.py', DOC/'evidence/combined_settling.json', out/'reference_checks.json', ROOT/'docs/Pose_Velocity_and_Landmark_Position_Estimation_Using_IMU_and_Bearing_Measurements.pdf']:
        sources[str(p)]=digest(p.read_bytes())
    write(dest/'sources_sha256.json',sources)
    write(dest/'artifacts_sha256.json',{p.name:digest(p.read_bytes()) for p in sorted(dest.iterdir()) if p.is_file() and p.name!='artifacts_sha256.json'})
    print(status, 'new simulations: 0; budget:',len(entries),'/64')
    print(json.dumps(blockers,indent=2))
    if status!='COMPLETED': raise RuntimeError('Geometry quadrature precision contract failed')


if __name__=='__main__':
    main()
