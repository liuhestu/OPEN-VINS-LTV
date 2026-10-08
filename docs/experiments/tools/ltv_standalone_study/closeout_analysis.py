"""Closeout-only offline evidence; historical run directories are read-only."""
import json
from pathlib import Path
import numpy as np
from run_study import config,ledger,metric,write
from paper_review import settling,load
from truth_model import geometry,sample
from diagnostics import compare_runs,repeated_transients,track_outcomes,seed_summary,peak_diagnostics,camera_correction_deltas


def find(out,**kw):
    expected=config(**kw)
    matches=[e for e in ledger(out) if e['config']==expected and e['status']=='COMPLETED']
    if len(matches)!=1:raise RuntimeError('Missing/ambiguous identity '+str(expected))
    return matches[0]


def gain_fixed(out):
    rows=[]
    for candidate in ['C0','C2','C4','C6']:
        for impl in ['CONT_REF','EXPERIMENTAL']:
            e=find(out,impl=impl,candidate=candidate);m=metric(e)
            rows.append({'run':e['id'],'candidate':candidate,'impl':impl,'parameters':e['config'],'J_tail':m['J_tail'],'full':m['0-60'],'tail':m['50-60'],'selection_role':'historical only; not reselected'})
    return rows


def derived(e,out):
    """Read original arrays through links; all new helper outputs go to a new directory."""
    rd=out/'closeout_review'/'derived'/e['id'];rd.mkdir(parents=True,exist_ok=True)
    for name in ['trace.npz','camera.npz','matrices.npz','lifecycle.json','observations.json','seeds.json','metrics.json','initialization_snapshots_reconstructed.npz']:
        source=Path(e['directory'])/name;dest=rd/name
        if source.exists() and not dest.exists():dest.symlink_to(source)
    return {**e,'directory':str(rd)}


def interval(e,start,end):
    z=load(Path(e['directory'])/'trace.npz');mask=(z['t']>=start)&(z['t']<=end);v=z['values'][mask];l=z['landmarks'][mask]
    return {'v_rmse':float(np.sqrt(np.mean(v[:,0]**2))),'eta_rmse':float(np.sqrt(np.mean(v[:,1]**2))),'landmark_rmse':float(np.sqrt(np.nanmean(l[:,:,0]**2))),'landmark_relative_p95':float(np.nanpercentile(l[:,:,1],95)),'positive_ray_fraction':float(np.sum(l[:,:,2]>0)/np.sum(np.isfinite(l[:,:,2]))),'near_center_fraction':float(np.sum(l[:,:,4]<.1)/np.sum(np.isfinite(l[:,:,4]))),'wide_VG_fraction':float(np.mean((v[:,0]<=.1)&(v[:,2]<=1)&(v[:,3]<=.2))),'strict_VG_fraction':float(np.mean((v[:,0]<=.05)&(v[:,2]<=.5)&(v[:,3]<=.1))),'samples':len(v)}


def component_comparison(a,b):
    result=compare_runs(a,b)
    A=load(Path(a['directory'])/'trace.npz');B=load(Path(b['directory'])/'trace.npz')
    assert np.array_equal(A['ids'],B['ids'])
    n=len(geometry(a['config']['scene'])[0]);d=3*n+6;dx=A['x'][:,:d]-B['x'][:,:d]
    for key,values in [('v',dx[:,-6:-3]),('eta',dx[:,-3:]),('landmark',dx[:,:-6].reshape(-1,n,3))]:
        axes=tuple(range(values.ndim-1))
        result[key]={'component_rmse':np.sqrt(np.mean(values**2,axis=axes)).tolist(),'component_signed_mean':np.mean(values,axis=axes).tolist(),'norm_rmse':float(np.sqrt(np.mean(np.sum(values**2,axis=-1)))),'norm_max':float(np.linalg.norm(values,axis=-1).max())}
    return result


def occupancy(e):
    from truth_model import ids_at
    c=e['config'];camera=load(Path(e['directory'])/'camera.npz');n=len(geometry(c['scene'])[0]);rows=[]
    for t,ids,side in zip(camera['t'],camera['ids'],camera['side']):
        if side!='post':continue
        active=set(map(int,ids[ids>=0]));observed=set(map(int,ids_at(round(t*20),20,c['lifetime'],n)))
        rows.append({'t':float(t),'slots':len(active),'observations':len(observed),'visible_slotted':len(active&observed),'unobserved_retained':len(active-observed),'waiting_observed':len(observed-active),'dimension':3*len(active)+6})
    tracks=json.loads((Path(e['directory'])/'lifecycle.json').read_text());observations=json.loads((Path(e['directory'])/'observations.json').read_text())
    delays=[r['slot_birth']-r['observation_birth'] for r in tracks]
    return {'rows':rows,'max_slots':max(r['slots'] for r in rows),'min_slots':min(r['slots'] for r in rows),'max_dimension':max(r['dimension'] for r in rows),'never_slotted':sum('slot_birth' not in r for r in observations),'observation_tracks':len(observations),'residence_tracks':len(tracks),'residence_right_censored':sum(r['right_censored'] for r in tracks),'observation_right_censored':sum(r['observation_right_censored'] for r in tracks),'slot_delay_median':float(np.median(delays)),'slot_delay_max':float(max(delays))}


def init_review(e):
    rd=Path(e['directory']);z=load(rd/'trace.npz');tracks=json.loads((rd/'joint_outcomes.json').read_text());lookup={r['id']:r for r in tracks};last={}
    for t,ids,lands,v in zip(z['t'],z['ids'],z['landmarks'],z['values']):
        for j,id in enumerate(ids):
            if id>=0 and t<=lookup[int(id)]['last_visible']+1e-9:last[int(id)]=(lands[j,1],v.copy())
    completed=[r for r in tracks if not r['observation_right_censored']];result={'seed':seed_summary(e),'completed_denominator':len(completed),'right_censored':len(tracks)-len(completed),'breakdown':{}}
    for grade,vm,ga,gm,lm in [('wide',.1,1,.2,.05),('strict',.05,.5,.1,.02)]:
        counts={'landmark_and_VG_pass':0,'landmark_only_fail':0,'VG_only_fail':0,'both_fail':0};separate={'V_pass':0,'G_pass':0,'landmark_pass':0,'VG_pass':0};evaluated=0
        for track in completed:
            if track['id'] not in last:continue
            error,v=last[track['id']];lp=error<=lm;vp=v[0]<=vm;gp=v[2]<=ga and v[3]<=gm;vg=vp and gp;evaluated+=1
            counts['landmark_and_VG_pass' if lp and vg else 'landmark_only_fail' if vg else 'VG_only_fail' if lp else 'both_fail']+=1
            for name,value in [('V_pass',vp),('G_pass',gp),('landmark_pass',lp),('VG_pass',vg)]:separate[name]+=int(value)
        held_land=sum(r[grade]['success_before_loss'] for r in completed);held_joint=sum(r['joint_'+grade]['success_by_last_visible'] for r in completed)
        result['breakdown'][grade]={'at_last_visible_partition':counts,'at_last_visible_separate':separate,'evaluated_denominator':evaluated,'missing_at_last_visible':len(completed)-evaluated,'held_landmark_success':held_land,'held_joint_success':held_joint,'held_denominator':len(completed),'note':'endpoint partition is distinct from requiring a prior continuous 0.10 s held interval'}
    hook=rd/'initialization_snapshots_reconstructed.npz'
    if hook.exists():
        h=load(hook);bytime={round(float(t),9):k for k,t in enumerate(h['t'])};events=json.loads((rd/'seeds.json').read_text());vectors=[];along=[];transverse=[];_,Rbc,_=geometry(e['config']['scene'])
        for event in events:
            k=bytime[round(event['t'],9)];slot=list(h['ids'][k]).index(event['id']);seed=h['x_after'][k,3*slot:3*slot+3];gt,_,_,bearing=sample(event['t'],e['config']['scene'],[event['id']]);delta=seed-gt[:3];ray=(bearing@Rbc.T)[0];a=float(delta@ray)
            vectors.append(delta);along.append(a);transverse.append(np.linalg.norm(delta-a*ray))
        vectors=np.array(vectors)
        result['seed_directions']={'source':'previously saved explicitly reconstructed hook x_after, evaluated against analytic truth; not a live seed-vector log','count':len(vectors),'body_component_rmse':np.sqrt(np.mean(vectors**2,axis=0)).tolist(),'body_component_mean':vectors.mean(axis=0).tolist(),'ray_signed_mean':float(np.mean(along)),'ray_rmse':float(np.sqrt(np.mean(np.array(along)**2))),'transverse_rmse':float(np.sqrt(np.mean(np.array(transverse)**2)))}
    else:result['seed_directions']={'status':'MISSING_NOT_RECONSTRUCTED_NOW'}
    return result


def continuity(e,dest):
    import csv
    from paper_review import merged
    rd=Path(e['directory']);t,v,_,sides=merged(load(rd/'trace.npz'),load(rd/'camera.npz'),30)
    rows=[];result={}
    for name,col,threshold in [('velocity',0,.05),('gravity_angle',2,.5),('gravity_magnitude',3,.1)]:
        bad=~(v[:,col]<=threshold);new=bad&(t>20)
        times,ix=np.unique(t,return_index=True);good=~np.logical_or.reduceat(bad,ix);badix=np.flatnonzero(~good);start=badix[-1]+1 if len(badix) else 0
        suffix=float(times[start]) if start<len(times) else None
        result[name]={'last_bad':float(t[bad][-1]) if bad.any() else None,'first_bad_after20':float(t[new][0]) if new.any() else None,'max_after20':float(np.max(v[t>20,col])),'terminal_good_start':suffix,'remaining_validation':None if suffix is None else 30-suffix,'bad_sample_sides_after20':int(new.sum())}
        for i in np.flatnonzero(new):rows.append({'metric':name,'time':t[i],'side':sides[i],'value':v[i,col]})
    with (dest/'evidence/E2_new_violations.csv').open('w') as f:
        writer=csv.DictWriter(f,fieldnames=['metric','time','side','value'],lineterminator='\n');writer.writeheader();writer.writerows(rows)
    write(dest/'evidence/E2_continuity.json',result)
    return result


def report(out,dest):
    from closeout_review import MATRIX,prefix_check
    from closeout_plots import plots
    completed={};missing=[]
    for label,c in MATRIX.items():
        matches=[e for e in ledger(out) if e['config']==c and e['status']=='COMPLETED']
        if len(matches)==1:completed[label]=matches[0]
        else:missing.append(label)
    if 'E2' in completed:
        prefix_check(out,completed['E2']);continuity(completed['E2'],dest)
    comparisons={};settles={};windows={};outcomes={};occupation={};transients={};corrections={}
    for scene in ['REGULAR','FAST','PAPER']:
        a=find(out,impl='CORE',scene=scene);b=find(out,impl='CONT_REF',scene=scene)
        split=completed.get('E1') if scene=='PAPER' else find(out,impl='SPLIT_REF',scene=scene)
        for name,x,y in [('CORE-CONT',a,b),('CORE-SPLIT',a,split),('SPLIT-CONT',split,b)]:
            if x and y:comparisons[scene+'_'+name]=component_comparison(x,y)
        for e in [a,b,split]:
            if e:
                tr,cam=load(Path(e['directory'])/'trace.npz'),load(Path(e['directory'])/'camera.npz')
                settles[e['id']]=settling(tr,cam,e['config']['duration'],len(geometry(scene)[0]))
    if 'E2' in completed:
        e=completed['E2'];tr,cam=load(Path(e['directory'])/'trace.npz'),load(Path(e['directory'])/'camera.npz');settles[e['id']]=settling(tr,cam,30,16)
        for grade in settles[e['id']].values():
            for key,value in grade.items():
                if isinstance(value,str):grade[key]='NOT_OBSERVED_WITHIN_30S'
        windows[e['id']]={f'{lo}-{hi}':interval(e,lo,hi) for lo,hi in [(0,20),(5,20),(15,20),(20,30),(0,30)]}
    compare_entries=[]
    for scene,label in [('REGULAR','E3'),('FAST','E4')]:
        baseline=find(out,impl='EXPERIMENTAL',scene=scene,lifetime=1)
        es=[baseline,find(out,impl='EXPERIMENTAL',scene=scene,candidate='C6',lifetime=1)]
        if label in completed:es.append(completed[label])
        for e in es:
            compare_entries.append(e);windows[e['id']]={f'{lo}-{hi}':interval(e,lo,hi) for lo,hi in [(0,60),(50,60)]}
            if e!=baseline:transients[e['id']]=repeated_transients(baseline,e)
    paper_base=find(out,impl='CORE',scene='PAPER');paper_es=[paper_base]+[completed[k] for k in ['E5','E6'] if k in completed]
    for e in paper_es:
        compare_entries.append(e);windows[e['id']]={f'{lo}-{hi}':interval(e,lo,hi) for lo,hi in [(0,20),(5,20),(15,20)]};occupation[e['id']]=occupancy(e)
    for e in compare_entries:
        view=derived(e,out);outcomes[e['id']]=track_outcomes(view);corrections[e['id']]=camera_correction_deltas(view)
    inits={};init_outcomes={};init_pairs={}
    for scene in ['REGULAR','FAST']:
        for mode in ['ZERO','GT_SEED','GEOM_IDEAL','GT_MATCHED','GEOM_NOISY']:
            e=find(out,impl='EXPERIMENTAL',scene=scene,lifetime=1,initialization=mode)
            inits[e['id']]=init_review(e);init_outcomes[e['id']]=json.loads((Path(e['directory'])/'config.json').read_text())
        a=find(out,impl='EXPERIMENTAL',scene=scene,lifetime=1,initialization='GEOM_IDEAL');b=find(out,impl='EXPERIMENTAL',scene=scene,lifetime=1,initialization='GT_MATCHED')
        def events(e):return [(r['id'],r['t']) for r in json.loads((Path(e['directory'])/'seeds.json').read_text())]
        assert events(a)==events(b)
        A=load(Path(a['directory'])/'trace.npz');B=load(Path(b['directory'])/'trace.npz');assert np.array_equal(A['ids'],B['ids'])
        difference=float(np.nanmax(np.abs(A['x']-B['x'])))
        init_pairs[scene]={'events_identical':True,'max_absolute_state_difference':difference,'interpretation':'ideal triangulation residual; not exact floating point equality'}
    shared=[]
    for group in [[find(out,impl='EXPERIMENTAL',scene=s,candidate=c,lifetime=1) for c in ['C0','C2','C6'] if c!='C2' or ('E3' if s=='REGULAR' else 'E4') in completed] for s in ['REGULAR','FAST']]+[paper_es+([completed['E1']] if 'E1' in completed else [])]:
        hashes=[metric(e)['physical_bearing_sha'] for e in group];inputs=[metric(e)['input_sha'] for e in group]
        assert len(set(hashes))==len(set(inputs))==1
        shared.append({'runs':[e['id'] for e in group],'physical_bearing_sha':hashes[0],'input_sha':inputs[0],'pass':True})
    numerics={e['id']:{k:metric(e)[k] for k in ['P_min','P_max','P_condition_max','symmetry_max','substeps_max','protection_count','protection_norm','seconds']} for e in compare_entries+list(completed.values())}
    status='COMPLETED_WITH_RESULTS' if not missing else 'PARTIAL_BUDGET_LIMIT' if len(ledger(out))>=64 else 'BLOCKED_REFERENCE'
    for name,value in [('comparisons',comparisons),('settling',settles),('intervals',windows),('outcomes',outcomes),('occupancy',occupation),('transients',transients),('camera_corrections',corrections),('initialization',inits),('initialization_pairs',init_pairs),('initialization_identities',init_outcomes),('shared_inputs',shared),('numerics',numerics),('fixed_gain_tradeoff',gain_fixed(out))]:write(dest/'evidence'/f'{name}.json',value)
    write(dest/'evidence/summary.json',{'status':status,'total_attempts':len(ledger(out)),'new_attempts':len(ledger(out))-58,'completed':{k:e['id'] for k,e in completed.items()},'missing':missing,'historical_C_star':'C6','selection_repeated':False})
    write(dest/'evidence/ledger.json',ledger(out))
    figs=plots(out,dest,completed,compare_entries)
    render_report(out,dest,completed,status,missing,comparisons,settles,windows,outcomes,numerics,inits,figs)
    print(status, 'total attempts',len(ledger(out)),'missing',missing,flush=True)


def render_report(out,dest,completed,status,missing,comparisons,settles,windows,outcomes,numerics,inits,figs):
    def f(x):return f'{x:.6g}' if isinstance(x,(float,int)) else str(x)
    lines=['# LTV v2 六项补充验证','',f'状态：**{status}**；总账 {len(ledger(out))}/64。本补充不属于历史预注册实验，C*=C6 不变。执行依据：[04 六项补充方案](../../../04-LTV_v2_六项补充验证_Codex执行方案.md)。','',
           '所有原始58次结果和旧报告保持只读。990个历史文件及生产白名单SHA于运行前后核对；完整账本只追加59–64，不改原58条。新数据在外部根目录各运行目录，新的离线衍生文件只写 `closeout_review/derived/`。','',
           '## 实验身份与验收','', '|项目|运行|参数|寿命 s|时长 s|退出码|','|---|---|---|---:|---:|---:|']
    for label,e in completed.items():
        c=e['config'];lines.append(f"|{label}|{e['id']}|{c['candidate']}|{c['lifetime']}|{c['duration']}|{e['exit_code']}|")
    lines+=['','缺失项目：'+(', '.join(missing) or '无')+'。原数值源码、库、输入和评价合同复用；没有新增算法修改。短测与完整仿真串行，最多一个模拟进程，BLAS单线程。PAPER动态槽位hook关闭等价短测通过，完整交叉块与旧RHS/精确子流数学短测通过。','',
            '## 连续、分步与CORE','', '|场景／差异|v差 RMSE m/s|η差 RMSE m/s²|路标差 RMSE m|整秒完整P相对差最大值|','|---|---:|---:|---:|---:|']
    for key,c in comparisons.items():lines.append(f"|{key}|{f(c['v']['norm_rmse'])}|{f(c['eta']['norm_rmse'])}|{f(c['landmark']['norm_rmse'])}|{f(c['P_max_relative_at_seconds'])}|")
    lines+=['','各分量带符号均值、分量RMSE、范数峰值和归一化状态差另见 `evidence/comparisons.json`，不以混合单位状态范数替代分项指标。REGULAR/FAST 为共同0–60 s，PAPER为共同0–20 s。','',
            '|身份|宽t_VG|严t_VG|宽t_L|严t_L|','|---|---:|---:|---:|---:|']
    for key,s in settles.items():lines.append(f"|{key}|{f(s['wide']['t_VG'])}|{f(s['strict']['t_VG'])}|{f(s['wide']['t_L'])}|{f(s['strict']['t_L'])}|")
    lines+=['','所有固定ID时间合并200 Hz和相机前/后侧，要求持续至结束且至少验证5 s。E2的0–20 s输入、状态、slot、离散事件与完整可比P快照逐值一致，见 `E2_prefix.json`。未比较运行耗时和未使用的随机位姿先验。20–30 s仅为CORE持续性检查，不假设拥有该段CONT_REF。','',
            '## 固定点增益历史与新权衡','', '|候选／实现|50–60 s J_tail|v RMSE|η RMSE|路标 RMSE|','|---|---:|---:|---:|---:|']
    for r in gain_fixed(out):lines.append(f"|{r['candidate']} {r['impl']}|{f(r['J_tail'])}|{f(r['tail']['v_rmse'])}|{f(r['tail']['eta_rmse'])}|{f(r['tail']['landmark_rmse'])}|")
    lines+=['','C2/C4原连续η误差的小幅增加仍构成原规则下的排除依据；数值可分辨不等于工程上必然重要。E3/E4为EXPLORATORY_GAIN_TRADEOFF，不再选参，无新增C2噪声或PAPER固定点验证。','',
            '## 生命周期与共同区间','', '|运行／区间|v RMSE|η RMSE|路标RMSE|正ray比例|近中心比例|宽／严VG目标内比例|','|---|---:|---:|---:|---:|---:|---|']
    for key,bywindow in windows.items():
        for window,m in bywindow.items():lines.append(f"|{key} {window}s|{f(m['v_rmse'])}|{f(m['eta_rmse'])}|{f(m['landmark_rmse'])}|{f(m['positive_ray_fraction'])}|{f(m['near_center_fraction'])}|{f(m['wide_VG_fraction'])} / {f(m['strict_VG_fraction'])}|")
    lines+=['','PAPER的5–20 s不是最后5秒稳态；另列15–20 s，E2另列20–30 s。所有点的误差统计包括失踪保留槽，但消失前成功仅截止最后可见时刻；无valid过滤。正ray分母为存在的槽样本；near-center固定为距离<0.1 m。','',
            '|运行|结束观测／右删失数|宽路标／联合成功|严路标／联合成功|','|---|---|---|---|']
    for key,m in outcomes.items():
        lines.append(f"|{key}|{m['completed_observation_tracks']} / {m['observation_right_censored']}|{f(m['wide']['landmark_success_completed'])} / {f(m['wide']['joint_success_completed'])}|{f(m['strict']['landmark_success_completed'])} / {f(m['strict']['joint_success_completed'])}|")
    lines+=['','成功要求曾连续保持0.10 s且最后可见时仍达标，右删失不作为必然失败。固定PAPER点全部观测右删失，以上已结束分母为0，显示None而非0%失败；固定点另用5 s持续验收。更保守的末端连续保持成功、逐点观测／驻留年龄和确认年龄见新衍生 `joint_outcomes.json`。','',
            'PAPER原max_features=30、min_features=15、max_missed_frames=2、warmup=20保持原值。实际槽数、观测数、延迟入槽、维度、未入槽数见 `evidence/occupancy.json`。物理ID仅供配对与真值评价，有符号bearing不裁剪、不翻轴。','',
            '## 峰值、校正和数值成本','', '|运行|秒数|最大子步|P条件数最大值|实际floor次数（仅EXPERIMENTAL）|','|---|---:|---:|---:|---:|']
    for key,m in numerics.items():lines.append(f"|{key}|{f(m['seconds'])}|{f(m['substeps_max'])}|{f(m['P_condition_max'])}|{f(m['protection_count']) if 'EXPERIMENTAL' in key else '未插桩/不适用'}|")
    lines+=['','`evidence/transients.json`保留全部启动峰值、10 s后峰值、超过同场景C0峰值的持续时间和每次出生后0.2 s配对窗口；窗口可能重叠，不累计为独立时间。`camera_corrections.json`及外部新衍生数组保存去掉生命周期重排后的实际净相机校正。原始gain日志仍保存分块K r、交叉块尺度与逐路标贡献。成本为本机观测值，不是受控性能基准。','',
            '## 既有初始化日志复核','', '|运行|结束观测分母|宽：路标成功数／联合成功数|严：路标成功数／联合成功数|','|---|---:|---|---|']
    for key,m in inits.items():lines.append(f"|{key}|{m['completed_denominator']}|{m['breakdown']['wide']['held_landmark_success']} / {m['breakdown']['wide']['held_joint_success']}|{m['breakdown']['strict']['held_landmark_success']} / {m['breakdown']['strict']['held_joint_success']}|")
    lines+=['','`evidence/initialization.json`分别给出最后可见时刻的路标失败、VG失败、两者失败、两者通过分区，另列V/G各自通过数；瞬时分区不替代连续0.10 s成功率。接受率、观测与驻留等待、seed误差另列，GEOM_IDEAL与GT_MATCHED触发序列逐项一致。','',
            '种子方向误差来自历史明确标记的重建hook快照与解析真值，报告机体系分量、沿ray和横向误差；这不是新现场日志。ZERO无种子方向记录，明确标为缺失。P不随均值种子改动属于控制变量实验，不代表统计一致在线初始化。本轮不增加Gate或重调P。','',
            '## 四项结论与停止','', '见 [conclusions.md](conclusions.md)。本阶段只提出一项最小后续建议，不继续执行。','',
            '## 曲线与复现','']
    for name in figs:lines.append(f'- [{name} PNG](figures/{name}.png) / [SVG](figures/{name}.svg)')
    lines+=['','```bash','python3 docs/experiments/tools/ltv_standalone_study/closeout_review.py all','```','',
            '可单独使用audit/run/report；run复用同身份成功结果，预算不重置。外部总账包含命令、退出码、源码身份、输入路径及SHA。历史两项执行偏差保留于原决策日志，本次未重复其行为。']
    (dest/'report.md').write_text('\n'.join(lines)+'\n')
