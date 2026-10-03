"""Generate reviewable compact evidence and standalone figures from completed identities."""
import json,shutil
from pathlib import Path
import numpy as np
from diagnostics import geometry_diagnostics,compare_runs,overshoot,seed_summary,inspect_saved_numerics,dynamic_geometry,repeated_transients,track_outcomes,combined_settling,noise_response,verified_events,initialization_snapshots,camera_correction_deltas,reported_settling,peak_diagnostics,initialization_covariance_parity
from plot_results import make_plots

def report(out,doc):
    from run_study import ledger,metric,config,CANDIDATES,write,frozen_check
    all_entries=ledger(out); entries=[e for e in all_entries if e['status']=='COMPLETED']; evidence=doc/'evidence'; evidence.mkdir(parents=True,exist_ok=True)
    metrics={e['id']:metric(e) for e in entries}
    selection=json.loads((out/'selection.json').read_text()) if (out/'selection.json').exists() else {'C_star':None,'status':'NOT_EXECUTED'}
    star=selection['C_star']
    expanded={}
    for candidate in CANDIDATES:
        c=config('EXPERIMENTAL',candidate=candidate)
        expanded[candidate]={'q_landmark':c['q'],'v_landmark':c['v'],'v_velocity':c['v'],'v_gravity':c['v'],'initial_p_landmark':c['p'],'initial_p_velocity':c['p'],'initial_p_gravity':c['p'],'max_features':30,'max_missed_frames':2,'camera_euler_safety':.5,'max_camera_substeps':100,'covariance_floor':1e-9}
    write(evidence/'standalone_configs.json',{'purpose':'diagnostic study only; not a production configuration update','candidates':expanded,'C_star':star,'selected':expanded.get(star)})
    def find(**kw): return [e for e in entries if all(e['config'].get(k)==v for k,v in kw.items())]
    def unique(**kw):
        es=find(**kw)
        return es[-1] if es else None
    def baseline(impl,scene='REGULAR',candidate='C0',**kw):
        expected=config(impl,scene,candidate,**kw)
        es=[e for e in entries if e['config']==expected]; return es[-1] if es else None
    expected=[]
    for scene in ['REGULAR','FAST']:
        expected += [config('CONT_REF',scene),config('CONT_REF',scene,tight=True),config('SPLIT_REF',scene),config('CORE',scene)]
    expected += [config('CONT_REF','PAPER'),config('CONT_REF','PAPER',tight=True),config('CORE','PAPER'),config('CONT_REF',duration=10,initialization='EXACT'),config('EXPERIMENTAL',duration=10,initialization='EXACT'),config('CONT_REF','STATIC'),config('EXPERIMENTAL')]
    expected += [config(imu_hz=i,camera_hz=c) for i,c in [(1000,20),(200,100),(1000,100)]]+[config('SPLIT_REF',camera_hz=100)]
    expected += [config(impl,candidate=candidate) for candidate in list(CANDIDATES)[1:] for impl in ['CONT_REF','EXPERIMENTAL']]
    if star and star!='C0': expected += [config('CONT_REF','FAST',star),config('EXPERIMENTAL','FAST',star),config('CONT_REF',candidate=star,tight=True)]
    if star: expected += [config('EXPERIMENTAL',candidate=c,noise=s) for c in dict.fromkeys(['C0',star]) for s in [42,43]]
    for scene in ['REGULAR','FAST']:
        expected += [config('EXPERIMENTAL',scene,lifetime=life) for life in [.5,1,2,5]]
        if star and star!='C0': expected.append(config('EXPERIMENTAL',scene,star,lifetime=1))
        expected += [config('EXPERIMENTAL',scene,lifetime=1,initialization=mode) for mode in ['GT_SEED','GEOM_IDEAL','GT_MATCHED','GEOM_NOISY']]
    expected += [config('SPLIT_REF',lifetime=1),config('CONT_REF',duration=120)]
    accepted_failures=[e['config'] for e in all_entries if e['status']=='FAILED' and e['config']['impl']=='EXPERIMENTAL' and 'excluded' in selection.get('decisions',{}).get(e['config']['candidate'],{})]
    missing=[c for c in expected if not any(e['config']==c for e in entries) and c not in accepted_failures]
    references=json.loads((out/'reference_checks.json').read_text()) if (out/'reference_checks.json').exists() else []
    parity=json.loads((out/'full_parity.json').read_text()) if (out/'full_parity.json').exists() else {}
    complete=not missing and references and all(r['pass'] for r in references) and parity and all(parity.values()) and star is not None
    status='COMPLETED' if complete else ('BUDGET_EXHAUSTED' if len(all_entries)>=64 else 'BLOCKED_REFERENCE')
    summary={'protocol_deviations':['Original broad SHA freeze read ov_data reference-file bytes only; excluded from subsequent rechecks, never used as scientific inputs','Two 0.1 s scheduler probes briefly overlapped one full reference: three short-lived calculation processes; full simulations always at most two'],'status':status,'runs_used':len(all_entries),'runs_remaining':64-len(all_entries),'completed_runs':len(entries),'missing':missing,'selection':selection,'reference_checks':references,'full_parity':parity,'metrics':metrics,'integrity':frozen_check()}
    write(evidence/'summary.json',summary); write(evidence/'ledger.json',all_entries)
    for name in ['verification.json','selection.json','reference_checks.json','full_parity.json','core_build.json','experimental_build.json','environment.json','additional_tests.json','selection_contract.json','selection_source.py.txt','extension_prefix.json','noise_input_audit.json']:
        if (out/name).exists(): shutil.copyfile(out/name,evidence/name)
    geometry={}
    for scene in ['REGULAR','FAST','PAPER','STATIC']:
        if find(scene=scene): geometry[scene]=geometry_diagnostics(scene,20 if scene=='PAPER' else 60,out)
    write(evidence/'geometry.json',geometry)
    dynamic_geo={e['id']:dynamic_geometry(e,out) for e in entries if e['config']['lifetime']>0 and e['config']['initialization']=='ZERO'}
    write(evidence/'dynamic_geometry.json',dynamic_geo)
    repeated={}
    if star and star!='C0':
        for scene in ['REGULAR','FAST']:
            a,b=baseline('EXPERIMENTAL',scene,lifetime=1),baseline('EXPERIMENTAL',scene,star,lifetime=1)
            if a and b: repeated[scene]=repeated_transients(a,b)
    write(evidence/'repeated_transients.json',repeated)
    event_checks={e['id']:verified_events(e) for e in entries if e['config']['impl']!='CONT_REF'}
    write(evidence/'event_checks.json',event_checks)
    outcomes={e['id']:track_outcomes(e) for e in entries if e['config']['lifetime']>0}
    write(evidence/'track_outcomes.json',outcomes)
    combined={e['id']:reported_settling(e) for e in entries if e['config']['lifetime']==0}
    write(evidence/'combined_settling.json',combined)
    numerics={e['id']:inspect_saved_numerics(e) for e in entries}; write(evidence/'energy.json',numerics)
    def fmt(v):
        if v is None: return '—'
        if isinstance(v,str): return '未在时域内观测到' if v=='NOT_OBSERVED_WITHIN_HORIZON' else v
        if isinstance(v,bool): return str(v)
        return f'{v:.6g}'
    lines=['# LTV 第一层独立验证 v2', '',f'状态：**{status}**。完整仿真尝试 {len(all_entries)}/64；成功 {len(entries)}。大数组、完整矩阵、每次命令与日志：`{out}`。','',
           '基线为 `43215b84e90c18d79d55528c811d85ab22fc04a8`。本轮只使用解析合成输入；没有启动 ROS、OpenVINS、VINS-Fusion，没有使用真实序列进行仿真或评价 ATE。首次 SHA 冻结曾意外读取仓库 `ov_data` 参考轨迹的字节用于哈希，没有解析或用于科学评价；后续已排除，详见 [decision_log.md](decision_log.md)。冻结清单含论文、执行文档、配置、生产源码、交接副本及历史证据；末尾复核生产、配置、论文、执行文档、交接副本及历史证据未变；数据目录不再读取复核。','',
           '模型依据为冻结的[本地 2025 ACC 论文](../../Pose_Velocity_and_Landmark_Position_Estimation_Using_IMU_and_Bearing_Measurements.pdf)式 (10)/(14)、(15)–(17) 与 (19)。CONT_REF 使用原始、未缩放的完整 Riccati 方程与 DOP853。SPLIT_REF 的冻结 IMU 子流使用解析矩阵指数/多项式噪声积分，冻结观测子流使用精确信息更新的 Woodbury 等价式。CORE 直接编译生产源码；EXPERIMENTAL 是同一核心的均值 hook/数值计数副本，未修订数值算法。参数、寿命及初始化阶段以已通过完整时域 parity 的 EXPERIMENTAL 为离散目标。首次相机事件校正长度为零。共同 x(0)/P(0) 定义在首帧建槽及规定初始化之后；准确全状态初值的 t=0 建槽前六维记录只作事件审计，不冒充初始化后的估计值。','',
           '所有误差保留启动瞬态，共同物理采样率 200 Hz；相机前后另存。宽档为速度 0.10 m/s、重力方向 1°、模长 0.20 m/s²、路标相对三维误差 5%；严档为 0.05、0.5°、0.10、2%。固定点表格与选参的 settling 合并共同 200 Hz 检查及相机前/后全部样本，要求持续至结束且余下至少 5 s；未达标仅表示当前时域内未观测到。','',
           '## 1. 固定点质量','',
           '|运行/场景/实现|全程 V RMSE|全程 η RMSE|全程路标 RMSE|末段 V RMSE|末段 η RMSE|末段路标 RMSE|宽/严 t_VG|',
           '|---|---:|---:|---:|---:|---:|---:|---|']
    for e in entries:
        c=e['config']; m=metrics[e['id']]
        if c['lifetime'] or c['noise'] or c['candidate']!='C0' or c['imu_hz']!=200 or c['camera_hz']!=20: continue
        duration=c['duration']; full=m.get(f'0-{duration:g}',m.get('0-60')); tail=m.get(f'{duration-10:g}-{duration:g}',m.get('50-60',m.get('5-20',full)))
        lines.append(f"|{e['id']} {c['duration']}s {c['initialization']} {'tight' if c['tight'] else ''}|{fmt(full['v_rmse'])}|{fmt(full['eta_rmse'])}|{fmt(full['landmark_rmse'])}|{fmt(tail['v_rmse'])}|{fmt(tail['eta_rmse'])}|{fmt(tail['landmark_rmse'])}|{fmt(combined[e['id']]['wide']['t_VG_all_samples'])} / {fmt(combined[e['id']]['strict']['t_VG_all_samples'])}|")
    lines+=['','参考加严验收（逐个 200 Hz 时刻、完整状态和完整 P）：','', '|普通 / 加严|最大归一化状态差|最大完整 P 相对差|≤1e−6|','|---|---:|---:|---|']
    for r in references: lines.append(f"|{r['base']} / {r['tight']}|{fmt(r['state_max_normalized'])}|{fmt(r['P_max_relative'])}|{r['pass']}|")
    lines+=['','末段列：60 s 运行取 50–60 s，PAPER 取规定的 5–20 s，准确初值短测取 0–10 s。120 s 延长行的全程列保留原 0–60 s，末段列为 110–120 s，完整 120 s 曲线另存。每次运行的全部规定区间、三轴误差、P95/max、速度范数 RMS 比、角度有效分母、模长误差、逐点严格/宽档 settling 及相机前后统计见 `evidence/summary.json` 和外部 `trace.npz` / `camera.npz`。角度在 η≈0 时为无定义，不计为零。','',
            '## 2. 离散来源','', '|场景 / A−B|速度差 RMSE|η 差 RMSE|最大归一化 x 差|整秒 P 最大相对差|','|---|---:|---:|---:|---:|']
    comparisons={}
    for scene in ['REGULAR','FAST']:
        for ia,ib in [('CORE','SPLIT_REF'),('SPLIT_REF','CONT_REF')]:
            a,b=baseline(ia,scene),baseline(ib,scene)
            if a and b:
                r=compare_runs(a,b); comparisons[f'{scene}_{ia}-{ib}']=r
                lines.append(f"|{scene} {ia}−{ib}|{fmt(r['velocity_difference_rmse'])}|{fmt(r['gravity_difference_rmse'])}|{fmt(r['x_max_normalized'])}|{fmt(r['P_max_relative_at_seconds'])}|")
    lines+=['','|实现 / IMU / camera Hz|末段 V RMSE|末段 η RMSE|最大相机子步|','|---|---:|---:|---:|']
    for e in find(scene='REGULAR',candidate='C0',lifetime=0,noise=0,initialization='ZERO',tight=False):
        c=e['config']; m=metrics[e['id']]
        if c['impl'] not in ['CORE','SPLIT_REF']: continue
        lines.append(f"|{c['impl']} {c['imu_hz']} / {c['camera_hz']}|{fmt(m['50-60']['v_rmse'])}|{fmt(m['50-60']['eta_rmse'])}|{fmt(m['substeps_max'])}|")
    write(evidence/'discretization.json',comparisons)
    lines+=['','CORE−SPLIT_REF 隔离子流积分、谱保护和实现；SPLIT_REF−CONT_REF 包含采样、bearing 冻结与分步组织。相机校正时间 τ 不推进物理时间。floor 计数只统计特征值被抬至下限的实际动作，不是谱检查/重构的总调用次数。生产核心未暴露谱保护计数；只有 EXPERIMENTAL 的计数是真实插桩值，不能把 CORE 日志的占位零解释为未触发保护。','',
            '## 3. 增益对照','',f"冻结诊断选择：**{star or '未执行'}**（{selection['status']}）。排序只用宽档 t_VG，未达标或分辨不了时用 50–60 s 的 J_tail。严档及其他区间不参与事后改换目标。允许诊断候选增加瞬态峰值，不宣称全面改善。",'',
            '|候选 / 实现|Q / V / P0|宽 t_VG|J_tail|200 Hz 速度峰值|全程 V RMSE|耗时 s|最大相机子步|floor 次数 / 累计谱抬升|','|---|---|---|---:|---:|---:|---:|---:|---|']
    for candidate in CANDIDATES:
        for impl in ['CONT_REF','EXPERIMENTAL']:
            e=baseline(impl,candidate=candidate)
            if not e: continue
            c=e['config']; m=metrics[e['id']]
            lines.append(f"|{candidate} {impl}|{c['q']:g} / {c['v']:g} / {c['p']:g}|{fmt(combined[e['id']]['wide']['t_VG_all_samples'])}|{fmt(m['J_tail'])}|{fmt(m['0-60']['v_max'])}|{fmt(m['0-60']['v_rmse'])}|{fmt(m['seconds'])}|{fmt(m['substeps_max']) if impl!='CONT_REF' else '—'}|{fmt(m['protection_count'])} / {fmt(m['protection_norm'])}|")
    overshoots={}
    for candidate in list(CANDIDATES)[1:]:
        a,b=baseline('EXPERIMENTAL'),baseline('EXPERIMENTAL',candidate=candidate)
        if a and b: overshoots[candidate]=overshoot(a,b)
    write(evidence/'overshoot.json',overshoots)
    write(evidence/'peaks_all_samples.json',{e['id']:peak_diagnostics(e) for e in entries})
    noise_effect={}
    for e in entries:
        if e['config']['noise']:
            clean=baseline('EXPERIMENTAL',candidate=e['config']['candidate'])
            if clean: noise_effect[e['id']]=noise_response(clean,e)
    write(evidence/'noise_response.json',noise_effect)
    if star in overshoots:
        lines+=['',f"C* 相对 C0 的瞬态：`{json.dumps(overshoots[star],ensure_ascii=False)}`。峰值使用共同 200 Hz 与相机前/后两侧的最大值；超过该 C0 峰值的持续时间按共同 200 Hz 完整时间轴梯形积分（0.005 s 时间分辨率），全部种类峰值另见 `peaks_all_samples.json`；10 s 后的值另列。"]
    lines+=['','|噪声候选 / seed|J_tail|末段 V RMSE|末段 η RMSE|宽档目标内时间比例|','|---|---:|---:|---:|---:|']
    for e in entries:
        c=e['config']; m=metrics[e['id']]
        if c['noise']: lines.append(f"|{c['candidate']} / {c['noise']}|{fmt(m['J_tail'])}|{fmt(m['50-60']['v_rmse'])}|{fmt(m['50-60']['eta_rmse'])}|{fmt(m['wide']['target_fraction_VG'])}|")
    lines+=['','噪声只做两种子的敏感性检查，不是充分统计验证。`evidence/noise_response.json` 另报同候选带噪输出相对无噪输出的扰动 RMSE，以免收敛偏差改善掩盖噪声放大。IMU 与 bearing 噪声均为冻结的每样本噪声，C0/C* 同种子完全共用；不得将不同采样率下的每样本噪声混为同一连续谱密度。`gain_diagnostics.npy` 保存 K 范数、三类实际 K r、P_vL/P_etaL 及各块尺度；`gain_landmark_contributions.npy` 保存逐点 K_i r。离散诊断取相机事件前的已驻留状态与当前可见观测，连续诊断取整秒。`camera_correction_deltas.npz` 从实际相机前后状态中扣除 slot 重排与一次均值写入，给出真实净校正量，包含全部相机子步的最终作用；不把它冒充每个子步内部的瞬时 K r。','',
            '## 4. 寿命对照','', '|场景 / 实现 / 候选 / 寿命 s|physical bearing SHA 前12位|驻留中位数 s|完整观测轨迹最后可见前路标宽档达标率|末段 V RMSE|末段 η RMSE|','|---|---|---:|---:|---:|---:|']
    for e in entries:
        c=e['config']; m=metrics[e['id']]
        if c['lifetime']<=0 or c['initialization']!='ZERO': continue
        l=m['lifecycle']; lines.append(f"|{c['scene']} {c['impl']} {c['candidate']} {c['lifetime']}|{m['physical_bearing_sha'][:12]}|{fmt(l['residence_median'])}|{fmt(outcomes[e['id']]['wide']['landmark_success_completed'])}|{fmt(m['50-60']['v_rmse'])}|{fmt(m['50-60']['eta_rmse'])}|")
    lines+=['','最后可见与删除分别记在 `observations.json` / `lifecycle.json`，成功截止于最后可见，要求先连续保持 0.10 s 并在最后可见采样仍达标。首次保持段的起点年龄与确认年龄（至少再等待 0.10 s）分别保存，不把起点当作在线确认时刻。末端仍存活点与观测仍延续点分别标记右删失。表中成功率的分母只含观测已结束的入槽轨迹；所有轨迹截至时域末的达标比例、右删失分母、v/g/路标联合达标率，以及更保守的“最后可见前连续 0.10 s 始终达标”比例另见 `evidence/track_outcomes.json`，没有将右删失当作必然失败。未入槽观测、观测出生、驻留出生均有记录；实际首次非零校正在 `events_verified.json`，其中剔除了 t=0 后未再可见、实际没有被校正的启动轨迹。原始 `observations.json` 的该字段在这些点上是下一相机时刻的计划值，不能当成发生过的校正。驻留中位数包含末端截断样本，右删失数量另列，不能当作无偏寿命估计。稳定 ID 只是理想身份连续参照。','',
            '## 5. 一次初始化','', '|场景 / 模式|接受数 / 驻留数|观测延迟中位数 s|种子误差 RMSE m|完整观测轨迹最后可见前路标宽档达标率|末段 V RMSE|末段 η RMSE|末段路标 RMSE|','|---|---|---:|---:|---:|---:|---:|---:|']
    seeds={}
    for e in entries:
        c=e['config']; m=metrics[e['id']]
        if c['lifetime']!=1 or c['candidate']!='C0' or c['impl']!='EXPERIMENTAL': continue
        s=seed_summary(e); seeds[e['id']]=s
        lines.append(f"|{c['scene']} {c['initialization']}|{str(s.get('accepted','—'))} / {str(s.get('slotted',m['lifecycle']['slotted']))}|{fmt(s.get('delay_observed_median'))}|{fmt(s.get('seed_error_rmse'))}|{fmt(outcomes[e['id']]['wide']['landmark_success_completed'])}|{fmt(m['50-60']['v_rmse'])}|{fmt(m['50-60']['eta_rmse'])}|{fmt(m['50-60']['landmark_rmse'])}|")
    write(evidence/'seeds.json',seeds)
    covariance_parity=[initialization_covariance_parity(baseline('EXPERIMENTAL',e['config']['scene'],lifetime=1),e) for e in entries if e['config']['initialization'] in ['GT_SEED','GEOM_IDEAL','GT_MATCHED','GEOM_NOISY']]
    write(evidence/'initialization_covariance_parity.json',covariance_parity)
    hook_snapshots={e['id']:initialization_snapshots(e) for e in entries if e['config']['initialization'] in ['GT_SEED','GEOM_IDEAL','GT_MATCHED','GEOM_NOISY']}
    write(evidence/'initialization_snapshots.json',hook_snapshots)
    write(evidence/'camera_corrections.json',{e['id']:camera_correction_deltas(e) for e in entries if e['config']['impl']!='CONT_REF'})
    lines+=['','GT_SEED 是入槽真值诊断；GEOM_IDEAL 使用同 ID 过去/当前 bearing 与精确模拟相机位姿；GT_MATCHED 复用完全相同触发规则、ID 和时刻；GEOM_NOISY 使用 seed42 的位置/姿态噪声，三角化和执行时的机体系变换均使用带噪先验。至少三帧且跨度 0.10 s，条件数≤1e8、最大视线夹角≥0.5°、残差≤0.5°、正射线，最多 1 s 历史。每点最多写一次，只改该点均值，P 和 v/η 均值规则保持原样。初始化瞬间的完整 x/P 由已保存的相机事件前状态、实际 slot 重排及冻结种子输入精确重建，另存 `initialization_snapshots_reconstructed.npz`；该文件明确标记重建来源，不冒充额外现场采样，P 为均值写入前后共用矩阵。全部保存的协方差快照与同场景零种子运行逐值相同，见 `evidence/initialization_covariance_parity.json`。这是额外位姿先验诊断，不是已完成在线融合。','',
            '## 6. 几何与完整证据','',
            '有限窗口指标不乘 Q，按物理点/连续 ID 的世界 bearing 计算；固定 ID 的 1/5/10 s 完整窗口见 `evidence/geometry.json`，窗口不完整为缺失。这些有限窗口不能证明全时间 uniform PE。S_PAPER 使用文档假设的 4×4 地面网格、P0=I、R(t)=Exp(t[−1,3,0]×) R_y(−2t) 与有符号理想 bearing；相机 Z 负比例独立记录，未翻轴，未裁剪后方点，也未按 Fig.3 读图拟合。','',
            '所有运行的 `trace.npz` 保留三维路标误差、相对误差、ray distance、投影横向误差、距相机中心距离、沿射线误差；近相机中心定义为 <0.1 m。矩阵文件保留整秒与生命周期事件前后完整 P；连续加严比较额外保留每个 200 Hz 时刻的完整 P。`energy_cross_blocks.npy` 用线性求解计算 U，不显式求逆。','']
    figures=make_plots(entries,doc/'figures',star)
    for f in figures: lines.append(f'- [{f}](figures/{f}.png)（另有独立 SVG）')
    lines+=['','## 7. 结论与下一项修改','',
            '具体归因与决策见 [conclusions.md](conclusions.md)。本报告的表格由原始结果自动生成；结论必须结合连续加严、分步比较、完整瞬态与右删失，不能以 valid 或低投影残差代替收敛。','',
            '## 8. 预算、未执行项与复现','',
            f"未执行身份数：{len(missing)}。详见 `evidence/summary.json` 的 missing；失败与中断仍保留在账本，不从预算删除。",'',
            '```bash','python3 scripts/ltv_standalone_study/run_study.py freeze','python3 scripts/ltv_standalone_study/run_study.py verify','python3 scripts/ltv_standalone_study/run_study.py fixed','python3 scripts/ltv_standalone_study/run_study.py discretization','python3 scripts/ltv_standalone_study/run_study.py diagnostic','python3 scripts/ltv_standalone_study/run_study.py gains','python3 scripts/ltv_standalone_study/run_study.py lifecycle','python3 scripts/ltv_standalone_study/run_study.py initialization','python3 scripts/ltv_standalone_study/run_study.py report','```','',
            '默认外部输出路径固定在 protocol.json；也可用 `--out` 指定新目录。仅完整身份（配置、输入生成器、代码与库版本）一致的成功结果复用，绝不复用历史 probe 汇总值。驱动进程排他锁避免重叠调度；固定点阶段串行，后续阶段最多两个模拟子进程。短测并发偏差见 decision_log.md。生产文件不写回，后续 VIO 闭环与 ATE 不属于本轮。']
    (doc/'report.md').write_text('\n'.join(lines)+'\n')
    print(status,len(entries),'completed;',len(missing),'missing')
