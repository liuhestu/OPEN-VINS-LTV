"""Build an auditable research report from all first formal runs, not best repeats."""
import csv,json,shutil
import numpy as np
from common import *
from analyze import selected,cache
from evaluate import evaluate
from audit import audit

def render():
 paths=read(OUT/'final_paths.json');assert set(paths)==set(SEQUENCES)
 records=latest_runs();freeze=read(OUT/'weight_freeze.json');repeat=read(OUT/'repeat_summary.json')
 extended=read(OUT/'extended_evidence.json');cost=read(OUT/'cost_profile.json')
 dest=ROOT/'docs/ltv/evidence/value_study';dest.mkdir(parents=True,exist_ok=True)
 result={};index=[];diagnostics=[];mechanisms={}
 for seq in SEQUENCES:
  result[seq]={}
  for mode in MODES:
   run=records[paths[seq][mode]];p=Path(run['path']);metrics=read(p/'metrics.json');eng=read(p/'engineering_audit.json');result[seq][mode]=metrics
   item={'sequence':seq,'mode':mode,'id':run['id'],'path':str(p),'identity':run['identity']['binary'],'files':{name:sha(p/name) for name in ['manifest.json','trajectory.csv','audit.csv','value.jsonl.gz','metrics.json','engineering_audit.json','mechanism.json']}}
   index.append(item)
   diagnostics.append({'sequence':seq,'mode':mode,'ATE_m':metrics.get('ate_rmse_m'),'rotation_RMSE_deg':metrics.get('rotation_rmse_deg'),'velocity_RMSE_mps':metrics.get('velocity_rmse_mps'),'coverage':metrics.get('coverage'),'sensor_coverage':metrics.get('sensor_coverage'),'runtime_s':run['seconds'],'localization_status':metrics.get('localization_status'), 'G_updates':eng.get('counts',{}).get('accepted_G',0),'V_updates':eng.get('counts',{}).get('accepted_V',0),'visual_update_events':eng.get('visual_update_events'), 'shadow_native_max_relative':eng['checks'].get('shadow_actual_max_relative_error'),**{name:eng.get(name,{}).get('p95') if eng.get(name) else None for name in ['G_nis','V_nis','G_residual','V_residual','G_gain_norm','V_gain_norm','G_update_norm','V_update_norm']}})
  p=Path(records[paths[seq]['P']]['path']);mechanisms[seq]=read(p/'mechanism.json')
  shutil.copyfile(p/'mechanism.json',dest/(seq+'_mechanism.json'))
 write(OUT/'final_results.json',result);write(dest/'final_results.json',result);write(dest/'artifact_index.json',index)
 with (dest/'diagnostics.csv').open('w') as f:
  w=csv.DictWriter(f,fieldnames=list(diagnostics[0]));w.writeheader();w.writerows(diagnostics)
 for name in ['qualification.json','strata.json','weight_freeze.json','repeat_summary.json','gt_preflight.json','calibration_audit.json','smoke_parity.json','previous_baseline_parity.json','extended_evidence.json','cost_profile.json']:
  shutil.copyfile(OUT/name,dest/name)
 shutil.copyfile(OUT/'provenance/gt_release_verification.json',dest/'gt_release_verification.json')
 shutil.copyfile(OUT/'provenance/runner_loaded_source_note.json',dest/'runner_loaded_source_note.json')
 shutil.copyfile(OUT/'environment.json',dest/'environment.json')
 fmt=lambda x:f'{x:.6f}' if x is not None else 'N/A'
 percent=lambda x:f'{100*x:+.3f}%'
 lines=['# LTV 信息价值与 UZH-FPV 代表集验证报告','',
 '本轮以[value_study_goal.md](value_study_goal.md)为合同；旧EuRoC全量报告不改写，MH_04按用户要求跳过。',
 '完整运行矩阵与科学有效性分开判断。数据选择、5%机制门槛、GT求导窗口和最大80次回放均在本轮效果分析前确定。','',
 '## 实现、冻结与完整性','',
 f"冻结辅助参数：{freeze['selected']}，G=10°、V=1m/s；额外权重候选{len(freeze['extra_candidates'])}组。G/V均未达到2/3 UZH开发序列的预定机制门槛，因此没有增强权重，也没有改observer参数。",'',
 '新增默认OFF的只读诊断，在真实压缩视觉块上保存prior/FEJ/P、LTV snapshot及H/res/R、单步shadow、实际posterior。正式主状态仍只调用一次原生EKF。新runner读取sensor-only派生目录，GT仅由Python离线程序读取。',
 '完整源文件/配置/二进制/库身份见输出根experiment_manifest.json与逐run manifest。原生预测、State、StateHelper、IMU、JPLQuat、UpdaterHelper和observer核心未改。',
 'B/P记录包含完整状态/P/FEJ/clone/视觉集合digest；新EuRoC B与旧正式B三条全部字节一致。诊断OFF/ON及B/P短段均字节一致。',
 '所有formal模式使用同一原生设置；UZH使用对应固定标定、MSCKF-only配置，不能代替OpenVINS完整原默认配置的性能结论。','',
 '### 输入完整性与初始化','',
 '完整回放指消费全部IMU和全部**精确同步的双目对**，并不把未配对图像算作已送入双目估计器。以下为B；每种模式使用相同输入。V2_03左右图像数量不等，未配对数量如实保留，未按效果选择裁剪区间。','',
 '|序列|同步双目对|未配对左/右图|IMU样本|B输出数|首次输出距首图秒数|','|---|---:|---|---:|---:|---:|']
 inputs=read(OUT/'inputs.json')
 for seq in SEQUENCES:
  r=records[paths[seq]['B']];p=Path(r['path']);replay=r['replay']
  first_camera=int((Path(inputs[seq]['replay_root'])/'cam0/data.csv').read_text().splitlines()[1].split(',')[0])*1e-9
  first_output=float((p/'trajectory.csv').read_text().splitlines()[1].split(',')[0])
  delay=first_output-inputs[seq]['offset']-first_camera
  lines.append(f"|{seq}|{replay['camera_packets']}|{replay['unmatched_left_images']}/{replay['unmatched_right_images']}|{replay['imu_consumed']}|{replay['output_rows']}|{delay:.3f}|")
 lines+=['',
 '## 直接误差：Passive LTV 与 OpenVINS prior','',
 '同一物理IMU时刻、同机体系，不施加ATE对齐旋转。G为方向夹角RMSE(deg)，V为向量RMSE(m/s)。每分支仅在双方参考量和LTV该分支有效的相同支持上计算；有效比例另列。V1_01姿态参考有官方已知限制。','',
 '|序列|Prior G|LTV G|LTV G胜出|Prior V|LTV V|LTV V胜出|G/V有效事件比例|','|---|---:|---:|---:|---:|---:|---:|---:|']
 for seq,m in mechanisms.items():
  g=m['strata']['all']['G']['own'];v=m['strata']['all']['V']['own']
  lines.append(f"|{seq}|{fmt(g.get('baseline_rmse'))}|{fmt(g.get('method_rmse'))}|{100*g.get('method_win_fraction',0):.2f}%|{fmt(v.get('baseline_rmse'))}|{fmt(v.get('method_rmse'))}|{100*v.get('method_win_fraction',0):.2f}%|{100*g['effective_fraction']:.1f}%/{100*v['effective_fraction']:.1f}%|")
 lines+=['','有效事件比例的分母是该序列**全部更新诊断事件**，包含尚无GT支持、warmup及无效snapshot；未把这些事件从分母删除。以下另列参考支持，避免把GT缺失等同LTV不可用。','',
 '|序列|全部事件|GT姿态|GT速度|GT+LTV G|GT+LTV V|','|---|---:|---:|---:|---:|---:|']
 for seq,e in extended.items():
  c=e['reference_coverage'];lines.append('|'+ '|'.join([seq]+[str(c[k]) for k in ['diagnostic_events','pose','velocity','G_and_LTV','V_and_LTV']])+'|')
 lines+=['','![Direct errors](../evidence/value_study/figures/direct_errors.png)','',
 '## UZH是否更困难，以及困难区间是否受益','',
 '下面比较完整原始IMU输入与GT支持区间的一秒窗口RMS之P95。角速度单位rad/s；比力变化为abs(norm(a)−g)，单位m/s²，**不是真实平移加速度**。末尾无GT支持的冲击保留在全输入列，不用于解释飞行区间优势。','',
 '|序列|全输入角速P95|GT支持角速P95|全输入比力P95|GT支持比力P95|','|---|---:|---:|---:|---:|']
 for seq,e in extended.items():
  motion=e['motion'];lines.append('|'+ '|'.join([seq]+[fmt(motion[support][key]['p95']) for key in ['omega_rms_radps','specific_force_variation_mps2'] for support in ['all_input','GT_pose_support']])+'|')
 lines+=['','困难分层阈值由EuRoC三条和UZH开发三条分别冻结；验证复用UZH阈值。下表按原先规定的整体/高旋转/高比力/弱视觉/无视觉报告，交集的全部结果见mechanism JSON；时长不足5s仅描述。G/V单步改善以百分比表示，正值为受益。','',
 '|序列|分层|G/V有效秒数|LTV/Prior G RMSE比|LTV/Prior V RMSE比|G单步改善|V单步改善|','|---|---|---:|---:|---:|---:|---:|']
 for seq,m in mechanisms.items():
  for label in ['high_rotation','high_force','weak_visual','zero_visual']:
   d=m['strata'][label];g=d['G']['own'];v=d['V']['own'];ig=d['G']['incremental'];iv=d['V']['incremental']
   ratio=lambda z:fmt(z['method_rmse']/z['baseline_rmse']) if z.get('baseline_rmse',0)>0 else 'N/A'
   gain=lambda z:percent(z['improvement']) if z.get('improvement') is not None else 'N/A'
   lines.append(f"|{seq}|{label}|{g.get('seconds',0):.1f}/{v.get('seconds',0):.1f}|{ratio(g)}|{ratio(v)}|{gain(ig)}|{gain(iv)}|")
 lines+=['',
 '## 同prior更新归因与误差互补性','',
 '表中正值表示visual+辅助的单步误差比visual-only更小。此处保持历史、prior/P和已接受视觉块不变，只是单步反事实；不是闭环干预证明。K分块范数不被当作增量收益。','',
 '|序列|G单步RMSE改善|G 95%块区间|V单步RMSE改善|V 95%块区间|','|---|---:|---|---:|---|']
 for seq,m in mechanisms.items():
  g=m['strata']['all']['G']['incremental'];v=m['strata']['all']['V']['incremental']
  ci=lambda x:'['+', '.join(percent(z) for z in x['ci95'])+']' if x.get('ci95') else 'N/A'
  lines.append(f"|{seq}|{percent(g['improvement'])}|{ci(g)}|{percent(v['improvement'])}|{ci(v)}|")
 lines+=['','块bootstrap使用1s、2000次、seed42，区间仅描述该序列内部的连续数据，不构成跨场景统计显著性。三轴/切平面相关、交叉二阶矩、prior误差与修正方向内积以及连续受益/受损区间保存在逐序列mechanism JSON。',
 '近零方差相关系数标为null；共享输入相关性仍未建模，不把经验相关或NIS通过解释成全系统一致性。','',
 '|序列|G相关对角线|V相关对角线|G误差·候选方向均值|V误差·候选方向均值|','|---|---|---|---:|---:|']
 for seq,m in mechanisms.items():
  c=m['correlation'];diag=lambda b:', '.join(fmt(row[i]) for i,row in enumerate(c[b]['centered_correlation'])) if c[b]['centered_correlation'] else 'N/A'
  lines.append(f"|{seq}|{diag('G')}|{diag('V')}|{fmt(c['G'].get('mean_prior_error_dot_correction'))}|{fmt(c['V'].get('mean_prior_error_dot_correction'))}|")
 lines+=['','候选方向为LTV−prior；内积为负只表示小步朝此方向的一阶趋势，不保证有限步或实际Kalman更新获益。G在共同参考切平面以rad计算，二维基随参考方向确定；V单位为m/s，不能跨单位比较内积大小。','',
 '### 参考速度求导敏感性','',
 '各窗口均使用自己的有效支持，支持数随窗口变化保存在JSON；不外推。下表列出0.05/0.10/0.20s各自的LTV/prior速度RMSE比和单步V改善；大于1表示LTV较差。','',
 '|UZH序列|LTV/prior比（0.05/0.10/0.20s）|单步V改善（0.05/0.10/0.20s）|','|---|---|---|']
 for seq in DEV+VALIDATION:
  s=mechanisms[seq]['strata']['all']['V']['sensitivity'];ratios=[];gains=[]
  for w in ['0.05','0.1','0.2']:
   d=s[w];ratios.append(fmt(d['own']['method_rmse']/d['own']['baseline_rmse']));gains.append(percent(d['incremental']['improvement']))
  lines.append('|'+ '|'.join([seq,' / '.join(ratios),' / '.join(gains)])+'|')
 lines+=['',
 '## 闭环ATE与鲁棒性','',
 '单位m，Δ=100(B−method)/B。沿用最近GT≤20ms、无尺度SE3；传感器输出覆盖与GT可评价支持分开。基线首次输出后固定支持，门槛99%，未为某模式裁困难区间。RPE1s/5s、旋转/速度及count/NIS等见原始metrics和diagnostics.csv。','',
 '|序列|B|P|G|ΔG|V|ΔV|GV|ΔGV|','|---|---:|---:|---:|---:|---:|---:|---:|---:|']
 for seq,ms in result.items():
  b=ms['B'].get('ate_rmse_m');cells=[seq,fmt(b),fmt(ms['P'].get('ate_rmse_m'))]
  for mode in ['G','V','GV']:
   v=ms[mode].get('ate_rmse_m');valid=ms[mode].get('localization_status')=='VALID' and ms['B'].get('localization_status')=='VALID'
   cells += [('FAIL / ' if not valid else '')+fmt(v),percent(1-v/b) if valid and b and v is not None else '—']
  lines.append('|'+ '|'.join(cells)+'|')
 lines+=['','|分组|共同有效/总数|B均值|G均值|V均值|GV均值|ΔGV|','|---|---:|---:|---:|---:|---:|---:|']
 for name,seqs in [('EuRoC回归',EUROC),('UZH开发',DEV),('UZH独立验证',VALIDATION),('全部代表集',SEQUENCES)]:
  valid=[s for s in seqs if all(result[s][m].get('localization_status')=='VALID' for m in MODES)]
  means={m:float(np.mean([result[s][m]['ate_rmse_m'] for s in valid])) if valid else None for m in ['B','G','V','GV']}
  lines.append('|'+ '|'.join([name,f'{len(valid)}/{len(seqs)}']+[fmt(means[m]) for m in ['B','G','V','GV']]+[percent(1-means['GV']/means['B']) if valid else '—'])+'|')
 lines+=['','B/GV的RPE平移RMSE(m)如下；对应旋转、配对数及G/V单分支结果均在原始结果JSON。','',
 '|序列|B 1s RPE|GV 1s RPE|B 5s RPE|GV 5s RPE|','|---|---:|---:|---:|---:|']
 for seq,ms in result.items():
  lines.append('|'+ '|'.join([seq]+[fmt(ms[m].get('rpe',{}).get(dt,{}).get('translation_rmse_m')) for dt in ['1.0','5.0'] for m in ['B','GV']])+'|')
 lines+=['','巨大定位漂移预先定义为ATE>max(5m,GT包围盒对角线)，与数值有限/输入完成区分；该标签不覆盖所有可能的定位质量问题。若发生失败，原ATE和运行目录全部保留。','',
 '### 辅助是否实际参与','',
 '以下取正式GV；分母为进入MSCKF阶段的诊断事件。NIS拒绝和warmup分别保留，不能将接受次数或小NIS解释为量测更接近GT。每个事件EKF调用数均审计为0或1。','',
 '|序列|事件数|G接受|V接受|V NIS拒绝|warmup/core无效|G/V更新范数P95|shadow/native IMU均值最大相对差|','|---|---:|---:|---:|---:|---:|---|---:|']
 for seq in SEQUENCES:
  e=read(Path(records[paths[seq]['GV']]['path'])/'engineering_audit.json');counts=e['counts'];reasons=e['reasons']
  lines.append(f"|{seq}|{e['diagnostic_events']}|{counts.get('accepted_G',0)}|{counts.get('accepted_V',0)}|{reasons['V_reason'].get('nis',0)}|{reasons['V_reason'].get('warmup_or_core_invalid',0)}|{fmt(e['G_update_norm']['p95'])}/{fmt(e['V_update_norm']['p95'])}|{e['checks']['shadow_actual_max_relative_error']:.2e}|")
 lines+=['','更新范数是日志中的整体状态修正分量范数，混合不同状态单位，仅作同实现内部的作用量诊断，不能当作统一物理误差。RPE与各模式覆盖完整数值保留在final_results.json。','',
 '## 复测','', '|序列|模式|首次ATE|复测ATE|轨迹字节一致|完整审计字节一致|','|---|---|---:|---:|---|---|']
 for s,modes in repeat.items():
  for m,v in modes.items():lines.append(f"|{s}|{m}|{fmt(v['first_ate'])}|{fmt(v['repeat_ate'])}|{v['trajectory_exact']}|{v['audit_exact']}|")
 lines+=['','同机同输入的重复只检验确定性复现；不替代不同平台、噪声试验或跨场景统计。','',
 '## 分项成本','',
 '为保持正式回放的冻结库不变，另用可选LD_PRELOAD包裹现有辅助build、原生EKF和gzwrite符号，只计时并调用原函数。四次短段计入短段预算；每次均与原未插桩GV短段比较轨迹及完整状态审计。实际辅助构造与三个shadow辅助构造按每事件调用顺序区分，并严格检查次数。诊断OFF列最接近本机估计链路开销；不是独立实时性能基准。','',
 '|序列|诊断|实际辅助构造均值/P95 ms|原生EKF均值/P95 ms|gzip均值/P95 ms|轨迹/审计一致|','|---|---|---:|---:|---:|---|']
 for seq,modes in cost.items():
  for mode,c in modes.items():
   cell=lambda key:f"{fmt(c[key]['mean_ms'])}/{fmt(c[key]['p95_ms'])}"
   lines.append('|'+ '|'.join([seq,mode,cell('actual_auxiliary_build'),cell('native_ekf'),cell('gzip_write'),str(all(c['exact_to_uninstrumented'].values()))])+'|')
 lines+=['','Observer、MSCKF扣除诊断计算、诊断计算总时长见各mechanism JSON；短段每事件均值/P95见cost_profile.json。gzwrite只计压缩写入调用，不含析构flush；插桩日志自身开销在计时区间之外。','',
 '## 参考数据、时间与成本限制','',
 '- UZH本地6条GT SHA与官网当前v3包逐字节一致；核对仅下载ZIP目录和GT，不替换原文件。其生成涉及视觉、IMU、全局位置批优化，姿态与速度不是完全独立传感器真值。',
 '- UZH参考速度由0.10s局部三次位置拟合求导；0.05/0.20s敏感性全部保留。姿态SLERP，不跨10ms间隙/不外推；GT晚开始的输入仍完整回放。',
 '- OpenVINS官方指出V1_01原始姿态GT不准确；本轮保留原GT，G及机体系V结论带该限制，不依赖这一条判断有效性。',
 '- 固定cam0时差用于预测、LTV和输出的物理时刻，cam1标定差异记录。亚纳秒文本舍入≤0.5ns，双目还核对原Decimal一致，未用舍入伪造同步。',
 '- 正式运行总成本包含完整矩阵JSON压缩、反事实求解与数值诊断；辅助/EKF细分取自独立短段，不能把正式总耗时直接解释成生产实时性能。',
 '- LTV Riccati、主P和辅助R含义区分；原生数学测试不证明observer估计质量，也不证明共享输入相关性已解决。','',
 '## 证据与复现','',
 f"输出根：`{OUT}`。完整回放{sum(not r['short'] for r in records.values())}/80，短段{sum(r['short'] for r in records.values())}/20。",'',
 '- [逐run文件哈希索引](../evidence/value_study/artifact_index.json)、[闭环原始结果](../evidence/value_study/final_results.json)、[辅助诊断CSV](../evidence/value_study/diagnostics.csv)。',
 '- [实现与测试验收](value_study_validation.md)、[完成核验](../evidence/value_study/completion_audit.json)；后者核对45个正式结果、12个复测及全部运行的冻结身份和只读边界。',
 '- [构建与分析环境](../evidence/value_study/environment.json)、[两条runner磁盘/已导入源码哈希说明](../evidence/value_study/runner_loaded_source_note.json)；原始manifest保留，实际数值命令和冻结估计器不变。',
 '- [资格门槛](../evidence/value_study/qualification.json)、[参数冻结](../evidence/value_study/weight_freeze.json)、[复测](../evidence/value_study/repeat_summary.json)。',
 '- [GT约定检查](../evidence/value_study/gt_preflight.json)、[官方GT版本核对](../evidence/value_study/gt_release_verification.json)、[标定差异](../evidence/value_study/calibration_audit.json)。',
 '- 各序列时序图（同名PDF可导出）：'+', '.join(f'[{s}](../evidence/value_study/figures/{s}.png)' for s in SEQUENCES)+'。',
 '- 脚本顺序：prepare.py（首次）/--check（核对）→study.py diagnostic→freeze→final→repeats→plots.py→report.py；已有输入和冻结权重不会自动覆盖。',
 '', '参考：[OpenVINS数据集说明](https://docs.openvins.com/gs-datasets.html)、[UZH官方GT方法](https://rpg.ifi.uzh.ch/docs/RAL2021_Cioffi.pdf)、[UZH数据](https://fpv.ifi.uzh.ch/datasets/)。','']
 means={mode:float(np.mean([result[s][mode]['ate_rmse_m'] for s in VALIDATION])) for mode in ['B','G','V','GV']}
 verdict=['## 结论与下一步判断','',
 '当前实现有少量可复现的闭环ATE收益，但证据不支持“换更激烈的数据集后，LTV就成为更准确、更有增量价值的G/V观测”。本轮未达到预先约定的机制门槛，因此保持原权重，不继续调参或扩展序列。','',
 f"UZH独立验证三条的ATE均值相对B：G {percent(1-means['G']/means['B'])}，V {percent(1-means['V']/means['B'])}，GV {percent(1-means['GV']/means['B'])}。G三条均有正向变化，但主要收益来自outdoor_forward_5；GV有两条改善、一条近零退化；V有改善也有退化。这些是固定配置下的实际结果，不是跨场景显著性结论。",'',
 '验证集原生输出中的GT可评价比例分别为'+', '.join(f"{s} {100*result[s]['B']['gt_time_fraction']:.1f}%" for s in VALIDATION)+'。尤其主要ATE收益所在outdoor_forward_5的GT支持较短。所有模式都完整回放且在固定GT支持上输出覆盖100%，但不能将这些ATE结果推广为整段无GT输入的定位可靠性。','',
 '|分支|本轮判断|依据|','|---|---|---|',
 '|G|局部闭环收益，机制证据不足|六条UZH中LTV G方向RMSE均明显大于prior；同prior单步收益极小，开发资格为否。不能据少量ATE改善增强G。|',
 '|V|当前配置不支持稳定增益|六条UZH中LTV速度RMSE均明显大于prior；参考求导窗口不改变这一量级结论；验证平均ATE略退化。|',
 '|GV|局部ATE收益，尚不能证明互补机制|验证平均有小幅改善，但未达到预定直接精度/单步机制门槛；组合并未稳定优于G-only。|','',
 'UZH在本代表集中的参考速度和比力变化确实更大；并非每条UZH的角速度都高于每条EuRoC。六条UZH的GT支持区间比力变化P95约3.99–9.32m/s²，EuRoC为1.77–1.95m/s²。更激烈的运动没有自动转化为当前LTV输出质量优势，因此不能把“EuRoC太简单”作为未见明显提升的主要解释。','',
 '更直接的瓶颈是第一层输出质量：在共同有效支持上，UZH参考速度范数RMS为4.08–6.91m/s，LTV为0.61–2.21m/s，同时存在明显方向/重力误差。这是观测到的幅度不足，不等于已证明某个增益、坐标或时延是根因。源码中路标和基础状态从零初始化、特征更替及连续健康帧条件值得单独检验，但本轮没有通过修改它们来寻找更好ATE。','',
 '经验误差相关性随序列和轴变化，不能认定全为独立，也不能把相关性宣布为本轮失败的唯一原因。先证明第一层在受控几何/已知输入下能收敛并保持速度幅度，再检查特征寿命与动态可观性，之后才值得讨论相关融合或更强权重。这是后续建议，本Goal到此停止；不自动改核心、不扩展全量UZH。','',
 '工程验收和科学结论分开：输入、数学/FEJ、旁路一致性、联合提交及复测通过，并不证明有新的独立信息，也不保证Riccati可作为量测噪声。','']
 lines[5:5]=verdict
 (ROOT/'docs/ltv/value_study/value_study_report.md').write_text('\n'.join(lines))
if __name__=='__main__':render()
