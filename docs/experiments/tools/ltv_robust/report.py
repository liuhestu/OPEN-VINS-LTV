"""Report every frozen candidate, including regressions, on shared EuRoC support."""
import json,sys
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from study import OUT,ROOT,ASL,SEQS,CANDIDATES,nearest,write

def main():
 metrics=json.loads((OUT/'metrics.json').read_text());selection=json.loads((OUT/'selection.json').read_text())
 rpe={};summary={}
 for seq in SEQS:
  gt=np.loadtxt(ASL/seq/'mav0/state_groundtruth_estimate0/data.csv',delimiter=',',comments='#');gt[:,0]*=1e-9
  b=np.loadtxt(OUT/'runs'/(seq+'-B')/'trajectory.csv',delimiter=',',skiprows=1)
  t=b[:,0];gi=nearest(gt[:,0],t);ok=abs(gt[gi,0]-t)<=.02;t=t[ok];truth=gt[gi[ok]]
  rpe[seq]={}
  for name in CANDIDATES:
   e=np.loadtxt(OUT/'runs'/(seq+'-'+name)/'trajectory.csv',delimiter=',',skiprows=1);j=nearest(e[:,0],t)
   assert np.max(abs(e[j,0]-t))<=1e-6
   est=e[j];rg=Rotation.from_quat(truth[:,[5,6,7,4]]);re=Rotation.from_quat(est[:,1:5]);item={}
   for dt in [1.,5.]:
    end=nearest(t,t+dt);a=np.flatnonzero(abs(t[end]-(t+dt))<=.025);z=end[a]
    delta=rg[a].inv().apply(truth[z,1:4]-truth[a,1:4])-re[a].inv().apply(est[z,5:8]-est[a,5:8])
    rot=(rg[a].inv()*rg[z]).inv()*(re[a].inv()*re[z])
    item[str(dt)]={'pairs':len(a),'translation_rmse_m':float(np.sqrt(np.mean(np.sum(delta**2,axis=1)))),'rotation_rmse_deg':float(np.degrees(np.sqrt(np.mean(rot.magnitude()**2))))}
   rpe[seq][name]=item
 for name in CANDIDATES:
  records=[metrics[s]['modes'][name] for s in SEQS];audits=[json.loads((OUT/'runs'/(s+'-'+name)/'audit_results.json').read_text()) for s in SEQS]
  item={'mean_ATE_m':float(np.mean([r['ate_rmse_m'] for r in records])),'mean_RPE_1s_m':float(np.mean([rpe[s][name]['1.0']['translation_rmse_m'] for s in SEQS])),'mean_RPE_5s_m':float(np.mean([rpe[s][name]['5.0']['translation_rmse_m'] for s in SEQS])),'complete_sequences':len(records),'runtime_seconds_sum':sum(a['runtime_seconds'] for a in audits)}
  if name!='B':
   item.update(selection['scores'][name]);item['branches']={}
   for branch in ['G','V']:
    count=sum(a[branch]['updates'] for a in audits);weighted=sum(a[branch]['huber_weight']['below_one'] for a in audits)
    item['branches'][branch]={'updates':count,'huber_downweighted_updates':weighted,'mean_weight_on_accepted':sum((a[branch]['huber_weight']['mean'] or 0)*a[branch]['huber_weight']['n'] for a in audits)/max(1,count),'coverage_output':count/sum(a['replay']['output_rows'] for a in audits),'mean_update_norm':sum((a[branch]['update_norm']['mean'] if a[branch]['update_norm'] else 0)*a[branch]['updates'] for a in audits)/max(1,count)}
  summary[name]=item
 write(OUT/'rpe.json',rpe);write(OUT/'summary.json',summary)
 repeat={s:json.loads((OUT/'runs'/(s+'-'+selection['best']+'-repeat')/'repeat_check.json').read_text()) for s in SEQS};write(OUT/'repeat_checks.json',repeat)
 lines=['# 质量门控 / Huber 全 EuRoC 调参对照','', '**这是全10条样本内调参结果，MH04按用户原指示跳过；没有加入滑窗非线性后端，也不构成跨数据集泛化验证。**','', '按预先固定的ATE选参目标，本轮没有发现质量门控、Huber或两档组合权重优于原GV。原GV的平均逐序列相对ATE改善约0.86%，6条改善、4条退化；这说明存在小幅样本内收益，不能判定LTV与OpenVINS不兼容，也不能称已证明稳定泛化。', '', '候选与选参目标在运行前冻结；70次候选完整运行、10次选中配置重复。重复的轨迹和完整审计要求逐字节相同。原始NIS始终开启，observer参数不变。现有OpenVINS门控的innovation归一化为sqrt(3N)，VINS为sqrt(N)，因此同名0.03阈值并不等价；本轮也未复刻G最低15特征和独立cooldown，不能称T2逐项parity。质量门控和Huber见[计划](plan.md)，滑窗方案边界见[设计](window_design.md)。','', '|候选|平均ATE m|平均相对B变化 %（负为改善）|中位变化 %|最坏变化 %|1s RPE m|','|---|---:|---:|---:|---:|---:|']
 for n,x in summary.items():lines.append(f"|{n}|{x['mean_ATE_m']:.6f}|{x.get('mean_relative_ATE_percent',0):+.3f}|{x.get('median_relative_ATE_percent',0):+.3f}|{x.get('worst_relative_ATE_percent',0):+.3f}|{x['mean_RPE_1s_m']:.6f}|")
 lines+=['',f"按预先固定目标选中的辅助配置为 **{selection['best']}**。这里是辅助候选中的最优；若仍高于B，不能称优于原生。平均绝对ATE和平均逐序列相对变化属于不同统计量。",'', '|序列|B ATE|raw ATE|Q ATE|H ATE|QH ATE|QH strong ATE|QH weak ATE|','|---|---:|---:|---:|---:|---:|---:|---:|']
 for s in SEQS:lines.append('|'+s+'|'+'|'.join(f"{metrics[s]['modes'][n]['ate_rmse_m']:.6f}" for n in CANDIDATES)+'|')
 lines+=['','QH_strong的平均1秒RPE低于raw（0.040272 vs 0.040625 m），但平均相对ATE变为退化0.062%，最坏单序列退化1.684%。这是指标间的取舍；本轮没有看到RPE后改换选参目标。', '', '|候选|G接受数|V接受数|G被Huber降权数|V被Huber降权数|','|---|---:|---:|---:|---:|']
 for n,x in summary.items():
  if n=='B':continue
  g=x['branches']['G'];v=x['branches']['V'];lines.append(f"|{n}|{g['updates']}|{v['updates']}|{g['huber_downweighted_updates']}|{v['huber_downweighted_updates']}|")
 lines+=['','Huber只处理通过原始NIS和quality gate的辅助块，因此可能没有实际降权。特别是V门控要求残差≤0.5m/s，而组合候选sigma≥0.5m/s，白化范数≤1<delta=2，所以这些组合的V分支在本次单步更新中理论上不会触发Huber。若权重恒为1，该候选就是对照验证，不能声称Huber带来提升。sigma同时影响更新增益和原始NIS，因此strong/weak并不是仅改变已接受观测的增益。接受率、更新量、逐序列RPE/ATE和全部失败记录需共同解读，不能只报告最佳均值。','', '![逐序列ATE变化](evidence/ate_changes.png)', '', '## 验证与产物','', '独立ROS2构建和10个原生测试通过；其中新测试直接覆盖实际UpdaterLTV的降权、原始NIS/quality拒绝、完整Joseph协方差与一次提交。源observer、原预测、golden未修改。10次选中配置重复的轨迹/审计逐字节相同。扩展G大角度和native delta测试通过；第一次调用误用build路径退出127，程序未执行，修正路径后的退出码为0，两个日志均保留。','', '紧凑证据见[evidence](evidence/)，全部配置、输入哈希、有效选项、命令、退出码和日志保存在 `/home/he/output/openvins_ltv_robust_fusion_20261003`。构建复用旧冻结ov_core/ov_init依赖，ov_msckf单独编译；完整矩阵各自单线程，最多4任务并发；两次短段回归及扩展测试曾与矩阵重叠运行。耗时是该并发设置的墙钟时间，不与旧单任务测量直接作性能比。','', '复现入口：`python3 -B docs/experiments/tools/ltv_robust/study.py`（严格身份核对包括HEAD，应使用冻结实现提交0a7a5aa；新文档提交不会改写旧实验身份）；`python3 -B docs/experiments/tools/ltv_robust/report.py`。']
 (ROOT/'docs/ltv/robust_fusion/report.md').write_text('\n'.join(lines)+'\n')
 evidence=ROOT/'docs/ltv/robust_fusion/evidence';evidence.mkdir(exist_ok=True)
 for name in ['protocol','tests','extended_huber_test','build_result','disabled_regression','metrics','selection','summary','rpe','repeat_checks']:(evidence/(name+'.json')).write_bytes((OUT/(name+'.json')).read_bytes())
 print(json.dumps(summary,indent=2))
if __name__=='__main__':main()
