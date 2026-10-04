import hashlib,json,sys
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path('/home/he/open_vins_ltv_ws/src/open_vins_ltv');OUT=Path('/home/he/output/ltv_active_landmark_consistency_r4');DOC=ROOT/'docs/ltv/active_landmark_consistency_r4'
sys.path.insert(0,str(ROOT/'scripts/ltv_active_consistency_r4'))
import real_evaluate
f=json.loads((DOC/'frozen_config.json').read_text());plan=json.loads((OUT/'formal_evaluation_plan.json').read_text())
records=[json.loads((Path(q['out'])/'metrics.json').read_text()) for q in plan]
raw=real_evaluate.base.assess_all11(records);assert raw['status']=='NOT_MET' and raw['failed_sequences']==['MH_04_difficult']
views=[dict(r) for r in records]
for r in views:
 if r['sequence']=='MH_04_difficult':r.update(baseline_valid=False,baseline_invalid_reason='native_estimator_failure')
view=real_evaluate.base.assess_all11(views)
view.update(view='B_ONLY_PREDECLARED_FAILURE_CONTRACT',original_assessment=str(OUT/'formal_all11_assessment.json'),exception_proof=str(DOC/'drafts/mh04_predeclared_baseline_failure_review.md'),full_task_success=False,original_records_unchanged=True)
(OUT/'formal_B_only_contract_assessment.json').write_text(json.dumps(view,indent=2)+'\n')
figdir=DOC/'figures';figdir.mkdir(exist_ok=True)
lines=['# Frozen C02 real confirmation','', '| Sequence | G ready % | V ready % | Joint longest s | ready V RMSE m/s | ready G RMSE deg | Raw V PREV / NEW m/s | Result |','|---|---:|---:|---:|---:|---:|---:|---|']
provenance={}
def fmt(v):return 'N/A' if v is None else f'{v:.4f}'
for q,r in zip(plan,records):
 seq=r['sequence'];cov=r['coverage']['P_NEW'];own=r['own_support']['P_NEW'];v=r['raw_regression']['v']
 lines.append(f"| {seq} | {100*cov['ready_G']['fraction']:.2f} | {100*cov['ready_V']['fraction']:.2f} | {cov['joint_ready']['longest_episode']:.2f} | {fmt(own['ready_V']['v_RMSE'])} | {fmt(own['ready_G']['g_angle_deg_RMSE'])} | {fmt(v['before'])} / {fmt(v['after'])} | {r['status']} |")
 curves=Path(q['out'])/'error_curves.npz';a=np.load(curves);t=a['time']-a['time'][0]
 fig,axes=plt.subplots(4,1,figsize=(11,10),sharex=True)
 for ax,key,label in zip(axes[:3],('v','eta','g_angle_deg'),('Velocity error (m/s)','Gravity vector error (m/s2)','Gravity angle error (deg)')):
  for mode,color in (('OpenVINS','0.5'),('P_PREV','#e68613'),('P_NEW','#1678ba')):ax.plot(t,a[mode+'_error_'+key],label=mode,color=color,lw=.8)
  ax.set_ylabel(label);ax.set_yscale('symlog',linthresh=.1);ax.grid(alpha=.25)
 axes[0].legend(ncol=3);axes[0].set_title(seq+' / fixed initialized physical timeline; complete raw errors')
 for i,(mode,key,label) in enumerate((('P_PREV','ready_G','PREV G'),('P_PREV','ready_V','PREV V'),('P_NEW','ready_G','NEW G'),('P_NEW','ready_V','NEW V'))):
  mask=a[mode+'_'+key].astype(bool);axes[3].scatter(t[mask],np.full(mask.sum(),i),s=2,label=label)
 axes[3].set_yticks(range(4),['PREV G','PREV V','NEW G','NEW V']);axes[3].set_ylim(-.5,3.5);axes[3].set_xlabel('Seconds since first initialized B camera packet');axes[3].grid(alpha=.25)
 fig.tight_layout();fig.savefig(figdir/(seq+'.png'),dpi=120);plt.close(fig)
 provenance[seq]={'metrics':str(Path(q['out'])/'metrics.json'),'curves':str(curves),'curves_sha':hashlib.sha256(curves.read_bytes()).hexdigest(),'figure_sha':hashlib.sha256((figdir/(seq+'.png')).read_bytes()).hexdigest()}
 pass
lines.extend(['']+[f"[{r['sequence']} complete curves](../figures/{r['sequence']}.png)" for r in records])
(DOC/'evidence/formal_real_results.md').write_text('\n'.join(lines)+'\n')
for c in f['synthetic_confirmation_cases']:
 p=OUT/'synthetic_confirmation'/c['name']/'evaluation_v1/error_curves.npz';a=np.load(p);t=a['t'];fig,axes=plt.subplots(3,1,figsize=(10,7),sharex=True)
 for ax,key,label in zip(axes,('v','eta','angle'),('Velocity error (m/s)','Gravity vector error (m/s2)','Gravity angle error (deg)')):
  ax.plot(t,a[key],lw=.8,color='#1678ba');ax.set_ylabel(label);ax.set_yscale('symlog',linthresh=.1);ax.grid(alpha=.25)
 axes[0].set_title(c['name']+' / all raw samples / no matching OFF');axes[-1].set_xlabel('Physical seconds');fig.tight_layout();fig.savefig(figdir/(c['name']+'.png'),dpi=120);plt.close(fig)
 provenance[c['name']]={'curves':str(p),'curves_sha':hashlib.sha256(p.read_bytes()).hexdigest(),'figure_sha':hashlib.sha256((figdir/(c['name']+'.png')).read_bytes()).hexdigest()}
(DOC/'evidence/formal_figures_identity.json').write_text(json.dumps(provenance,indent=2)+'\n')
print(json.dumps({'status':'PASS_CLOSEOUT_PLOTS_AND_CONTRACT_VIEWS','original':raw,'B_only':view,'figures':len(provenance)}))
