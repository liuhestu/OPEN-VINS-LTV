"""Read-only event diagnosis for the counted R3 V2_03 development run."""
import json,collections,hashlib
from pathlib import Path
import numpy as np
p=Path('/home/he/output/ltv_passive_hardening_v2/real_development/R3_V2_03_01'); out=p/'evaluation_v1';m=json.loads((out/'metrics.json').read_text());a=dict(np.load(out/'error_curves.npz'))
new={r['camera_ns']:r for r in map(json.loads,(p/'P_NEW/features.jsonl').read_text().splitlines())}
oldpath=Path(m['reuse']['historical_metrics']).parent/'overlay_features.jsonl';old={r['camera_ns']:r for r in map(json.loads,oldpath.read_text().splitlines())}
ns=a['camera_ns'];dv=np.linalg.norm(a['P_NEW_v']-a['P_PREV_v'],axis=1);t0=float(a['time'][0]);fields=['epoch','sequence','availability_state','reason','bootstrap_count','actual_corrections','observed_features','mature_features','camera_substeps','prediction_angle_p95_rad','velocity_correction_rate','gravity_correction_rate','identity_guard_rejected_ids','capacity_rejected_ids','retirement_events']
def record(i):
 r=new[int(ns[i])];o=old[int(ns[i])]
 return dict(index=int(i),camera_ns=int(ns[i]),t_from_initialized=float(a['time'][i]-t0),v_delta=float(dv[i]),new_v=a['P_NEW_v'][i].tolist(),prev_v=a['P_PREV_v'][i].tolist(),new={k:r.get(k) for k in fields},new_retained_only=sorted(set(r.get('retained_ids',[]))-set(o.get('retained_ids',[]))),prev_retained_only=sorted(set(o.get('retained_ids',[]))-set(r.get('retained_ids',[]))),new_births=[s for s in r.get('seeds',[]) if s['admitted']],prev_births=[s for s in o.get('seeds',[]) if s['admitted']])
first={str(th):int(np.flatnonzero(dv>th)[0]) if np.any(dv>th) else None for th in [0,1e-12,1e-9,1e-6,.01,.1]}
changes=[]
for i,k in enumerate(ns):
 r=new[int(k)];o=old[int(k)]
 if set(r.get('retained_ids',[]))!=set(o.get('retained_ids',[])):changes.append(i)
transitions=[]; reasons=collections.Counter()
for i in range(1,len(ns)):
 r=new[int(ns[i])];prev=new[int(ns[i-1])]
 if prev['joint_ready'] and not r['joint_ready']:
  bad=[]
  for q,limit in [('prediction_angle_p95_rad',.02),('velocity_correction_rate',.5),('gravity_correction_rate',1.)]:
   if r[q] is None or r[q]>limit:bad.append(q)
  if not r['pool_ready']:bad.append('pool_not_ready')
  if not r['correction_diagnostics_valid']:bad.append('diagnostics_invalid')
  if not 8<=np.linalg.norm(r['eta_body'])<=11.5:bad.append('gravity_norm')
  for b in bad:reasons[b]+=1
  transitions.append(dict(index=i,time_from_initialized=float(a['time'][i]-t0),ready_G=r['ready_G'],ready_V=r['ready_V'],reason=r['reason'],violations=bad,**{q:r[q] for q in ['prediction_angle_p95_rad','velocity_correction_rate','gravity_correction_rate','observed_features','mature_features']}))
indices=set()
for i in first.values():
 if i is not None:indices.update(range(max(0,i-1),min(len(ns),i+2)))
if changes:indices.update(range(max(0,changes[0]-1),min(len(ns),changes[0]+2)))
result=dict(first_delta_indices=first,first_management_difference=changes[0] if changes else None,management_difference_packets=len(changes),records=[record(i) for i in sorted(indices)],joint_end_transitions=transitions,joint_end_cause_counts=dict(reasons),bootstrap_counts=sorted(set(r['bootstrap_count'] for r in new.values())),states=dict(collections.Counter(r['availability_state'] for r in new.values())),provenance={'curves_sha':hashlib.sha256((out/'error_curves.npz').read_bytes()).hexdigest(),'historical_management_overlay':str(oldpath),'usage':'Overlay management only; raw numerical values from separately audited original arrays'})
(out/'event_diagnosis.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({k:v for k,v in result.items() if k not in ['records','joint_end_transitions','provenance']},indent=2))
