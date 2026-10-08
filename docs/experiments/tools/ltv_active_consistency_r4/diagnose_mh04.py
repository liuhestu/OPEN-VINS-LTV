"""Read-only frozen MH04 lifecycle summary; no observer, GT read or policy change."""
import collections, hashlib, json
from pathlib import Path
import numpy as np
OUT=Path('/home/he/output/ltv_active_landmark_consistency_r4')
MET=OUT/'real_confirmation/CONF_ON_C02_01_ON_MH_04_difficult/evaluation_v1/metrics.json'
def sha(p):
 h=hashlib.sha256()
 with p.open('rb') as f:
  for b in iter(lambda:f.read(1048576),b''):h.update(b)
 return h.hexdigest()
def brief(r,origin):
 keys=['camera_ns','epoch','availability_state','reason','raw_current','observer_started','bootstrap_count','physical_fault_count','last_bootstrap_source','eligible_seeds','state_features','observed_features','mature_features','actual_corrections','velocity_correction_rate','gravity_correction_rate']
 d={k:r.get(k) for k in keys};d['relative_time']=r['target_imu_time']-origin
 for k in ['v_body','eta_body']:
  a=r.get(k);d[k+'_norm']=float(np.linalg.norm(a)) if a else None
 return d
m=json.loads(MET.read_text());result={'metrics_sha':sha(MET),'scope':'Existing frozen logs and error arrays only; no reference recomputation','methods':{}}
for name,run in m['provenance']['runs'].items():
 p=Path(run)/'features.jsonl';n=0;init=0;origin=None;first=None;last=None;prev=None;trans=[];reasons=collections.Counter();seed=collections.Counter();active=collections.Counter();birth=[];rows=[];checks=[]
 with p.open() as f:
  for line in f:
   r=json.loads(line);n+=1
   if not r.get('initialized'):continue
   init+=1
   if origin is None:origin=r['target_imu_time'];first=brief(r,origin);checks=list(r)
   last=brief(r,origin);key=tuple(r.get(k) for k in ['availability_state','raw_current','bootstrap_count','physical_fault_count'])
   if key!=prev:trans.append(last);prev=key
   reasons[r.get('reason','MISSING')]+=1
   for s in r.get('seeds',[]):
    seed[s.get('reason','MISSING')]+=1
    if s.get('mean_written'):birth.append({'time':last['relative_time'],'id':s['id'],'epoch':r.get('epoch'),'source':s.get('source')})
   for a in r.get('active_consistency',[]):active[str(a.get('reason','MISSING'))+' / '+str(a.get('action','MISSING'))]+=1
   rows.append((r['camera_ns'],r.get('raw_current'),r.get('observed_features'),r.get('mature_features')))
 result['methods'][name]={'path':str(p),'sha':sha(p),'packets':n,'initialized':init,'first':first,'last':last,'transitions':trans,'reason_counts':dict(reasons),'seed_reason_counts':dict(seed),'mean_writes':birth,'active_reason_action_counts':dict(active),'field_names':checks,'raw_current_count':sum(bool(x[1]) for x in rows)}
z=np.load(MET.parent/'error_curves.npz');result['curves']={'sha':sha(MET.parent/'error_curves.npz'),'schema':{k:list(z[k].shape) for k in z.files}}
# Preserve existing support, do not invent cross-method masks.
for k in z.files:
 a=z[k]
 if a.ndim==1 and np.issubdtype(a.dtype,np.number) and any(v in k.lower() for v in ['error','rmse']):
  finite=a[np.isfinite(a)];result['curves'][k]={'finite':len(finite),'max':float(finite.max()) if len(finite) else None}
p=OUT/'diagnostics/MH04_lifecycle_01.json';p.parent.mkdir(exist_ok=True);p.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n');print(p)
