"""Read-only fixed-ID chronology and whole-timeline original rate diagnostics."""
import json,hashlib,math,argparse
from pathlib import Path
TARGETS=(27537,29636)
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def load(p):return [json.loads(l) for l in Path(p).read_text().splitlines() if l]
def run(new,old,out):
 out=Path(out)
 if out.exists():raise FileExistsError(out)
 ns=load(new);os=load(old);lookup={r['camera_ns']:r for r in os}
 if len(lookup)!=len(os) or len(set(r['camera_ns'] for r in ns))!=len(ns):raise ValueError('duplicate integer identity')
 if set(lookup)!=set(r['camera_ns'] for r in ns):raise ValueError('timeline receipt mismatch')
 t0=next(r['imu_time'] for r in ns if r['initialized']);old_t0=next(r['imu_time'] for r in os if r['initialized'])
 if t0!=old_t0:raise ValueError('initialization origin mismatch')
 ids={str(i):dict(births=[],active_checks=[],retirements=[],corrected=[],guard_rejections=[],target=None) for i in TARGETS}
 timeline=[]
 for r in ns:
  t=r['target_imu_time']-t0;o=lookup[r['camera_ns']]
  if r['target_imu_time']!=o['target_imu_time']:raise ValueError('physical target mismatch')
  for i in TARGETS:
   q=ids[str(i)]
   for s in r.get('seeds',[]):
    if s['id']==i and (s['admitted'] or s['mean_written']):q['births'].append(dict(time=t,epoch=r['epoch'],source=s['source'],admitted=s['admitted'],mean_written=s['mean_written']))
   for d in r.get('active_consistency',[]):
    if d['feature_id']==i:q['active_checks'].append(dict(time=t,epoch=r['epoch'],diagnostic=d,retained=i in r['retained_ids'],corrected=i in r['corrected_ids']))
   for d in r.get('retirement_events',[]):
    if d['id']==i:q['retirements'].append(dict(time=t,event=d))
   if i in r['corrected_ids']:q['corrected'].append(t)
   if i in r.get('identity_guard_rejected_ids',[]):q['guard_rejections'].append(t)
   if r['camera_ns']=={27537:1413394964105760512,29636:1413394966255760384}[i]:
    q['target']=dict(time=t,current_cam0=i in r['current_cam0_ids'],retained=i in r['retained_ids'],corrected=i in r['corrected_ids'],guard_rejected=i in r.get('identity_guard_rejected_ids',[]),ready_G=r['ready_G'],ready_V=r['ready_V'],observed=r['observed_features'],mature=r['mature_features'])
  timeline.append(dict(camera_ns=r['camera_ns'],time=t,old={k:o[k] for k in ('correction_diagnostics_valid','velocity_correction_rate','gravity_correction_rate','prediction_angle_p95_rad')},new={k:r[k] for k in ('correction_diagnostics_valid','velocity_correction_rate','gravity_correction_rate','prediction_angle_p95_rad')}))
 def rates(which,common=False):
  valid=[r for r in timeline if r[which]['correction_diagnostics_valid'] and (not common or (r['old']['correction_diagnostics_valid'] and r['new']['correction_diagnostics_valid']))]
  result=dict(valid=len(valid),excluded=len(timeline)-len(valid),rates={})
  for key,limit in [('velocity_correction_rate',.5),('gravity_correction_rate',1.),('prediction_angle_p95_rad',.02)]:
   if any(not math.isfinite(r[which][key]) for r in valid):raise ValueError('valid scalar nonfinite')
   ranked=sorted(valid,key=lambda r:r[which][key],reverse=True)
   result['rates'][key]=dict(original_limit=limit,above_original_limit=sum(r[which][key]>limit for r in valid),maximum=ranked[0][which][key] if ranked else None,top10=[dict(camera_ns=r['camera_ns'],time=r['time'],value=r[which][key]) for r in ranked[:10]])
  return result
 result=dict(identity='C02_FULL_CONTINUOUS_FIXED_ID_REVIEW_NO_GT_SELECTION',origin=t0,sha={new:sha(new),old:sha(old),'script':sha(__file__)},ids=ids,full_timeline_rows=len(timeline),own_valid={m:rates(m) for m in ('old','new')},common_valid={m:rates(m,True) for m in ('old','new')},timeline=timeline)
 out.parent.mkdir(parents=True,exist_ok=True);out.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
 for i,q in ids.items():print(json.dumps(dict(id=i,births=q['births'],failures=[r for r in q['active_checks'] if not r['diagnostic']['consistency_pass']],retirements=q['retirements'],corrected=len(q['corrected']),guard_count=len(q['guard_rejections']),guard_first=q['guard_rejections'][:1],guard_last=q['guard_rejections'][-1:],target=q['target'])))
 print(json.dumps(result['own_valid']));print('COMMON',json.dumps(result['common_valid']))
if __name__=='__main__':
 p=argparse.ArgumentParser()
 for x in ('new','old','out'):p.add_argument('--'+x,required=True)
 run(**vars(p.parse_args()))
