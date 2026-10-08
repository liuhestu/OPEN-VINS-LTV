"""Registered286 shadow policy on frozen R3B core; not implementation acceptance."""
import os
for k in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS'):os.environ[k]='1'
import argparse,hashlib,json,math
from pathlib import Path
import numpy as np
IDENTITY='SHADOW_POLICY_ON_FROZEN_R3B_CORE_NOT_IMPLEMENTATION_ACCEPTANCE'
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def load(p):return [json.loads(x) for x in Path(p).read_text().splitlines() if x]
def keyed(rows,key):
 d={r[key]:r for r in rows}
 if len(d)!=len(rows):raise ValueError('duplicate identity '+key)
 return d
class Window:
 def __init__(self):self.clear();self.last=None
 def clear(self):self.entries=[];self.anchor=None;self.cursor=None;self.epoch=None;self.g=self.v=0
 def step(self,t,epoch,hard,phi,delta,angle,from_time):
  out=dict(time=t,ready_G=False,ready_V=False,complete=False,confirm_G=0,confirm_V=0,samples=0,clear_reason=None,budgets=None)
  if self.last is not None and t<=self.last:out['clear_reason']='REJECTED_DUPLICATE_NO_MUTATION';return out
  gap=self.last is not None and t-self.last>.20+1e-9;self.last=t
  if hard or gap:
   self.clear();out['clear_reason']=hard or 'physical_confirmation_gap';return out
  phi=np.asarray(phi);delta=np.asarray(delta)
  if phi.shape!=(6,6) or delta.shape!=(6,) or not np.isfinite(phi).all() or not np.isfinite(delta).all():
   self.clear();out['clear_reason']='transition_unavailable';return out
  if self.anchor is not None and (epoch!=self.epoch or from_time!=self.cursor):
   self.clear();out['clear_reason']='transition_identity';return out
  if self.anchor is None:self.anchor=t;self.cursor=t;self.epoch=epoch
  left=t-1.;self.entries=[e for e in self.entries if e['to']>left]
  if len(self.entries)==64:self.clear();out['clear_reason']='capacity';return out
  for e in self.entries:e['transported']=phi@e['transported']
  self.entries.append(dict(start=self.cursor,to=t,angle=angle,transported=delta.copy(),delta=delta.copy()));self.cursor=t
  cursor=max(left,self.anchor);weights=[];terms=[]
  for e in self.entries:
   a=max(e['start'],left);b=min(e['to'],t)
   if b>a:
    if a!=cursor:raise ValueError('uncovered interval')
    cursor=b;weights.append(b-a);terms.append((b-a)*e['angle'])
  complete=t-self.anchor>=1.;out['complete']=complete
  if complete and cursor!=t:raise ValueError('coverage endpoint')
  angle_mean=math.fsum(terms)/math.fsum(weights) if weights else 0.
  budgets={'angle_mean':angle_mean,'angle_pass':angle_mean<=.02}
  for name,sl,netcap,activitycap,impulsecap in [('G',slice(3,6),1.,2.,1.),('V',slice(0,3),.5,1.,.5)]:
   vectors=[e['transported'][sl] for e in self.entries];s=np.array([math.fsum(x[k] for x in vectors) for k in range(3)])
   net=float(np.linalg.norm(s));activity=math.fsum(float(np.linalg.norm(x)) for x in vectors);impulse=max(float(np.linalg.norm(e['delta'][sl])) for e in self.entries)
   budgets[name]=dict(net=net,activity=activity,impulse=impulse,net_pass=net<=netcap,activity_pass=activity<=activitycap,impulse_pass=impulse<=impulsecap)
  goodg=complete and budgets['angle_pass'] and all(budgets['G'][k] for k in ('net_pass','activity_pass','impulse_pass'))
  goodv=goodg and all(budgets['V'][k] for k in ('net_pass','activity_pass','impulse_pass'))
  self.g=min(self.g+1,20) if goodg else 0;self.v=min(self.v+1,20) if goodv else 0
  out.update(ready_G=self.g==20,ready_V=self.v==20,confirm_G=self.g,confirm_V=self.v,samples=len(self.entries),budgets=budgets,anchor=self.anchor)
  return out

def audit(features,diagnostics,transitions,contract,candidate,errors,out):
 out=Path(out)
 if out.exists():raise FileExistsError(out)
 registration=json.loads(Path(candidate).read_text())
 if sha(contract)!=registration['contract_sha256']:raise ValueError('frozen contract mismatch')
 fs=load(features);ds=keyed(load(diagnostics),'camera_ns');ps=keyed(load(transitions),'imu_time');keyed(fs,'camera_ns')
 t0=next(f['imu_time'] for f in fs if f['initialized']);policy=Window();rows=[];previous=None
 for f in fs:
  t=f['target_imu_time'];d=ds.get(f['camera_ns']);p=ps.get(t);reason=None
  if not f['raw_current'] or not f['observer_valid']:reason='not_current'
  elif previous is not None and (f['epoch']!=previous['epoch'] or f['bootstrap_count']!=previous['bootstrap_count']):reason='epoch_or_bootstrap'
  elif f['observed_features']<15 or f['mature_features']<15 or not f['pool_ready']:reason='pool'
  elif f['actual_corrections']<20:reason='actual_corrections'
  elif not f['correction_diagnostics_valid']:reason='diagnostics_invalid'
  elif not all(math.isfinite(f[k]) and f[k]>=0 for k in ('prediction_angle_p95_rad','velocity_correction_rate','gravity_correction_rate')):reason='nonfinite_diagnostics'
  elif f['prediction_angle_p95_rad']>.04:reason='current_angle_outer'
  elif f['v_body'] is None or f['eta_body'] is None or not np.isfinite(f['v_body']).all() or not np.isfinite(f['eta_body']).all():reason='state_nonfinite'
  elif np.linalg.norm(f['v_body'])>100 or np.linalg.norm(f['eta_body'])>50:reason='state_norm'
  elif not 8<=np.linalg.norm(f['eta_body'])<=11.5:reason='gravity_norm'
  elif d is None or p is None:reason='UNAVAILABLE_transition_or_delta'
  if d is not None and p is not None:
   if d['time']!=t or d['epoch']!=f['epoch'] or p['epoch']!=f['epoch']:raise ValueError('exact diagnostic identity mismatch')
  prevcursor=previous['cursor'] if previous is not None else t
  if reason is None and (previous is None or t-prevcursor!=d['dt']):reason='UNAVAILABLE_transition_cursor_identity'
  r=policy.step(t,f['epoch'],reason,p['Phi'] if p else None,d['delta_v']+d['delta_eta'] if d else None,f['prediction_angle_p95_rad'],prevcursor)
  r.update(camera_ns=f['camera_ns'],epoch=f['epoch'],relative_time=t-t0,original_ready_G=bool(f['ready_G']),original_ready_V=bool(f['ready_V']))
  rows.append(r);previous=f
 out.mkdir(parents=True)
 policyfile=out/'policy.jsonl';policyfile.write_text(''.join(json.dumps(r,allow_nan=False)+'\n' for r in rows))
 provenance={'identity':IDENTITY,'script_sha':sha(__file__),'inputs':{k:sha(v) for k,v in dict(features=features,diagnostics=diagnostics,transitions=transitions,contract=contract,candidate=candidate).items()},'policy_sha':sha(policyfile),'policy_saved_before_reading_errors':True}
 (out/'policy_provenance.json').write_text(json.dumps(provenance,indent=2)+'\n')
 # Reference is opened only after the full policy trace and its provenance exist.
 z=np.load(errors);index={int(k):j for j,k in enumerate(z['camera_ns'])}
 if len(index)!=len(z['camera_ns']):raise ValueError('duplicate reference key')
 linked=[]
 for r in rows:
  q=dict(r);j=index.get(r['camera_ns']);q['errors']=None
  if j is not None:
   if abs(float(z['time'][j])-r['time'])>1e-9:raise ValueError('exact reference pairing failed')
   q['errors']={k:float(z['P_NEW_error_'+k][j]) if np.isfinite(z['P_NEW_error_'+k][j]) else None for k in ('v','eta','g_angle_deg','g_magnitude')}
   q['reference_v_finite']=bool(np.isfinite(z['reference_v'][j]).all());q['reference_eta_finite']=bool(np.isfinite(z['reference_eta'][j]).all())
  linked.append(q)
 def summary(rs,branch):
  key='v' if branch=='V' else 'g_angle_deg';cut=1. if branch=='V' else 5.
  vals=[r['errors'][key] for r in rs if r['errors'] is not None and r['errors'][key] is not None and r['reference_'+('v' if branch=='V' else 'eta')+'_finite']]
  return dict(frames=len(rs),reference_available=len(vals),unavailable=len(rs)-len(vals),severe=sum(v>cut for v in vals),severe_fraction=sum(v>cut for v in vals)/len(vals) if vals else None,max_error=max(vals) if vals else None)
 report={'identity':IDENTITY,'policy_sha':sha(policyfile),'errors_sha':sha(errors),'newly_ready':{},'events':{},'all_rows':linked}
 for b in ('G','V'):
  new=[r for r in linked if r['ready_'+b] and not r['original_ready_'+b]]
  report['newly_ready'][b]=summary(new,b);report['newly_ready'][b]['camera_ns']=[r['camera_ns'] for r in new]
 for t in (76.75,78.90):report['events'][str(t)]=[r for r in linked if abs(r['relative_time']-t)<=.5+1e-6]
 (out/'error_association.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
 print(json.dumps({k:v for k,v in report.items() if k not in ('all_rows','events')}))
if __name__=='__main__':
 p=argparse.ArgumentParser()
 for key in ('features','diagnostics','transitions','contract','candidate','errors','out'):p.add_argument('--'+key,required=True)
 audit(**vars(p.parse_args()))
