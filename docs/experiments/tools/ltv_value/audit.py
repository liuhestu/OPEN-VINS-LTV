"""Engineering audit, independent of GT errors; allowed before validation unsealing."""
import argparse,gzip,json
import numpy as np
from common import *
def audit(run):
 p=Path(run['path']);r={'run':run['id'],'sequence':run['sequence'],'mode':run['mode'],'status':'PASS','checks':{}}
 if run['status']!='complete':return dict(r,status='FAIL',reason=run.get('failure','execution_failure'))
 a=np.loadtxt(p/'trajectory.csv',delimiter=',',skiprows=1,ndmin=2)
 if not a.size: return dict(r,status='FAIL',reason='no_initialization')
 if not np.isfinite(a).all():raise ValueError('nonfinite native output')
 r['checks']['trajectory_finite']=True;r['output_rows']=len(a)
 r['checks']['complete_sensors']=run['short'] or (run['replay']['complete'] and run['replay']['camera_packets']==run['replay']['input_camera_packets'] and run['replay']['imu_consumed']==run['replay']['input_imu_samples'])
 errors=[];events=set();times=[];rows=[];timing=[]
 if run['diagnostics']:
  with gzip.open(p/'value.jsonl.gz','rt') as f:
   for line in f:
    x=json.loads(line);key=(x['epoch'],x['version'],x['camera_time'])
    if key in events:raise ValueError('duplicate diagnostic event')
    events.add(key);times.append(x['imu_time']);rows.append(x['visual_rows'])
    post=np.asarray(x['posterior']).ravel();pred=np.asarray(x['shadow_visual' if run['mode'] in ['B','P'] else 'shadow_'+run['mode']]).ravel()
    if not np.isfinite(post).all() or not np.isfinite(pred).all():raise ValueError('nonfinite shadow')
    errors.append(float(np.linalg.norm(post-pred)/max(1,np.linalg.norm(post))))
    timing.append([x['observer_seconds'],x['msckf_seconds_without_diagnostics'],x['diagnostic_compute_seconds']])
    if x['P'] and np.shape(x['P'])[0]!=np.shape(x['P'])[1]:raise ValueError('invalid matrix export')
  if errors and max(errors)>1e-9:raise ValueError('actual native/shadow mismatch')
  r['checks']['shadow_actual_max_relative_error']=max(errors,default=0);r['diagnostic_events']=len(events);r['visual_update_events']=sum(x>0 for x in rows);r['timing_sums']=np.sum(timing,axis=0).tolist()
 if run['mode']!='B':
  import csv
  logs=list(csv.DictReader((p/'ltv.csv').open()))
  attempted=[x for x in logs if int(x['available'])]
  if any(int(x['EKF_calls'])>1 for x in logs):raise ValueError('multiple updates')
  token=[(x['epoch'],x['attempt_id']) for x in logs if int(x['used_once'])]
  if len(token)!=len(set(token)):raise ValueError('duplicate consumed token')
  r['counts']={name:sum(int(x[name]) for x in logs) for name in ['accepted_G','accepted_V','EKF_calls']}
  from collections import Counter
  r['reasons']={name:dict(Counter(x[name] for x in logs)) for name in ['reason','G_reason','V_reason','submit_reason','core_reset']}
  for field in ['G_nis','V_nis','G_residual','V_residual','G_gain_norm','V_gain_norm','G_update_norm','V_update_norm','P_symmetry','P_min_eigenvalue','PL_min_eigenvalue']:
   v=np.array([float(x[field]) for x in logs]);v=v[np.isfinite(v)]
   r[field]={'median':float(np.median(v)),'p95':float(np.quantile(v,.95)),'max':float(np.max(v))} if len(v) else None
 write(p/'engineering_audit.json',r);return r
if __name__=='__main__':
 for run in latest_runs().values():
  if run['status']=='complete' and not (Path(run['path'])/'engineering_audit.json').exists():
   print(run['id'],audit(run)['status'],flush=True)
