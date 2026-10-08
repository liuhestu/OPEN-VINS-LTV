"""Offline reference association, after actual-core event results are immutable."""
import os
for k in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS'):os.environ[k]='1'
import argparse,json,hashlib
from pathlib import Path
import numpy as np
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def evaluate(results,curves,out):
 out=Path(out)
 if out.exists():raise FileExistsError(out)
 z=np.load(curves);keys=[int(x) for x in z['camera_ns']]
 if len(set(keys))!=len(keys):raise ValueError('duplicate reference integer camera identity')
 report=[]
 for path in results:
  r=json.loads(Path(path).read_text());k=keys.index(r['camera_ns']);refv=z['reference_v'][k];refg=z['reference_eta'][k]
  def error(v,g):
   v=np.asarray(v);g=np.asarray(g);rv=np.isfinite(refv).all() and np.isfinite(v).all();rg=np.isfinite(refg).all() and np.isfinite(g).all()
   return dict(v=float(np.linalg.norm(v-refv)) if rv else None,eta=float(np.linalg.norm(g-refg)) if rg else None,
    g_angle_deg=float(np.degrees(np.arccos(np.clip(g@refg/np.linalg.norm(g)/np.linalg.norm(refg),-1,1)))) if rg and np.linalg.norm(g)>0 and np.linalg.norm(refg)>0 else None,
    g_magnitude=float(abs(np.linalg.norm(g)-np.linalg.norm(refg))) if rg else None)
  phases={name:error(r[name+'_x'][-6:-3],r[name+'_x'][-3:]) for name in ('pre','old','new')}
  phases['OpenVINS']=error(z['OpenVINS_v'][k],z['OpenVINS_eta'][k])
  if max(abs(np.array(r['old_x'][-6:-3])-z['P_NEW_v'][k]))>1e-9 or max(abs(np.array(r['old_x'][-3:])-z['P_NEW_eta'][k]))>1e-9:raise ValueError('original reference-paired posterior mismatch')
  report.append(dict(camera_ns=r['camera_ns'],feature_id=r['feature_id'],reason=r['reason'],errors=phases,
    old_delta_v_norm=float(np.linalg.norm(np.array(r['old_x'][-6:-3])-r['pre_x'][-6:-3])),new_delta_v_norm=float(np.linalg.norm(np.array(r['new_x'][-6:-3])-r['pre_x'][-6:-3])),
    result_sha=sha(path),old_x_relative_error=r['old_x_relative_error'],old_P_relative_error=r['old_P_relative_error'],history_count=r['history_count'],history_span=r['history_span'],history_max_residual=r['history_max_residual'],holdout_residual=r['holdout_residual'],old_substeps=r['old_substeps'],new_substeps=r['new_substeps']))
 out.write_text(json.dumps(dict(scope='OFFLINE_GT_ASSOCIATION_OF_FIXED_TARGET_SINGLE_EVENT_INTERVENTIONS',curves_sha=sha(curves),evaluator_sha=sha(__file__),events=report),indent=2,allow_nan=False)+'\n')
 print(json.dumps(report))
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--results',nargs='+',required=True);p.add_argument('--curves',required=True);p.add_argument('--out',required=True);evaluate(**vars(p.parse_args()))
