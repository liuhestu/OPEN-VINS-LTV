"""Fixed causal probes of unchanged production core, not dataset parameter tuning."""
from pathlib import Path
import subprocess,json,time,hashlib
import numpy as np
OUT=Path('/home/he/output/openvins_ltv_observer_diagnosis_20261003')
CASES=[('stable',.005,0,1,1),('life_0.5',.005,.5,1,1),('life_1',.005,1,1,1),('life_2',.005,2,1,1),('life_5',.005,5,1,1),('fast_stable',.005,0,3,1),('fast_life_1',.005,1,3,1),('static',.005,0,1,0),('fine_stable',.001,0,1,1),('fine_life_1',.001,1,1,1)]
def run():
 results={}
 for name,dt,life,scale,moving in CASES:
  p=OUT/'synthetic'/name;p.mkdir(parents=True,exist_ok=True)
  cmd=[str(OUT/'core_probe'),str(p/'trace.csv'),'60',str(dt),str(life),str(scale),str(moving)]
  if not (p/'command.json').exists():
   begin=time.monotonic();print('START',name,flush=True)
   with (p/'stdout.log').open('w') as so,(p/'stderr.log').open('w') as se:r=subprocess.run(cmd,stdout=so,stderr=se)
   (p/'command.json').write_text(json.dumps({'command':cmd,'exit_code':r.returncode,'seconds':time.monotonic()-begin,'binary_sha':hashlib.sha256((OUT/'core_probe').read_bytes()).hexdigest()},indent=2))
  if json.loads((p/'command.json').read_text())['exit_code']:raise RuntimeError('probe failed '+name)
  x=np.genfromtxt(p/'trace.csv',delimiter=',',names=True,dtype=None,encoding='utf8');mask=x['t']>=30
  z={k:float(np.sqrt(np.mean(x[k][mask]**2))) for k in ['v_error','g_error_deg','speed_true','speed_ltv']}
  z.update(observed_mean=float(x['observed'][mask].mean()),valid_fraction=float(x['valid'][mask].mean()),max_propagation_error=float(x['propagation_error'].max()),max_measurement_residual=float(x['model_residual'].max()),resets=sorted(set(x['reset'])),tail_start=30,tail_end=60,dt=dt,track_lifetime=life,scale=scale)
  results[name]=z;(OUT/'synthetic_results.json').write_text(json.dumps(results,indent=2));print('END',name,z,flush=True)
if __name__=='__main__':run()
