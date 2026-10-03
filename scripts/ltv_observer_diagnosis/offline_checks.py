"""Read-only comparison: old VINS Passive logs and frozen OpenVINS Passive arrays."""
from pathlib import Path
import sys,json,itertools,hashlib
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'ltv_value'))
from common import EUROC,SEQUENCES,OUT as VALUE_OUT,read
from analyze import selected,cache
from reference import reference,angle
OUT=Path('/home/he/output/openvins_ltv_observer_diagnosis_20261003')
def rms(x):return float(np.sqrt(np.mean(np.asarray(x)**2)))
def run():
 evidence={};vins={};sources={}
 for seq in SEQUENCES:
  run=selected(seq);a=cache(run);mask=np.isfinite(a['ltv_V_error']);v=a['ltv_v'][mask];truth=a['gt_v_body'][mask]
  norm_bound=rms(np.linalg.norm(v,axis=1)-np.linalg.norm(truth,axis=1));best=(float('inf'),None)
  for order in itertools.permutations(range(3)):
   for signs in itertools.product([-1,1],repeat=3):
    e=rms(np.linalg.norm(v[:,order]*signs-truth,axis=1))
    if e<best[0]:best=(e,{'order':order,'signs':signs})
  # Deliberately broad coarse timestamp diagnostic. Fixed support across all lags.
  shifts=np.arange(-.1,.1001,.01);refs=[]
  for shift in shifts:refs.append(np.column_stack([np.interp(a['t']+shift,a['t'],a['gt_v_body'][:,i],left=np.nan,right=np.nan) for i in range(3)]))
  support=mask.copy()
  for ref in refs:support &= np.isfinite(ref).all(axis=1)
  errors=[rms(np.linalg.norm(a['ltv_v'][support]-ref[support],axis=1)) for ref in refs]
  evidence[seq]={'run':run['id'],'n':int(mask.sum()),'prior_V_RMSE':rms(a['prior_V_error'][mask]),'LTV_V_RMSE':rms(a['ltv_V_error'][mask]),'orthogonal_transform_lower_bound':norm_bound,'best_signed_permutation_RMSE':best[0],'best_signed_permutation':best[1],'lag_grid_seconds':shifts.tolist(),'lag_fixed_support':int(support.sum()),'lag_RMSE':errors,'best_lag_seconds':float(shifts[np.argmin(errors)]),'best_lag_RMSE':min(errors),'zero_lag_RMSE_same_support':errors[10]}
  if seq not in EUROC:continue
  path=Path('/home/he/output/ltv_euroc_final_unified')/seq/'baseline/ltv_debug.csv'
  x=np.genfromtxt(path,delimiter=',',names=True,dtype=None,encoding='utf8');x=x[x['frame_timestamp']>0]
  t=x['imu_timestamp'];assert np.all(np.diff(t)>0)
  src_v=np.column_stack([x['velocity_body_'+k] for k in ['x','y','z']]);src_prior=np.column_stack([x['vins_velocity_body_'+k] for k in ['x','y','z']]);src_g=np.column_stack([x['gravity_body_'+k] for k in ['x','y','z']])
  j=np.searchsorted(t,a['t']).clip(0,len(t)-1);left=(j>0)&(np.abs(t[np.maximum(j-1,0)]-a['t'])<np.abs(t[j]-a['t']));j[left]-=1
  common=np.abs(t[j]-a['t'])<=1e-6
  good=common&mask&(x['valid'][j]>0)&(x['velocity_valid'][j]>0)
  ggood=common&np.isfinite(a['ltv_G_error'])&(x['valid'][j]>0)&(x['gravity_valid'][j]>0)
  source_G=angle(src_g[j[ggood]],a['gt_g_body'][ggood])
  vins[seq]={'source_log':str(path),'common_V_samples':int(good.sum()),'common_G_samples':int(ggood.sum()),'VINS_LTV_V_RMSE':rms(np.linalg.norm(src_v[j[good]]-a['gt_v_body'][good],axis=1)),'VINS_prior_V_RMSE':rms(np.linalg.norm(src_prior[j[good]]-a['gt_v_body'][good],axis=1)),'OpenVINS_LTV_V_RMSE':rms(a['ltv_V_error'][good]),'OpenVINS_prior_V_RMSE':rms(a['prior_V_error'][good]),'VINS_LTV_G_RMSE_deg':rms(source_G),'OpenVINS_LTV_G_RMSE_deg':rms(a['ltv_G_error'][ggood]),'max_common_time_delta':float(np.max(np.abs(t[j[common]]-a['t'][common])))}
  sources[str(path)]=hashlib.sha256(path.read_bytes()).hexdigest()
 result={'scope':'Diagnostic fits only; no calibration/parameter change or closed-loop use. VINS historical baseline differs in frontend/calibration/IMU integration; common timestamp support, same GT body reference, not a controlled estimator A/B. Lag scan interpolates the already computed GT body reference on its event grid.','frame_time_checks':evidence,'vins_common_support':vins,'source_sha256':sources}
 (OUT/'offline_checks.json').write_text(json.dumps(result,indent=2,allow_nan=False));print(json.dumps(vins,indent=2))
if __name__=='__main__':run()
