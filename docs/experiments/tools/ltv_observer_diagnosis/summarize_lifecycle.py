"""Summarize selected-slot residence; survivors are right-censored, not deaths."""
import sys,json
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'ltv_value'))
from analyze import selected,cache
OUT=Path('/home/he/output/openvins_ltv_observer_diagnosis_20261003')
result={}
for seq in ['V1_01_easy','indoor_forward_3']:
 p=OUT/('lifecycle_'+seq)
 x=np.genfromtxt(p/'lifecycle.csv',delimiter=',',names=True,dtype=None,encoding='utf8')
 l=np.genfromtxt(p/'ltv.csv',delimiter=',',names=True,dtype=None,encoding='utf8')
 resets=sorted(set(l['core_reset'][l['camera_time']>=x['t'].min()]));assert set(resets)<= {'none'} ,resets
 a=cache(selected(seq));j=np.searchsorted(a['camera_time'],x['t']).clip(0,len(a['t'])-1)
 supported=(abs(a['camera_time'][j]-x['t'])<1e-6)&a['gt_pose_valid'][j]
 z={'core_reset':resets,'completed_residences':int(sum(x['event']=='death')),'births':int(sum(x['event']=='birth')),'right_censored':int(sum(x['event']=='birth')-sum(x['event']=='death'))}
 deaths=x['resident_age'][x['event']=='death']
 z['completed_lifetime_quantiles_s']=dict(zip(['p10','p50','p90'],map(float,np.quantile(deaths,[.1,.5,.9]))))
 for name,mask in [('all',np.ones(len(x),bool)),('GT_pose_support',supported)]:
  y=x[mask&(x['event']!='death')]
  z[name]={'selected_observations':len(y),'resident_age_quantiles_s':dict(zip(['p10','p50','p90'],map(float,np.quantile(y['resident_age'],[.1,.5,.9])))),'age_less_than_fraction':{str(t):float(np.mean(y['resident_age']<t)) for t in [.5,1,2,5]},'negative_depth_fraction':float(np.mean(y['ray_depth']<0)),'range_quantiles_m':list(map(float,np.quantile(y['range'],[.1,.5,.9]))),'projection_residual_median_m':float(np.median(y['projection_residual']))}
 result[seq]=z
(OUT/'lifecycle_summary.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
