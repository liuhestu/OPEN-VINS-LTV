"""Offline exact-camera pre/post unit-ray diagnostics; no GT or policy changes."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import statistics
import numpy as np

WINDOWS=[('prior_window_5_80_13_95',5.8,13.95),('prior_window_72_15_81_70',72.15,81.70),('whole_logged_timeline',-math.inf,math.inf)]
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def load(p):return [json.loads(l) for l in Path(p).read_text().splitlines() if l.strip()]
def keyed(rows):
 d={r['camera_ns']:r for r in rows}
 if len(d)!=len(rows):raise ValueError('Duplicate integer camera identity')
 return d
def stats(xs):return {'count':len(xs),'minimum':min(xs) if xs else None,'median':statistics.median(xs) if xs else None,'maximum':max(xs) if xs else None}
def ray(ell,center,unit):
 v=np.asarray(ell)-center;norm=float(np.linalg.norm(v));dot=float(v@unit)
 angle=float(np.arccos(np.clip(v@unit/norm,-1,1))) if norm>0 else None
 return dict(distance=norm,signed_ray=dot,negative=dot<0,zero_ray=dot==0,near_center=norm<.1,angle_rad=angle)

def analyze(diagnostics,geometry,features,out):
 out=Path(out)
 if out.exists():raise FileExistsError(out)
 ds=load(diagnostics);gs=keyed(load(geometry));fs=keyed(load(features))
 t0=next(r['imu_time'] for r in fs.values() if r['initialized'])
 packets=[];maximum_pre_difference=0.
 for d in ds:
  key=d['camera_ns'];g=gs[key];f=fs[key]
  if d['epoch']!=g['epoch'] or d['epoch']!=f['epoch'] or abs(d['time']-g['time'])>1e-9 or abs(d['time']-f['imu_time'])>1e-9:raise ValueError('Paired time/epoch mismatch')
  R=np.asarray(g['calibration']['R_BC']).reshape(3,3);center=np.asarray(g['calibration']['p_BC'])
  obs={x['id']:x for x in g['managed_observations'] if x['camera']==0}
  points=[]
  for p in d['points']:
   unit=R@np.asarray(obs[p['id']]['bearing']);unit/=np.linalg.norm(unit)
   pre=ray(p['pre_landmark'],center,unit);post=ray(p['post_landmark'],center,unit)
   if p['in_scalar_p95']:
    difference=abs(pre['angle_rad']-p['angle_rad']);maximum_pre_difference=max(maximum_pre_difference,difference)
    if difference>1e-10:raise ValueError('Reconstructed pre angle mismatch')
   delta=None if pre['angle_rad'] is None or post['angle_rad'] is None else post['angle_rad']-pre['angle_rad']
   points.append(dict(id=p['id'],pre_existing=p['pre_existing'],in_scalar_p95=p['in_scalar_p95'],pre=pre,post=post,
                      post_minus_pre_angle=delta,change='UNAVAILABLE' if delta is None else 'IMPROVED' if delta<0 else 'WORSENED' if delta>0 else 'UNCHANGED'))
  s=d['scalars']
  packets.append(dict(camera_ns=key,epoch=d['epoch'],relative_time=f['imu_time']-t0,time=f['imu_time'],valid=d['valid'],
                      prediction_p95=s['prediction_angle_p95_rad'],v_rate=s['velocity_correction_rate'],eta_rate=s['gravity_correction_rate'],points=points))
 report={'t0_initialized_imu_time':t0,'original_limits':{'angle_rad':.02,'near_center_m':.1},'maximum_pre_angle_difference_from276':maximum_pre_difference,
         'provenance':{k:sha(v) for k,v in dict(diagnostics=diagnostics,geometry=geometry,features=features,script=__file__).items()},
         'scope':'Unit-ray angles/distance/signed measured-ray projection, NOT core projection innovation. No GT. Birth premean is cache hook contract, not independently validated by P95. Signed shared-rate associations are not per-point causal attribution.', 'windows':{}}
 for name,start,end in WINDOWS:
  rows=[p for p in packets if start-1e-6<=p['relative_time']<=end+1e-6]
  points=[x for p in rows for x in p['points']]
  prehigh=[x for x in points if x['pre']['angle_rad'] is not None and x['pre']['angle_rad']>.02]
  includedhigh=[x for x in prehigh if x['in_scalar_p95']]
  def describe(xs):
   return dict(points=len(xs),improved=sum(x['change']=='IMPROVED' for x in xs),worsened=sum(x['change']=='WORSENED' for x in xs),unchanged=sum(x['change']=='UNCHANGED' for x in xs),
               post_at_or_below_original_angle=sum(x['post']['angle_rad'] is not None and x['post']['angle_rad']<=.02 for x in xs),
               pre_angle=stats([x['pre']['angle_rad'] for x in xs if x['pre']['angle_rad'] is not None]),post_angle=stats([x['post']['angle_rad'] for x in xs if x['post']['angle_rad'] is not None]),
               pre_near=sum(x['pre']['near_center'] for x in xs),post_near=sum(x['post']['near_center'] for x in xs),pre_negative=sum(x['pre']['negative'] for x in xs),post_negative=sum(x['post']['negative'] for x in xs))
  exceptional=[]
  for p in rows:
   if not(p['valid'] and p['prediction_p95']<=.02 and p['v_rate']>.5):continue
   angles=[x for x in p['points'] if x['pre']['angle_rad'] is not None]
   largest=max(angles,key=lambda x:x['pre']['angle_rad']) if angles else None
   exceptional.append(dict(relative_time=p['relative_time'],prediction_p95=p['prediction_p95'],v_rate=p['v_rate'],eta_rate=p['eta_rate'],
                           above_count=sum(x['pre']['angle_rad']>.02 for x in angles),near_excluded=[x['id'] for x in p['points'] if x['pre_existing'] and not x['in_scalar_p95'] and x['pre']['near_center']],
                           birth_excluded=[x['id'] for x in p['points'] if not x['pre_existing']],largest_pre_angle_point=largest))
  report['windows'][name]=dict(packets=len(rows),all_points=describe(points),pre_high_all=describe(prehigh),pre_high_in_original_p95=describe(includedhigh),
                               normal_p95_large_v_rate_packets=len(exceptional),normal_p95_large_v_rate_with_any_high_angle=sum(x['above_count']>0 for x in exceptional),
                               normal_p95_large_v_rate_with_excluded_near=sum(bool(x['near_excluded']) for x in exceptional),
                               exceptional_packets=exceptional,packets_detail=rows)
 out.mkdir(parents=True);(out/'summary.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
 print(json.dumps({k:{a:b for a,b in v.items() if a not in ['packets_detail','exceptional_packets']} for k,v in report['windows'].items()}))

if __name__=='__main__':
 p=argparse.ArgumentParser(description=__doc__)
 for name in ['diagnostics','geometry','features','out']:p.add_argument('--'+name,required=True)
 a=p.parse_args();analyze(**vars(a))
