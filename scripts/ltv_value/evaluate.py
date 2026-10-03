"""Prospective SE3/no-scale trajectory metrics on fixed baseline support."""
import argparse
import numpy as np
from scipy.spatial.transform import Rotation
from common import *
from reference import load_gt,reference
from analyze import guard,selected

def nearest(t,u):
 hi=np.searchsorted(t,u).clip(0,len(t)-1);lo=np.maximum(0,hi-1)
 return np.where(np.abs(t[lo]-u)<=np.abs(t[hi]-u),lo,hi)
def align(gt,p):
 a=p-p.mean(axis=0);b=gt-gt.mean(axis=0);u,_,vt=np.linalg.svd(a.T@b);d=np.eye(3);d[2,2]=np.linalg.det(vt.T@u.T)
 R=vt.T@d@u.T;return R,gt.mean(axis=0)-R@p.mean(axis=0)
def trajectory(run):
 p=Path(run['path'])/'trajectory.csv'
 if not p.exists():return np.empty((0,17))
 x=np.loadtxt(p,delimiter=',',skiprows=1,ndmin=2)
 if not x.size:return np.empty((0,17))
 if x.shape[1]!=17 or not np.isfinite(x).all() or np.any(np.diff(x[:,0])<=0):raise ValueError('bad trajectory')
 return x

def evaluate(run):
 seq=run['sequence'];guard(seq);base=trajectory(selected(seq,'B'));x=trajectory(run);inp=read(OUT/'inputs.json')[seq]
 if not len(base):return {'status':'FAIL','reason':'baseline_initialization_failure'}
 camera=np.loadtxt(Path(inp['replay_root'])/'cam0/data.csv',delimiter=',',dtype=str,comments='#')[:,0].astype(np.int64)*1e-9+inp['offset']
 sensor=camera[camera>=base[0,0]-1e-6];t,p,q,_=load_gt(seq)
 gi=nearest(t,sensor);inside=(sensor>=t[0])&(sensor<=t[-1])&(np.abs(t[gi]-sensor)<=.02)
 support=sensor[inside];gt=p[gi[inside]];gtq=q[gi[inside]]
 result={'sequence':seq,'mode':run['mode'],'candidate':run['candidate'],'path':run['path'],'status':'FAIL','scale':1.,'target_sensor_rows':len(sensor),'target_gt_rows':len(support),'gt_time_fraction':float(inside.mean()),'output_rows':len(x),'gt_sha':run['gt_sha'],'evaluator_sha':sha(__file__),'protocol':'nearest_GT_20ms; fixed_B_support; SE3_no_scale; output_match_1us; >=99pct'}
 if not len(x) or len(support)<3:result['reason']='missing_output_or_reference';return result
 j=nearest(x[:,0],support);available=np.abs(x[j,0]-support)<=1e-6;js=nearest(x[:,0],sensor);sensor_ok=np.abs(x[js,0]-sensor)<=1e-6
 result['coverage']=float(available.mean());result['sensor_coverage']=float(sensor_ok.mean())
 result['initialization_delta_s']=float(x[0,0]-base[0,0]);result['missing_rows']=int((~available).sum())
 gaps=np.flatnonzero(np.diff(np.r_[False,~sensor_ok,False]));result['longest_missing_packets']=int(max(gaps[1::2]-gaps[::2],default=0))
 if result['coverage']<.99 or result['sensor_coverage']<.99 or abs(result['initialization_delta_s'])>1e-6:result['reason']='coverage_or_initialization_mismatch';return result
 est=x[j[available]];gt=gt[available];gtq=gtq[available];times=support[available]
 R,translation=align(gt,est[:,5:8]);aligned=est[:,5:8]@R.T+translation;error=aligned-gt
 result.update(status='VALID',ate_rmse_m=float(np.sqrt(np.mean(np.sum(error**2,axis=1)))),alignment_R=R.tolist(),alignment_t=translation.tolist(),samples=len(gt))
 rg=Rotation.from_quat(gtq);re=Rotation.from_quat(est[:,1:5]);rel=rg.inv()*(Rotation.from_matrix(R)*re)
 result['rotation_rmse_deg']=float(np.degrees(np.sqrt(np.mean(rel.magnitude()**2))))
 reference_v=reference(seq,times)['v_world'];ve=est[:,8:11]@R.T-reference_v
 result['velocity_rmse_mps']=float(np.sqrt(np.nanmean(np.sum(ve**2,axis=1))))
 result['rpe']={}
 for dt in [1.,5.]:
  end=nearest(times,times+dt);ok=np.abs(times[end]-(times+dt))<=.025
  a=np.flatnonzero(ok);b=end[ok]
  delta_gt=rg[a].inv().apply(gt[b]-gt[a]);delta_est=re[a].inv().apply(est[b,5:8]-est[a,5:8]);delta_rot=(rg[a].inv()*rg[b]).inv()*(re[a].inv()*re[b])
  result['rpe'][str(dt)]={'pairs':len(a),'translation_rmse_m':float(np.sqrt(np.mean(np.sum((delta_est-delta_gt)**2,axis=1)))) if len(a) else None,'rotation_rmse_deg':float(np.degrees(np.sqrt(np.mean(delta_rot.magnitude()**2)))) if len(a) else None}
 # Separately expose gross localization divergence; numerical validity is not success.
 extent=float(np.linalg.norm(np.ptp(gt,axis=0)));path_length=float(np.sum(np.linalg.norm(np.diff(gt,axis=0),axis=1)))
 result['gt_extent_m']=extent;result['gt_length_m']=path_length
 result['localization_status']='FAIL_GROSS_DRIFT' if result['ate_rmse_m']>max(5.,extent) else 'VALID'
 result['gross_failure_rule']='ATE > max(5m, GT bounding-box diagonal); all raw metrics retained'
 write(Path(run['path'])/'metrics.json',result);return result

def self_test():
 rng=np.random.default_rng(42);p=rng.normal(size=(100,3));R=Rotation.from_rotvec([.2,-.3,.1]).as_matrix();gt=p@R.T+[1,2,3]
 r,t=align(gt,p);np.testing.assert_allclose(r,R,atol=1e-12);np.testing.assert_allclose(t,[1,2,3],atol=1e-12)
 r,t=align(2*gt,p);assert np.sqrt(np.mean(np.sum((p@r.T+t-2*gt)**2,axis=1)))>.1
 assert nearest(np.array([0.,.01,.02]),np.array([.005,.019])).tolist()==[0,2]
 print('SE3/no-scale/nearest PASS')
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--self-test',action='store_true');a=p.parse_args()
 if a.self_test:self_test()
