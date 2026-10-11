"""Fabricated three-frame ABI/schema fixture; no full synthetic run."""
import os
for k in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS'):os.environ[k]='1'
import argparse,ctypes as C,hashlib,json,sys
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'ltv_passive_hardening'))
import synthetic_runner as runner
def test(library,out):
 data=dict(camera_times=np.array([0.,.05,.1]),camera_ns=np.array([0,50000000,100000000],dtype=np.int64),observation_offsets=np.zeros(4,np.int64),
 feature_ids=np.zeros(0,np.int64),camera_ids=np.zeros(0,np.int32),actual_ns=np.zeros(0,np.int64),bearings=np.zeros((0,3)),
 prior_R_WB=np.repeat(np.eye(3)[None],3,axis=0),prior_p_WB=np.zeros((3,3)),R_BC=np.repeat(np.eye(3)[None],2,axis=0),p_BC=np.array([[0.,0,0],[.2,0,0]]),
 pose_model_std=np.zeros(6),pose_model_jitter=np.zeros(6),pose_correlation_seconds=np.array(1.))
 result=[]
 for mode in range(6):
  a=runner.Adapter(library,mode,'HYBRID',.0008726646259971648)
  if mode in (4,5):
   imu_tick=-1
   a.imu(-.005,[0,0,0,0,0,-9.81])
   for k,t in enumerate(data['camera_times']):
    while imu_tick<int(round(t/.005)):
     imu_tick+=1;a.imu(imu_tick*.005,[0,0,0,0,0,-9.81])
    r=a.frame(data,k)
    assert r['ready_soft_grace_enabled']==1
    if mode==4:assert 'active_consistency_schema' not in r
    else:assert r['active_consistency_schema']=='ACTIVE_CONSISTENCY_V1' and r['consistency_active_count']==0
   result.append(dict(mode=mode,schema=r.get('active_consistency_schema'),grace=r['ready_soft_grace_enabled']))
  a.lib.ph_destroy(a.handle)
 assert not a.lib.ph_create(6,2,.001)
 Path(out).write_text(json.dumps(dict(status='PASS',wrapper_sha=hashlib.sha256(Path(library).read_bytes()).hexdigest(),modes_0_through_5_construct=True,frames=result),indent=2)+'\n')
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--library',required=True);p.add_argument('--out',required=True);test(**vars(p.parse_args()))
