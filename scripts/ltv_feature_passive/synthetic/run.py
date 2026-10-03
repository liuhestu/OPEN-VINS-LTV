"""Execute the actual C++ core+pipeline with frozen sensor/prior input only.

No truth imports and no labels file reads. The parent shared scheduler owns the
full-run reservation. Input covariance preprocessing preserves all pose blocks.
"""
import os
for name in ['OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS']:
    os.environ[name]='1'
import argparse
import ctypes as C
import hashlib
import json
from pathlib import Path
import time
import numpy as np
METHODS={'OLD':0,'SEL':1,'SEED_TEMPORAL':2,'SEED_STEREO':3,'SEED_HYBRID':4}
DP=C.POINTER(C.c_double);IP=C.POINTER(C.c_int)

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def dptr(a):return np.ascontiguousarray(a,dtype=np.float64).ctypes.data_as(DP)
def iptr(a):return np.ascontiguousarray(a,dtype=np.int32).ctypes.data_as(IP)
def skew(p):
    x,y,z=p
    return np.array([[0,-z,y],[z,0,-x],[-y,x,0.]])

def pose_covariance(data,start,stop):
    """Known synthetic joint prior model, transformed without truth access."""
    times=data['camera_times'][start:stop];R=data['prior_R_WB'][start:stop]
    Rbc=data['R_BC'][0];pc=data['p_BC'][0];n=len(times)
    cov=np.zeros((6*n,6*n));std=data['pose_model_std'];jitter=data['pose_model_jitter']
    transforms=[]
    for r in R:
        j=np.zeros((6,6));j[:3,3:]=Rbc;j[3:,:3]=np.eye(3);j[3:,3:]=r@skew(pc)@Rbc
        transforms.append(j)
    for j in range(n):
        for k in range(n):
            scale=np.exp(-abs(times[j]-times[k])/float(data['pose_correlation_seconds']))
            raw=np.diag(std**2*scale+(jitter**2 if j==k else 0))
            cov[6*j:6*j+6,6*k:6*k+6]=transforms[j]@raw@transforms[k].T
    return cov

class Core:
    def __init__(self,library,method,risk,sigma):
        self.lib=C.CDLL(str(library));l=self.lib
        l.fp_create.argtypes=[C.c_int,C.c_double,C.c_double];l.fp_create.restype=C.c_void_p
        l.fp_destroy.argtypes=[C.c_void_p]
        l.fp_predict.argtypes=[C.c_void_p,C.c_double,DP,DP];l.fp_predict.restype=C.c_int
        l.fp_frame.argtypes=[C.c_void_p,C.c_double,C.c_int,C.c_int,IP,DP,DP,C.c_int,DP,DP,DP,DP,DP,DP]
        l.fp_frame.restype=C.c_int
        l.fp_frame_json.argtypes=[C.c_void_p];l.fp_frame_json.restype=C.c_char_p
        l.fp_error.argtypes=[C.c_void_p];l.fp_error.restype=C.c_char_p
        l.fp_read.argtypes=[C.c_void_p,DP,DP,IP,C.c_int];l.fp_read.restype=C.c_int
        self.handle=l.fp_create(METHODS[method],risk,sigma)
        if not self.handle:raise RuntimeError('C++ create rejected configuration')
    def predict(self,dt,imu):
        if not self.lib.fp_predict(self.handle,dt,dptr(imu[:3]),dptr(imu[3:])):raise RuntimeError('C++ propagation failure')
    def frame(self,data,k):
        start=max(0,k-20);cov=pose_covariance(data,start,k+1)
        ok=self.lib.fp_frame(self.handle,float(data['camera_times'][k]),k,len(data['feature_ids'][k]),
            iptr(data['feature_ids'][k]),dptr(data['bearings_left'][k]),dptr(data['bearings_right'][k]),k+1-start,
            dptr(data['camera_times'][start:k+1]),dptr(data['prior_R_WB'][start:k+1]),dptr(data['prior_p_WB'][start:k+1]),
            dptr(cov),dptr(data['R_BC']),dptr(data['p_BC']))
        if not ok:raise RuntimeError(self.lib.fp_error(self.handle).decode())
        return json.loads(self.lib.fp_frame_json(self.handle).decode())
    def state(self,with_P=False):
        x=np.full(96,np.nan);P=np.full((96,96),np.nan);ids=np.full(30,-1,dtype=np.int32)
        flat=np.zeros(96*96)
        d=self.lib.fp_read(self.handle,dptr(x),dptr(flat),iptr(ids),int(with_P))
        if d>96 or d<6 or (d-6)%3:raise RuntimeError('Invalid state dimension')
        if with_P:P[:d,:d]=flat[:d*d].reshape(d,d)
        if not np.isfinite(x[:d]).all():raise RuntimeError('Nonfinite C++ state')
        return x,P,ids,d
    def close(self):
        if self.handle:self.lib.fp_destroy(self.handle);self.handle=None

def run(input_dir,out,library,method,max_risk=.05):
    input_dir=Path(input_dir);out=Path(out);library=Path(library).resolve()
    identity=json.loads((input_dir/'identity.json').read_text())
    if sha(input_dir/'inputs.npz')!=identity['input_sha']:raise RuntimeError('Frozen input SHA mismatch')
    out.mkdir(parents=True,exist_ok=False)
    data=np.load(input_dir/'inputs.npz')
    metadata={'method':method,'max_relative_risk':max_risk,'input_identity':identity,'library_sha':sha(library),
              'runner_sha':sha(__file__),'start_wall_time':time.time(),'status':'RUNNING',
              'readiness':'core valid branch AND >=15 current mature managed points; OLD core branch validity',
              'pose_covariance':'full synthetic OU camera model mapped to unique body poses; fixed extrinsics, independent bearing approximation',
              'GT_estimator_access':False}
    (out/'run.json').write_text(json.dumps(metadata,indent=2))
    core=Core(library,method,max_risk,float(data['bearing_sigma_rad']))
    xs=[];ids_out=[];dims=[];P_times=[];P_tags=[];Ps=[];P_x=[];P_ids=[];P_dims=[];events=[]
    def snapshot(t,tag):
        x,P,ids,d=core.state(True);P_times.append(t);P_tags.append(tag);Ps.append(P);P_x.append(x);P_ids.append(ids);P_dims.append(d)
    try:
        for tick,t in enumerate(data['imu_times']):
            if tick:core.predict(float(data['imu_times'][tick]-data['imu_times'][tick-1]),data['imu'][tick-1])
            if tick%10==0:
                snapshot(float(t),'before_camera')
                event=core.frame(data,tick//10);events.append(event)
                snapshot(float(t),'after_camera')
            elif tick%200==0:snapshot(float(t),'second')
            x,_,ids,d=core.state(False);xs.append(x);ids_out.append(ids);dims.append(d)
        metadata['status']='COMPLETED_ESTIMATION'
    except Exception as error:
        metadata['status']='FAILED';metadata['error']=repr(error)
        try:snapshot(float(data['imu_times'][len(xs)]),'exception')
        except Exception:pass
        raise
    finally:
        core.close()
        np.savez_compressed(out/'states.npz',t=data['imu_times'][:len(xs)],x=xs,ids=ids_out,dimension=dims)
        np.savez_compressed(out/'matrices.npz',t=P_times,tags=P_tags,x=P_x,ids=P_ids,dimension=P_dims,P=Ps)
        (out/'events.jsonl').write_text(''.join(json.dumps(e,sort_keys=True)+'\n' for e in events))
        metadata['elapsed_seconds']=time.time()-metadata['start_wall_time'];metadata['samples']=len(xs);metadata['camera_events']=len(events)
        metadata['final_state_sha']=sha(out/'states.npz');metadata['full_input_consumed']=len(xs)==len(data['imu_times'])
        (out/'run.json').write_text(json.dumps(metadata,indent=2))
    return metadata

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--input',type=Path,required=True);p.add_argument('--out',type=Path,required=True)
    p.add_argument('--library',type=Path,required=True);p.add_argument('--method',choices=METHODS,required=True);p.add_argument('--max-risk',type=float,default=.05)
    a=p.parse_args();print(json.dumps(run(a.input,a.out,a.library,a.method,a.max_risk),indent=2))
