"""CSR sensor/prior input runner for the deployed C++ LtvAdapter.

No labels or truth module imports. Feed 200Hz endpoint IMU into Adapter; camera
calls perform its causal propagation. Between-camera snapshots retain their
actual cursor and are explicitly not new 200Hz observer estimates.
"""
import os
for key in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS'):
    os.environ[key] = '1'
import argparse
import ctypes as C
import hashlib
import importlib.util
import json
from pathlib import Path
import time
import numpy as np

ROOT = Path(__file__).resolve().parents[4]
spec = importlib.util.spec_from_file_location('previous_input_covariance', ROOT/'docs/experiments/tools/ltv_feature_passive/synthetic/run.py')
old = importlib.util.module_from_spec(spec)
spec.loader.exec_module(old)
DP=C.POINTER(C.c_double); IP=C.POINTER(C.c_int32); LP=C.POINTER(C.c_int64)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


class Adapter:
    def __init__(self,library,hardened,source,sigma,diagnostics=True):
        self.lib=C.CDLL(str(Path(library).resolve()));lib=self.lib
        lib.ph_create.argtypes=[C.c_int,C.c_int,C.c_double];lib.ph_create.restype=C.c_void_p
        lib.ph_destroy.argtypes=[C.c_void_p]
        diagnostic_setter=getattr(lib,'ph_set_diagnostics',None)
        if diagnostic_setter:diagnostic_setter.argtypes=[C.c_void_p,C.c_int]
        lib.ph_imu.argtypes=[C.c_void_p,C.c_double,DP];lib.ph_imu.restype=C.c_int
        lib.ph_frame.argtypes=[C.c_void_p,C.c_int64,C.c_double,C.c_int,LP,IP,LP,DP,C.c_int,DP,DP,DP,DP,DP,DP]
        lib.ph_frame.restype=C.c_int
        lib.ph_json.argtypes=[C.c_void_p];lib.ph_json.restype=C.c_char_p
        lib.ph_error.argtypes=[C.c_void_p];lib.ph_error.restype=C.c_char_p
        lib.ph_read.argtypes=[C.c_void_p,DP,DP,LP,C.c_int];lib.ph_read.restype=C.c_int
        self.handle=lib.ph_create(int(hardened),{'TEMPORAL':0,'STEREO':1,'HYBRID':2}[source],sigma)
        if not self.handle:raise RuntimeError('Adapter configuration rejected')
        if diagnostic_setter:diagnostic_setter(self.handle,int(diagnostics))
        elif not diagnostics:raise RuntimeError('This legacy wrapper cannot disable heavy diagnostics')

    def check(self,result):
        if not result:raise RuntimeError(self.lib.ph_error(self.handle).decode())

    def imu(self,t,measurement):
        value=np.ascontiguousarray(measurement,dtype=np.float64)
        self.check(self.lib.ph_imu(self.handle,float(t),value.ctypes.data_as(DP)))

    def frame(self,data,k,timings=None):
        start_time=time.perf_counter()
        a,b=data['observation_offsets'][k:k+2];start=max(0,k-20)
        arrays=[np.ascontiguousarray(data['feature_ids'][a:b],dtype=np.int64),
                np.ascontiguousarray(data['camera_ids'][a:b],dtype=np.int32),
                np.ascontiguousarray(data['actual_ns'][a:b],dtype=np.int64),
                np.ascontiguousarray(data['bearings'][a:b],dtype=np.float64),
                np.ascontiguousarray(data['camera_times'][start:k+1]),
                np.ascontiguousarray(data['prior_R_WB'][start:k+1]),
                np.ascontiguousarray(data['prior_p_WB'][start:k+1]),
                np.ascontiguousarray(old.pose_covariance(data,start,k+1)),
                np.ascontiguousarray(data['R_BC']),np.ascontiguousarray(data['p_BC'])]
        # Keep all arrays alive until the synchronous C ABI call has returned.
        args=[x.ctypes.data_as(pointer) for x,pointer in zip(arrays,[LP,IP,LP,DP,DP,DP,DP,DP,DP,DP])]
        abi_start=time.perf_counter()
        self.check(self.lib.ph_frame(self.handle,int(data['camera_ns'][k]),float(data['camera_times'][k]),int(b-a),
                                    *args[:4],k+1-start,*args[4:]))
        parse_start=time.perf_counter()
        result=json.loads(self.lib.ph_json(self.handle).decode())
        if timings is not None:
            timings.update(construct_ms=(abi_start-start_time)*1000,abi_ms=(parse_start-abi_start)*1000,
                           parse_ms=(time.perf_counter()-parse_start)*1000)
        return result

    def state(self,copy_P=False):
        x=np.full(96,np.nan);flat=np.full(96*96,np.nan);ids=np.full(30,-1,np.int64)
        d=self.lib.ph_read(self.handle,x.ctypes.data_as(DP),flat.ctypes.data_as(DP),ids.ctypes.data_as(LP),int(copy_P))
        if d<0 or d>96 or (d and (d<6 or (d-6)%3)):raise RuntimeError('Invalid actual core dimension')
        if d and not np.isfinite(x[:d]).all():raise RuntimeError('Nonfinite actual core state')
        P=np.full((96,96),np.nan)
        if copy_P and d:
            P[:d,:d]=flat[:d*d].reshape(d,d)
            if not np.isfinite(P[:d,:d]).all():raise RuntimeError('Nonfinite actual core P')
        return x,P,ids,d

    def close(self):
        if self.handle:self.lib.ph_destroy(self.handle);self.handle=None


def run(input_dir,out,library,mode='P_NEW',source='HYBRID',max_seconds=None,diagnostics=True):
    input_dir=Path(input_dir);out=Path(out)
    identity=json.loads((input_dir/'identity.json').read_text())
    if sha(input_dir/'inputs.npz')!=identity['input_sha']:raise ValueError('Input SHA mismatch')
    with np.load(input_dir/'inputs.npz',allow_pickle=False) as z:data={k:z[k] for k in z.files}
    if int(data['schema_version'])!=2:raise ValueError('CSR schema version required')
    if mode not in ('P_PREV','P_NEW','P_NEW_WARMUP','P_NEW_PRESERVE','P_NEW_GRACE'):raise ValueError('Unknown mode')
    if 'imu_sample_times' not in data or 'imu_samples' not in data:raise ValueError('Adapter requires explicit timestamped IMU samples; do not relabel midpoint values')
    out.mkdir(parents=True,exist_ok=False)
    metadata={'mode':mode,'source':source,'input_sha':identity['input_sha'],'library_sha':sha(library),'runner_sha':sha(__file__),
              'prior_covariance_helper_sha':sha(old.__file__),'GT_estimator_access':False,'status':'RUNNING',
              'imu_contract':identity.get('adapter_imu_contract','Frozen 200Hz endpoint measurements; Adapter does interpolation/propagation'),
              'between_camera_contract':'Held internal snapshot with actual camera cursor, never a new current estimate',
              'max_seconds':max_seconds,'heavy_diagnostics':bool(diagnostics)}
    (out/'run.json').write_text(json.dumps(metadata,indent=2))
    adapter=Adapter(library,{'P_PREV':0,'P_NEW':1,'P_NEW_WARMUP':2,'P_NEW_PRESERVE':3,'P_NEW_GRACE':4}[mode],source,float(data['bearing_sigma_rad']),diagnostics)
    samples=[];dims=[];sample_ids=[];times=[];cursors=[];current=[]
    matrices=[];matrix_x=[];matrix_ids=[];matrix_dims=[];matrix_t=[];matrix_tags=[]
    cursor=0;k=0;frame={'cursor':0.,'raw_current':False};started=time.monotonic()
    def snapshot(t,tag):
        x,P,ids,d=adapter.state(True);matrix_t.append(t);matrix_tags.append(tag);matrices.append(P)
        matrix_x.append(x);matrix_ids.append(ids);matrix_dims.append(d)
    try:
        with (out/'events.jsonl').open('w') as events:
            for tick,t in enumerate(data['imu_times']):
                if max_seconds is not None and t>max_seconds:break
                # Same real-runner bracket rule: available through target plus one
                # right endpoint. The Adapter never integrates the future endpoint.
                while cursor<len(data['imu_sample_times']) and data['imu_sample_times'][cursor]<=t:
                    adapter.imu(data['imu_sample_times'][cursor],data['imu_samples'][cursor]);cursor+=1
                if tick%10==0:
                    if cursor<len(data['imu_sample_times']):
                        adapter.imu(data['imu_sample_times'][cursor],data['imu_samples'][cursor]);cursor+=1
                    if diagnostics:snapshot(float(t),'before_camera')
                    frame=adapter.frame(data,k);k+=1
                    events.write(json.dumps(frame,allow_nan=False)+'\n');events.flush()
                    if diagnostics:snapshot(float(t),'after_camera')
                x,_,ids,d=adapter.state()
                samples.append(x);sample_ids.append(ids);dims.append(d);times.append(t);cursors.append(frame['cursor'])
                current.append(bool(frame['raw_current'] and frame['cursor']==t))
        metadata['status']='COMPLETED_ESTIMATION'
    except Exception as error:
        metadata['status']='FAILED';metadata['error']=repr(error)
        try:snapshot(float(t),'exception')
        except Exception:pass
        raise
    finally:
        adapter.close()
        np.savez_compressed(out/'states.npz',t=times,x=samples,ids=sample_ids,dimension=dims,observer_cursor=cursors,raw_current=current)
        np.savez_compressed(out/'matrices.npz',t=matrix_t,tags=matrix_tags,P=matrices,x=matrix_x,ids=matrix_ids,dimension=matrix_dims)
        metadata.update(elapsed_seconds=time.monotonic()-started,camera_events=k,samples=len(times),
                        full_input_consumed=len(times)==len(data['imu_times']),states_sha=sha(out/'states.npz'))
        (out/'run.json').write_text(json.dumps(metadata,indent=2))
    return metadata


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--input-dir',required=True);p.add_argument('--out',required=True);p.add_argument('--library',required=True)
    p.add_argument('--mode',choices=['P_PREV','P_NEW','P_NEW_WARMUP','P_NEW_PRESERVE','P_NEW_GRACE'],default='P_NEW');p.add_argument('--source',choices=['TEMPORAL','STEREO','HYBRID'],default='HYBRID')
    p.add_argument('--light-diagnostics',action='store_false',dest='diagnostics',default=True)
    p.add_argument('--max-seconds',type=float)
    print(json.dumps(run(**vars(p.parse_args())),indent=2))
