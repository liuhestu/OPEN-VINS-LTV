"""A 0.2s correctness fixture; not scientific convergence evidence."""
import os
for name in ['OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS']:os.environ[name]='1'
import argparse
import ctypes as C
import json
from pathlib import Path
import numpy as np
from generate import generate
from run import run,pose_covariance,DP,IP,dptr,iptr
from evaluate import evaluate,first_hold

def main():
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);p.add_argument('--library',type=Path,required=True);a=p.parse_args()
    a.out.mkdir(parents=True,exist_ok=False)
    assert first_hold([(0.,0.),(.05,0.),(.1,0.),(.1,.2)],.1)==(None,None)
    assert first_hold([(0.,0.),(.05,0.),(.1,0.)],.1)==(0.,.1)
    generate(a.out/'input','REGULAR',42,1.,duration=.2,noise=False)
    data=np.load(a.out/'input/inputs.npz');labels=np.load(a.out/'input/labels.npz')
    assert 'state_body' not in data.files and 'points_W' not in data.files and 'pose_noise_camera' not in data.files
    assert np.max(abs(data['prior_R_WB']-labels['R_WB']))<1e-14
    assert np.max(abs(data['prior_p_WB']-labels['p_WB']))<1e-15
    for k in range(len(data['camera_times'])):
        gt=labels['state_body'][k*10,:90].reshape(30,3)
        for camera,key in [(0,'bearings_left'),(1,'bearings_right')]:
            z=(gt-data['p_BC'][camera])@data['R_BC'][camera];z/=np.linalg.norm(z,axis=1,keepdims=True)
            assert np.max(abs(z-data[key][k]))<1e-15
    # Reuse exact arrays through the ABI's explicit Eigen maps (including full P).
    l=C.CDLL(str(a.library));l.fp_echo.argtypes=[C.c_int,IP,DP,DP,C.c_int,DP,DP,DP,DP,DP,DP,DP];l.fp_echo.restype=C.c_int
    k=4;cov=pose_covariance(data,0,k+1)
    arrays=[data['feature_ids'][k],data['bearings_left'][k],data['bearings_right'][k],data['camera_times'][:k+1],
            data['prior_R_WB'][:k+1],data['prior_p_WB'][:k+1],cov,data['R_BC'],data['p_BC']]
    expected=np.concatenate([x.ravel() for x in arrays]);actual=np.zeros_like(expected)
    size=l.fp_echo(30,iptr(arrays[0]),dptr(arrays[1]),dptr(arrays[2]),k+1,*[dptr(x) for x in arrays[3:]],dptr(actual))
    assert size==len(expected) and np.array_equal(actual,expected)
    result={}
    for method in ['OLD','SEED_STEREO']:
        labels_path=a.out/'input/labels.npz'; hidden=a.out/'input/evaluator_labels_hidden.npz'
        labels_path.rename(hidden)
        try:run(a.out/'input',a.out/method,a.library,method)
        finally:hidden.rename(labels_path)
        metrics=evaluate(a.out/'input',a.out/method,a.out/'OLD' if method!='OLD' else None)
        m=np.load(a.out/method/'matrices.npz');s=np.load(a.out/method/'states.npz')
        assert len(s['t'])==41 and len(m['t'])==10
        for P,d in zip(m['P'],m['dimension']):
            assert np.isfinite(P[:d,:d]).all() and np.max(abs(P[:d,:d]-P[:d,:d].T))<1e-10
        result[method]=metrics
    seeds=json.loads((a.out/'SEED_STEREO/seed_evaluation.json').read_text())
    assert seeds and all(x['t']>=.1-1e-12 and x['relative_error']<1e-9 for x in seeds)
    assert result['SEED_STEREO']['seed']['accepted_seeds']>0
    assert result['SEED_STEREO']['diagnostics']['injection_G']==0 and result['SEED_STEREO']['diagnostics']['injection_V']==0
    # An independently generated noisy input also traverses the actual covariance/pipeline path.
    generate(a.out/'noisy_input','REGULAR',42,1.,duration=.2,noise=True)
    noisy_summaries={}
    for method in ['OLD','SEED_STEREO']:
        run(a.out/'noisy_input',a.out/('noisy_'+method),a.library,method)
        noisy_summaries[method]=evaluate(a.out/'noisy_input',a.out/('noisy_'+method),a.out/'noisy_OLD' if method!='OLD' else None)
        assert noisy_summaries[method]['diagnostics']['full_input_consumed']
    (a.out/'unit_result.json').write_text(json.dumps({'status':'PASS','input_roundtrip_exact':True,
        'noisy_methods':['OLD','SEED_STEREO'],'state_ticks':41,'matrices_per_method':10,'seed_count':len(seeds),'max_seed_relative_error':max(x['relative_error'] for x in seeds)},indent=2))
    print((a.out/'unit_result.json').read_text())
if __name__=='__main__':main()
