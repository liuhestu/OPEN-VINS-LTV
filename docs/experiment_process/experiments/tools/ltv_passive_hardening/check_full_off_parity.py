"""Numerical full-run parity, including slots and all saved cross blocks."""
import os
os.environ['OPENBLAS_NUM_THREADS']='1'
import argparse,json
from pathlib import Path
import numpy as np

def check(previous,current):
    previous,current=Path(previous),Path(current)
    a=json.loads((previous/'run.json').read_text());b=json.loads((current/'run.json').read_text())
    assert a['input_sha']==b['input_sha'] and a['mode']==b['mode']=='P_PREV'
    assert a['full_input_consumed'] and b['full_input_consumed']
    counts={}
    for filename in ('states.npz','matrices.npz'):
        with np.load(previous/filename) as p,np.load(current/filename) as q:
            assert set(p.files)==set(q.files)
            for key in p.files:
                x,y=p[key],q[key];assert x.shape==y.shape and x.dtype==y.dtype
                for start in range(0,len(x),64):
                    a,b=x[start:start+64],y[start:start+64]
                    same=a==b
                    if np.issubdtype(a.dtype,np.floating):same|=np.isnan(a)&np.isnan(b)
                    assert np.all(same),(filename,key,start)
                counts[filename+'/'+key]=list(x.shape)
    result={'status':'PASS_FULL_NUMERICAL_PARITY','previous':str(previous),'current':str(current),'arrays':counts,'tolerance':0,'padding_nan_compared_as_padding':True}
    (current/'full_off_parity.json').write_text(json.dumps(result,indent=2)+'\n');return result
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('previous');p.add_argument('current');a=p.parse_args();print(json.dumps(check(a.previous,a.current),indent=2))
