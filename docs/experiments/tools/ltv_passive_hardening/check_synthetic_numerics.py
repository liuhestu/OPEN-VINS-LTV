"""Full saved covariance/state checks, separate from scientific accuracy."""
import os
for key in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS'):os.environ[key]='1'
import argparse,json
from pathlib import Path
import numpy as np

def check(directory):
    root=Path(directory)
    metadata=json.loads((root/'run.json').read_text())
    assert metadata['status']=='COMPLETED_ESTIMATION' and metadata['full_input_consumed']
    minimum=float('inf');symmetry=0.;count=0
    with np.load(root/'matrices.npz') as data:
        for d,P,x,ids in zip(data['dimension'],data['P'],data['x'],data['ids']):
            assert d>=6 and (d-6)%3==0
            P=P[:d,:d];x=x[:d];active=ids[:(d-6)//3]
            assert np.isfinite(P).all() and np.isfinite(x).all()
            assert len(active)==len(set(active.tolist())) and (active>=0).all()
            error=float(np.linalg.norm(P-P.T)/max(np.linalg.norm(P),1.))
            symmetry=max(symmetry,error)
            eigen=float(np.linalg.eigvalsh((P+P.T)*.5)[0]);minimum=min(minimum,eigen);count+=1
            assert error<=1e-10,'covariance asymmetry'
            assert eigen>=-1e-8,'covariance PSD violation'
    result={'status':'PASS_NUMERICS_ONLY','saved_matrices':count,'minimum_eigenvalue':minimum,'max_relative_asymmetry':symmetry}
    (root/'numerics.json').write_text(json.dumps(result,indent=2)+'\n')
    return result
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('directory');print(json.dumps(check(p.parse_args().directory),indent=2))
