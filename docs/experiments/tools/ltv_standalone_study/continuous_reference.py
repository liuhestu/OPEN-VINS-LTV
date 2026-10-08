"""Full, unscaled Riccati equation; no covariance clipping."""
import numpy as np
from scipy.integrate import solve_ivp
from truth_model import sample,geometry,matrices

def rhs(t, packed, scene, q, v):
    _,a,w,z=sample(t,scene); points,Rbc,pc=geometry(scene); d=3*len(points)+6
    x=packed[:d]; P=packed[d:].reshape(d,d)
    A,u,C,y=matrices(w,a,z,Rbc,pc,len(points)); PC=P@C.T
    return np.r_[A@x+u+q*PC@(y-C@x),(A@P+P@A.T-q*PC@C@P+v*np.eye(d)).ravel()]

def integrate(scene,q,v,p,times,exact=False,tight=False):
    d=3*len(geometry(scene)[0])+6
    x=sample(0,scene)[0] if exact else np.zeros(d)
    result=solve_ivp(rhs,(0,times[-1]),np.r_[x,(p*np.eye(d)).ravel()],args=(scene,q,v),method='DOP853',t_eval=times,rtol=1e-11 if tight else 1e-9,atol=1e-13 if tight else 1e-11,max_step=.0025 if tight else .005)
    if not result.success: raise RuntimeError(result.message)
    return result.y[:d].T,result.y[d:].T.reshape(-1,d,d),result.nfev
