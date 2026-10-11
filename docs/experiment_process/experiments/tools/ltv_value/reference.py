"""Offline physical-time references. Neither this module nor GT paths enter C++ replay."""
import numpy as np
from scipy.spatial.transform import Rotation, Slerp
from common import *
def load_gt(seq):
    data=read(OUT/'inputs.json')[seq];x=np.loadtxt(data['gt'],delimiter=',' if seq in EUROC else None,comments='#')
    if seq in EUROC: return x[:,0]*1e-9,x[:,1:4],x[:,[5,6,7,4]],x[:,8:11]
    return x[:,0],x[:,1:4],x[:,4:8],None
def local_derivative(t,p,query,width=.1,order=1):
    result=np.full((len(query),3),np.nan)
    for k,u in enumerate(query):
        if u-width/2<t[0] or u+width/2>t[-1]:continue
        a,b=np.searchsorted(t,[u-width/2,u+width/2])
        if b-a<9 or np.max(np.diff(t[a:b]))>.010000001:continue
        z=(t[a:b]-u)/width;design=np.column_stack([z**i for i in range(4)])
        coef=np.linalg.lstsq(design,p[a:b]-p[a:b].mean(axis=0),rcond=None)[0]
        result[k]=coef[order]*(1 if order==1 else 2)/width**order
    return result
def interpolate(t,p,q,query):
    idx=np.searchsorted(t,query).clip(1,len(t)-1);a=idx-1;b=idx
    valid=(query>=t[0])&(query<=t[-1])&((t[b]-t[a])<=.010000001)
    pos=np.full((len(query),3),np.nan);rot=np.full((len(query),3,3),np.nan)
    w=(query[valid]-t[a[valid]])/(t[b[valid]]-t[a[valid]])
    pos[valid]=p[a[valid]]*(1-w[:,None])+p[b[valid]]*w[:,None]
    rot[valid]=Slerp(t-t[0],Rotation.from_quat(q))(query[valid]-t[0]).as_matrix()
    return pos,rot,valid
def reference(seq,query,width=.1):
    t,p,q,v=load_gt(seq);pos,rot,valid=interpolate(t,p,q,query)
    vel=local_derivative(t,p,query,width) if v is None else np.column_stack([np.interp(query,t,v[:,i]) for i in range(3)])
    vel[~valid]=np.nan
    body=np.einsum('nji,nj->ni',rot,vel)
    grav=np.einsum('nji,j->ni',rot,np.array([0.,0.,-1.]))
    return {'p':pos,'R':rot,'v_world':vel,'v_body':body,'g_body':grav,'pose_valid':valid,'v_valid':np.isfinite(body).all(axis=1)}
def body_estimate(x):
    # Native JPL G->I stored coefficients equal Hamilton I->G coefficients.
    rot=Rotation.from_quat(x[:,:4]).as_matrix()
    return np.einsum('nji,nj->ni',rot,x[:,7:10]),np.einsum('nji,j->ni',rot,np.array([0.,0.,-1.]))
def angle(a,b):
    an=np.linalg.norm(a,axis=1);bn=np.linalg.norm(b,axis=1)
    return np.degrees(np.arccos(np.clip(np.einsum('ij,ij->i',a,b)/(an*bn),-1,1)))
def tangent_error(direction,truth):
    direction=direction/np.linalg.norm(direction,axis=1)[:,None];truth=truth/np.linalg.norm(truth,axis=1)[:,None]
    ref=np.zeros_like(truth);ref[np.arange(len(truth)),np.argmin(np.abs(truth),axis=1)]=1
    u=np.cross(truth,ref);u/=np.linalg.norm(u,axis=1)[:,None];v=np.cross(truth,u)
    dot=np.clip(np.einsum('ij,ij->i',direction,truth),-1,1);theta=np.arccos(dot)
    delta=direction-dot[:,None]*truth;norm=np.linalg.norm(delta,axis=1)
    delta*=np.divide(theta,norm,out=np.ones_like(theta),where=norm>1e-12)[:,None]
    return np.column_stack([np.einsum('ij,ij->i',delta,u),np.einsum('ij,ij->i',delta,v)])
