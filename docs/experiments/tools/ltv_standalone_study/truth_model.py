"""Analytic truth. Only sensors, explicitly labelled priors, and evaluation use it."""
import numpy as np
from scipy.spatial.transform import Rotation
G = np.array([0., 0., -9.81])
SCENES = ['REGULAR', 'FAST', 'PAPER', 'STATIC']
def skew(v):
    x,y,z=v
    return np.array([[0,-z,y],[z,0,-x],[-y,x,0.]])
def rot(v):
    return Rotation.from_rotvec(v).as_matrix()
AXIS = np.array([.3,-.4,.8]); AXIS /= np.linalg.norm(AXIS)
BASE = rot(np.array([.23,0,0])) @ rot(np.array([0,-.18,0]))
def truth(t, scene):
    if scene == 'PAPER':
        R = rot(t*np.array([-1.,3,0])) @ rot(np.array([0.,-2*t,0]))
        return R, 2*np.array([np.sin(t),np.sin(t)*np.cos(t),1]), np.array([2*np.cos(t),2*np.cos(2*t),0]), np.array([-2*np.sin(t),-4*np.sin(2*t),0]), np.array([-np.cos(2*t),1,np.sin(2*t)])
    if scene == 'STATIC':
        return BASE, np.zeros(3), np.zeros(3), np.zeros(3), np.zeros(3)
    scale = 3 if scene == 'FAST' else 1
    return BASE @ rot(.6*np.sin(.8*t)*AXIS), scale*np.array([1.8*np.sin(.7*t),1.2*np.sin(1.1*t),.7*np.sin(.9*t)]), scale*np.array([1.26*np.cos(.7*t),1.32*np.cos(1.1*t),.63*np.cos(.9*t)]), scale*np.array([-.882*np.sin(.7*t),-1.452*np.sin(1.1*t),-.567*np.sin(.9*t)]), .48*np.cos(.8*t)*AXIS

def geometry(scene):
    if scene == 'PAPER':
        return np.array([[x,y,0.] for x in [-1.5,-.5,.5,1.5] for y in [-1.5,-.5,.5,1.5]]), np.eye(3), np.array([.02,.06,.01])
    return np.array([[(j%6-2.5)*1.4,(j//6-2)*1.3,12+.17*j] for j in range(30)]),rot(np.array([0,.2,0.])),np.array([.1,-.04,.06])

def sample(t,scene,physical=None):
    points,Rbc,pc=geometry(scene)
    if physical is not None: points=points[np.asarray(physical,dtype=int)%len(points)]
    R,p,v,a,w=truth(t,scene)
    ell=(points-p)@R; cam=(ell-pc)@Rbc
    z=cam/np.linalg.norm(cam,axis=1)[:,None]
    return np.r_[ell.ravel(),R.T@v,R.T@G], R.T@(a-G), w, z

def ids_at(tick, camera_hz, lifetime, n):
    # floor(t/T+j/N), using integer camera ticks, including staggered boundaries.
    j=np.arange(n,dtype=np.int32)
    return j if lifetime == 0 else n*((tick*n+j*round(lifetime*camera_hz))//(round(lifetime*camera_hz)*n))+j

def matrices(w,a,z,Rbc,pc,n,slots=None):
    d=3*n+6
    A=np.kron(np.eye(n+2),-skew(w)); A[-6:-3,-3:]=np.eye(3)
    for j in range(n): A[3*j:3*j+3,-6:-3]=-np.eye(3)
    u=np.zeros(d); u[-6:-3]=a
    C=np.zeros((3*len(z),d)); y=np.empty(3*len(z))
    for j,b in enumerate(z@Rbc.T):
        proj=np.eye(3)-np.outer(b,b); s=j if slots is None else slots[j]
        C[3*j:3*j+3,3*s:3*s+3]=proj; y[3*j:3*j+3]=proj@pc
    return A,u,C,y
