"""Frozen-input exact dynamics, full P. Frozen correction does not advance time."""
import numpy as np
from scipy.linalg import expm,solve
from truth_model import skew

def predict(x,P,a,w,dt,v):
    n=(len(x)-6)//3; d=len(x)
    # A = I kron (-[w]x) + nilpotent coupling, whose terms commute.
    E=expm(-dt*skew(w)); F=np.kron(np.eye(n+2),E)
    for j in range(n):
        F[3*j:3*j+3,-6:-3]=-dt*E
        F[3*j:3*j+3,-3:]=-.5*dt*dt*E
    F[-6:-3,-3:]=dt*E
    # Scalar equal V blocks are invariant to rotation: exact polynomial integral.
    H=np.zeros((n+2,n+2)); H[:n,n]=-1; H[n,n+1]=1
    powers=[np.eye(n+2),H,H@H/2]
    W=sum(dt**(i+j+1)/(i+j+1)*(powers[i]@powers[j].T) for i in range(3) for j in range(3))
    # Integrate acceleration with a small augmented matrix exponential.
    T=np.zeros((10,10)); T[:3,:3]=T[3:6,3:6]=T[6:9,6:9]=-skew(w)
    T[:3,3:6]=-np.eye(3); T[3:6,6:9]=np.eye(3); T[3:6,9]=a
    b=expm(dt*T)[:9,9]; u=np.r_[np.tile(b[:3],n),b[3:6],b[6:9]]
    return F@x+u,F@P@F.T+v*np.kron(W,np.eye(3))

def correct(x,P,C,y,q,tau):
    # Woodbury form of exact information update, stable even at small P.
    if tau == 0 or len(y)==0: return x.copy(),P.copy()
    PC=P@C.T; S=np.eye(len(y))+tau*q*C@PC
    K=solve(S,PC.T,assume_a='pos').T*(tau*q)
    return x+K@(y-C@x),P-K@C@P
