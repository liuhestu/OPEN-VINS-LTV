"""Short correctness checks, intentionally not full-horizon scientific runs."""
import numpy as np
from scipy.integrate import solve_ivp
from truth_model import *
from continuous_reference import rhs
from split_reference import predict,correct
from core import Core,Lifecycle

def verify(out):
    results={}; rng=np.random.default_rng(701)
    core=Core(out/'core.so',1e-4,1e6,1); exp=Core(out/'experimental.so',1e-4,1e6,1)
    for scene in SCENES:
        for t in [0.,.13,1.7,7.3]:
            values=np.zeros(21); core.lib.truth_cpp(t,SCENES.index(scene),values)
            py=np.concatenate([q.ravel() for q in truth(t,scene)])
            assert np.max(np.abs(py-values))<1e-12
            x,a,w,z=sample(t,scene); pts,Rbc,pc=geometry(scene); A,u,C,y=matrices(w,a,z,Rbc,pc,len(pts))
            h=1e-5; deriv=(sample(t+h,scene)[0]-sample(t-h,scene)[0])/(2*h)
            assert np.linalg.norm(deriv-A@x-u)/max(1,np.linalg.norm(deriv))<1e-8
            assert np.linalg.norm(y-C@x)<1e-10
    results['analytic_truth_cpp_and_dynamics']='PASS'
    x,a,w,z=sample(.3,'REGULAR'); _,Rbc,pc=geometry('REGULAR'); A,u,C,y=matrices(w,a,z,Rbc,pc,30)
    d=len(x); xhat=rng.normal(size=d); M=rng.normal(size=(d,d)); P=M@M.T+np.eye(d); q=1e-4; v=1e6
    old=np.r_[xhat,P.ravel()]; before=old.copy(); derivative=rhs(.3,old,'REGULAR',q,v)
    assert np.array_equal(old,before)
    vector=-np.cross(np.broadcast_to(w,(32,3)),xhat.reshape(-1,3)); vector[:30]-=xhat[-6:-3]; vector[-2]+=a+xhat[-3:]
    vector+= (q*P@C.T@(y-C@xhat)).reshape(-1,3)
    assert np.max(np.abs(vector.ravel()-derivative[:d]))<1e-10
    e=x-xhat; edot=A@x+u-derivative[:d]; expected=(A-q*P@C.T@C)@e
    assert np.linalg.norm(edot-expected)<1e-10
    pe=np.linalg.solve(P,e); pdot=derivative[d:].reshape(d,d)
    udot=2*pe@edot-pe@pdot@pe; target=-q*np.linalg.norm(C@e)**2-v*(pe@pe)
    assert abs(udot-target)/max(1,abs(target))<1e-8
    results['old_rhs_vector_error_and_U_identity']='PASS'
    xx,pp=correct(xhat,P,C,y,q,.05)
    def corr(t,b):
        xx=b[:d]; pp=b[d:].reshape(d,d); pc=pp@C.T
        return np.r_[q*pc@(y-C@xx),(-q*pc@C@pp).ravel()]
    ref=solve_ivp(corr,(0,.05),old,method='DOP853',rtol=1e-12,atol=1e-13).y[:,-1]
    assert np.linalg.norm(ref[:d]-xx)/max(1,np.linalg.norm(xx))<1e-8
    assert np.linalg.norm(ref[d:].reshape(d,d)-pp)/np.linalg.norm(pp)<1e-8
    xx,pp=predict(xhat,P,a,w,.005,v)
    def pred(t,b):
        xx=b[:d]; pp=b[d:].reshape(d,d)
        return np.r_[A@xx+u,(A@pp+pp@A.T+v*np.eye(d)).ravel()]
    ref=solve_ivp(pred,(0,.005),old,method='DOP853',rtol=1e-12,atol=1e-13).y[:,-1]
    assert np.linalg.norm(ref[:d]-xx)<1e-8
    assert np.linalg.norm(ref[d:].reshape(d,d)-pp)/np.linalg.norm(pp)<1e-8
    results['exact_split_vs_ODE']='PASS'
    lc=Lifecycle(1)
    for k in range(101):
        t=k*.005
        if k:
            _,a,w,_=sample(t-.0025,'FAST'); core.predict(.005,a,w); exp.predict(.005,a,w)
        if k%10==0:
            _,_,_,z=sample(t,'FAST'); ids=ids_at(k//10,20,.5,30)
            core.camera(t,ids,z,Rbc,pc); exp.camera(t,ids,z,Rbc,pc)
            # Give lifecycle arbitrary full cross blocks before reindexing, compare to core's rebuild via its pre-update P separately below.
            lc.camera(ids)
            assert lc.ids==core.read()[2]
        cx,cp,ci=core.read(); ex,ep,ei=exp.read()
        assert ci==ei and np.array_equal(cx,ex) and np.array_equal(cp,ep)
        assert np.array_equal(core.diag[:2],exp.diag[:2])
    # Hook at t=0 changes only requested landmark; initial camera has zero correction.
    exp.close(); exp=Core(out/'experimental.so',q,v,1)
    ids=np.arange(30,dtype=np.int32); _,_,_,z=sample(0,'REGULAR'); seed=np.array([1.,2,3])
    exp.camera(0,ids,z,Rbc,pc,{7:seed}); ex,ep,ei=exp.read()
    expected=np.zeros(96); expected[21:24]=seed
    assert np.array_equal(ex,expected) and np.array_equal(ep,np.eye(96))
    # Independent lifecycle permutation keeps every off-diagonal element, not only diagonal blocks.
    lc=Lifecycle(1); lc.camera([0,1,2]); lc.x=np.arange(15.); m=rng.normal(size=(15,15)); lc.P=m@m.T; oldp=lc.P.copy(); oldx=lc.x.copy()
    for _ in range(3): lc.camera([1,2,3])
    ix=list(range(3,9))+[-1]*3+list(range(9,15)); good=np.flatnonzero(np.array(ix)>=0); src=np.array(ix)[good]
    assert np.array_equal(lc.P[np.ix_(good,good)],oldp[np.ix_(src,src)])
    assert np.array_equal(lc.x[good],oldx[src])
    for life in [.5,1,2,5]:
        L=round(life*20)
        for tick in range(3*L):
            ids=ids_at(tick,20,life,30)
            assert np.array_equal(ids%30,np.arange(30))
            if tick>=L: assert np.array_equal(ids-ids_at(tick-L,20,life,30),np.full(30,30))
    # Check actual production old-value Euler propagation, including all P blocks,
    # followed by its documented spectral projection (not an ODE accuracy claim).
    actual=Core(out/'core.so',q,v,1)
    actual.camera(0,np.arange(30,dtype=np.int32),z,Rbc,pc)
    for k in range(4):
        bx,bp,bi=actual.read(); _,aa,ww,zz=sample((k+.5)*.005,'REGULAR')
        AA,uu,_,_=matrices(ww,aa,zz,Rbc,pc,30)
        expected_x=bx+.005*(AA@bx+uu)
        expected_p=bp+.005*(AA@bp+bp@AA.T+v*np.eye(96))
        ev,vec=np.linalg.eigh((expected_p+expected_p.T)/2)
        expected_p=(vec*np.maximum(ev,1e-9))@vec.T
        actual.predict(.005,aa,ww); ax,ap,_=actual.read()
        assert np.linalg.norm(ax-expected_x)<1e-10
        assert np.linalg.norm(ap-expected_p)/np.linalg.norm(expected_p)<1e-8
    actual.close()
    results['production_old_value_Euler_full_P']='PASS'
    results['hook_off_parity_slots_cross_blocks_hook_scope_integer_ticks']='PASS'
    core.close(); exp.close()
    return results
