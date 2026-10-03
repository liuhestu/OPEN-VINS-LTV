import ctypes as ct
import numpy as np
from pathlib import Path
D=np.ctypeslib.ndpointer(dtype=np.float64,flags='C_CONTIGUOUS')
I=np.ctypeslib.ndpointer(dtype=np.int32,flags='C_CONTIGUOUS')
class Core:
    def __init__(self,path,q,v,p):
        self.lib=ct.CDLL(str(path)); l=self.lib
        l.create.argtypes=[ct.c_double]*3; l.create.restype=ct.c_void_p
        l.destroy.argtypes=[ct.c_void_p]
        l.predict.argtypes=[ct.c_void_p,ct.c_double,D,D]
        l.camera.argtypes=[ct.c_void_p,ct.c_double,ct.c_int,I,D,D,D,ct.c_int,I,D,ct.c_void_p]
        l.read_state.argtypes=[ct.c_void_p,D,D,I,D]; l.slot.argtypes=[ct.c_void_p,ct.c_int]
        l.truth_cpp.argtypes=[ct.c_double,ct.c_int,D]
        self.ptr=l.create(q,v,p); self.active=set(); self.diag=np.zeros(6)
    def close(self):
        if self.ptr: self.lib.destroy(self.ptr); self.ptr=None
    def predict(self,dt,a,w):
        if not self.lib.predict(self.ptr,dt,np.ascontiguousarray(a),np.ascontiguousarray(w)): raise RuntimeError('CORE reset during propagation')
    def camera(self,t,ids,z,Rbc,pc,seeds=None,exact=None):
        seeds=seeds or {}; si=np.array(list(seeds),dtype=np.int32); sx=np.array(list(seeds.values()),dtype=float).reshape(-1,3)
        exact=None if exact is None else np.ascontiguousarray(exact)
        ok=self.lib.camera(self.ptr,t,len(ids),np.ascontiguousarray(ids,dtype=np.int32),np.ascontiguousarray(z),np.ascontiguousarray(Rbc),np.ascontiguousarray(pc),len(si),si,sx,None if exact is None else exact.ctypes.data)
        self.active.update(map(int,ids)); self.active={i for i in self.active if self.lib.slot(self.ptr,i)>=0}
        if not ok: raise RuntimeError('CORE reset during camera')
    def read(self):
        x=np.zeros(96); p=np.zeros(96*96); ids=np.zeros(30,dtype=np.int32)
        d=self.lib.read_state(self.ptr,x,p,ids,self.diag)
        ordered=sorted(self.active,key=lambda i:self.lib.slot(self.ptr,i))
        return x[:d].copy(),p[:d*d].reshape(d,d).copy(),ordered

class Lifecycle:
    """Independent transcription of core slot/candidate rules, retaining all cross blocks."""
    def __init__(self,p):
        self.x=np.zeros(6); self.P=p*np.eye(6); self.ids=[]; self.missed={}; self.age={}; self.p=p
    def camera(self,ids):
        seen=set(map(int,ids)); old=self.ids
        self.age={i:a for i,a in self.age.items() if i in seen or i in old}
        for i in seen: self.age[i]=self.age.get(i,0)+1
        kept=[]
        for i in old:
            self.missed[i]=0 if i in seen else self.missed.get(i,0)+1
            if self.missed[i]<=2: kept.append(i)
            else: self.missed.pop(i); self.age.pop(i,None)
        candidates=sorted(seen-set(old),key=lambda i:(-self.age[i],i))
        new=kept+candidates[:30-len(kept)]
        if new!=old:
            oi={id:s for s,id in enumerate(old)}; index=[]
            for id in new: index += list(range(3*oi[id],3*oi[id]+3)) if id in oi else [-1]*3
            index+=list(range(len(self.x)-6,len(self.x)))
            x=np.zeros(len(index)); P=np.zeros((len(index),len(index))); good=np.flatnonzero(np.array(index)>=0); src=np.array(index)[good]
            x[good]=self.x[src]; P[np.ix_(good,good)]=self.P[np.ix_(src,src)]
            for j,id in enumerate(new):
                if id not in oi: P[3*j:3*j+3,3*j:3*j+3]=self.p*np.eye(3)
            self.x,self.P,self.ids=x,P,new
        return self.x,self.P,self.ids
