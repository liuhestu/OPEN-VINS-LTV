import numpy as np
from truth_model import truth,rot
class Seeder:
    def __init__(self,scene,mode,Rbc,pc,priors):
        self.scene,self.mode,self.Rbc,self.pc,self.priors=scene,mode,Rbc,pc,priors
        self.history={}; self.done=set(); self.events=[]; self.birth={}
    def update(self,t,tick,ids,z,active):
        R,p,*_=truth(t,self.scene); c=p+R@self.pc
        if self.mode=='GEOM_NOISY':
            c=c+self.priors[tick,:3]; R=R@self.Rbc@rot(self.priors[tick,3:])@self.Rbc.T
            p=c-R@self.pc
        seeds={}
        for id,b in zip(ids,z):
            id=int(id); self.birth.setdefault(id,t)
            h=self.history.setdefault(id,[]); h.append((t,c.copy(),R@self.Rbc@b))
            self.history[id]=h=[e for e in h if e[0]>=t-1-1e-9]
            if id in self.done or id not in active: continue
            if self.mode=='GT_SEED':
                from truth_model import sample
                seeds[id]=sample(t,self.scene,[id])[0][:3]; self.done.add(id)
                self.events.append({'id':id,'t':t,'delay':t-self.birth[id],'seed_error':0.}); continue
            if len(h)<3 or t-h[0][0]<.10-1e-9: continue
            directions=np.array([e[2] for e in h]); centers=np.array([e[1] for e in h])
            proj=np.eye(3)-directions[:,:,None]*directions[:,None,:]; A=proj.sum(axis=0)
            cond=np.linalg.cond(A); angle=np.degrees(np.arccos(np.clip((directions@directions.T).min(),-1,1)))
            if cond>1e8 or angle<.5: continue
            point=np.linalg.solve(A,np.einsum('nij,nj->i',proj,centers))
            delta=point-centers; rays=np.einsum('ij,ij->i',delta,directions)
            residual=np.degrees(np.arccos(np.clip(np.einsum('ij,ij->i',delta/np.linalg.norm(delta,axis=1)[:,None],directions),-1,1))).max()
            if rays.min()<=0 or residual>.5: continue
            from truth_model import sample
            gt=sample(t,self.scene,[id])[0][:3]
            seed=R.T@(point-p)
            seeds[id]=gt if self.mode=='GT_MATCHED' else seed
            self.done.add(id); self.events.append({'id':id,'t':t,'delay':t-self.birth[id],'seed_error':float(np.linalg.norm(seed-gt)),'condition':float(cond),'angle':float(angle),'residual':float(residual)})
        # Drop invisible histories: no future use for expired IDs.
        self.history={i:h for i,h in self.history.items() if i in set(map(int,ids))}
        return seeds
