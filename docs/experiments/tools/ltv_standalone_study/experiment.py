import json,hashlib,time
from pathlib import Path
import numpy as np
from truth_model import *
from core import Core,Lifecycle
from continuous_reference import integrate
from split_reference import predict,correct
from geometry_seed import Seeder
from evaluate import errors,summarize,lifecycle_metrics

def sha_file(p): return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def input_arrays(config,out):
    scene=config['scene']; duration=config['duration']; hz=config['imu_hz']; chz=config['camera_hz']; seed=config['noise']
    identity={'scene':scene,'duration':duration,'imu_hz':hz,'camera_hz':chz,'noise':seed,'truth_sha':sha_file(Path(__file__).with_name('truth_model.py'))}
    key=hashlib.sha256(json.dumps(identity,sort_keys=True).encode()).hexdigest()[:20]; path=out/'inputs'/f'{key}.npz'; path.parent.mkdir(exist_ok=True)
    if not path.exists():
        rng=np.random.default_rng(seed if seed else 0); steps=round(duration*hz); frames=round(duration*chz)+1
        imu=np.array([np.r_[sample((i+.5)/hz,scene)[1:3]] for i in range(steps)])
        bearings=np.array([sample(k/chz,scene)[3] for k in range(frames)])
        if seed:
            imu[:,:3]+=rng.normal(0,.02,(steps,3)); imu[:,3:]+=rng.normal(0,.002,(steps,3))
            for frame in bearings:
                for j,b in enumerate(frame):
                    axis=np.eye(3)[np.argmin(np.abs(b))]; u=np.cross(b,axis); u/=np.linalg.norm(u); v=np.cross(b,u)
                    perturb=rng.normal(0,np.deg2rad(.05),2); bb=b+perturb[0]*u+perturb[1]*v; frame[j]=bb/np.linalg.norm(bb)
        prng=np.random.default_rng(42); priors=np.c_[prng.normal(0,.01,(frames,3)),prng.normal(0,np.deg2rad(.1),(frames,3))]
        np.savez(path,imu=imu,bearings=bearings,priors=priors)
        path.with_suffix('.json').write_text(json.dumps(identity,indent=2))
    return np.load(path),sha_file(path),str(path)

def execute(config,out,run_dir):
    started=time.monotonic(); scene=config['scene']; duration=config['duration']; hz=config['imu_hz']; chz=config['camera_hz']; impl=config['impl']; q,v,p=config['q'],config['v'],config['p']
    inputs,input_sha,input_path=input_arrays(config,out); points,Rbc,pc=geometry(scene); n=len(points); d=3*n+6
    ts=np.arange(round(duration*200)+1)/200; xs=[]; idsout=[]; diagout=[]; pst=[]; ps=[]; pids=[]; px=[]; tags=[]; camera_records=[]; eventmap={}; gain_records=[]; gain_landmarks=[]
    def snapshot(t,x,P,ids,tag):
        pp=np.full((96,96),np.nan); pp[:len(x),:len(x)]=P; xx=np.full(96,np.nan); xx[:len(x)]=x; ii=np.full(30,-1,dtype=np.int32); ii[:len(ids)]=ids
        pst.append(t); ps.append(pp); pids.append(ii); px.append(xx); tags.append(tag)
    def record(t,x,ids,diag):
        xx=np.full(96,np.nan); xx[:len(x)]=x; ii=np.full(30,-1,dtype=np.int32); ii[:len(ids)]=ids
        xs.append(xx); idsout.append(ii); diagout.append(diag.copy())
    if impl=='CONT_REF':
        x,P,nfev=integrate(scene,q,v,p,ts,config['initialization']=='EXACT',config['tight'])
        # Full P at every common time is retained for rigorous max reference comparison.
        np.save(run_dir/'reference_P.npy',P)
        for k,t in enumerate(ts):
            record(t,x[k],list(range(n)),np.zeros(6))
            if k%200==0:
                snapshot(t,x[k],P[k],list(range(n)),'continuous')
                _,a,w,z=sample(t,scene); _,_,cm,ym=matrices(w,a,z,Rbc,pc,n)
                K=q*P[k]@cm.T; contribution=K@(ym-cm@x[k]); pp=P[k]
                gain_landmarks.append(np.linalg.norm(contribution[:-6].reshape(-1,3),axis=1))
                gain_records.append([t,np.linalg.norm(K),np.linalg.norm(contribution[:-6]),np.linalg.norm(contribution[-6:-3]),np.linalg.norm(contribution[-3:]),np.linalg.norm(pp[-6:-3,:-6]),np.linalg.norm(pp[-3:,:-6]),np.linalg.norm(pp[:-6,:-6]),np.linalg.norm(pp[-6:-3,-6:-3]),np.linalg.norm(pp[-3:,-3:])])
            if k%(200//chz)==0:
                for side in ['pre','post']: camera_records.append((t,side,x[k].copy(),list(range(n))))
        del P
    else:
        nfev=0; lc=Lifecycle(p); core=None
        if impl!='SPLIT_REF': core=Core(out/('core.so' if impl=='CORE' else 'experimental.so'),q,v,p)
        seeder=Seeder(scene,config['initialization'],Rbc,pc,inputs['priors']) if config['initialization'] not in ['ZERO','EXACT'] else None
        for step in range(round(duration*hz)+1):
            t=step/hz
            if step:
                a,w=inputs['imu'][step-1,:3],inputs['imu'][step-1,3:]
                if core: core.predict(1/hz,a,w)
                else: lc.x,lc.P=predict(lc.x,lc.P,a,w,1/hz,v)
            if core: x,P,active=core.read()
            else: x,P,active=lc.x,lc.P,lc.ids
            if step%(hz//chz)==0:
                tick=step//(hz//chz); ids=ids_at(tick,chz,config['lifetime'],n); z=inputs['bearings'][tick]
                old=list(active); camera_records.append((t,'pre',x.copy(),list(active)))
                use=[j for j,id in enumerate(ids) if int(id) in active]
                slots=[active.index(int(ids[j])) for j in use]
                _,_,cm,ym=matrices(np.zeros(3),np.zeros(3),z[use],Rbc,pc,len(active),slots)
                K=q*P@cm.T; contribution=K@(ym-cm@x)
                gain_landmarks.append(np.linalg.norm(contribution[:-6].reshape(-1,3),axis=1))
                gain_records.append([t,np.linalg.norm(K),np.linalg.norm(contribution[:-6]),np.linalg.norm(contribution[-6:-3]),np.linalg.norm(contribution[-3:]),np.linalg.norm(P[-6:-3,:-6]),np.linalg.norm(P[-3:,:-6]),np.linalg.norm(P[:-6,:-6]),np.linalg.norm(P[-6:-3,-6:-3]),np.linalg.norm(P[-3:,-3:])])
                dynamic=config['lifetime']>0
                if dynamic or step==0: snapshot(t,x,P,active,'before_event')
                lc.camera(ids)
                for id in ids:
                    id=int(id); e=eventmap.setdefault(id,{'id':id,'observation_birth':t}); e['last_visible']=t
                for id in lc.ids:
                    e=eventmap.setdefault(id,{'id':id,'observation_birth':t,'last_visible':t}); e.setdefault('slot_birth',t)
                    if id in ids: e.setdefault('first_correction',t if tick else 1/chz)
                for id in set(old)-set(lc.ids): eventmap[id]['deleted']=t
                seeds=seeder.update(t,tick,ids,z,set(lc.ids)) if seeder else {}
                exact=sample(0,scene)[0] if step==0 and config['initialization']=='EXACT' else None
                if core:
                    core.camera(t,ids,z,Rbc,pc,seeds,exact); x,P,active=core.read()
                    if active!=lc.ids: raise AssertionError('independent lifecycle differs from CORE')
                else:
                    x,P,active=lc.x,lc.P,lc.ids
                    if exact is not None: x=exact
                    for id,seed in seeds.items(): s=active.index(id); x[3*s:3*s+3]=seed
                    use=[j for j,id in enumerate(ids) if int(id) in active]; slots=[active.index(int(ids[j])) for j in use]
                    _,_,C,y=matrices(np.zeros(3),np.zeros(3),z[use],Rbc,pc,len(active),slots)
                    x,P=correct(x,P,C,y,q,0 if tick==0 else 1/chz); lc.x,lc.P=x,P
                camera_records.append((t,'post',x.copy(),list(active)))
                if dynamic or step==0: snapshot(t,x,P,active,'after_event')
            if step%(hz//200)==0:
                record(t,x,active,core.diag if core else np.zeros(6))
            if step%hz==0: snapshot(t,x,P,active,'second')
        if core: core.close()
        if seeder: (run_dir/'seeds.json').write_text(json.dumps(seeder.events,indent=2))
    np.save(run_dir/'gain_diagnostics.npy',np.array(gain_records))
    padded_gain=np.full((len(gain_landmarks),30),np.nan)
    for j,item in enumerate(gain_landmarks): padded_gain[j,:len(item)]=item
    np.save(run_dir/'gain_landmark_contributions.npy',padded_gain)
    X=np.array(xs); ID=np.array(idsout); values=[]; landmarks=[]; axes=[]
    for k,t in enumerate(ts):
        ids=ID[k][ID[k]>=0]; dd=3*len(ids)+6; val,land,axis=errors(scene,t,X[k,:dd],ids)
        lp=np.full((30,6),np.nan); lp[:len(ids)]=land; values.append(val); landmarks.append(lp); axes.append(axis)
    values=np.array(values); landmarks=np.array(landmarks)
    np.savez_compressed(run_dir/'trace.npz',t=ts,x=X,ids=ID,values=values,landmarks=landmarks,axes=axes,diag=diagout)
    Pstack=np.array(ps); eig=[]; symmetry=[]
    for P,xx in zip(Pstack,px):
        dd=np.isfinite(xx).sum(); pp=P[:dd,:dd]; ev=np.linalg.eigvalsh((pp+pp.T)/2); scale=max(1.,abs(ev).max())
        symmetry.append(float(np.linalg.norm(pp-pp.T)/max(1,np.linalg.norm(pp))))
        eig.append([float(ev[0]),float(ev[-1]),float(ev[-1]/ev[0])])
        if impl in ['CONT_REF','SPLIT_REF'] and ev[0]<-1e-10*scale: raise AssertionError('reference P outside rounding boundary')
    np.savez_compressed(run_dir/'matrices.npz',t=pst,P=Pstack,ids=pids,x=px,tags=tags,eigen=eig,symmetry=symmetry)
    # Preserve pre/post states plus correction contributions and cross-block diagnostics.
    ct=[]; cs=[]; cv=[]; cl=[]; cx=[]; ci=[]
    for t,side,x,ids in camera_records:
        val,land,_=errors(scene,t,x,ids); lp=np.full((30,6),np.nan); lp[:len(ids)]=land
        xx=np.full(96,np.nan); xx[:len(x)]=x; ii=np.full(30,-1); ii[:len(ids)]=ids
        ct.append(t); cs.append(side); cv.append(val); cl.append(lp); cx.append(xx); ci.append(ii)
    np.savez_compressed(run_dir/'camera.npz',t=ct,side=cs,values=cv,landmarks=cl,x=cx,ids=ci)
    metrics=summarize(ts,values,landmarks,duration)
    metrics['camera']={}
    if config['lifetime']==0:
        from evaluate import settle
        metrics['fixed_per_point_settling']={label:[settle(ts,landmarks[:,j,1]<=threshold) for j in range(n)] for label,threshold in [('wide',.05),('strict',.02)]}
    cam_z=[]
    for tt in np.arange(round(duration*chz)+1)/chz:
        R,position,*_=truth(tt,scene); cam_z.extend((((points-position)@R-pc)@Rbc)[:,2])
    metrics['true_camera_Z']={'min':float(np.min(cam_z)),'negative_fraction':float(np.mean(np.array(cam_z)<0))}
    if scene in ['REGULAR','FAST'] and np.min(cam_z)<=0: raise AssertionError('forward visibility violated')
    for side in ['pre','post']:
        mask=np.array(cs)==side
        metrics['camera'][side]=summarize(np.array(ct)[mask],np.array(cv)[mask],np.array(cl)[mask],duration)
    if impl!='CONT_REF':
        tracks=lifecycle_metrics(ts,ID,landmarks,[e for e in eventmap.values() if 'slot_birth' in e],duration)
        (run_dir/'lifecycle.json').write_text(json.dumps(tracks,indent=2))
        metrics['lifecycle']={'slotted':len(tracks),'observed':len(eventmap),'residence_median':float(np.median([e['residence_duration'] for e in tracks])),'wide_success_before_loss':float(np.mean([e['wide']['success_before_loss'] for e in tracks])),'strict_success_before_loss':float(np.mean([e['strict']['success_before_loss'] for e in tracks])),'right_censored':sum(e['right_censored'] for e in tracks)}
        (run_dir/'observations.json').write_text(json.dumps(list(eventmap.values()),indent=2))
    metrics.update({'seconds':time.monotonic()-started,'input_sha':input_sha,'input_path':input_path,'physical_bearing_sha':hashlib.sha256(inputs['bearings'].tobytes()).hexdigest(),'nfev':nfev,'P_min':min(e[0] for e in eig),'P_max':max(e[1] for e in eig),'P_condition_max':max(e[2] for e in eig),'symmetry_max':max(symmetry),'substeps_max':float(np.array(diagout)[:,0].max()),'protection_count':float(np.array(diagout)[:,3].max()),'protection_norm':float(np.array(diagout)[:,4].max())})
    (run_dir/'metrics.json').write_text(json.dumps(metrics,indent=2,allow_nan=False))
    return metrics
