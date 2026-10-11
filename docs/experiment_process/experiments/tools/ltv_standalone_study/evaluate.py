"""Metrics never discard startup, invalid labels, or camera pre-update values."""
import numpy as np
from truth_model import sample,geometry,truth
MISSING='NOT_OBSERVED_WITHIN_HORIZON'
def settle(t,good,hold=5.):
    bad=np.flatnonzero(~good)
    start=(bad[-1]+1) if len(bad) else 0
    return float(t[start]) if start<len(t) and t[-1]-t[start]>=hold-1e-9 else MISSING

def errors(scene,t,x,ids):
    _,Rbc,pc=geometry(scene); n=len(ids); gt,a,w,z=sample(t,scene,ids)
    delta=x-gt; v=np.linalg.norm(delta[-6:-3]); eta=np.linalg.norm(delta[-3:]); en=np.linalg.norm(x[-3:])
    angle=np.degrees(np.arccos(np.clip(x[-3:]@gt[-3:]/(en*9.81),-1,1))) if en>1e-12 else np.nan
    e=np.linalg.norm(delta[:3*n].reshape(-1,3),axis=1)
    b=z@Rbc.T; r=x[:3*n].reshape(-1,3)-pc
    ray=np.einsum('ij,ij->i',r,b); transverse=np.linalg.norm(r-ray[:,None]*b,axis=1)
    rel=e/np.maximum(.1,np.linalg.norm(gt[:3*n].reshape(-1,3)-pc,axis=1))
    along=np.einsum('ij,ij->i',delta[:3*n].reshape(-1,3),b)
    return np.array([v,eta,angle,abs(en-9.81),np.linalg.norm(x[-6:-3]),np.linalg.norm(gt[-6:-3])]),np.array([e,rel,ray,transverse,np.linalg.norm(r,axis=1),along]).T,delta[-6:]

def summarize(t,values,landmarks,horizon):
    result={}; intervals=[(0,60),(0,10),(10,30),(30,60),(50,60)] if horizon>=60 else ([(0,20),(0,5),(5,20)] if horizon==20 else [(0,horizon)])
    if horizon>60: intervals+=[(60,horizon),(horizon-10,horizon)]
    for a,b in intervals:
        mask=(t>=a-1e-9)&(t<=b+1e-9); v=values[mask]; l=landmarks[mask]; norms=v[:,0]; angles=v[:,2]; valid=np.isfinite(angles)
        result[f'{a:g}-{b:g}']={'v_rmse':float(np.sqrt(np.mean(norms**2))),'v_p95':float(np.percentile(norms,95)),'v_max':float(norms.max()),'eta_rmse':float(np.sqrt(np.mean(v[:,1]**2))),'g_angle_rmse':float(np.sqrt(np.mean(angles[valid]**2))) if valid.any() else None,'g_angle_defined':int(valid.sum()),'g_angle_total':len(v),'g_magnitude_rmse':float(np.sqrt(np.mean(v[:,3]**2))),'speed_rms_ratio':float(np.sqrt(np.mean(v[:,4]**2)/np.mean(v[:,5]**2))) if np.mean(v[:,5]**2)>1e-24 else None,'landmark_rmse':float(np.sqrt(np.nanmean(l[:,:,0]**2))),'landmark_relative_p95':float(np.nanpercentile(l[:,:,1],95)),'negative_ray_fraction':float(np.nanmean(np.where(np.isfinite(l[:,:,2]),l[:,:,2]<0,np.nan))),'near_center_fraction':float(np.nanmean(np.where(np.isfinite(l[:,:,4]),l[:,:,4]<.1,np.nan))),'projection_rmse':float(np.sqrt(np.nanmean(l[:,:,3]**2)))}
    for name,vm,angle,mag,lm in [('wide',.1,1,.2,.05),('strict',.05,.5,.1,.02)]:
        vg=values[:,0]<=vm; gg=(values[:,2]<=angle)&(values[:,3]<=mag)
        lg=np.all(np.where(np.isfinite(landmarks[:,:,1]),landmarks[:,:,1]<=lm,True),axis=1)
        result[name]={'t_V':settle(t,vg),'t_G':settle(t,gg),'t_VG':settle(t,vg&gg),'t_L_all':settle(t,lg),'target_fraction_VG':float(np.mean(vg&gg))}
    tail=result.get('50-60',result[next(iter(result))]); result['J_tail']=tail['v_rmse']/.1+tail['eta_rmse']/.2
    return result

def lifecycle_metrics(t,ids,landmarks,events,horizon):
    tracks={int(e['id']):dict(e) for e in events}
    # Sample at common 200 Hz, not only at camera events. Stop eligibility at LAST VISIBLE.
    for k,time in enumerate(t):
        for j,id in enumerate(ids[k]):
            if id<0: continue
            e=tracks.setdefault(int(id),{'id':int(id),'observation_birth':float(time),'slot_birth':float(time),'last_visible':float(time)})
            e.setdefault('_samples',[]).append((float(time),float(landmarks[k,j,1])))
    for e in tracks.values():
        ss=e.pop('_samples',[]); end=e['last_visible']; visible=[(time,err) for time,err in ss if time<=end+1e-9]
        e['observed_duration']=end-e['observation_birth']; e['residence_duration']=e.get('deleted',horizon)-e.get('slot_birth',horizon)
        e['right_censored']=e.get('deleted') is None
        e['observation_right_censored']=abs(end-horizon)<1e-8
        for label,threshold in [('wide',.05),('strict',.02)]:
            start=None; first=None
            for time,err in visible:
                if err<=threshold:
                    if start is None: start=time
                    if time-start>=.1-1e-9 and first is None: first=start
                else: start=None
            e[label]={'first_held_observation_age':None if first is None else first-e['observation_birth'],'first_held_residence_age':None if first is None else first-e['slot_birth'],'at_last_visible':bool(visible and visible[-1][1]<=threshold),'success_before_loss':bool(first is not None and visible[-1][1]<=threshold)}
        # Values retained after loss are reported, but never used for success_before_loss.
        e['at_last_resident_relative_error']=ss[-1][1] if ss else None
    return list(tracks.values())
