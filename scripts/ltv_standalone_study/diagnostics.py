"""Offline diagnostics from saved traces; never changes a simulation identity."""
import json
from pathlib import Path
import numpy as np
from scipy.integrate import cumulative_trapezoid
from truth_model import truth,geometry,sample

def geometry_diagnostics(scene,duration,out):
    t=np.arange(round(duration*200)+1)/200; points,Rbc,pc=geometry(scene)
    world=[]
    for tt in t:
        R,p,*_=truth(tt,scene); c=p+R@pc; d=points-c; d/=np.linalg.norm(d,axis=1)[:,None]; world.append(d)
    b=np.array(world); proj=np.eye(3)-b[:,:,:,None]*b[:,:,None,:]
    integ=cumulative_trapezoid(proj,t,axis=0,initial=0); values={}
    for window in [1,5,10]:
        tick=window*200; W=(integ[tick:]-integ[:-tick])/window; eig=np.linalg.eigvalsh(W)[:,:,0]
        values[f'window_{window}']=np.r_[np.full((tick,len(points)),np.nan),eig]
    np.savez_compressed(out/f'geometry_{scene}.npz',t=t,**values)
    return {name:{'minimum':float(np.nanmin(v)),'median':float(np.nanmedian(v)),'coverage':float(np.isfinite(v).mean())} for name,v in values.items()}

def compare_runs(a,b):
    xa=np.load(Path(a['directory'])/'trace.npz'); xb=np.load(Path(b['directory'])/'trace.npz')
    if not np.array_equal(xa['t'],xb['t']) or not np.array_equal(xa['ids'],xb['ids']): raise ValueError('incompatible comparison identity/time/slots')
    diff=xa['x']-xb['x']; d=3*np.sum(xa['ids'][0]>=0)+6
    result={'x_max_normalized':float(np.nanmax(np.linalg.norm(diff[:,:d],axis=1)/np.maximum(1,np.linalg.norm(xb['x'][:,:d],axis=1)))),'velocity_difference_rmse':float(np.sqrt(np.mean(np.sum(diff[:,-6:]**2,axis=1)))) if d==96 else float(np.sqrt(np.mean(np.sum(diff[:,d-6:d-3]**2,axis=1))))}
    # Correct the velocity slice for all dimensions (including PAPER).
    result['velocity_difference_rmse']=float(np.sqrt(np.mean(np.sum(diff[:,d-6:d-3]**2,axis=1))))
    result['gravity_difference_rmse']=float(np.sqrt(np.mean(np.sum(diff[:,d-3:d]**2,axis=1))))
    pa=np.load(Path(a['directory'])/'matrices.npz'); pb=np.load(Path(b['directory'])/'matrices.npz')
    def seconds(p):
        return {round(float(t),6):P[:d,:d] for t,P,tag in zip(p['t'],p['P'],p['tags']) if tag in ['second','continuous']}
    A,B=seconds(pa),seconds(pb); keys=sorted(set(A)&set(B)); result['P_max_relative_at_seconds']=max(float(np.linalg.norm(A[t]-B[t])/max(1,np.linalg.norm(B[t]))) for t in keys)
    return result

def overshoot(base,candidate):
    a=np.load(Path(base['directory'])/'trace.npz'); b=np.load(Path(candidate['directory'])/'trace.npz'); result={}
    for col,key in [(0,'v'),(1,'eta')]:
        av=a['values'][:,col]; bv=b['values'][:,col]; peak=float(av.max()); mask=bv>peak
        result[key]={'base_peak':peak,'candidate_peak':float(bv.max()),'peak_increase':float(bv.max()-peak),'peak_ratio':float(bv.max()/peak),'duration_above_C0_peak':float(np.trapz(mask.astype(float),b['t'])),'duration_above_C0_peak_after_10s':float(np.trapz(mask[b['t']>=10].astype(float),b['t'][b['t']>=10])),'rmse_full':float(np.sqrt(np.mean(bv*bv))),'rmse_tail':float(np.sqrt(np.mean(bv[b['t']>=50]**2)))}
    return result

def seed_summary(e):
    path=Path(e['directory'])/'seeds.json'
    if not path.exists(): return {}
    seeds=json.loads(path.read_text()); tracks=json.loads((path.parent/'lifecycle.json').read_text()); slotted=len(tracks)
    by_id={v['id']:v for v in tracks}
    return {'accepted':len(seeds),'slotted':slotted,'acceptance':len(seeds)/slotted,'delay_observed_median':float(np.median([v['delay'] for v in seeds])) if seeds else None,'delay_residence_median':float(np.median([v['t']-by_id[v['id']]['slot_birth'] for v in seeds])) if seeds else None,'seed_error_rmse':float(np.sqrt(np.mean([v['seed_error']**2 for v in seeds]))) if seeds else None}

def inspect_saved_numerics(e):
    rd=Path(e['directory']); c=e['config']; m=np.load(rd/'matrices.npz'); summaries=[]
    for time,x,P,ids,tag in zip(m['t'],m['x'],m['P'],m['ids'],m['tags']):
        ids=ids[ids>=0]; d=3*len(ids)+6; xx=x[:d]; pp=P[:d,:d]; true=sample(time,c['scene'],ids)[0]; error=true-xx
        U=float(error@np.linalg.solve(pp,error)); summaries.append([time,U,np.linalg.norm(pp[-6:-3,:-6]),np.linalg.norm(pp[-3:,:-6])])
    np.save(rd/'energy_cross_blocks.npy',np.array(summaries))
    result={'energy_snapshots':len(summaries),'min_U':float(np.min(np.array(summaries)[:,1]))}
    if c['impl']=='CONT_REF': result['largest_U_increase_at_seconds']=float(np.max(np.diff(np.array(summaries)[:,1])))
    return result

def dynamic_geometry(e,out):
    """Mask fixed-physical geometry by THIS ID's available history; never join IDs."""
    c=e['config']; geo=np.load(out/f"geometry_{c['scene']}.npz")
    obs=json.loads((Path(e['directory'])/'observations.json').read_text()); t=geo['t']; total=0; summaries={}
    n=len(geometry(c['scene'])[0])
    for window in [1,5,10]:
        vals=[]; count=0; eligible=0
        for track in obs:
            mask=(t>=track['observation_birth']-1e-9)&(t<=track['last_visible']+1e-9); count+=int(mask.sum())
            full=mask&(t>=track['observation_birth']+window-1e-9); vv=geo[f'window_{window}'][full,track['id']%n]
            vv=vv[np.isfinite(vv)]; eligible+=len(vv); vals.extend(vv)
        summaries[f'window_{window}']={'complete_samples':eligible,'observation_samples':count,'coverage':eligible/count if count else 0,'minimum':float(np.min(vals)) if vals else None,'median':float(np.median(vals)) if vals else None}
    return summaries

def repeated_transients(base,candidate):
    """Matched dynamic slots at common timestamps; full timeline, including startup."""
    result=overshoot(base,candidate)
    a=np.load(Path(base['directory'])/'trace.npz'); b=np.load(Path(candidate['directory'])/'trace.npz')
    assert np.array_equal(a['ids'],b['ids'])
    tracks=json.loads((Path(candidate['directory'])/'lifecycle.json').read_text())
    # Windows anchored at every accepted slot birth; overlap is explicit and never summed as time.
    windows=[]
    for t in sorted({v['slot_birth'] for v in tracks}):
        mask=(b['t']>=t)&(b['t']<=min(t+.2,b['t'][-1]))
        windows.append({'birth':t,'candidate_v_peak_next_02s':float(b['values'][mask,0].max()),'C0_v_peak_next_02s':float(a['values'][mask,0].max()),'candidate_eta_peak_next_02s':float(b['values'][mask,1].max()),'C0_eta_peak_next_02s':float(a['values'][mask,1].max())})
    result['birth_windows']=windows
    result['births_after_10s_with_larger_v_peak']=sum(w['birth']>=10 and w['candidate_v_peak_next_02s']>w['C0_v_peak_next_02s'] for w in windows)
    result['births_after_10s_total']=sum(w['birth']>=10 for w in windows)
    return result
