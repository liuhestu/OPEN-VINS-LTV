"""Offline diagnostics from saved traces; never changes a simulation identity."""
import json
from pathlib import Path
import numpy as np
from scipy.integrate import cumulative_trapezoid
from truth_model import truth,geometry,sample

def load_arrays(path):
    """Decompress each NPZ member once before per-sample loops."""
    with np.load(path) as arrays:
        return {key:arrays[key] for key in arrays.files}

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
    base_peaks=peak_diagnostics(base); candidate_peaks=peak_diagnostics(candidate)
    for col,key in [(0,'v'),(1,'eta')]:
        av=a['values'][:,col]; bv=b['values'][:,col]; pk='v' if col==0 else 'eta_vector'; peak=base_peaks['full'][pk]; candidate_peak=candidate_peaks['full'][pk]; mask=bv>peak
        result[key]={'base_peak':peak,'candidate_peak':candidate_peak,'peak_increase':candidate_peak-peak,'peak_ratio':candidate_peak/peak,'duration_above_C0_peak':float(np.trapz(mask.astype(float),b['t'])),'duration_above_C0_peak_after_10s':float(np.trapz(mask[b['t']>=10].astype(float),b['t'][b['t']>=10])),'C0_peak_after_10s':float(av[a['t']>=10].max()),'candidate_peak_after_10s':float(bv[b['t']>=10].max()),'duration_above_late_C0_peak_after_10s':float(np.trapz((bv[b['t']>=10]>av[a['t']>=10].max()).astype(float),b['t'][b['t']>=10])),'rmse_full':float(np.sqrt(np.mean(bv*bv))),'rmse_tail':float(np.sqrt(np.mean(bv[b['t']>=50]**2)))}
    return result

def seed_summary(e):
    path=Path(e['directory'])/'seeds.json'
    if not path.exists(): return {}
    seeds=json.loads(path.read_text()); tracks=json.loads((path.parent/'lifecycle.json').read_text()); slotted=len(tracks)
    by_id={v['id']:v for v in tracks}
    return {'accepted':len(seeds),'slotted':slotted,'acceptance':len(seeds)/slotted,'delay_observed_median':float(np.median([v['delay'] for v in seeds])) if seeds else None,'delay_residence_median':float(np.median([v['t']-by_id[v['id']]['slot_birth'] for v in seeds])) if seeds else None,'seed_error_rmse':(0. if e['config']['initialization']=='GT_MATCHED' else float(np.sqrt(np.mean([v['seed_error']**2 for v in seeds])))) if seeds else None,'paired_geometry_error_rmse':float(np.sqrt(np.mean([v['seed_error']**2 for v in seeds]))) if seeds else None}

def initialization_covariance_parity(base, initialized):
    """Check all saved P snapshots against the same zero-seed simulation."""
    with np.load(Path(base['directory'])/'matrices.npz') as a, np.load(Path(initialized['directory'])/'matrices.npz') as b:
        checks={key:bool(np.array_equal(a[key],b[key],equal_nan=True)) for key in ['t','ids','P']}
        checks['tags']=bool(np.array_equal(a['tags'],b['tags']))
        result={'baseline':base['id'],'initialized':initialized['id'],'snapshots':len(a['t']),'exact_equal':checks,'pass':all(checks.values())}
    if not result['pass']: raise AssertionError(f'Initialization changed covariance or lifecycle: {result}')
    return result

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
    c=e['config']; geo=load_arrays(out/f"geometry_{c['scene']}.npz")
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
    a=load_arrays(Path(base['directory'])/'trace.npz'); b=load_arrays(Path(candidate['directory'])/'trace.npz')
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

def track_outcomes(e):
    """Separate observed-ended and right-censored tracks; add joint v/g/point outcomes."""
    rd=Path(e['directory']); trace=load_arrays(rd/'trace.npz'); path=rd/'lifecycle.json'
    if not path.exists(): return {}
    tracks=json.loads(path.read_text()); lookup={v['id']:v for v in tracks}; rows={v['id']:[] for v in tracks}
    for k,t in enumerate(trace['t']):
        val=trace['values'][k]
        for j,id in enumerate(trace['ids'][k]):
            if id<0: continue
            track=lookup[int(id)]
            if t<=track['last_visible']+1e-9:
                rows[int(id)].append((t,trace['landmarks'][k,j,1],val[0],val[2],val[3]))
    for track in tracks:
        samples=rows[track['id']]
        for grade,vm,gm,mag,lm in [('wide',.1,1,.2,.05),('strict',.05,.5,.1,.02)]:
            start=None; first=None; last=False
            for t,l,v,g,m in samples:
                last=bool(l<=lm and v<=vm and g<=gm and m<=mag)
                if last:
                    if start is None: start=t
                    if t-start>=.1-1e-9 and first is None: first=start
                else: start=None
            first_age=None if first is None else float(first-track['observation_birth'])
            track['joint_'+grade]={'first_held_observation_age':first_age,'confirmation_observation_age':None if first_age is None else first_age+.1,'at_last_visible':last,'success_by_last_visible':bool(first is not None and last),'terminal_held_success':bool(last and start is not None and track['last_visible']-start>=.1-1e-9)}
            landmark_start=None
            for t,l,_,_,_ in samples:
                if l<=lm:
                    if landmark_start is None: landmark_start=t
                else: landmark_start=None
            track[grade]['terminal_held_success']=bool(landmark_start is not None and track['last_visible']-landmark_start>=.1-1e-9)
            age=track[grade]['first_held_observation_age']
            track[grade]['confirmation_observation_age']=None if age is None else age+.1
    completed=[t for t in tracks if not t['observation_right_censored']]
    def rate(tracks,key,field): return float(np.mean([t[key][field] for t in tracks])) if tracks else None
    result={'slotted':len(tracks),'completed_observation_tracks':len(completed),'observation_right_censored':len(tracks)-len(completed)}
    for grade in ['wide','strict']:
        result[grade]={'landmark_success_completed':rate(completed,grade,'success_before_loss'),'joint_success_completed':rate(completed,'joint_'+grade,'success_by_last_visible'),'landmark_success_all_by_horizon':rate(tracks,grade,'success_before_loss'),'joint_success_all_by_horizon':rate(tracks,'joint_'+grade,'success_by_last_visible'),'landmark_terminal_success_completed':rate(completed,grade,'terminal_held_success'),'joint_terminal_success_completed':rate(completed,'joint_'+grade,'terminal_held_success')}
    (rd/'joint_outcomes.json').write_text(json.dumps(tracks,indent=2))
    return result

def combined_settling(e):
    """Conservative fixed-ID settling includes every common sample AND camera pre/post."""
    from evaluate import settle
    rd=Path(e['directory']); c=e['config']
    if c['lifetime']>0: return {}
    trace=np.load(rd/'trace.npz'); camera=np.load(rd/'camera.npz')
    times=np.r_[trace['t'],camera['t']]; order=np.argsort(times,kind='stable'); times=times[order]
    values=np.r_[trace['values'],camera['values']][order]
    landmarks=np.r_[trace['landmarks'],camera['landmarks']][order]
    results={}; n=len(geometry(c['scene'])[0])
    unique,index=np.unique(times,return_index=True)
    def all_settle(good):
        return settle(unique,np.logical_and.reduceat(good,index))
    for name,vm,gm,mag,lm in [('wide',.1,1,.2,.05),('strict',.05,.5,.1,.02)]:
        vg=(values[:,0]<=vm)&(values[:,2]<=gm)&(values[:,3]<=mag)
        # Empty pre-lifecycle slots at t=0 do not certify landmark convergence.
        lg=landmarks[:,:n,1]<=lm
        results[name]={'t_V_all_samples':all_settle(values[:,0]<=vm),'t_G_all_samples':all_settle((values[:,2]<=gm)&(values[:,3]<=mag)),'t_VG_all_samples':all_settle(vg),'t_L_all_samples':all_settle(np.all(lg,axis=1)),'per_point_all_samples':[all_settle(lg[:,j]) for j in range(n)]}
    return results


def noise_response(clean,noisy):
    a=np.load(Path(clean['directory'])/'trace.npz'); b=np.load(Path(noisy['directory'])/'trace.npz')
    assert np.array_equal(a['t'],b['t']) and np.array_equal(a['ids'],b['ids'])
    dx=b['x']-a['x']; result={}
    for name,mask in [('full',a['t']>=0),('tail',a['t']>=50)]:
        result[name]={'velocity_response_rmse':float(np.sqrt(np.mean(np.sum(dx[mask,-6:-3]**2,axis=1)))),'gravity_response_rmse':float(np.sqrt(np.mean(np.sum(dx[mask,-3:]**2,axis=1)))),'landmark_response_rmse':float(np.sqrt(np.mean(np.sum(dx[mask,:90].reshape(-1,30,3)**2,axis=2))))}
    return result

def verified_events(e):
    """Resolve the t=0 zero-length correction edge from saved observation/slot times."""
    rd=Path(e['directory']); path=rd/'observations.json'
    if not path.exists(): return {}
    observations=json.loads(path.read_text()); corrected=0
    for track in observations:
        actual=None
        if 'slot_birth' in track:
            first=max(track['slot_birth'],1/e['config']['camera_hz'])
            if first<=track['last_visible']+1e-9: actual=first
        if track.get('first_correction')!=actual: corrected+=1
        track['first_correction']=actual
        track['first_correction_is_nonzero_subflow']=actual is not None
    (rd/'events_verified.json').write_text(json.dumps(observations,indent=2))
    return {'records':len(observations),'initial_planned_time_resolved_to_actual_event':corrected}

def initialization_snapshots(e):
    """Exact offline reconstruction at the hook: lifecycle is only a permutation/copy.

    This is not another observer run. Input state/P are saved event records. The
    hook cannot access P; store one shared P for before/after the mean write.
    """
    c=e['config']; rd=Path(e['directory'])
    if c['initialization'] not in ['GT_SEED','GEOM_IDEAL','GT_MATCHED','GEOM_NOISY']: return {}
    from geometry_seed import Seeder
    from truth_model import ids_at
    metrics=json.loads((rd/'metrics.json').read_text()); inputs=load_arrays(metrics['input_path'])
    pts,Rbc,pc=geometry(c['scene']); seeder=Seeder(c['scene'],c['initialization'],Rbc,pc,inputs['priors'])
    matrices=load_arrays(rd/'matrices.npz'); pre=np.flatnonzero(matrices['tags']=='before_event'); post=np.flatnonzero(matrices['tags']=='after_event')
    saved_events=json.loads((rd/'seeds.json').read_text()); snapshots=[]; before=[]; after=[]; cov=[]; ids_saved=[]; seeded_ids=set()
    for a,b in zip(pre,post):
        t=float(matrices['t'][a]); assert t==matrices['t'][b]
        old=matrices['ids'][a]; old=old[old>=0].tolist(); new=matrices['ids'][b]; new=new[new>=0].tolist(); tick=round(t*c['camera_hz'])
        seeds=seeder.update(t,tick,ids_at(tick,c['camera_hz'],c['lifetime'],len(pts)),inputs['bearings'][tick],set(new))
        if not seeds: continue
        assert not seeded_ids.intersection(seeds); seeded_ids.update(seeds)
        do,dn=3*len(old)+6,3*len(new)+6; x0=matrices['x'][a,:do]; P0=matrices['P'][a,:do,:do]
        mapping=[]
        for id in new: mapping+=list(range(3*old.index(id),3*old.index(id)+3)) if id in old else [-1]*3
        mapping+=list(range(do-6,do)); good=np.flatnonzero(np.array(mapping)>=0); src=np.array(mapping)[good]
        x=np.zeros(dn); P=np.zeros((dn,dn)); x[good]=x0[src]; P[np.ix_(good,good)]=P0[np.ix_(src,src)]
        for j,id in enumerate(new):
            if id not in old: P[3*j:3*j+3,3*j:3*j+3]=c['p']*np.eye(3)
        changed=x.copy()
        for id,seed in seeds.items(): j=new.index(id); changed[3*j:3*j+3]=seed
        assert np.array_equal(changed[-6:],x[-6:])
        xp=np.full(96,np.nan); xp[:dn]=x; yp=np.full(96,np.nan); yp[:dn]=changed; pp=np.full((96,96),np.nan); pp[:dn,:dn]=P; ip=np.full(30,-1); ip[:len(new)]=new
        snapshots.append(t); before.append(xp); after.append(yp); cov.append(pp); ids_saved.append(ip)
    assert [(v['id'],v['t']) for v in seeder.events]==[(v['id'],v['t']) for v in saved_events]
    np.savez_compressed(rd/'initialization_snapshots_reconstructed.npz',t=snapshots,x_before=before,x_after=after,P_shared=cov,ids=ids_saved)
    return {'source':'exact reconstruction from saved pre-event P/x, actual post-event slot IDs and frozen seed inputs; not an extra simulation','seed_writes':len(seeded_ids),'events':len(snapshots),'unique_once':True,'vg_means_unchanged_at_hook':True,'P_is_shared_before_after':True}

def camera_correction_deltas(e):
    """Actual net camera correction from saved endpoints, removing lifecycle/seed writes."""
    c=e['config']; rd=Path(e['directory'])
    if c['impl']=='CONT_REF': return {}
    camera=load_arrays(rd/'camera.npz'); trace=load_arrays(rd/'trace.npz')
    pre=np.flatnonzero(camera['side']=='pre'); post=np.flatnonzero(camera['side']=='post'); seeded={}
    hook_path=rd/'initialization_snapshots_reconstructed.npz'
    if hook_path.exists():
        h=load_arrays(hook_path)
        seeded={round(float(t),9):x for t,x in zip(h['t'],h['x_after'])}
    times=[]; deltas=[]; idsout=[]; previous=0.
    for a,b in zip(pre,post):
        t=float(camera['t'][a]); assert t==camera['t'][b]
        old=camera['ids'][a]; old=old[old>=0].astype(int).tolist(); new=camera['ids'][b]; new=new[new>=0].astype(int).tolist()
        do,dn=3*len(old)+6,3*len(new)+6; before=np.zeros(dn); before[-6:]=camera['x'][a,do-6:do]
        for j,id in enumerate(new):
            if id in old: s=old.index(id); before[3*j:3*j+3]=camera['x'][a,3*s:3*s+3]
        if round(t,9) in seeded: before=seeded[round(t,9)][:dn]
        dx=camera['x'][b,:dn]-before
        if t==0: dx=np.zeros(dn)  # First event only initializes, with zero correction length.
        row=np.full(96,np.nan); row[:dn]=dx
        times.append(t); deltas.append(row); idsout.append(camera['ids'][b]); previous=t
    delta=np.array(deltas); np.savez_compressed(rd/'camera_correction_deltas.npz',t=times,delta_x=delta,ids=idsout)
    v=[]; g=[]
    for dx,ids in zip(delta,idsout):
        d=3*int(np.sum(ids>=0))+6; v.append(np.linalg.norm(dx[d-6:d-3])); g.append(np.linalg.norm(dx[d-3:d]))
    return {'source':'actual camera endpoint difference, excluding slot rebuild and one-time mean seed; not a replay','maximum_velocity_correction':float(max(v)),'maximum_gravity_correction':float(max(g))}

def reported_settling(e):
    """t=0 is AFTER establishing x(0); keep pre-establishment data as audit only.

    The frozen C* selector concerns ZERO initialization, for which this boundary
    makes no difference. Accurate full-state initialization needs explicit handling.
    """
    result=combined_settling(e)
    if e['config']['initialization']!='EXACT': return result
    trace=np.load(Path(e['directory'])/'trace.npz'); camera=np.load(Path(e['directory'])/'camera.npz')
    mask=(camera['t']>0)|(camera['side']=='post')
    values=np.r_[trace['values'],camera['values'][mask]]; landmarks=np.r_[trace['landmarks'],camera['landmarks'][mask]]
    n=len(geometry(e['config']['scene'])[0])
    for grade,vm,gm,mag,lm in [('wide',.1,1,.2,.05),('strict',.05,.5,.1,.02)]:
        v=values[:,0]<=vm; g=(values[:,2]<=gm)&(values[:,3]<=mag); land=landmarks[:,:n,1]<=lm
        for key,good in [('t_V_all_samples',v),('t_G_all_samples',g),('t_VG_all_samples',v&g),('t_L_all_samples',np.all(land,axis=1))]:
            if np.all(good) and trace['t'][-1]>=5: result[grade][key]=0.
        for j in range(n):
            if np.all(land[:,j]) and trace['t'][-1]>=5: result[grade]['per_point_all_samples'][j]=0.
    return result

def peak_diagnostics(e):
    rd=Path(e['directory']); trace=np.load(rd/'trace.npz'); camera=np.load(rd/'camera.npz')
    valid=(camera['t']>0)|(camera['side']=='post')
    values=np.r_[trace['values'],camera['values'][valid]]; land=np.r_[trace['landmarks'],camera['landmarks'][valid]]
    t=np.r_[trace['t'],camera['t'][valid]]
    result={}
    for label,mask in [('full',t>=0),('after_10s',t>=10)]:
        vals=values[mask]; ll=land[mask]
        result[label]={'v':float(np.max(vals[:,0])),'eta_vector':float(np.max(vals[:,1])),'eta_direction':float(np.nanmax(vals[:,2])),'eta_magnitude':float(np.max(vals[:,3])),'landmark_3D':float(np.nanmax(ll[:,:,0])),'landmark_relative':float(np.nanmax(ll[:,:,1]))}
    result['basis']='maximum over all 200 Hz and camera pre/post checks; pre-establishment t=0 audit excluded'
    return result
