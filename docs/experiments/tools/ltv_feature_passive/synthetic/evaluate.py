"""Evaluator-only access to truth; no estimate or admission changes are possible."""
import os
for name in ['OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS']:
    os.environ[name]='1'
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np

def load_arrays(path, keys=None):
    # NpzFile indexing decompresses on every access; materialize each member once.
    with np.load(path,allow_pickle=False) as archive:
        return {key:archive[key] for key in (archive.files if keys is None else keys)}

def verify_input_identity(input_dir, run_identity):
    """Both sensor and evaluator labels must match the frozen run identity."""
    input_dir=Path(input_dir)
    identity=json.loads((input_dir/'identity.json').read_text())
    if identity != run_identity:
        raise RuntimeError('Run input identity differs from current frozen identity')
    for name,key in [('inputs.npz','input_sha'),('labels.npz','labels_sha')]:
        digest=hashlib.sha256((input_dir/name).read_bytes()).hexdigest()
        if digest != identity[key]:
            raise RuntimeError('Frozen input/label SHA mismatch: '+name)

def verify_camera_events(events, camera_times):
    if len(events)!=len(camera_times):
        raise RuntimeError('Camera event count mismatch')
    times=np.asarray([event['t'] for event in events],dtype=float)
    if not np.isfinite(times).all() or not np.array_equal(times,camera_times):
        raise RuntimeError('Camera event timestamps mismatch')
    if len(times)>1 and not np.all(np.diff(times)>0):
        raise RuntimeError('Nonmonotonic camera events')

def physical_ready_seconds(times, mask):
    """Causal sample holds to the next camera tick; final tick has zero support."""
    times=np.asarray(times,dtype=float);mask=np.asarray(mask,dtype=bool)
    if times.ndim!=1 or mask.shape!=times.shape or not np.isfinite(times).all():
        raise ValueError('Invalid readiness time support')
    if len(times)<2:return 0.
    if not np.all(np.diff(times)>0):raise ValueError('Nonmonotonic readiness times')
    return float(np.sum(np.diff(times)*mask[:-1]))

def rms(x):return float(np.sqrt(np.mean(np.asarray(x)**2))) if len(x) else None

def branch_metrics(x,truth,dimensions,mask=None):
    if mask is None:mask=np.ones(len(x),dtype=bool)
    v=np.array([a[d-6:d-3] for a,d in zip(x,dimensions)])
    eta=np.array([a[d-3:d] for a,d in zip(x,dimensions)])
    vg=truth[:,-6:-3];eg=truth[:,-3:]
    ve=np.linalg.norm(v-vg,axis=1);ee=np.linalg.norm(eta-eg,axis=1)
    norms=np.linalg.norm(eta,axis=1)
    angle=np.full(len(x),180.)
    good=norms>1e-12
    angle[good]=np.degrees(np.arccos(np.clip(np.sum(eta[good]*eg[good],axis=1)/(norms[good]*np.linalg.norm(eg[good],axis=1)),-1,1)))
    mag=abs(norms-np.linalg.norm(eg,axis=1))
    return {'samples':int(np.sum(mask)),'v_RMSE':rms(ve[mask]),'eta_RMSE':rms(ee[mask]),
            'g_angle_RMSE_deg':rms(angle[mask]),'g_magnitude_RMSE':rms(mag[mask]),
            'wide_v_accuracy':float(np.mean(ve[mask]<=.1)) if mask.any() else None,
            'wide_g_accuracy':float(np.mean((angle[mask]<=1)&(mag[mask]<=.2))) if mask.any() else None},(ve,ee,angle,mag)

def longest_false(times,mask):
    start=None;longest=0.
    if len(times)<2:return 0.
    for t,ok in zip(times,mask):
        if not ok and start is None:start=t
        if ok and start is not None:longest=max(longest,t-start);start=None
    if start is not None:longest=max(longest,times[-1]-start)
    return float(longest)

def first_hold(samples,threshold):
    # Every camera pre/post sample participates. Any tied-time failure breaks hold.
    grouped={}
    for t,error in samples:grouped[t]=grouped.get(t,True) and error<=threshold
    start=None
    for t,ok in sorted(grouped.items()):
        if not ok:start=None
        elif start is None:start=t
        if start is not None and t-start>=.1-1e-9:return float(start),float(t)
    return None,None

def evaluate(input_dir,run_dir,baseline=None):
    input_dir=Path(input_dir);run_dir=Path(run_dir)
    meta=json.loads((run_dir/'run.json').read_text())
    if meta['status']!='COMPLETED_ESTIMATION' or not meta['full_input_consumed']:raise RuntimeError('Incomplete estimator run')
    verify_input_identity(input_dir,meta['input_identity'])
    labels=load_arrays(input_dir/'labels.npz');inputs=load_arrays(input_dir/'inputs.npz');states=load_arrays(run_dir/'states.npz')
    if len(states['t'])!=len(labels['times']) or not np.array_equal(states['t'],labels['times']):raise RuntimeError('Truth time mismatch')
    events=[json.loads(line) for line in (run_dir/'events.jsonl').read_text().splitlines()]
    cam_times=labels['camera_times'];camera_ids=labels['feature_ids'];npoints=len(labels['points_W'])
    verify_camera_events(events,cam_times)
    if not np.array_equal(inputs['camera_times'],cam_times):raise RuntimeError('Input and label camera time mismatch')
    if not np.array_equal(inputs['feature_ids'],camera_ids):raise RuntimeError('Estimator observation IDs differ from evaluator identity map')
    if not np.array_equal(camera_ids%npoints,labels['physical_indices']):raise RuntimeError('ID encoding does not match independently saved physical slot map')
    tracks={}
    for k,(t,ids) in enumerate(zip(cam_times,camera_ids)):
        for slot,id in enumerate(ids):
            r=tracks.setdefault(int(id),{'id':int(id),'first_visible':float(t),'last_visible':float(t),'frames':0,'physical_index':int(labels['physical_indices'][k,slot])})
            if r['physical_index']!=int(labels['physical_indices'][k,slot]):raise RuntimeError('Identity switches physical point')
            r['last_visible']=float(t);r['frames']+=1
    opportunities={id for id,r in tracks.items() if r['frames']>=3 and r['last_visible']-r['first_visible']>=.1-1e-9}
    for r in tracks.values():r.update(opportunity=r['id'] in opportunities,right_censored=r['last_visible']==float(cam_times[-1]),accepted=False)
    seed_rows=[];seen=set();manager_opportunity=set();reasons={}
    for k,event in enumerate(events):
        for r in event['tracks']:
            if r.get('opportunity'):manager_opportunity.add(r['id'])
            reason=r.get('reason','unknown');reasons[reason]=reasons.get(reason,0)+1
        for birth in event['births']:
            id=birth['id']
            if id in seen:raise RuntimeError('Duplicate seed/admission identity')
            seen.add(id)
            if id not in tracks:raise RuntimeError('Admission of nonexistent observation ID')
            t=float(event['t']);physical=tracks[id]['physical_index'];tick=round(t*200)
            current_match=np.flatnonzero(camera_ids[k]==id)
            identity_matches=len(current_match)==1 and int(labels['physical_indices'][k,current_match[0]])==physical
            if not identity_matches:raise RuntimeError('Seed identity does not match current physical observation')
            gt=labels['state_body'][tick,3*physical:3*physical+3]
            ray=gt-inputs['p_BC'][0];ray/=np.linalg.norm(ray)
            seed=np.array(birth['seed_B']);error=seed-gt;relative=float(np.linalg.norm(error)/np.linalg.norm(gt))
            signed=float((seed-inputs['p_BC'][0])@ray)
            tracks[id].update(admitted=True,admission_t=t,accepted=bool(birth['apply_seed']))
            if birth['apply_seed']:
                row={'id':id,'t':t,'source':birth['source'],'relative_error':relative,'error_m':float(np.linalg.norm(error)),
                     'parallel_error_m':float(error@ray),'transverse_error_m':float(np.linalg.norm(error-(error@ray)*ray)),
                     'positive_true_ray':signed>0,'identity_matches':bool(identity_matches),'risk':birth['risk'],
                     'reliable_10pct':relative<=.1 and signed>0 and identity_matches,'reliable_5pct':relative<=.05 and signed>0 and identity_matches,
                     'waiting_seconds':t-tracks[id]['first_visible'],
                     'remaining_observed_seconds_EVALUATOR_ONLY':tracks[id]['last_visible']-t}
                seed_rows.append(row)
    if meta['method']!='OLD' and manager_opportunity!=opportunities:
        raise RuntimeError('Manager opportunity denominator disagrees with gate-independent observations')
    raw,errors=branch_metrics(states['x'],labels['state_body'],states['dimension'])
    indices=np.rint(cam_times*200).astype(int)
    g_ready=np.array([e['ready_G'] for e in events],dtype=bool);v_ready=np.array([e['ready_V'] for e in events],dtype=bool)
    g_metrics,_=branch_metrics(states['x'][indices],labels['state_body'][indices],states['dimension'][indices],g_ready)
    v_metrics,_=branch_metrics(states['x'][indices],labels['state_body'][indices],states['dimension'][indices],v_ready)
    baseline_metrics={}
    if baseline:
        old=load_arrays(Path(baseline)/'states.npz')
        oldmeta=json.loads((Path(baseline)/'run.json').read_text())
        if oldmeta['input_identity']['input_sha']!=meta['input_identity']['input_sha'] or not np.array_equal(old['t'],states['t']):raise RuntimeError('Baseline input/support mismatch')
        baseline_metrics['raw'],_=branch_metrics(old['x'],labels['state_body'],old['dimension'])
        baseline_metrics['same_ready_G'],_=branch_metrics(old['x'][indices],labels['state_body'][indices],old['dimension'][indices],g_ready)
        baseline_metrics['same_ready_V'],_=branch_metrics(old['x'][indices],labels['state_body'][indices],old['dimension'][indices],v_ready)
    # Landmark lifetime quality includes both correction sides and every physical tick,
    # ends at last actual observation, and never uses retained missing-frame duration.
    samples={id:[] for id in tracks};joint_samples={id:[] for id in tracks}
    def add_landmarks(times,xs,ids,ds):
        for t,x,idrow,d in zip(times,xs,ids,ds):
            tick=round(float(t)*200);truth=labels['state_body'][tick]
            ve=np.linalg.norm(x[d-6:d-3]-truth[-6:-3]);eta=x[d-3:d];eta_true=truth[-3:]
            norm=np.linalg.norm(eta);angle=180. if norm<1e-12 else np.degrees(np.arccos(np.clip(eta@eta_true/(norm*np.linalg.norm(eta_true)),-1,1)))
            vg=ve<=.1 and angle<=1 and abs(norm-np.linalg.norm(eta_true))<=.2
            for slot,id in enumerate(idrow[:(d-6)//3]):
                id=int(id)
                if id not in tracks or t>tracks[id]['last_visible']+1e-9:continue
                physical=tracks[id]['physical_index']
                gt=truth[3*physical:3*physical+3]
                relative=float(np.linalg.norm(x[3*slot:3*slot+3]-gt)/np.linalg.norm(gt))
                samples[id].append((float(t),relative));joint_samples[id].append((float(t),relative if vg else float('inf')))
    add_landmarks(states['t'],states['x'],states['ids'],states['dimension'])
    matrices=load_arrays(run_dir/'matrices.npz',('t','x','ids','dimension'))
    add_landmarks(matrices['t'],matrices['x'],matrices['ids'],matrices['dimension'])
    for id,r in tracks.items():
        for key,source,threshold in [('landmark_10pct',samples,.1),('landmark_5pct',samples,.05),('joint_wide_5pct',joint_samples,.05)]:
            start,confirmed=first_hold(source[id],threshold)
            r[key+'_first_age']=None if start is None else start-r['first_visible']
            r[key+'_confirmed_age']=None if confirmed is None else confirmed-r['first_visible']
            deadline=r['last_visible'];last=[error for t,error in source[id] if abs(t-deadline)<1e-9]
            last_pass=bool(last) and all(error<=threshold for error in last)
            r[key+'_last_visible_pass']=last_pass
            r[key+'_attained_and_last_visible_pass']=confirmed is not None and last_pass
            terminal=[(t,error) for t,error in source[id] if deadline-.1-1e-9<=t<=deadline+1e-9]
            r[key+'_terminal_hold']=last_pass and bool(terminal) and min(t for t,_ in terminal)<=deadline-.1+1e-9 and all(error<=threshold for _,error in terminal)
    closed=[r for r in tracks.values() if not r['right_censored']]
    reliable=sum(r['reliable_10pct'] for r in seed_rows)
    metrics={'status':'EVALUATED','method':meta['method'],'input_sha':meta['input_identity']['input_sha'],
        'raw':raw,'ready_G':g_metrics,'ready_V':v_metrics,'baseline':baseline_metrics,
        'coverage':{'camera_packages':len(events),'ready_G_fraction':float(g_ready.mean()),'ready_V_fraction':float(v_ready.mean()),
            'joint_fraction':float((g_ready&v_ready).mean()),'ready_G_seconds':physical_ready_seconds(cam_times,g_ready),'ready_V_seconds':physical_ready_seconds(cam_times,v_ready),
            'seconds_support':'left camera sample held until next tick; final sample has zero duration',
            'longest_G_gap_s':longest_false(cam_times,g_ready),'longest_V_gap_s':longest_false(cam_times,v_ready)},
        'seed':{'all_birth_tracks':len(tracks),'opportunity_tracks':len(opportunities),'accepted_seeds':len(seed_rows),
            'accepted_fraction_all':len(seed_rows)/len(tracks),'accepted_fraction_opportunity':len(seed_rows)/len(opportunities),
            'reliable_10pct_fraction':reliable/len(seed_rows) if seed_rows else None,
            'reliable_5pct_fraction':sum(r['reliable_5pct'] for r in seed_rows)/len(seed_rows) if seed_rows else None,
            'minimum_100_accepted':len(seed_rows)>=100,'unavailable_opportunity_tracks':len(tracks)-len(opportunities)},
        'lifecycle':{'admitted_tracks':len(seen),'completed_observation_tracks':len(closed),'right_censored_tracks':len(tracks)-len(closed),
            'landmark_5pct_before_last_visible':sum(r['landmark_5pct_attained_and_last_visible_pass'] for r in closed)/len(closed) if closed else None,
            'joint_wide_5pct_before_last_visible':sum(r['joint_wide_5pct_attained_and_last_visible_pass'] for r in closed)/len(closed) if closed else None,
            'landmark_5pct_terminal_hold':sum(r['landmark_5pct_terminal_hold'] for r in closed)/len(closed) if closed else None,
            'joint_wide_5pct_terminal_hold':sum(r['joint_wide_5pct_terminal_hold'] for r in closed)/len(closed) if closed else None,
            'mean_active':float(np.mean([e['state_features'] for e in events])),
            'mean_mature_visible':float(np.mean([e['mature_visible'] for e in events])),'rejection_reason_frames':reasons},
        'diagnostics':{'full_input_consumed':meta['full_input_consumed'],'elapsed_seconds':meta['elapsed_seconds'],
            'injection_G':sum(e['injection_G'] for e in events),'injection_V':sum(e['injection_V'] for e in events),
            'probabilistic_interval_claim':False}}
    (run_dir/'metrics.json').write_text(json.dumps(metrics,indent=2))
    (run_dir/'seed_evaluation.json').write_text(json.dumps(seed_rows,indent=2))
    (run_dir/'track_evaluation.json').write_text(json.dumps(list(tracks.values()),indent=2))
    np.savez_compressed(run_dir/'error_curves.npz',t=states['t'],velocity=errors[0],eta=errors[1],gravity_angle_deg=errors[2],gravity_magnitude=errors[3])
    return metrics

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--input',type=Path,required=True);p.add_argument('--run',type=Path,required=True);p.add_argument('--baseline',type=Path)
    a=p.parse_args();print(json.dumps(evaluate(a.input,a.run,a.baseline),indent=2))
