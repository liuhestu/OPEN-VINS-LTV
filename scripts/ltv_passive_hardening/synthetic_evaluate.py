"""Offline labels-only evaluator for hardening; never used by the estimator.

Camera receipts are the measurement support. 200 Hz saved snapshots are held
between events and MUST NOT be compared to current 200 Hz truth as new output.
"""
import os
for name in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS'):
    os.environ[name]='1'
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[2]
DOC=ROOT/'docs/ltv/passive_hardening_v2'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def episodes(t, mask):
    """Adjacent checked samples, inclusive endpoints; never claim held final time."""
    t=np.asarray(t,float);mask=np.asarray(mask,bool)
    if len(t)!=len(mask) or (len(t)>1 and np.any(np.diff(t)<=0)):
        raise ValueError('Invalid episode timeline')
    result=[];start=None
    for k,yes in enumerate(mask):
        if yes and start is None:start=k
        if start is not None and (not yes or k==len(mask)-1):
            end=k if yes else k-1
            result.append({'start':float(t[start]),'end':float(t[end]),'duration':float(t[end]-t[start]),'samples':end-start+1})
            start=None
    return result


def coverage(t,mask):
    mask=np.asarray(mask,bool);eps=episodes(t,mask);gaps=episodes(t,~mask)
    dt=np.diff(t)
    return {'samples':int(mask.sum()),'fraction':float(mask.mean()) if len(mask) else None,
            'left_sample_seconds':float(np.sum(dt*mask[:-1])),'episodes':eps,
            'longest_episode_seconds':max((e['duration'] for e in eps),default=0.),
            'longest_gap_seconds':max((e['duration'] for e in gaps),default=0.)}


def errors(v,g,truth):
    v=np.asarray(v,float);g=np.asarray(g,float);truth=np.asarray(truth,float)
    ev=np.linalg.norm(v-truth[:,:3],axis=1);eg=np.linalg.norm(g-truth[:,3:],axis=1)
    gn=np.linalg.norm(g,axis=1);tn=np.linalg.norm(truth[:,3:],axis=1)
    valid=np.isfinite(g).all(axis=1)&(gn>1e-12)&(tn>1e-12)
    # An undefined estimated gravity direction is invalid, not a perfect angle.
    angle=np.full(len(v),np.nan)
    angle[valid]=np.degrees(np.arccos(np.clip(np.sum(g[valid]*truth[valid,3:],axis=1)/(gn[valid]*tn[valid]),-1,1)))
    return dict(v=ev,eta=eg,angle=angle,norm=np.abs(gn-tn),gravity_norm=gn)


def summary(e,mask):
    mask=np.asarray(mask,bool);out={'samples':int(mask.sum())}
    for key in ('v','eta','angle','norm'):
        selected=e[key][mask];good=np.isfinite(selected)
        out[key]={'finite_samples':int(good.sum()),'invalid_samples':int((~good).sum()),
                  'RMSE':float(np.sqrt(np.mean(selected[good]**2))) if good.any() else None,
                  'P95':float(np.quantile(selected[good],.95)) if good.any() else None}
    def fraction(test):return float(np.mean(test[mask])) if mask.any() else None
    out['wide_v_fraction']=fraction(e['v']<=.10)
    out['wide_g_fraction']=fraction((e['angle']<=1)&(e['norm']<=.20))
    out['severe_v_fraction']=fraction(e['v']>1)
    out['severe_g_fraction']=fraction((e['angle']>5)|~np.isfinite(e['angle']))
    return out


def recovery_metrics(t,joint,e,phase,supply=None,run_start=None):
    t=np.asarray(t);phase=np.asarray(phase)
    if not np.any(phase==2):return {'applicable':False}
    if supply is None:return {'applicable':True,'pass':False,'episodes':[{'status':'UNVERIFIED_RELIABLE_SUPPLY_LABEL'}]}
    supply=np.asarray(supply,bool)
    if run_start is not None:
        expected=np.full(len(t),-1.)
        for ep in episodes(t,supply):expected[(t>=ep['start'])&(t<=ep['end'])]=ep['start']
        if not np.allclose(expected,run_start,rtol=0,atol=1e-9):raise ValueError('Reliable supply run_start mismatch')
    wide=np.asarray(joint)&(e['v']<=.1)&(e['angle']<=1)&(e['norm']<=.2)
    result=[]
    for supply_ep in episodes(t,supply&(phase==2)):
        start,end=supply_ep['start'],supply_ep['end'];deadline=start+10
        eps=episodes(t,wide&(t>=start)&(t<=end))
        qualifying=[ep for ep in eps if ep['duration']>=5-1e-9 and ep['start']+5<=deadline+1e-9]
        status='PASS' if qualifying else ('SUPPLY_INTERRUPTED_BEFORE_DEADLINE' if end<deadline-1e-9 else 'FAILED_RECOVERY_DEADLINE')
        result.append({'supply_start':start,'supply_end':end,'deadline':deadline,'status':status,
                       'pass':bool(qualifying),'first_interval':qualifying[0] if qualifying else None})
    return {'applicable':True,'episodes':result,'pass':bool(result) and all(x['pass'] for x in result),
            'status':'EVALUATED' if result else 'NO_SUFFICIENT_SUPPLY'}


def aligned(events,labels):
    t=np.array([x['t'] for x in events]);lt=np.asarray(labels['times']);idx=np.searchsorted(lt,t)
    if np.any(idx>=len(lt)) or not np.allclose(lt[idx],t,rtol=0,atol=1e-9):raise ValueError('Truth time mismatch')
    current=np.array([bool(x['raw_current']) for x in events]);v=np.full((len(t),3),np.nan);g=v.copy()
    for k,row in enumerate(events):
        if current[k]:
            if row['raw_v'] is None or row['raw_eta'] is None or abs(row['cursor']-row['t'])>1e-9:
                raise ValueError('Current raw output missing or stale')
            v[k]=row['raw_v'];g[k]=row['raw_eta']
            if not np.isfinite(v[k]).all() or not np.isfinite(g[k]).all():raise ValueError('Current output nonfinite')
        if (row['ready_G'] or row['ready_V']) and not current[k]:raise ValueError('Ready without current raw')
    return t,current,errors(v,g,np.asarray(labels['velocity_gravity_body'])[idx])


def seed_metrics(events,inputs,labels):
    points=dict(zip(map(int,labels['point_ids']),labels['points_W']))
    opportunities={source:set(map(int,labels[key][:,1])) for source,key in
                   [('STEREO','stereo_opportunities'),('TEMPORAL_POSE','temporal_opportunities')]}
    births=set(map(int,labels['births'][:,1]));records=[];used=set()
    birth_time={int(fid):float(inputs['camera_times'][int(k)]) for k,fid in labels['births']}
    last_visible={}
    for k,time in enumerate(inputs['camera_times']):
        a,b=inputs['observation_offsets'][k:k+2]
        for fid in inputs['feature_ids'][a:b][inputs['camera_ids'][a:b]==0]:last_visible[int(fid)]=float(time)
    for k,event in enumerate(events):
        for seed in event['seeds']:
            if not seed['admitted'] or not seed['mean_written']:continue
            fid=int(seed['id']);key=(event['epoch'],fid)
            if key in used:raise ValueError('Repeated seed in same observer epoch')
            used.add(key);source=seed['source']
            if source not in opportunities or fid not in points:raise ValueError('Unknown seed identity/source')
            truth=labels['R_WB'][k].T@(points[fid]-labels['p_WB'][k]);estimate=np.asarray(seed['landmark_B'])
            error=estimate-truth;ray=truth-inputs['p_BC'][0];ray=ray/np.linalg.norm(ray)
            parallel=float(error@ray);relative=float(np.linalg.norm(error)/np.linalg.norm(truth))
            positive=bool(seed['min_ray']>0 and (estimate-inputs['p_BC'][0])@ray>0)
            first=k if source=='STEREO' else max(0,k-20)
            valid=True
            for j in range(first,k+1):
                a,b=inputs['observation_offsets'][j:j+2]
                ids=inputs['feature_ids'][a:b];cams=inputs['camera_ids'][a:b]
                relevant=(ids==fid)&((cams<=1) if source=='STEREO' else (cams==0))
                valid=valid and not bool(np.any(labels['wrong_match'][a:b][relevant]) or np.any(labels['wrong_time'][a:b][relevant]))
            records.append({'id':fid,'epoch':event['epoch'],'time':event['t'],'source':source,'ID_match_correct':valid,
                            'positive_ray':positive,'relative_3d_error':relative,'parallel_error_m':parallel,
                            'transverse_error_m':float(np.linalg.norm(error-parallel*ray)),
                            'reliable10':bool(valid and positive and relative<=.1),'reliable5':bool(valid and positive and relative<=.05),
                            'predicted_relative_risk':seed['relative_risk'],'sigma_parallel':seed['sigma_parallel'],
                            'waiting_seconds':float(event['t']-birth_time[fid]),'remaining_visible_seconds':float(last_visible[fid]-event['t'])})
    out={}
    for source,opp in opportunities.items():
        rows=[r for r in records if r['source']==source];accepted={r['id'] for r in rows}
        out[source]={'accepted_writes':len(rows),'unique_accepted_ids':len(accepted),'opportunity_ids':len(opp),
                     'accepted_opportunity_ids':len(accepted&opp),'all_birth_ids':len(births),
                     'opportunity_acceptance':len(accepted&opp)/len(opp) if opp else None,
                     'all_birth_acceptance':len(accepted&births)/len(births) if births else None,
                     'reliable10':float(np.mean([r['reliable10'] for r in rows])) if rows else None,
                     'reliable5':float(np.mean([r['reliable5'] for r in rows])) if rows else None,
                     'parallel_RMSE':float(np.sqrt(np.mean([r['parallel_error_m']**2 for r in rows]))) if rows else None,
                     'transverse_RMSE':float(np.sqrt(np.mean([r['transverse_error_m']**2 for r in rows]))) if rows else None}
        out[source]['normal_source_pass']=bool(len(rows)>=100 and out[source]['reliable10']>=.9 and out[source]['opportunity_acceptance'] is not None and out[source]['opportunity_acceptance']>=.2)
    return out,records


def regression_metric(new, previous, support, floor, times=None):
    """Compare finite evidence without hiding an asymmetric undefined direction."""
    new=np.asarray(new,float);previous=np.asarray(previous,float);support=np.asarray(support,bool)
    invalid_new=support&~np.isfinite(new);invalid_previous=support&~np.isfinite(previous)
    common=support&np.isfinite(new)&np.isfinite(previous)
    a=float(np.sqrt(np.mean(new[common]**2))) if common.any() else None
    b=float(np.sqrt(np.mean(previous[common]**2))) if common.any() else None
    identical_invalid=bool(np.array_equal(invalid_new,invalid_previous))
    numeric=bool(a is not None and b is not None and a-b<=max(.05*b,floor))
    timeline=np.arange(len(new)) if times is None else np.asarray(times)
    return {'new':a,'previous':b,'requested_samples':int(support.sum()),'common_finite_samples':int(common.sum()),
            'excluded_invalid_samples':int((support&~common).sum()),
            'new_invalid_times':timeline[invalid_new].tolist(),'previous_invalid_times':timeline[invalid_previous].tolist(),
            'identical_invalid_support':identical_invalid,'numeric_pass_on_common_finite':numeric,
            'pass':bool(identical_invalid and numeric)}


def evaluate(input_dir,run_dir,out,baseline=None):
    input_dir,run_dir,out=map(Path,(input_dir,run_dir,out))
    if out.exists():raise FileExistsError(out)
    identity=json.loads((input_dir/'identity.json').read_text());run=json.loads((run_dir/'run.json').read_text())
    if sha(input_dir/'inputs.npz')!=identity['input_sha'] or sha(input_dir/'labels.npz')!=identity['labels_sha'] or run['input_sha']!=identity['input_sha']:
        raise ValueError('Input/label identity mismatch')
    if run['status']!='COMPLETED_ESTIMATION':raise ValueError('Cannot evaluate failed estimator as completed')
    with np.load(input_dir/'labels.npz',allow_pickle=False) as archive:labels={key:archive[key] for key in archive.files}
    with np.load(input_dir/'inputs.npz',allow_pickle=False) as archive:inputs={key:archive[key] for key in archive.files}
    events=[json.loads(x) for x in (run_dir/'events.jsonl').read_text().splitlines()]
    if [x['receipt_id'] for x in events]!=list(range(1,len(events)+1)):raise ValueError('Missing/duplicate/out-of-order receipt')
    if sha(run_dir/'states.npz')!=run['states_sha']:raise ValueError('State evidence hash mismatch')
    t,raw,e=aligned(events,labels)
    if len(t)!=len(labels['camera_times']) or not np.allclose(t,labels['camera_times'],rtol=0,atol=1e-9):raise ValueError('All camera input receipts required')
    if not run['full_input_consumed']:raise ValueError('Partial input cannot satisfy full evaluation')
    readyG=np.array([x['ready_G'] for x in events],bool);readyV=np.array([x['ready_V'] for x in events],bool)
    masks={'raw':raw,'ready_G':readyG,'ready_V':readyV,'joint_ready':readyG&readyV}
    report={'status':'EVALUATED','run':str(run_dir),'input_identity':identity,'mode':run['mode'],
            'support':'all camera receipts; inclusive checked episode endpoints, left-sample duration; held 200Hz snapshots excluded',
            'raw_missing_samples':int((~raw).sum()),'raw_missing_fraction':float((~raw).mean()),
            'supports':{key:{'errors':summary(e,mask),'coverage':coverage(t,mask)} for key,mask in masks.items()}}
    report['gravity_ready_physical_valid']=bool(np.all(np.isfinite(e['gravity_norm'][readyG])&(e['gravity_norm'][readyG]>=8)&(e['gravity_norm'][readyG]<=11.5)))
    report['severe_ready_pass']={'V':bool(readyV.any() and np.mean(e['v'][readyV]>1)<=.01),
                                 'G':bool(readyG.any() and np.mean((e['angle'][readyG]>5)|~np.isfinite(e['angle'][readyG]))<=.01)}
    report['absolute_contract']={}
    for name,key,rmse,p95 in [('ready_G','angle',1.,2.),('ready_V','v',.2,.5)]:
        support=report['supports'][name];metric=support['errors'][key];cov=support['coverage']
        report['absolute_contract'][name]={'accuracy':bool(metric['RMSE'] is not None and metric['invalid_samples']==0 and metric['RMSE']<=rmse and metric['P95']<=p95),'coverage':bool(cov['fraction']>=.2 and cov['left_sample_seconds']>=10)}
    report['absolute_contract']['joint_5s']=report['supports']['joint_ready']['coverage']['longest_episode_seconds']>=5
    report['transient_windows']={str(end):summary(e,raw&(t<=end)) for end in [1,5,10,20,60]}
    report['raw_peaks']={key:({'value':float(np.nanmax(e[key][raw])),'time':float(t[np.nanargmax(np.where(raw,e[key],np.nan))])} if np.any(raw&np.isfinite(e[key])) else None) for key in ['v','eta','angle','norm']}
    report['seeds'],seed_rows=seed_metrics(events,inputs,labels)
    report['recovery']=recovery_metrics(t,masks['joint_ready'],e,labels['phase'],labels['sufficient_reliable_supply'] if 'sufficient_reliable_supply' in labels else None,labels['sufficient_reliable_supply_run_start'] if 'sufficient_reliable_supply_run_start' in labels else None)
    if baseline:
        baseline=Path(baseline);bmeta=json.loads((baseline/'run.json').read_text())
        if bmeta['status']!='COMPLETED_ESTIMATION' or not bmeta['full_input_consumed']:raise ValueError('Incomplete baseline')
        if bmeta['input_sha']!=run['input_sha']:raise ValueError('Baseline inputs differ')
        bevents=[json.loads(x) for x in (baseline/'events.jsonl').read_text().splitlines()];bt,braw,be=aligned(bevents,labels)
        if not np.array_equal(t,bt):raise ValueError('Baseline event support differs')
        report['baseline_same_support']={key:{'new':summary(e,mask&braw),'previous':summary(be,mask&braw),
                                              'requested_samples':int(mask.sum()),'baseline_missing_samples':int((mask&~braw).sum())} for key,mask in masks.items()}
        joint=raw&braw;report['raw_regression']={}
        for key,floor in [('v',.01),('eta',.05),('angle',.1)]:
            report['raw_regression'][key]=regression_metric(e[key],be[key],joint,floor,t)

    report['bootstrap_events']=[]
    last_count=0
    for row in events:
        count=row['bootstrap_count']
        if count>last_count:report['bootstrap_events'].append({key:row[key] for key in ['t','epoch','bootstrap_count','last_bootstrap_time','bootstrap_source','physical_fault_count']})
        last_count=count
    report['provenance']={'evaluator_sha':sha(__file__),'acceptance_sha':sha(DOC/'acceptance.json'),'protocol_sha':sha(DOC/'protocol.json'),
                          'run_sha':sha(run_dir/'run.json'),'events_sha':sha(run_dir/'events.jsonl'),'states_sha':sha(run_dir/'states.npz')}
    out.mkdir(parents=True)
    (out/'metrics.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    (out/'seed_evaluation.json').write_text(json.dumps(seed_rows,indent=2,allow_nan=False)+'\n')
    np.savez_compressed(out/'error_curves.npz',t=t,raw=raw,ready_G=readyG,ready_V=readyV,**e)
    return report

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('input-dir','run-dir','out'):p.add_argument('--'+name,required=True)
    p.add_argument('--baseline');a=p.parse_args();r=evaluate(**vars(a));print(json.dumps({'status':r['status'],'output':a.out}))
