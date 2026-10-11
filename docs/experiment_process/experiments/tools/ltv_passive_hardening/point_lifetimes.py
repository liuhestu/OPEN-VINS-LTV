"""Offline per-(observer epoch, original ID) lifecycle ledger for native/wrapper JSONL.

Counts camera correction EVENTS explicitly named by corrected_ids, never visible
frames or Riccati substeps. Missing historical fields remain UNAVAILABLE/null.
This tool reads logs only; it cannot supply labels or feedback to the observer.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path


def finite(value, field):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError('Invalid numeric field: '+field)
    return float(value)


def summarize(rows):
    points={}; epoch_end={}; missing_corrections=set(); previous_receipt=None; seed_events=set()
    def get(epoch, fid):
        if not isinstance(fid,int) or isinstance(fid,bool) or fid<0:raise ValueError('Invalid original ID')
        key=(epoch,fid)
        if key not in points:
            points[key]=dict(epoch=epoch,id=fid,first_observed=None,manager_first_seen=None,first_opportunity_logged=None,
                             admission=None,seed_time=None,seed_source=None,last_observed=None,last_seen=None,
                             retirement_time=None,retirement_kind=None,retirement_time_evidence=None,
                             actual_correction_events_observed=0,seed_events_observed=0)
        return points[key]
    def set_once(p,key,value):
        if p[key] is not None and p[key]!=value:raise ValueError('Inconsistent '+key+' for identity')
        p[key]=value
    for row in rows:
        receipt=row.get('receipt_id')
        if not isinstance(receipt,int) or (previous_receipt is not None and receipt<=previous_receipt):
            raise ValueError('Missing or non-increasing receipt_id')
        previous_receipt=receipt
        epoch=row.get('epoch')
        if not isinstance(epoch,int) or epoch<0:raise ValueError('Invalid observer epoch')
        time=finite(row.get('imu_time',row.get('t')),'physical time')
        if epoch in epoch_end and time<epoch_end[epoch]:raise ValueError('Physical time decreased within epoch')
        epoch_end[epoch]=time
        for fid in row.get('current_cam0_ids',[]):
            p=get(epoch,fid)
            if p['first_observed'] is None:p['first_observed']=time
            p['last_observed']=time
        track_rows=list(row.get('tracks',[]))
        for event in row.get('retirement_events',[]):
            if event.get('epoch',epoch)!=epoch:raise ValueError('Retirement epoch mismatch')
            if event['kind'] not in ('ACTIVE_RETIRED','CANDIDATE_TTL'):raise ValueError('Unknown retirement kind')
            p=get(epoch,event['id']);set_once(p,'retirement_time',finite(event['time'],'retirement time'))
            set_once(p,'retirement_kind',event['kind']);p['retirement_time_evidence']='EXPLICIT_EVENT'
            if 'record' in event:track_rows.append(event['record'])
        for track in track_rows:
            p=get(epoch,track['id'])
            if 'first_seen' in track:set_once(p,'manager_first_seen',finite(track['first_seen'],'first_seen'))
            for field,key in [('entered','admission'),('seeded','seed_time')]:
                if field in track:
                    val=finite(track[field],field)
                    if val>=0:set_once(p,key,val)
            if 'last_seen' in track:
                val=finite(track['last_seen'],'last_seen')
                if p['last_seen'] is None or val>p['last_seen']:p['last_seen']=val
            if track.get('ever_opportunity') and p['first_opportunity_logged'] is None:p['first_opportunity_logged']=time
            if track.get('phase')=='RETIRED' and p['retirement_time'] is None:
                p['retirement_time']=time;p['retirement_time_evidence']='FIRST_LOGGED_RETIRED_UPPER_BOUND'
                p['retirement_kind']='ACTIVE_RETIRED' if p['admission'] is not None else ('CANDIDATE_TTL' if track.get('reason')=='candidate_ttl' else 'UNKNOWN')
        # births remain available when heavy seed diagnostics are disabled.
        for birth in row.get('births',[]):
            set_once(get(epoch,birth['id']),'admission',time)
        written={b['id']:b for b in row.get('births',[]) if b.get('apply_seed')}
        for seed in row.get('seeds',[]):
            if seed.get('admitted'):
                p=get(epoch,seed['id']);set_once(p,'admission',finite(seed.get('time',time),'seed time'))
            if seed.get('admitted') and seed.get('mean_written'):
                written[seed['id']]=seed
        for fid,seed in written.items():
            key=(epoch,fid)
            if key in seed_events:raise ValueError('Repeated seed event in same epoch/ID')
            seed_events.add(key);p=get(epoch,fid);p['seed_events_observed']+=1
            set_once(p,'seed_time',finite(seed.get('time',time),'seed time'))
            set_once(p,'admission',finite(seed.get('time',time),'admission time'))
            p['seed_source']=seed.get('source')
        if 'corrected_ids' not in row:
            missing_corrections.add(epoch)
        else:
            ids=row['corrected_ids']
            if len(set(ids))!=len(ids):raise ValueError('Duplicate corrected ID')
            if ids and (not row.get('raw_current') or not row.get('available') or row.get('camera_substeps',0)<=0):
                raise ValueError('Corrected IDs without actual available correction')
            for fid in ids:
                p=get(epoch,fid)
                if p['retirement_time'] is not None:raise ValueError('Correction of retired ID')
                p['actual_correction_events_observed']+=1
    result=[]
    for (epoch,fid),p in sorted(points.items()):
        end=p['retirement_time'] if p['retirement_time'] is not None else epoch_end[epoch]
        p['right_censored']=p['retirement_time'] is None
        p['censor_time']=epoch_end[epoch] if p['right_censored'] else None
        p['censor_reason']='LOG_OR_OBSERVER_EPOCH_END' if p['right_censored'] else None
        p['observation_age_at_last_observation']=None if p['first_observed'] is None or p['last_observed'] is None else p['last_observed']-p['first_observed']
        p['residence_age_at_exit_or_censor']=None if p['admission'] is None else end-p['admission']
        p['waiting_to_admission']=None if p['first_observed'] is None or p['admission'] is None else p['admission']-p['first_observed']
        if p['residence_age_at_exit_or_censor'] is not None and p['residence_age_at_exit_or_censor']<0:raise ValueError('Admission after exit')
        p['correction_count_status']='UNAVAILABLE_MISSING_CORRECTED_IDS_IN_EPOCH' if epoch in missing_corrections else 'EXPLICIT_CORRECTION_EVENTS'
        p['actual_correction_events']=None if epoch in missing_corrections else p['actual_correction_events_observed']
        p['opportunity_time_status']='FIRST_LOGGED_FLAG_NOT_GUARANTEED_EXACT_EVENT' if p['first_opportunity_logged'] is not None else 'UNAVAILABLE'
        p['error_metrics']=None;p['error_status']='UNAVAILABLE_NO_TRUTH_IN_LIFECYCLE_LOG'
        result.append(p)
    return dict(schema='POINT_LIFETIMES_V1',points=result,point_count=len(result),
                correction_unit='one explicit camera correction event per ID, not integrator substeps',
                missing_corrected_ids_epochs=sorted(missing_corrections),
                observation_birth_scope='first logged current cam0 observation within observer epoch; not global frontend birth',
                admission_vs_visibility='last_observed is distinct from delayed core retirement; no disappearance success rate inferred')


def run(source,out):
    source,out=Path(source),Path(out)
    if out.exists():raise FileExistsError(out)
    with source.open() as stream:result=summarize(json.loads(line) for line in stream if line.strip())
    result['source_sha']=hashlib.sha256(source.read_bytes()).hexdigest()
    result['tool_sha']=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    out.parent.mkdir(parents=True,exist_ok=True)
    with out.open('x') as stream:json.dump(result,stream,indent=2,allow_nan=False);stream.write('\n')
    return result

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--source',required=True);p.add_argument('--out',required=True)
    a=p.parse_args();r=run(a.source,a.out);print(json.dumps({'point_count':r['point_count'],'output':a.out}))
