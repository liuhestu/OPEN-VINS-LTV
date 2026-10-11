"""Offline resource trace audit; never execute an observer or change thresholds.

Input/event/resource records are joined as streams. Offline identity sets and
timing arrays are permitted here and are never included in measured-process RSS.
"""
import argparse
from collections import defaultdict
import hashlib
import json
from pathlib import Path
import numpy as np


def sha(path):
    result=hashlib.sha256()
    with Path(path).open('rb') as source:
        for block in iter(lambda:source.read(1024*1024),b''):result.update(block)
    return result.hexdigest()


def rows(path):
    with Path(path).open() as source:
        for line in source:
            yield json.loads(line)


def camera_inputs(path,counters,require):
    previous_imu=-np.inf
    for row in rows(path):
        kind=row['type']
        if kind=='context_model':
            counters['context_models']+=1
            require(counters['context_models']==1,'Duplicate context model')
        elif kind=='imu':
            require(row['time']>previous_imu,'IMU times not strictly increasing')
            require(np.isfinite(row['measurement']).all(),'Nonfinite IMU input')
            previous_imu=row['time'];counters['imu']+=1
        elif kind=='camera':
            counters['camera']+=1
            require(previous_imu>=row['time'],'Camera lacks right IMU bracket')
            yield row
        else:require(False,'Unknown input record type')


def analyze(directory,out):
    directory=Path(directory);out=Path(out)
    if out.exists():raise FileExistsError(out)
    run=json.loads((directory/'run.json').read_text());errors=[];error_count=0
    def require(condition,message):
        nonlocal error_count
        if not condition:
            error_count+=1
            if len(errors)<200:errors.append(message)
    require(run['status']=='COMPLETED_RESOURCE_TRACE','Benchmark did not complete')
    require(run['invariant_violations']==0,'Benchmark itself recorded invariant violations')
    provenance={name:sha(directory/name) for name in ('run.json','inputs.jsonl','events.jsonl','resources.jsonl')}
    for name in ('inputs.jsonl','events.jsonl','resources.jsonl'):
        require(provenance[name]==run[name+'_sha'],'SHA mismatch '+name)
    counters=defaultdict(int);epochs={};last_time=-np.inf;last_epoch=-1;seen_epochs=set()
    resources=iter(rows(directory/'resources.jsonl'));inputs=iter(camera_inputs(directory/'inputs.jsonl',counters,require))
    times=[];rss=[];timing=defaultdict(list);bins=defaultdict(list);max_counts=defaultdict(int)
    birth_sources=defaultdict(int);guard_known=0;guard_possible_collision=0;previous_bootstrap_count=0;bootstrap_times=[]
    for event_index,event in enumerate(rows(directory/'events.jsonl')):
        resource=next(resources,None);observed=next(inputs,None)
        require(resource is not None and observed is not None,'Missing resource/input row')
        if resource is None or observed is None:break
        t=event['t'];epoch=int(event['epoch'])
        require(t>last_time,'Nonmonotonic camera events');last_time=t
        require(event['receipt_id']==event_index+1,'Receipt sequence gap or duplicate')
        require(t==resource['t']==observed['time'],'Input/event/resource physical time mismatch')
        require(event['camera_ns']==observed['camera_ns'],'Original integer receipt mismatch')
        require(epoch>=last_epoch,'Observer epoch went backwards')
        last_epoch=epoch;seen_epochs.add(epoch)
        state=epochs.setdefault(epoch,{'admitted':set(),'written':set(),'active_retired':set(),'candidate_ttl':set(),
                                      'guard_rejections':0,'bits':0,'insertions':0,'highwater':-1,'frames':0})
        state['frames']+=1
        legal=[o for o in observed['observations'] if o[2]==observed['camera_ns']]
        for camera,key in ((0,'current_cam0_ids'),(1,'current_cam1_ids')):
            expected=sorted({int(o[0]) for o in legal if o[1]==camera})
            require(expected==event[key],'Causal current camera IDs differ from input')
        expected_rejected=[i for i,o in enumerate(observed['observations']) if o[2]!=observed['camera_ns']]
        require(expected_rejected==event['rejected_async_observation_indices'],'Asynchronous rejection indices differ')
        require(event['heavy_diagnostics']==0 and not event['seeds'] and not event['tracks'],'Heavy diagnostics unexpectedly enabled')
        require(event['bounded_management_schema']=='BOUNDED_V1','Unknown bounded schema')
        for birth in event['births']:
            identity=int(birth['id'])
            require(identity not in state['admitted'],'Repeated same-epoch admission/seed')
            require(identity not in state['candidate_ttl'],'Previously expired candidate admitted again')
            state['admitted'].add(identity);birth_sources[birth['source']]+=1
            if birth['apply_seed']:
                require(identity not in state['written'],'Repeated same-epoch mean write')
                state['written'].add(identity)
        active_events=set()
        for retirement in event['retirement_events']:
            identity=int(retirement['id']);record=retirement['record'];kind=retirement['kind']
            require(retirement['epoch']==epoch and retirement['time']==t,'Retirement event epoch/time mismatch')
            require(record['id']==identity and record['phase']=='RETIRED','Incomplete retirement identity/phase')
            require(np.isfinite([record[k] for k in ('first_seen','entered','seeded','last_seen')]).all(),'Nonfinite retirement record')
            if kind=='ACTIVE_RETIRED':
                require(identity in state['admitted'] and record['entered']>=0,'Active retirement lacks prior admission')
                require(identity not in state['active_retired'],'Duplicate active retirement')
                state['active_retired'].add(identity);active_events.add(identity)
            elif kind=='CANDIDATE_TTL':
                require(identity not in state['admitted'] and record['entered']<0,'Candidate TTL was previously admitted')
                require(identity not in state['candidate_ttl'],'Duplicate candidate TTL')
                state['candidate_ttl'].add(identity)
            else:require(False,'Unknown typed retirement')
        require(active_events==set(event['retired_ids']),'Typed active retirements differ from legacy retired_ids')
        rejected=event['identity_guard_rejected_ids']
        require(len(rejected)==len(set(rejected)),'Duplicate guard rejection ID in one event')
        state['guard_rejections']+=len(rejected)
        for identity in rejected:
            if identity in state['admitted'] or identity in state['candidate_ttl']:guard_known+=1
            else:guard_possible_collision+=1
        if event['management_accepted_input']:
            require(event['admitted_tracks']==len(state['admitted']),'Admission cumulative counter mismatch')
            require(event['active_retired_total']==len(state['active_retired']),'Active retirement cumulative counter mismatch')
            require(event['candidate_ttl_total']==len(state['candidate_ttl']),'Candidate TTL cumulative counter mismatch')
            require(event['identity_guard_rejections_total']==state['guard_rejections'],'Guard rejection cumulative counter mismatch')
            require(event['identity_guard_insertions']==len(state['admitted'])+len(state['candidate_ttl']),'Guard insertions differ from admitted+TTL')
            require(event['identity_guard_set_bits']>=state['bits'],'Same-epoch guard bit count decreased')
            require(event['identity_guard_insertions']>=state['insertions'],'Same-epoch guard insertion count decreased')
            state['bits']=event['identity_guard_set_bits'];state['insertions']=event['identity_guard_insertions']
        retained=event['retained_ids'];require(len(retained)==len(set(retained)),'Duplicate retained identity')
        require(set(retained)<=state['admitted']-state['active_retired'],'Retained identity lacks live admission')
        count=resource['counts'];require(count==event['resource_diagnostics'],'Two resource records differ')
        for key,value in count.items():max_counts[key]=max(max_counts[key],value)
        require(count['manager_records']<=256 and count['history_records']<=256,'Record capacity exceeded')
        require(count['max_samples_per_track']<=21 and count['total_history_samples']<=21*256,'History sample capacity exceeded')
        require(count['adapter_map_size']<=count['retained_ids']<=30,'Adapter/active capacity exceeded')
        require(count['core_admission_history_size']==0,'Unbounded core admission history populated')
        require(count['imu_buffer_size']<=4096,'IMU buffer capacity exceeded')
        require(count['guard_capacity_bytes']==1048576 and event['identity_guard_set_bits']<=8388608,'Bloom capacity exceeded')
        highwater=count['core_admission_high_water']
        suspended=(not event['raw_current'] and highwater==-1 and event['health_state'] in ('DORMANT','RECOVERING','COLLECTING'))
        require(suspended or highwater>=state['highwater'],'Core local ID highwater decreased without explicit suspension')
        if highwater>=0:state['highwater']=highwater
        require(all(resource['checks'].values()),'Online invariant check was false')
        if not event['raw_current']:require(event['raw_v'] is None and event['raw_eta'] is None,'Stale state claimed as current raw')
        require(event['bootstrap_count']>=previous_bootstrap_count,'Bootstrap counter decreased')
        if event['bootstrap_count']>previous_bootstrap_count:
            bootstrap_times.append({'time':event['last_bootstrap_time'],'source':event['bootstrap_source'],'epoch':epoch})
        previous_bootstrap_count=event['bootstrap_count']
        times.append(t);rss.append(resource['rss_bytes'])
        bins[min(int(t//60),max(0,int(np.ceil(run['seconds']/60))-1))].append(len(times)-1)
        for key in ('construct_ms','abi_ms','adapter_compute_ms','parse_ms','write_ms'):
            value=float(resource[key]);require(np.isfinite(value) and value>=0,'Invalid timing '+key);timing[key].append(value)
        require(resource['adapter_compute_ms']==event['compute_time_ms'],'Adapter computation timers disagree')
    require(next(resources,None) is None,'Extra resource rows')
    require(next(inputs,None) is None,'Extra camera input rows')
    require(len(times)==run['frames']==counters['camera'],'Full record count mismatch')
    require(len(times)==round(run['seconds']*20)+1,'Trace lacks prescribed camera events')
    duration=times[-1]-times[0] if times else 0
    long_complete=duration>=600 and len(times)>=12001 and run['full_600s']
    timing['whole_branch_ms']=(np.array(timing['construct_ms'])+np.array(timing['abi_ms'])).tolist()
    timing_summary={key:{'mean':float(np.mean(values)),'p95':float(np.percentile(values,95)),'max':float(np.max(values))}
                    for key,values in timing.items() if values}
    minute_bins=[]
    for minute,indices in sorted(bins.items()):
        values=np.array(rss)[indices]
        minute_bins.append({'minute_index':minute,'samples':len(indices),'rss_min':int(values.min()),'rss_median':float(np.median(values)),
                            'rss_max':int(values.max()),'whole_branch_mean_ms':float(np.mean(np.array(timing['whole_branch_ms'])[indices])),
                            'whole_branch_p95_ms':float(np.percentile(np.array(timing['whole_branch_ms'])[indices],95))})
    first=[r for t,r in zip(times,rss) if t<60];last=[r for t,r in zip(times,rss) if max(0,run['seconds']-60)<=t<run['seconds']]
    result={'status':'FAIL' if error_count else ('RESOURCE_TRACE_VALID_600S' if long_complete else 'SHORT_TRACE_VALID_NOT_600S'),
            'error_count':error_count,'errors':errors,'duration_s':duration,'frames':len(times),'input_records':dict(counters),
            'once_seed_pass':error_count==0,'per_epoch':{str(k):{name:len(value) if isinstance(value,set) else value for name,value in state.items()}
                                                       for k,state in epochs.items()},'birth_sources':dict(birth_sources),
            'guard_rejections_known_inserted_identity':guard_known,'guard_rejections_possible_collision':guard_possible_collision,
            'max_counts':dict(max_counts),'bootstrap_events':bootstrap_times,'timings_ms':timing_summary,
            'performance_target':{'whole_branch_mean_under50ms':timing_summary.get('whole_branch_ms',{}).get('mean',float('inf'))<50,
                                  'whole_branch_p95_atmost100ms':timing_summary.get('whole_branch_ms',{}).get('p95',float('inf'))<=100},
            'minute_bins':minute_bins,'rss_first_minute_median':float(np.median(first)) if first else None,
            'rss_last_minute_median':float(np.median(last)) if last else None,
            'rss_interpretation':'/proc/self/statm resident pages of measured Python+C++ process, including shared runtime pages and allocator warmup; ordinary write-backed kernel page cache is not process RSS. No input arrays or whole log objects were loaded by benchmark; offline analysis sets/arrays are a different process. Flat RSS alone does not establish bounds; container invariants and source contract also checked.',
            'performance_interpretation':'whole_branch_ms = input geometry/prior covariance construction + C ABI call including light JSON; adapter_compute_ms excludes bridge and serialization; parse/write separate. Synthetic construction may include generator work and thus is conservative for observer branch, not actual full VIO timing.',
            'trace_identity':provenance,'analyzer_sha':sha(__file__),'observer_executed':False,'GT_read':False}
    out.mkdir(parents=True)
    (out/'metrics.json').write_text(json.dumps(result,indent=2,allow_nan=False))
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--directory',required=True);p.add_argument('--out',required=True)
    result=analyze(**vars(p.parse_args()))
    print(json.dumps({'status':result['status'],'errors':result['error_count'],'frames':result['frames'],'performance':result['performance_target']}))
    raise SystemExit(1 if result['error_count'] else 0)
