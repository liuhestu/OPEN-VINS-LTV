"""Read-only soft-grace accounting; preserves ready support and missing reference.

Accepts native features.jsonl or wrapper events.jsonl. Optional error_curves.npz
must match the existing evaluator's camera identity/time and ready masks.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import numpy as np


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def summarize(rows, curves=None):
    result={'schema':'SOFT_GRACE_REPORT_V1','branches':{},'evaluation_changes':False}
    receipt=[r['receipt_id'] for r in rows]
    if any(b<=a for a,b in zip(receipt,receipt[1:])):raise ValueError('Receipt order/uniqueness violated')
    native=curves is not None and 'camera_ns' in curves
    lookup={}
    if curves is not None:
        keys=curves['camera_ns'] if native else curves['t']
        if len(set(keys.tolist()))!=len(keys):raise ValueError('Duplicate evaluator identity')
        lookup={key:k for k,key in enumerate(keys)}
    for branch in ['G','V']:
        held=[];ready=0;missing=0;violations=[];episodes=[];active=None;previous=None
        for index,row in enumerate(rows):
            is_ready=bool(row.get('ready_'+branch,False));ready+=is_ready
            if 'grace_'+branch not in row:
                missing+=1;active=None;previous=None;continue
            grace=bool(row['grace_'+branch]);time=float(row.get('imu_time',row.get('t')))
            if not math.isfinite(time):raise ValueError('Nonfinite physical time')
            epoch=row['epoch']
            if not grace:
                active=None;previous=row;continue
            count=row.get('soft_failure_frames_'+branch);last=row.get('last_strict_good_'+branch)
            record={'receipt_id':row['receipt_id'],'epoch':epoch,'time':time,'soft_failure_frames':count,'last_strict_good':last,
                    'error':None,'reference_status':'UNAVAILABLE','severe':None}
            def fail(reason):violations.append({'receipt_id':row['receipt_id'],'reason':reason})
            if not is_ready or not row.get('raw_current') or not row.get('ready_soft_grace_enabled'):fail('held_without_ready_current_enabled')
            valid_last=isinstance(last,(int,float)) and math.isfinite(last) and last>=0
            if not valid_last:fail('invalid_last_strict_good')
            elif time<last or time>math.nextafter(last+.10,math.inf):fail('outside_fixed_one_ULP_deadline')
            if not isinstance(count,int) or isinstance(count,bool) or not 1<=count<=2:fail('outside_two_failure_events')
            consecutive=previous is not None and previous['epoch']==epoch and bool(previous.get('grace_'+branch))
            if consecutive:
                if last!=previous.get('last_strict_good_'+branch):fail('held_refreshed_last_good')
                if count!=previous.get('soft_failure_frames_'+branch,0)+1:fail('held_counter_not_consecutive')
            elif count!=1:fail('first_held_count_not_one')
            if active is None or not consecutive:
                active={'epoch':epoch,'start':time,'end':time,'packets':0,'duration':0.}
                episodes.append(active)
            active['packets']+=1;active['end']=time;active['duration']=time-active['start']
            if active['packets']>2:fail('more_than_two_consecutive_held_packets')
            key=row.get('camera_ns') if native else row.get('t',time)
            if key in lookup:
                k=lookup[key]
                curve_time=float(curves['time'][k] if native else curves['t'][k])
                if abs(curve_time-time)>1e-9:raise ValueError('Evaluator physical time mismatch')
                mask=curves['P_NEW_ready_'+branch] if native else curves['ready_'+branch]
                if bool(mask[k])!=is_ready:raise ValueError('Evaluator ready support mismatch')
                values=curves['P_NEW_error_g_angle_deg' if branch=='G' else 'P_NEW_error_v'] if native else curves['angle' if branch=='G' else 'v']
                value=float(values[k])
                if math.isfinite(value):
                    record.update(error=value,reference_status='EXISTING_EVALUATOR_FINITE_REFERENCE',severe=value>(5. if branch=='G' else 1.))
            held.append(record);previous=row
        errors=[r['error'] for r in held if r['error'] is not None]
        result['branches'][branch]={'held_packets':len(held),'ready_packets':ready,'held_fraction_of_ready':len(held)/ready if ready else None,
                                   'missing_grace_field_packets':missing,'episodes':episodes,
                                   'longest_consecutive_held_packets':max((e['packets'] for e in episodes),default=0),
                                   'longest_consecutive_held_seconds':max((e['duration'] for e in episodes),default=0.),
                                   'constraint_status':'UNAVAILABLE_INCOMPLETE_LOG' if missing else ('FAIL' if violations else 'PASS'),
                                   'violations':violations,'held_records':held,'reference_available_packets':len(errors),
                                   'reference_missing_packets':len(held)-len(errors),
                                   'held_error_RMSE':float(np.sqrt(np.mean(np.square(errors)))) if errors else None,
                                   'held_error_P95':float(np.quantile(errors,.95)) if errors else None,
                                   'held_severe_fraction':sum(r['severe'] is True for r in held)/len(errors) if errors else None,
                                   'error_units':'degrees' if branch=='G' else 'm/s'}
    result['limits']='two consecutive held camera events and time<=nextafter(last_strict_good+0.10,+infinity); existing severe cutoffs G>5deg,V>1m/s'
    result['scope']='Held-only descriptive subset of existing ready output; never replaces overall ready acceptance; missing/invalid reference remains null'
    return result


def run(source,out,errors=None):
    source,out=Path(source),Path(out)
    if out.exists():raise FileExistsError(out)
    rows=[json.loads(line) for line in source.read_text().splitlines() if line.strip()]
    curves=None
    if errors:
        with np.load(errors,allow_pickle=False) as archive:curves={key:archive[key] for key in archive.files}
    result=summarize(rows,curves)
    result['provenance']={'source_sha':digest(source),'tool_sha':digest(__file__),'existing_error_curves_sha':digest(errors) if errors else None}
    out.parent.mkdir(parents=True,exist_ok=True)
    with out.open('x') as stream:json.dump(result,stream,indent=2,allow_nan=False);stream.write('\n')
    return result

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--source',required=True);p.add_argument('--out',required=True);p.add_argument('--errors')
    a=p.parse_args();r=run(a.source,a.out,a.errors);print(json.dumps({b:r['branches'][b]['constraint_status'] for b in ['G','V']}))
