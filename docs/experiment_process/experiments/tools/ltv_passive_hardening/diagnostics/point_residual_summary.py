"""Describe exact reconstructed point residuals, fixed .02 rad and fixed prior windows.
No GT, candidate selection, threshold sweep or observer execution.
"""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import statistics

WINDOWS=[('prior_window_5_80_13_95',5.80,13.95),('prior_window_72_15_81_70',72.15,81.70),('whole_logged_timeline',-math.inf,math.inf)]

def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def stats(values):
    return dict(count=len(values),minimum=min(values) if values else None,median=statistics.median(values) if values else None,maximum=max(values) if values else None)

def analyze(diagnostics,features,out):
    diagnostics,features,out=map(Path,(diagnostics,features,out))
    if out.exists():raise FileExistsError(out)
    rows=[json.loads(line) for line in features.read_text().splitlines()]
    by_ns={r['camera_ns']:r for r in rows}
    if len(by_ns)!=len(rows):raise ValueError('Nonunique camera integer identity')
    t0=next(r['imu_time'] for r in rows if r['initialized'])
    packets=[]
    for line in diagnostics.read_text().splitlines():
        r=json.loads(line);f=by_ns[r['camera_ns']]
        if r['epoch']!=f['epoch']:raise ValueError('Observer epoch mismatch')
        if abs(r['time']-f['imu_time'])>1e-9:raise ValueError('Physical time mismatch')
        tracks={p['id']:p for p in f['tracks']}
        points=[]
        for p in r['points']:
            if not p['in_scalar_p95']:continue
            meta=tracks.get(p['id'],{});entered=meta.get('entered');first=meta.get('first_seen')
            angle=p['angle_rad']
            if not math.isfinite(angle):raise ValueError('Nonfinite angle')
            points.append(dict(id=p['id'],angle_rad=angle,above_original_limit=angle>.02,
                               residence_age=f['imu_time']-entered if entered is not None and entered>=0 else None,
                               observation_age=f['imu_time']-first if first is not None else None))
        above=[p for p in points if p['above_original_limit']]
        s=r['scalars'];dv=r['delta_v'];dg=r['delta_eta']
        packets.append(dict(camera_ns=r['camera_ns'],receipt_id=f['receipt_id'],epoch=r['epoch'],time=f['imu_time'],relative_time=f['imu_time']-t0,
                            valid=r['valid'],points=points,point_count=len(points),above_count=len(above),
                            above_fraction=len(above)/len(points) if points else None,prediction_p95=s['prediction_angle_p95_rad'],
                            prediction_bad=bool(r['valid'] and s['prediction_angle_p95_rad']>.02),
                            v_rate=s['velocity_correction_rate'],eta_rate=s['gravity_correction_rate'],delta_v=dv,delta_eta=dg,
                            births=sum(x.get('admitted',False) for x in f.get('seeds',[])),
                            immature_above=sum(p['residence_age'] is not None and p['residence_age']+1e-12<.1 for p in above),
                            mature_observed=f['mature_features'],observed=f['observed_features']))
    report={'t0':t0,'threshold_rad':.02,'windows':{},'provenance':{'diagnostics_sha':sha(diagnostics),'features_sha':sha(features),'script_sha':sha(__file__)},
            'post_angle_status':'UNAVAILABLE: reconstructed records lack exact per-event camera center and measured unit bearing; post_landmark alone does not determine angular residual',
            'scope':'Descriptive original-threshold point counts. No GT used. Birth points absent from original pre-existing-slot P95 are excluded identically. Shared corrections are aggregate, not attributed causally to individual points.'}
    for name,start,end in WINDOWS:
        # One microsecond only handles predeclared decimal window boundary vs absolute double subtraction; no observation is interpolated.
        selected=[p for p in packets if p['relative_time']>=start-1e-6 and p['relative_time']<=end+1e-6]
        bad=[p for p in selected if p['prediction_bad']]
        hits=Counter();runs=[];active={}
        for packet in selected:
            keys={(packet['epoch'],p['id']):p for p in packet['points'] if p['above_original_limit']}
            for key in list(active):
                if key not in keys or packet['receipt_id']!=active[key]['last_receipt']+1:
                    runs.append(active.pop(key))
            for key,p in keys.items():
                hits[key]+=1
                if key not in active:active[key]=dict(epoch=key[0],id=key[1],start=packet['relative_time'],end=packet['relative_time'],packets=0,last_receipt=packet['receipt_id'],max_angle_rad=p['angle_rad'],start_residence_age=p['residence_age'])
                run=active[key];run['end']=packet['relative_time'];run['packets']+=1;run['last_receipt']=packet['receipt_id'];run['max_angle_rad']=max(run['max_angle_rad'],p['angle_rad'])
        runs.extend(active.values())
        for r in runs:r['duration']=r['end']-r['start']
        atoms=[p for e in selected for p in e['points'] if p['above_original_limit']]
        bad_atoms=[p for e in bad for p in e['points'] if p['above_original_limit']]
        top=hits.most_common()
        report['windows'][name]=dict(packets=len(selected),valid_diagnostic_packets=sum(p['valid'] for p in selected),p95_bad_packets=len(bad),
            above_count_on_p95_bad=stats([p['above_count'] for p in bad]),above_fraction_on_p95_bad=stats([p['above_fraction'] for p in bad]),
            majority_above_packets=sum(p['above_count']>p['point_count']/2 for p in bad),
            anomaly_point_events=len(atoms),unique_anomalous_ids=len(hits),
            per_id_anomaly_counts=[dict(epoch=k[0],id=k[1],packets=v) for k,v in top],
            top1_anomaly_share=top[0][1]/len(atoms) if atoms else None,top3_anomaly_share=sum(v for k,v in top[:3])/len(atoms) if atoms else None,
            residence_age_of_bad_packet_anomalies=stats([p['residence_age'] for p in bad_atoms if p['residence_age'] is not None]),
            immature_anomaly_events=sum(p['residence_age'] is not None and p['residence_age']+1e-12<.1 for p in bad_atoms),
            births_on_p95_bad=stats([p['births'] for p in bad]),p95_bad_without_birth=sum(p['births']==0 for p in bad),
            p95_bad_with_all_observed_mature=sum(p['observed']==p['mature_observed'] for p in bad),
            p95_bad_and_v_rate_bad=sum(p['v_rate']>.5 for p in bad),p95_bad_and_eta_rate_bad=sum(p['eta_rate']>1 for p in bad),
            v_rate_bad_without_p95_bad=sum(p['valid'] and p['v_rate']>.5 and not p['prediction_bad'] for p in selected),
            longest_anomaly_runs=sorted(runs,key=lambda r:(-r['duration'],-r['packets'],r['id'])),packets_detail=selected)
    out.mkdir(parents=True)
    (out/'summary.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print(json.dumps({k:{a:b for a,b in v.items() if a not in ['packets_detail','longest_anomaly_runs','per_id_anomaly_counts']} for k,v in report['windows'].items()}))

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--diagnostics',required=True);p.add_argument('--features',required=True);p.add_argument('--out',required=True)
    a=p.parse_args();analyze(a.diagnostics,a.features,a.out)
