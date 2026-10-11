"""Read-only input/management facts for two user-selected receipts, no GT or observer."""
import os
for k in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS'):os.environ[k]='1'
import argparse
from collections import deque
import json
from pathlib import Path
import subprocess
import numpy as np
from reconstruct_prediction import interpolate,sha
TARGETS=(1413394964105760512,1413394966255760384)
HALF_WINDOW=.5

def norm_stats(values):
    a=np.array(values);norm=np.linalg.norm(a,axis=1)
    return dict(vector_mean=a.mean(axis=0).tolist(),norm_min=float(norm.min()),norm_median=float(np.median(norm)),norm_max=float(norm.max()))

def run(cache,features,reconstruction,decoder,out):
    out=Path(out);out.mkdir(parents=True,exist_ok=False)
    rows=[json.loads(l) for l in Path(features).read_text().splitlines()]
    by_ns={r['camera_ns']:r for r in rows};by_time={r['camera_ns']*1e-9:r for r in rows}
    origin=next(r['imu_time'] for r in rows if r['initialized'])
    target_times={ns:by_ns[ns]['imu_time'] for ns in TARGETS}
    previous_seed={};seed_history={}
    for r in rows:
        for seed in r.get('seeds',[]):
            if seed['admitted']:seed_history[(r['epoch'],seed['id'])]=seed
    path=Path(reconstruction)
    meta=json.loads((path/'summary.json').read_text())
    if meta['status']!='ALIGNED' or meta['identity'][str(Path(cache))]!=sha(cache):raise ValueError('Cache not bound to aligned reconstruction')
    diag={d['camera_ns']:d for d in map(json.loads,(path/'camera_diagnostics.jsonl').read_text().splitlines())}
    samples=deque();previous=None;result={str(ns):[] for ns in TARGETS}
    process=subprocess.Popen([decoder,cache],stdout=subprocess.PIPE,stderr=(out/'decoder_stderr.log').open('w'),text=True)
    try:
        with (out/'neighborhoods.jsonl').open('w') as output:
            for line in process.stdout:
                r=json.loads(line)
                if r['kind']=='imu':samples.append(r);continue
                if r['kind']=='pause':previous=None;continue
                log=by_time[r['time']];ns=log['camera_ns']
                windows=[target for target,time in target_times.items() if abs(r['target']-time)<=HALF_WINDOW+1e-6]
                if windows:
                    if previous is None or previous['epoch']!=r['epoch'] or ns not in diag:raise ValueError('Selected event crosses unsupported reconstruction boundary')
                    previous_log=by_time[previous['time']]
                    start,end=previous['cursor'],r['target']
                    endpoints=[interpolate(list(samples),start)]+[s for s in samples if start<s['time']<end]+[interpolate(list(samples),end)]
                    c={k:np.array(v).reshape(3,3) for k,v in r['calibration'].items() if k not in ('offset','p_BC')}
                    ba=np.array(r['ba']);bg=np.array(r['bg']);measured=[];acc=[];gyro=[]
                    for point in endpoints:
                        am=np.array(point['am']);wm=np.array(point['wm']);corrected_acc=c['R_ACCtoIMU']@c['Da']@(am-ba)
                        corrected_gyro=c['R_GYROtoIMU']@c['Dw']@(wm-bg-c['Tg']@corrected_acc)
                        measured.append(dict(time=point['time'],raw_acc=am.tolist(),raw_gyro=wm.tolist(),corrected_acc=corrected_acc.tolist(),corrected_gyro=corrected_gyro.tolist()))
                        acc.append(corrected_acc);gyro.append(corrected_gyro)
                    substeps=[dict(dt=b['time']-a['time'],acc_midpoint=(.5*(np.array(a['corrected_acc'])+np.array(b['corrected_acc']))).tolist(),gyro_midpoint=(.5*(np.array(a['corrected_gyro'])+np.array(b['corrected_gyro']))).tolist()) for a,b in zip(measured,measured[1:])]
                    rotation_integral=sum(s['dt']*np.array(s['gyro_midpoint']) for s in substeps)
                    integral_norm=sum(s['dt']*np.linalg.norm(s['gyro_midpoint']) for s in substeps)
                    points=[];measurements={o['id']:np.array(o['bearing']) for o in r['observations'] if o['camera']==0}
                    tracks={t['id']:t for t in log.get('tracks',[])}
                    for point in diag[ns]['points']:
                        p=dict(point);fid=p['id'];bearing=c['R_BC']@measurements[fid];bearing/=np.linalg.norm(bearing)
                        ray=np.array(p['post_landmark'])-np.array(r['calibration']['p_BC'])
                        p['post_ray_length']=float(np.linalg.norm(ray))
                        p['post_angle_rad']=float(np.arccos(np.clip(ray@bearing/np.linalg.norm(ray),-1.,1.))) if np.linalg.norm(ray)>=.1 else None
                        p['admission_seed']=seed_history.get((r['epoch'],fid))
                        track=tracks.get(fid);p['current_track']=track
                        p['residence_age']=r['target']-track['entered'] if track and track['entered']>=0 else None
                        points.append(p)
                    frame=dict(camera_ns=ns,imu_time=end,relative_initialized_time=end-origin,selected_centers=windows,epoch=r['epoch'],interval_start=start,dt=end-start,
                               exact_imu_endpoints=measured,midpoint_substeps=substeps,calibration=r['calibration'],accel_bias=r['ba'],gyro_bias=r['bg'],
                               corrected_acc=norm_stats(acc),corrected_gyro=norm_stats(gyro),integral_gyro_vector=rotation_integral.tolist(),integral_gyro_norm=float(integral_norm),
                               previous_cam0_ids=previous_log['current_cam0_ids'],current_cam0_ids=log['current_cam0_ids'],previous_retained_ids=previous_log.get('retained_ids',[]),retained_ids=log.get('retained_ids',[]),
                               previous_managed_observations=previous['observations'],current_managed_observations=r['observations'],births=r['births'],retirement_events=log.get('retirement_events',[]),
                               candidate_seeds=log.get('seeds',[]),corrected_ids=log.get('corrected_ids',[]),ready_G=log['ready_G'],ready_V=log['ready_V'],pool_ready=log['pool_ready'],
                               delta_v=diag[ns]['delta_v'],delta_eta=diag[ns]['delta_eta'],scalars=diag[ns]['scalars'],points=points)
                    output.write(json.dumps(frame,allow_nan=False)+'\n')
                    for target in windows:result[str(target)].append(frame)
                previous=r
                while len(samples)>2 and samples[1]['time']<=r['target']:samples.popleft()
        if process.wait()!=0:raise RuntimeError('Cache decode failed')
    except Exception:
        process.kill();process.wait();raise
    summary={'status':'INPUT_FACTS_ONLY','GT_read':False,'observer_invoked':False,'origin_first_initialized_imu_time':origin,'half_window_s':HALF_WINDOW,'events':{}}
    for ns,window in result.items():
        center=next(r for r in window if r['camera_ns']==int(ns));angles=[p for p in center['points'] if p.get('in_scalar_p95')]
        summary['events'][ns]={k:center[k] for k in ('relative_initialized_time','dt','corrected_acc','corrected_gyro','integral_gyro_vector','integral_gyro_norm','births','retirement_events','delta_v','delta_eta','scalars','ready_G','ready_V','pool_ready')}
        summary['events'][ns].update(neighborhood_frames=len(window),raw_cam0_count=len(center['current_cam0_ids']),managed_count=len(center['points']),previous_retained=len(center['previous_retained_ids']),current_retained=len(center['retained_ids']),
            largest_prediction_angle=max(angles,key=lambda p:p['angle_rad']) if angles else None,
            gyro_neighborhood_norm_min=min(r['corrected_gyro']['norm_min'] for r in window),gyro_neighborhood_norm_max=max(r['corrected_gyro']['norm_max'] for r in window),
            acc_neighborhood_norm_min=min(r['corrected_acc']['norm_min'] for r in window),acc_neighborhood_norm_max=max(r['corrected_acc']['norm_max'] for r in window),
            neighborhood_births=sum(len(r['births']) for r in window),neighborhood_retirements=sum(len(r['retirement_events']) for r in window),
            candidate_seed_count=len(center['candidate_seeds']),admitted_candidate_seed_count=sum(bool(s['admitted']) for s in center['candidate_seeds']))
    summary['identity']={str(Path(p)):sha(p) for p in (cache,features,decoder,__file__,path/'summary.json')}
    (out/'summary.json').write_text(json.dumps(summary,indent=2)+'\n');return summary
if __name__=='__main__':
    p=argparse.ArgumentParser()
    for key in ('cache','features','reconstruction','decoder','out'):p.add_argument('--'+key,required=True)
    print(json.dumps(run(**vars(p.parse_args())),indent=2))
