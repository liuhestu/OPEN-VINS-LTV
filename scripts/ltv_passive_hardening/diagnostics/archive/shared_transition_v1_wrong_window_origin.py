"""Offline signed shared-state correction transport; never a readiness rule."""
import os
for k in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS'):os.environ[k]='1'
import argparse
from collections import Counter,deque
import json
from pathlib import Path
import subprocess
import numpy as np
from reconstruct_prediction import interpolate,skew,sha

WINDOW_SECONDS=1.0
# Actual frozen observer limits, checked against the supplied configuration.
MAX_IMU_DT=.05
RESET_GAP=.2
WINDOWS=[('pool_early',5.80,13.95),('pool_late',72.15,81.70)]

def transition_step(dt,omega):
    R=np.eye(3)-dt*skew(omega)
    F=np.zeros((6,6));F[:3,:3]=R;F[:3,3:]=dt*np.eye(3);F[3:,3:]=R
    return F

def transition(samples,start,end,c,ba,bg):
    grid=[interpolate(samples,start)]+[s for s in samples if start<s['time']<end]
    if end>start:grid.append(interpolate(samples,end))
    def omega(s):
        am=c['R_ACCtoIMU']@c['Da']@(np.array(s['am'])-ba)
        return c['R_GYROtoIMU']@c['Dw']@(np.array(s['wm'])-bg-c['Tg']@am)
    Phi=np.eye(6)
    for a,b in zip(grid,grid[1:]):
        dt=b['time']-a['time']
        if dt<=0 or dt>MAX_IMU_DT:raise ValueError('IMU substep physical gap')
        Phi=transition_step(dt,.5*(omega(a)+omega(b)))@Phi
    return Phi

def window_metrics(entries):
    if not entries:return None
    vectors=np.array([e['transported'] for e in entries]);raw=np.array([e['delta'] for e in entries]);dts=np.array([e['dt'] for e in entries])
    result={'samples':len(entries),'contribution_start':entries[0]['time']}
    for key,block in [('v',slice(0,3)),('eta',slice(3,6))]:
        part=vectors[:,block];norms=np.linalg.norm(part,axis=1);vector=part.sum(axis=0)
        result[key]={'sum_vector':vector.tolist(),'norm_of_sum':float(np.linalg.norm(vector)),
                     'sum_of_norms':float(norms.sum()),'max_transported_single_norm':float(norms.max()),
                     'max_original_single_norm':float(np.linalg.norm(raw[:,block],axis=1).max()),
                     'max_original_single_rate':float((np.linalg.norm(raw[:,block],axis=1)/dts).max())}
    return result

def summary_stats(rows):
    result={'frames':len(rows),'complete_window_frames':sum(r['complete_1s'] for r in rows)}
    for name in ('v','eta'):
        result[name]={}
        for key in ('norm_of_sum','sum_of_norms','max_transported_single_norm','max_original_single_rate'):
            values=np.array([r['window'][name][key] for r in rows if r['complete_1s']])
            result[name][key]=None if not len(values) else {'median':float(np.median(values)),'p95':float(np.percentile(values,95)),'maximum':float(values.max())}
    return result

def run(cache,features,decoder,reconstruction,config,out):
    out=Path(out);out.mkdir(parents=True,exist_ok=False)
    config_text=Path(config).read_text()
    for key,value in [('ltv_observer_max_imu_dt',MAX_IMU_DT),('ltv_observer_reset_gap',RESET_GAP)]:
        matches=[line.split(':',1)[1].strip() for line in config_text.splitlines() if line.startswith(key+':')]
        if len(matches)!=1 or float(matches[0])!=value:raise ValueError('Frozen physical gap contract mismatch '+key)
    logs=[json.loads(s) for s in Path(features).read_text().splitlines()];origin=logs[0]['camera_ns']*1e-9
    logs={r['camera_ns']*1e-9:r for r in logs}
    source=Path(reconstruction);validation=json.loads((source/'summary.json').read_text())
    if validation['status']!='ALIGNED' or validation['identity'].get(str(Path(cache)))!=sha(cache) or validation['identity'].get(str(Path(features)))!=sha(features):raise ValueError('Reconstruction/source identity not aligned')
    diagnostics={r['time']:r for r in map(json.loads,(source/'camera_diagnostics.jsonl').read_text().splitlines())}
    samples=deque();entries=deque();previous=None;segment_start=None;breaks=Counter();all_rows=[];max_transport_check=0.
    process=subprocess.Popen([str(decoder),str(cache)],stdout=subprocess.PIPE,stderr=(out/'decoder_stderr.log').open('w'),text=True)
    try:
        with (out/'shared_transition.jsonl').open('w') as output,(out/'actual_camera_geometry.jsonl').open('w') as geometry:
            for line in process.stdout:
                r=json.loads(line)
                if r['kind']=='imu':samples.append(r);continue
                if r['kind']=='pause':breaks['pause']+=1;previous=None;segment_start=None;entries.clear();continue
                log=logs[r['time']];d=diagnostics.get(r['time']);current=bool(log['raw_current'])
                geometry.write(json.dumps({'time':r['time'],'camera_ns':log['camera_ns'],'epoch':r['epoch'],'calibration':r['calibration'],'managed_observations':r['observations']},allow_nan=False)+'\n')
                reason=None
                if previous is None:reason='first_state'
                elif previous['epoch']!=r['epoch']:reason='epoch_change'
                elif not previous['current'] or not current:reason='cold_dormant_or_unavailable'
                elif previous['bootstrap_count']!=log['bootstrap_count']:reason='bootstrap'
                elif r['target']<=previous['cursor'] or r['target']-previous['cursor']>RESET_GAP:reason='camera_physical_gap'
                elif d is None:reason='missing_aligned_reconstruction'
                if reason:
                    breaks[reason]+=1;entries.clear();segment_start=r['target'] if current else None
                else:
                    c={k:np.array(v).reshape(3,3) for k,v in r['calibration'].items() if k not in ('offset','p_BC')}
                    Phi=transition(list(samples),previous['cursor'],r['target'],c,np.array(r['ba']),np.array(r['bg']))
                    # An independent differential check against the already aligned mean path.
                    before=np.array(previous['x'][-6:]);pred=np.r_[d['predicted_v'],d['predicted_eta']]
                    # Acceleration is affine; use the old-state eta block, which is homogeneous.
                    error=float(np.max(np.abs((Phi@before)[3:]-pred[3:])))
                    max_transport_check=max(max_transport_check,error)
                    if error>1e-9:raise ValueError('Phi homogeneous eta disagrees with aligned prediction')
                    for entry in entries:entry['transported']=Phi@entry['transported']
                    while entries and entries[0]['time']<=r['target']-WINDOW_SECONDS:entries.popleft()
                    delta=np.r_[d['delta_v'],d['delta_eta']]
                    entries.append(dict(time=r['target'],delta=delta,transported=delta.copy(),dt=d['dt']))
                    row={'time':r['time'],'relative_time':r['time']-origin,'epoch':r['epoch'],'dt':d['dt'],'Phi':Phi.tolist(),'window_seconds':WINDOW_SECONDS,
                         'complete_1s':segment_start is not None and r['target']-segment_start>=WINDOW_SECONDS,'window':window_metrics(entries)}
                    output.write(json.dumps(row,allow_nan=False)+'\n');all_rows.append(row)
                previous=dict(r,current=current,bootstrap_count=log['bootstrap_count'])
                while len(samples)>2 and samples[1]['time']<=r['target']:samples.popleft()
        if process.wait()!=0:raise RuntimeError('Cache decoder failed')
    except Exception:
        process.kill();process.wait();raise
    result={'status':'DESCRIPTIVE_ONLY','window_seconds':WINDOW_SECONDS,'window_interval':'(t-1s,t], transported to current body coordinates; no gap-spanning across reset/pause/bootstrap',
            'gap_contract':{'max_imu_dt':MAX_IMU_DT,'reset_gap':RESET_GAP,'irregular_camera_cadence':'actual dt retained below frozen physical gap; no invented camera outputs'},
            'breaks':dict(breaks),'max_homogeneous_eta_prediction_difference':max_transport_check,'all':summary_stats(all_rows),
            'fixed_windows':{label:{'start_s':a,'end_s':b,**summary_stats([r for r in all_rows if a-1e-6<=r['relative_time']<=b+1e-6])} for label,a,b in WINDOWS},
            'interpretation':'Signed cancellation describes corrections only; neither accuracy nor readiness. v and eta norms reported separately because units differ.',
            'identity':{str(Path(p)):sha(p) for p in (cache,features,decoder,config,__file__,source/'summary.json',source/'camera_diagnostics.jsonl')}}
    (out/'summary.json').write_text(json.dumps(result,indent=2)+'\n');return result
if __name__=='__main__':
    p=argparse.ArgumentParser()
    for key in ('cache','features','decoder','reconstruction','config','out'):p.add_argument('--'+key,required=True)
    print(json.dumps(run(**vars(p.parse_args())),indent=2))
