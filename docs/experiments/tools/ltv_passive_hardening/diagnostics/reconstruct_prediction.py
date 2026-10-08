"""Pure offline mean reconstruction. No observer, Riccati, gain or GT invocation."""
import os
for key in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS'):os.environ[key]='1'
import argparse
from collections import Counter,deque
import hashlib
import json
import math
from pathlib import Path
import subprocess
import numpy as np

TOLERANCE=1e-9  # Frozen before opening candidate diagnostics; absolute scalar equality.
def sha(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as f:
        for chunk in iter(lambda:f.read(1024*1024),b''):h.update(chunk)
    return h.hexdigest()
def skew(w):
    a,b,c=w;return np.array([[0.,-c,b],[c,0.,-a],[-b,a,0.]])
def step(x,dt,acc,gyro):
    # Same old-state Euler equation. Explicit dense A preserves the core expression.
    dimension=len(x);A=np.zeros((dimension,dimension));B=np.zeros((dimension,3));W=-skew(gyro)
    for slot in range((dimension-6)//3):
        A[3*slot:3*slot+3,3*slot:3*slot+3]=W;A[3*slot:3*slot+3,-6:-3]=-np.eye(3)
    A[-6:-3,-6:-3]=W;A[-6:-3,-3:]=np.eye(3);A[-3:,-3:]=W;B[-6:-3]=np.eye(3)
    return x+dt*(A@x+B@acc)
def interpolate(samples,t):
    for i,row in enumerate(samples):
        if row['time']>=t:
            if row['time']==t:return row
            if i==0:raise ValueError('Missing left bracket')
            lo=samples[i-1];alpha=(t-lo['time'])/(row['time']-lo['time'])
            return dict(time=t,am=(1-alpha)*np.asarray(lo['am'])+alpha*np.asarray(row['am']),wm=(1-alpha)*np.asarray(lo['wm'])+alpha*np.asarray(row['wm']))
    raise ValueError('Missing right bracket')
def predict(x,samples,start,end,c,ba,bg):
    grid=[interpolate(samples,start)]+[r for r in samples if start<r['time']<end]
    if end>start:grid.append(interpolate(samples,end))
    def corrected(r):
        am=c['R_ACCtoIMU']@c['Da']@(np.asarray(r['am'])-ba)
        wm=c['R_GYROtoIMU']@c['Dw']@(np.asarray(r['wm'])-bg-c['Tg']@am)
        return am,wm
    x=x.copy()
    for a,b in zip(grid,grid[1:]):
        aa,aw=corrected(a);ba_,bw=corrected(b)
        x=step(x,b['time']-a['time'],.5*(aa+ba_),.5*(aw+bw))
    return x

def run(cache,features,decoder,out):
    out=Path(out);out.mkdir(parents=True,exist_ok=False)
    logs={int(r['camera_ns']):r for r in map(json.loads,Path(features).read_text().splitlines())}
    by_time={k*1e-9:v for k,v in logs.items()}
    summary=dict(status='RUNNING',tolerance=TOLERANCE,observer_invoked=False,P_propagated=False,GT_read=False,checked=0,valid_diagnostics=0,skip_reasons={},epochs={},max_scalar_difference={'prediction_angle_p95_rad':0.,'velocity_correction_rate':0.,'gravity_correction_rate':0.},failures=[])
    samples=deque();previous=None;skips=Counter();last_correction=None
    process=subprocess.Popen([str(decoder),str(cache)],stdout=subprocess.PIPE,stderr=(out/'decoder_stderr.log').open('w'),text=True)
    try:
        with (out/'camera_diagnostics.jsonl').open('w') as output:
            for line in process.stdout:
                r=json.loads(line)
                if r['kind']=='imu':samples.append(r);continue
                if r['kind']=='pause':previous=None;last_correction=None;skips['pause_event']+=1;continue
                log=by_time.get(r['time'])
                if log is None:raise ValueError('Cache camera not joined to original integer-ns receipt')
                for key in ('epoch','sequence','camera_substeps','available'):
                    if r[key]!=log[key]:raise ValueError('Cache/log mismatch '+key)
                x=np.array(r['x']);current=bool(log['raw_current'])
                if current:
                    if not np.array_equal(x[-6:-3],log['v_body']) or not np.array_equal(x[-3:],log['eta_body']):raise ValueError('Cache raw mean mismatch')
                reason=None
                if not current or len(x)<6:reason='no_current_post_state'
                elif previous is None:reason='first_state_after_pause_or_startup'
                elif previous['epoch']!=r['epoch']:reason='epoch_transition'
                elif not previous['current']:reason='previous_not_current_cold_or_dormant'
                if reason:
                    skips[reason]+=1;last_correction=r['target'] if current else None
                    previous=dict(r,current=current)
                    continue
                c={k:np.array(v).reshape(3,3) if k not in ('p_BC','offset') else np.array(v) for k,v in r['calibration'].items()}
                predicted=predict(np.array(previous['x']),list(samples),previous['cursor'],r['target'],c,np.array(r['ba']),np.array(r['bg']))
                dt=r['target']-last_correction if last_correction is not None else 0
                oldslots=dict(previous['slots']);postslots=dict(r['slots']);angles=[];points=[]
                births={b['id']:b for b in r['births']}
                for o in r['observations']:
                    if o['camera']!=0:continue
                    fid=o['id'];point=dict(id=fid,in_scalar_p95=False)
                    if fid in oldslots and oldslots[fid]>=0:
                        ell=predicted[3*oldslots[fid]:3*oldslots[fid]+3]
                        ray=ell-c['p_BC'];unit=c['R_BC']@np.array(o['bearing']);unit/=np.linalg.norm(unit)
                        point['pre_landmark']=ell.tolist();point['pre_existing']=True
                        if np.linalg.norm(ray)>=.1 and np.isfinite(ray).all():
                            direction=ray/np.linalg.norm(ray);angle=float(np.arccos(np.clip(direction@unit,-1.,1.)))
                            angles.append(angle);point.update(angle_rad=angle,bearing_residual_body=(direction-unit).tolist(),in_scalar_p95=True)
                    elif fid in births:
                        b=births[fid];point.update(pre_landmark=b['mean'] if b['apply_seed'] else [0.,0.,0.],pre_existing=False)
                    else:raise ValueError('Managed point lacks previous slot or actual birth')
                    slot=postslots[fid];post=x[3*slot:3*slot+3];point['post_landmark']=post.tolist()
                    point['delta_landmark']=(post-np.array(point['pre_landmark'])).tolist();points.append(point)
                dv=x[-6:-3]-predicted[-6:-3];dg=x[-3:]-predicted[-3:]
                valid=dt>0 and len(angles)>=15 and r['camera_substeps']>0
                if valid!=bool(log['correction_diagnostics_valid']):raise ValueError('Scalar validity mismatch')
                reconstructed={'prediction_angle_p95_rad':sorted(angles)[math.ceil(.95*len(angles))-1] if valid else 0.,'velocity_correction_rate':float(np.linalg.norm(dv)/dt) if valid else 0.,'gravity_correction_rate':float(np.linalg.norm(dg)/dt) if valid else 0.}
                for key,value in reconstructed.items():
                    error=abs(value-log[key]);summary['max_scalar_difference'][key]=max(error,summary['max_scalar_difference'][key])
                    if error>TOLERANCE:summary['failures'].append(dict(time=r['time'],key=key,error=error,reconstructed=value,logged=log[key]))
                summary['checked']+=1;summary['valid_diagnostics']+=valid
                ep=summary['epochs'].setdefault(str(r['epoch']),dict(checked=0,valid=0,first_time=r['time'],last_time=r['time']))
                ep['checked']+=1;ep['valid']+=valid;ep['last_time']=r['time']
                output.write(json.dumps(dict(camera_ns=log['camera_ns'],time=r['time'],epoch=r['epoch'],dt=dt,valid=valid,predicted_v=predicted[-6:-3].tolist(),predicted_eta=predicted[-3:].tolist(),delta_v=dv.tolist(),delta_eta=dg.tolist(),scalars=reconstructed,points=points),allow_nan=False)+'\n')
                last_correction=r['target'];previous=dict(r,current=current)
                while len(samples)>2 and samples[1]['time']<=r['target']:samples.popleft()
        if process.wait()!=0:raise RuntimeError('Decoder failed; see stderr')
        summary['status']='ALIGNED' if not summary['failures'] and summary['valid_diagnostics'] else 'NOT_ALIGNED'
    except Exception as error:
        summary['status']='TOOL_ERROR';summary['error']=str(error);process.kill();process.wait()
    summary['skip_reasons']=dict(skips)
    # Cache hashing streams only; no observer input is loaded into the evaluator.
    summary['identity']={str(p):sha(p) for p in (Path(cache),Path(features),Path(decoder),Path(__file__))}
    (out/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    if summary['status']!='ALIGNED':raise RuntimeError(summary.get('error',summary['status']))
    return summary
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--cache',required=True);p.add_argument('--features',required=True);p.add_argument('--decoder',required=True);p.add_argument('--out',required=True)
    print(json.dumps(run(**vars(p.parse_args())),indent=2))
