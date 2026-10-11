"""Independent local Euler/Riccati reconstruction anchored to each cached predecessor.

Only two fixed +/-0.5s neighborhoods. Not a full observer or leave-one-out run.
"""
import os
for k in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS'):os.environ[k]='1'
import argparse
from collections import deque
import json
from pathlib import Path
import subprocess
import numpy as np
from reconstruct_prediction import interpolate,skew,sha

TARGETS=(1413394964105760512,1413394966255760384)
RELATIVE_TOLERANCE=1e-9
DELTA_ABSOLUTE_TOLERANCE=1e-9

def sanitize(P,cfg,diagnostics):
    P=.5*(P+P.T);w,U=np.linalg.eigh(P);scale=max(1.,float(np.max(np.abs(w))))
    if w[0]<cfg['covariance_failure_threshold']*scale:raise ValueError('Actual covariance failure threshold reached')
    diagnostics['calls']+=1;diagnostics['floor_clamped_values']+=int(np.count_nonzero(w<cfg['covariance_floor']))
    diagnostics['minimum_pre_floor_eigenvalue']=min(diagnostics['minimum_pre_floor_eigenvalue'],float(w[0]))
    # Unconditionally reconstruct, even if no eigenvalue is clamped: core does so.
    result=(U*np.maximum(w,cfg['covariance_floor']))@U.T
    return .5*(result+result.T)

def system_matrix(n,omega):
    A=np.zeros((n,n));W=-skew(omega)
    for k in range((n-6)//3):A[3*k:3*k+3,3*k:3*k+3]=W;A[3*k:3*k+3,-6:-3]=-np.eye(3)
    A[-6:-3,-6:-3]=W;A[-6:-3,-3:]=np.eye(3);A[-3:,-3:]=W
    return A

def prediction(previous,current,samples,cfg,diagnostics):
    x=np.array(previous['x']);P=np.array(previous['P']).reshape(len(x),len(x));c=current['calibration']
    c={k:np.array(v).reshape(3,3) for k,v in c.items() if k not in ('offset','p_BC')}
    ba=np.array(current['ba']);bg=np.array(current['bg']);start,end=previous['cursor'],current['target']
    grid=[interpolate(samples,start)]+[s for s in samples if start<s['time']<end]+[interpolate(samples,end)]
    def correct(r):
        am=c['R_ACCtoIMU']@c['Da']@(np.array(r['am'])-ba)
        return am,c['R_GYROtoIMU']@c['Dw']@(np.array(r['wm'])-bg-c['Tg']@am)
    V=np.diag([cfg['v_landmark']]*(len(x)-6)+[cfg['v_velocity']]*3+[cfg['v_gravity']]*3)
    for left,right in zip(grid,grid[1:]):
        dt=right['time']-left['time']
        if not 0<dt<=cfg['max_imu_dt']:raise ValueError('Local IMU gap')
        aa,aw=correct(left);ba_,bw=correct(right);A=system_matrix(len(x),.5*(aw+bw));u=np.zeros(len(x));u[-6:-3]=.5*(aa+ba_)
        x=x+dt*(A@x+u)
        P=P+dt*(A@P+P@A.T+V) # Original covariance Euler; NOT F P F^T.
        P=sanitize(P,cfg,diagnostics)
    return x,P,len(grid)-1

def lifecycle(x,P,previous,current,cfg):
    old=dict(previous['slots']);ordered=[fid for fid,slot in sorted(current['slots'],key=lambda item:item[1])]
    n=3*len(ordered)+6;indices=[]
    for fid in ordered:indices.extend([3*old[fid]+j for j in range(3)] if fid in old else [-1]*3)
    indices.extend(range(len(x)-6,len(x)));indices=np.array(indices)
    newx=np.zeros(n);newP=np.zeros((n,n));valid=np.flatnonzero(indices>=0)
    newx[valid]=x[indices[valid]];newP[np.ix_(valid,valid)]=P[np.ix_(indices[valid],indices[valid])]
    births={b['id']:b for b in current['births']}
    for slot,fid in enumerate(ordered):
        if fid not in old:
            newP[3*slot:3*slot+3,3*slot:3*slot+3]=cfg['initial_p_landmark']*np.eye(3)
            b=births[fid]
            if b['apply_seed']:newx[3*slot:3*slot+3]=b['mean']
    return newx,newP,ordered

def camera_correction(x,P,current,ordered,dt,cfg,diagnostics):
    observations={o['id']:np.array(o['bearing']) for o in current['observations'] if o['camera']==0}
    visible=[(slot,fid) for slot,fid in enumerate(ordered) if fid in observations]
    H=np.zeros((3*len(visible),len(x)));y=np.zeros(3*len(visible));c=current['calibration'];R=np.array(c['R_BC']).reshape(3,3);pc=np.array(c['p_BC'])
    for j,(slot,fid) in enumerate(visible):
        bearing=observations[fid];bearing=R@(bearing/np.linalg.norm(bearing));projector=np.eye(3)-np.outer(bearing,bearing)
        H[3*j:3*j+3,3*slot:3*slot+3]=projector;y[3*j:3*j+3]=projector@pc
    q=cfg['q_landmark'];rate=max(0.,float(np.linalg.eigvalsh(q*H@P@H.T).max()))
    substeps=max(1,int(np.ceil(dt*rate/cfg['camera_euler_safety'])))
    if substeps>cfg['max_camera_substeps']:raise ValueError('Camera substep limit')
    h=dt/substeps;initial=x.copy();contribution=np.zeros((len(visible),len(x)));step_rows=[]
    for s in range(substeps):
        residual=y-H@x;PC=P@H.T;gain=h*q*PC
        terms=np.array([gain[:,3*j:3*j+3]@residual[3*j:3*j+3] for j in range(len(visible))])
        contribution+=terms
        dx=gain@residual
        step_rows.append(dict(index=s,projection_innovation=residual.tolist(),shared_gain=gain[-6:,:].tolist(),delta_shared=dx[-6:].tolist(),point_shared_terms=terms[:,-6:].tolist()))
        x=x+dx;P=P-h*q*PC@H@P;P=sanitize(P,cfg,diagnostics)
    P=sanitize(P,cfg,diagnostics)
    return x,P,substeps,rate,[fid for _,fid in visible],contribution,step_rows,float(np.max(np.abs(contribution.sum(axis=0)-(x-initial))))

def load_config(path):
    text=Path(path).read_text();keys=['max_imu_dt','reset_gap','q_landmark','v_landmark','v_velocity','v_gravity','initial_p_landmark','covariance_floor','covariance_failure_threshold','camera_euler_safety','max_camera_substeps']
    result={}
    for key in keys:
        values=[line.split(':',1)[1].split('#')[0].strip() for line in text.splitlines() if line.startswith('ltv_observer_'+key+':')]
        if len(values)!=1:raise ValueError('Missing/ambiguous actual configuration '+key)
        result[key]=float(values[0])
    return result

def run(cache,features,reconstruction,decoder,config,out):
    out=Path(out);out.mkdir(parents=True,exist_ok=False);cfg=load_config(config)
    logs={r['camera_ns']*1e-9:r for r in map(json.loads,Path(features).read_text().splitlines())}
    source=Path(reconstruction);validation=json.loads((source/'summary.json').read_text())
    if validation['status']!='ALIGNED' or validation['identity'][str(Path(cache))]!=sha(cache):raise ValueError('Aligned source mismatch')
    mean={r['camera_ns']:r for r in map(json.loads,(source/'camera_diagnostics.jsonl').read_text().splitlines())}
    times=[ns*1e-9 for ns in TARGETS];previous=None;samples=deque();results=[];failure=None
    process=subprocess.Popen([decoder,cache],stdout=subprocess.PIPE,stderr=(out/'decoder_stderr.log').open('w'),text=True)
    try:
        with (out/'events.jsonl').open('w') as output:
            for line in process.stdout:
                r=json.loads(line)
                if r['kind']=='imu':samples.append(r);continue
                if r['kind']=='pause':previous=None;continue
                selected=any(abs(r['time']-t)<=.500001 for t in times)
                if selected:
                    if previous is None or previous['epoch']!=r['epoch'] or r['target']-previous['cursor']>cfg['reset_gap']:raise ValueError('Selected event lacks immediate cached predecessor')
                    log=logs[r['time']];ns=log['camera_ns'];diagnostics=dict(calls=0,floor_clamped_values=0,minimum_pre_floor_eigenvalue=float('inf'))
                    xpred,Ppred,steps=prediction(previous,r,list(samples),cfg,diagnostics)
                    expected_pred=np.r_[mean[ns]['predicted_v'],mean[ns]['predicted_eta']]
                    before_error=float(np.max(np.abs(xpred[-6:]-expected_pred)))
                    x0,P0,ordered=lifecycle(xpred,Ppred,previous,r,cfg)
                    x,P,substeps,rate,ids,terms,substep_rows,sum_error=camera_correction(x0.copy(),P0.copy(),r,ordered,r['target']-previous['target'],cfg,diagnostics)
                    actualx=np.array(r['x']);actualP=np.array(r['P']).reshape(len(actualx),len(actualx))
                    xerror=float(np.linalg.norm(x-actualx)/(1+np.linalg.norm(actualx)));Perror=float(np.linalg.norm(P-actualP)/(1+np.linalg.norm(actualP)))
                    delta_error=float(np.max(np.abs(terms.sum(axis=0)[-6:]-np.r_[mean[ns]['delta_v'],mean[ns]['delta_eta']])))
                    passed=max(xerror,Perror)<=RELATIVE_TOLERANCE and max(delta_error,sum_error,before_error)<=DELTA_ABSOLUTE_TOLERANCE and substeps==r['camera_substeps']
                    result=dict(camera_ns=ns,time=r['time'],epoch=r['epoch'],passed=passed,predecessor_time=previous['time'],imu_steps=steps,substeps=substeps,recorded_substeps=r['camera_substeps'],max_rate=rate,predicted_shared_error=before_error,x_relative_error=xerror,P_relative_error=Perror,delta_absolute_error=delta_error,contribution_telescope_error=sum_error,spectral_protection=diagnostics,
                                slots=ordered,point_ids=ids,point_shared_contributions=terms[:,-6:].tolist(),total_shared_delta=terms.sum(axis=0)[-6:].tolist(),substep_details=substep_rows,
                                predicted_x=xpred.tolist(),predicted_P=Ppred.tolist(),after_lifecycle_x=x0.tolist(),after_lifecycle_P=P0.tolist(),reconstructed_post_x=x.tolist(),reconstructed_post_P=P.tolist())
                    output.write(json.dumps(result,allow_nan=False)+'\n');results.append(result)
                previous=r
                while len(samples)>2 and samples[1]['time']<=r['target']:samples.popleft()
        if process.wait()!=0:raise RuntimeError('Decoder failed')
    except Exception as error:
        failure=str(error);process.kill();process.wait()
    compact=[{k:r[k] for k in ('camera_ns','passed','imu_steps','substeps','recorded_substeps','predicted_shared_error','x_relative_error','P_relative_error','delta_absolute_error','contribution_telescope_error','spectral_protection')} for r in results]
    status='ALIGNED_LOCAL_RECONSTRUCTION' if failure is None and len(results)==42 and all(r['passed'] for r in results) else 'NOT_ALIGNED'
    summary=dict(status=status,relative_tolerance=RELATIVE_TOLERANCE,delta_absolute_tolerance=DELTA_ABSOLUTE_TOLERANCE,frames=len(results),error=failure,events=compact,
                 interpretation='Actual-path additive substep terms, not deleting-feature counterfactual. Predicted P is reconstructed, not separately logged. Final full cached x/P are validation oracle.',
                 identity={str(Path(p)):sha(p) for p in (cache,features,decoder,config,__file__,source/'summary.json')})
    (out/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    if status!='ALIGNED_LOCAL_RECONSTRUCTION':raise RuntimeError(failure or status)
    return summary
if __name__=='__main__':
    p=argparse.ArgumentParser()
    for key in ('cache','features','reconstruction','decoder','config','out'):p.add_argument('--'+key,required=True)
    print(json.dumps(run(**vars(p.parse_args())),indent=2))
