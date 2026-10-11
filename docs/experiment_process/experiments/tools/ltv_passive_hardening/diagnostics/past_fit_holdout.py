"""Two fixed past-only point fits in a single current OpenVINS pose context."""
import os
for k in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS'):os.environ[k]='1'
import argparse,json,subprocess
from pathlib import Path
import numpy as np
from reconstruct_prediction import sha
TARGETS=((1413394964105760512,27537),(1413394966255760384,29636))

def ray(pose,camera,z):
    R=np.array(pose['R_WB']).reshape(3,3);p=np.array(pose['p_WB']);Rc=np.array(camera['R_BC']).reshape(3,3);pc=np.array(camera['p_BC'])
    z=np.array(z);z=z/np.linalg.norm(z)
    return p+R@pc,R@Rc@z

def select(rows,current,fid):
    times=[p['time'] for p in current['poses']]
    if len(set(times))!=len(times) or not np.isfinite(times).all() or abs(times[current['execution_pose_index']]-current['time'])>1e-9:
        raise ValueError('Invalid current coherent context timestamps')
    for pose in current['poses']:
        R=np.array(pose['R_WB']).reshape(3,3)
        if not np.isfinite(R).all() or not np.isfinite(pose['p_WB']).all() or np.linalg.norm(R.T@R-np.eye(3))>=1e-8:
            raise ValueError('Nonfinite or invalid current pose context')
    support=[];missing=[];observed=[];camera=current['cameras'][0]
    for r in rows:
        if r['epoch']!=current['epoch'] or not current['time']-1.0<=r['time']<current['time']:continue
        for o in r['observations']:
            if o['id']!=fid or o['camera']!=0 or not o['valid']:continue
            observed.append(r['time']);poses=[p for p in current['poses'] if abs(p['time']-r['time'])<=1e-9]
            if len(poses)!=1:missing.append(r['time']);continue
            c,b=ray(poses[0],camera,o['bearing']);support.append(dict(time=r['time'],center=c.tolist(),direction=b.tolist(),bearing=o['bearing']))
    hold=[o for o in current['observations'] if o['id']==fid and o['camera']==0 and o['valid']]
    return dict(id=fid,time=current['time'],version=current['version'],past_observation_times=observed,missing_current_context_clone_times=missing,past=support,current_bearing=hold[0]['bearing'] if len(hold)==1 else None,
                execution_pose=current['poses'][current['execution_pose_index']],camera=camera,current_context_pose_times=[p['time'] for p in current['poses']])

def fit(record):
    past=record['past']
    if len(past)<2 or record['current_bearing'] is None:return {'status':'NOT_IDENTIFIABLE','reason':'insufficient coherent past rays or missing current ray'}
    centers=np.array([p['center'] for p in past]);directions=np.array([p['direction'] for p in past]);Pi=np.eye(3)[None,:,:]-directions[:,:,None]*directions[:,None,:]
    A=Pi.sum(axis=0);eigen=np.linalg.eigvalsh(A)
    if eigen[0]<=1e-12*eigen[-1]:return {'status':'NOT_IDENTIFIABLE','reason':'DEGENERATE','normal_eigenvalues':eigen.tolist()}
    point=np.linalg.solve(A,np.einsum('nij,nj->i',Pi,centers));rays=point-centers;ranges=np.linalg.norm(rays,axis=1)
    if np.any(ranges<1e-10):return {'status':'NOT_IDENTIFIABLE','reason':'NEAR_CENTER'}
    angles=np.arccos(np.clip(np.einsum('ij,ij->i',rays/ranges[:,None],directions),-1.,1.))
    center,direction=ray(record['execution_pose'],record['camera'],record['current_bearing']);v=point-center
    if np.linalg.norm(v)<1e-10:return {'status':'NOT_IDENTIFIABLE','reason':'CURRENT_NEAR_CENTER'}
    signed=np.einsum('ij,ij->i',rays,directions)
    return dict(status='FIT_DIAGNOSTIC_ONLY',point_W=point.tolist(),normal_eigenvalues=eigen.tolist(),condition=float(eigen[-1]/eigen[0]),
                actual_baseline_m=float(np.linalg.norm(centers[:,None,:]-centers[None,:,:],axis=2).max()),past_nonpositive_ray_count=int(np.count_nonzero(signed<=0)),current_nonpositive_ray=bool(v@direction<=0),
                past_count=len(past),past_span=past[-1]['time']-past[0]['time'],past_angle_rad=angles.tolist(),past_max_angle_rad=float(angles.max()),past_median_angle_rad=float(np.median(angles)),
                past_signed_ray_distances=signed.tolist(),heldout_current_angle_rad=float(np.arccos(np.clip(v@direction/np.linalg.norm(v),-1.,1.))),
                current_signed_ray_distance=float(v@direction),current_range=float(np.linalg.norm(v)))

def support(cache,decoder,out):
    out=Path(out);out.mkdir(parents=True,exist_ok=False)
    with (out/'contexts.jsonl').open('w') as stream:subprocess.run([decoder,cache],stdout=stream,check=True)
    rows=[json.loads(l) for l in (out/'contexts.jsonl').read_text().splitlines()];records=[]
    for ns,fid in TARGETS:
        matches=[r for r in rows if r['camera_time']==ns*1e-9]
        if len(matches)!=1:raise ValueError('Target context receipt is not unique')
        record=select(rows,matches[0],fid);record['camera_ns']=ns;records.append(record)
    result={'status':'INPUT_SUPPORT_ONLY_NO_FITS','records':records,'identity':{str(Path(p)):sha(p) for p in (cache,decoder,__file__)}}
    (out/'support.json').write_text(json.dumps(result,indent=2)+'\n')
    return {'records':[{k:r[k] for k in ('id','past_observation_times','missing_current_context_clone_times','current_context_pose_times')} for r in records]}

def execute(support_path,reconstruction,out):
    out=Path(out);out.mkdir(parents=True,exist_ok=False);metadata=json.loads(Path(support_path).read_text())
    diag={r['camera_ns']:r for r in map(json.loads,Path(reconstruction).read_text().splitlines())};results=[]
    for record in metadata['records']:
        result=fit(record);result.update(id=record['id'],camera_ns=record['camera_ns'],missing_clone_times=record['missing_current_context_clone_times'])
        d=next(p for p in diag[record['camera_ns']]['points'] if p['id']==record['id']);result['ltv_pre_angle_rad']=d['angle_rad']
        if result['status']=='FIT_DIAGNOSTIC_ONLY':
            pose=record['execution_pose'];R=np.array(pose['R_WB']).reshape(3,3)
            fitted_B=R.T@(np.array(result['point_W'])-np.array(pose['p_WB']));delta=np.array(d['pre_landmark'])-fitted_B
            direction=np.array(record['camera']['R_BC']).reshape(3,3)@np.array(record['current_bearing']);direction/=np.linalg.norm(direction)
            result.update(fitted_landmark_B=fitted_B.tolist(),ltv_minus_fit_body=delta.tolist(),ltv_minus_fit_parallel=float(delta@direction),ltv_minus_fit_transverse=float(np.linalg.norm(delta-(delta@direction)*direction)))
        results.append(result)
    value={'status':'DIAGNOSTIC_ONLY','GT_read':False,'current_ray_used_in_fit':False,'results':results,'identity':{str(Path(p)):sha(p) for p in (support_path,reconstruction,__file__)}}
    (out/'results.json').write_text(json.dumps(value,indent=2)+'\n');return value
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('mode',choices=['support','fit']);p.add_argument('--cache');p.add_argument('--decoder');p.add_argument('--support');p.add_argument('--reconstruction');p.add_argument('--out',required=True);a=p.parse_args()
    print(json.dumps(support(a.cache,a.decoder,a.out) if a.mode=='support' else execute(a.support,a.reconstruction,a.out),indent=2))
