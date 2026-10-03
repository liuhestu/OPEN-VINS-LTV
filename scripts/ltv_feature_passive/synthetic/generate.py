"""Freeze synthetic sensor/prior inputs separately from evaluator-only truth.

No observer code here. Historical scene geometry is reused without rerunning an
old experiment. Confirmation generation requires the main agent's frozen hash.
"""
import os
for name in ['OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS']:
    os.environ[name]='1'
import argparse
import hashlib
import json
from pathlib import Path
import sys
import numpy as np
ROOT=Path(__file__).resolve().parents[3]
sys.path.append(str(ROOT/'scripts/ltv_standalone_study'))
from truth_model import sample,truth,geometry,ids_at,rot


def sha(path): return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def tangent_noise(z,rng,std):
    raw=rng.normal(0,std,z.shape)
    tangent=raw-np.sum(raw*z,axis=-1,keepdims=True)*z
    noisy=z+tangent
    return noisy/np.linalg.norm(noisy,axis=-1,keepdims=True)

def generate(out,scene,seed,lifetime,duration=60.,noise=True,confirmation_freeze_sha=None):
    if scene not in ('REGULAR','FAST') or seed not in (42,43,101,102) or lifetime not in (.5,1.):
        raise ValueError('Outside frozen synthetic scope')
    if not 0<duration<=60: raise ValueError('Invalid duration')
    if seed in (101,102):
        frozen=ROOT/'docs/ltv/feature_readiness_passive/frozen_config.json'
        if not frozen.exists() or not confirmation_freeze_sha or sha(frozen)!=confirmation_freeze_sha:
            raise RuntimeError('Confirmation inputs require explicit matching main freeze hash')
    out=Path(out);out.mkdir(parents=True,exist_ok=False)
    protocol=ROOT/'docs/ltv/feature_readiness_passive/protocol.json'
    config={'scene':scene,'seed':seed,'lifetime':lifetime,'duration':duration,'noise':bool(noise),
            'imu_hz':200,'camera_hz':20,'stereo_baseline_camera_x_m':.11,
            'protocol_sha':sha(protocol),'generator_sha':sha(__file__),
            'truth_model_sha':sha(sys.modules['truth_model'].__file__),
            'confirmation_freeze_sha':confirmation_freeze_sha}
    imu_times=np.arange(round(duration*200)+1)/200
    cam_times=np.arange(round(duration*20)+1)/20
    points,Rbc,pc=geometry(scene);right_pc=pc+Rbc@np.array([.11,0.,0.])
    streams=np.random.SeedSequence(seed).spawn(5)
    imu_rng,bearing_rng,right_rng,pose_rng,jitter_rng=[np.random.default_rng(s) for s in streams]
    imu=np.array([np.r_[sample((i-.5)/200,scene)[1],sample((i-.5)/200,scene)[2]] for i in range(1,len(imu_times))])
    if noise:
        imu[:,:3]+=imu_rng.normal(0,.02,imu[:,:3].shape)
        imu[:,3:]+=imu_rng.normal(0,.002,imu[:,3:].shape)
    true_R=[];true_p=[];true_x=[];left=[];right=[];ids=[]
    for t in cam_times:
        R,p,*_=truth(t,scene);x,_,_,z=sample(t,scene)
        ell=x[:-6].reshape(-1,3)
        zr=(ell-right_pc)@Rbc;zr/=np.linalg.norm(zr,axis=1,keepdims=True)
        true_R.append(R);true_p.append(p);true_x.append(x);left.append(z);right.append(zr)
        ids.append(ids_at(round(t*20),20,lifetime,len(points)))
    true_R=np.array(true_R);true_p=np.array(true_p);left=np.array(left);right=np.array(right)
    if noise:
        left=tangent_noise(left,bearing_rng,np.deg2rad(.05));right=tangent_noise(right,right_rng,np.deg2rad(.05))
    std=np.array([.01]*3+[np.deg2rad(.1)]*3)
    jitter_std=np.array([.002]*3+[np.deg2rad(.02)]*3)
    pose_noise=pose_rng.normal(size=(len(cam_times),6))*std
    a=np.exp(-.05/.5)
    for j in range(1,len(cam_times)):
        pose_noise[j]=a*pose_noise[j-1]+np.sqrt(1-a*a)*pose_noise[j]
    pose_noise+=jitter_rng.normal(size=pose_noise.shape)*jitter_std
    if not noise: pose_noise[:]=0
    prior_R=[];prior_p=[]
    for R,p,eps in zip(true_R,true_p,pose_noise):
        c=p+R@pc+eps[:3];Rn=R@Rbc@rot(eps[3:])@Rbc.T
        prior_R.append(Rn);prior_p.append(c-Rn@pc)
    # Estimator input includes no physical point positions, noise realization,
    # true pose/state, future lifespan or reliability label. IDs are causal only.
    np.savez_compressed(out/'inputs.npz',imu_times=imu_times,imu=imu,camera_times=cam_times,
        feature_ids=np.array(ids,dtype=np.int64),bearings_left=left,bearings_right=right,
        prior_R_WB=np.array(prior_R),prior_p_WB=np.array(prior_p),
        R_BC=np.array([Rbc,Rbc]),p_BC=np.array([pc,right_pc]),
        pose_model_std=std if noise else np.zeros(6),pose_model_jitter=jitter_std if noise else np.zeros(6),
        pose_correlation_seconds=np.array(.5),bearing_sigma_rad=np.array(np.deg2rad(.05) if noise else 0.))
    truth_200=np.array([sample(t,scene)[0] for t in imu_times])
    np.savez_compressed(out/'labels.npz',times=imu_times,state_body=truth_200,
        camera_times=cam_times,R_WB=true_R,p_WB=true_p,points_W=points,
        pose_noise_camera=pose_noise,feature_ids=np.array(ids,dtype=np.int64),
        physical_indices=np.tile(np.arange(len(points)),(len(cam_times),1)))
    config['input_sha']=sha(out/'inputs.npz');config['labels_sha']=sha(out/'labels.npz')
    (out/'identity.json').write_text(json.dumps(config,indent=2))
    return config

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);p.add_argument('--scene',choices=['REGULAR','FAST'],required=True)
    p.add_argument('--seed',type=int,required=True);p.add_argument('--lifetime',type=float,required=True);p.add_argument('--duration',type=float,default=60.)
    p.add_argument('--no-noise',action='store_true');p.add_argument('--confirmation-freeze-sha')
    a=p.parse_args();print(json.dumps(generate(a.out,a.scene,a.seed,a.lifetime,a.duration,not a.no_noise,a.confirmation_freeze_sha),indent=2))
