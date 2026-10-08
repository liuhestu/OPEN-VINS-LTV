"""New far-pressure identity using unchanged historical numerical formulas.

The source archive is new, not a relabeling of an old experiment. Only input
production is implemented here; the observer never receives evaluator labels.
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
ROOT=Path(__file__).resolve().parents[4]
sys.path.append(str(ROOT/'docs/experiments/tools/ltv_standalone_study'))
from truth_model import sample,truth,geometry,ids_at,rot
import importlib.util
_bridge_spec=importlib.util.spec_from_file_location('_r4_readonly_bridge',ROOT/'docs/experiments/tools/ltv_passive_hardening/bridge_historical_inputs.py')
_bridge=importlib.util.module_from_spec(_bridge_spec);_bridge_spec.loader.exec_module(_bridge)
bridge=_bridge.bridge


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def validate_seed(seed,confirmation_freeze_sha=None):
    if seed in (42,43):return None
    frozen=ROOT/'docs/ltv/active_landmark_consistency_r4/frozen_config.json'
    if seed not in (201,202,301,302) or not frozen.is_file():
        raise ValueError('Unregistered or unfrozen confirmation seed')
    digest=sha(frozen)
    contract=json.loads(frozen.read_text())
    if confirmation_freeze_sha!=digest or seed not in contract.get('confirmation_seeds',[]):
        raise ValueError('Confirmation seed requires matching freeze SHA and explicit confirmation_seeds membership')
    return digest


def tangent_noise(z,rng,std):
    raw=rng.normal(0,std,z.shape)
    tangent=raw-np.sum(raw*z,axis=-1,keepdims=True)*z
    noisy=z+tangent
    return noisy/np.linalg.norm(noisy,axis=-1,keepdims=True)


def generate_source(out,seed,duration=.2,confirmation_freeze_sha=None):
    validate_seed(seed,confirmation_freeze_sha)
    if not 0<duration<=60 or abs(duration*20-round(duration*20))>1e-9:
        raise ValueError('Duration must be on the camera grid and at most60s')
    scene='REGULAR';lifetime=.5;noise=True
    out=Path(out);out.mkdir(parents=True,exist_ok=False)
    protocol=ROOT/'docs/ltv/passive_hardening_v2/protocol.json'
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


def generate(out,seed,duration=60.,confirmation_freeze_sha=None):
    validate_seed(seed,confirmation_freeze_sha)
    out=Path(out);source=out.with_name(out.name+'_source')
    if out.exists() or source.exists():raise FileExistsError('Output/source identity already exists')
    generate_source(source,seed,duration,confirmation_freeze_sha)
    result=bridge(source,out)
    result.update(unseen_confirmation=seed>=200,confirmation_freeze_sha=confirmation_freeze_sha,
                  producer_sha=sha(__file__),source_kind='NEW_FAR_PRESSURE_FROM_HISTORICAL_FORMULAS',
                  original_generator_sha=sha(ROOT/'docs/experiments/tools/ltv_feature_passive/synthetic/generate.py'),
                  geometry_name='ORIGINAL_FAR_REGULAR_NEW_RANDOM_SEED',
                  historical_prefix_note='bridge archival prefix denotes pre-bridge source, not a reused historical experiment')
    (out/'identity.json').write_text(json.dumps(result,indent=2)+'\n')
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out',required=True);parser.add_argument('--seed',type=int,required=True)
    parser.add_argument('--duration',type=float,default=60.);parser.add_argument('--confirmation-freeze-sha')
    print(json.dumps(generate(**vars(parser.parse_args())),indent=2))
