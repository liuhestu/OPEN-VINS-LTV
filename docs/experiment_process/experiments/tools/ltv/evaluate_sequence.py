#!/usr/bin/env python3
"""Frozen prospective ATE protocol: shared camera support, nearest GT <=20ms, SE(3), no scale."""
import argparse,csv,hashlib,json,pathlib
import numpy as np
from scipy.spatial.transform import Rotation
from run_sequence import ASL,OUT,sha
PROTOCOL=dict(version=1,association_tolerance_s=0.02,camera_tolerance_s=1e-6,min_coverage=0.99,alignment='SE3_no_scale',initialization='first_B_output_no_additional_exclusion',orientation='JPL_GtoI_components_as_Hamilton_ItoG',velocity='official_state_GT_velocity_rotated_by_each_run_SE3')
def nearest(reference,query):
    hi=np.clip(np.searchsorted(reference,query),0,len(reference)-1);lo=np.maximum(hi-1,0)
    return np.where(np.abs(reference[lo]-query)<=np.abs(reference[hi]-query),lo,hi)
def align(reference,estimate):
    a=estimate-estimate.mean(axis=0);b=reference-reference.mean(axis=0)
    u,_,vt=np.linalg.svd(a.T@b);sign=np.ones(3);sign[-1]=np.linalg.det(vt.T@u.T)
    r=vt.T@np.diag(sign)@u.T;t=reference.mean(axis=0)-r@estimate.mean(axis=0)
    return r,t
def metrics(gt,estimate):
    r,t=align(gt[:,1:4],estimate[:,5:8]);err=(r@estimate[:,5:8].T).T+t-gt[:,1:4]
    rg=Rotation.from_quat(gt[:,[5,6,7,4]]).as_matrix();re=Rotation.from_quat(estimate[:,1:5]).as_matrix()
    angle=Rotation.from_matrix(np.einsum('nij,njk->nik',rg.transpose(0,2,1),r@re)).magnitude()
    velocity=(r@estimate[:,8:11].T).T-gt[:,8:11]
    return dict(ate_rmse_m=float(np.sqrt(np.mean(np.sum(err*err,axis=1)))),rotation_rmse_deg=float(np.sqrt(np.mean(angle*angle))*180/np.pi),velocity_rmse_mps=float(np.sqrt(np.mean(np.sum(velocity*velocity,axis=1)))),alignment_R=r.tolist(),alignment_t=t.tolist(),scale=1.0,samples=len(gt))
def self_test():
    rng=np.random.default_rng(42);n=100;x=rng.normal(size=(n,3));r=Rotation.from_rotvec([0.3,-0.2,0.1]).as_matrix();t=np.array([1.,2.,3.]);rr,tt=align(x@r.T+t,x)
    assert np.linalg.norm(rr-r)<1e-12 and np.linalg.norm(tt-t)<1e-12
    y=2*x@r.T+t;rs,ts=align(y,x);assert np.sqrt(np.mean(np.sum((x@rs.T+ts-y)**2,axis=1)))>0.1
    gt=np.zeros((n,17));est=np.zeros((n,17));gt[:,0]=est[:,0]=np.arange(n)*0.05;est[:,5:8]=x;gt[:,1:4]=x@r.T+t
    est[:,4]=1;gt[:,4:8]=np.roll(Rotation.from_matrix(r).as_quat(),1);est[:,8:11]=rng.normal(size=(n,3));gt[:,8:11]=est[:,8:11]@r.T
    m=metrics(gt,est);assert m['ate_rmse_m']<1e-12 and m['rotation_rmse_deg']<1e-10 and m['velocity_rmse_mps']<1e-12
    assert nearest(np.array([0.,0.01,0.02]),np.array([0.005,0.019])).tolist()==[0,2]
    print('evaluator self-test PASS: known rigid transform, orientation/velocity, no scale, nearest association')
def evaluate(sequence,paths):
    base=ASL/sequence/'mav0';gt=np.loadtxt(base/'state_groundtruth_estimate0/data.csv',delimiter=',',comments='#');gt[:,0]*=1e-9
    cam=np.array([int(row[0])*1e-9 for row in csv.reader((base/'cam0/data.csv').open()) if row and not row[0].startswith('#')])
    result=dict(sequence=sequence,protocol=PROTOCOL,evaluator_sha256=sha(__file__),gt_sha256=sha(base/'state_groundtruth_estimate0/data.csv'),modes={})
    trajectories={}
    for mode,path in paths.items():
        path=pathlib.Path(path);replay=json.loads((path/'replay.json').read_text()) if (path/'replay.json').exists() else {}
        record=dict(path=str(path),replay=replay,status='FAIL')
        result['modes'][mode]=record
        if not replay.get('complete') or replay.get('camera_packets')!=replay.get('input_camera_packets') or replay.get('imu_consumed')!=replay.get('input_imu_samples'):
            record['reason']='incomplete_input';continue
        estimate=np.loadtxt(path/'trajectory.csv',delimiter=',',skiprows=1,ndmin=2)
        if estimate.shape[0]<3 or estimate.shape[1]!=17 or not np.isfinite(estimate).all() or np.any(np.diff(estimate[:,0])<=0):record['reason']='invalid_trajectory';continue
        trajectories[mode]=estimate;record.update(start=float(estimate[0,0]),end=float(estimate[-1,0]),rows=len(estimate),trajectory_sha256=sha(path/'trajectory.csv'))
    if 'B' not in trajectories:result['reason']='baseline_failed_no_relative_comparison';return result
    # The experiment has a fixed zero cam/IMU offset (asserted before running it).
    first=trajectories['B'][0,0];sensor_support=cam[cam>=first-PROTOCOL['camera_tolerance_s']];support=sensor_support.copy()
    gi=nearest(gt[:,0],support);within=np.abs(gt[gi,0]-support)<=PROTOCOL['association_tolerance_s'];support=support[within];gi=gi[within]
    result['expected_support_rows']=len(support);result['gt_support_start']=float(support[0]);result['gt_support_end']=float(support[-1]);common=np.ones(len(support),dtype=bool);indices={}
    for mode,e in trajectories.items():
        rec=result['modes'][mode];idx=nearest(e[:,0],support);present=np.abs(e[idx,0]-support)<=PROTOCOL['camera_tolerance_s'];coverage=float(present.mean());indices[mode]=idx
        gaps=[];start=None
        for i,available in enumerate(np.r_[present,True]):
            if not available and start is None:start=i
            if available and start is not None:gaps.append((start,i));start=None
        longest=max((b-a for a,b in gaps),default=0)
        sensor_idx=nearest(e[:,0],sensor_support);sensor_coverage=float((np.abs(e[sensor_idx,0]-sensor_support)<=PROTOCOL['camera_tolerance_s']).mean())
        rec.update(sensor_output_coverage=sensor_coverage,coverage=coverage,missing_rows=int((~present).sum()),longest_missing_packets=longest,initialization_delta_s=float(e[0,0]-first))
        if coverage<PROTOCOL['min_coverage'] or sensor_coverage<PROTOCOL['min_coverage'] or abs(e[0,0]-first)>PROTOCOL['camera_tolerance_s']:rec['reason']='coverage_or_initialization_mismatch';continue
        rec['status']='VALID';common &=present
    result['common_support_rows']=int(common.sum());result['common_support_fraction']=float(common.mean())
    for mode,e in trajectories.items():
        rec=result['modes'][mode]
        if rec['status']!='VALID':continue
        if common.mean()<PROTOCOL['min_coverage']:rec.update(status='FAIL',reason='common_support_below_99_percent');continue
        rec.update(metrics(gt[gi[common]],e[indices[mode][common]]))
    return result
if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--self-test',action='store_true');p.add_argument('--sequence');p.add_argument('--paths',type=pathlib.Path);p.add_argument('--output',type=pathlib.Path);a=p.parse_args()
    if a.self_test:self_test()
    else:
        frozen=OUT/'evaluator_protocol.json'
        if not frozen.exists() or json.loads(frozen.read_text())['evaluator_sha256']!=sha(__file__):raise SystemExit('freeze evaluator hash before real results')
        result=evaluate(a.sequence,json.loads(a.paths.read_text()));a.output.write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
