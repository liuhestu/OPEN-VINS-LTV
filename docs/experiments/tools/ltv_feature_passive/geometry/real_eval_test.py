"""Fabricated reference/support tests: no real dataset or confirmation reads."""
import os
for key in ['OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS']:os.environ[key]='1'
import argparse
import copy
import json
from pathlib import Path
import sys
import numpy as np
from scipy.spatial.transform import Rotation
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from real_evaluate import load_gt,physical_reference,local_derivative,evaluate,packet_weights,manager_metrics

def fixture(root,ready_fraction=.8,raw_bad=False,injection=False):
    root.mkdir(parents=True,exist_ok=False)
    R=Rotation.from_rotvec([.2,-.4,.1]);q=R.as_quat();gravity=9.81
    gt_times=np.arange(0,20.001,.005)+1000
    velocity=np.array([1.,.2,-.1]);p=(gt_times-1000)[:,None]*velocity
    gt=np.c_[gt_times*1e9,p,np.tile(q[[3,0,1,2]],(len(gt_times),1)),np.tile(velocity,(len(gt_times),1))]
    np.savetxt(root/'gt.csv',gt,delimiter=',',header='fabricated EuRoC schema',comments='#')
    times=np.rint((np.arange(0,20.001,.05)+1000)*1e9)*1e-9
    vb=R.inv().apply(velocity);eta=R.inv().apply([0,0,-gravity]);qmat=np.tile(q,(len(times),1))
    trajectory=np.c_[times,qmat,(times-1000)[:,None]*velocity,np.tile(velocity,(len(times),1)),np.zeros((len(times),6))]
    audit_header='camera_ns,initialized,state_time,clones,state_digest,tracker_digest,visual_digest,auxiliary_receipts,gravity_submissions,velocity_submissions\n'
    for mode in ['B','P_OLD','P_NEW']:
        path=root/mode;path.mkdir()
        np.savetxt(path/'trajectory.csv',trajectory,delimiter=',',header='timestamp,qx,qy,qz,qw,px,py,pz,vx,vy,vz,bgx,bgy,bgz,bax,bay,baz',comments='')
        audit=[];features=[]
        for j,t in enumerate(times):
            ready=j<int(len(times)*ready_fraction)
            # Outside NEW readiness, OLD also has large error: verify same-support comparison.
            dv=1. if ready else 2.
            if mode=='P_NEW':dv=.7 if ready else (10. if raw_bad else 2.)
            row={'camera_ns':round(t*1e9),'initialized':True,'target_imu_time':t,'imu_time':t,'observer_started':mode!='B','available':mode!='B',
                'ready_G':ready and mode!='B','ready_V':ready and mode!='B','v_body':(vb+[dv,0,0]).tolist(),'eta_body':(eta+[.2,0,0]).tolist(),
                'epoch':1,'state_features':20,'mature_features':20,'current_cam0_ids':[1],'current_stereo_ids':[1],'tracks':[],'seeds':[]}
            features.append(row)
            inject=int(injection and mode=='P_NEW')
            digests=','.join(c*40 for c in 'abc')
            audit.append(f"{round(t*1e9)},1,{t},10,{digests},0,{inject},0\n")
        (path/'features.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in features))
        (path/'audit.csv').write_text(audit_header+''.join(audit))
        (path/'replay.json').write_text(json.dumps({'complete':True,'camera_packets':len(times),'input_camera_packets':len(times),
             'imu_consumed':4001,'input_imu_samples':4001,'actual_G_submissions':int(injection and mode=='P_NEW'),'actual_V_submissions':0,'output_rows':len(times),'mode':mode}))
        (path/'effective_options.json').write_text(json.dumps({'ltv_enable_gravity':False,'ltv_enable_velocity':False,'ltv_passive_audit_enabled':True,'ltv_enabled':mode!='B','ltv_feature_readiness_enabled':mode=='P_NEW'}))
        (path/'identity.json').write_text(json.dumps({'mode':mode,'source':'STEREO','config_sha':'fabricated_explicit_contract'}))
    return times,vb,eta

def main():
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);a=p.parse_args();a.out.mkdir(parents=True,exist_ok=False)
    times,vb,eta=fixture(a.out/'good')
    # V1_03 explicitly parses official CSV and velocity columns, unlike legacy membership.
    gt=load_gt('V1_03_difficult',a.out/'good/gt.csv');ref=physical_reference(gt,times)
    assert np.max(abs(ref['v']-vb))<1e-12 and np.max(abs(ref['eta']-eta))<1e-12
    results={}
    for name,fraction,bad,injection in [('good',.8,False,False),('sparse',.01,False,False),('raw_bad',.8,True,False),('injected',.8,False,True)]:
        if name!='good':fixture(a.out/name,fraction,bad,injection)
        folder=a.out/name
        results[name]=evaluate('V1_01_easy',folder/'gt.csv',folder/'B',folder/'P_OLD',folder/'P_NEW',folder/'evaluation')
    assert results['good']['status']=='MEETS_SEQUENCE_OUTPUT_TARGET'
    assert abs(results['good']['comparison']['NEW_ready_V']['P_OLD']['v_RMSE']-1)<1e-12
    assert results['good']['comparison']['raw_common']['P_OLD']['v_RMSE']>1
    assert results['sparse']['coverage_status']=='INSUFFICIENT_COVERAGE'
    assert results['raw_bad']['qualifying_improvement'] and not results['raw_bad']['raw_non_degradation']
    assert results['raw_bad']['status']=='NOT_MET'
    assert not results['injected']['engineering']['pass']
    # A stale snapshot must not become valid by fitting a time offset or shrinking support.
    stale=a.out/'stale';fixture(stale)
    rows=[json.loads(line) for line in (stale/'P_NEW/features.jsonl').read_text().splitlines()]
    rows[10]['imu_time']+=.002
    (stale/'P_NEW/features.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows))
    badtime=evaluate('V1_01_easy',stale/'gt.csv',stale/'B',stale/'P_OLD',stale/'P_NEW',stale/'evaluation')
    assert badtime['status']=='BLOCKED_CORRECTNESS' and badtime['coverage']['P_NEW']['stale_packets']==1
    # Cubic polynomial confirms frozen derivative with nontrivial body frame.
    t=np.arange(0,2.001,.005);position=np.c_[t**3,2*t**2,3*t]
    query=np.array([.2,.5,1.,1.5,1.8]);expected=np.c_[3*query**2,4*query,3*np.ones(len(query))]
    np.testing.assert_allclose(local_derivative(t,position,query),expected,atol=1e-10)
    R=Rotation.from_rotvec([.2,-.4,.1]);q=np.tile(R.as_quat(),(len(t),1))
    np.savetxt(a.out/'uzh_fabricated.txt',np.c_[t,position,q])
    ref=physical_reference(load_gt('indoor_forward_3',a.out/'uzh_fabricated.txt'),query)
    np.testing.assert_allclose(ref['v'],R.inv().apply(expected),atol=1e-10)
    np.testing.assert_allclose(np.linalg.norm(ref['eta'],axis=1),9.8065,atol=1e-12)
    boundary=physical_reference(load_gt('indoor_forward_3',a.out/'uzh_fabricated.txt'),np.array([0.,.02,2.]))
    assert np.isnan(boundary['v']).all() and np.isfinite(boundary['eta']).all()
    weights=packet_weights(np.array([0.,.05,.1,.15]));np.testing.assert_allclose(weights,[.05,.05,.05,0.],atol=1e-14)
    assert abs(weights.sum()-.15)<1e-14
    # More raw eligible tracks than the manager cache; denominator must not shrink.
    overflow=[]
    for epoch in [0,1]:
        for k in range(3):
            t=epoch+0.05*k
            overflow.append({'initialized':True,'epoch':epoch,'camera_ns':round(t*1e9),'target_imu_time':t,
                'current_cam0_ids':list(range(300)),'current_stereo_ids':list(range(299)),
                'state_features':0,'mature_features':0,'seeds':[],
                'tracks':[{'id':0,'ever_opportunity':k==2,'phase':'CANDIDATE','reason':'capacity'}]})
    pop=manager_metrics(overflow,'STEREO')
    assert pop['opportunity_tracks']==598 and pop['manager_opportunity_tracks']==2
    assert pop['candidate_tracks']==600 and pop['startup_epoch0']['opportunity_tracks']==299
    assert pop['observer_positive_epochs']['opportunity_tracks']==299 and pop['raw_opportunities_not_in_manager']==596
    assert manager_metrics(overflow,'TEMPORAL_POSE')['opportunity_tracks']==600
    assert manager_metrics(overflow,'STEREO_THEN_TEMPORAL')['opportunity_tracks']==600
    # A dropped middle frame breaks continuity, even when the same ID reappears.
    gap=copy.deepcopy(overflow[:3]);gap[1]['current_cam0_ids']=[];gap[1]['current_stereo_ids']=[]
    assert manager_metrics(gap,'STEREO')['opportunity_tracks']==0
    summary={'status':'PASS','cases':list(results),'classification_V1_03_official_csv':True,'UZH_derivative_width_s':.1,
             'no_fitted_frame_scale_time':True,'same_ready_support':True,'sparse_and_raw_degradation_rejected':True,
             'actual_injection_rejected':True,'endpoint_duration_no_extra_packet':True,'raw_overflow_denominator_and_startup_epochs':True,'data_source':'fabricated only'}
    (a.out/'result.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary,indent=2))
if __name__=='__main__':main()
