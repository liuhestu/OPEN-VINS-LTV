"""Bounded-memory streaming observer stress; scheduler must count as synthetic.

No whole-run arrays or unique-ID history live in this process. Accuracy labels,
if needed, are evaluated separately; this is a resource/recovery stress trace.
"""
import os
for name in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS'):
    os.environ[name]='1'
import argparse
from collections import deque
import hashlib
import json
from pathlib import Path
import time
import numpy as np
import sys
sys.path.append(str(Path(__file__).resolve().parents[2]/'scripts/ltv_passive_hardening'))
import synthetic_inputs as generator
import synthetic_runner as runner
from pressure_inputs import validate_seed


def rss_bytes():
    return int(Path('/proc/self/statm').read_text().split()[1])*os.sysconf('SC_PAGE_SIZE')


def run(out,library,calibration,seconds=600.,seed=42,mode='P_NEW_PRESERVE',confirmation_freeze_sha=None):
    validate_seed(seed,confirmation_freeze_sha)
    if seconds<=0 or seconds>600:raise ValueError('Unregistered resource fixture scope')
    if mode not in ('P_NEW','P_NEW_WARMUP','P_NEW_PRESERVE','P_NEW_GRACE','P_NEW_ACTIVE'):raise ValueError('Bounded hardening mode required')
    out=Path(out);out.mkdir(parents=True,exist_ok=False)
    cameras,T=generator.calibration(calibration);Rbc=T[:,:3,:3];pc=T[:,:3,3]
    rngs=[np.random.default_rng(s) for s in np.random.SeedSequence([seed,600,3]).spawn(5)]
    imu_rng,left_rng,right_rng,pose_rng,jitter_rng=rngs
    std=np.array([.01]*3+[np.deg2rad(.1)]*3);jitter=np.array([.002]*3+[np.deg2rad(.02)]*3)
    OU=pose_rng.normal(size=6)*std;alpha=np.exp(-.05/.5)
    poses=deque(maxlen=21);points={};retired=deque(maxlen=60)
    previous_active=set();previous_ttl=set();last_bits=None;last_epoch=None;last_highwater=None
    max_counts={};violations=0;frames=0;rss_start=rss_bytes();peak_rss=rss_start
    coverage={'guard_rejected_ids':0,'active_retirement_events':0,'candidate_ttl_events':0,'old_input_identity_reappearances':0}
    native=runner.Adapter(library,{'P_NEW':1,'P_NEW_WARMUP':2,'P_NEW_PRESERVE':3,'P_NEW_GRACE':4,'P_NEW_ACTIVE':5}[mode],'HYBRID',np.deg2rad(.05),False)
    metadata={'status':'RUNNING','seconds':seconds,'seed':seed,'mode':mode,'library_sha':runner.sha(library),
              'confirmation_freeze_sha':confirmation_freeze_sha,
              'script_sha':runner.sha(__file__),'calibration_sha':runner.sha(calibration),'generator_math_sha':runner.sha(generator.math.__file__),
              'runner_sha':runner.sha(runner.__file__),'input_contract':'REGULAR analytic motion; actual EuRoC extrinsics; 30 near points, staggered0.5s IDs;10 far TTL candidates per3s visible0.2s; up to3 original IDs absent from primary input for>=0.5s reappear each1s; no observations15<=t<30',
              'noise_contract':'200Hz endpoint acc.02/gyro.002; bearing.05deg; shared camera pose OU.5s std .01m/.1deg plus .002m/.02deg jitter',
              'bounded_generator':'21 poses, <=40 current points plus <=60 retired point records; logs never collected in RAM',
              'performance_contract':'construct_ms, ABI_call_with_light_JSON_ms, adapter_compute_ms, parse_ms and write_ms separate; compute excludes input context build/logger',
              'minimum_required_long_test_seconds':600,'resource_only_not_independent_accuracy_evidence':True}
    (out/'run.json').write_text(json.dumps(metadata,indent=2))
    # Running scalar aggregates only. Exact whole-run P95 is an offline log query.
    aggregate={name:{'sum':0.,'max':0.} for name in ('construct_ms','abi_ms','adapter_compute_ms','parse_ms','write_ms')}
    def imu_sample(t):
        R,p,v,a,w=generator.math.truth(t,'REGULAR')
        return np.r_[R.T@(a-generator.G)+imu_rng.normal(0,.02,3),w+imu_rng.normal(0,.002,3)]
    started=time.monotonic();imu_index=0
    try:
        with (out/'events.jsonl').open('w') as event_file,(out/'resources.jsonl').open('w') as resources,(out/'inputs.jsonl').open('w') as input_file:
            input_file.write(json.dumps({'type':'context_model','R_BC':Rbc.tolist(),'p_BC':pc.tolist(),
                                        'pose_model_std':std.tolist(),'pose_model_jitter':jitter.tolist(),
                                        'pose_correlation_seconds':.5,'bearing_sigma_rad':float(np.deg2rad(.05))})+'\n')
            for tick in range(round(seconds*20)+1):
                t=tick/20;ns=tick*50000000
                # A right bracket sample is available to interpolation but is not
                # propagated beyond the current camera's physical time.
                while imu_index<=round(t*200)+1:
                    it=imu_index/200;measurement=imu_sample(it);native.imu(it,measurement)
                    input_file.write(json.dumps({'type':'imu','time':it,'measurement':measurement.tolist()})+'\n');imu_index+=1
                construct_begin=time.perf_counter()
                R,p,*_=generator.math.truth(t,'REGULAR')
                if tick:OU=alpha*OU+np.sqrt(1-alpha*alpha)*pose_rng.normal(size=6)*std
                eps=OU+jitter_rng.normal(size=6)*jitter
                Rp=R@Rbc[0]@generator.math.rot(eps[3:])@Rbc[0].T
                pp=p+R@pc[0]+eps[:3]-Rp@pc[0]
                poses.append((t,Rp,pp))
                active=set(int(i) for i in generator.math.ids_at(tick,20,.5,30))
                for identity in previous_active-active:
                    if identity in points:retired.append((t,identity,points.pop(identity)))
                for identity in active:
                    if identity not in points:
                        slot=identity%30;depth=2.+(slot%3)
                        local=depth*np.array([.12*(slot%6-2.5),.10*(slot//6-2),1.])
                        points[identity]=p+R@(pc[0]+Rbc[0]@local)
                ttl_active=set(100000000+(tick//60)*10+j for j in range(10)) if tick%60<4 else set()
                for identity in previous_ttl-ttl_active:points.pop(identity,None)
                for identity in ttl_active:
                    if identity not in points:points[identity]=p+R@(pc[0]+Rbc[0]@np.array([.01*(identity%10),0.,1.e6]))
                proposed=[(i,points[i]) for i in sorted(active|ttl_active)]
                reappearing=[]
                if tick%20<4:
                    reappearing=[(identity,point) for left_time,identity,point in retired if t-left_time+1e-12>=.5][-3:]
                    proposed+=reappearing
                previous_active=active;previous_ttl=ttl_active
                observations=[]
                if not 15<=t<30:
                    for identity,point in proposed:
                        local=R.T@(point-p)
                        for camera,rng in ((0,left_rng),(1,right_rng)):
                            x=Rbc[camera].T@(local-pc[camera])
                            if generator.visible(x,cameras[camera]):
                                z=generator.math.tangent_noise((x/np.linalg.norm(x))[None,:],rng,np.deg2rad(.05))[0]
                                observations.append((identity,camera,ns,z))
                data={'camera_times':np.array([pose[0] for pose in poses]),'camera_ns':np.array([round(pose[0]*1e9) for pose in poses],np.int64),
                      'prior_R_WB':np.array([pose[1] for pose in poses]),'prior_p_WB':np.array([pose[2] for pose in poses]),
                      'R_BC':Rbc,'p_BC':pc,'pose_model_std':std,'pose_model_jitter':jitter,'pose_correlation_seconds':np.array(.5),
                      'observation_offsets':np.array([0]*(len(poses))+[len(observations)],np.int64),
                      'feature_ids':np.array([o[0] for o in observations],np.int64),'camera_ids':np.array([o[1] for o in observations],np.int32),
                      'actual_ns':np.array([o[2] for o in observations],np.int64),'bearings':np.array([o[3] for o in observations]).reshape(-1,3)}
                construct_ms=(time.perf_counter()-construct_begin)*1000
                # Adapter.frame includes Python covariance construction + ABI +
                # parse. Measure those separately via its optional timing sink.
                timings={};frame=native.frame(data,len(poses)-1,timings=timings)
                coverage['guard_rejected_ids']+=len(frame['identity_guard_rejected_ids'])
                coverage['active_retirement_events']+=sum(e['kind']=='ACTIVE_RETIRED' for e in frame['retirement_events'])
                coverage['candidate_ttl_events']+=sum(e['kind']=='CANDIDATE_TTL' for e in frame['retirement_events'])
                coverage['old_input_identity_reappearances']+=len({i for i,c,_,_ in observations if c==0}&{i for i,_ in reappearing})
                count=frame['resource_diagnostics'];epoch=frame['epoch'];bits=frame['identity_guard_set_bits']
                checks={'manager':count['manager_records']<=256,'history':count['history_records']<=256,
                        'samples':count['max_samples_per_track']<=21,'adapter':count['adapter_map_size']<=count['retained_ids']<=30,
                        'core_ids':count['core_admission_history_size']==0,'imu':count['imu_buffer_size']<=4096,
                        'guard_bytes':count['guard_capacity_bytes']==1048576,'generator_points':len(points)<=40,'generator_retired':len(retired)<=60,
                        'guard_monotone':last_epoch!=epoch or last_bits is None or bits>=last_bits,
                        'highwater_monotone':last_epoch!=epoch or last_highwater is None or count['core_admission_high_water']>=last_highwater}
                # Legitimate suspension may clear core highwater before a new
                # epoch commits. Explicit dormant/no-current state is separate.
                if (not frame['raw_current'] and count['core_admission_high_water']==-1
                        and frame['health_state'] in ('DORMANT','RECOVERING','COLLECTING')):
                    checks['highwater_monotone']=True
                violations+=sum(not value for value in checks.values())
                if any(not value for value in checks.values()):raise RuntimeError('Resource invariant: '+str(checks))
                last_epoch=epoch;last_bits=bits;last_highwater=count['core_admission_high_water']
                for key,value in count.items():max_counts[key]=max(max_counts.get(key,0),value)
                rss=rss_bytes();peak_rss=max(peak_rss,rss)
                serial_begin=time.perf_counter()
                event_file.write(json.dumps(frame,allow_nan=False)+'\n')
                input_file.write(json.dumps({'type':'camera','camera_ns':ns,'time':t,'observations':[[i,c,n,z.tolist()] for i,c,n,z in observations],
                                            'prior_R':Rp.tolist(),'prior_p':pp.tolist()})+'\n')
                write_ms=(time.perf_counter()-serial_begin)*1000
                measurements={'construct_ms':construct_ms+timings['construct_ms'],'abi_ms':timings['abi_ms'],
                              'adapter_compute_ms':frame['compute_time_ms'],'parse_ms':timings['parse_ms'],'write_ms':write_ms}
                for key,value in measurements.items():aggregate[key]['sum']+=value;aggregate[key]['max']=max(aggregate[key]['max'],value)
                resources.write(json.dumps({'t':t,'rss_bytes':rss,**measurements,'counts':count,'checks':checks})+'\n')
                frames+=1
                if tick%1200==0:
                    event_file.flush();resources.flush();input_file.flush()
            if seconds>=3 and any(coverage[key]<=0 for key in coverage):
                raise RuntimeError('Required retirement-reappearance/TTL coverage absent: '+str(coverage))
            metadata['status']='COMPLETED_RESOURCE_TRACE'
    except Exception as error:
        metadata['status']='FAILED';metadata['error']=repr(error);raise
    finally:
        native.close()
        metadata.update(frames=frames,elapsed_wall_seconds=time.monotonic()-started,rss_start=rss_start,rss_peak=peak_rss,
                        max_counts=max_counts,invariant_violations=violations,full_600s=frames==12001,required_input_coverage=coverage)
        metadata['timings']={key:{'mean':value['sum']/frames if frames else None,'max':value['max'],'p95':'OFFLINE_REQUIRED'} for key,value in aggregate.items()}
        for name in ('inputs.jsonl','events.jsonl','resources.jsonl'):
            if (out/name).exists():
                digest=hashlib.sha256()
                with (out/name).open('rb') as source:
                    for block in iter(lambda:source.read(1024*1024),b''):digest.update(block)
                metadata[name+'_sha']=digest.hexdigest()
        (out/'run.json').write_text(json.dumps(metadata,indent=2))
    return metadata


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--out',required=True);p.add_argument('--library',required=True)
    p.add_argument('--calibration',required=True);p.add_argument('--seconds',type=float,default=600);p.add_argument('--seed',type=int,default=42)
    p.add_argument('--mode',default='P_NEW_PRESERVE',choices=['P_NEW','P_NEW_WARMUP','P_NEW_PRESERVE','P_NEW_GRACE','P_NEW_ACTIVE'])
    p.add_argument('--confirmation-freeze-sha')
    print(json.dumps(run(**vars(p.parse_args())),indent=2))
