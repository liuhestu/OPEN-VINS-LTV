"""Frozen physical-time/body-frame Passive evaluation. GT is evaluator-only.

No fitted rotation, velocity scale, time offset, or updated derivative window.
This module does not import legacy sequence classification or access GT implicitly.
"""
import os
for key in ['OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS']:
    os.environ[key]='1'
import argparse
import csv
import hashlib
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation,Slerp
from audit.passive_outputs import inspect as inspect_passive_output

ROOT=Path(__file__).resolve().parents[2]
EUROC={'V1_01_easy','V2_02_medium','V1_03_difficult','V2_03_difficult'}
UZH={'indoor_forward_3','indoor_forward_6'}
CONFIRMATION={'V1_03_difficult','V2_03_difficult','indoor_forward_6'}
REFERENCE_GAP=.010000001
TIME_MATCH=1e-6

def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def scalar(value):return float(value) if np.isfinite(value) else None

def load_gt(sequence,path):
    if sequence not in EUROC|UZH:raise ValueError('Sequence outside frozen scope')
    x=np.loadtxt(path,delimiter=',' if sequence in EUROC else None,comments='#',ndmin=2)
    required=11 if sequence in EUROC else 8
    if len(x)<2 or x.shape[1]<required:raise ValueError('Insufficient GT columns/samples')
    if not np.isfinite(x[:,:required]).all():raise ValueError('Nonfinite GT')
    if sequence in EUROC:
        t=x[:,0]*1e-9;p=x[:,1:4];q=x[:,[5,6,7,4]];v=x[:,8:11];gravity=9.81
    else:
        t=x[:,0];p=x[:,1:4];q=x[:,4:8];v=None;gravity=9.8065
    if np.any(np.diff(t)<=0) or np.any(abs(np.linalg.norm(q,axis=1)-1)>1e-3):raise ValueError('GT ordering/quaternion invalid')
    return t,p,q,v,gravity

def local_derivative(t,p,query):
    """Exactly the pre-existing v3 cubic 0.1s derivative/gap support."""
    width=.1;result=np.full((len(query),3),np.nan)
    for k,u in enumerate(query):
        if u-width/2<t[0] or u+width/2>t[-1]:continue
        a,b=np.searchsorted(t,[u-width/2,u+width/2])
        if b-a<9 or np.max(np.diff(t[a:b]))>REFERENCE_GAP:continue
        z=(t[a:b]-u)/width;design=np.column_stack([z**i for i in range(4)])
        coef=np.linalg.lstsq(design,p[a:b]-p[a:b].mean(axis=0),rcond=None)[0]
        result[k]=coef[1]/width
    return result

def physical_reference(gt,query):
    t,p,q,v,gravity=gt;query=np.asarray(query)
    idx=np.searchsorted(t,query).clip(1,len(t)-1);lo=idx-1
    valid=(query>=t[0])&(query<=t[-1])&((t[idx]-t[lo])<=REFERENCE_GAP)
    R=np.full((len(query),3,3),np.nan)
    if valid.any():R[valid]=Slerp(t-t[0],Rotation.from_quat(q))(query[valid]-t[0]).as_matrix()
    velocity=local_derivative(t,p,query) if v is None else np.column_stack([np.interp(query,t,v[:,i]) for i in range(3)])
    velocity[~valid]=np.nan
    return {'v':np.einsum('nji,nj->ni',R,velocity),'eta':np.einsum('nji,j->ni',R,np.array([0.,0.,-gravity])),
            'pose_valid':valid,'gravity':gravity}

def read_features(path):
    rows=[json.loads(s) for s in Path(path).read_text().splitlines() if s]
    by_id={int(r['camera_ns']):r for r in rows}
    if len(rows)!=len(by_id) or any(b['camera_ns']<=a['camera_ns'] for a,b in zip(rows,rows[1:])):
        raise ValueError('Duplicate/unordered camera packet')
    return rows,by_id

def packet_weights(times):
    """Half-open outgoing intervals; final packet has zero elapsed duration.

    Intervals beyond the nominal median cadence are capped: missing gaps are not
    credited as evaluated ready time. Packet coverage still counts every packet.
    """
    if len(times)<2:return np.zeros(len(times))
    dt=np.diff(times)
    if np.any(dt<=0):raise ValueError('Physical packet times not increasing')
    return np.r_[np.minimum(dt,np.median(dt)),0.]

def longest_gap(times,ok):
    begin=None;duration=0.
    for t,valid in zip(times,ok):
        if not valid and begin is None:begin=t
        if valid and begin is not None:duration=max(duration,t-begin);begin=None
    if begin is not None:duration=max(duration,times[-1]-begin)
    return float(duration)

def vector_or_nan(row,key):
    try:
        x=np.asarray(row[key],dtype=float)
        return x if x.shape==(3,) else np.full(3,np.nan)
    except (KeyError,TypeError,ValueError):return np.full(3,np.nan)

def align_output(keys,times,lookup):
    v=np.full((len(keys),3),np.nan);eta=v.copy();ready_G=np.zeros(len(keys),bool);ready_V=ready_G.copy();valid=ready_G.copy()
    stale=0;missing=0;invalid_ready=0
    for j,(key,t) in enumerate(zip(keys,times)):
        row=lookup.get(key)
        if row is None or not row.get('initialized'):missing+=1;continue
        vector_v=vector_or_nan(row,'v_body');vector_eta=vector_or_nan(row,'eta_body')
        physical=bool(row.get('observer_started')) and bool(row.get('available'))
        same_time=abs(float(row.get('imu_time',float('nan')))-t)<=TIME_MATCH
        if physical and not same_time:stale+=1
        valid[j]=physical and same_time and np.isfinite(vector_v).all() and np.isfinite(vector_eta).all()
        if (row.get('ready_G') or row.get('ready_V')) and not valid[j]:invalid_ready+=1
        if valid[j]:v[j]=vector_v;eta[j]=vector_eta
        ready_G[j]=bool(row.get('ready_G')) and valid[j]
        ready_V[j]=bool(row.get('ready_V')) and valid[j]
    return {'v':v,'eta':eta,'finite_current':valid,'ready_G':ready_G,'ready_V':ready_V,'stale_packets':stale,'missing_packets':missing,'invalid_ready_packets':invalid_ready}

def main_reference(path,times,gravity):
    x=np.loadtxt(path,delimiter=',',skiprows=1,ndmin=2)
    result={'v':np.full((len(times),3),np.nan),'eta':np.full((len(times),3),np.nan)}
    if x.size==0:return result
    if x.shape[1]!=17 or not np.isfinite(x).all() or np.any(np.diff(x[:,0])<=0):raise ValueError('Bad native trajectory')
    hi=np.searchsorted(x[:,0],times).clip(0,len(x)-1);lo=np.maximum(0,hi-1)
    idx=np.where(abs(x[lo,0]-times)<=abs(x[hi,0]-times),lo,hi)
    ok=abs(x[idx,0]-times)<=TIME_MATCH
    # Native JPL world->body coefficient storage equals Hamilton body->world.
    R=Rotation.from_quat(x[idx[ok],1:5]).as_matrix()
    result['v'][ok]=np.einsum('nji,nj->ni',R,x[idx[ok],8:11])
    result['eta'][ok]=np.einsum('nji,j->ni',R,np.array([0.,0.,-gravity]))
    return result

def errors(output,reference):
    v=output['v']-reference['v'];eta=output['eta']-reference['eta']
    vn=np.linalg.norm(v,axis=1);en=np.linalg.norm(eta,axis=1)
    magnitude=abs(np.linalg.norm(output['eta'],axis=1)-np.linalg.norm(reference['eta'],axis=1))
    norm=np.linalg.norm(output['eta'],axis=1);refnorm=np.linalg.norm(reference['eta'],axis=1)
    angle=np.full(len(norm),np.nan)
    valid=np.isfinite(output['eta']).all(axis=1)&np.isfinite(reference['eta']).all(axis=1)
    positive=valid&(norm>1e-12)&(refnorm>1e-12)
    angle[positive]=np.degrees(np.arccos(np.clip(np.sum(output['eta'][positive]*reference['eta'][positive],axis=1)/(norm[positive]*refnorm[positive]),-1,1)))
    angle[valid&~positive]=180.
    return {'v_components':v,'eta_components':eta,'v':vn,'eta':en,'g_angle_deg':angle,'g_magnitude':magnitude}

def summarize_error(error,mask,weights):
    out={}
    for key in ['v','eta','g_angle_deg','g_magnitude']:
        valid=mask&np.isfinite(error[key]);x=error[key][valid]
        out[key+'_samples']=int(valid.sum());out[key+'_seconds']=float(weights[valid].sum())
        out[key+'_RMSE']=float(np.sqrt(np.mean(x*x))) if len(x) else None
        out[key+'_peak']=float(np.max(abs(x))) if len(x) else None
    for key in ['v_components','eta_components']:
        valid=mask&np.isfinite(error[key]).all(axis=1)
        out[key+'_RMSE']=np.sqrt(np.mean(error[key][valid]**2,axis=0)).tolist() if valid.any() else None
    v_ok=mask&np.isfinite(error['v']);g_ok=mask&np.isfinite(error['g_angle_deg'])&np.isfinite(error['g_magnitude'])
    out['wide_v_accuracy']=float(np.mean(error['v'][v_ok]<=.1)) if v_ok.any() else None
    out['wide_g_accuracy']=float(np.mean((error['g_angle_deg'][g_ok]<=1)&(error['g_magnitude'][g_ok]<=.2))) if g_ok.any() else None
    return out

def no_degradation(old,new,quantity):
    key=quantity+'_RMSE';a=old.get(key);b=new.get(key)
    if a is None or b is None:return False
    return b-a<=max(.05*a,.01 if quantity=='v' else .05)

def has_improvement(old,new,quantity):
    a=old.get(quantity+'_RMSE');b=new.get(quantity+'_RMSE')
    return a is not None and b is not None and a>0 and b<=.8*a

def engineering(paths):
    audits={name:list(csv.DictReader((path/'audit.csv').open())) for name,path in paths.items()}
    reference=audits['B'];fields=['camera_ns','initialized','state_time','clones','state_digest','tracker_digest','visual_digest']
    equal={name:len(rows)==len(reference) and all(all(a.get(k)==b.get(k) for k in fields) for a,b in zip(reference,rows)) for name,rows in audits.items()}
    trajectory_equal={name:sha(path/'trajectory.csv')==sha(paths['B']/'trajectory.csv') for name,path in paths.items()}
    injected={name:any(int(float(row.get(k,0)))!=0 for row in rows for k in ['auxiliary_receipts','gravity_submissions','velocity_submissions']) for name,rows in audits.items()}
    complete={};metadata={};record_integrity={};independent_audit={}
    for name,path in paths.items():
        try:independent_audit[name]=inspect_passive_output(path)
        except (ValueError,KeyError,OSError) as error:independent_audit[name]={'status':'FAIL','reason':str(error)}
    for name,path in paths.items():
        f,_=read_features(path/'features.jsonl');rows=audits[name]
        required=fields+['auxiliary_receipts','gravity_submissions','velocity_submissions']
        record_integrity[name]=len(f)==len(rows) and all(all(k in row for k in required) and int(row['camera_ns'])==int(feature['camera_ns']) and bool(int(row['initialized']))==bool(feature['initialized']) for row,feature in zip(rows,f))
    for name,path in paths.items():
        r=json.loads((path/'replay.json').read_text());metadata[name]=r
        complete[name]=all(k in r for k in ['actual_G_submissions','actual_V_submissions']) and bool(r.get('complete')) and r['camera_packets']==r['input_camera_packets'] and r['imu_consumed']==r['input_imu_samples']
        injected[name]=injected[name] or r.get('actual_G_submissions',0)!=0 or r.get('actual_V_submissions',0)!=0
    return {'audit_fields':fields,'main_audit_equal':equal,'trajectory_exact_equal':trajectory_equal,'any_actual_injection':injected,
            'independent_output_audit':independent_audit,'record_integrity':record_integrity,'complete_input':complete,'replay':metadata,'pass':all(r['status']=='PASS' for r in independent_audit.values()) and all(record_integrity.values()) and all(equal.values()) and all(trajectory_equal.values()) and not any(injected.values()) and all(complete.values())}

def raw_candidate_population(rows,seed_source):
    if seed_source not in ('STEREO','TEMPORAL_POSE','STEREO_THEN_TEMPORAL'):
        raise ValueError('Explicit known seed source required for raw opportunity denominator')
    birth={};opportunities=set();segments={};index=0
    for row in rows:
        if not row.get('initialized'):continue
        epoch=int(row.get('epoch',0));ns=int(row['camera_ns'])
        # Current IDs are the bridge's legal frontend observation proxy, not GT.
        ids=[int(i) for i in row['current_cam0_ids']]
        stereo=set(int(i) for i in row['current_stereo_ids'])
        if len(ids)!=len(set(ids)) or not stereo.issubset(set(ids)):
            raise ValueError('Invalid raw frontend identity record')
        for id in ids:
            key=(epoch,id)
            birth.setdefault(key,{'first_ns':ns,'first_time':float(row['target_imu_time'])})
            previous=segments.get(key)
            if previous is None or previous['last_index']!=index-1:
                previous={'start_ns':ns,'frames':0,'last_index':index}
            previous['frames']+=1;previous['last_index']=index;segments[key]=previous
            history=previous['frames']>=3 and ns-previous['start_ns']>=100000000
            source_available=seed_source!='STEREO' or id in stereo
            if history and source_available:opportunities.add(key)
        index+=1
    return birth,opportunities

def manager_metrics(rows,seed_source):
    birth,opportunities=raw_candidate_population(rows,seed_source)
    source_counts={};admitted=set();written=set();manager_opportunities=set();retired=set();tracks={};risk=[];waiting=[];reasons={}
    epochs=[]
    for row in rows:
        if not row.get('initialized'):continue
        epoch=int(row.get('epoch',0));epochs.append(epoch)
        for track in row.get('tracks',[]):
            key=(epoch,int(track['id']));tracks[key]=track
            if track.get('ever_opportunity'):manager_opportunities.add(key)
            if track.get('phase') in ('RETIRED','retired'):retired.add(key)
            reason=track.get('reason','unknown');reasons[reason]=reasons.get(reason,0)+1
        for id in row.get('retired_ids',[]):retired.add((epoch,int(id)))
        for seed in row.get('seeds',[]):
            allowed_sources={'STEREO','TEMPORAL_POSE'} if seed_source=='STEREO_THEN_TEMPORAL' else {seed_source}
            if seed['source'] not in allowed_sources:raise ValueError('Seed source differs from explicit run identity')
            if not seed.get('admitted'):continue
            key=(epoch,int(seed['id']))
            if key in admitted:raise ValueError('Duplicate same-epoch admission in actual log')
            if key not in birth or key not in opportunities:raise ValueError('Admitted seed without raw legal initialization opportunity')
            admitted.add(key);actual_source=seed['source'];source_counts[actual_source]=source_counts.get(actual_source,0)+1
            if seed.get('mean_written'):written.add(key)
            if seed.get('relative_risk') is not None:risk.append(seed['relative_risk'])
            waiting.append(float(seed['time'])-birth[key]['first_time'])
    startup={k for k in birth if k[0]==0};active_epochs=set(birth)-startup
    startup_opportunities=opportunities&startup;active_opportunities=opportunities&active_epochs
    return {'candidate_tracks':len(birth),'opportunity_tracks':len(opportunities),'manager_opportunity_tracks':len(manager_opportunities),
            'raw_opportunities_not_in_manager':len(opportunities-manager_opportunities),
            'admitted_tracks':len(admitted),'seed_writes':len(written),'seed_source_contract':seed_source,
            'startup_epoch0':{'birth_tracks':len(startup),'opportunity_tracks':len(startup_opportunities)},
            'observer_positive_epochs':{'birth_tracks':len(active_epochs),'opportunity_tracks':len(active_opportunities)},
            'raw_frontend_unique_ids_ignoring_observer_epochs':len({k[1] for k in birth}),
            'startup_ids_also_seen_in_observer_epochs':len({k[1] for k in startup}&{k[1] for k in active_epochs}),
            'denominator_contract':'all initialized P_NEW current cam0 tracks keyed (epoch,id), >=3 consecutive camera packets and >=100000000ns span; current stereo additionally required for STEREO; before solver/risk/manager capacity',
            'epoch_contract':'epoch0 startup kept separately AND included in all-initialized denominator; not treated as actual LTV reset; B default epoch0 never used as NEW identity',
            'legality_contract':'bridge current frontend IDs are observed-match proxy, not real physical-ID ground truth',
            'retired_tracks':len(retired),'source_admissions':source_counts,'reliable_seed_fraction':None,'landmark_GT_available':False,
            'relative_risk_median':float(np.median(risk)) if risk else None,'waiting_median_s':float(np.median(waiting)) if waiting else None,
            'admission_fraction_opportunity':len(admitted)/len(opportunities) if opportunities else None,
            'admission_fraction_all_births':len(admitted)/len(birth) if birth else None,
            'mean_active':float(np.mean([r['state_features'] for r in rows if r.get('initialized')])),
            'mean_mature':float(np.mean([r.get('mature_features',0) for r in rows if r.get('initialized')])),
            'epoch_transitions':sum(a!=b and a>0 and b>0 for a,b in zip(epochs,epochs[1:])),'reason_frames':reasons}

def evaluate(sequence,gt_path,baseline,old,new,out):
    paths={'B':Path(baseline),'P_OLD':Path(old),'P_NEW':Path(new)};out=Path(out);out.mkdir(parents=True,exist_ok=False)
    row_data={name:read_features(path/'features.jsonl') for name,path in paths.items()}
    new_identity=json.loads((paths['P_NEW']/'identity.json').read_text())
    if new_identity.get('mode')!='P_NEW' or new_identity.get('source') not in ('STEREO','TEMPORAL_POSE','STEREO_THEN_TEMPORAL') or not new_identity.get('config_sha'):
        raise ValueError('Missing explicit P_NEW source/configuration identity')
    seed_source=new_identity['source']
    base=[r for r in row_data['B'][0] if r.get('initialized')]
    if not base:raise ValueError('No baseline initialized packets')
    keys=[int(r['camera_ns']) for r in base];times=np.array([r['target_imu_time'] for r in base]);weights=packet_weights(times)
    gt=load_gt(sequence,gt_path);ref=physical_reference(gt,times)
    outputs={name:align_output(keys,times,row_data[name][1]) for name in ['P_OLD','P_NEW']}
    outputs['OpenVINS']=main_reference(paths['B']/'trajectory.csv',times,ref['gravity'])
    error={name:errors(value,ref) for name,value in outputs.items()}
    common=outputs['P_OLD']['finite_current']&outputs['P_NEW']['finite_current']
    masks={'raw_common':common,'NEW_ready_G':common&outputs['P_NEW']['ready_G'],'NEW_ready_V':common&outputs['P_NEW']['ready_V'],
           'NEW_ready_joint':common&outputs['P_NEW']['ready_G']&outputs['P_NEW']['ready_V']}
    comparison={support:{name:summarize_error(err,mask,weights) for name,err in error.items()} for support,mask in masks.items()}
    coverage={}
    for name in ['P_OLD','P_NEW']:
        o=outputs[name];coverage[name]={'initialized_camera_packets':len(keys),'finite_current_packets':int(o['finite_current'].sum()),
            'finite_current_fraction':float(o['finite_current'].mean()),'stale_packets':o['stale_packets'],'missing_packets':o['missing_packets'],'invalid_ready_packets':o['invalid_ready_packets']}
        for branch,quantity in [('G','eta'),('V','v')]:
            ready=o['ready_'+branch];supported=ready&np.isfinite(error[name][quantity])
            coverage[name].update({f'ready_{branch}_fraction':float(ready.mean()),f'ready_{branch}_packets':int(ready.sum()),
                f'ready_{branch}_reference_seconds':float(weights[supported].sum()),f'ready_{branch}_reference_packets':int(supported.sum()),
                f'ready_{branch}_longest_gap_s':longest_gap(times,ready)})
        coverage[name]['joint_ready_fraction']=float((o['ready_G']&o['ready_V']).mean())
    for branch in ['G','V']:
        coverage['P_NEW']['ready_'+branch+'_without_OLD_current_packets']=int((outputs['P_NEW']['ready_'+branch]&~outputs['P_OLD']['finite_current']).sum())
    nc=coverage['P_NEW'];coverage_ok=all(nc[f'ready_{b}_fraction']>=.2 and nc[f'ready_{b}_reference_seconds']>=10 for b in ['G','V'])
    comparisons={}
    for support in masks:
        before=comparison[support]['P_OLD'];after=comparison[support]['P_NEW']
        comparisons[support]={'v_improvement_20pct':has_improvement(before,after,'v'),'eta_improvement_20pct':has_improvement(before,after,'eta'),
             'v_no_significant_degradation':no_degradation(before,after,'v'),'eta_no_significant_degradation':no_degradation(before,after,'eta')}
    vg=comparisons['NEW_ready_V'];gg=comparisons['NEW_ready_G'];raw=comparisons['raw_common']
    benefit=(vg['v_improvement_20pct'] and vg['eta_no_significant_degradation'] and comparison['NEW_ready_V']['P_NEW']['v_seconds']>=10) or (gg['eta_improvement_20pct'] and gg['v_no_significant_degradation'] and comparison['NEW_ready_G']['P_NEW']['eta_seconds']>=10)
    raw_ok=raw['v_no_significant_degradation'] and raw['eta_no_significant_degradation'];eng=engineering(paths)
    eng['ltv_time_integrity']={name:outputs[name]['stale_packets']==0 and outputs[name]['invalid_ready_packets']==0 for name in ['P_OLD','P_NEW']}
    eng['pass']=eng['pass'] and all(eng['ltv_time_integrity'].values())
    manager=manager_metrics(row_data['P_NEW'][0],seed_source)
    dt=np.diff(times);nominal=float(np.median(dt)) if len(dt) else 0.
    camera_gaps={'nominal_cadence_s':nominal,'longest_packet_interval_s':float(dt.max()) if len(dt) else 0.,
        'gap_intervals_over_1p5_nominal':int(np.sum(dt>1.5*nominal)),
        'uncredited_elapsed_seconds_due_to_interval_capping':float(times[-1]-times[0]-weights.sum()),
        'note':'Unobserved camera intervals flagged separately; no invented ready or error samples inside gaps.'}
    limitations=['No real feature landmark ground truth; no seed correctness percentage claimed.',
        'P_NEW pose/stereo source and missing state-bearing cross correlation remain stated approximations.',
        'No rotation/scale/time fitting; physical IMU timestamps and actual body frames only.']
    if sequence=='V1_01_easy':limitations.append('V1_01 original attitude-reference limitation retained; gravity/velocity-body evidence is conditional on that reference.')
    if sequence in UZH:limitations.append('UZH v3 finite GT coverage and fixed0.1s cubic position derivative; unsupported windows remain missing.')
    result={'sequence':sequence,'status':('BLOCKED_CORRECTNESS' if not eng['pass'] else ('MEETS_SEQUENCE_OUTPUT_TARGET' if coverage_ok and benefit and raw_ok else 'NOT_MET')),
        'coverage_status':'SUFFICIENT_COVERAGE' if coverage_ok else 'INSUFFICIENT_COVERAGE','coverage':coverage,
        'comparison':comparison,'decisions':comparisons,'qualifying_improvement':bool(benefit),'raw_non_degradation':bool(raw_ok),
        'engineering':eng,'feature_management':manager,'limitations':limitations,
        'camera_gap_diagnostics':camera_gaps,'seed_source_identity':new_identity,
        'reference':{'path':str(gt_path),'sha':sha(gt_path),'gravity':ref['gravity'],'velocity':'official velocity columns' if sequence in EUROC else 'v3 cubic position derivative width0.1s',
            'pose_reference_fraction':float(np.isfinite(ref['eta']).all(axis=1).mean()),'velocity_reference_fraction':float(np.isfinite(ref['v']).all(axis=1).mean()),
            'max_reference_gap_s':REFERENCE_GAP,'output_time_tolerance_s':TIME_MATCH,'packet_duration':'min(outgoing_dt,median_dt); last packet0'},
        'provenance':{'evaluator_sha':sha(__file__),'run_files':{name:{f:sha(path/f) for f in ['features.jsonl','trajectory.csv','audit.csv','replay.json','identity.json','effective_options.json']} for name,path in paths.items()}}}
    result['tables']={'seed_reliability_available_evidence':{**manager,'reliability_claim':'UNAVAILABLE_REAL_LANDMARK_GT'},
        'feature_management':{'coverage':coverage,'management':manager},'real_outputs':comparison,'passive_engineering':eng}
    (out/'metrics.json').write_text(json.dumps(result,indent=2,allow_nan=False))
    arrays={'time':times,'camera_ns':np.array(keys,dtype=np.int64),'packet_weight_s':weights,'reference_v_body':ref['v'],'reference_eta_body':ref['eta']}
    arrays.update({f'support_{key}':value for key,value in masks.items()})
    for name,values in outputs.items():
        arrays[name+'_v_body']=values['v'];arrays[name+'_eta_body']=values['eta']
        for key,value in error[name].items():arrays[name+'_'+key]=value
    np.savez_compressed(out/'error_curves.npz',**arrays)
    return result

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--sequence',choices=sorted(EUROC|UZH),required=True);p.add_argument('--gt',type=Path,required=True)
    for name in ['baseline','old','new','out']:p.add_argument('--'+name,type=Path,required=True)
    p.add_argument('--confirmation-freeze-sha');a=p.parse_args()
    if a.sequence in CONFIRMATION:
        frozen=ROOT/'docs/ltv/feature_readiness_passive/frozen_config.json'
        if not frozen.exists() or not a.confirmation_freeze_sha or sha(frozen)!=a.confirmation_freeze_sha:
            p.error('Confirmation GT evaluation requires exact main frozen configuration SHA')
    result=evaluate(a.sequence,a.gt,a.baseline,a.old,a.new,a.out)
    print(json.dumps({'sequence':result['sequence'],'status':result['status'],'coverage_status':result['coverage_status'],
        'qualifying_improvement':result['qualifying_improvement'],'raw_non_degradation':result['raw_non_degradation']},indent=2))
