"""Offline hardening real evaluation; immutable physical reference, no GT online."""
import os
for k in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS'):os.environ[k]='1'
import argparse,copy,csv,hashlib,importlib.util,json,sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[4];DOC=ROOT/'docs/ltv/passive_hardening_v2'
sys.path.insert(0,str(ROOT/'docs/experiments/tools/ltv_feature_passive'))
spec=importlib.util.spec_from_file_location('_frozen_physical_evaluation',ROOT/'docs/experiments/tools/ltv_feature_passive/real_evaluate.py')
physical=importlib.util.module_from_spec(spec);spec.loader.exec_module(physical)
PROTOCOL=json.loads((DOC/'protocol.json').read_text());CONTRACT=json.loads((DOC/'acceptance.json').read_text())
# Extend the allowed sequence names, not the official physical reference formulas.
physical.EUROC=set(PROTOCOL['real_final'])

def load(p):return json.loads(Path(p).read_text())
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def identity_path(p):return p/'identity.json' if (p/'identity.json').exists() else p.parent/'identity.json'
def episodes(t,mask,epoch=None):
    t=np.asarray(t);mask=np.asarray(mask,bool);epoch=np.zeros(len(t)) if epoch is None else np.asarray(epoch)
    if len(t)!=len(mask) or len(epoch)!=len(t) or (len(t)>1 and np.any(np.diff(t)<=0)):raise ValueError('Invalid episode timeline')
    cadence=float(np.median(np.diff(t))) if len(t)>1 else 0.;result=[];start=None
    for k in range(len(t)):
        broken=k>0 and (t[k]-t[k-1]>1.5*cadence or epoch[k]!=epoch[k-1])
        if start is not None and (not mask[k] or broken):
            end=k-1;result.append(dict(start=float(t[start]),end=float(t[end]),duration=float(t[end]-t[start]),samples=end-start+1));start=None
        if mask[k] and start is None:start=k
    if start is not None:result.append(dict(start=float(t[start]),end=float(t[-1]),duration=float(t[-1]-t[start]),samples=len(t)-start))
    return result

def summarize(e,mask,weights):
    out=physical.summarize_error(e,mask,weights)
    for key in ('v','eta','g_angle_deg','g_magnitude'):
        x=e[key][mask&np.isfinite(e[key])];out[key+'_P95']=float(np.quantile(x,.95,interpolation='linear')) if len(x) else None
    for branch,key,cutoff in [('V','v',1.),('G','g_angle_deg',5.)]:
        valid=mask&np.isfinite(e[key]);out['severe_'+branch+'_fraction']=float(np.mean(e[key][valid]>cutoff)) if valid.any() else None
    return out

def align(rows,keys,times,hardened):
    lookup={int(r['camera_ns']):r for r in rows};n=len(keys)
    result={k:np.full((n,3),np.nan) for k in ('v','eta')}
    result.update({k:np.zeros(n,bool) for k in ('finite_current','ready_G','ready_V')})
    reasons=[];epochs=[]
    for j,(key,t) in enumerate(zip(keys,times)):
        if key not in lookup:raise ValueError('Missing initialized camera receipt')
        r=lookup[key];epochs.append(r['epoch'])
        current=bool(r['raw_current']) if hardened else bool(r['available'] and r['observer_started'])
        if current:
            if abs(r['imu_time']-t)>physical.TIME_MATCH:raise ValueError('Stale raw claimed current')
            for key2,field in [('v','v_body'),('eta','eta_body')]:
                value=np.asarray(r[field],float)
                if value.shape!=(3,) or not np.isfinite(value).all():raise ValueError('Invalid raw vector')
                result[key2][j]=value
        elif hardened and (r['v_body'] is not None or r['eta_body'] is not None):raise ValueError('Missing raw not explicit null')
        result['finite_current'][j]=current
        for branch in ('G','V'):
            declared=bool(r['ready_'+branch])
            if declared and not current:raise ValueError('Ready declaration without current raw')
            result['ready_'+branch][j]=declared
        if hardened and result['ready_G'][j]:
            norm=np.linalg.norm(result['eta'][j]);lo,hi=PROTOCOL['G_health_norm_range']
            if not lo<=norm<=hi:raise ValueError('Ready gravity physically invalid')
        reasons.append(r.get('availability_state',r.get('reason','UNDECLARED')))
    result['reasons']=reasons;result['epoch']=np.asarray(epochs);return result

def csv_semantics(path,camera):
    rows=[]
    with Path(path).open() as f:
        for r in csv.reader(f):
            if not r or r[0].startswith('#'):continue
            ns=int(r[0]);rows.append((ns,r[1].strip()) if camera else (ns,*map(float,r[1:7])))
    return rows

def sensor_equivalence(old_identity,new_identity):
    proof={};old=old_identity['sensor_csv_sha'];new=new_identity['inputs']['sensors']
    for sensor in ('cam0','cam1','imu0'):
        candidates=[Path(p) for p in old if Path(p).parent.name==sensor]
        if len(candidates)!=1:raise ValueError('Ambiguous historical sensor identity')
        left=candidates[0];right=Path(new[sensor]['path'])
        if sha(left)!=old[str(left)] or sha(right)!=new[sensor]['sha256']:raise ValueError('Changed sensor CSV identity')
        a=csv_semantics(left,sensor!='imu0');b=csv_semantics(right,sensor!='imu0')
        if a!=b:raise ValueError('Different parsed sensor inputs')
        if sensor!='imu0':
            # Historical normalized captures use original-image symlinks.
            if any((left.parent/'data'/filename).resolve()!=(right.parent/'data'/filename).resolve() for _,filename in a):
                raise ValueError('Camera images not identical resolved files')
        proof[sensor]={'historical_sha':sha(left),'current_sha':sha(right),'parsed_exact':True,'rows':len(a),'same_image_files':sensor!='imu0'}
    return proof

# Exact allowlist: only newly authorized hardening engineering controls may differ.
HARDENING_KEYS = {'ltv_passive_hardening_enabled', 'ltv_hardening_health_readiness',
                 'ltv_hardening_initial_warmup', 'ltv_hardening_preserve_constrained_state', 'ltv_hardening_ready_soft_grace', 'ltv_hardening_prediction_angle_limit_rad',
                 'ltv_hardening_velocity_correction_rate_limit', 'ltv_hardening_gravity_correction_rate_limit'}

def yaml_values(path):
    import yaml
    text='\n'.join(line for line in Path(path).read_text().splitlines() if not line.startswith('%YAML:'))
    value=yaml.safe_load(text)
    if not isinstance(value,dict):raise ValueError('Configuration must be a YAML mapping')
    return value

def config_equivalence(old_path,new_path):
    old=yaml_values(old_path);new=yaml_values(new_path)
    if any(k in old and old[k] not in (False,None) for k in HARDENING_KEYS):
        raise ValueError('Historical method unexpectedly enabled hardening')
    old_stable={k:v for k,v in old.items() if k not in HARDENING_KEYS}
    new_stable={k:v for k,v in new.items() if k not in HARDENING_KEYS}
    if old_stable!=new_stable:
        changed=sorted(k for k in set(old_stable)|set(new_stable) if old_stable.get(k)!=new_stable.get(k))
        raise ValueError('Non-hardening configuration changed: '+','.join(changed))
    return {'old_path':str(old_path),'new_path':str(new_path),'old_sha':sha(old_path),'new_sha':sha(new_path),
            'nonhardening_yaml_exact':True,'hardening_controls':{k:{'previous':old.get(k,False),'new':new.get(k,False)} for k in sorted(HARDENING_KEYS) if k in old or k in new}}

def resolve_config_identity(prev,pi,new,ni):
    root=prev.parent.parent/'mode_configs'
    matches=[p for p in root.glob('**/estimator_config.yaml') if sha(p)==pi['config_sha']]
    if len(matches)!=1:raise ValueError('No unique SHA-bound historical configuration')
    oldpath=matches[0];newpath=new.parent/'config/estimator_config.yaml'
    for name,digest in pi['config_tree_sha'].items():
        if sha(oldpath.parent/name)!=digest:raise ValueError('Historical config tree changed')
    for name,digest in ni['config_sha'].items():
        if sha(newpath.parent/name)!=digest:raise ValueError('New config tree changed')
    return config_equivalence(oldpath,newpath)

def resolve_historical(sequence,new,manifest):
    manifest=Path(manifest);m=load(manifest);metric_path=Path(m['real_metrics'][sequence]);metric=load(metric_path)
    recovery=metric.get('diagnostic_recovery')
    if not recovery:raise ValueError('Historical selected method origin not recorded')
    prev=Path(recovery['original_run']);pi=load(identity_path(prev));ni=load(identity_path(new))
    if ni['sequence']!=sequence or pi['sequence']!=sequence or pi['mode']!='P_NEW':raise ValueError('Method/sequence mismatch')
    sensors=sensor_equivalence(pi,ni)
    config=resolve_config_identity(prev,pi,new,ni)
    candidates=[]
    for p in prev.parent.glob(sequence+'_B_*/identity.json'):
        bi=load(p)
        if bi.get('runtime')==pi['runtime'] and bi.get('sensor_csv_sha')==pi['sensor_csv_sha'] and bi.get('phase')==pi.get('phase'):candidates.append(p.parent)
    if len(candidates)!=1:raise ValueError('No unique same-runtime historical baseline')
    base=candidates[0]
    for sensor in ('kalibr_imu_chain.yaml','kalibr_imucam_chain.yaml'):
        if pi['config_tree_sha'][sensor]!=ni['config_sha'][sensor]:raise ValueError('Changed calibration input')
    return base,prev,dict(manifest=str(manifest),manifest_sha=sha(manifest),historical_metrics=str(metric_path),historical_metrics_sha=sha(metric_path),parsed_sensor_equivalence=sensors,configuration_equivalence=config,
                         historical_overlay_used_for_numeric=False,reason='Only immutable original P_NEW raw fields reused as P_PREV; historical overlay used solely to locate its original run.')

def sensor_identity(value):
    if 'sensor_csv_sha' in value:return value['sensor_csv_sha']
    return {v['path']:v['sha256'] for v in value['inputs']['sensors'].values()}

def config_identity(value):
    if 'config_tree_sha' in value:return value['config_tree_sha']
    if isinstance(value.get('config_sha'),dict):return value['config_sha']
    raise ValueError('Missing full configuration tree identity')

def current_config(run,identity):
    path=Path(identity['config_path']) if 'config_path' in identity else run.parent/'config/estimator_config.yaml'
    for name,digest in config_identity(identity).items():
        if sha(path.parent/name)!=digest:raise ValueError('Explicit run configuration tree changed')
    return path

def normalized_input_identity(identity):
    if 'inputs' in identity:return identity
    sensors={}
    for path,digest in sensor_identity(identity).items():
        key=Path(path).parent.name
        if key in sensors:raise ValueError('Ambiguous sensor input identity')
        sensors[key]={'path':path,'sha256':digest}
    return {'inputs':{'sensors':sensors}}

def resolve_explicit(sequence,new,baseline,previous):
    base,prev=Path(baseline),Path(previous)
    identities={k:load(identity_path(p)) for k,p in [('B',base),('P_PREV',prev),('P_NEW',new)]}
    for mode,value in identities.items():
        if value.get('sequence')!=sequence or value.get('mode')!=mode:
            raise ValueError('Explicit identities require exact sequence/mode: '+mode)
    ni=identities['P_NEW'];sensor_proofs={}
    for mode in ('B','P_PREV'):
        sensor_proofs[mode]=sensor_equivalence({'sensor_csv_sha':sensor_identity(identities[mode])},normalized_input_identity(ni))
    configs={mode:current_config(path,identities[mode]) for mode,path in [('B',base),('P_PREV',prev),('P_NEW',new)]}
    config=config_equivalence(configs['P_PREV'],configs['P_NEW'])
    # Baseline may disable all LTV, but all original main-estimator parameters must match.
    b=yaml_values(configs['B']);n=yaml_values(configs['P_NEW'])
    if {k:v for k,v in b.items() if not k.startswith('ltv_')}!={k:v for k,v in n.items() if not k.startswith('ltv_')}:
        raise ValueError('Baseline native configuration differs')
    for mode in ('B','P_PREV'):
        for name,digest in config_identity(ni).items():
            if name!='estimator_config.yaml' and config_identity(identities[mode]).get(name)!=digest:
                raise ValueError('Explicit calibration/include identity differs')
    return base,prev,{'explicit_runs':True,'identities':identities,'identity_sha':{k:sha(identity_path(p)) for k,p in [('B',base),('P_PREV',prev),('P_NEW',new)]},
                      'parsed_sensor_equivalence':sensor_proofs,'configuration_equivalence':config,'baseline_native_yaml_exact':True,'historical_overlay_used_for_numeric':False}

def assess_all11(records):
    """Record aggregation only; no selection by NEW error or hidden exclusions."""
    expected=set(PROTOCOL['real_final'])
    if len(records)!=len(expected) or {r['sequence'] for r in records}!=expected:
        raise ValueError('Exactly all eleven unique EuRoC sequence records are required')
    for row in records:
        if not row.get('P_NEW_native_complete') or not row.get('engineering_pass'):
            return {'status':'BLOCKED_CORRECTNESS','reason':'incomplete native P_NEW or nonintrusion evidence','sequence':row['sequence']}
        if not row.get('baseline_valid') and row.get('baseline_invalid_reason') not in ('native_initialization_failure','native_estimator_failure','native_input_failure'):
            raise ValueError('Baseline invalidity lacks an allowed B-only cause')
    valid=[r for r in records if r['baseline_valid']]
    if len(valid)<CONTRACT['minimum_B_valid_sequences']:
        return {'status':'BLOCKED_INPUT','baseline_valid':len(valid),'total':11,'reason':'fewer_than_eight_baseline_valid'}
    unavailable=[r['sequence'] for r in valid if not r.get('reference_valid')]
    failed=[r['sequence'] for r in valid if r.get('status')!='MEETS_SEQUENCE_CONTRACT']
    return {'status':'OUTPUT_CONTRACT_MET' if not failed and not unavailable else 'NOT_MET','baseline_valid':len(valid),'total':11,
            'failed_sequences':failed,'reference_unestablished':unavailable,
            'scope':'Outputs only; seed/recovery/resource/performance/freeze/repeat contracts remain independent.'}

def threshold(value,maximum):return value is not None and value<=maximum

def evaluate(sequence,new,out,manifest=None,baseline=None,previous=None):
    new=Path(new);out=Path(out)
    if out.exists():raise FileExistsError(out)
    if bool(baseline)!=bool(previous):raise ValueError('Explicit baseline and previous must be supplied together')
    base,prev,reuse=resolve_explicit(sequence,new,baseline,previous) if baseline else resolve_historical(sequence,new,manifest)
    paths={'B':base,'P_PREV':prev,'P_NEW':new}
    engineering=physical.engineering(paths)
    if not engineering['pass']:raise ValueError('Main state/time/full input parity failed')
    rows={k:physical.read_features(p/'features.jsonl')[0] for k,p in paths.items()}
    timeline=[r for r in rows['B'] if r['initialized']]
    if not timeline:raise ValueError('BASELINE_INVALID: no initialized camera packets')
    keys=[r['camera_ns'] for r in timeline];times=np.asarray([r['target_imu_time'] for r in timeline]);weights=physical.packet_weights(times)
    outputs={name:align(rows[name],keys,times,name=='P_NEW') for name in ('P_PREV','P_NEW')}
    ni=load(identity_path(new));gtpath=Path(ni['inputs']['reference_metadata_only']['path'])
    if sha(gtpath)!=ni['inputs']['reference_metadata_only']['sha256']:raise ValueError('GT metadata identity changed')
    ref=physical.physical_reference(physical.load_gt(sequence,gtpath),times)
    outputs['OpenVINS']=physical.main_reference(base/'trajectory.csv',times,ref['gravity'])
    error={k:physical.errors(o,ref) for k,o in outputs.items()}
    own=outputs['P_NEW'];common=own['finite_current']&outputs['P_PREV']['finite_current']
    supports={'raw':common,'ready_G':common&own['ready_G'],'ready_V':common&own['ready_V'],'joint_ready':common&own['ready_G']&own['ready_V']}
    comparisons={}
    for s,mask in supports.items():
        # Exact shared physical support per quantity, including native output.
        comparisons[s]={}
        for q in ('v','eta','g_angle_deg','g_magnitude'):
            shared=mask.copy()
            for e in error.values():shared&=np.isfinite(e[q])
            comparisons[s][q]={'samples':int(shared.sum()),'seconds':float(weights[shared].sum()),'mask_sha':hashlib.sha256(shared.tobytes()).hexdigest(),
                               'methods':{k:summarize(e,shared,weights) for k,e in error.items()}}
    coverage={};own_summary={}
    for name in ('P_PREV','P_NEW'):
        o=outputs[name];coverage[name]={'initialized_packets':len(times),'raw_current_packets':int(o['finite_current'].sum()),'raw_missing_packets':int((~o['finite_current']).sum()),
                                     'missing_by_state':{r:sum(not valid and reason==r for valid,reason in zip(o['finite_current'],o['reasons'])) for r in sorted(set(o['reasons']))}}
        own_summary[name]={}
        for support,mask in {'raw':o['finite_current'],'ready_G':o['ready_G'],'ready_V':o['ready_V'],'joint_ready':o['ready_G']&o['ready_V']}.items():
            eps=episodes(times,mask,o['epoch']);gaps=episodes(times,~mask)
            quantities=['g_angle_deg'] if support=='ready_G' else ['v'] if support=='ready_V' else ['v','g_angle_deg']
            valid=mask.copy()
            for q in quantities:valid&=np.isfinite(error[name][q])
            coverage[name][support]={'packets':int(mask.sum()),'fraction':float(mask.mean()),'reference_seconds':float(weights[valid].sum()),'episodes':eps,
                                    'longest_episode':max((x['duration'] for x in eps),default=0.),'longest_gap':max((x['duration'] for x in gaps),default=0.)}
            own_summary[name][support]=summarize(error[name],mask,weights)
    target=CONTRACT['each_valid_sequence'];ng=own_summary['P_NEW']['ready_G'];nv=own_summary['P_NEW']['ready_V'];exception=sequence in CONTRACT['gravity_accuracy_exception']
    decisions={'coverage':all(coverage['P_NEW']['ready_'+b]['fraction']>=target[b+'_ready_fraction_min'] and coverage['P_NEW']['ready_'+b]['reference_seconds']>=target[b+'_ready_reference_seconds_min'] for b in ('G','V')),
               'V_absolute':threshold(nv['v_RMSE'],.20) and threshold(nv['v_P95'],.50),
               'G_absolute':exception or (threshold(ng['g_angle_deg_RMSE'],1.) and threshold(ng['g_angle_deg_P95'],2.)),
               'joint_episode':coverage['P_NEW']['joint_ready']['longest_episode']>=5.,
               'severe_V':threshold(nv['severe_V_fraction'],.01),'severe_G':exception or threshold(ng['severe_G_fraction'],.01)}
    regression={}
    for q,margin in [('v',.01),('eta',.05),('g_angle_deg',.1)]:
        v=comparisons['raw'][q]['methods'];a=v['P_PREV'][q+'_RMSE'];b=v['P_NEW'][q+'_RMSE']
        regression[q]={'before':a,'after':b,'allowed_increase':max(.05*a,margin) if a is not None else None,'pass':a is not None and b is not None and b-a<=max(.05*a,margin)}
    decisions['raw_regression']=all(v['pass'] for k,v in regression.items() if not(exception and k=='g_angle_deg'))
    lost=outputs['P_PREV']['finite_current']&~own['finite_current'];unexplained=lost&np.array([s not in ('COLLECTING','BOOTSTRAPPING','DORMANT','RECOVERING','DEGRADED') for s in own['reasons']])
    decisions['raw_missing_explained']=not bool(unexplained.any())
    first_current=int(np.flatnonzero(own['finite_current'])[0]) if own['finite_current'].any() else 0
    startup=np.arange(len(times))<first_current
    review_required=lost&~(startup&np.array([s=='COLLECTING' for s in own['reasons']]))
    decisions['raw_support_no_unreviewed_loss']=not bool(review_required.any())
    missing={'PREV_current_NEW_missing':int(lost.sum()),'NEW_current_PREV_missing':int((own['finite_current']&~outputs['P_PREV']['finite_current']).sum()),'lost_episodes':episodes(times,lost),
             'unexplained_missing':int(unexplained.sum()),'nonstartup_support_loss_requires_review':int(review_required.sum()),'contract':'No missing sample is treated as correct or removed from coverage; same-support regression is explicitly conditional and must be read with complete per-method raw summaries.'}
    result={'sequence':sequence,'baseline_valid':True,'P_NEW_native_complete':bool(engineering['complete_input']['P_NEW']),'engineering_pass':bool(engineering['pass']),'reference_valid':bool(all(weights[np.isfinite(ref[q]).all(axis=1)].sum()>=10 for q in ('v','eta'))),'status':'MEETS_SEQUENCE_CONTRACT' if all(decisions.values()) else 'NOT_MET','decisions':decisions,'coverage':coverage,'own_support':own_summary,'same_support':comparisons,'raw_regression':regression,'raw_missing':missing,'engineering':engineering,'reuse':reuse,
            'G_reference_exception':exception,'reference':{'path':str(gtpath),'sha':sha(gtpath),'physical_module_sha':sha(physical.__file__),'official_velocity':sequence in physical.EUROC},
            'provenance':{'evaluator_sha':sha(__file__),'acceptance_sha':sha(DOC/'acceptance.json'),'protocol_sha':sha(DOC/'protocol.json'),'runs':{k:str(p) for k,p in paths.items()},'files':{k:{f:sha(p/f) for f in ('features.jsonl','audit.csv','trajectory.csv','replay.json')} for k,p in paths.items()},'new_identity_sha':sha(identity_path(new))},
            'limitations':['Single sequence decision is not EuRoC >=8-valid/11-run or recovery/resource success.','P95 uses numpy linear quantile. V1_01 gravity accuracy is diagnostic only; coverage and physical health still enforced.']}
    arrays={'time':times,'camera_ns':np.asarray(keys,np.int64),'weights':weights,'reference_v':ref['v'],'reference_eta':ref['eta']}
    for name,o in outputs.items():
        for q in ('v','eta'):arrays[name+'_'+q]=o[q]
        for q in error[name]:arrays[name+'_error_'+q]=error[name][q]
        if name!='OpenVINS':
            for q in ('finite_current','ready_G','ready_V','epoch'):arrays[name+'_'+q]=o[q]
    out.mkdir(parents=True);np.savez_compressed(out/'error_curves.npz',**arrays)
    result['provenance']['curves_sha']=sha(out/'error_curves.npz');(out/'metrics.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    return result

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--sequence',required=True);p.add_argument('--new',required=True);p.add_argument('--out',required=True);p.add_argument('--baseline');p.add_argument('--previous');p.add_argument('--manifest',default='/home/he/output/ltv_feature_readiness_passive/final_manifest_recovered.json');a=p.parse_args();r=evaluate(**vars(a));print(json.dumps({'sequence':r['sequence'],'status':r['status'],'decisions':r['decisions']},indent=2))
