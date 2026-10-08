"""One budgeted deterministic sensor-only replay; all attempts are retained."""
import argparse, datetime, fcntl, json, os, re, shutil, subprocess, time, uuid
import numpy as np
from common import *
def identity():
    paths=[OUT/'install'/x/'lib'/f'lib{x}_lib.so' for x in ['ov_core','ov_init','ov_msckf']]
    return {'sources':source_identity(),'binary':sha(OUT/'install/ov_msckf/lib/ov_msckf/run_ltv_value'),'libraries':{str(p):sha(p) for p in paths}}
def run(seq,mode,candidate='C0',stage='diagnostic',short=False,diagnostics=True,repeat=False,profile=None):
    if seq not in SEQUENCES or mode not in MODES:raise ValueError('outside fixed matrix')
    lock=(OUT/'replay.lock').open('a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    manifest=read(OUT/'experiment_manifest.json')
    if stage=='final' and not manifest.get('weights_frozen'): raise ValueError('freeze before final')
    if candidate!='C0' and (seq not in DEV or stage!='candidate') and not manifest.get('weights_frozen'):raise ValueError('candidate outside development')
    if manifest.get('weights_frozen') and candidate!=manifest['selected_candidate']:raise ValueError('post-freeze candidate changed')
    inputs=read(OUT/'inputs.json')[seq];current=identity()
    if manifest.get('frozen_identity') and current!=manifest['frozen_identity']:raise ValueError('frozen executable/source identity changed')
    profile_identity={'path':str(Path(profile).resolve()),'sha256':sha(profile)} if profile else None
    if profile and not short:raise ValueError('instrumentation is restricted to short cost checks')
    old=list(latest_runs().values())
    full_count=sum(not x['short'] for x in old); short_count=sum(x['short'] for x in old)
    # Reuse only exact executed identity and diagnostics policy; repeats explicitly bypass reuse.
    if not repeat:
        for x in reversed(old):
            if all(x.get(k)==v for k,v in dict(sequence=seq,mode=mode,candidate=candidate,short=short,diagnostics=diagnostics,status='complete',identity=current,profile=profile_identity).items()):
                print('REUSE',x['id'],flush=True);return x
    if full_count>=80 and not short or short_count>=20 and short:raise RuntimeError('budget exhausted')
    for x in old:
        if x['status']=='running':
            if Path('/proc',str(x.get('pid',-1))).exists(): raise RuntimeError('recorded process live')
            raise RuntimeError('reconcile abandoned attempt before another run')
    for p,h in inputs['hashes'].items():
        if not p.endswith('.png') and sha(p)!=h:raise ValueError('source changed '+p)
    for p,h in inputs['derived_csv'].items():
        if sha(p)!=h:raise ValueError('derived input changed')
    name=datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'-'+uuid.uuid4().hex[:6]
    directory=OUT/'runs'/f'{seq}-{mode}-{candidate}-{name}.partial';directory.mkdir(parents=True)
    for p in Path(inputs['config']).iterdir():
        if p.is_file():shutil.copyfile(p,directory/p.name)
    values={'ltv_enabled':mode!='B','ltv_enable_gravity':mode in ['G','GV'],'ltv_enable_velocity':mode in ['V','GV'],
            'ltv_sigma_gravity_deg':WEIGHTS[candidate][0],'ltv_sigma_velocity_mps':WEIGHTS[candidate][1],
            'ltv_value_diagnostics_enabled':diagnostics,'ltv_value_diagnostics_matrices':diagnostics,
            'ltv_value_diagnostics_path':str(directory/'value.jsonl.gz'),'ltv_log_path':str(directory/'ltv.csv')}
    cfg=(directory/'estimator_config.yaml').read_text()
    for k,v in values.items():
        line=k+': '+json.dumps(v)
        cfg=re.sub(r'^'+k+r':.*$',line,cfg,flags=re.M) if re.search(r'^'+k+':',cfg,re.M) else cfg+'\n'+line+'\n'
    (directory/'estimator_config.yaml').write_text(cfg)
    limit=0
    if short:
        camera=np.loadtxt(Path(inputs['replay_root'])/'cam0/data.csv',delimiter=',',dtype=str,comments='#')[:,0].astype(np.int64)
        limit=int(np.sum(camera<=camera[0]+int(min(30.,.2*inputs['duration'])*1e9)))
    cmd=['bash','-c','source /opt/ros/humble/setup.bash && source "$1/install/setup.bash" && exec "$1/install/ov_msckf/lib/ov_msckf/run_ltv_value" "$2" "$3" "$4" "$5"','value-replay',str(OUT),str(directory/'estimator_config.yaml'),inputs['replay_root'],str(directory),str(limit)]
    if profile:
        cmd[2]=cmd[2].replace('&& exec ', '&& exec env "$6" "$7" ')
        cmd.extend(['LD_PRELOAD='+profile_identity['path'],'LTV_PROFILE_OUTPUT='+str(directory/'profile.csv')])
    r=dict(id=name,sequence=seq,mode=mode,candidate=candidate,stage=stage,short=short,limit=limit,diagnostics=diagnostics,identity=current,status='running',path=str(directory),command=cmd,config_sha=sha(directory/'estimator_config.yaml'),input_manifest_sha=sha(OUT/'inputs.json'),runner_sha=sha(__file__),gt_sha=inputs['hashes'][inputs['gt']])
    r['profile']=profile_identity
    def record():
        write(Path(r['path'])/'manifest.json',r)
        with (OUT/'runs.jsonl').open('a') as f:f.write(json.dumps(r)+'\n')
        write(OUT/'checkpoint.json',{'stage':stage,'full_starts':full_count+int(not short),'short_starts':short_count+int(short),'active':[{'pid':r['pid'],'id':name}] if r['status']=='running' else [],'last':name,'next':'poll live PID then audit result' if r['status']=='running' else 'continue stage after evidence checks'})
    start=time.monotonic()
    with (directory/'stdout.log').open('w') as so,(directory/'stderr.log').open('w') as se:
        p=subprocess.Popen(cmd,stdout=so,stderr=se);r['pid']=p.pid;record();print('START',seq,mode,candidate,name,flush=True);code=p.wait()
    r.update(exit_code=code,seconds=time.monotonic()-start,status='failed')
    if code==0 and (directory/'replay.json').exists():
        a=read(directory/'replay.json');r['replay']=a
        full_ok=a['complete'] and a['camera_packets']==a['input_camera_packets'] and a['imu_consumed']==a['input_imu_samples']
        if short or full_ok:r['status']='complete'
    if (directory/'effective_options.json').exists():
        effective=read(directory/'effective_options.json');r['effective_options']=effective
        mismatches={k:[v,effective.get(k)] for k,v in values.items() if k in effective and effective[k]!=v}
        if mismatches:r.update(status='failed',failure='effective_configuration',mismatches=mismatches)
        if abs(effective['calib_camimu_dt']-inputs['offset'])>1e-12:r.update(status='failed',failure='wrong_fixed_offset')
    if not short and r.get('replay',{}).get('output_rows',0)<3:r.update(status='failed',failure='initialization_or_output_failure')
    final=directory.with_suffix('.done' if r['status']=='complete' else '.failed');directory.rename(final);r['path']=str(final);record()
    print('END',seq,mode,r['status'],round(r['seconds'],2),str(final),flush=True)
    return r
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('sequence',choices=SEQUENCES);p.add_argument('mode',choices=MODES);p.add_argument('--candidate',default='C0',choices=WEIGHTS);p.add_argument('--stage',default='diagnostic');p.add_argument('--short',action='store_true');p.add_argument('--no-diagnostics',action='store_true');p.add_argument('--repeat',action='store_true');a=p.parse_args()
    r=run(a.sequence,a.mode,a.candidate,a.stage,a.short,not a.no_diagnostics,a.repeat)
    if r['status']!='complete':raise SystemExit(1)
