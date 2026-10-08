#!/usr/bin/env python3
"""One audited deterministic EuRoC replay. All full starts count, including failures."""
import argparse, datetime, hashlib, json, os, pathlib, re, shutil, subprocess, time, uuid, fcntl
ROOT=pathlib.Path(__file__).resolve().parents[4]
OUT=pathlib.Path('/home/he/output/openvins_ltv_autonomous_20261002')
ASL=pathlib.Path('/home/he/datasets/euroc/ASL')
def sha(path):
    h=hashlib.sha256()
    with open(path,'rb') as f:
        for data in iter(lambda:f.read(1024*1024),b''):h.update(data)
    return h.hexdigest()
def write(path,obj):
    tmp=path.with_suffix(path.suffix+'.tmp');tmp.write_text(json.dumps(obj,indent=2));tmp.replace(path)
def source_hashes():
    return {str(p.relative_to(ROOT)):sha(p) for package in ['ov_core','ov_init','ov_msckf','config/ltv_euroc','docs/experiments/tools/ltv'] for p in (ROOT/package).rglob('*') if p.is_file() and p.suffix in ['.h','.cpp','.cmake','.txt','.yaml','.py']}
def execution_identity():
    return dict(git_head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
                source_sha256=source_hashes(),
                binary_sha256=sha(OUT/'install/ov_msckf/lib/ov_msckf/run_ltv_euroc'),
                libraries_sha256={str(p):sha(p) for p in [OUT/'install'/pkg/'lib'/('lib'+pkg+'_lib.so') for pkg in ['ov_core','ov_init','ov_msckf']]},
                input_manifest_sha256=sha(OUT/'input_manifest.json'),
                evaluator_protocol_sha256=sha(OUT/'evaluator_protocol.json'))
def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('sequence');p.add_argument('mode',choices=['B','P','G','V','GV']);p.add_argument('--original',action='store_true');p.add_argument('--limit',type=int,default=0);p.add_argument('--stage',default='development');a=p.parse_args()
    lock=(OUT/'replay.lock').open('a')
    fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    manifest=json.loads((OUT/'experiment_manifest.json').read_text())
    if a.sequence not in manifest['final_sequences']:raise SystemExit('unknown sequence')
    if not manifest['frozen'] and a.sequence not in manifest['development_sequences']:raise SystemExit('pre-freeze sequence forbidden')
    if a.stage=='final' and not manifest['frozen']:raise SystemExit('final evaluation requires frozen manifest')
    if manifest['frozen']:
        if a.original or a.limit:raise SystemExit('formal frozen runs require current binary and full input')
        if execution_identity()!=manifest['execution_identity']:raise SystemExit('frozen code/config/input/evaluator/binary identity mismatch')
    inputs=json.loads((OUT/'input_manifest.json').read_text())[a.sequence]
    for relative,digest in inputs['files'].items():
        if relative.endswith('.csv') and sha(ASL/a.sequence/'mav0'/relative)!=digest:raise SystemExit('sensor/GT CSV changed: '+relative)
    runs=[json.loads(x) for x in (OUT/'runs.jsonl').read_text().splitlines() if x]
    latest={x['run_id']:x for x in runs}
    if any(x.get('status')=='running' and pathlib.Path('/proc',str(x.get('pid',-1))).exists() for x in latest.values()):raise SystemExit('another replay is live; inspect checkpoint')
    full_starts=len({x['run_id'] for x in runs if x.get('full')})
    if not a.limit and full_starts>=manifest['replay_limit']:raise SystemExit('full replay budget exhausted')
    run_id=datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'-'+uuid.uuid4().hex[:8]
    directory=OUT/'runs'/f'{a.sequence}-{a.mode}-{run_id}.partial';directory.mkdir(parents=True)
    cfg=(ROOT/'config/ltv_euroc/estimator_config.yaml').read_text()
    values={'ltv_enabled':a.mode!='B','ltv_enable_gravity':a.mode in ['G','GV'],'ltv_enable_velocity':a.mode in ['V','GV']}
    for key,value in values.items():cfg=re.sub(r'^'+key+r':.*$',key+': '+str(value).lower(),cfg,flags=re.M)
    cfg=re.sub(r'^ltv_log_path:.*$','ltv_log_path: '+json.dumps(str(directory/'ltv.csv')),cfg,flags=re.M)
    (directory/'estimator_config.yaml').write_text(cfg)
    for name in ['kalibr_imu_chain.yaml','kalibr_imucam_chain.yaml']:shutil.copyfile(ROOT/'config/ltv_euroc'/name,directory/name)
    binary=OUT/'baseline/bin/run_ltv_euroc' if a.original else OUT/'install/ov_msckf/lib/ov_msckf/run_ltv_euroc'
    libdir=OUT/'baseline/lib' if a.original else OUT/'install/ov_msckf/lib'
    command=['bash','-c','source /opt/ros/humble/setup.bash && export LD_LIBRARY_PATH="$1:$2/install/ov_core/lib:$2/install/ov_init/lib:$LD_LIBRARY_PATH" && exec "$3" "$4" "$5" "$6" "$7"', 'ltv_replay',str(libdir),str(OUT),str(binary),str(directory/'estimator_config.yaml'),str(ASL/a.sequence/'mav0'),str(directory),str(a.limit)]
    record=dict(run_id=run_id,sequence=a.sequence,mode=a.mode,original=a.original,stage=a.stage,full=not bool(a.limit),limit=a.limit,status='running',path=str(directory),command=command,binary_sha256=sha(binary),config_sha256=sha(directory/'estimator_config.yaml'),git_head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip())
    record['source_sha256']=source_hashes()
    record['input_sha256']=inputs['sha256']
    record['gt_sha256']=inputs['files']['state_groundtruth_estimate0/data.csv']
    record['evaluator_sha256']=sha(ROOT/'docs/experiments/tools/ltv/evaluate_sequence.py')
    record['evaluator_protocol_sha256']=sha(OUT/'evaluator_protocol.json')
    record['effective_mode_flags']=values
    record['runner_overrides']={'use_multi_threading_subs':False,'use_multi_threading_pubs':False}
    record['calibration_sha256']={name:sha(directory/name) for name in ['kalibr_imu_chain.yaml','kalibr_imucam_chain.yaml']}
    record['frozen']=manifest['frozen']
    record['libraries_sha256']={str(p):sha(p) for p in ((OUT/'baseline/lib').glob('*.so') if a.original else [OUT/'install'/pkg/'lib'/('lib'+pkg+'_lib.so') for pkg in ['ov_core','ov_init','ov_msckf']])}
    (directory/'worktree.diff').write_bytes(subprocess.check_output(['git','diff','HEAD'],cwd=ROOT))
    start=time.monotonic()
    with (directory/'stdout.log').open('w') as stdout,(directory/'stderr.log').open('w') as stderr:
        process=subprocess.Popen(command,stdout=stdout,stderr=stderr);record['pid']=process.pid
        write(directory/'manifest.json',record)
        with (OUT/'runs.jsonl').open('a') as f:f.write(json.dumps(record)+'\n')
        write(OUT/'checkpoint.json',dict(phase=a.stage,replays_started=full_starts+int(not a.limit),active_processes=[dict(pid=process.pid,run_id=run_id)],reset_cards_used=0,next='poll recorded process; do not restart while live'))
        print(json.dumps(record),flush=True);code=process.wait()
    record.update(exit_code=code,runtime_seconds=time.monotonic()-start,status='failed')
    if code==0 and (directory/'replay.json').exists():
        replay=json.loads((directory/'replay.json').read_text());record['replay']=replay
        if a.limit or (replay['complete'] and replay['camera_packets']==replay['input_camera_packets'] and replay['imu_consumed']==replay['input_imu_samples'] and replay['output_rows']>=3):record['status']='complete'
    effective_path=directory/'effective_options.json'
    if effective_path.exists():
        effective=json.loads(effective_path.read_text())
        record['effective_options']=effective
        expected={}
        for line in cfg.splitlines():
            if ':' not in line or line.lstrip().startswith(('#','%')):continue
            key,value=line.split(':',1);value=value.split('#',1)[0].strip()
            if key.strip() not in effective:continue
            try:expected[key.strip()]=json.loads(value)
            except json.JSONDecodeError:
                try:expected[key.strip()]=float(value)
                except ValueError:expected[key.strip()]=value.strip('"')
        mismatches={k:dict(expected=v,actual=effective[k]) for k,v in expected.items() if effective[k]!=v}
        if mismatches:record.update(status='failed',failure_reason='effective_configuration_mismatch',mismatches=mismatches)
        if effective.get('calib_camimu_dt')!=0:record.update(status='failed',failure_reason='evaluator_requires_zero_fixed_offset')
    elif manifest['frozen']:
        record.update(status='failed',failure_reason='missing_effective_options')
    final=directory.with_suffix('.done' if record['status']=='complete' else '.failed')
    # Log paths intentionally retain the executed partial name in the immutable effective config.
    record['executed_path']=str(directory);record['path']=str(final);write(directory/'manifest.json',record);directory.rename(final)
    with (OUT/'runs.jsonl').open('a') as f:f.write(json.dumps(record)+'\n')
    write(OUT/'checkpoint.json',dict(phase=a.stage,replays_started=full_starts+int(not a.limit),active_processes=[],reset_cards_used=0,last_run=record,next='audit run, advance only after stage gate'))
    print(json.dumps(record),flush=True)
    if record['status']!='complete':raise SystemExit(code or 1)
if __name__=='__main__':main()
