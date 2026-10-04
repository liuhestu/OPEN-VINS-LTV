import hashlib, importlib.util, json, shutil, subprocess
from pathlib import Path
OUT=Path(__file__).resolve().parent
REPO=Path('/home/he/open_vins_ltv_ws/src/open_vins_ltv')
DEST=REPO/'docs/ltv/integration_cleanup/layout_and_openvins_input/real_20261005'
DEST.mkdir(exist_ok=False)
def read(p): return json.loads(Path(p).read_text())
def sha(p):
 h=hashlib.sha256()
 with Path(p).open('rb') as f:
  for b in iter(lambda:f.read(1<<20),b''):h.update(b)
 return h.hexdigest()
def git(*args,cwd=REPO):return subprocess.check_output(['git',*args],cwd=cwd,text=True).strip()
def write(p,d):p.write_text(json.dumps(d,indent=2)+'\n')
identity=read(OUT/'initial_identity.json')
assert git('branch','--show-current')=='feature/ltv-openvins-integration-cleanup'
assert git('rev-parse','HEAD')==identity['current_ref']
subprocess.run(['git','merge-base','--is-ancestor','f1df913','HEAD'],cwd=REPO,check=True)
assert not git('diff','--name-only',identity['current_ref'],'--','ov_core','ov_init','ov_msckf','config','AGENTS.md')
assert git('rev-parse','HEAD',cwd=identity['baseline'])==identity['baseline_ref']
assert not git('status','--porcelain',cwd=identity['baseline'])
assert sha(REPO/'AGENTS.md')==hashlib.sha256(subprocess.check_output(['git','show',identity['current_ref']+':AGENTS.md'],cwd=REPO)).hexdigest()
inputs=read(OUT/'real_inputs.json')
for case in inputs.values():
 frozen=read(case['frozen_C02_run_identity']);assert sha(case['frozen_C02_run_identity'])==case['frozen_identity_sha']
 assert frozen['config_sha']==case['config_sha']
 for name,digest in case['config_sha'].items():
  assert sha(Path(case['config']).parent/name)==digest
  assert sha(Path(frozen['config_path']).parent/name)==digest
 for key,sensor in case['sensors'].items():
  assert sha(sensor['path'])==sensor['sha256']==frozen['inputs']['sensors'][key]['sha256']
images=read(OUT/'images_unchanged_after_replay.json');assert images['status']=='PASS_IMAGE_IDENTITIES_UNCHANGED'
assert all(images['aggregates'][seq]==case['images_aggregate_sha'] for seq,case in inputs.items())
assert all(x['exit_code']==0 for x in read(OUT/'build_results.json'))
assert read(OUT/'build_identity.json')['status']=='PASS_SAME_ROS_TOOLCHAIN_FLAGS_AND_SOURCE_LIST'
assert read(OUT/'runtime_identity.json')['status']=='PASS_ISOLATED_LOCAL_LIBRARIES_SAME_SYSTEM_LIBRARIES'
assert read(OUT/'source_migration_recheck.json')['status']=='PASS'
assert all(x['total']==24 and not x['failed'] for x in read(OUT/'tests_summary.json'))
checker=REPO/'scripts/ltv_integration_cleanup/check_real_outputs.py'
spec=importlib.util.spec_from_file_location('exact_checker',checker);module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
comparisons={}
for seq in inputs:
 for label,index,name in [('baseline',2,'baseline_repeat_'+seq+'_strict.json'),('current',1,'regression_'+seq+'.json')]:
  fresh=module.compare(OUT/'runs'/('baseline_'+seq+'_01'),OUT/'runs'/f'{label}_{seq}_{index:02d}')
  assert fresh==read(OUT/name)
  comparisons[name]=fresh
selftest_cmd=['python3',str(REPO/'scripts/ltv_integration_cleanup/test_real_output_checker.py')]
r=subprocess.run(selftest_cmd,capture_output=True,text=True);assert r.returncode==0
write(DEST/'checker_selftest.json',dict(command=selftest_cmd,exit_code=r.returncode,stdout=r.stdout,stderr=r.stderr,checker_sha=sha(checker),selftest_sha=sha(selftest_cmd[1])))
coverage={}
for run in sorted((OUT/'runs').iterdir()):
 ident=read(run/'identity.json');assert ident['exit_code']==0
 assert sha(ident['binary'])==ident['binary_sha']
 rows=[json.loads(s) for s in (run/'features.jsonl').open()]
 coverage[run.name]=dict(camera_events=len(rows),available=sum(bool(x.get('available')) for x in rows),ready_G=sum(bool(x.get('ready_G')) for x in rows),ready_V=sum(bool(x.get('ready_V')) for x in rows),mean_writes=sum(bool(s.get('mean_written')) for x in rows for s in x.get('seeds',[])),retirement_event_records=sum(len(x.get('retirement_events',[])) for x in rows),consistency_skip_count_sum=sum(x.get('consistency_skip_count',0) for x in rows),consistency_retire_count_sum=sum(x.get('consistency_retire_count',0) for x in rows),max_state_features=max(x.get('state_features',0) for x in rows))
for seq in inputs:
 assert coverage['baseline_'+seq+'_01']==coverage['baseline_'+seq+'_02']==coverage['current_'+seq+'_01']
write(DEST/'event_coverage.json',coverage)
# Archive compact identities and exact command records; large binaries/data stay external.
names=['initial_identity.json','build_results.json','build_identity.json','runtime_identity.json','source_migration_recheck.json','real_inputs.json','tests_summary.json','checker_scope_fix.json','images_unchanged_after_replay.json','baseline_run_jobs.json','current_run_jobs.json',*comparisons]
for name in names:shutil.copyfile(OUT/name,DEST/name)
for label in ['baseline','current']:
 shutil.copyfile(OUT/label/'tests/results.json',DEST/(label+'_tests.json'))
commands=DEST/'commands';commands.mkdir()
for name in ['build_both.py','check_build_identity.py','test_both.py','runtime_identity.py','run_cases.py','image_identity.py','final_audit.py']:
 shutil.copyfile(OUT/name,commands/name)
for label in ['baseline','current']:shutil.copyfile(OUT/label/'build.sh',commands/(label+'_build.sh'))
external_files={}
for pattern in ['*/build.stdout','*/build.stderr','*/tests/*.stdout','*/tests/*.stderr','runs/*/identity.json','runs/*/stdout.log','runs/*/stderr.log','runs/*/replay.json','images_identity.json']:
 for p in OUT.glob(pattern):external_files[str(p.relative_to(OUT))]=dict(bytes=p.stat().st_size,sha256=sha(p))
manifest=dict(status='PASS_REAL_ROS_PASSIVE_C02_CLEANUP_EQUIVALENCE',date='2026-10-05',branch=git('branch','--show-current'),tested_source=identity['current_ref'],baseline_source=identity['baseline_ref'],external_output=str(OUT),production_source_and_config_unchanged_since_tested_ref=True,baseline_worktree_clean=True,AGENTS_unchanged=True,requirements={
 '1':'Independent clean f3e055ab worktree; isolated builds, same compiler/flags/config/data: initial_identity, build_identity, runtime_identity, real_inputs',
 '2':'Both actual ROS/colcon builds exit 0; 24 related tests each pass with LTV assertions enabled; source migration audit passes',
 '3':'Six complete native real EuRoC replays exit 0: four baseline, two current; no short limit; C02 config exact frozen bytes',
 '4':'All camera timestamps: main values/FEJ/full covariance/layout SHA1 digests exact; full LTV serialized x/P,input,ID/slot,birth/retire,active suffix and event JSON exact',
 '5':'Both baseline repeats exact; no first divergence or numerical tolerance; cache fallback not needed',
 '6':'Commands, identities, exit codes, exact comparison and limits saved here; evidence and checker only, commit/push follow'},excluded_fields=['features.jsonl top-level compute_time_ms'],tolerance=0,unexecuted=['Frozen-input fallback replay (not needed: native baseline repeat and current comparison exact)','Live ROS topic/bag launch, ROS1 build, datasets beyond V2_02/V2_03, ATE, further scientific R4 acceptance (outside this cleanup regression)','Unrelated upstream simulator/dataset test executables and intentional-failure alias/golden harness (not needed for the related test suite)'],scientific_R4_status='NOT_ACHIEVED_UNCHANGED',G_V_feedback_enabled=False,external_file_identities=external_files)
manifest['archived_files']={str(p.relative_to(DEST)):dict(bytes=p.stat().st_size,sha256=sha(p)) for p in sorted(DEST.rglob('*')) if p.is_file()}
write(DEST/'manifest.json',manifest)
print(json.dumps(dict(status=manifest['status'],coverage=coverage,archived_files=len(manifest['archived_files'])),indent=2))
