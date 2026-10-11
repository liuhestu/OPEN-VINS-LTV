import json,re,shutil,sys
from pathlib import Path
ROOT=Path('/home/he/open_vins_ltv_ws/src/open_vins_ltv');OUT=Path('/home/he/output/ltv_active_landmark_consistency_r4')
sys.path.insert(0,str(ROOT/'scripts/ltv_active_consistency_r4'))
import artifacts,budget,real_run
f=json.loads((budget.DOC/'frozen_config.json').read_text())
for p,d in f['source_sha'].items():assert artifacts.sha(ROOT/p)==d,p
artifacts.verify(f['runtime'])
on=OUT/'real_confirmation/CONF_ON_C02_01_ON_V2_03_difficult/P_NEW'
assert json.loads((on.parent/'attempt.json').read_text())['exit_code']==0
on_identity=json.loads((on/'identity.json').read_text())
assert on_identity['freeze_sha']==artifacts.sha(budget.DOC/'frozen_config.json')
assert on_identity['sequence']=='V2_03_difficult' and on_identity['mode']=='P_NEW'
assert on_identity['runtime']==f['runtime'] and on_identity['manifest_sha']==f['manifest_sha']
assert on_identity['source_sha']==budget.source_identity()
assert artifacts.tree(on.parent/'config')==on_identity['config_sha']
assert artifacts.sha(real_run.OLD/'input_metadata.json')==f['input_metadata_sha']
assert on_identity['inputs']==real_run.load(real_run.OLD/'input_metadata.json')['sequences']['V2_03_difficult']
for sensor in on_identity['inputs']['sensors'].values():assert artifacts.sha(sensor['path'])==sensor['sha256']
root=OUT/'real_confirmation/CONF_LOGOFF_C02_V2_03_01';assert not root.exists();root.mkdir()
shutil.copytree(on.parent/'config',root/'config')
config=root/'config/estimator_config.yaml';txt=config.read_text()
assert len(re.findall(r'^ltv_log_enabled: true$',txt,re.M))==1
config.write_text(re.sub(r'^ltv_log_enabled: true$','ltv_log_enabled: false',txt,flags=re.M))
identity=json.loads((on/'identity.json').read_text());identity.update(config_path=str(config),config_sha=artifacts.tree(config.parent),verification='logging_on_off_only',original_on=str(on),dispatcher_sha=artifacts.sha(__file__))
identity['active_config']=dict(identity['active_config'],ltv_log_enabled=False)
off=root/'P_NEW';off.mkdir()
for p in (root/'identity.json',off/'identity.json'):p.write_text(json.dumps(identity,indent=2)+'\n')
command=artifacts.command(f['runtime'],'run_ltv_feature_passive')+[str(config),identity['inputs']['root'],str(off),'P_NEW']
a=budget.run('real',command,identity,phase='confirmation');(root/'attempt.json').write_text(json.dumps(a,indent=2)+'\n')
assert a['exit_code']==0,a
proof={}
for name in ('cache.bin','audit.csv','trajectory.csv'):
 proof[name]={'ON':artifacts.sha(on/name),'OFF':artifacts.sha(off/name)}
 assert proof[name]['ON']==proof[name]['OFF'],name
checker=real_run.helper(ROOT/'scripts/ltv_feature_passive/audit/passive_outputs.py','_logger_audit')
engineering=checker.inspect(off);assert engineering['complete']
active=real_run.check_active(off/'features.jsonl')
result=dict(status='PASS_NATIVE_LOGGING_ON_OFF_EXACT',sequence=identity['sequence'],attempt=a,files=proof,passive=engineering,active_log=active,only_config_difference='ltv_log_enabled true -> false')
(root/'verification.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result),flush=True)
