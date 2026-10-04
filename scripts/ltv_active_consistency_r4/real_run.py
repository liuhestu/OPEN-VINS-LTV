"""Registered sensor-only native R4 replay; no reference file is opened."""
import argparse, importlib.util, json, re, shutil
from pathlib import Path
import budget, artifacts
from check_live_log import check as check_active
OLD=Path('/home/he/output/ltv_passive_hardening_v2')
def load(p):return json.loads(Path(p).read_text())
def helper(path,name):
 s=importlib.util.spec_from_file_location(name,path);m=importlib.util.module_from_spec(s);s.loader.exec_module(m);return m

def run(name,sequence,config,runtime,enabled,angle,phase,baseline=False):
 if not re.fullmatch(r'[A-Za-z0-9_.-]+',name):raise ValueError('safe run name required')
 if angle not in (.02,.04,.08):raise ValueError('outside authorized angular candidates')
 if baseline and enabled:raise ValueError('baseline cannot enable consistency')
 if phase=='confirmation':
  frozen=load(budget.DOC/'frozen_config.json')
  if angle!=frozen['angle'] or sequence not in frozen['sequences']:raise ValueError('frozen candidate mismatch')
  if not enabled and not frozen['allow_passive_off_controls']:raise ValueError('frozen OFF control not authorized')
  if baseline and not frozen.get('allow_new_B_controls',False):raise ValueError('frozen B control not authorized')
  if Path(runtime).resolve()!=Path(frozen['runtime']).resolve() or artifacts.sha(Path(runtime)/'manifest.json')!=frozen['manifest_sha']:raise ValueError('frozen runtime mismatch')
  if artifacts.tree(config)!=frozen['base_config_tree_sha']:raise ValueError('frozen base config mismatch')
  for k in ('protocol','acceptance'):
   if artifacts.sha(budget.DOC/(k+'.json'))!=frozen[k+'_sha']:raise ValueError('frozen contract changed')
  if artifacts.sha(OLD/'input_metadata.json')!=frozen['input_metadata_sha']:raise ValueError('frozen input metadata changed')
  for relative,digest in frozen['source_sha'].items():
   if artifacts.sha(budget.ROOT/relative)!=digest:raise ValueError('frozen execution source changed: '+relative)
 root=budget.OUT/('real_confirmation' if phase=='confirmation' else 'real_development')/name
 if root.exists():raise FileExistsError(root)
 artifacts.verify(runtime)
 inputs=load(OLD/'input_metadata.json')['sequences'][sequence]
 for s in inputs['sensors'].values():
  if artifacts.sha(s['path'])!=s['sha256']:raise ValueError('sensor changed')
 root.mkdir(parents=True);shutil.copytree(config,root/'config')
 path=root/'config/estimator_config.yaml';lines=path.read_text().splitlines()
 updates=dict(ltv_active_consistency_enabled=enabled,ltv_active_consistency_min_history_frames=5,
  ltv_active_consistency_min_history_span_s=.2,ltv_active_consistency_max_history_residual_rad=angle,
  ltv_active_consistency_max_holdout_residual_rad=angle,ltv_active_consistency_retire_after_consecutive_failures=2)
 if baseline:
  updates.update(ltv_enabled=False,ltv_feature_readiness_enabled=False,ltv_passive_hardening_enabled=False)
 for key,value in updates.items():
  loc=[k for k,l in enumerate(lines) if re.match(r'^\s*'+re.escape(key)+r'\s*:',l)]
  if len(loc)>1:raise ValueError('duplicate option '+key)
  text=key+': '+(str(value).lower() if isinstance(value,bool) else str(value))
  if loc:lines[loc[0]]=text
  else:lines.append(text)
 path.write_text('\n'.join(lines)+'\n')
 out=root/('B' if baseline else 'P_NEW');out.mkdir()
 identity=dict(sequence=sequence,mode='B' if baseline else ('P_NEW' if enabled else 'P_PREV'),phase=phase,revision='R4_ACTIVE_CONSISTENCY',
  runtime=str(runtime),manifest_sha=artifacts.sha(Path(runtime)/'manifest.json'),config_path=str(path),
  config_sha=artifacts.tree(path.parent),inputs=inputs,source_sha=budget.source_identity(),active_config=updates,
  baseline_config_tree_sha=artifacts.tree(config),protocol_sha=artifacts.sha(budget.DOC/'protocol.json'))
 if phase=='confirmation':identity['freeze_sha']=artifacts.sha(budget.DOC/'frozen_config.json')
 for p in (root/'identity.json',out/'identity.json'):p.write_text(json.dumps(identity,indent=2)+'\n')
 command=artifacts.command(runtime,'run_ltv_feature_passive')+[str(path),inputs['root'],str(out),'B' if baseline else 'P_NEW']
 attempt=budget.run('real',command,identity,phase=phase)
 (root/'attempt.json').write_text(json.dumps(attempt,indent=2)+'\n')
 if attempt['exit_code']:raise RuntimeError('failed native attempt preserved')
 audit=helper(budget.ROOT/'scripts/ltv_feature_passive/audit/passive_outputs.py','_r4_passive_audit')
 log=helper(budget.ROOT/'scripts/ltv_passive_hardening/check_hardened_log.py','_r4_log_check')
 engineering=dict(passive=audit.inspect(out))
 if not baseline:engineering['live_log']=log.check(out)
 if enabled:engineering['active_log']=check_active(out/'features.jsonl')
 else:
  for line in (out/'features.jsonl').read_text().splitlines():
   if 'active_consistency_schema' in json.loads(line):raise ValueError('disabled module emitted ON schema')
  engineering['active_log']='OFF_SCHEMA_ABSENT; full cache parity evaluated separately'
 if not engineering['passive']['complete']:raise ValueError('incomplete full input')
 (root/'engineering.json').write_text(json.dumps(engineering,indent=2)+'\n')
 print(json.dumps(dict(output=str(out),attempt=attempt,engineering=engineering)))
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('name');p.add_argument('sequence');p.add_argument('--config',type=Path,required=True);p.add_argument('--runtime',type=Path,required=True);p.add_argument('--enabled',action='store_true');p.add_argument('--baseline',action='store_true');p.add_argument('--angle',type=float,default=.04);p.add_argument('--phase',choices=['development','confirmation'],default='development');run(**vars(p.parse_args()))
