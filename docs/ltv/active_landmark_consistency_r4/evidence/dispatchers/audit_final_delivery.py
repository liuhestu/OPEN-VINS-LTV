import hashlib,json,sys
from pathlib import Path
ROOT=Path('/home/he/open_vins_ltv_ws/src/open_vins_ltv');OUT=Path('/home/he/output/ltv_active_landmark_consistency_r4');DOC=ROOT/'docs/ltv/active_landmark_consistency_r4'
sys.path.insert(0,str(ROOT/'scripts/ltv_active_consistency_r4'));import budget
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
old=json.loads((DOC/'evidence/final_protected_files_checkpoint.json').read_text())
for p,digest in old['protected_files'].items():assert sha(ROOT/p)==digest,p
f=json.loads((DOC/'frozen_config.json').read_text())
for p,digest in f['source_sha'].items():assert sha(ROOT/p)==digest,p
for k in ('protocol','acceptance','goal'):assert sha(DOC/(k+('.md' if k=='goal' else '.json')))==f[k+'_sha'],k
baseline=json.loads((DOC/'evidence/baseline_identity.json').read_text());parent=Path('/home/he/output/ltv_passive_hardening_v2/attempts.jsonl');assert sha(parent)==baseline['parent_attempts_sha256']
events=budget.events();reserved={e['id']:e for e in events if e['event']=='reserved'};terminal={e['id']:e for e in events if e['event']=='finished'}
# The currently executing final analysis is the only permitted live ledger entry.
live=sorted(set(reserved)-set(terminal));assert len(live)==1 and reserved[live[0]]['kind']=='analysis',live
rows=[]
for i,e in sorted(reserved.items()):
 if i in live:continue
 t=terminal[i];directory=OUT/'attempts'/f'{i:04d}'
 rows.append(dict(id=i,kind=e['kind'],units=e['units'],command=e['command'],exit_code=t['exit_code'],elapsed_seconds=t.get('seconds'),phase=e['identity'].get('phase'),attempt_directory=str(directory),files={n:sha(directory/n) for n in ('source_sha256.json','command_files_sha256.json','stdout.log','stderr.log') if (directory/n).exists()}))
(DOC/'evidence/final_attempts_index.json').write_text(json.dumps(rows,indent=2)+'\n')
result=dict(status='PASS_FINAL_PROTECTED_FILES_AND_FROZEN_EXECUTION',baseline=f['baseline'],protected_files_checked=len(old['protected_files']),protected_manifest_sha=sha(DOC/'evidence/final_protected_files_checkpoint.json'),frozen_source_files_checked=len(f['source_sha']),freeze_sha=sha(DOC/'frozen_config.json'),parent_ledger_sha=sha(parent),usage=budget.usage(),completed_attempts=len(rows),failed_attempts=[r['id'] for r in rows if r['exit_code']!=0],currently_executing_final_audit_id=live[0],no_other_unfinished_registered_processes=True,user_files_excluded_from_commit=baseline['user_untracked_preserved'])
(DOC/'evidence/final_delivery_audit.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
