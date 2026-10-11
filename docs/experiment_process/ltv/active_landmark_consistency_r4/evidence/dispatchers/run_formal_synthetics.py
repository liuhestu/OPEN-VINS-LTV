import concurrent.futures,hashlib,json,sys
from pathlib import Path
ROOT=Path('/home/he/open_vins_ltv_ws/src/open_vins_ltv')
OUT=Path('/home/he/output/ltv_active_landmark_consistency_r4')
sys.path.insert(0,str(ROOT/'scripts/ltv_active_consistency_r4'))
import budget,artifacts
freeze=budget.DOC/'frozen_config.json';f=json.loads(freeze.read_text());freeze_sha=artifacts.sha(freeze)
for name,sha in f['source_sha'].items():assert artifacts.sha(ROOT/name)==sha,name
assert artifacts.sha(f['synthetic_library'])==f['synthetic_library_sha']
assert artifacts.tree(f['base_config'])==f['base_config_tree_sha']
assert artifacts.sha(budget.DOC/'protocol.json')==f['protocol_sha']
assert artifacts.sha(budget.DOC/'acceptance.json')==f['acceptance_sha']
artifacts.verify(f['runtime'])
summary=OUT/'formal_synthetic_jobs.json';assert not summary.exists()
script=ROOT/'scripts/ltv_active_consistency_r4';cal=Path(f['base_config'])/'kalibr_imucam_chain.yaml'
def run(c):
 name=c['name'];inputs=OUT/'inputs/formal'/name;run_dir=OUT/'synthetic_confirmation'/name/'P_NEW_ACTIVE';eval_dir=run_dir.parent/'evaluation_v1'
 stages=[]
 def stage(kind,command,label):
  a=budget.run(kind,command,dict(stage=label,case=c,freeze_sha=freeze_sha,dispatcher_sha=artifacts.sha(__file__)),phase='confirmation');stages.append(dict(stage=label,attempt=a))
  if a['exit_code']:raise RuntimeError(dict(case=name,stage=label,attempt=a))
 if c['condition']=='PRESSURE':
  command=[sys.executable,str(script/'pressure_inputs.py'),'--out',str(inputs),'--seed',str(c['seed']),'--duration','60','--confirmation-freeze-sha',freeze_sha]
 else:
  command=[sys.executable,str(script/'synthetic_inputs.py'),'--out',str(inputs),'--calibration',str(cal),'--scene',c['scene'],'--lifetime',str(c['lifetime']),'--condition',c['condition'],'--seed',str(c['seed']),'--duration','60','--confirmation-sha',freeze_sha]
 stage('analysis',command,'frozen_input_generation')
 stage('synthetic',[sys.executable,str(script/'synthetic_runner.py'),'--input-dir',str(inputs),'--out',str(run_dir),'--library',f['synthetic_library'],'--mode','P_NEW_ACTIVE','--source',c['source']],'frozen_complete_C02')
 stage('analysis',[sys.executable,str(script/'synthetic_evaluate.py'),'--input-dir',str(inputs),'--run-dir',str(run_dir),'--out',str(eval_dir)],'frozen_offline_evaluation')
 result=dict(case=c,stages=stages,evaluation=str(eval_dir),baseline='UNVERIFIED_NO_IDENTICAL_OFF')
 print(json.dumps(result),flush=True);return result
summary.write_text(json.dumps(dict(complete=False,freeze_sha=freeze_sha,cases=f['synthetic_confirmation_cases']),indent=2)+'\n')
with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:results=list(pool.map(run,f['synthetic_confirmation_cases']))
summary.write_text(json.dumps(dict(complete=True,freeze_sha=freeze_sha,results=results),indent=2)+'\n')
