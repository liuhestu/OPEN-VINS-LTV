import concurrent.futures, hashlib, json, subprocess, sys
from pathlib import Path
ROOT=Path('/home/he/open_vins_ltv_ws/src/open_vins_ltv')
OUT=Path('/home/he/output/ltv_active_landmark_consistency_r4')
sys.path.insert(0,str(ROOT/'scripts/ltv_active_consistency_r4'))
import budget,real_evaluate
frozen=budget.DOC/'frozen_config.json';f=json.loads(frozen.read_text())
for relative,digest in f['source_sha'].items():assert real_evaluate.base.sha(ROOT/relative)==digest,relative
jobs=json.loads((OUT/'CONF_ON_C02_01_confirmation_jobs.json').read_text())
assert jobs['freeze_sha']==real_evaluate.base.sha(frozen)
assert jobs['complete'] and len(jobs['results'])==11 and all(q['exit_code']==0 for q in jobs['results'])
plan_path=OUT/'formal_evaluation_plan.json';plan=json.loads(plan_path.read_text())
assert len(plan)==11 and len({q['sequence'] for q in plan})==11 and {q['sequence'] for q in plan}==set(f['sequences'])
job_paths={q['sequence']:OUT/'real_confirmation'/q['name']/'P_NEW' for q in jobs['results']}
for q in plan:assert Path(q['new'])==job_paths[q['sequence']]
def run(q):
 command=[sys.executable,str(ROOT/'scripts/ltv_active_consistency_r4/real_evaluate.py')]
 for k,v in q.items():command.extend(['--'+k,v])
 attempt=budget.run('analysis',command,{'stage':'all11_fixed_evaluation','sequence':q['sequence'],'dispatcher_sha':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),'plan_sha':real_evaluate.base.sha(plan_path)},phase='confirmation')
 print(json.dumps(dict(sequence=q['sequence'],attempt=attempt)),flush=True)
 if attempt['exit_code']:raise RuntimeError(q['sequence'])
 return json.loads((Path(q['out'])/'metrics.json').read_text())
with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:records=list(pool.map(run,plan))
result=real_evaluate.base.assess_all11(records)
result['records']=[dict(sequence=q['sequence'],status=q['status'],decisions=q['decisions'],metrics=plan[i]['out']+'/metrics.json') for i,q in enumerate(records)]
p=OUT/'formal_all11_assessment.json'
assert not p.exists()
p.write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result),flush=True)
