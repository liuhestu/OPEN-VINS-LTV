"""Budgeted synthetic development dispatcher. Confirmation is a separate frozen phase."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import sys
import budget

HERE=Path(__file__).resolve().parent

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()

def synthetic(scene,seed=42):
    if scene not in ['REGULAR','FAST'] or seed not in [42,43]:raise ValueError('Development identity only')
    library=budget.OUT/'synthetic_pipeline.so'
    frozen=budget.OUT/'binaries'/('synthetic_'+sha(library)+'.so');frozen.parent.mkdir(exist_ok=True)
    if not frozen.exists():shutil.copy2(library,frozen)
    inputs=budget.OUT/'synthetic_inputs'/f'{scene}_{seed}_1s'
    if not inputs.exists():
        r=budget.run('analysis',[sys.executable,str(HERE/'synthetic/generate.py'),'--out',str(inputs),'--scene',scene,'--seed',str(seed),'--lifetime','1'],'development_input_'+scene+'_'+str(seed))
        if r['exit_code']:raise RuntimeError(r)
    input_identity=json.loads((inputs/'identity.json').read_text())
    previous=None
    for method in ['OLD','SEL','SEED_TEMPORAL','SEED_STEREO']:
        identity={'scene':scene,'seed':seed,'method':method,'max_risk':.05,'library_sha':sha(frozen),'input_sha':input_identity['input_sha'],'phase':'development'}
        key=hashlib.sha256(json.dumps(identity,sort_keys=True).encode()).hexdigest()[:12]
        out=budget.OUT/'synthetic_runs'/f'{scene}_{seed}_{method}_{key}'
        meta=out/'run.json'
        if meta.exists():
            saved=json.loads(meta.read_text())
            if saved['status']!='COMPLETED_ESTIMATION' or not saved['full_input_consumed']:raise RuntimeError('Failed/interrupted directory remains reserved; explicit retry identity required')
            if saved['runner_sha']!=sha(HERE/'synthetic/run.py'):raise RuntimeError('Runner changed; explicit provenance review needed before reuse')
            if saved['library_sha']!=identity['library_sha'] or saved['input_identity']['input_sha']!=identity['input_sha']:raise RuntimeError('Identity mismatch')
        else:
            if budget.usage()['synthetic']>=32:raise RuntimeError('Preserve16 confirmation attempts')
            result=budget.run('synthetic',[sys.executable,str(HERE/'synthetic/run.py'),'--input',str(inputs),'--out',str(out),'--library',str(frozen),'--method',method],json.dumps(identity,sort_keys=True))
            print(json.dumps(result),flush=True)
            if result['exit_code']:raise RuntimeError('Estimator attempt failed; inspect preserved logs')
        command=[sys.executable,str(HERE/'synthetic/evaluate.py'),'--input',str(inputs),'--run',str(out)]
        if previous:command+=['--baseline',str(previous)]
        r=budget.run('analysis',command,'evaluate_'+key)
        if r['exit_code']:raise RuntimeError('Evaluator failure; no simulation retry authorized by tool error')
        if method=='OLD':previous=out
        print(json.dumps(json.loads((out/'metrics.json').read_text())),flush=True)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--scene',required=True,choices=['REGULAR','FAST']);p.add_argument('--seed',type=int,default=42);a=p.parse_args();synthetic(a.scene,a.seed)
