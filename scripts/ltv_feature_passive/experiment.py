"""One immutable synthetic development identity, with explicit reuse and retry rules."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import sys
import budget

HERE=Path(__file__).resolve().parent

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()

def run(scene, seed=42, lifetime=1., method='SEED_HYBRID', baseline=None):
    if scene not in ('REGULAR','FAST') or seed not in (42,43) or lifetime not in (.5,1.):
        raise ValueError('Development inputs only')
    if method not in ('OLD','SEL','SEED_TEMPORAL','SEED_STEREO','SEED_HYBRID'):
        raise ValueError('Unregistered candidate')
    library=budget.OUT/('synthetic_hybrid.so' if method=='SEED_HYBRID' else 'synthetic_pipeline.so')
    frozen=budget.OUT/'binaries'/('synthetic_'+sha(library)+'.so');frozen.parent.mkdir(exist_ok=True)
    if not frozen.exists():shutil.copy2(library,frozen)
    inputs=budget.OUT/'synthetic_inputs'/f'{scene}_{seed}_{lifetime:g}s'
    if not inputs.exists():
        r=budget.run('analysis',[sys.executable,str(HERE/'synthetic/generate.py'),'--out',str(inputs),
            '--scene',scene,'--seed',str(seed),'--lifetime',str(lifetime)],f'dev_input_{scene}_{seed}_{lifetime}')
        if r['exit_code']:raise RuntimeError(r)
    inp=json.loads((inputs/'identity.json').read_text())
    identity={'scene':scene,'seed':seed,'method':method,'max_risk':.05,'library_sha':sha(frozen),
              'input_sha':inp['input_sha'],'runner_sha':sha(HERE/'synthetic/run.py'),'phase':'development'}
    key=hashlib.sha256(json.dumps(identity,sort_keys=True).encode()).hexdigest()[:12]
    out=budget.OUT/'synthetic_runs'/f'{scene}_{seed}_{method}_{key}'
    if out.exists():
        if (out/'artifacts_sha256.json').exists():
            for name,digest in json.loads((out/'artifacts_sha256.json').read_text()).items():
                if sha(out/name)!=digest:raise RuntimeError('Saved run artifact changed: '+name)
        saved=json.loads((out/'run.json').read_text())
        if (saved['status']!='COMPLETED_ESTIMATION' or not saved['full_input_consumed'] or
            saved['library_sha']!=identity['library_sha'] or saved['input_identity']!=inp or
            saved['method']!=method or saved['max_relative_risk']!=.05 or
            saved['runner_sha']!=identity['runner_sha'] or
            saved['final_state_sha']!=sha(out/'states.npz')):
            raise RuntimeError('Existing result incomplete or identity mismatch; never silently retry')
    else:
        if budget.usage()['synthetic']>=32:raise RuntimeError('Preserve sixteen confirmation attempts')
        r=budget.run('synthetic',[sys.executable,str(HERE/'synthetic/run.py'),'--input',str(inputs),
            '--out',str(out),'--library',str(frozen),'--method',method],json.dumps(identity,sort_keys=True))
        print(json.dumps(r),flush=True)
        if r['exit_code']:raise RuntimeError('Simulation failed; preserve evidence before explicit retry')
    seal=out/'artifacts_sha256.json'
    if not seal.exists():
        seal.write_text(json.dumps({name:sha(out/name) for name in ('run.json','states.npz','matrices.npz','events.jsonl')},indent=2)+'\n')
    cmd=[sys.executable,str(HERE/'synthetic/evaluate.py'),'--input',str(inputs),'--run',str(out)]
    if baseline:cmd+=['--baseline',str(baseline)]
    r=budget.run('analysis',cmd,'evaluate_'+key)
    if r['exit_code']:raise RuntimeError('Analysis failed; simulation remains reusable')
    print(json.dumps({'output':str(out),'metrics':str(out/'metrics.json')}),flush=True)
    return out

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--scene',required=True);p.add_argument('--seed',type=int,default=42)
    p.add_argument('--lifetime',type=float,default=1);p.add_argument('--method',default='SEED_HYBRID');p.add_argument('--baseline',type=Path)
    a=p.parse_args();run(a.scene,a.seed,a.lifetime,a.method,a.baseline)
