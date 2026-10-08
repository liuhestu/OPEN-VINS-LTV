"""Execute only predeclared confirmation identities after a verified method freeze."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import budget
import artifacts
from freeze import check

HERE=Path(__file__).resolve().parent

def synthetic(scene,seed,lifetime,before_launch=lambda:None):
    frozen,digest=check()
    protocol=json.loads((budget.DOC/'protocol.json').read_text())
    if scene not in protocol['synthetic_scenarios'] or seed not in protocol['confirmation_seeds'] or lifetime not in protocol['synthetic_confirmation_lifetimes']:
        raise ValueError('Not a frozen confirmation input')
    inputs=budget.OUT/'synthetic_inputs'/f'{scene}_{seed}_{lifetime:g}s'
    if not inputs.exists():
        before_launch()
        r=budget.run('analysis',[sys.executable,str(HERE/'synthetic/generate.py'),'--out',str(inputs),'--scene',scene,
            '--seed',str(seed),'--lifetime',str(lifetime),'--confirmation-freeze-sha',digest],f'confirmation_input_{scene}_{seed}_{lifetime}')
        if r['exit_code']:raise RuntimeError(r)
    inp=json.loads((inputs/'identity.json').read_text())
    if inp['confirmation_freeze_sha']!=digest:raise RuntimeError('Input freeze mismatch')
    baseline=None
    result=[]
    for method in ['OLD',frozen['method']]:
        before_launch()
        identity={'phase':'confirmation','freeze_sha':digest,'input_identity':inp,'method':method,
            'library_sha':artifacts.sha(frozen['synthetic_library']),'runner_sha':artifacts.sha(HERE/'synthetic/run.py')}
        key=hashlib.sha256(json.dumps(identity,sort_keys=True).encode()).hexdigest()[:12]
        out=budget.OUT/'synthetic_runs'/f'{scene}_{seed}_{method}_{key}'
        if out.exists():
            seal=json.loads((out/'artifacts_sha256.json').read_text())
            if json.loads((out/'identity.json').read_text())!=identity:raise RuntimeError('Run identity mismatch')
            for name,value in seal.items():
                if artifacts.sha(out/name)!=value:raise RuntimeError('Saved artifact changed')
            saved=json.loads((out/'run.json').read_text())
            if saved['status']!='COMPLETED_ESTIMATION' or not saved['full_input_consumed']:raise RuntimeError('Interrupted attempt retained')
        else:
            r=budget.run('synthetic',[sys.executable,str(HERE/'synthetic/run.py'),'--input',str(inputs),'--out',str(out),
                '--library',frozen['synthetic_library'],'--method',method,'--max-risk',str(frozen['risk'])],json.dumps(identity,sort_keys=True))
            if r['exit_code']:raise RuntimeError('Preserved failed confirmation attempt')
            (out/'identity.json').write_text(json.dumps(identity,indent=2)+'\n')
            (out/'artifacts_sha256.json').write_text(json.dumps(artifacts.tree(out),indent=2)+'\n')
        command=[sys.executable,str(HERE/'synthetic/evaluate.py'),'--input',str(inputs),'--run',str(out)]
        if baseline:command+=['--baseline',str(baseline)]
        before_launch()
        r=budget.run('analysis',command,'confirmation_evaluate_'+key)
        if r['exit_code']:raise RuntimeError(r)
        if method=='OLD':baseline=out
        result.append(str(out))
    return result

def real(sequence,mode,repeat=0):
    from real_run import run
    frozen,_=check()
    return [str(p) for p in run(sequence,mode,frozen['source'],repeat=repeat,confirmation=True)]

if __name__=='__main__':
    p=argparse.ArgumentParser();sub=p.add_subparsers(dest='kind',required=True)
    s=sub.add_parser('synthetic');s.add_argument('--scene',required=True);s.add_argument('--seed',type=int,required=True);s.add_argument('--lifetime',type=float,required=True)
    r=sub.add_parser('real');r.add_argument('--sequence',required=True);r.add_argument('--mode',required=True);r.add_argument('--repeat',type=int,default=0)
    a=p.parse_args();print(json.dumps(synthetic(a.scene,a.seed,a.lifetime) if a.kind=='synthetic' else real(a.sequence,a.mode,a.repeat)))
