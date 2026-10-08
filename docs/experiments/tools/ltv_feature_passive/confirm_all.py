"""Fixed confirmation matrix, two workers maximum, resumable verified identities."""
import concurrent.futures
import json
from pathlib import Path
import sys
import threading
import budget
import artifacts
import validate
from freeze import check

HERE=Path(__file__).resolve().parent
STOP=threading.Event()

def step():
    if STOP.is_set():raise RuntimeError('Stopped after another confirmation task failed')

def analysis(command,identity):
    step()
    r=budget.run('analysis',[sys.executable]+command,identity)
    if r['exit_code']:raise RuntimeError(r)
    return Path(r['directory'])/'stdout.log'

def real(sequence):
    frozen,digest=check();runs={}
    for mode in ('B','P_OLD','P_NEW'):
        step()
        runs[mode]=Path(validate.real(sequence,mode)[0])
    gt=(Path('/home/he/datasets/euroc/ASL')/sequence/'mav0/state_groundtruth_estimate0/data.csv'
        if sequence.startswith('V') else Path('/home/he/datasets/uzhfpv/archives')/(sequence+'_snapdragon_with_gt')/'groundtruth.txt')
    out=budget.OUT/'real_metrics'/(sequence+'_HYBRID_confirmation_'+digest[:12])
    seal=out/'artifacts_sha256.json'
    if out.exists():
        for name,value in json.loads(seal.read_text()).items():
            if artifacts.sha(out/name)!=value:raise RuntimeError('Evaluated confirmation artifact changed')
        d=json.loads((out/'metrics.json').read_text())
        if d['provenance']['evaluator_sha']!=artifacts.sha(HERE/'real_evaluate.py'):raise RuntimeError('Changed evaluator')
        for mode,files in d['provenance']['run_files'].items():
            for name,value in files.items():
                if artifacts.sha(runs[mode]/name)!=value:raise RuntimeError('Evaluated input changed')
    else:
        analysis([str(HERE/'real_evaluate.py'),'--sequence',sequence,'--gt',str(gt),
            '--baseline',str(runs['B']),'--old',str(runs['P_OLD']),'--new',str(runs['P_NEW']),
            '--out',str(out),'--confirmation-freeze-sha',digest],'confirmation_evaluate_'+sequence)
        seal.write_text(json.dumps(artifacts.tree(out),indent=2)+'\n')
    repeated=None
    if sequence in ('V2_03_difficult','indoor_forward_6'):
        step()
        second=Path(validate.real(sequence,'P_NEW',1)[0])
        repeated=analysis([str(HERE/'audit/repeat.py'),str(runs['P_NEW']),str(second)],'confirmation_repeat_exact_'+sequence)
    return {'kind':'real','sequence':sequence,'metrics':str(out/'metrics.json'),'repeat':str(repeated) if repeated else None}

def synthetic(scene,seed,life):
    old,new=validate.synthetic(scene,seed,life,before_launch=step)
    return {'kind':'synthetic','old':old,'new_metrics':str(Path(new)/'metrics.json')}

def execute(task):
    if STOP.is_set():raise RuntimeError('Stopped after another confirmation task failed')
    try:
        result=real(task[1]) if task[0]=='real' else synthetic(*task[1:])
        with budget.locked():
            p=budget.OUT/'confirmation_tasks.jsonl'
            with p.open('a') as stream:stream.write(json.dumps(result,sort_keys=True)+'\n')
        print(json.dumps(result),flush=True)
        return result
    except BaseException:
        STOP.set();raise

def main():
    frozen,digest=check()
    tasks=[('real','V1_03_difficult'),('synthetic','REGULAR',101,.5),
           ('real','V2_03_difficult'),('synthetic','FAST',101,.5),
           ('real','indoor_forward_6'),('synthetic','REGULAR',101,1.),
           ('synthetic','FAST',101,1.),('synthetic','REGULAR',102,.5),
           ('synthetic','FAST',102,.5),('synthetic','REGULAR',102,1.),('synthetic','FAST',102,1.)]
    with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
        results=list(pool.map(execute,tasks))
    readonly=analysis([str(HERE/'verify_readonly.py')],'final_readonly_check')
    m={'phase':'frozen_confirmation_with_development_context','freeze_sha':digest,
       'real_metrics':dict(frozen['development_evidence']['real_metrics']),
       'synthetic_metrics':[],'synthetic_pairs':{},'repeat_checks':{},
       'cache_check':str(budget.OUT/'V1_01_hybrid_full_cache_parity.json'),'readonly_check':str(readonly)}
    for r in results:
        if r['kind']=='real':
            m['real_metrics'][r['sequence']]=r['metrics']
            if r['repeat']:m['repeat_checks'][r['sequence']]=r['repeat']
        else:
            m['synthetic_metrics'].append(r['new_metrics']);m['synthetic_pairs'][r['new_metrics']]=r['old']
    path=budget.OUT/'final_manifest.json'
    value=json.dumps(m,indent=2)+'\n'
    if path.exists() and path.read_text()!=value:raise RuntimeError('Existing final manifest differs; preserve and review')
    if not path.exists():path.write_text(value)
    print(str(path),flush=True)

if __name__=='__main__':main()
