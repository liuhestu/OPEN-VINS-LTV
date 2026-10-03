"""Final evidence completeness and frozen scientific criteria; no algorithm tuning."""
import argparse
import json
from pathlib import Path
import budget
from artifacts import sha
from freeze import check

def assess(manifest):
    frozen,digest=check();m=json.loads(Path(manifest).read_text())
    protocol=json.loads((budget.DOC/'protocol.json').read_text())
    if m['freeze_sha']!=digest:raise ValueError('Final manifest freeze mismatch')
    if set(m['real_metrics'])!=set(protocol['real_development']+protocol['real_confirmation']):
        raise ValueError('Incomplete six-sequence real evidence')
    real={seq:json.loads(Path(p).read_text()) for seq,p in m['real_metrics'].items()}
    for seq,d in real.items():
        identity=d['seed_source_identity']
        if d['sequence']!=seq or identity['sequence']!=seq or identity['source']!=frozen['source'] or identity['runtime']!=frozen['runtime']:
            raise ValueError('Mismatched real sequence or selected method')
    seeds={}
    for path in m['synthetic_metrics']:
        p=Path(path);metric=json.loads(p.read_text());r=json.loads((p.parent/'run.json').read_text())
        inp=r['input_identity'];key=(inp['scene'],inp['seed'],inp['lifetime'])
        if inp['confirmation_freeze_sha']!=digest or r['method']!=frozen['method']:raise ValueError('Nonconfirmation seed evidence')
        old_path=Path(m['synthetic_pairs'][str(p)])
        old=json.loads((old_path/'run.json').read_text())
        if old['method']!='OLD' or old['input_identity']!=inp:raise ValueError('Missing same-input OLD pair')
        for directory,run in ((p.parent,r),(old_path,old)):
            if run['status']!='COMPLETED_ESTIMATION' or not run['full_input_consumed']:
                raise ValueError('Incomplete confirmation pair')
            if run['library_sha']!=sha(frozen['synthetic_library']) or run['runner_sha']!=sha(Path(__file__).parent/'synthetic/run.py'):
                raise ValueError('Confirmation pair used another numerical implementation')
            if run['final_state_sha']!=sha(directory/'states.npz'):raise ValueError('State artifact changed')
        if metric['status']!='EVALUATED' or not metric['diagnostics']['full_input_consumed'] or not metric.get('baseline'):
            raise ValueError('Missing complete paired evaluation')
        if key in seeds:raise ValueError('Duplicate confirmation scenario')
        s=metric['seed'];seeds[key]={'accepted':s['accepted_seeds'],'acceptance':s['accepted_fraction_opportunity'],
            'reliable10':s['reliable_10pct_fraction'],'reliable5':s['reliable_5pct_fraction'],
            'pass':s['accepted_seeds']>=100 and s['accepted_fraction_opportunity']>=.2 and s['reliable_10pct_fraction'] is not None and s['reliable_10pct_fraction']>=.9}
    expected={(scene,seed,life) for scene in protocol['synthetic_scenarios'] for seed in protocol['confirmation_seeds'] for life in protocol['synthetic_confirmation_lifetimes']}
    if set(seeds)!=expected:raise ValueError('Incomplete eight-scenario seed evidence')
    engineering=all(d['engineering']['pass'] for d in real.values())
    if set(m['repeat_checks'])!=set(protocol['repeat_P_NEW']):raise ValueError('Missing prescribed repeats')
    for seq,path in m['repeat_checks'].items():
        d=json.loads(Path(path).read_text())
        if d['sequence']!=seq or d['identity']['sequence']!=seq or d['repeat_indices']!=[0,1] or d['identity']['freeze_sha']!=digest:
            raise ValueError('Repeat evidence belongs to another identity')
        engineering &= d['status']=='PASS' and d['repeated_auxiliary_exact']
    for key in ('cache_check','readonly_check'):
        engineering &= json.loads(Path(m[key]).read_text())['status']=='PASS'
    conf=[real[s] for s in protocol['real_confirmation']]
    for d in conf:
        if d['seed_source_identity'].get('freeze_sha')!=digest:raise ValueError('Wrong real confirmation identity')
    coverage=all(d['coverage_status']=='SUFFICIENT_COVERAGE' for d in conf)
    improved=sum(d['qualifying_improvement'] for d in conf)
    raw_ok=all(d['raw_non_degradation'] for d in conf)
    screening=all(d['pass'] for d in seeds.values());outputs=coverage and improved>=2 and raw_ok
    status='BLOCKED_CORRECTNESS' if not engineering else ('GOAL_ACHIEVED' if screening and outputs else 'COMPLETED_WITH_LIMITATIONS')
    return {'status':status,'screening':'MET' if screening else 'NOT_MET','output_quality':'MET' if outputs else 'NOT_MET',
            'passive_engineering':'PASS' if engineering else 'FAIL','confirmation_coverage_all':coverage,
            'confirmation_qualifying_sequences':improved,'confirmation_raw_non_degradation_all':raw_ok,
            'seeds':{str(k):v for k,v in seeds.items()},'budget':budget.usage(),'freeze_sha':digest}

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--manifest',required=True,type=Path);a=p.parse_args()
    print(json.dumps(assess(a.manifest),indent=2))
