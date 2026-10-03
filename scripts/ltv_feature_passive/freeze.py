"""Freeze one fully evaluated development method; never replace an existing freeze."""
import argparse
import hashlib
import json
from pathlib import Path
import budget
import artifacts
from real_run import mode_config
from prepare import prepare

FROZEN=budget.DOC/'frozen_config.json'

def check():
    data=json.loads(FROZEN.read_text())
    current=budget.source_identity()
    for name,digest in data['source_sha256'].items():
        if current.get(name)!=digest:raise RuntimeError('Source changed after freeze: '+name)
    for name,digest in data['artifacts_sha256'].items():
        if artifacts.sha(name)!=digest:raise RuntimeError('Frozen artifact changed: '+name)
    artifacts.verify(data['runtime'])
    return data,artifacts.sha(FROZEN)

def freeze(evidence_path):
    if FROZEN.exists():raise RuntimeError('Freeze exists; never silently replace after confirmation')
    evidence_path=Path(evidence_path);evidence=json.loads(evidence_path.read_text())
    if evidence['method']!='SEED_HYBRID' or evidence['source']!='STEREO_THEN_TEMPORAL':
        raise ValueError('Only explicitly registered candidate5 supported by this freeze dispatcher')
    prepare();protocol=json.loads((budget.DOC/'protocol.json').read_text())
    if set(evidence['real_metrics'])!=set(protocol['real_development']):raise ValueError('Need all three development evaluations')
    runtime=artifacts.snapshot()
    final_library_sha=artifacts.sha(budget.OUT/'synthetic_hybrid.so')
    paths=[evidence_path,budget.DOC/'goal.md',budget.DOC/'protocol.json',budget.DOC/'acceptance.json']
    for seq,path in evidence['real_metrics'].items():
        d=json.loads(Path(path).read_text())
        if not d['engineering']['pass']:raise ValueError('Development bypass failed: '+seq)
        if d['sequence']!=seq or d['seed_source_identity']['source']!=evidence['source']:
            raise ValueError('Wrong selected real source')
        if d['seed_source_identity'].get('runtime')!=str(runtime):raise ValueError('Development used another runtime')
        paths.append(Path(path))
    if not evidence['synthetic_metrics']:raise ValueError('Need synthetic development evidence')
    for path in evidence['synthetic_metrics']:
        d=json.loads(Path(path).read_text())
        if d['method']!='SEED_HYBRID' or not d['diagnostics']['full_input_consumed']:raise ValueError('Incomplete synthetic development')
        run_path=Path(path).parent/'run.json'
        run=json.loads(run_path.read_text())
        if run['library_sha']!=final_library_sha or run['runner_sha']!=artifacts.sha(Path(__file__).parent/'synthetic/run.py'):
            raise ValueError('Development used another numerical wrapper/library')
        paths.extend([Path(path),run_path])
    completed={e['id']:e for e in budget.events() if e['event']=='finished'}
    groups={'geometry_uncertainty','causality_lifecycle','controlled_seed','old_off_equivalence',
            'pose_convention','timing_options','cache_equivalence','passive_bypass','resource_recovery','hybrid_route','evaluation_criteria'}
    if set(evidence['engineering_tests'])!=groups:raise ValueError('Missing mandatory test group')
    reserved={e['id']:e for e in budget.events() if e['event']=='reserved'}
    for group,ids in evidence['engineering_tests'].items():
        if not ids:raise ValueError('No test evidence for '+group)
        for run_id in ids:
            if completed.get(run_id,{}).get('exit_code')!=0:raise ValueError('Unpassed required test attempt')
            if reserved[run_id]['kind'] not in ('test','analysis','short_real','real'):raise ValueError('A build is not a test')
            for name in ('source_sha256.json','command_files_sha256.json','stdout.log','stderr.log'):
                paths.append(budget.OUT/'attempts'/f'{run_id:04d}'/name)
    configs={}
    for family in ('ltv_euroc','uzhfpv_indoor'):
        for mode in protocol['real_modes']:
            p=mode_config(evidence['source'],family,mode)
            configs[family+'/'+mode]=str(p)
            paths.extend(x for x in p.parent.rglob('*') if x.is_file())
    library=budget.OUT/'synthetic_hybrid.so'
    frozen_library=budget.OUT/'binaries'/('synthetic_'+artifacts.sha(library)+'.so')
    if not frozen_library.exists():raise ValueError('Final synthetic library was not used in development')
    paths.extend([frozen_library,runtime/'manifest.json'])
    data={'method':evidence['method'],'source':evidence['source'],'risk':.05,
          'manager':protocol['initial_manager'],'runtime':str(runtime),'synthetic_library':str(frozen_library),
          'configurations':configs,'source_sha256':budget.source_identity(),
          'artifacts_sha256':{str(p):artifacts.sha(p) for p in paths},
          'development_evidence':evidence,'budget_at_freeze':budget.usage(),
          'policy':'No outcome-based tuning; GT offline only; G/V injections OFF; risk score is not calibrated probability.'}
    with FROZEN.open('x') as f:f.write(json.dumps(data,indent=2)+'\n')
    return {'freeze':str(FROZEN),'sha':artifacts.sha(FROZEN)}

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--evidence',type=Path);p.add_argument('--check',action='store_true');a=p.parse_args()
    if a.check:print(json.dumps({'freeze_sha':check()[1]}))
    elif a.evidence:print(json.dumps(freeze(a.evidence)))
    else:p.error('--evidence or --check required')
