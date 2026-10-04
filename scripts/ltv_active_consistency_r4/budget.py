"""New-task budget adapter over the previously tested locked scheduler.

No previous ledger or output is modified. Reserve final verification capacity
until a version is explicitly frozen; all retries are new paid attempts.
"""
import importlib.util
import json
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[2]
DOC=ROOT/'docs/ltv/active_landmark_consistency_r4'
OUT=Path('/home/he/output/ltv_active_landmark_consistency_r4')
LIMITS={'synthetic':12,'real':100,'short_real':16,'geometry':29998,'candidate':4}
FINAL_RESERVE={'real':29,'synthetic':8}
spec=importlib.util.spec_from_file_location('_verified_scheduler',ROOT/'scripts/ltv_feature_passive/budget.py')
base=importlib.util.module_from_spec(spec);spec.loader.exec_module(base)
base.DOC=DOC;base.OUT=OUT;base.LIMITS=LIMITS
old_source=base.source_identity

def source_identity():
    result=old_source()
    for p in sorted((ROOT/'scripts/ltv_active_consistency_r4').rglob('*')):
        if p.is_file() and p.suffix in ('.py','.cpp','.h','.json','.sh'):
            result[str(p.relative_to(ROOT))]=base.hashlib.sha256(p.read_bytes()).hexdigest()
    for p in sorted((ROOT/'scripts/ltv_passive_hardening').rglob('*')):
        if p.is_file() and p.suffix in ('.py','.cpp','.h','.json','.sh'):
            result[str(p.relative_to(ROOT))]=base.hashlib.sha256(p.read_bytes()).hexdigest()
    # Read-only synthetic generators import the standalone truth/geometry helpers.
    for p in sorted((ROOT/'scripts/ltv_standalone_study').rglob('*')):
        if p.is_file() and p.suffix in ('.py','.cpp','.h','.json','.sh'):
            result[str(p.relative_to(ROOT))]=base.hashlib.sha256(p.read_bytes()).hexdigest()
    for relative in ('docs/ltv/passive_hardening_v2/protocol.json',
                     'docs/ltv/passive_hardening_v2/acceptance.json'):
        result[relative]=base.hashlib.sha256((ROOT/relative).read_bytes()).hexdigest()
    return result
base.source_identity=source_identity
process_identity=base.process_identity
process_alive=base.process_alive

def events(out=OUT):return base.events(out)
def usage(out=OUT):return base.usage(out)
def locked(out=OUT):return base.locked(out)
def append(record,out=OUT):return base.append(record,out)

def reserve(kind,command,identity,units=1,input_seconds=None,out=OUT):
    if kind not in (*LIMITS,'build','test','analysis') or units<1:
        raise ValueError('Invalid attempt kind/units')
    if kind=='short_real' and (input_seconds is None or not 0<input_seconds<=10):
        raise ValueError('Short real input must be capped at ten seconds')
    phase=identity.get('phase','development') if isinstance(identity,dict) else 'development'
    with locked(out):
        used=usage(out)
        ceiling=LIMITS.get(kind)
        if phase!='confirmation' and kind in ('real','synthetic'):
            ceiling=LIMITS[kind]-FINAL_RESERVE[kind]
        if ceiling is not None and used[kind]+units>ceiling:
            raise RuntimeError('BUDGET_EXHAUSTED_OR_FINAL_RESERVE: '+kind)
        es=events(out);run_id=sum(e['event']=='reserved' for e in es)+1
        append({'event':'reserved','id':run_id,'kind':kind,'units':units,'identity':identity,
                'command':command,'input_seconds':input_seconds,'time':base.time.time()},out)
    return run_id
base.reserve=reserve

def run(kind,command,identity,units=1,input_seconds=None,out=OUT,phase='development'):
    if phase not in ('development','confirmation','engineering'):
        raise ValueError('Explicit legal execution phase required')
    if phase=='confirmation':
        frozen=DOC/'frozen_config.json'
        if not frozen.is_file():raise RuntimeError('Final confirmation requires frozen version')
    return base.run(kind,command,{'phase':phase,'details':identity},units,input_seconds,out)

if __name__=='__main__':
    import argparse
    argv=sys.argv[1:];split=argv.index('--') if '--' in argv else len(argv)
    p=argparse.ArgumentParser();p.add_argument('kind',choices=[*LIMITS,'build','test','analysis','status']);p.add_argument('--identity',default='');p.add_argument('--phase',default='development',choices=['development','confirmation','engineering']);p.add_argument('--units',type=int,default=1);p.add_argument('--input-seconds',type=float);a=p.parse_args(argv[:split])
    if a.kind=='status':print(json.dumps(usage(),indent=2))
    else:
        command=argv[split+1:]
        if not command or not a.identity:p.error('command and identity required')
        r=run(a.kind,command,a.identity,a.units,a.input_seconds,phase=a.phase);print(json.dumps(r));raise SystemExit(r['exit_code'])
