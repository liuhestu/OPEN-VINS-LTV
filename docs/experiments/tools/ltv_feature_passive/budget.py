"""Main-owned append-only attempt ledger and two-heavy-task scheduler.

Reservation is durable before child launch. Failed launch/retry consumes budget.
A lost parent leaves RUNNING for explicit PID reconciliation, never auto-reuse.
"""
import contextlib
import fcntl
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time

ROOT = Path(__file__).resolve().parents[4]
DOC = ROOT / 'docs/ltv/feature_readiness_passive'
OUT = Path('/home/he/output/ltv_feature_readiness_passive')
LIMITS = {'synthetic':48, 'real':36, 'short_real':16, 'geometry':25000, 'candidate':6}

def process_identity(pid):
    """Linux PID birth identity; prevents confusing a recycled PID with our child."""
    try:
        fields=Path(f'/proc/{pid}/stat').read_text().rsplit(')',1)[1].split()
        return {'pid':pid,'start_ticks':fields[19],
                'boot_id':Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
                'state':fields[0]}
    except FileNotFoundError:
        return None

def process_alive(saved):
    if not saved:return False
    current=process_identity(saved['pid'])
    return bool(current and current['state']!='Z' and
                all(current[k]==saved[k] for k in ('pid','start_ticks','boot_id')))

def source_identity():
    result = {}
    for folder in ['ov_core/src', 'ov_init/src', 'ov_msckf/src', 'ov_msckf/tests/ltv',
                   'docs/experiments/tools/ltv_feature_passive', 'ov_msckf/cmake']:
        for p in sorted((ROOT / folder).rglob('*')):
            if p.is_file() and p.suffix in {'.cpp', '.h', '.py', '.cmake', '.txt'}:
                result[str(p.relative_to(ROOT))] = hashlib.sha256(p.read_bytes()).hexdigest()
    for name in ['protocol.json', 'acceptance.json']:
        p = DOC / name
        result[str(p.relative_to(ROOT))] = hashlib.sha256(p.read_bytes()).hexdigest()
    return result

@contextlib.contextmanager
def locked(out=OUT):
    out.mkdir(parents=True, exist_ok=True)
    with (out / 'budget.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        yield

def events(out=OUT):
    p = out / 'attempts.jsonl'
    return [json.loads(line) for line in p.read_text().splitlines()] if p.exists() else []

def append(record, out=OUT):
    with (out / 'attempts.jsonl').open('a') as f:
        f.write(json.dumps(record, sort_keys=True) + '\n'); f.flush(); os.fsync(f.fileno())

def usage(out=OUT):
    result = dict.fromkeys(LIMITS, 0)
    for e in events(out):
        if e['event'] == 'reserved' and e['kind'] in result:
            result[e['kind']] += e['units']
    return result

def reserve(kind, command, identity, units=1, input_seconds=None, out=OUT):
    if kind not in (*LIMITS, 'build', 'test', 'analysis') or units < 1:
        raise ValueError('Invalid attempt kind/units')
    if kind == 'short_real' and (input_seconds is None or not 0 < input_seconds <= 10):
        raise ValueError('Short real input must be capped at ten seconds')
    with locked(out):
        used = usage(out)
        if kind in LIMITS and used[kind] + units > LIMITS[kind]:
            raise RuntimeError('BUDGET_EXHAUSTED: ' + kind)
        es=events(out); run_id=sum(e['event']=='reserved' for e in es)+1
        append({'event':'reserved','id':run_id,'kind':kind,'units':units,'identity':identity,
                'command':command,'input_seconds':input_seconds,'time':time.time()}, out)
    return run_id

def run(kind, command, identity, units=1, input_seconds=None, out=OUT):
    # Slot lock lifetime spans launch and completion. No third heavy child can launch.
    out.mkdir(parents=True, exist_ok=True)
    slot=None
    for i in range(2):
        f=(out/f'heavy_{i}.lock').open('a')
        try: fcntl.flock(f, fcntl.LOCK_EX | fcntl.LOCK_NB); slot=f; break
        except BlockingIOError: f.close()
    if slot is None: raise RuntimeError('Both heavy slots occupied; wait for a live handle')
    try:
        run_id=reserve(kind,command,identity,units,input_seconds,out)
        rd=out/'attempts'/f'{run_id:04d}';rd.mkdir(parents=True)
        (rd/'source_sha256.json').write_text(json.dumps(source_identity(), indent=2))
        files={}
        for arg in command:
            p=Path(arg)
            if p.is_file():
                files[str(p.resolve())]=hashlib.sha256(p.read_bytes()).hexdigest()
        (rd/'command_files_sha256.json').write_text(json.dumps(files,indent=2))
        env=dict(os.environ, OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1', MKL_NUM_THREADS='1', NUMEXPR_NUM_THREADS='1')
        started=time.time(); rc=None
        try:
            with (rd/'stdout.log').open('w') as stdout, (rd/'stderr.log').open('w') as stderr:
                child=subprocess.Popen(command,cwd=ROOT,env=env,stdout=stdout,stderr=stderr,pass_fds=(slot.fileno(),))
                with locked(out): append({'event':'started','id':run_id,'pid':child.pid,
                                          'process':process_identity(child.pid),
                                          'owner':process_identity(os.getpid()),'time':time.time()},out)
                rc=child.wait()
        except Exception as error:
            with locked(out): append({'event':'finished','id':run_id,'exit_code':rc,'error':repr(error),'time':time.time()},out)
            raise
        with locked(out): append({'event':'finished','id':run_id,'exit_code':rc,'seconds':time.time()-started,'time':time.time()},out)
        return {'id':run_id,'exit_code':rc,'directory':str(rd)}
    finally:
        slot.close()

if __name__ == '__main__':
    import argparse
    import sys
    argv=sys.argv[1:]
    split=argv.index('--') if '--' in argv else len(argv)
    command=argv[split+1:]
    p=argparse.ArgumentParser();p.add_argument('kind', choices=[*LIMITS,'build','test','analysis','status']);p.add_argument('--identity',default='');p.add_argument('--units',type=int,default=1);p.add_argument('--input-seconds',type=float);a=p.parse_args(argv[:split])
    if a.kind=='status': print(json.dumps(usage(),indent=2))
    else:
        if not command or not a.identity: p.error('Explicit command and identity required')
        result=run(a.kind,command,a.identity,a.units,a.input_seconds);print(json.dumps(result));raise SystemExit(result['exit_code'])
