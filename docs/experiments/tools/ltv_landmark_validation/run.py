"""Independent append-only study ledger; caches and failures are retained."""
import argparse
import hashlib
import fcntl
import json
import os
from pathlib import Path
import subprocess
import time

ROOT = Path(__file__).resolve().parents[4]
ENV = {k: v for k, v in os.environ.items() if k not in ('LD_LIBRARY_PATH', 'PYTHONPATH', 'AMENT_PREFIX_PATH', 'CMAKE_PREFIX_PATH', 'COLCON_PREFIX_PATH')}
ENV.update(OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1', MKL_NUM_THREADS='1', NUMEXPR_NUM_THREADS='1')


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for data in iter(lambda: stream.read(4 * 1024 * 1024), b''):
            h.update(data)
    return h.hexdigest()


def write(path, data):
    Path(path).write_text(json.dumps(data, indent=2, allow_nan=False) + '\n')


def _append(out, data):
    with (out / 'attempts.jsonl').open('a') as stream:
        stream.write(json.dumps(data, allow_nan=False) + '\n')
        stream.flush()


def command(out, name, args):
    return ['bash', '--noprofile', '--norc', '-c',
            'source /opt/ros/humble/setup.bash && source "$1/setup.bash" && exec "$2" "${@:3}"',
            'landmark', str(out / 'install'), str(out / 'install/ov_msckf/lib/ov_msckf' / name), *map(str, args)]


def run(out, label, kind, cmd, cwd=None):
    handles = []
    slot = None
    for i in range(2):
        handle=(out/f'compute_{i}.lock').open('a')
        handles.append(handle)
        try:
            fcntl.flock(handle,fcntl.LOCK_EX|fcntl.LOCK_NB)
            slot=handle
            break
        except BlockingIOError:
            continue
    if slot is None:
        slot=handles[0]
        fcntl.flock(slot,fcntl.LOCK_EX)
    with (out/'attempts.lock').open('a') as ledger_lock:
        fcntl.flock(ledger_lock,fcntl.LOCK_EX)
        attempts=[json.loads(s) for s in (out/'attempts.jsonl').read_text().splitlines()] if (out/'attempts.jsonl').exists() else []
        if kind=='real' and sum(e['event']=='start' and e['kind']=='real' for e in attempts)>=12:
            raise RuntimeError('Full real replay budget exhausted')
        index=sum(e['event']=='start' for e in attempts)+1
        record=dict(event='start',id=index,label=label,kind=kind,command=cmd,cwd=str(cwd or ROOT),started=time.time())
        _append(out,record)
    resource_path = out / f'{index:03d}_{label}_resources.json'
    measured = ['/usr/bin/time', '-q', '-f', '{"max_rss_kb":%M,"elapsed_s":%e,"user_s":%U,"system_s":%S}', '-o', str(resource_path), *cmd]
    with (out / f'{index:03d}_{label}.log').open('w') as stream:
        result = subprocess.run(measured, env=ENV, cwd=cwd or ROOT, stdout=stream, stderr=subprocess.STDOUT, pass_fds=(slot.fileno(),))
    record.update(event='finish', exit_code=result.returncode, seconds=time.time()-record['started'], resource_path=str(resource_path))
    with (out/'attempts.lock').open('a') as ledger_lock:
        fcntl.flock(ledger_lock,fcntl.LOCK_EX)
        _append(out,record)
    for handle in handles:
        handle.close()
    print(label, result.returncode, record['seconds'], flush=True)
    return result.returncode


def collect(out, past_geometry=False):
    frozen = json.loads((out / 'freeze.json').read_text())
    binary = out / 'install/ov_msckf/lib/ov_msckf/replay_ltv_landmark_shadow'
    dest=out/'past_geometry_audit' if past_geometry else out
    dest.mkdir(exist_ok=True)
    for seq, inp in frozen['inputs'].items():
        old = Path(inp['off_path'])
        for name, digest in inp['off_evidence_sha'].items():
            assert sha(old / name) == digest, (seq, name)
        for name, digest in inp['config_sha'].items():
            assert sha(Path(inp['config']).parent / name) == digest
        args = [inp['config'], old / 'cache.bin', dest / f'{seq}_predictions.csv', dest / f'{seq}_exact.json']
        if run(out, seq+('_past_geometry' if past_geometry else '_shadow'), 'cache', command(out, binary.name, args+(['PAST_GEOMETRY'] if past_geometry else []))):
            raise SystemExit('Shadow failed; retain failure and diagnose')
        write(dest / f'{seq}_identity.json', dict(binary_sha=sha(binary), config=inp, predictions_sha=sha(args[2]), exact=json.loads(args[3].read_text())))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('out', type=Path)
    parser.add_argument('action', choices=['build', 'collect', 'check_install', 'off_parity', 'collect_past'])
    args = parser.parse_args()
    out = args.out.resolve()
    if args.action in ('collect','collect_past'):
        collect(out,args.action=='collect_past')
    elif args.action == 'off_parity':
        sys_path = str(ROOT/'docs/experiments/tools/ltv_integration_cleanup')
        import sys
        sys.path.insert(0, sys_path)
        import check_real_outputs as exact
        frozen = json.loads((out/'freeze.json').read_text())
        (out/'runs').mkdir(exist_ok=True)
        for seq, inp in frozen['inputs'].items():
            dest=out/'runs'/f'{seq}_OFF_01'
            dest.mkdir(exist_ok=False)
            code=run(out,seq+'_OFF','real',command(out,'run_ltv_gv_evaluation',[inp['config'],inp['root'],dest,'OFF']),cwd=dest)
            if code:
                raise SystemExit('OFF replay failed; preserve evidence')
            parity=exact.compare(inp['off_path'],dest)
            parity['ltv_rows']=exact.csv_exact(Path(inp['off_path'])/'ltv.csv',dest/'ltv.csv')
            parity['matrices_sha']=sha(dest/'matrices.jsonl')
            for name in ['fusion.jsonl','matrices.jsonl']:
                assert sha(dest/name)==inp['off_evidence_sha'][name]
            write(out/f'{seq}_off_parity.json',parity)
    elif args.action == 'build':
        raise SystemExit(run(out, 'build', 'build', ['bash', str(out/'build.sh')]))
    else:
        raise SystemExit(run(out, 'check_install', 'test', ['python3', str(ROOT/'docs/experiments/tools/ltv_gv_evaluation/verify_install.py'), str(out)]))
