#!/usr/bin/env python3
"""Six authorized closeout runs; immutable prior evidence, shared 64-run ledger."""
from run_study import DEFAULT, DOC, ROOT, HERE, config, code_identity, digest, write, frozen_check, run, ledger
import argparse
import fcntl
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time
import numpy as np

DEST = DOC / 'closeout_review'
SPEC = ROOT / 'docs/04-LTV_v2_六项补充验证_Codex执行方案.md'
MATRIX = {
    'E1':config('SPLIT_REF','PAPER'),
    'E2':config('CORE','PAPER',duration=30),
    'E3':config('EXPERIMENTAL','REGULAR','C2',lifetime=1),
    'E4':config('EXPERIMENTAL','FAST','C2',lifetime=1),
    'E5':config('EXPERIMENTAL','PAPER',lifetime=1),
    'E6':config('EXPERIMENTAL','PAPER',lifetime=5),
}


def file_sha(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as f:
        for block in iter(lambda:f.read(4*1024*1024),b''):h.update(block)
    return h.hexdigest()


def snapshot_paths(out, initial):
    paths=set()
    paths.update(p for p in DOC.rglob('*') if p.is_file() and DEST not in p.parents)
    paths.update(ROOT/k for k in code_identity())
    paths.update([out/'core.so',out/'experimental.so',out/'environment.json',out/'verification.json',out/'selection.json',out/'selection_contract.json'])
    for e in initial:
        paths.update(p for p in Path(e['directory']).iterdir() if p.is_file())
        m=json.loads((Path(e['directory'])/'metrics.json').read_text())
        paths.add(Path(m['input_path']));paths.add(Path(m['input_path']).with_suffix('.json'))
    paths.add(SPEC)
    return sorted(paths)


def audit(out):
    DEST.mkdir(parents=True,exist_ok=True); extra=out/'closeout_review';extra.mkdir(exist_ok=True)
    original=DEST/'evidence/history_ledger.json'
    if not original.exists():
        entries=ledger(out)
        if len(entries)!=58 or not all(e['status']=='COMPLETED' for e in entries):
            raise RuntimeError('Expected reviewed 58-run starting ledger; inspect existing supplements before reserving anything')
        write(original,entries)
        write(DEST/'protocol.json',{'spec':str(SPEC),'spec_sha256':file_sha(SPEC),'starting_head':subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),'starting_runs':58,'maximum_total_attempts':64,'maximum_new_attempts':6,'matrix':MATRIX,'E3_E4_role':'EXPLORATORY_GAIN_TRADEOFF','historical_C_star':'C6','initialization':'ZERO, no mean writes','concurrency':'serial full runs, no overlapping short tests; maximum two authorized','output':str(out),'covariance_and_settling_contract':'unchanged v2','native_core_defaults':{'max_features':30,'min_features':15,'max_missed_frames':2,'warmup_camera_updates':20},'historical_ledger_policy':'append only; first 58 records immutable'})
        write(DEST/'evidence/history_sha256.json',{str(p):file_sha(p) for p in snapshot_paths(out,entries)})
    initial=json.loads(original.read_text());current=ledger(out)
    assert current[:58]==initial and len(current)<=64
    for e in current[58:]:
        if e['status']=='RUNNING':
            raise RuntimeError('Unresolved RUNNING attempt: inspect child PID/exit evidence before resuming; never remove its budget reservation')
    import platform,scipy
    environment={'python':platform.python_version(),'numpy':np.__version__,'scipy':scipy.__version__,'compiler':subprocess.check_output(['g++','--version'],text=True).splitlines()[0],'blas_threads':1}
    assert json.loads((out/'environment.json').read_text())==environment
    assert json.loads((out/'verification.json').read_text())['code']==code_identity()
    for e in initial:
        rd=Path(e['directory']);assert json.loads((rd/'code.json').read_text())==code_identity()
        assert json.loads((rd/'config.json').read_text())==e['config']
    expected=json.loads((DEST/'evidence/history_sha256.json').read_text())
    changed=[p for p,h in expected.items() if not Path(p).is_file() or file_sha(p)!=h]
    if changed:raise RuntimeError('Historical evidence changed: '+repr(changed))
    write(DEST/'evidence/integrity.json',{'historical_files_checked':len(expected),'historical_changed':changed,'production':frozen_check(),'ledger_prefix_equal':True,'total_attempts':len(current)})
    print('AUDIT PASS:',len(current),'total attempts;',len(expected),'historical files unchanged',flush=True)


def short_tests(out):
    from tests import verify
    from core import Core,Lifecycle
    from truth_model import sample,geometry,ids_at
    checks=verify(out)
    a=Core(out/'core.so',1e-4,1e6,1);b=Core(out/'experimental.so',1e-4,1e6,1)
    lc=Lifecycle(1);_,Rbc,pc=geometry('PAPER');counts=[]
    try:
        for tick in range(251):
            t=tick/200
            if tick:
                _,acc,w,_=sample(t-.0025,'PAPER');a.predict(.005,acc,w);b.predict(.005,acc,w)
            if tick%10==0:
                ids=ids_at(tick//10,20,1,16);_,_,_,z=sample(t,'PAPER')
                lc.camera(ids);a.camera(t,ids,z,Rbc,pc);b.camera(t,ids,z,Rbc,pc)
            ax,ap,ai=a.read();bx,bp,bi=b.read()
            assert ai==bi==lc.ids and np.array_equal(ax,bx) and np.array_equal(ap,bp)
            assert len(ax)==3*len(ai)+6;counts.append(len(ai))
    finally:a.close();b.close()
    assert max(counts)>16
    checks['PAPER_dynamic_hook_off_slot_P_parity']={'pass':True,'min_slots':min(counts),'max_slots':max(counts),'duration':1.25,'full_simulation':False}
    write(DEST/'evidence/short_tests.json',checks)
    print('SHORT TESTS PASS',flush=True)


def find(out,c):
    matches=[e for e in ledger(out) if e['config']==c and e['status']=='COMPLETED']
    if len(matches)!=1:raise RuntimeError('Missing or ambiguous successful identity: '+str(c))
    return matches[0]


def prefix_check(out,new):
    old=find(out,config('CORE','PAPER'));checks={}
    def arrays(e,name):
        with np.load(Path(e['directory'])/name) as z:return {k:z[k] for k in z.files}
    for name,keys in [('trace.npz',['t','x','ids','values','landmarks','axes']),('camera.npz',['t','side','x','ids','values','landmarks']),('matrices.npz',['t','tags','P','x','ids','eigen','symmetry'])]:
        a,b=arrays(old,name),arrays(new,name);mask=b['t']<=20
        for key in keys:
            av,bv=a[key],b[key][mask]
            checks[name+'/'+key]=bool(np.array_equal(av,bv,equal_nan=True)) if av.dtype.kind not in 'USO' else bool(np.array_equal(av,bv))
        if name=='trace.npz': checks['discrete_events']=bool(np.array_equal(a['diag'][:,:2],b['diag'][mask,:2]))
    am=json.loads((Path(old['directory'])/'metrics.json').read_text());bm=json.loads((Path(new['directory'])/'metrics.json').read_text())
    with np.load(am['input_path']) as a,np.load(bm['input_path']) as b:
        for key in ['imu','bearings']:checks['input/'+key]=bool(np.array_equal(a[key],b[key][:len(a[key])]))
    result={'old':old['id'],'new':new['id'],'checks':checks,'pass':all(checks.values()),'excluded':'wall-clock costs; unused pose-prior arrays in ZERO mode; file headers differ with horizon'}
    write(DEST/'evidence/E2_prefix.json',result)
    if not result['pass']:raise RuntimeError('BLOCKED_REFERENCE: E2 prefix mismatch')
    print('E2 PREFIX PASS',flush=True)


def recover_interrupted(out):
    entries=ledger(out);changed=False
    for e in entries[58:]:
        if e['status']!='RUNNING':continue
        pid=e.get('child_pid');cmdline=Path(f'/proc/{pid}/cmdline')
        if pid and cmdline.exists() and str(Path(e['directory'])/'config.json').encode() in cmdline.read_bytes():
            raise RuntimeError('Previous simulation child still running; recovery will not overlap it')
        e['status']='INTERRUPTED';e['exit_code']=None
        e['recovery_note']='Driver lost completion status; attempt remains counted and is not reused'
        changed=True
    if changed:write(out/'ledger.json',entries)


def execute_all(out):
    recover_interrupted(out)
    audit(out)
    if not (DEST/'evidence/short_tests.json').exists():short_tests(out)
    mapping=json.loads((DEST/'evidence/experiment_map.json').read_text()) if (DEST/'evidence/experiment_map.json').exists() else {}
    for label,c in MATRIX.items():
        if label=='E3':
            from closeout_analysis import gain_fixed
            write(DEST/'evidence/fixed_gain_tradeoff.json',gain_fixed(out))
        matches=[e for e in ledger(out) if e['config']==c and e['status']=='COMPLETED']
        if not matches and len(ledger(out))>=64:break
        e=run(out,'diagnostic',c)
        mapping[label]={'run':e['id'],'role':'EXPLORATORY_GAIN_TRADEOFF' if label in ['E3','E4'] else 'SUPPLEMENTAL_DIAGNOSTIC','identity':e['identity']}
        write(DEST/'evidence/experiment_map.json',mapping)
        if label=='E2':prefix_check(out,e)
        print(label,'PASS',flush=True)
    audit(out)


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('command',choices=['audit','run','report','all']);parser.add_argument('--out',type=Path,default=DEFAULT);args=parser.parse_args();out=args.out.resolve()
    with (out/'driver.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        try:
            if args.command=='audit':audit(out)
            if args.command in ['run','all']:execute_all(out)
            if args.command in ['report','all']:
                from closeout_analysis import report
                report(out,DEST)
                audit(out)
        except Exception as error:
            write(DEST/'checkpoint.json',{'error':str(error),'total_attempts':len(ledger(out)),'remaining':64-len(ledger(out)),'time':time.time()})
            raise

if __name__=='__main__':main()
