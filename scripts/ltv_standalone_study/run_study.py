#!/usr/bin/env python3
"""Resumable study driver. Ledger reservations precede ALL full simulation attempts."""
import os
for key in ['OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS','NUMEXPR_NUM_THREADS','VECLIB_MAXIMUM_THREADS']:
    os.environ[key]='1'
import argparse,json,hashlib,subprocess,sys,time,traceback,fcntl,difflib,threading
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import numpy as np
HERE=Path(__file__).resolve().parent; ROOT=HERE.parents[1]
DOC=ROOT/'docs/ltv/standalone_convergence_v2'
DEFAULT=Path('/home/he/output/ltv_standalone_convergence_v2')
CANDIDATES={'C0':(1,1,1),'C1':(.1,1,1),'C2':(10,1,1),'C3':(1,.1,1),'C4':(1,10,1),'C5':(1,1,100),'C6':(1,1,10000)}
LEDGER_LOCK=threading.Lock()
CAPS={'fixed':14,'discretization':4,'gains':19,'lifecycle':11,'initialization':8,'diagnostic':8}
def digest(data): return hashlib.sha256(data).hexdigest()
def write(path,obj):
    path.parent.mkdir(parents=True,exist_ok=True); tmp=path.with_suffix('.tmp'); tmp.write_text(json.dumps(obj,indent=2,allow_nan=False)); tmp.replace(path)
def code_identity():
    names=['truth_model.py','continuous_reference.py','split_reference.py','core.py','geometry_seed.py','experiment.py','evaluate.py','core_runner.cpp']
    files=[HERE/n for n in names]+list((HERE/'experimental_core').glob('*'))+[ROOT/'ov_msckf/src/ltv'/n for n in ['ltv_observer.h','ltv_observer.cpp','ltv_types.h']]+[ROOT/'ov_core/src/utils/quat_ops.h']
    return {str(p.relative_to(ROOT)):digest(p.read_bytes()) for p in files}
def frozen_check():
    freeze=json.loads((DOC/'evidence/frozen_manifest.json').read_text())
    changed=[p for p,h in freeze['sha256'].items() if not (ROOT/p).exists() or digest((ROOT/p).read_bytes())!=h]
    if changed: raise RuntimeError('Frozen files modified: '+repr(changed))
    return {'checked':len(freeze['sha256']),'changed':changed,'head':freeze['head']}
def freeze(out):
    check=frozen_check(); write(DOC/'evidence/integrity.json',check)
    protocol={'baseline':check['head'],'output':str(out),'maximum_runs':64,'maximum_simulators':2,'blas_threads':1,'stage_caps':CAPS,'candidates':CANDIDATES,'solver':{'method':'DOP853','rtol':1e-9,'atol':1e-11,'max_step':.005,'tight_rtol':1e-11,'tight_atol':1e-13,'tight_max_step':.0025,'P_scale':1},'reference_state_and_full_P_tolerance':1e-6,'algebra_tolerance':1e-8,'spectrum_relative_roundoff':1e-10,'selection':'wide t_VG at .005 s resolution, otherwise 50-60 s J_tail; continuous nondegradation and discrete benefit required; peaks may increase','input_boundary':'truth -> sensor arrays; only labelled initialization uses pose/point priors','commands':['freeze','verify','fixed','discretization','gains','lifecycle','initialization','report'],'execution_spec_sha':digest(next((ROOT/'docs').glob('03_LTV*')).read_bytes())}
    write(DOC/'protocol.json',protocol)
    patch=''
    for name in ['ltv_observer.h','ltv_observer.cpp','ltv_types.h']:
        patch+=''.join(difflib.unified_diff((ROOT/'ov_msckf/src/ltv'/name).read_text().splitlines(True),(HERE/'experimental_core'/name).read_text().splitlines(True),fromfile='production/'+name,tofile='experimental_core/'+name))
    (HERE/'experimental_core.patch').write_text(patch)
    return check

def build(out):
    for kind,path in [('core',ROOT/'ov_msckf/src/ltv'),('experimental',HERE/'experimental_core')]:
        cmd=['g++','-std=c++14','-O2','-DNDEBUG','-shared','-fPIC','-I'+str(path),'-I'+str(ROOT/'ov_core/src'),'-I/usr/include/eigen3',str(path/'ltv_observer.cpp'),str(HERE/'core_runner.cpp'),'-o',str(out/(kind+'.so'))]
        if kind=='experimental': cmd.insert(1,'-DEXPERIMENTAL')
        result=subprocess.run(cmd,capture_output=True,text=True)
        write(out/(kind+'_build.json'),{'command':cmd,'exit_code':result.returncode,'stdout':result.stdout,'stderr':result.stderr})
        if result.returncode: raise RuntimeError(result.stderr)

def config(impl='CORE',scene='REGULAR',candidate='C0',**kw):
    qm,vm,pm=CANDIDATES[candidate]
    c={'impl':impl,'scene':scene,'candidate':candidate,'q':1e-4*qm,'v':1e6*vm,'p':pm,'imu_hz':200,'camera_hz':20,'duration':20 if scene=='PAPER' else 60,'tight':False,'lifetime':0,'initialization':'ZERO','noise':0}
    c.update(kw); return c

def ledger(out): return json.loads((out/'ledger.json').read_text()) if (out/'ledger.json').exists() else []
def run(out,stage,c):
    from experiment import execute
    codes=code_identity(); identity=digest(json.dumps({'config':c,'code':codes,'numpy':np.__version__},sort_keys=True).encode())
    with LEDGER_LOCK:
        entries=ledger(out)
        for e in entries:
            if e['identity']==identity and e['status']=='COMPLETED':
                return e
        if len(entries)>=64 or sum(e['stage']==stage for e in entries)>=CAPS[stage]: raise RuntimeError('BUDGET_EXHAUSTED')
        run_id=f'{len(entries)+1:02d}_{c["scene"]}_{c["impl"]}_{c["candidate"]}'
        rd=out/'runs'/run_id; rd.mkdir(parents=True,exist_ok=False)
        e={'id':run_id,'identity':identity,'config':c,'stage':stage,'directory':str(rd),'status':'RUNNING','started':time.time(),'command':[sys.executable,str(HERE/'run_study.py'),'--out',str(out),'worker','--config',str(rd/'config.json')]}
        write(rd/'config.json',c); write(rd/'code.json',codes); entries.append(e); write(out/'ledger.json',entries)
    print('START',run_id,json.dumps(c),flush=True)
    # The driver pool permits at most two child simulations; the ledger is serialized.
    with (rd/'stdout.log').open('w') as stdout,(rd/'stderr.log').open('w') as stderr:
        result=subprocess.Popen(e['command'],stdout=stdout,stderr=stderr)
        e['child_pid']=result.pid
        with LEDGER_LOCK:
            latest=ledger(out); latest=[e if item['id']==run_id else item for item in latest]; write(out/'ledger.json',latest)
        result.wait()
    e['exit_code']=result.returncode; e['finished']=time.time(); e['status']='COMPLETED' if result.returncode==0 else 'FAILED'; e['seconds']=e['finished']-e['started']
    with LEDGER_LOCK:
        latest=ledger(out); latest=[e if item['id']==run_id else item for item in latest]; write(out/'ledger.json',latest)
    if result.returncode: raise RuntimeError('simulation failed '+run_id+': '+(rd/'stderr.log').read_text()[-3000:])
    print('DONE',run_id,round(e['seconds'],2),flush=True)
    return e

def parallel_runs(out,stage,configs):
    # Two simulator children total; reservations and completions are serialized.
    with ThreadPoolExecutor(max_workers=2) as pool:
        futures=[pool.submit(run,out,stage,c) for c in configs]
        return [future.result() for future in futures]

def data(e,name='trace.npz'): return np.load(Path(e['directory'])/name)
def metric(e): return json.loads((Path(e['directory'])/'metrics.json').read_text())
def reference_check(a,b,out):
    x,y=data(a),data(b); assert np.array_equal(x['t'],y['t']) and np.array_equal(x['ids'],y['ids'])
    d=3*np.sum(x['ids'][0]>=0)+6
    state=float(np.max(np.linalg.norm(x['x'][:,:d]-y['x'][:,:d],axis=1)/np.maximum(1,np.linalg.norm(y['x'][:,:d],axis=1))))
    P=np.load(Path(a['directory'])/'reference_P.npy',mmap_mode='r'); Q=np.load(Path(b['directory'])/'reference_P.npy',mmap_mode='r')
    rel=0.
    for start in range(0,len(P),200):
        rel=max(rel,float(np.max(np.linalg.norm(P[start:start+200]-Q[start:start+200],axis=(1,2))/np.maximum(1,np.linalg.norm(Q[start:start+200],axis=(1,2))))))
    entry={'base':a['id'],'tight':b['id'],'state_max_normalized':state,'P_max_relative':rel,'pass':state<=1e-6 and rel<=1e-6}
    path=out/'reference_checks.json'; checks=json.loads(path.read_text()) if path.exists() else []; checks=[c for c in checks if c['base']!=a['id']]; checks.append(entry); write(path,checks)
    if not entry['pass']: raise RuntimeError('BLOCKED_REFERENCE: '+repr(entry))
    return entry

def fixed(out):
    for scene in ['REGULAR','FAST']:
        a=run(out,'fixed',config('CONT_REF',scene)); b=run(out,'fixed',config('CONT_REF',scene,tight=True)); reference_check(a,b,out)
        run(out,'fixed',config('SPLIT_REF',scene)); run(out,'fixed',config('CORE',scene))
    a=run(out,'fixed',config('CONT_REF','PAPER')); b=run(out,'fixed',config('CONT_REF','PAPER',tight=True)); reference_check(a,b,out); run(out,'fixed',config('CORE','PAPER'))
    run(out,'fixed',config('CONT_REF',duration=10,initialization='EXACT')); run(out,'fixed',config('EXPERIMENTAL',duration=10,initialization='EXACT'))
    run(out,'fixed',config('CONT_REF','STATIC'))
    # Full horizon hook-off parity counts as a diagnostic simulation.
    a=run(out,'fixed',config()); b=run(out,'diagnostic',config('EXPERIMENTAL'))
    x,y=data(a),data(b); p,q=data(a,'matrices.npz'),data(b,'matrices.npz')
    checks={'x':np.array_equal(x['x'],y['x'],equal_nan=True),'ids':np.array_equal(x['ids'],y['ids']),'P':np.array_equal(p['P'],q['P'],equal_nan=True),'discrete_events':np.array_equal(x['diag'][:,:2],y['diag'][:,:2])}
    write(out/'full_parity.json',checks)
    if not all(checks.values()): raise RuntimeError('BLOCKED_REFERENCE: hook-off parity')

def discretization(out):
    for ih,ch in [(1000,20),(200,100),(1000,100)]: run(out,'discretization',config(imu_hz=ih,camera_hz=ch))
    run(out,'discretization',config('SPLIT_REF',camera_hz=100))

def gains(out):
    entries={}
    jobs=[config(impl,candidate=candidate) for candidate in CANDIDATES for impl in ['CONT_REF','EXPERIMENTAL']]
    completed=parallel_runs(out,'gains',jobs)
    for e in completed: entries.setdefault(e['config']['candidate'],{})[e['config']['impl']]=e
    checks=json.loads((out/'reference_checks.json').read_text()); precision=max(c['state_max_normalized'] for c in checks)
    # Resolve scientific metrics by the actual C0 tightened discrepancy, never a post-hoc percentage band.
    base_ref=entries['C0']['CONT_REF']; tight=next(e for e in ledger(out) if e['config']==config('CONT_REF',tight=True) and e['status']=='COMPLETED')
    base_m=metric(base_ref); tight_m=metric(tight)
    j_resolution=max(abs(base_m['J_tail']-tight_m['J_tail']),np.finfo(float).eps)
    choices=[]; decisions={}
    def time_ok(new,old):
        if isinstance(old,str): return True
        return not isinstance(new,str) and new<=old+.005/2
    def better(new,old):
        tn,to=new['wide']['t_VG'],old['wide']['t_VG']
        if not isinstance(tn,str) and isinstance(to,str): return True
        if not isinstance(tn,str) and not isinstance(to,str) and tn<to-.005/2: return True
        if (isinstance(tn,str) and isinstance(to,str)) or (not isinstance(tn,str) and not isinstance(to,str) and abs(tn-to)<.005/2): return new['J_tail']<old['J_tail']-j_resolution
        return False
    for candidate in list(CANDIDATES)[1:]:
        cm=metric(entries[candidate]['CONT_REF']); dm=metric(entries[candidate]['EXPERIMENTAL']); bm=metric(entries['C0']['EXPERIMENTAL'])
        continuous_ok=time_ok(cm['wide']['t_VG'],base_m['wide']['t_VG']) and cm['J_tail']<=base_m['J_tail']+j_resolution
        # Check individual velocity/eta tail quantities, not only their scalar sum.
        for key in ['v_rmse','eta_rmse','g_angle_rmse','g_magnitude_rmse','landmark_rmse']:
            resolution=max(abs(base_m['50-60'][key]-tight_m['50-60'][key]),np.finfo(float).eps)
            continuous_ok &= cm['50-60'][key]<=base_m['50-60'][key]+resolution
        discrete_ok=better(dm,bm)
        decisions[candidate]={'continuous_no_degradation':bool(continuous_ok),'discrete_benefit':bool(discrete_ok),'continuous_only':bool(better(cm,base_m) and not discrete_ok)}
        if continuous_ok and discrete_ok: choices.append(candidate)
    def ranking(candidate):
        m=metric(entries[candidate]['EXPERIMENTAL']); t=m['wide']['t_VG']; return (isinstance(t,str), t if not isinstance(t,str) else float('inf'),m['J_tail'])
    star=min(choices,key=ranking) if choices else 'C0'
    write(out/'selection.json',{'C_star':star,'J_resolution':j_resolution,'state_reference_resolution':precision,'decisions':decisions,'status':'SELECTED_DIAGNOSTIC' if choices else 'NO_GAIN_IMPROVEMENT','frozen_at':time.time()})
    if star!='C0':
        run(out,'gains',config('CONT_REF','FAST',star)); run(out,'gains',config('EXPERIMENTAL','FAST',star))
        a=entries[star]['CONT_REF']; b=run(out,'gains',config('CONT_REF',candidate=star,tight=True)); reference_check(a,b,out)
    for candidate in dict.fromkeys(['C0',star]):
        for seed in [42,43]: run(out,'gains',config('EXPERIMENTAL',candidate=candidate,noise=seed))

def lifecycle(out):
    star=json.loads((out/'selection.json').read_text())['C_star']
    parallel_runs(out,'lifecycle',[config('EXPERIMENTAL',scene=scene,lifetime=life) for scene in ['REGULAR','FAST'] for life in [.5,1,2,5]])
    run(out,'lifecycle',config('SPLIT_REF',lifetime=1))
    if star!='C0':
        for scene in ['REGULAR','FAST']: run(out,'lifecycle',config('EXPERIMENTAL',scene=scene,candidate=star,lifetime=1))

def initialization(out):
    parallel_runs(out,'initialization',[config('EXPERIMENTAL',scene,lifetime=1,initialization=mode) for scene in ['REGULAR','FAST'] for mode in ['GT_SEED','GEOM_IDEAL','GT_MATCHED','GEOM_NOISY']])
    for scene in ['REGULAR','FAST']:
        entries=ledger(out)
        a=next(e for e in entries if e['config']==config('EXPERIMENTAL',scene,lifetime=1,initialization='GEOM_IDEAL') and e['status']=='COMPLETED')
        b=next(e for e in entries if e['config']==config('EXPERIMENTAL',scene,lifetime=1,initialization='GT_MATCHED') and e['status']=='COMPLETED')
        def events(e): return [(r['id'],r['t']) for r in json.loads((Path(e['directory'])/'seeds.json').read_text())]
        if events(a)!=events(b): raise AssertionError('GT_MATCHED schedule differs')

def main():
    parser=argparse.ArgumentParser(); parser.add_argument('--out',type=Path,default=DEFAULT); parser.add_argument('command',choices=['freeze','verify','fixed','discretization','gains','lifecycle','initialization','report','all','worker']); parser.add_argument('--config',type=Path)
    args=parser.parse_args(); out=args.out.resolve(); out.mkdir(parents=True,exist_ok=True)
    if args.command=='worker':
        from experiment import execute
        c=json.loads(args.config.read_text()); execute(c,out,args.config.parent); return
    # One driver lock prevents accidental concurrent schedules or overspending on resume.
    with (out/'driver.lock').open('w') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        import scipy,platform
        environment={'python':platform.python_version(),'numpy':np.__version__,'scipy':scipy.__version__,'compiler':subprocess.check_output(['g++','--version'],text=True).splitlines()[0],'blas_threads':1}
        env_path=out/'environment.json'
        if env_path.exists() and json.loads(env_path.read_text())!=environment:
            raise RuntimeError('Numerical environment changed; choose a fresh external output directory')
        write(env_path,environment)
        # A lock can be reacquired only after the previous driver has gone away.
        # Do not erase an interrupted reservation or return its budget.
        old=ledger(out)
        for entry in old:
            if entry['status']=='RUNNING':
                pid=entry.get('child_pid'); proc=Path(f'/proc/{pid}/cmdline')
                if pid and proc.exists() and str(Path(entry['directory'])/'config.json').encode() in proc.read_bytes():
                    raise RuntimeError('Previous simulation child is still running; do not overlap recovery')
                entry['status']='INTERRUPTED'; entry['exit_code']=None
                entry['recovery_note']='Previous driver exited without recording the child result; no identity reuse'
        if old: write(out/'ledger.json',old)
        stages=['freeze','verify','fixed','discretization','gains','lifecycle','initialization','report'] if args.command=='all' else [args.command]
        for stage in stages:
            if stage=='freeze': print(freeze(out))
            elif stage=='verify':
                build(out)
                from tests import verify
                result=verify(out); result['code']=code_identity(); write(out/'verification.json',result); print(result)
            elif stage=='report':
                from report import report
                report(out,DOC); write(DOC/'evidence/integrity.json',frozen_check())
            else:
                verified=json.loads((out/'verification.json').read_text())
                if verified['code']!=code_identity(): raise RuntimeError('Code changed: rerun verify before full simulations')
                frozen_check(); globals()[stage](out)
            print('STAGE_DONE',stage,flush=True)
if __name__=='__main__': main()
