"""Short, separately identified timing runs; never used for weight selection."""
import csv,json
import gzip as compression
import numpy as np
from common import *
from run import run

def stats(x):
    x=np.asarray(x,dtype=float)
    return {'calls':len(x),'sum_seconds':float(x.sum()),'mean_ms':float(x.mean()*1e3) if len(x) else None,
            'p95_ms':float(np.quantile(x,.95)*1e3) if len(x) else None}

def profile_cost():
    lib=OUT/'profiling/libltv_profile.so'
    if not lib.exists():raise RuntimeError('build optional wrapper from profiling/compile_command.json first')
    result={}
    for seq in ['V1_01_easy','indoor_forward_3']:
        result[seq]={}
        original=next(r for r in latest_runs().values() if r['sequence']==seq and r['mode']=='GV' and r['short'] and r['diagnostics'] and not r.get('profile') and r['status']=='complete')
        for enabled in [True,False]:
            r=run(seq,'GV',stage='profile',short=True,diagnostics=enabled,profile=lib)
            if r['status']!='complete':raise RuntimeError('profile execution failed')
            p=Path(r['path'])
            exact={f:sha(p/f)==sha(Path(original['path'])/f) for f in ['trajectory.csv','audit.csv']}
            if not all(exact.values()):raise RuntimeError('profiling changed trajectory/state')
            rows=list(csv.DictReader((p/'profile.csv').open()));build={};ekf=[];gzip=[]
            for row in rows:
                t=float(row['camera_time']);dt=float(row['seconds'])
                if row['function']=='auxiliary_build':build.setdefault(t,[]).append(dt)
                elif row['function']=='native_ekf':ekf.append(dt)
                elif row['function']=='gzip_write':gzip.append(dt)
                else:raise RuntimeError('unknown instrumented function')
            if not build or not ekf:raise RuntimeError('exported call interception did not work')
            if any(len(v)!=(4 if enabled else 1) for v in build.values()):raise RuntimeError('unexpected per-event auxiliary call count')
            actual=[v[0] for v in build.values()];shadow=[x for v in build.values() for x in v[1:]]
            data={'run':r['id'],'profile_sha':sha(lib),'exact_to_uninstrumented':exact,'runtime_seconds':r['seconds'],
                  'actual_auxiliary_build':stats(actual),'shadow_auxiliary_build':stats(shadow),'native_ekf':stats(ekf),'gzip_write':stats(gzip)}
            if enabled:
                # Cost/identity checks have no GT dependency, including pre-GT short segments.
                timing={k:[] for k in ['observer_seconds','msckf_seconds_without_diagnostics','diagnostic_compute_seconds']};errors=[]
                with compression.open(p/'value.jsonl.gz','rt') as f:
                    for line in f:
                        row=json.loads(line)
                        for key in timing:timing[key].append(row[key])
                        post=np.asarray(row['posterior']).ravel();shadow=np.asarray(row['shadow_GV']).ravel()
                        errors.append(float(np.linalg.norm(post-shadow)/max(1,np.linalg.norm(post))))
                if len(gzip)!=len(errors):raise RuntimeError('diagnostic gzip/event count differs')
                if not errors or max(errors)>1e-9:raise RuntimeError('native/shadow mismatch')
                for key in timing:data[key]=stats(timing[key])
                data['shadow_native_max_relative_error']=max(errors)
            result[seq]['diagnostics_on' if enabled else 'diagnostics_off']=data
            write(OUT/'cost_profile.json',result)
    return result
if __name__=='__main__':profile_cost()
