"""One-event actual-core intervention, with test-only full snapshot injection.

Reads cache as evidence; executes only one original and one filtered camera
update. This is not an enabled-from-start trajectory or a lifetime replay.
"""
import os
for k in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS'):os.environ[k]='1'
import argparse,hashlib,json,subprocess
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[2]
OLD=Path('/home/he/output/ltv_passive_hardening_v2')
TARGETS={1413394964105760512:27537,1413394966255760384:29636}
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def rows(p):return [json.loads(l) for l in Path(p).read_text().splitlines() if l]
def run(camera_ns,binary,out):
 out=Path(out);out.mkdir(parents=True,exist_ok=False)
 cache=OLD/'real_development/R3B_grace_V2_03_01/P_NEW/cache.bin';decoder=OLD/'export_correction_cache_01'
 ep=OLD/'diagnostics/R3B_local_contributions_01/events.jsonl';cp=OLD/'diagnostics/R3B_holdout_support_02/contexts.jsonl'
 featurefile=OLD/'real_development/R3B_grace_V2_03_01/P_NEW/features.jsonl'
 cfgfile=OLD/'real_development/R3B_grace_V2_03_01/config/estimator_config.yaml'
 prior=json.loads((ep.parent/'summary.json').read_text())
 for p in (cache,decoder,cfgfile):
  if prior['identity'][str(p)]!=sha(p):raise ValueError('prior source identity changed')
 matches=[e for e in rows(ep) if e['camera_ns']==camera_ns]
 if len(matches)!=1:raise ValueError('unique event required')
 e=matches[0];fid=TARGETS[camera_ns];logs=rows(featurefile);f=[x for x in logs if x['camera_ns']==camera_ns]
 if len(f)!=1 or f[0]['imu_time']!=e['time']:raise ValueError('integer receipt physical-time mapping')
 contexts=rows(cp);cs=[c for c in contexts if c['time']==e['time'] and any(o['id']==fid and o['camera']==0 for o in c['observations'])]
 if len(cs)!=1:raise ValueError('unique current context')
 ctx=cs[0];history=[c for c in contexts if ctx['time']-1.<=c['time']<ctx['time'] and c['epoch']==ctx['epoch'] and any(o['id']==fid and o['camera']==0 for o in c['observations'])]
 actual=[]
 with (out/'decoder_stderr.log').open('w') as stderr:
  proc=subprocess.Popen([str(decoder),str(cache)],stdout=subprocess.PIPE,stderr=stderr,text=True)
  for line in proc.stdout:
   r=json.loads(line)
   if r.get('kind')=='camera' and r['target']==e['time']:actual.append(r)
  if proc.wait()!=0:raise RuntimeError('decoder failure')
 if len(actual)!=1:raise ValueError('unique actual cache process required')
 raw=actual[0];n=len(raw['x']);P=np.array(raw['P']).reshape(n,n).tolist()
 slots=[fid for fid,slot in sorted(raw['slots'],key=lambda p:p[1])]
 if slots!=e['slots']:raise ValueError('cached slot ordering mismatch')
 keys=['max_imu_dt','reset_gap','q_landmark','v_landmark','v_velocity','v_gravity','initial_p_landmark','covariance_floor','covariance_failure_threshold','camera_euler_safety','max_camera_substeps']
 config={}
 for key in keys:
  values=[line.split(':',1)[1].split('#')[0].strip() for line in cfgfile.read_text().splitlines() if line.startswith('ltv_observer_'+key+':')]
  if len(values)!=1:raise ValueError('config field '+key)
  config[key]=int(values[0]) if key=='max_camera_substeps' else float(values[0])
 data=dict(event=e,actual=raw,actual_P=P,feature_id=fid,context=ctx,history=history,core_config=config)
 inp=out/'input.json';inp.write_text(json.dumps(data,allow_nan=False)+'\n')
 command=[str(binary),str(inp),str(out/'result.json')]
 with (out/'stdout.log').open('w') as stdout,(out/'stderr.log').open('w') as stderr:exitcode=subprocess.run(command,stdout=stdout,stderr=stderr).returncode
 evidence=dict(scope='ONE_CAMERA_EVENT_ORIGINAL_VS_FILTERED_PRODUCTION_CORE',camera_ns=camera_ns,feature_id=fid,exit_code=exitcode,command=command,
  limitations=['After-lifecycle x/P comes from independently validated reconstruction, with original full cached post x/P as oracle.','Snapshots installed with test-only const_cast into a nonconst core; production setters and mathematics unchanged.','Only the requested old ID is filtered; other active points are unchanged.','First-failure counter is deliberately zero; sequential retirement and seed guards are tested separately, not claimed here.','Context epoch is original bridge context identity, not necessarily core epoch.'],
  sha={str(p):sha(p) for p in (cache,decoder,ep,cp,cfgfile,featurefile,Path(binary),Path(__file__),inp)})
 (out/'provenance.json').write_text(json.dumps(evidence,indent=2)+'\n')
 if exitcode:raise RuntimeError('core harness failed; artifacts retained')
 print(json.dumps({'camera_ns':camera_ns,'output':str(out),'status':'PASS'}))
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--camera-ns',type=int,choices=list(TARGETS),required=True);p.add_argument('--binary',required=True);p.add_argument('--out',required=True)
 a=p.parse_args();run(a.camera_ns,a.binary,a.out)
