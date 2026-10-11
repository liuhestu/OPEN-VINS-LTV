import concurrent.futures,hashlib,json,os,subprocess,sys,time
from pathlib import Path
OUT=Path(__file__).resolve().parent
ID=json.loads((OUT/'initial_identity.json').read_text());INPUTS=json.loads((OUT/'real_inputs.json').read_text())
ENV={k:v for k,v in os.environ.items() if k not in ('CMAKE_PREFIX_PATH','AMENT_PREFIX_PATH','COLCON_PREFIX_PATH','LD_LIBRARY_PATH','PYTHONPATH')}
ENV.update(OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1',NUMEXPR_NUM_THREADS='1')
def sha(path):
 h=hashlib.sha256()
 with Path(path).open('rb') as f:
  for b in iter(lambda:f.read(1<<20),b''):h.update(b)
 return h.hexdigest()
def command(label,binary,args):
 return ['bash','--noprofile','--norc','-c','source /opt/ros/humble/setup.bash && source "$1/setup.bash" && exec "$2" "${@:3}"','regression',str(OUT/label/'install'),str(binary),*map(str,args)]
def logged(label,sequence,index):
 assert json.loads((OUT/label/'build_attempt.json').read_text())['exit_code']==0
 config=INPUTS[sequence];root=OUT/'runs'/f'{label}_{sequence}_{index:02d}';root.mkdir(parents=True,exist_ok=False)
 for name,digest in config['config_sha'].items():assert sha(Path(config['config']).parent/name)==digest
 for v in config['sensors'].values():assert sha(v['path'])==v['sha256']
 binary=OUT/label/'install/ov_msckf/lib/ov_msckf/run_ltv_feature_passive'
 cmd=command(label,binary,[config['config'],config['root'],root,'P_NEW'])
 identity=dict(label=label,sequence=sequence,index=index,source=ID[label],source_ref=ID[label+'_ref'],input_identity=config,binary=str(binary),binary_sha=sha(binary),command=cmd,cwd=str(root),started=time.time(),threads={k:ENV[k] for k in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS','NUMEXPR_NUM_THREADS')})
 (root/'identity.json').write_text(json.dumps(identity,indent=2)+'\n')
 with (root/'stdout.log').open('w') as a,(root/'stderr.log').open('w') as b:
  p=subprocess.Popen(cmd,cwd=root,env=ENV,stdout=a,stderr=b);identity['pid']=p.pid;(root/'identity.json').write_text(json.dumps(identity,indent=2)+'\n');identity['exit_code']=p.wait()
 identity['seconds']=time.time()-identity['started'];(root/'identity.json').write_text(json.dumps(identity,indent=2)+'\n');print(json.dumps({'run':str(root),'exit_code':identity['exit_code'],'seconds':identity['seconds']}),flush=True)
 if identity['exit_code']:raise RuntimeError('failed native attempt preserved: '+str(root))
 return identity
phase=sys.argv[1]
if phase=='baseline':jobs=[('baseline',seq,i) for i in (1,2) for seq in INPUTS]
elif phase=='current':jobs=[('current',seq,1) for seq in INPUTS]
else:raise ValueError(phase)
summary=OUT/(phase+'_run_jobs.json');assert not summary.exists();summary.write_text(json.dumps({'complete':False,'jobs':jobs},indent=2)+'\n')
with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:results=list(pool.map(lambda q:logged(*q),jobs))
summary.write_text(json.dumps({'complete':True,'results':results},indent=2)+'\n')
