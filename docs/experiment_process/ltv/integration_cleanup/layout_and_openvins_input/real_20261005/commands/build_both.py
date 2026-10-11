import concurrent.futures,json,os,subprocess,time
from pathlib import Path
OUT=Path(__file__).resolve().parent
ID=json.loads((OUT/'initial_identity.json').read_text())
def build(label):
 ws=OUT/label;script=ws/'build.sh'
 script.write_text('''#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 NUMEXPR_NUM_THREADS=1 CMAKE_BUILD_PARALLEL_LEVEL=1 MAKEFLAGS=-j1
cd '''+str(ws)+'''
colcon --log-base '''+str(ws/'log')+''' build --base-paths '''+ID[label]+''' --packages-up-to ov_msckf --executor sequential --build-base '''+str(ws/'build')+''' --install-base '''+str(ws/'install')+''' --cmake-args -DENABLE_ROS=ON -DBUILD_LTV_PHASE0_TESTS=ON -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DPYTHON_EXECUTABLE=/usr/bin/python3
''')
 cmd=['/usr/bin/env']
 for key in ('CMAKE_PREFIX_PATH','AMENT_PREFIX_PATH','COLCON_PREFIX_PATH','LD_LIBRARY_PATH','PYTHONPATH'):cmd+=['-u',key]
 cmd+=['bash','--noprofile','--norc',str(script)]
 identity={'label':label,'source':ID[label],'ref':ID[label+'_ref'],'command':cmd,'started':time.time()}
 (ws/'build_attempt.json').write_text(json.dumps(identity,indent=2)+'\n')
 with (ws/'build.stdout').open('w') as a,(ws/'build.stderr').open('w') as b:
  p=subprocess.Popen(cmd,stdout=a,stderr=b);identity['pid']=p.pid;(ws/'build_attempt.json').write_text(json.dumps(identity,indent=2)+'\n');identity['exit_code']=p.wait()
 identity['seconds']=time.time()-identity['started'];(ws/'build_attempt.json').write_text(json.dumps(identity,indent=2)+'\n');print(json.dumps(identity),flush=True)
 return identity
with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:results=list(pool.map(build,('baseline','current')))
(OUT/'build_results.json').write_text(json.dumps(results,indent=2)+'\n')
if any(x['exit_code'] for x in results):raise SystemExit(1)
