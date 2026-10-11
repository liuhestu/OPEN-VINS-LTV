#!/bin/bash
set -eo pipefail
ltv_install_root="$1"
shift
source /opt/ros/humble/setup.bash
if [ "$ltv_install_root" != "-" ]; then
  source "$ltv_install_root/local_setup.bash"
fi
/usr/bin/python3 - "$ltv_install_root" "$@" <<'PY'
import os,sys,json,hashlib,time,subprocess
from pathlib import Path
install=sys.argv[1];command=sys.argv[2:]
allowed=[Path('/opt/ros/humble').resolve()]
if install!='-':allowed.append(Path(install).resolve())
for key in ('AMENT_PREFIX_PATH','CMAKE_PREFIX_PATH','COLCON_PREFIX_PATH','LD_LIBRARY_PATH','PYTHONPATH'):
 for item in os.environ.get(key,'').split(':'):
  if not item:continue
  p=Path(item).resolve()
  if not any(p==root or root in p.parents for root in allowed) and not str(p).startswith(('/usr/','/lib/')):
   raise RuntimeError(f'foreign overlay in {key}: {p}')
out=Path(os.environ['LTV_TASK_ROOT'])/'results'/os.environ['LTV_RUN_ID']
metadata={'environment':{key:os.environ.get(key) for key in ('AMENT_PREFIX_PATH','CMAKE_PREFIX_PATH','COLCON_PREFIX_PATH','LD_LIBRARY_PATH','PYTHONPATH','ROS_DOMAIN_ID','RMW_IMPLEMENTATION','TMPDIR','ROS_LOG_DIR','COLCON_DEFAULTS_FILE')},'command':command,'install_root':install,'runtime_kind':'ROS_underlay_loaded; offline command is not ROS transport replay'}
if command and Path(command[0]).is_file():
 binary=Path(command[0]).resolve();metadata['executable']={'path':str(binary),'sha256':hashlib.sha256(binary.read_bytes()).hexdigest()}
 ldd=subprocess.run(['ldd',str(binary)],capture_output=True,text=True)
 metadata['dynamic_library_resolution']=ldd.stdout+ldd.stderr
 if 'not found' in ldd.stdout:raise RuntimeError(ldd.stdout)
(out/'environment.json').write_text(json.dumps(metadata,indent=2)+'\n')
pid=os.getppid();text=Path(f'/proc/{pid}/stat').read_text();f=text[text.rfind(')')+2:].split()
record={'event':'EXEC_READY','task':os.environ['LTV_TASK_NAME'],'run_id':os.environ['LTV_RUN_ID'],'pid':pid,'pgid':os.getpgid(pid),'start_ticks':f[19],'time':time.time(),'command':command,'expected_executable':str(Path(command[0]).resolve()) if command else None}
with (Path(os.environ['LTV_COORD_ROOT'])/'registry'/('task_'+os.environ['LTV_TASK_NAME']+'.jsonl')).open('a') as stream:stream.write(json.dumps(record)+'\n')
PY
exec "$@"
