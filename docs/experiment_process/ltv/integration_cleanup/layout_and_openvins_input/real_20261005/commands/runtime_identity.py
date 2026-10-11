import hashlib,json,os,re,subprocess
from pathlib import Path
OUT=Path(__file__).resolve().parent
results={}
def sha(path):
 h=hashlib.sha256()
 with Path(path).open('rb') as f:
  for b in iter(lambda:f.read(1<<20),b''):h.update(b)
 return h.hexdigest()
env={k:v for k,v in os.environ.items() if k not in ('CMAKE_PREFIX_PATH','AMENT_PREFIX_PATH','COLCON_PREFIX_PATH','LD_LIBRARY_PATH','PYTHONPATH')}
for label in ('baseline','current'):
 binary=OUT/label/'install/ov_msckf/lib/ov_msckf/run_ltv_feature_passive';cmd=['bash','--noprofile','--norc','-c','source /opt/ros/humble/setup.bash && source "$1/setup.bash" && ldd "$2"','identity',str(OUT/label/'install'),str(binary)]
 text=subprocess.check_output(cmd,env=env,text=True);assert 'not found' not in text
 libs={}
 for name,path in re.findall(r'^\s*(\S+) => (/\S+) ',text,re.M):libs[name]=dict(path=path,sha=sha(path))
 for name in ('libov_msckf_lib.so','libov_core_lib.so','libov_init_lib.so'):assert Path(libs[name]['path']).is_relative_to(OUT/label),libs[name]
 results[label]=dict(binary=str(binary),binary_sha=sha(binary),ldd_command=cmd,libraries=libs)
a,b=[results[x]['libraries'] for x in ('baseline','current')];assert a.keys()==b.keys()
for name in a:
 if not name.startswith('libov_'):assert a[name]==b[name],name
(OUT/'runtime_identity.json').write_text(json.dumps(dict(status='PASS_ISOLATED_LOCAL_LIBRARIES_SAME_SYSTEM_LIBRARIES',results=results),indent=2)+'\n');print('PASS runtime libraries resolved to each isolated workspace; external dependencies identical')
