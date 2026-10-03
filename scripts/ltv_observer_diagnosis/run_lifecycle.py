"""Two bounded passive replays; assert instrumentation leaves trajectory and audit unchanged."""
import sys,json,subprocess,time,hashlib
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'ltv_value'))
from analyze import selected
from common import OUT as OLD,read
OUT=Path('/home/he/output/openvins_ltv_observer_diagnosis_20261003')
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
results={}
for seq in ['V1_01_easy','indoor_forward_3']:
 old=Path(selected(seq)['path']);dest=OUT/('lifecycle_'+seq);dest.mkdir(exist_ok=False)
 oldpartial=str(old).removesuffix('.done')+'.partial'
 for p in old.glob('*.yaml'):
  (dest/p.name).write_text(p.read_text().replace(oldpartial,str(dest)))
 command=['bash','-c','source /opt/ros/humble/setup.bash && source "$1/install/setup.bash" && exec env "$6" "$7" "$1/install/ov_msckf/lib/ov_msckf/run_ltv_value" "$2" "$3" "$4" "$5"','lifecycle',str(OLD),str(dest/'estimator_config.yaml'),read(OLD/'inputs.json')[seq]['replay_root'],str(dest),'0','LD_PRELOAD='+str(OUT/'liblifecycle_trace.so'),'LTV_LIFECYCLE_TRACE='+str(dest/'lifecycle.csv')]
 record={'command':command,'reference':str(old),'preload_sha':sha(OUT/'liblifecycle_trace.so'),'config_sha':sha(dest/'estimator_config.yaml'),'status':'started'}
 (dest/'command.json').write_text(json.dumps(record,indent=2));start=time.monotonic()
 with (dest/'stdout.log').open('w') as o,(dest/'stderr.log').open('w') as e:r=subprocess.run(command,stdout=o,stderr=e)
 record.update(exit_code=r.returncode,seconds=time.monotonic()-start)
 record['byte_equal']={p:sha(dest/p)==sha(old/p) for p in ['trajectory.csv','audit.csv']} if r.returncode==0 else {}
 record['status']='pass' if r.returncode==0 and all(record['byte_equal'].values()) and (dest/'lifecycle.csv').stat().st_size>100 else 'fail'
 (dest/'command.json').write_text(json.dumps(record,indent=2));results[seq]=record
 (OUT/'lifecycle_runs.json').write_text(json.dumps(results,indent=2));print(seq,record['status'],flush=True)
 if record['status']!='pass':raise SystemExit(1)
