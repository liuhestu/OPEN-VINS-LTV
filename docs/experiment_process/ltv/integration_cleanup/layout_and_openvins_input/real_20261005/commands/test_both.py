import concurrent.futures,json,subprocess,os,time
from pathlib import Path
OUT=Path(__file__).resolve().parent
names=['test_ltv_alias_fixed','test_ltv_core_lifecycle','test_ltv_gv_jacobian','test_ltv_gv_covariance','test_ltv_fej','test_ltv_timing','test_ltv_joint_layout','test_ltv_joint_covariance','test_ltv_innovation','test_ltv_options','test_ltv_value_shadow','test_ltv_huber','test_ltv_controlled','test_ltv_feature_manager','test_ltv_feature_pipeline','test_ltv_pose_convention','test_ltv_active_lifecycle','test_ltv_active_consistency','test_ltv_readiness','test_ltv_adapter_hardened','test_ltv_manager_bounded','test_bounded_history','test_identity_guard','test_ltv_history_competition']
def test(label):
 assert json.loads((OUT/label/'build_attempt.json').read_text())['exit_code']==0
 root=OUT/label/'tests';root.mkdir(exist_ok=False);results=[]
 for name in names:
  cmd=['bash','--noprofile','--norc','-c','source /opt/ros/humble/setup.bash && source "$1/setup.bash" && exec "$2"','tests',str(OUT/label/'install'),str(OUT/label/'install/ov_msckf/lib/ov_msckf'/name)]
  start=time.time()
  with (root/(name+'.stdout')).open('w') as a,(root/(name+'.stderr')).open('w') as b:r=subprocess.run(cmd,cwd=root,env=dict({k:v for k,v in os.environ.items() if k not in ('CMAKE_PREFIX_PATH','AMENT_PREFIX_PATH','COLCON_PREFIX_PATH','LD_LIBRARY_PATH','PYTHONPATH')},OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1'),stdout=a,stderr=b)
  results.append(dict(name=name,command=cmd,exit_code=r.returncode,seconds=time.time()-start));print(label,name,r.returncode,flush=True)
 (root/'results.json').write_text(json.dumps(results,indent=2)+'\n');return dict(label=label,total=len(results),failed=[x['name'] for x in results if x['exit_code']])
with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:results=list(pool.map(test,('baseline','current')))
(OUT/'tests_summary.json').write_text(json.dumps(results,indent=2)+'\n')
if any(x['failed'] for x in results):raise SystemExit(1)
