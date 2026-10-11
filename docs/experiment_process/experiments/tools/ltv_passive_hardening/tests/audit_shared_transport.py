"""Independent affine-difference Phi test and fixed-output timeline inspection."""
import sys,json
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'diagnostics'))
from shared_transition import transition,initialized_origin,fixed_window_rows
from reconstruct_prediction import predict
rng=np.random.RandomState(812);maxdiff=0.
c={'Da':np.diag([1.01,.99,1.02]),'Dw':np.diag([.98,1.02,1.01]),'R_ACCtoIMU':np.array([[0.,-1,0],[1,0,0],[0,0,1]]),'R_GYROtoIMU':np.array([[1.,0,0],[0,0,-1],[0,1,0]]),'Tg':np.array([[.002,.003,0],[0,.001,.004],[.001,0,.002]])}
for k in range(30):
 samples=[dict(time=t,am=rng.randn(3),wm=rng.randn(3)) for t in [-.005,.005,.015,.025]];ba=rng.randn(3);bg=rng.randn(3);x=rng.randn(6);delta=rng.randn(6);P=transition(samples,0.,.02,c,ba,bg);actual=predict(x+delta,samples,0.,.02,c,ba,bg)-predict(x,samples,0.,.02,c,ba,bg);err=np.max(np.abs(actual-P@delta));maxdiff=max(maxdiff,float(err));assert err<2e-14
root=Path('/home/he/output/ltv_passive_hardening_v2');d=root/'diagnostics/R3B_V2_03_shared_transition_02';rows=[json.loads(l) for l in (d/'shared_transition.jsonl').read_text().splitlines()];logs=[json.loads(l) for l in (root/'real_development/R3B_grace_V2_03_01/P_NEW/features.jsonl').read_text().splitlines()];origin=initialized_origin(logs);assert all(x['relative_time']==x['imu_time']-origin for x in rows);offsets=[x['imu_time']-x['time'] for x in rows];selected={}
for label,a,b in [('early',5.8,13.95),('late',72.15,81.70)]:
 rr=fixed_window_rows(rows,a,b);assert rr and rr[0]['complete_1s'];selected[label]={'rows':len(rr),'first_relative':rr[0]['relative_time'],'last_relative':rr[-1]['relative_time'],'first_contribution_relative':rr[0]['window']['contribution_start']-origin,'first_window_complete':rr[0]['complete_1s']};assert rr[0]['window']['contribution_start']<rr[0]['imu_time']
result={'status':'PASS_DESCRIPTIVE_TOOL_AUDIT','affine_difference_nonidentity_calibration_max_error':maxdiff,'origin_imu_time':origin,'first_raw_camera_time':logs[0]['camera_ns']*1e-9,'actual_camera_imu_offset_minmax':[min(offsets),max(offsets)],'fixed_windows':selected,'limits':['No GT or observer/Riccati run.','Actual sequence offset is zero; nonzero offset supported by utility/math tests, not a second actual full dataset.','Signed transported corrections do not imply accuracy or causal decomposition by point.']};(d/'independent_audit.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
