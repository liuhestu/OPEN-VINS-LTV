"""Verify saved point predictions/support only; never solve for a point."""
import json,hashlib
from pathlib import Path
import numpy as np
root=Path('/home/he/output/ltv_passive_hardening_v2/diagnostics')
support_path=root/'R3B_holdout_support_02/support.json';context_path=support_path.parent/'contexts.jsonl';result_path=root/'R3B_holdout_fits_01/results.json'
support=json.loads(support_path.read_text());results=json.loads(result_path.read_text());contexts=[json.loads(s) for s in context_path.read_text().splitlines()]
reports=[]
for record,result in zip(support['records'],results['results']):
 fid=record['id'];assert result['id']==fid and result['camera_ns']==record['camera_ns']
 targets=[c for c in contexts if c['camera_time']==record['camera_ns']*1e-9];assert len(targets)==1
 current=targets[0];assert current['time']==record['time'] and current['version']==record['version']
 valid=[];missing=[];observed=[];errors=[]
 cam=current['cameras'][0];Rc=np.array(cam['R_BC']).reshape(3,3);pc=np.array(cam['p_BC']);point=np.array(result['point_W'])
 assert np.isfinite(Rc).all() and np.linalg.det(Rc)>0 and np.linalg.norm(Rc.T@Rc-np.eye(3))<1e-8
 for row in contexts:
  if row['epoch']!=current['epoch'] or not current['time']-1<=row['time']<current['time']:continue
  obs=[o for o in row['observations'] if o['id']==fid and o['camera']==0 and o['valid']]
  assert len(obs)<=1
  if not obs:continue
  observed.append(row['time']);poses=[p for p in current['poses'] if abs(p['time']-row['time'])<=1e-9]
  if len(poses)!=1:missing.append(row['time']);continue
  pose=poses[0];R=np.array(pose['R_WB']).reshape(3,3);z=np.array(obs[0]['bearing']);assert np.isfinite(z).all() and np.linalg.norm(z)>0 and np.linalg.det(R)>0
  c=np.array(pose['p_WB'])+R@pc;b=R@Rc@(z/np.linalg.norm(z));valid.append((row['time'],c,b))
 assert observed==record['past_observation_times'] and missing==record['missing_current_context_clone_times']==result['missing_clone_times']
 assert len(valid)==len(record['past'])==result['past_count']==11
 angles=[];signed=[]
 for (t,c,b),saved in zip(valid,record['past']):
  assert t==saved['time'] and np.array_equal(c,np.array(saved['center'])) and np.array_equal(b,np.array(saved['direction']))
  v=point-c;angles.append(float(np.arccos(np.clip(v@b/np.linalg.norm(v),-1,1))));signed.append(float(v@b))
 pose=current['poses'][current['execution_pose_index']];assert abs(pose['time']-current['time'])<=1e-9
 R=np.array(pose['R_WB']).reshape(3,3);c=np.array(pose['p_WB'])+R@pc
 hold=[o for o in current['observations'] if o['id']==fid and o['camera']==0 and o['valid']];assert len(hold)==1
 z=np.array(hold[0]['bearing']);b=R@Rc@(z/np.linalg.norm(z));v=point-c
 held=float(np.arccos(np.clip(v@b/np.linalg.norm(v),-1,1)))
 err=max(float(np.max(abs(np.array(angles)-result['past_angle_rad']))),float(np.max(abs(np.array(signed)-result['past_signed_ray_distances']))),abs(held-result['heldout_current_angle_rad']))
 assert err<1e-11
 reports.append({'id':fid,'camera_ns':record['camera_ns'],'single_current_version':current['version'],'past_count':len(valid),'missing_clone_count':len(missing),'past_span_s':valid[-1][0]-valid[0][0],'minimum_past_offset_s':valid[0][0]-current['time'],'maximum_past_offset_s':valid[-1][0]-current['time'],'max_saved_prediction_difference':err,'heldout_angle_rad':held,'past_max_angle_rad':max(angles)})
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
o={'status':'PASS_SAVED_POINT_SUPPORT_AND_RESIDUALS','new_point_solves':0,'GT_read':False,'results':reports,'provenance':{str(p):sha(p) for p in (Path(__file__),support_path,context_path,result_path)},'limits':['Independent numerical check uses saved point_W, not an independent triangulation solver.','Onlytwo registered points and actual current-pose contexts; no covariance calibration.']}
(result_path.parent/'independent_saved_point_audit.json').write_text(json.dumps(o,indent=2)+'\n');print(json.dumps(o,indent=2))
