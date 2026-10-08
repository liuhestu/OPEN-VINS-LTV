#!/usr/bin/env python3
"""Synthetic nonzero-clock-offset/frozen-gauge test, independent known truth."""
import argparse
import csv
import json
from pathlib import Path
import subprocess
import numpy as np
from scipy.spatial.transform import Rotation
from map_finite_main_true_error import jpl_product, skew

p=argparse.ArgumentParser();p.add_argument('output',type=Path);a=p.parse_args()
if a.output.exists():raise RuntimeError('refusing fixture overwrite')
a.output.mkdir();off=a.output/'off';run=a.output/'run';off.mkdir();run.mkdir()
t=np.arange(0.,2.505,.005);positions=np.c_[np.sin(t),np.cos(1.3*t),.2*t*t]
gt=np.zeros((len(t),17));gt[:,0]=t*1e9;gt[:,1:4]=positions;gt[:,4]=1
np.savetxt(a.output/'gt.csv',gt,delimiter=',',header='time,p1,p2,p3,qw,qx,qy,qz,v1,v2,v3,bg1,bg2,bg3,ba1,ba2,ba3',comments='')
rotation=Rotation.from_rotvec([.1,-.2,.25]).as_matrix();translation=np.array([.2,-.1,.05]);offset=.013
support=np.arange(.2,2.25,.05);gi=np.round(support/.005).astype(int);estimated=(positions[gi]-translation)@rotation
trajectory=np.zeros((len(support),17));trajectory[:,0]=support;trajectory[:,4]=1;trajectory[:,5:8]=estimated
np.savetxt(off/'trajectory.csv',trajectory,delimiter=',',header='timestamp,qx,qy,qz,qw,px,py,pz,vx,vy,vz,bgx,bgy,bgz,bax,bay,baz',comments='')
with (off/'audit.csv').open('w',newline='') as f:
 writer=csv.writer(f);writer.writerow(['camera_ns']);writer.writerows([[int(round((time-offset)*1e9))] for time in support])
(off/'effective_options.json').write_text(json.dumps({'calib_camimu_dt':offset}))
e0=np.array([.2,-.1,.05]);qe=np.r_[e0/2,1.];qe/=np.linalg.norm(qe);inverse=qe.copy();inverse[:3]*=-1
true=Rotation.from_matrix(rotation.T).as_quat();estimated_q=jpl_product(inverse,true)
pose=np.r_[support[20],0.,3.,estimated_q,estimated[20]][None,:]
matrices={'nominal_poses_time_orientation_row_position_row_qGtoI_xyzw_pWorld':pose,'Sigma_main_local_program':.01*np.eye(6),'P_stored_main':.01*np.eye(6),'C_main_local_program_observer':np.zeros((6,9))}
index={}
with (run/'finite.csv.matrices.bin').open('wb') as f:
 for name,value in matrices.items():
  index[name]={'offset':f.tell(),'rows':value.shape[0],'cols':value.shape[1]};f.write(value.astype(np.float64).tobytes(order='F'))
(run/'finite.csv.matrices.jsonl').write_text(json.dumps({'time':support[20],'epoch':1,'feature':1,'matrices':index})+'\n')
subprocess.run(['/usr/bin/python3',str(Path(__file__).with_name('map_finite_main_true_error.py')),str(run),str(a.output/'gt.csv'),str(off/'trajectory.csv'),str(a.output/'mapped'),'--camera-imu-offset',str(offset)],check=True)
identity=json.loads((a.output/'mapped/identity.json').read_text());assert np.linalg.norm(np.asarray(identity['rotation_est_to_gt'])-rotation)<1e-10;assert np.linalg.norm(np.asarray(identity['translation_est_to_gt'])-translation)<1e-10
record=json.loads((a.output/'mapped/mapped_covariances.jsonl').read_text());item=record['matrices']['Sigma_main_true_error_under_declared_source_law']
with (a.output/'mapped/mapped_covariances.bin').open('rb') as f:f.seek(item['offset']);actual=np.frombuffer(f.read(8*36),dtype=np.float64).reshape((6,6),order='F')
mu=np.eye(6);mu[:3,:3]=np.eye(3)-.5*skew(e0)+.25*np.outer(e0,e0);assert np.linalg.norm(actual-.01*mu@mu.T)<1e-10
print('nonzero_offset_once=PASS shared_OFF_nearest20ms_SE3_gauge=PASS nonzero_true_error_chart_mapping=PASS real_statistical_calibration=NOT_PROVEN')
