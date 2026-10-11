"""Input-only convention checks. No estimator outputs are read, including validation."""
import numpy as np
from scipy.spatial.transform import Rotation
from reference import *
def main():
 result={}
 for seq,source in read(OUT/'inputs.json').items():
  t,p,q,v=load_gt(seq);imu=np.loadtxt(Path(source['replay_root'])/'imu0/data.csv',delimiter=',',comments='#');imu[:,0]*=1e-9
  # Relative GT rotations predict angular rates in the same IMU axes (official sanity check).
  rr=Rotation.from_quat(q);indices=np.arange(1,len(t)-1,10)
  rate=(rr[indices-1].inv()*rr[indices+1]).as_rotvec()/(t[indices+1]-t[indices-1])[:,None]
  w=np.column_stack([np.interp(t[indices],imu[:,0],imu[:,i]) for i in [1,2,3]])
  gyro_error=np.linalg.norm(rate-w,axis=1)
  query=t[::100];a=local_derivative(t,p,query,.1,2)
  rot=rr[::100].as_matrix();f=np.column_stack([np.interp(query,imu[:,0],imu[:,i]) for i in [4,5,6]])
  g=a-np.einsum('nij,nj->ni',rot,f);g=g[np.isfinite(g).all(axis=1)];med=np.median(g,axis=0)
  tilt=float(np.degrees(np.arccos(np.clip(-med[2]/np.linalg.norm(med),-1,1))))
  result[seq]={'gt_first':float(t[0]),'gt_last':float(t[-1]),'gt_rows':len(t),'gt_max_gap_s':float(np.max(np.diff(t))),'quaternion_norm_max_error':float(np.max(np.abs(np.linalg.norm(q,axis=1)-1))), 'gyro_disagreement_median_rad_s':float(np.median(gyro_error)),'gyro_disagreement_p95_rad_s':float(np.quantile(gyro_error,.95)),'median_inferred_gravity_world':med.tolist(),'inferred_gravity_tilt_deg':tilt,'body_orientation_convention_consistent':bool(np.median(gyro_error)<.3),'gravity_sign_consistent':bool(med[2]<-7 and np.linalg.norm(med)<13 and tilt<5),'gravity_precision_note':'input sanity only; no claim of sub-degree ground truth accuracy','velocity_reference':'provided' if v is not None else 'position derivative; shared visual/IMU reconstruction possible'}
  print(seq,'gyro median',round(np.median(gyro_error),4),'gravity',np.round(med,3),'tilt',round(tilt,2),flush=True)
 write(OUT/'gt_preflight.json',result)
if __name__=='__main__':main()
