import copy,unittest
import numpy as np
from past_fit_holdout import select,fit,ray
class PastFit(unittest.TestCase):
 def test_strict_history_boundary_is_not_clone_match_tolerance(self):
  rows,current,_=self.fixture()
  for r,pose,t in zip(rows,current['poses'],(1.-5e-10,1.,1.1,2.)):
   r['time']=t;pose['time']=t
  current['time']=2.
  record=select(rows,current,7)
  self.assertEqual(record['past_observation_times'],[1.,1.1])
  self.assertNotIn(1.-5e-10,record['past_observation_times'])
 def fixture(self):
  Rc=np.array([[0.,-1.,0.],[1.,0.,0.],[0.,0.,1.]])
  camera={'R_BC':Rc.ravel().tolist(),'p_BC':[.1,.02,0.]};point=np.array([.2,.1,3.]);poses=[];rows=[]
  for t in (0.,.1,.2,.3):
   pose={'time':t,'R_WB':np.eye(3).ravel().tolist(),'p_WB':[t,0.,0.]};poses.append(pose)
   z=Rc.T@(point-np.array(pose['p_WB'])-np.array(camera['p_BC']));z/=np.linalg.norm(z)
   rows.append({'time':t,'epoch':1,'observations':[{'id':7,'camera':0,'valid':True,'bearing':z.tolist()}]})
  current=dict(rows[-1],poses=poses,cameras=[camera],execution_pose_index=3,version=5)
  return rows,current,point
 def test_static_point_nonzero_extrinsic_and_current_exclusion(self):
  rows,current,point=self.fixture();a=fit(select(rows,current,7))
  np.testing.assert_allclose(a['point_W'],point,atol=2e-12,rtol=0)
  changed=copy.deepcopy(current);changed['observations'][0]['bearing']=[1.,0.,0.]
  rows[-1]=changed;rows.append(dict(changed,time=.35))
  b=fit(select(rows,changed,7));np.testing.assert_array_equal(a['point_W'],b['point_W'])
  self.assertGreater(b['heldout_current_angle_rad'],.5)
  self.assertEqual(a['past_count'],3)
 def test_missing_current_clones_no_stale_fallback(self):
  rows,current,_=self.fixture();current['poses']=current['poses'][-1:];current['execution_pose_index']=0
  for r in rows:r['poses']=[{'time':r['time'],'p_WB':[999,0,0]}]
  record=select(rows,current,7)
  self.assertEqual(len(record['missing_current_context_clone_times']),3)
  self.assertEqual(fit(record)['status'],'NOT_IDENTIFIABLE')
 def test_degenerate_past_and_right_exclusion(self):
  rows,current,_=self.fixture()
  for pose in current['poses']:pose['p_WB']=[0.,0.,0.]
  for r in rows:r['observations'][0]['bearing']=[0.,0.,1.];r['observations'].append({'id':7,'camera':1,'valid':True,'bearing':[1.,0.,0.]})
  record=select(rows,current,7);self.assertEqual(len(record['past']),3)
  self.assertEqual(fit(record)['reason'],'DEGENERATE')
if __name__=='__main__':unittest.main()
