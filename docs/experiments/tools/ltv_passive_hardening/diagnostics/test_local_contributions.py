import unittest
import numpy as np
from local_correction_contributions import camera_correction,lifecycle
CFG=dict(q_landmark=.2,camera_euler_safety=.5,max_camera_substeps=100,covariance_floor=1e-9,covariance_failure_threshold=-.001,initial_p_landmark=1.)
class LocalMath(unittest.TestCase):
 def test_oldP_simultaneous_and_sum(self):
  x=np.arange(12,dtype=float)/10;L=np.eye(12)+.07*np.ones((12,12));P=L@L.T
  current={'observations':[{'id':9,'camera':0,'bearing':[0.,0.,1.]},{'id':8,'camera':0,'bearing':[1.,0.,0.]}], 'calibration':{'R_BC':np.eye(3).ravel().tolist(),'p_BC':[.02,.03,0.]}}
  d=dict(calls=0,floor_clamped_values=0,minimum_pre_floor_eigenvalue=float('inf'))
  after,Q,steps,rate,ids,terms,detail,error=camera_correction(x.copy(),P.copy(),current,[8,9],.01,CFG,d)
  H=np.zeros((6,12));H[:3,:3]=np.diag([0.,1,1]);H[3:,3:6]=np.diag([1.,1,0])
  y=np.r_[[0.,.03,0.],[.02,.03,0.]]
  expected=x+.01*.2*P@H.T@(y-H@x)
  np.testing.assert_allclose(after,expected,rtol=0,atol=1e-14)
  np.testing.assert_allclose(terms.sum(axis=0),after-x,rtol=0,atol=1e-14)
  self.assertEqual(ids,[8,9]);self.assertEqual(steps,1);self.assertLess(error,1e-14)
 def test_birth_block_and_old_cross_blocks_preserved(self):
  x=np.arange(9,dtype=float);P=np.arange(81,dtype=float).reshape(9,9)
  prior={'slots':[[7,0]]};current={'slots':[[8,0],[7,1]],'births':[{'id':8,'apply_seed':1,'mean':[3.,4.,5.]}]}
  a,B,ids=lifecycle(x,P,prior,current,CFG)
  np.testing.assert_array_equal(a[:3],[3,4,5]);np.testing.assert_array_equal(B[3:,3:],P)
  np.testing.assert_array_equal(B[:3,3:],0);np.testing.assert_array_equal(B[:3,:3],np.eye(3))
if __name__=='__main__':unittest.main()
