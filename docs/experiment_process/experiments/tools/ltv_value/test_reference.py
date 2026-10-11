import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from reference import local_derivative,interpolate,body_estimate,tangent_error
from prepare import ns,monotonic
class ReferenceTests(unittest.TestCase):
 def test_decimal(self):
  self.assertEqual(ns('1403636580.013555527'),1403636580013555527)
  self.assertEqual(ns('4878.802975732000'),4878802975732)
  self.assertEqual(ns('0.0000000001'),0)
  self.assertEqual(ns('0.0000000016'),2)
  with self.assertRaises(ValueError):monotonic([1,1])
 def test_polynomial_irregular(self):
  t=np.arange(0,2,.002);t[1:-1]+=np.sin(np.arange(1,len(t)-1))*.0001
  p=np.column_stack([t**3,2*t*t,3*t]);query=np.array([.3,.8,1.4])
  for width in [.05,.1,.2]:
   np.testing.assert_allclose(local_derivative(t,p,query,width),np.column_stack([3*query**2,4*query,np.full(3,3)]),atol=1e-10)
  self.assertTrue(np.isnan(local_derivative(t,p,[0,2])).all())
  take=(t<.75)|(t>.85);self.assertTrue(np.isnan(local_derivative(t[take],p[take],[.8],.2)).all())
 def test_rotation_sign_and_lever_arm(self):
  R=Rotation.from_rotvec([.3,-.8,.2]);q=R.as_quat();x=np.zeros((2,16));x[:,:4]=[q,-q];x[:,7:10]=[2,3,4]
  v,g=body_estimate(x);np.testing.assert_allclose(v,[R.inv().apply([2,3,4])]*2);np.testing.assert_allclose(g,[R.inv().apply([0,0,-1])]*2)
  np.testing.assert_allclose(tangent_error(g,g),0,atol=1e-12)
  # Rigid reference point velocity must include omega cross lever arm.
  lever=np.array([.2,.1,-.3]);omega=np.array([0,0,2]);velocity_imu=np.array([1,2,3]);point=velocity_imu+R.apply(np.cross(omega,lever))
  np.testing.assert_allclose(point-R.apply(np.cross(omega,lever)),velocity_imu)
 def test_slerp_and_gap(self):
  t=np.arange(0,.101,.005);q=Rotation.from_rotvec(np.outer(t,[0,0,1])).as_quat();p=np.outer(t,[1,2,3]);query=np.array([.0125,-.1,.2])
  pos,r,valid=interpolate(t,p,q,query);self.assertEqual(valid.tolist(),[True,False,False]);np.testing.assert_allclose(pos[0],[.0125,.025,.0375]);np.testing.assert_allclose(r[0],Rotation.from_rotvec([0,0,.0125]).as_matrix(),atol=1e-12)
if __name__=='__main__': unittest.main()
