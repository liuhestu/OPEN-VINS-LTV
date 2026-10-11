import unittest
import numpy as np
from reconstruct_prediction import step,predict
class PredictionMath(unittest.TestCase):
 def test_old_state_euler(self):
  x=np.arange(12,dtype=float)/3;w=np.array([.3,-.2,.1]);a=np.array([.1,1.,2.]);dt=.005
  expected=x.copy()
  for j in range(2):expected[3*j:3*j+3]+=dt*(-np.cross(w,x[3*j:3*j+3])-x[-6:-3])
  expected[-6:-3]+=dt*(-np.cross(w,x[-6:-3])+x[-3:]+a)
  expected[-3:]+=dt*(-np.cross(w,x[-3:]))
  np.testing.assert_allclose(step(x,dt,a,w),expected,rtol=0,atol=2e-15)
 def test_bracket_calibration_and_no_future_integration(self):
  c={k:np.eye(3) for k in ('Da','Dw','R_ACCtoIMU','R_GYROtoIMU')};c['Tg']=np.zeros((3,3))
  samples=[dict(time=t,am=[t,0,0],wm=[0,0,0]) for t in (0.,1.,2.)]
  p=predict(np.zeros(6),samples,.25,1.25,c,np.zeros(3),np.zeros(3))
  self.assertAlmostEqual(p[0],.75);np.testing.assert_array_equal(p[1:],np.zeros(5))
if __name__=='__main__':unittest.main()
