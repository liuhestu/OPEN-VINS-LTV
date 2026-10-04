import unittest
import numpy as np
from shared_transition import transition_step,transition,window_metrics,initialized_origin,fixed_window_rows
class SharedTransition(unittest.TestCase):
 def test_nonzero_preinitialization_prefix_and_window_endpoints(self):
  logs=[dict(camera_ns=100000000000,initialized=False,imu_time=0.),
        dict(camera_ns=105750000000,initialized=True,imu_time=105.766)]
  origin=initialized_origin(logs);self.assertEqual(origin,105.766)
  rows=[{'relative_time':(origin+i/20)-origin} for i in range(1801)]
  early=fixed_window_rows(rows,5.8,13.95);late=fixed_window_rows(rows,72.15,81.70)
  self.assertEqual(len(early),164);self.assertEqual(len(late),192)
  self.assertAlmostEqual(early[0]['relative_time'],5.8);self.assertAlmostEqual(early[-1]['relative_time'],13.95)
  self.assertAlmostEqual(late[0]['relative_time'],72.15);self.assertAlmostEqual(late[-1]['relative_time'],81.70)
  # Nonzero camera->IMU offset: the selected packet's physical endpoint is
  # exact even though its camera timestamp is earlier by .016 seconds.
  physical_packets=[dict(target=origin+i/20,camera=origin+i/20-.016) for i in range(1801)]
  physical_rows=[dict(relative_time=r['target']-origin,**r) for r in physical_packets]
  selected=fixed_window_rows(physical_rows,5.8,13.95)
  self.assertAlmostEqual(selected[0]['target']-origin,5.8)
  self.assertAlmostEqual(selected[-1]['target']-origin,13.95)
  wrong=fixed_window_rows([dict(relative_time=r['camera']-origin,**r) for r in physical_packets],5.8,13.95)
  self.assertNotEqual(selected[0]['target'],wrong[0]['target'])
  with self.assertRaises(ValueError):initialized_origin(logs[:1])
 def test_noncommuting_product_order(self):
  c={k:np.eye(3) for k in ('Da','Dw','R_ACCtoIMU','R_GYROtoIMU')};c['Tg']=np.zeros((3,3))
  samples=[dict(time=t,am=[0,0,0],wm=w) for t,w in [(0.,[1,0,0]),(.01,[1,0,0]),(.02,[0,2,0])]]
  first=transition_step(.01,np.array([1.,0,0]));last=transition_step(.01,np.array([.5,1,0]))
  np.testing.assert_array_equal(transition(samples,0.,.02,c,np.zeros(3),np.zeros(3)),last@first)
  self.assertGreater(np.linalg.norm(last@first-first@last),1e-6)
 def test_eta_correction_propagates_to_velocity(self):
  Phi=transition_step(.01,np.zeros(3))@transition_step(.02,np.zeros(3))
  delta=np.array([0.,0,0,1.,-2.,3.]);transported=Phi@delta
  np.testing.assert_allclose(transported[:3],[.03,-.06,.09],rtol=0,atol=1e-15)
  metrics=window_metrics([dict(time=0,transported=transported,delta=delta,dt=.02)])
  self.assertGreater(metrics['v']['norm_of_sum'],0.)
 def test_common_coordinate_rotation(self):
  Q=np.array([[0.,-1,0],[1,0,0],[0,0,1.]]);D=np.zeros((6,6));D[:3,:3]=Q;D[3:,3:]=Q
  w=np.array([.2,.3,-.1]);F=transition_step(.005,w)
  np.testing.assert_allclose(transition_step(.005,Q@w),D@F@D.T,rtol=0,atol=1e-16)
if __name__=='__main__':unittest.main()
