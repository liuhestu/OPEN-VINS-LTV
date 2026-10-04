import unittest
import numpy as np
from r3c_shadow_audit import Window
class Contract(unittest.TestCase):
 def event(self,w,t,dt=.05,delta=None,phi=None,hard=None,angle=.01):
  return w.step(t,1,hard,np.eye(6) if phi is None else phi,np.zeros(6) if delta is None else np.array(delta),angle,t-dt)
 def test_full_then_twenty(self):
  w=Window();last=0.
  for k in range(40):
   t=k*.05;r=w.step(t,1,None,np.eye(6),np.zeros(6),.01,last);last=t
   if k<20:self.assertFalse(r['complete'])
   if k==20:self.assertEqual(r['confirm_G'],1)
   if k<39:self.assertFalse(r['ready_V'])
  self.assertTrue(r['ready_V']);dup=self.event(w,1.95);self.assertFalse(dup['ready_G']);self.assertEqual(w.g,20)
 def test_hard_reset(self):
  w=Window();last=0.
  for k in range(50):
   t=k*.05;r=w.step(t,1,None,np.eye(6),np.zeros(6),.01,last);last=t
  self.assertTrue(r['ready_G']);r=self.event(w,2.5,hard='outer_angle');self.assertFalse(r['ready_G']);self.assertEqual(len(w.entries),0)
  r=self.event(w,2.55);self.assertFalse(r['complete']);self.assertEqual(r['confirm_G'],0)
 def test_atomic_vs_weight(self):
  w=Window();times=[0,.2,.4,.6,.8,1.,1.1];prev=0
  for t in times:
   d=np.zeros(6);d[0]=.4 if t==.2 else 0
   r=w.step(t,1,None,np.eye(6),d,.03 if t==.2 else .01,prev);prev=t
  self.assertAlmostEqual(r['budgets']['V']['net'],.4)
  self.assertAlmostEqual(r['budgets']['angle_mean'],.012)
 def test_activity_and_independence(self):
  w=Window();prev=0
  for k in range(41):
   t=k*.05;d=np.zeros(6);d[0]=.1*(-1)**k
   r=w.step(t,1,None,np.eye(6),d,.01,prev);prev=t
  self.assertTrue(r['ready_G']);self.assertFalse(r['ready_V']);self.assertGreater(r['budgets']['V']['activity'],1)
  self.assertGreater(len(w.entries),0)
 def test_transport_and_impulse(self):
  w=Window();self.event(w,0,delta=[0,0,0,.2,0,0]);phi=np.eye(6);phi[0,3]=.1
  r=self.event(w,.1,dt=.1,phi=phi);self.assertAlmostEqual(r['budgets']['V']['net'],.02)
  self.assertEqual(r['budgets']['V']['impulse'],0)
 def test_capacity(self):
  w=Window();prev=0
  for k in range(65):
   t=k*.01;r=w.step(t,1,None,np.eye(6),np.zeros(6),0,prev);prev=t
  self.assertEqual(r['clear_reason'],'capacity');self.assertEqual(len(w.entries),0)
if __name__=='__main__':unittest.main()
