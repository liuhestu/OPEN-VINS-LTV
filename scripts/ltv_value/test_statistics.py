import unittest
import numpy as np
from statistics import paired,dependence,windows
class StatsTests(unittest.TestCase):
 def test_paired_blocks(self):
  t=np.arange(0,20,.02);a=1+.1*np.sin(t);b=.8*a
  r=paired(t,a,b);self.assertAlmostEqual(r['improvement'],.2);np.testing.assert_allclose(r['ci95'],[.2,.2]);self.assertEqual(r['blocks'],20)
  b[:100]=np.nan;self.assertEqual(paired(t,a,b)['n'],900)
 def test_dependence(self):
  a=np.random.default_rng(42).normal(size=(10000,3));c=dependence(a,a)['centered_correlation'];np.testing.assert_allclose(np.diag(c),1)
  np.testing.assert_allclose(np.diag(dependence(a,-a)['centered_correlation']),-1)
  b=np.random.default_rng(123).normal(size=a.shape);self.assertLess(np.max(np.abs(dependence(a,b)['centered_correlation'])),.05)
  self.assertTrue(all(x is None for row in dependence(a,np.ones_like(a))['centered_correlation'] for x in row))
 def test_gap_windows(self):
  t=np.r_[np.arange(0,1,.02),np.arange(3,4,.02)];r=windows(t,np.ones(100),np.zeros(100));self.assertEqual(len(r['benefit']),2)
if __name__=='__main__':unittest.main()
