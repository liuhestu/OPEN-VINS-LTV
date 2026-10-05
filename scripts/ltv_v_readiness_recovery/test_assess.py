"""Independent episode, censoring, quality-denominator and predeclared-decision fixtures."""
import unittest
import numpy as np
from assess import applied, deterioration, partition, quality, toggles


class AssessmentTests(unittest.TestCase):
    def test_fixed_recovery_partition_and_right_censor(self):
        t = np.arange(30)*.1
        ready = np.ones(30, bool)
        ready[:2] = False
        ready[4:6] = False
        ready[20:] = False
        labels, episodes = partition(ready,t)
        self.assertEqual(episodes, [(4,6),(20,None)])
        self.assertTrue(np.all(labels[:2]=='startup'))
        self.assertEqual(labels[3], 'normal')
        self.assertTrue(np.all(labels[4:17]=='recovery'))
        self.assertEqual(labels[17], 'normal')
        self.assertTrue(np.all(labels[20:]=='recovery'))
        labels, episodes = partition(np.zeros(30,bool),t)
        self.assertTrue(np.all(labels=='startup'))
        self.assertEqual(episodes, [])

    def test_final_ready_not_counted_as_revoke(self):
        r = np.array([False,True,True,False,True,True])
        v = toggles(r,np.arange(6)*.1,np.ones(6,bool))
        self.assertEqual(v['ready_on_transitions'], 2)
        self.assertEqual(v['ready_off_transitions'], 1)
        self.assertEqual(v['ready_segments_under_1s'], 2)

    def test_quality_missing_and_predeclared_deterioration(self):
        base = quality(np.array([.04,.04,np.nan]),np.ones(3,bool))
        self.assertEqual(base['target'],3)
        self.assertEqual(base['evaluated'],2)
        self.assertEqual(base['missing'],1)
        bad = quality(np.array([.08,.08]),np.ones(2,bool))
        self.assertTrue(deterioration(base,bad)['rmse_25pct_and_0_01'])
        self.assertFalse(deterioration(base,base)['rmse_25pct_and_0_01'])
        empty = quality(np.array([np.nan]),np.ones(1,bool))
        self.assertFalse(any(deterioration(base,empty).values()))
        # Empty admission is handled separately as absence of engineering evidence.

    def test_actual_requires_consumption_rows_joint_call(self):
        e=dict(consumed=1,V_rows=3,submit_reason='joint_applied',ekf_calls=1)
        self.assertTrue(applied(e))
        for key,value in [('consumed',0),('V_rows',0),('submit_reason','stale_prior'),('ekf_calls',0)]:
            changed=dict(e);changed[key]=value
            self.assertFalse(applied(changed))


if __name__=='__main__':
    unittest.main()
