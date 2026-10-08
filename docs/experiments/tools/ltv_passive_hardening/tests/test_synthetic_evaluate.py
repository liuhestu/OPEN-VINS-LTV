"""Independent small-array metric contracts, no LTV/GT generator execution."""
import sys
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import numpy as np
from synthetic_evaluate import episodes,coverage,errors,summary,recovery_metrics,aligned,regression_metric

class MetricsTest(unittest.TestCase):
    def test_final_sample_has_no_extrapolated_duration(self):
        t=np.array([0.,1.,2.]);c=coverage(t,[False,False,True])
        self.assertEqual(c['fraction'],1/3);self.assertEqual(c['left_sample_seconds'],0)
        self.assertEqual(c['longest_episode_seconds'],0)
    def test_gap_breaks_episode(self):
        es=episodes(np.arange(7),[1,1,0,1,1,1,0])
        self.assertEqual([x['duration'] for x in es],[1,2])
    def test_geometry_error_independent_vectors(self):
        truth=np.array([[0,0,0,0,0,-10],[0,0,0,0,0,-10.]])
        e=errors([[3,4,0],[0,0,0]],[[0,0,-10],[10,0,0]],truth)
        np.testing.assert_allclose(e['v'],[5,0]);np.testing.assert_allclose(e['angle'],[0,90]);np.testing.assert_allclose(e['norm'],0)
        result=summary(e,[True,False]);self.assertEqual(result['v']['RMSE'],5);self.assertEqual(result['severe_v_fraction'],1)
    def test_undefined_gravity_not_zero_error(self):
        e=errors([[0,0,0]],[[0,0,0]],[[0,0,0,0,0,-9.81]])
        self.assertTrue(np.isnan(e['angle'][0]));self.assertEqual(summary(e,[True])['angle']['invalid_samples'],1)
    def test_recovery_requires_interval_end_not_start_within_deadline(self):
        t=np.arange(0,21,.05);phase=np.where(t>=1,2,0);supply=t>=1
        e={k:np.zeros(len(t)) for k in ('v','angle','norm')}
        good=t>=7
        self.assertFalse(recovery_metrics(t,good,e,phase,supply)['pass'])
        good=t>=6
        self.assertTrue(recovery_metrics(t,good,e,phase,supply)['pass'])
    def test_missing_reliable_supply_is_not_pass(self):
        t=np.arange(20);e={k:np.zeros(20) for k in ('v','angle','norm')}
        r=recovery_metrics(t,np.ones(20,bool),e,np.full(20,2))
        self.assertFalse(r['pass']);self.assertEqual(r['episodes'][0]['status'],'UNVERIFIED_RELIABLE_SUPPLY_LABEL')
    def test_supply_gaps_cannot_be_stitched(self):
        t=np.arange(0,12,.05);e={k:np.zeros(len(t)) for k in ('v','angle','norm')}
        supply=(t<3)|(t>=4)
        result=recovery_metrics(t,np.ones(len(t),bool),e,np.full(len(t),2),supply)
        self.assertFalse(result['pass']);self.assertEqual(len(result['episodes']),2)
        self.assertTrue(result['episodes'][1]['pass'])
    def test_inconsistent_supply_run_start_rejected(self):
        t=np.arange(12);e={k:np.zeros(12) for k in ('v','angle','norm')}
        with self.assertRaisesRegex(ValueError,'run_start'):
            recovery_metrics(t,np.ones(12,bool),e,np.full(12,2),np.ones(12,bool),np.ones(12))
    def test_shared_undefined_is_not_new_regression(self):
        metric=regression_metric([np.nan,1.],[np.nan,1.],[True,True],.1,[0.,.05])
        self.assertTrue(metric['pass']);self.assertEqual(metric['new_invalid_times'],[0.])
        self.assertEqual(metric['common_finite_samples'],1)
    def test_new_only_undefined_regression_fails(self):
        metric=regression_metric([np.nan,1.],[0.,1.],[True,True],.1)
        self.assertFalse(metric['pass']);self.assertFalse(metric['identical_invalid_support'])
    def test_all_undefined_has_no_finite_evidence(self):
        metric=regression_metric([np.nan],[np.nan],[True],.1)
        self.assertFalse(metric['pass']);self.assertEqual(metric['common_finite_samples'],0)
    def test_stale_cursor_cannot_be_current(self):
        labels={'times':np.array([0.]),'velocity_gravity_body':np.array([[0,0,0,0,0,-9.81]])}
        event={'t':0.,'cursor':-.1,'raw_current':True,'raw_v':[0,0,0],'raw_eta':[0,0,-9.81],'ready_G':False,'ready_V':False}
        with self.assertRaisesRegex(ValueError,'stale'):aligned([event],labels)

if __name__=='__main__':unittest.main()
