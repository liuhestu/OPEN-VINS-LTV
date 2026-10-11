import unittest
import json
import tempfile
from pathlib import Path
from audit import covariance_evidence
import numpy as np
from analyze import labels, paired_group, match_times


class SupportTests(unittest.TestCase):
    def row(self, ns, l=.02, m=.01, anchor=0., mature=True):
        return dict(camera_ns=str(ns),ltv_valid='1',main_valid='1',anchor_time=str(anchor),
                    ltv_angle_rad=str(l),main_angle_rad=str(m),reason='ok',past_mature=str(int(mature)))

    def test_startup_without_seed_fields(self):
        with tempfile.TemporaryDirectory() as folder:
            old=Path(folder)
            seed=dict(admitted=1,mean_written=1,covariance_B=(np.eye(3)*.01).reshape(-1).tolist())
            (old/'features.jsonl').write_text(json.dumps({})+'\n'+json.dumps({'seeds':[seed]})+'\n')
            result=covariance_evidence(old,{'ltv_observer_initial_p_landmark':1.})
            self.assertEqual(result['missing_seed_logs'],1)
            self.assertEqual(result['admitted_written_seeds'],1)
            self.assertEqual(result['seed_trace_equal_observer_birth_trace'],0)

    def test_clock_mapping(self):
        np.testing.assert_array_equal(match_times([1000256,2000000],[1000000,2000000]),[1000000,2000000])
        with self.assertRaises(ValueError):match_times([1002000],[1000000])
        with self.assertRaises(ValueError):match_times([1000000,1000256],[1000000])

    def test_missing_denominator(self):
        rows=[self.row(1),self.row(2)]
        rows[1]['ltv_valid']='0';rows[1]['reason']='invalid_prediction'
        g=paired_group(rows,[True,True])
        self.assertEqual((g['denominator'],g['paired'],g['main_only']),(2,1,1))
        self.assertFalse(g['reliable_advantage'])

    def test_no_anchor_not_paired(self):
        g=paired_group([self.row(1,anchor=float('nan'))],[True])
        self.assertEqual((g['denominator'],g['paired'],g['neither']),(1,0,1))

    def test_equal_frame_weight_and_reproducibility(self):
        rows=[self.row(i*100000000, l=.005, m=.01) for i in range(300)]
        a=paired_group(rows,np.ones(300,bool));b=paired_group(rows,np.ones(300,bool))
        self.assertEqual(a,b);self.assertTrue(a['reliable_advantage'])
        self.assertAlmostEqual(a['frame_rms_gain'],.5)

    def test_worse_tail_rejects(self):
        rows=[self.row(i*100000000,l=.03,m=.01) for i in range(300)]
        g=paired_group(rows,np.ones(300,bool))
        self.assertFalse(g['reliable_advantage']);self.assertFalse(g['tail_nonworse'])

    def test_phase_is_frozen_off(self):
        fs=[dict(camera_ns=int(i*1e9),ready_V=x) for i,x in enumerate([0,1,0,1,1,1])]
        self.assertEqual(list(labels(fs).values()),['startup','normal','recovery','recovery','recovery','normal'])


if __name__=='__main__':
    unittest.main()
