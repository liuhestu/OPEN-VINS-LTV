"""Supplement control/aggregation tests; never launches an observer."""
import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch
from closeout_review import MATRIX,recover_interrupted
import numpy as np
from run_study import config
from closeout_analysis import interval


class CloseoutContracts(unittest.TestCase):
    def test_only_Q_changes_in_C2(self):
        for name,scene in [('E3','REGULAR'),('E4','FAST')]:
            baseline=config('EXPERIMENTAL',scene,lifetime=1)
            changed={k for k in baseline if baseline[k]!=MATRIX[name][k]}
            self.assertEqual(changed,{'q','candidate'})
            self.assertEqual(MATRIX[name]['q'],10*baseline['q'])

    def test_interrupted_attempt_is_retained(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d);history=[{'id':str(i),'status':'COMPLETED'} for i in range(58)]
            attempted={'id':'59','status':'RUNNING','child_pid':None,'directory':str(p/'59')}
            (p/'ledger.json').write_text(json.dumps(history+[attempted]))
            recover_interrupted(p)
            after=json.loads((p/'ledger.json').read_text())
            self.assertEqual(after[:58],history);self.assertEqual(len(after),59)
            self.assertEqual(after[-1]['status'],'INTERRUPTED')

    def test_dynamic_padding_does_not_change_error_denominators(self):
        t=np.array([0.,.005]);v=np.zeros((2,6));land=np.full((2,30,6),np.nan)
        land[0,:16,0]=2.;land[1,:18,0]=2.
        land[0,:16,1]=.01;land[1,:18,1]=.01
        land[0,:16,2]=1.;land[1,:18,2]=1.
        land[0,:16,4]=1.;land[1,:18,4]=1.
        with patch('closeout_analysis.load',return_value={'t':t,'values':v,'landmarks':land}):
            result=interval({'directory':'unused'},0,.005)
        self.assertEqual(result['landmark_rmse'],2.)
        self.assertEqual(result['positive_ray_fraction'],1.)
        self.assertEqual(result['near_center_fraction'],0.)


if __name__=='__main__':unittest.main()
