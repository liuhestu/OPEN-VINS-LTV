"""Offline aggregation contracts; no observer simulation."""
import unittest
import numpy as np
from paper_review import settling


def records(times, sides=None):
    result={'t':np.array(times,dtype=float),'values':np.zeros((len(times),6)),
            'landmarks':np.zeros((len(times),1,6))}
    if sides is not None:
        result['side']=np.array(sides)
    return result


class AggregationContract(unittest.TestCase):
    def test_preupdate_failure_cannot_be_hidden_by_postupdate(self):
        trace=records(np.arange(0,11,.005))
        camera=records([5,5],['pre','post'])
        camera['values'][0,0]=.2
        result=settling(trace,camera,10.5,1)
        self.assertAlmostEqual(result['wide']['t_VG'],5.005)

    def test_five_second_verification_not_rounded_up(self):
        trace=records(np.arange(4001)/200)
        camera=records([15.2,15.2],['pre','post'])
        camera['values'][0,0]=.06
        result=settling(trace,camera,20,1)
        self.assertEqual(result['strict']['t_VG'],'NOT_OBSERVED_WITHIN_HORIZON')
        self.assertEqual(result['wide']['t_VG'],0.)

    def test_common_horizon_does_not_use_later_failures(self):
        trace=records(np.arange(6001)/200)
        trace['values'][-1,0]=1
        camera=records([],[])
        self.assertEqual(settling(trace,camera,20,1)['wide']['t_VG'],0.)
        self.assertEqual(settling(trace,camera,30,1)['wide']['t_VG'],'NOT_OBSERVED_WITHIN_HORIZON')


if __name__=='__main__':
    unittest.main()
