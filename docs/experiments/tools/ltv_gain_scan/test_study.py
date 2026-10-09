#!/usr/bin/env python3
"""Protocol boundary and GT support tests independent of estimator outputs."""
import unittest
import numpy as np
from study import gate, REP, interpolate_gt

class Contract(unittest.TestCase):
    def rows(self, ate=1.1):
        return [dict(sequence=s, status='VALID_FULL_MATCHED', ate_rmse_m=ate, off_ate=1., reference_ate=1.) for s in REP]

    def test_exact_ten_percent_passes(self):
        self.assertEqual(gate(self.rows())['status'], 'PASS')

    def test_any_representative_blocks(self):
        for n in range(len(REP)):
            rows = self.rows(1.)
            rows[n]['ate_rmse_m'] = 1.10000000000001
            self.assertEqual(gate(rows)['status'], 'BLOCKED')

    def test_both_references_required(self):
        for key in ['off_ate', 'reference_ate']:
            for value in [None, 0., float('nan')]:
                rows = self.rows(1.)
                rows[0][key] = value
                self.assertEqual(gate(rows)['status'], 'BLOCKED')

    def test_invalid_run_and_missing_sequence(self):
        rows = self.rows(1.)
        rows[2]['status'] = 'FAIL'
        self.assertEqual(gate(rows)['status'], 'BLOCKED')
        self.assertEqual(gate(rows[:-1])['status'], 'BLOCKED')

    def test_uzh_gap_and_no_extrapolation(self):
        gt = np.array([[0., 0., 0., 0.], [.01, 1., 2., 3.], [.04, 4., 8., 12.]])
        valid, xyz = interpolate_gt(gt, np.array([-.001, 0., .005, .01, .02, .04, .041]))
        np.testing.assert_array_equal(valid, [False, True, True, True, False, True, False])
        np.testing.assert_allclose(xyz[2], [.5, 1., 1.5])

if __name__ == '__main__': unittest.main()
