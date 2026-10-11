"""Independent fixtures for frame, gain scattering, support and unit semantics."""
import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from analyze import angle, body, compare, inject, intervals, joint_gain, labels, reference


class DiagnosisTests(unittest.TestCase):
    def test_interval_never_bridges_missing_packets(self):
        blocks = intervals(np.ones(5, bool), np.array([0., .05, .1, .3, .35]))
        self.assertEqual([b['frames'] for b in blocks], [3, 2])

    def test_missing_state_and_native_injection(self):
        v, g = body(np.full((2, 16), np.nan))
        self.assertTrue(np.isnan(v).all() and np.isnan(g).all())
        x = np.zeros((1, 16))
        x[0, 3] = 1.
        delta = np.array([[.2, 0., 0., 1., 2., 3., 4., 5., 6.]])
        y = inject(x, delta)
        np.testing.assert_allclose(y[0, :4], np.array([.1, 0., 0., 1.])/np.sqrt(1.01))
        np.testing.assert_allclose(y[0, 4:10], [1., 2., 3., 4., 5., 6.])

    def test_native_body_frame(self):
        x = np.zeros((1, 16))
        x[0, :4] = Rotation.from_euler('x', 90, degrees=True).as_quat()
        x[0, 7:10] = [0., 1., 0.]
        v, g = body(x)
        np.testing.assert_allclose(v, [[0., 0., -1.]], atol=1e-15)
        np.testing.assert_allclose(g, [[0., -1., 0.]], atol=1e-15)
        np.testing.assert_allclose(angle(g, np.array([[0., -2., 0.]])), 0.)

    def test_gt_interpolation_and_gap(self):
        raw = np.zeros((3, 11))
        raw[:, 0] = np.array([0., .01, .03])*1e9
        raw[:, 4] = 1.
        raw[:, 8] = [0., 2., 6.]
        v, g, valid = reference(raw, np.array([.005, .02, .04]))
        np.testing.assert_array_equal(valid, [True, False, False])
        np.testing.assert_allclose(v[0], [1., 0., 0.])
        np.testing.assert_allclose(g[0], [0., 0., -1.])
        self.assertTrue(np.isnan(v[1:]).all())
        v, g, valid = reference(raw, np.array([.04, .05]))
        self.assertFalse(valid.any())
        self.assertTrue(np.isnan(v).all() and np.isnan(g).all())

    def test_complete_covariance_scatter_gain(self):
        P = np.array([[2., .5, .1], [.5, 3., .2], [.1, .2, 4.]])
        H = np.array([[1., 0., 2.], [0., 1., -1.]])
        # Serialized compact columns are in order [2, 0, 1].
        raw = dict(prior_P=P.tolist(), joint_H=H[:, [2, 0, 1]].tolist(),
                   joint_ids=[2, 0], joint_sizes=[1, 2], joint_S=(H@P@H.T+np.eye(2)).tolist())
        K = joint_gain(raw)
        np.testing.assert_allclose(K@(H@P@H.T+np.eye(2)), P@H.T, atol=1e-14)
        r = np.array([.3, -.2])
        np.testing.assert_allclose(K@r, K[:, :1]@r[:1]+K[:, 1:]@r[1:])

    def test_partition_and_missing_denominator(self):
        lab = labels(np.array([False, True, False, True]), np.array([True, True, True, False]))
        np.testing.assert_array_equal(lab, ['startup', 'weak_visual', 'interruption', 'normal'])
        c = compare(np.array([1., 2., 3.]), np.array([.5, np.nan, 4.]), np.ones(3, bool),
                    np.array([True, False, True]), np.array([True, False, False]))
        self.assertEqual(c['target_frames'], 3)
        self.assertEqual(c['missing_pairs'], 1)
        self.assertEqual(c['ltv_better_fraction'], .5)
        self.assertEqual(c['actual_better'], 1)


if __name__ == '__main__':
    unittest.main()
