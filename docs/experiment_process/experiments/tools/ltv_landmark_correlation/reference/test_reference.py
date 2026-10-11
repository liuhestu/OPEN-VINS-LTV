#!/usr/bin/env python3
"""Meaningful fault-injection and duplicate-observation reference checks."""
import unittest

import numpy as np

from run import A, H, X, ANCHOR, CURRENT, VISUAL, approximation, block_model, conditional_information, effective_inverse, source_map, verify_algebra


class CorrelationReferenceTests(unittest.TestCase):
    def test_source_map_and_block_paths(self):
        for scene in ["zero_noise", "independent_sources", "shared_imu_bias", "shared_bearing",
                      "seed_feedback", "duplicate_information", "near_rank_deficiency"]:
            with self.subTest(scene=scene):
                self.assertEqual(verify_algebra(scene, source_map(scene))["status"], "PASS")

    def test_nonzero_exact_duplicate_of_visual_has_no_conditional_information(self):
        m = source_map("shared_bearing")
        duplicate_rows = np.array([[1, 0], [0, 1], [0.5, -0.3]]) @ m[VISUAL]
        m[CURRENT] = A @ m[ANCHOR] - H @ m[X] + duplicate_rows
        q = m @ m.T
        model = block_model(q)
        self.assertEqual(model["rank"], 2)
        self.assertGreater(np.linalg.norm(model["K"]), 0.1)
        delta, rank, _ = conditional_information(q)
        self.assertEqual(rank, 0)
        self.assertLess(np.linalg.norm(delta), 1e-10)
        diagnostics = approximation("nonzero_visual_duplicate", m)
        self.assertLess(abs(diagnostics[0]["information_trace"]), 1e-10)
        self.assertGreater(diagnostics[1]["information_trace"], 0.1)

    def test_zero_cross_creates_false_confidence_for_exact_cancellation(self):
        rows = approximation("duplicate_information", source_map("duplicate_information"))
        self.assertAlmostEqual(rows[0]["posterior_trace"], rows[0]["prior_trace"])
        self.assertEqual(rows[0]["information_rank"], 0)
        self.assertLess(rows[1]["posterior_trace"], rows[1]["prior_trace"] - 0.1)

    def test_partial_cross_zero_can_make_joint_model_incompatible(self):
        joint = np.full((3, 3), 0.9)
        np.fill_diagonal(joint, 1.0)
        effective_inverse(joint)
        joint[0, 1] = joint[1, 0] = 0
        with self.assertRaisesRegex(ValueError, "incompatible"):
            effective_inverse(joint)

    def test_near_rank_uses_effective_subspace_without_ridge(self):
        model = block_model(source_map("near_rank_deficiency") @ source_map("near_rank_deficiency").T)
        self.assertEqual(model["rank"], 1)
        self.assertLess(abs(model["spectrum"][0]), 1e-10)
        self.assertTrue(np.all(np.isfinite(model["K"])))

    def test_bearing_visual_cross_block_is_nonzero(self):
        m = source_map("shared_bearing")
        model = block_model(m @ m.T)
        self.assertGreater(np.linalg.norm(model["R_L_visual"]), 0.1)


if __name__ == "__main__":
    unittest.main()
