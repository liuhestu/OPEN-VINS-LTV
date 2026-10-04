"""Development-only numerical identity check; never generate confirmation data."""
import importlib.util
import tempfile
import unittest
from pathlib import Path
import numpy as np
import pressure_inputs as producer

class PressureContract(unittest.TestCase):
    def test_original_formula_and_bridge_exact_arrays(self):
        original=producer.ROOT/'scripts/ltv_feature_passive/synthetic/generate.py'
        spec=importlib.util.spec_from_file_location('original_far_generator',original)
        old=importlib.util.module_from_spec(spec);spec.loader.exec_module(old)
        with tempfile.TemporaryDirectory() as temporary:
            root=Path(temporary)
            old.generate(root/'old','REGULAR',42,.5,.2)
            result=producer.generate(root/'new',42,.2)
            for filename in ['inputs.npz','labels.npz']:
                with np.load(root/'old'/filename) as a,np.load(root/'new_source'/filename) as b:
                    self.assertEqual(set(a.files),set(b.files))
                    for key in a.files:np.testing.assert_array_equal(a[key],b[key],err_msg=key)
            with np.load(root/'old/inputs.npz') as oldinput,np.load(root/'new/inputs.npz') as current:
                np.testing.assert_array_equal(current['imu_sample_times'][1:-1],(np.arange(len(oldinput['imu']))+.5)/200)
                np.testing.assert_array_equal(current['imu_samples'][1:-1],oldinput['imu'])
                np.testing.assert_array_equal(current['bearings'][current['camera_ids']==0],oldinput['bearings_left'].reshape(-1,3))
                np.testing.assert_array_equal(current['bearings'][current['camera_ids']==1],oldinput['bearings_right'].reshape(-1,3))
                self.assertNotIn('points_W',current.files)
                self.assertNotIn('state_body',current.files)
                np.testing.assert_array_equal(current['prior_R_WB'],oldinput['prior_R_WB'])
                np.testing.assert_array_equal(current['prior_p_WB'],oldinput['prior_p_WB'])
            self.assertFalse(result['unseen_confirmation'])
            self.assertEqual(result['original_generator_sha'],producer.sha(original))

    def test_unfrozen_confirmation_rejected_without_generation(self):
        with self.assertRaises(ValueError):producer.validate_seed(201,'deliberately-invalid-sha')
        with self.assertRaises(ValueError):producer.validate_seed(999,None)
        self.assertIsNone(producer.validate_seed(42))
        # The resource entry point checks the same freeze before touching its output/library.
        import resource_benchmark
        with tempfile.TemporaryDirectory() as directory:
            target=Path(directory)/'must_not_exist'
            with self.assertRaises(ValueError):resource_benchmark.run(target,'unused','unused',seed=201,confirmation_freeze_sha='invalid')
            self.assertFalse(target.exists())

if __name__=='__main__':unittest.main()
