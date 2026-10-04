"""Small generation-only contract checks; run under the new shared budget."""
import tempfile
import json
import unittest
from unittest.mock import patch
from pathlib import Path
import numpy as np
import synthetic_inputs as generate


class SyntheticInputContract(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.calibration = generate.ROOT/'config/ltv_euroc/kalibr_imucam_chain.yaml'

    def tearDown(self):
        self.temp.cleanup()

    def data(self, condition):
        directory = self.root/condition
        generate.generate(directory, self.calibration, 'REGULAR', 42, .5, condition, .2)
        with np.load(directory/'inputs.npz') as data:
            inputs = {k:data[k] for k in data.files}
        with np.load(directory/'labels.npz') as data:
            labels = {k:data[k] for k in data.files}
        return inputs, labels

    def test_csr_visibility_and_no_label_leakage(self):
        inputs, labels = self.data('STEREO')
        self.assertEqual(inputs['observation_offsets'].shape, (6,))
        self.assertEqual(inputs['observation_offsets'][-1], len(inputs['feature_ids']))
        forbidden = {'phase','lifetime','points_W','R_WB','p_WB','wrong_match','wrong_time',
                     'observation_physical_ids','stereo_opportunities','temporal_opportunities','pose_noise_camera'}
        self.assertFalse(forbidden & inputs.keys())
        cameras, _ = generate.calibration(self.calibration)
        world = dict(zip(labels['point_ids'], labels['points_W']))
        for tick, t in enumerate(inputs['camera_times']):
            begin, end = inputs['observation_offsets'][tick:tick+2]
            for index in range(begin,end):
                camera = inputs['camera_ids'][index]
                self.assertEqual(inputs['actual_ns'][index], inputs['camera_ns'][tick])
                point = world[labels['observation_physical_ids'][index]]
                local = (labels['R_WB'][tick].T@(point-labels['p_WB'][tick])-inputs['p_BC'][camera])@inputs['R_BC'][camera]
                self.assertTrue(generate.visible(local,cameras[camera]))
                self.assertAlmostEqual(np.linalg.norm(inputs['bearings'][index]),1.,places=12)
        self.assertTrue(len(labels['stereo_opportunities']))

    def test_real_extrinsics_and_shared_pose_noise(self):
        inputs, labels = self.data('STEREO')
        _, transform = generate.calibration(self.calibration)
        np.testing.assert_array_equal(inputs['R_BC'],transform[:,:3,:3])
        np.testing.assert_array_equal(inputs['p_BC'],transform[:,:3,3])
        self.assertFalse(np.array_equal(inputs['R_BC'][0],inputs['R_BC'][1]))
        baseline = np.linalg.norm(inputs['p_BC'][1]-inputs['p_BC'][0])
        self.assertGreater(baseline,.10);self.assertLess(baseline,.12)
        for k, eps in enumerate(labels['pose_noise_camera']):
            R = labels['R_WB'][k]
            expected = R@inputs['R_BC'][0]@generate.math.rot(eps[3:])@inputs['R_BC'][0].T
            np.testing.assert_allclose(inputs['prior_R_WB'][k],expected,atol=1e-14)
            expected_center=labels['p_WB'][k]+R@inputs['p_BC'][0]+eps[:3]
            np.testing.assert_allclose(inputs['prior_p_WB'][k]+expected@inputs['p_BC'][0],expected_center,atol=1e-14)
        # Exactly one prior per camera event, shared by all points and both cameras.
        self.assertEqual(len(inputs['prior_R_WB']),len(inputs['camera_times']))

    def test_temporal_and_negative_timestamps(self):
        temporal, labels = self.data('TEMPORAL')
        self.assertTrue(np.all(temporal['camera_ids']==0))
        self.assertEqual(len(labels['stereo_opportunities']),0)
        self.assertTrue(len(labels['temporal_opportunities']))
        wrong, labels = self.data('TIME_ERROR')
        self.assertTrue(labels['wrong_time'].any())
        for k, ns in enumerate(wrong['camera_ns']):
            begin,end=wrong['observation_offsets'][k:k+2]
            indices=np.arange(begin,end)
            mask=labels['wrong_time'][begin:end]
            self.assertTrue(np.all(wrong['actual_ns'][indices[mask]]==ns-50000000))
            self.assertTrue(np.all(wrong['camera_ids'][indices[mask]]==1))

    def test_wrong_match_truth_is_evaluator_only(self):
        inputs, labels = self.data('WRONG_MATCH')
        self.assertTrue(labels['wrong_match'].any())
        np.testing.assert_array_equal(inputs['feature_ids']!=labels['observation_physical_ids'],labels['wrong_match'])
        self.assertNotIn('wrong_match',inputs)

    def test_recovery_supply_model_counterexamples(self):
        cameras=np.array([[-.055,0.,0.],[.055,0.,0.]])
        point=np.array([0.,0.,3.])
        good,risk=generate.stereo_supply_qualified(point,cameras,True,3,.1)
        self.assertTrue(good);self.assertLess(risk,.05)
        self.assertFalse(generate.stereo_supply_qualified(point,np.zeros((2,3)),True,3,.1)[0])
        self.assertFalse(generate.stereo_supply_qualified(point,cameras,False,3,.1)[0]) # wrong match or asynchronous pair
        self.assertFalse(generate.stereo_supply_qualified(point,cameras,True,2,.05)[0])
        self.assertFalse(generate.stereo_supply_qualified([0,0,30.],cameras,True,3,.1)[0])
        self.assertFalse(generate.sufficient_supply(14));self.assertTrue(generate.sufficient_supply(15))
        inputs,labels=self.data('TIME_ERROR')
        wrong_ids=set(inputs['feature_ids'][labels['wrong_time']].tolist())
        self.assertFalse(wrong_ids&set(labels['reliable_supply_feature_ids'][:,1].tolist()))
        self.assertNotIn('sufficient_reliable_supply',inputs)

    def test_negative_heavy_tail_fixed_profile(self):
        directions=np.tile(np.array([0.,0.,1.]),(2,1000,1))
        changed,mask=generate.heavy_tail_negative(directions,np.random.default_rng(np.random.SeedSequence([42,57031])))
        self.assertTrue(mask.any())
        np.testing.assert_array_equal(changed[~mask],directions[~mask])
        np.testing.assert_allclose(np.linalg.norm(changed,axis=-1),1.,atol=1e-14)
        again,again_mask=generate.heavy_tail_negative(directions,np.random.default_rng(np.random.SeedSequence([42,57031])))
        np.testing.assert_array_equal(changed,again);np.testing.assert_array_equal(mask,again_mask)
        with patch.object(generate,'heavy_tail_negative',side_effect=AssertionError('Normal condition used negative profile')):
            normal,_=self.data('STEREO')
        self.assertNotIn('heavy_tail_outlier',normal)
        actual_masks=[]
        real_profile=generate.heavy_tail_negative
        def capture(bearings,rng):
            values,flags=real_profile(bearings,rng);actual_masks.append(flags.copy());return values,flags
        with patch.object(generate,'heavy_tail_negative',side_effect=capture):
            inputs,labels=self.data('WRONG_MATCH')
        self.assertNotIn('heavy_tail_outlier',inputs)
        for k in range(len(inputs['camera_times'])):
            start,stop=inputs['observation_offsets'][k:k+2]
            for index in range(start,stop):
                camera=inputs['camera_ids'][index];source_slot=int(labels['observation_physical_ids'][index]%30)
                self.assertEqual(labels['heavy_tail_outlier'][index],actual_masks[k][camera,source_slot])

    def test_frozen_normal_input_sha_unchanged(self):
        frozen=Path('/home/he/output/ltv_passive_hardening_v2/inputs/dev_near_REGULAR_42_05/inputs.npz')
        if not frozen.is_file():
            self.skipTest('Immutable development input 52 is not installed')
        expected='0c19416e7354b80968cbcc6fb38428aa954ccd50b75611ce572302e549cb870a'
        self.assertEqual(generate.sha(frozen),expected)
        directory=self.root/'normal_sha_check'
        generate.generate(directory,self.calibration,'REGULAR',42,.5,'STEREO',60.)
        self.assertEqual(generate.sha(directory/'inputs.npz'),expected)

    def test_matching_freeze_wrong_confirmation_group_rejected(self):
        fake_root=self.root/'fake_repository'
        frozen=fake_root/'docs/ltv/passive_hardening_v2/frozen_config.json'
        frozen.parent.mkdir(parents=True)
        frozen.write_text(json.dumps({'confirmation_seeds':[201,202]}))
        target=self.root/'must_not_generate_confirmation'
        with patch.object(generate,'ROOT',fake_root):
            with self.assertRaisesRegex(ValueError,'outside the current frozen group'):
                generate.generate(target,self.calibration,'REGULAR',301,.5,'STEREO',.2,generate.sha(frozen))
        self.assertFalse(target.exists())


if __name__ == '__main__':
    unittest.main()
