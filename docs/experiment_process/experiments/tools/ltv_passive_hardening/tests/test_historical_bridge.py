import json
import tempfile
import unittest
import sys
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from bridge_historical_inputs import bridge,sha

class BridgeTest(unittest.TestCase):
    def fixture(self, path):
        path.mkdir();times=np.arange(41)/200;cam=np.arange(5)/20
        ids=np.array([[2,7],[2,7],[2,7],[12,7],[12,7]],dtype=np.int64)
        imu=np.arange(240,dtype=float).reshape(40,6)/17
        bearings=np.tile([.2,.3,.9327379053],(5,2,1))
        inputs=dict(imu_times=times,imu=imu,camera_times=cam,feature_ids=ids,bearings_left=bearings,bearings_right=-bearings,
                    prior_R_WB=np.tile(np.eye(3),(5,1,1)),prior_p_WB=np.arange(15,dtype=float).reshape(5,3),
                    R_BC=np.tile(np.eye(3),(2,1,1)),p_BC=np.array([[.1,0,0],[.21,0,0]]),
                    pose_model_std=np.ones(6)*.1,pose_model_jitter=np.ones(6)*.01,
                    pose_correlation_seconds=np.array(.5),bearing_sigma_rad=np.array(.001))
        labels=dict(times=times,state_body=np.arange(41*12,dtype=float).reshape(41,12),camera_times=cam,
                    R_WB=inputs['prior_R_WB'],p_WB=inputs['prior_p_WB'],points_W=np.array([[0,0,12.],[1,1,13.]]),
                    feature_ids=ids,physical_indices=np.tile([0,1],(5,1)),pose_noise_camera=np.zeros((5,6)))
        np.savez_compressed(path/'inputs.npz',**inputs);np.savez_compressed(path/'labels.npz',**labels)
        identity=dict(scene='REGULAR',seed=101,lifetime=.5,duration=.2,imu_hz=200,camera_hz=20,
                      input_sha=sha(path/'inputs.npz'),labels_sha=sha(path/'labels.npz'))
        (path/'identity.json').write_text(json.dumps(identity))
        return inputs,labels

    def test_lossless_arrays_true_times_padding_and_labels(self):
        with tempfile.TemporaryDirectory() as d:
            source=Path(d)/'source';out=Path(d)/'out';old,truth=self.fixture(source)
            identity=bridge(source,out)
            for name in ['inputs.npz','labels.npz','identity.json']:
                self.assertEqual((source/name).read_bytes(),(out/('historical_'+name)).read_bytes())
            with np.load(out/'inputs.npz') as data:
                for key,value in old.items():
                    mapped='historical_feature_ids' if key=='feature_ids' else key
                    np.testing.assert_array_equal(data[mapped],value)
                np.testing.assert_array_equal(data['imu_samples'][1:-1],old['imu'])
                np.testing.assert_array_equal(data['imu_sample_times'][1:-1],(np.arange(40)+.5)/200)
                self.assertEqual(data['imu_sample_times'][0],-.0025);self.assertEqual(data['imu_sample_times'][-1],.2025)
                np.testing.assert_array_equal(data['imu_samples'][[0,-1]],old['imu'][[0,-1]])
                np.testing.assert_array_equal(data['feature_ids'].reshape(5,2,2)[:,:,0],old['feature_ids'])
                np.testing.assert_array_equal(data['feature_ids'].reshape(5,2,2)[:,:,1],old['feature_ids'])
                np.testing.assert_array_equal(data['bearings'].reshape(5,2,2,3)[:,:,0],old['bearings_left'])
                np.testing.assert_array_equal(data['bearings'].reshape(5,2,2,3)[:,:,1],old['bearings_right'])
                for t in old['camera_times']:
                    self.assertTrue(np.any(data['imu_sample_times']<t));self.assertTrue(np.any(data['imu_sample_times']>t))
            with np.load(out/'labels.npz') as labels:
                np.testing.assert_array_equal(labels['velocity_gravity_body'],truth['state_body'][:,-6:])
                self.assertEqual(labels['temporal_opportunities'].tolist(),[[2,2],[2,7],[3,7],[4,7]])
                points=dict(zip(labels['point_ids'],labels['points_W']))
                np.testing.assert_array_equal(points[12],truth['points_W'][0])
            self.assertTrue(identity['negative_control']);self.assertFalse(identity['unseen_confirmation'])
            self.assertNotEqual(identity['input_sha'],identity['historical_input_sha'])
            with self.assertRaises(FileExistsError):bridge(source,out)

    def test_identity_mismatch_refused_without_output(self):
        with tempfile.TemporaryDirectory() as d:
            source=Path(d)/'source';out=Path(d)/'out';self.fixture(source)
            with (source/'inputs.npz').open('ab') as f:f.write(b'changed')
            with self.assertRaisesRegex(ValueError,'SHA mismatch'):bridge(source,out)
            self.assertFalse(out.exists())

if __name__=='__main__':unittest.main()
