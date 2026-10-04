import importlib.util
from pathlib import Path
import unittest
import tempfile
import numpy as np
spec=importlib.util.spec_from_file_location('new_real_eval',Path(__file__).resolve().parents[1]/'real_evaluate.py');e=importlib.util.module_from_spec(spec);spec.loader.exec_module(e)
class Contracts(unittest.TestCase):
 def test_episode_gap_and_epoch(self):
  self.assertEqual(len(e.episodes([0,.05,5,5.05],[1,1,1,1])),2)
  self.assertEqual(len(e.episodes([0,.05,.1],[1,1,1],[1,1,2])),2)
 def test_five_second_boundary(self):
  t=np.arange(101)*.05
  self.assertEqual(e.episodes(t,np.ones(101,bool))[0]['duration'],5.)
  self.assertLess(e.episodes(t[:-1],np.ones(100,bool))[0]['duration'],5.)
 def test_ready_without_raw(self):
  row=dict(camera_ns=1,epoch=1,raw_current=False,v_body=None,eta_body=None,ready_G=True,ready_V=False)
  with self.assertRaisesRegex(ValueError,'Ready declaration'):e.align([row],[1],[0],True)
 def test_raw_stale_or_nonfinite(self):
  row=dict(camera_ns=1,epoch=1,raw_current=True,imu_time=0,v_body=[0,0,0],eta_body=[0,0,-9.81],ready_G=False,ready_V=False)
  for edit in ({'imu_time':1},{'v_body':[np.nan,0,0]}):
   with self.assertRaises(ValueError):e.align([{**row,**edit}],[1],[0],True)
 def test_invalid_gravity_cannot_be_filtered(self):
  row=dict(camera_ns=1,epoch=1,raw_current=True,imu_time=0,v_body=[0,0,0],eta_body=[0,0,0],ready_G=True,ready_V=False)
  with self.assertRaisesRegex(ValueError,'physically'):e.align([row],[1],[0],True)
 def test_p95_and_severe(self):
  x=np.array([0.,0.,0.,2.]);z=np.zeros(4)
  err=dict(v=x,eta=z,g_angle_deg=z,g_magnitude=z,v_components=np.zeros((4,3)),eta_components=np.zeros((4,3)))
  m=e.summarize(err,np.ones(4,bool),np.ones(4))
  self.assertAlmostEqual(m['v_P95'],1.7);self.assertEqual(m['severe_V_fraction'],.25)
 def test_missing_does_not_fill_zero(self):
  row=dict(camera_ns=1,epoch=1,raw_current=False,v_body=None,eta_body=None,ready_G=False,ready_V=False,availability_state='DORMANT')
  o=e.align([row],[1],[0],True)
  self.assertFalse(o['finite_current'][0]);self.assertTrue(np.isnan(o['v']).all())
 def test_config_only_new_hardening_controls(self):
  with tempfile.TemporaryDirectory() as folder:
   a=Path(folder)/'old.yaml';b=Path(folder)/'new.yaml'
   a.write_text('%YAML:1.0\nltv_q: 0.0001\nltv_feature_seed_source: STEREO_THEN_TEMPORAL\n')
   b.write_text(a.read_text()+'ltv_passive_hardening_enabled: true\n')
   self.assertTrue(e.config_equivalence(a,b)['nonhardening_yaml_exact'])
   for changed in ('ltv_q: 0.001\n','ltv_feature_seed_source: TEMPORAL_POSE\n','ltv_feature_max_relative_risk: 0.1\n'):
    b.write_text(a.read_text()+changed)
    with self.assertRaisesRegex(ValueError,'Non-hardening'):e.config_equivalence(a,b)
 def test_historical_gravity_health_not_retroactively_changed(self):
  row=dict(camera_ns=1,epoch=1,available=True,observer_started=True,imu_time=0,v_body=[0,0,0],eta_body=[0,0,-7.5],ready_G=True,ready_V=True)
  self.assertTrue(e.align([row],[1],[0],False)['ready_G'][0])
 def test_thresholds_inclusive(self):
  self.assertTrue(e.threshold(.2,.2));self.assertFalse(e.threshold(.2000001,.2));self.assertFalse(e.threshold(None,.2))
if __name__=='__main__':unittest.main()
