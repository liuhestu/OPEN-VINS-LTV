import importlib.util
from pathlib import Path
import unittest
import tempfile
import json
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
 def test_R3_key_allowed_without_loosening_existing_parameters(self):
  with tempfile.TemporaryDirectory() as folder:
   a=Path(folder)/'a';b=Path(folder)/'b';a.write_text('ltv_q: 0.0001\n')
   b.write_text(a.read_text()+'ltv_hardening_preserve_constrained_state: true\n')
   self.assertTrue(e.config_equivalence(a,b)['nonhardening_yaml_exact'])
 def test_all11_and_eight_baseline_valid(self):
  records=[dict(sequence=n,baseline_valid=True,P_NEW_native_complete=True,engineering_pass=True,reference_valid=True,status='MEETS_SEQUENCE_CONTRACT') for n in e.PROTOCOL['real_final']]
  self.assertEqual(e.assess_all11(records)['status'],'OUTPUT_CONTRACT_MET')
  with self.assertRaises(ValueError):e.assess_all11(records[:-1])
  for r in records[:3]:r.update(baseline_valid=False,baseline_invalid_reason='native_initialization_failure')
  self.assertEqual(e.assess_all11(records)['baseline_valid'],8)
  records[3].update(baseline_valid=False,baseline_invalid_reason='native_initialization_failure')
  self.assertEqual(e.assess_all11(records)['status'],'BLOCKED_INPUT')
 def test_NEW_failure_cannot_disqualify_baseline(self):
  records=[dict(sequence=n,baseline_valid=True,P_NEW_native_complete=True,engineering_pass=True,reference_valid=True,status='MEETS_SEQUENCE_CONTRACT') for n in e.PROTOCOL['real_final']]
  records[0].update(baseline_valid=False,baseline_invalid_reason='NEW_error_large')
  with self.assertRaisesRegex(ValueError,'B-only'):e.assess_all11(records)
 def test_incomplete_or_new_failure_cannot_hide_in_aggregate(self):
  records=[dict(sequence=n,baseline_valid=True,P_NEW_native_complete=True,engineering_pass=True,reference_valid=True,status='MEETS_SEQUENCE_CONTRACT') for n in e.PROTOCOL['real_final']]
  records[0]['status']='NOT_MET';self.assertEqual(e.assess_all11(records)['status'],'NOT_MET')
  records[0]['P_NEW_native_complete']=False;self.assertEqual(e.assess_all11(records)['status'],'BLOCKED_CORRECTNESS')
 def test_explicit_same_input_modes_and_parameter_binding(self):
  with tempfile.TemporaryDirectory() as folder:
   root=Path(folder);sensors={}
   for sensor in ('cam0','cam1','imu0'):
    d=root/'input'/sensor;d.mkdir(parents=True);csv=d/'data.csv'
    if sensor=='imu0':csv.write_text('#imu\n1,0,0,0,0,0,9.81\n')
    else:
     (d/'data').mkdir();(d/'data/x.png').write_bytes(b'fixture');csv.write_text('#camera\n1,x.png\n')
    sensors[sensor]={'path':str(csv),'sha256':e.sha(csv)}
   paths={}
   for mode in ('B','P_PREV','P_NEW'):
    d=root/mode;d.mkdir();cfg=d/'estimator_config.yaml'
    cfg.write_text('native_parameter: 3\nltv_q: 0.0001\n'+('ltv_passive_hardening_enabled: true\n' if mode=='P_NEW' else ''))
    identity=dict(sequence='MH_01_easy',mode=mode,inputs={'sensors':sensors},config_path=str(cfg),config_sha={cfg.name:e.sha(cfg)})
    (d/'identity.json').write_text(json.dumps(identity));paths[mode]=d
   b,old,proof=e.resolve_explicit('MH_01_easy',paths['P_NEW'],paths['B'],paths['P_PREV'])
   self.assertTrue(proof['configuration_equivalence']['nonhardening_yaml_exact'])
   (paths['P_PREV']/'estimator_config.yaml').write_text('ltv_q: 0.001\n')
   with self.assertRaisesRegex(ValueError,'tree changed'):e.resolve_explicit('MH_01_easy',paths['P_NEW'],paths['B'],paths['P_PREV'])
 def test_thresholds_inclusive(self):
  self.assertTrue(e.threshold(.2,.2));self.assertFalse(e.threshold(.2000001,.2));self.assertFalse(e.threshold(None,.2))
if __name__=='__main__':unittest.main()
