"""Preflight/configuration only; never launches native replay or reads GT."""
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import real_confirm as r

class ConfirmationEntry(unittest.TestCase):
 def setUp(self):
  self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
  self.root=Path(self.temp.name);self.doc=self.root/'doc';self.out=self.root/'out';self.doc.mkdir();self.out.mkdir()
  (self.doc/'protocol.json').write_text(json.dumps({'real_final':['MH_01_easy']}))
 def fixture(self):
  runtime=self.root/'runtime';runtime.mkdir();(runtime/'manifest.json').write_text('{}')
  base=self.root/'base';base.mkdir();(base/'estimator_config.yaml').write_text('%YAML:1.0\nnative_param: 11\nltv_feature_seed_source: STEREO_THEN_TEMPORAL\nltv_enabled: true\nltv_feature_readiness_enabled: true\nltv_enable_gravity: false\nltv_enable_velocity: false\n')
  sensors={}
  for name in ('cam0','cam1','imu0'):
   p=self.root/(name+'.csv');p.write_text('sensor only');sensors[name]={'path':str(p),'sha256':r.artifacts.sha(p)}
  metadata=self.out/'input_metadata.json';metadata.write_text(json.dumps({'sequences':{'MH_01_easy':{'root':str(self.root),'sensors':sensors,'reference_metadata_only':{'path':'/NEVER/READ/GT'}}}}))
  (self.doc/'acceptance.json').write_text('{}')
  source=self.root/'source_manifest.json';source.write_text('{}')
  flags={k:True for k in r.HARDENING_FLAGS}
  frozen={'runtime':str(runtime),'manifest_sha':r.artifacts.sha(runtime/'manifest.json'),'base_config':str(base),'base_config_tree_sha':r.artifacts.tree(base),'revision':'fixture','hardening_flags':flags,'input_metadata_sha':r.artifacts.sha(metadata)}
  frozen.update(protocol_sha=r.artifacts.sha(self.doc/'protocol.json'),acceptance_sha=r.artifacts.sha(self.doc/'acceptance.json'),source_manifest=str(source),source_manifest_sha=r.artifacts.sha(source))
  (self.doc/'frozen_config.json').write_text(json.dumps(frozen));return base,flags
 def test_missing_freeze_blocks_without_output(self):
  with self.assertRaisesRegex(RuntimeError,'Main-authored'):r.preflight('run','MH_01_easy','P_NEW',self.doc,self.out)
  self.assertFalse((self.out/'real_confirmation').exists())
 def test_bound_identity_tamper_repeat_and_scope(self):
  self.fixture()
  with patch.object(r.artifacts,'verify',return_value={}):
   value=r.preflight('run','MH_01_easy','P_NEW',self.doc,self.out)
   self.assertEqual(value['inputs']['reference_metadata_only']['path'],'/NEVER/READ/GT')
   value['root'].mkdir(parents=True)
   with self.assertRaises(FileExistsError):r.preflight('run','MH_01_easy','P_NEW',self.doc,self.out)
   with self.assertRaises(ValueError):r.preflight('../bad','MH_01_easy','P_NEW',self.doc,self.out)
   with self.assertRaises(ValueError):r.preflight('new','unknown','P_NEW',self.doc,self.out)
   (self.root/'cam0.csv').write_text('changed')
   with self.assertRaisesRegex(ValueError,'sensor CSV'):r.preflight('new','MH_01_easy','P_NEW',self.doc,self.out)
 def test_scientific_contract_tamper_rejected(self):
  self.fixture()
  (self.doc/'acceptance.json').write_text('{\"relaxed\":true}')
  with self.assertRaisesRegex(ValueError,'scientific contract'):
   r.preflight('run','MH_01_easy','P_NEW',self.doc,self.out)
 def test_mode_configs_keep_native_and_distinguish_previous(self):
  base,flags=self.fixture()
  for mode in ('B','P_PREV','P_NEW'):
   config=r.configure(base,self.root/mode,mode,flags);s=config.read_text()
   self.assertIn('native_param: 11',s)
   for flag in r.HARDENING_FLAGS:self.assertIn(flag+': '+str(mode=='P_NEW').lower(),s)
   self.assertIn('ltv_enabled: '+str(mode!='B').lower(),s)
   self.assertIn('ltv_feature_readiness_enabled: '+str(mode!='B').lower(),s)
   self.assertIn('ltv_enable_gravity: false',s);self.assertIn('ltv_enable_velocity: false',s)
 def test_soft_grace_false_and_true_do_not_auto_enable(self):
  base,flags=self.fixture()
  # Even an enabled value in copied YAML must yield to the frozen explicit flag.
  with (base/'estimator_config.yaml').open('a') as out:out.write('ltv_hardening_ready_soft_grace: true\n')
  for enabled in (False,True):
   flags['ltv_hardening_ready_soft_grace']=enabled
   for mode in ('B','P_PREV','P_NEW'):
    config=r.configure(base,self.root/(mode+str(enabled)),mode,flags)
    lines=[line for line in config.read_text().splitlines() if line.startswith('ltv_hardening_ready_soft_grace:')]
    self.assertEqual(lines,['ltv_hardening_ready_soft_grace: '+str(enabled and mode=='P_NEW').lower()])
if __name__=='__main__':unittest.main()
