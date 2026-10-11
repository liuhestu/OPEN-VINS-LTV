"""Isolated preflight/config/checker fixtures; never invokes a replay process."""
import importlib.util,json,sys,tempfile,unittest
from pathlib import Path
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import real_run as runner
import real_evaluate as evaluator
import check_live_log as checker
class Review(unittest.TestCase):
 def setUp(self):
  self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup);self.p=Path(self.tmp.name)
 def check(self,rows):
  p=self.p/'features';p.write_text(''.join(json.dumps(r)+'\n' for r in rows));return checker.check(p)
 def test_empty_and_no_management_rejected(self):
  for rows in ([],[{'management_present':False}]):
   with self.assertRaises(ValueError):self.check(rows)
 def test_duplicate_keys_rejected(self):
  p=self.p/'features';p.write_text('{"management_present":false,"management_present":false}\n')
  with self.assertRaises(ValueError):checker.check(p)
 def test_only_active_config_changes_allowed(self):
  a=self.p/'old.yaml';b=self.p/'new.yaml';a.write_text('ltv_observer_q_landmark: 0.0001\nmain_option: 7\n');b.write_text(a.read_text()+'ltv_active_consistency_enabled: true\nltv_active_consistency_max_holdout_residual_rad: 0.04\n')
  self.assertTrue(evaluator.config_equivalence(a,b)['non_R4_yaml_exact'])
  for text in (b.read_text().replace('main_option: 7','main_option: 8'),b.read_text().replace('0.0001','0.0002')):
   b.write_text(text)
   with self.assertRaises(ValueError):evaluator.config_equivalence(a,b)
 def test_previous_already_enabled_rejected(self):
  a=self.p/'a';b=self.p/'b';a.write_text('ltv_active_consistency_enabled: true\n');b.write_text(a.read_text())
  with self.assertRaises(ValueError):evaluator.config_equivalence(a,b)
 def fixture(self):
  doc=self.p/'doc';doc.mkdir();runtime=self.p/'runtime';runtime.mkdir();config=self.p/'config';config.mkdir()
  metadata_root=self.p/'metadata';metadata_root.mkdir()
  (metadata_root/'input_metadata.json').write_text(json.dumps({'sequences':{'TEST':{'sensors':{},'root':'UNUSED_NO_REPLAY'}}}))
  (runtime/'manifest.json').write_text('{}');(config/'estimator_config.yaml').write_text('ltv_enabled: true\n')
  for k in ('protocol','acceptance'):(doc/(k+'.json')).write_text('{}')
  source=self.p/'source.py';source.write_text('fixed')
  frozen={'angle':.04,'sequences':['TEST'],'runtime':str(runtime),'manifest_sha':runner.artifacts.sha(runtime/'manifest.json'),'base_config_tree_sha':runner.artifacts.tree(config),'protocol_sha':runner.artifacts.sha(doc/'protocol.json'),'acceptance_sha':runner.artifacts.sha(doc/'acceptance.json'),'source_sha':{'source.py':runner.artifacts.sha(source)},'input_metadata_sha':runner.artifacts.sha(metadata_root/'input_metadata.json'),'allow_passive_off_controls':True}
  (doc/'frozen_config.json').write_text(json.dumps(frozen));return doc,runtime,config,frozen,metadata_root
 def guard(self,mutate=None,angle=.04,enabled=True,sequence='TEST',expected='frozen',positive=False,missing=False):
  doc,runtime,config,frozen,metadata_root=self.fixture()
  if mutate:mutate(doc,runtime,config,frozen,metadata_root)
  class PreflightReached(Exception):pass
  with patch.object(runner.budget,'DOC',doc),patch.object(runner.budget,'ROOT',self.p),patch.object(runner.budget,'OUT',self.p/'out'),patch.object(runner,'OLD',metadata_root),patch.object(runner.artifacts,'verify',side_effect=PreflightReached('EXACT_VERIFY_SENTINEL')) as verify,patch.object(runner.budget,'run',side_effect=AssertionError('REAL_PROCESS_FORBIDDEN')) as run:
   if positive:
    with self.assertRaisesRegex(PreflightReached,'EXACT_VERIFY_SENTINEL'):
     runner.run('isolated',sequence,config,runtime,enabled,angle,'confirmation')
    verify.assert_called_once_with(runtime)
   else:
    error=FileNotFoundError if missing else ValueError
    with self.assertRaisesRegex(error,expected):
     runner.run('isolated',sequence,config,runtime,enabled,angle,'confirmation')
    verify.assert_not_called()
   run.assert_not_called()
  self.assertFalse((self.p/'out').exists())
 def test_valid_on_reaches_verify(self):self.guard(positive=True)
 def test_valid_off_reaches_verify(self):self.guard(enabled=False,positive=True)
 def test_missing_freeze(self):self.guard(lambda d,r,c,f,m:(d/'frozen_config.json').unlink(),expected='frozen_config.json',missing=True)
 def test_wrong_angle(self):self.guard(angle=.08,expected='frozen candidate mismatch')
 def test_off_not_authorized(self):
  def prohibit(d,r,c,f,m):
   f['allow_passive_off_controls']=False;(d/'frozen_config.json').write_text(json.dumps(f))
  self.guard(prohibit,enabled=False,expected='frozen OFF control not authorized')
 def test_on_does_not_require_off_permission(self):
  def prohibit(d,r,c,f,m):
   f['allow_passive_off_controls']=False;(d/'frozen_config.json').write_text(json.dumps(f))
  self.guard(prohibit,positive=True)
 def test_wrong_sequence(self):self.guard(sequence='OTHER',expected='frozen candidate mismatch')
 def test_changed_metadata(self):self.guard(lambda d,r,c,f,m:(m/'input_metadata.json').write_text('{}'),expected='frozen input metadata changed')
 def test_changed_source(self):self.guard(lambda d,r,c,f,m:(self.p/'source.py').write_text('changed'),expected='frozen execution source changed: source.py')
 def test_changed_manifest(self):self.guard(lambda d,r,c,f,m:(r/'manifest.json').write_text('{"changed":1}'),expected='frozen runtime mismatch')
 def test_changed_config(self):self.guard(lambda d,r,c,f,m:(c/'estimator_config.yaml').write_text('ltv_enabled: false\n'),expected='frozen base config mismatch')
 def test_changed_protocol(self):self.guard(lambda d,r,c,f,m:(d/'protocol.json').write_text('{"changed":1}'),expected='frozen contract changed')
 def test_changed_acceptance(self):self.guard(lambda d,r,c,f,m:(d/'acceptance.json').write_text('{"changed":1}'),expected='frozen contract changed')
if __name__=='__main__':unittest.main()
