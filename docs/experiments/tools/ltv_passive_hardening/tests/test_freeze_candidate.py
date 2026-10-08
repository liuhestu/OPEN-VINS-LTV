"""Archive/identity contract fixtures only; no confirmation inputs are generated."""
import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import freeze_candidate as f


class FreezeContract(unittest.TestCase):
    def fixture(self, root):
        repo=root/'repo';doc=repo/'docs/ltv/passive_hardening_v2';out=root/'out'
        doc.mkdir(parents=True)
        goal=doc/'goal.md';goal.write_text('Fixture goal')
        (doc/'protocol.json').write_text(json.dumps({'goal_sha256':f.artifacts.sha(goal),'confirmation_seed_groups':[[201,202],[301,302]]}))
        (doc/'acceptance.json').write_text('{"fixed":true}')
        config=out/'logger_short/config';config.mkdir(parents=True)
        (config/'estimator_config.yaml').write_text('fixture: true\n')
        (out/'input_metadata.json').write_text('{}')
        runtime=out/'runtime';(runtime/'lib').mkdir(parents=True)
        lib=runtime/'lib/libov_msckf_lib.so';lib.write_bytes(b'fixture observer')
        (runtime/'manifest.json').write_text('{}')
        wrapper=out/'wrapper.so';wrapper.write_bytes(b'fixture wrapper')
        source=repo/'ov_msckf/src/ltv/fixture.cpp';source.parent.mkdir(parents=True);source.write_text('int fixture=1;')
        return repo,doc,out,runtime,wrapper,lib,source

    def test_archive_preserves_working_source_and_refuses_overwrite(self):
        with tempfile.TemporaryDirectory() as tmp:
            repo,doc,out,runtime,wrapper,lib,source=self.fixture(Path(tmp))
            def output(args, **kwargs):
                return 'fixture-head\n' if kwargs.get('text') else b'fixture patch'
            with patch.multiple(f.budget,ROOT=repo,DOC=doc,OUT=out), \
                 patch.object(f.artifacts,'verify',return_value={}), \
                 patch.object(f.artifacts,'dependencies',return_value={'libov_msckf_lib.so':lib}), \
                 patch.object(f.subprocess,'check_output',side_effect=output):
                frozen=f.freeze('confirmation_1',runtime,wrapper)
                self.assertEqual(frozen['confirmation_seeds'],[201,202])
                self.assertEqual(json.loads(Path(frozen['source_manifest']).read_text())['ov_msckf/src/ltv/fixture.cpp'],f.artifacts.sha(source))
                self.assertEqual(Path(frozen['wrapper']).read_bytes(),wrapper.read_bytes())
                self.assertFalse(frozen['hardening_flags']['ltv_hardening_ready_soft_grace'])
                self.assertTrue(all(value for key,value in frozen['hardening_flags'].items() if key != 'ltv_hardening_ready_soft_grace'))
                self.assertEqual(frozen['source_archive_sha'],f.artifacts.sha(frozen['source_archive']))
                with self.assertRaises(FileExistsError):f.freeze('confirmation_1',runtime,wrapper)
                self.assertEqual((out/'logger_short/config/estimator_config.yaml').read_text(),'fixture: true\n')

    def test_changed_goal_does_not_authorize_confirmation(self):
        with tempfile.TemporaryDirectory() as tmp:
            repo,doc,out,runtime,wrapper,lib,source=self.fixture(Path(tmp))
            (doc/'goal.md').write_text('changed')
            with patch.multiple(f.budget,ROOT=repo,DOC=doc,OUT=out), \
                 patch.object(f.artifacts,'verify',return_value={}), \
                 patch.object(f.artifacts,'dependencies',return_value={'libov_msckf_lib.so':lib}):
                with self.assertRaisesRegex(ValueError,'goal changed'):f.freeze('confirmation_1',runtime,wrapper)
            self.assertFalse((doc/'frozen_config.json').exists())

    def test_explicit_soft_grace_is_frozen_and_configured(self):
        import real_confirm
        for enabled in (False,True):
            with self.subTest(enabled=enabled),tempfile.TemporaryDirectory() as tmp:
                repo,doc,out,runtime,wrapper,lib,source=self.fixture(Path(tmp))
                def output(args,**kwargs):
                    return 'fixture-head\n' if kwargs.get('text') else b'fixture patch'
                with patch.multiple(f.budget,ROOT=repo,DOC=doc,OUT=out), \
                     patch.object(f.artifacts,'verify',return_value={}), \
                     patch.object(f.artifacts,'dependencies',return_value={'libov_msckf_lib.so':lib}), \
                     patch.object(f.subprocess,'check_output',side_effect=output):
                    frozen=f.freeze('confirmation_1',runtime,wrapper,ready_soft_grace=enabled)
                stored=json.loads((doc/'frozen_config.json').read_text())
                self.assertIs(stored['hardening_flags']['ltv_hardening_ready_soft_grace'],enabled)
                config=real_confirm.configure(Path(frozen['base_config']),out/'configured','P_NEW',stored['hardening_flags'])
                self.assertIn('ltv_hardening_ready_soft_grace: '+str(enabled).lower(),config.read_text())

    def test_non_boolean_soft_grace_rejected_before_freeze(self):
        for value in ('false','true',1,None):
            with self.assertRaisesRegex(ValueError,'explicit boolean'):
                f.freeze('confirmation_1','unused','unused',ready_soft_grace=value)


if __name__=='__main__':unittest.main()
