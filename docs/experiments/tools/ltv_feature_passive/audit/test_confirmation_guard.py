"""Confirmation guards with isolated fake artifacts; never reads real datasets."""
import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import budget
import freeze
import real_run
import validate


class ConfirmationGuard(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.doc = self.root / 'docs'
        self.doc.mkdir()
        self.protocol = dict(synthetic_scenarios=['REGULAR', 'FAST'], confirmation_seeds=[101, 102],
                             synthetic_confirmation_lifetimes=[.5, 1.],
                             real_confirmation=['V1_03_difficult', 'V2_03_difficult', 'indoor_forward_6'],
                             real_development=['V1_01_easy', 'V2_02_medium', 'indoor_forward_3'],
                             repeat_P_NEW=['V2_03_difficult', 'indoor_forward_6'])
        (self.doc / 'protocol.json').write_text(json.dumps(self.protocol))
        self.frozen = dict(method='SEED_HYBRID', source='STEREO_THEN_TEMPORAL', risk=.05,
                           synthetic_library=str(self.root / 'library.so'), runtime=str(self.root / 'runtime'))
        self.freeze_sha = 'fake-verified-freeze'
        for patch in (mock.patch.object(budget, 'OUT', self.root), mock.patch.object(budget, 'DOC', self.doc),
                      mock.patch.object(real_run, 'prepare', return_value={'sensors': {}})):
            patch.start()
            self.addCleanup(patch.stop)

    def test_no_freeze_no_confirmation_launch(self):
        with mock.patch.object(validate, 'check', side_effect=FileNotFoundError('no freeze')), \
             mock.patch.object(budget, 'run') as launch:
            with self.assertRaises(FileNotFoundError):
                validate.synthetic('REGULAR', 101, 1.)
            with self.assertRaises(FileNotFoundError):
                validate.real('V2_03_difficult', 'P_NEW')
            launch.assert_not_called()

    def test_invalid_scene_seed_and_lifetime_rejected_before_input_generation(self):
        with mock.patch.object(validate, 'check', return_value=(self.frozen, self.freeze_sha)), \
             mock.patch.object(budget, 'run') as launch:
            for scene, seed, life in [('PAPER', 101, 1.), ('REGULAR', 42, 1.), ('FAST', 102, 5.),
                                      ('FAST', 103, .5), ('REGULAR', 101, float('nan'))]:
                with self.assertRaises(ValueError):
                    validate.synthetic(scene, seed, life)
            launch.assert_not_called()

    def test_real_source_mode_short_and_repeat_scope(self):
        with mock.patch.object(freeze, 'check', return_value=(self.frozen, self.freeze_sha)), \
             mock.patch.object(budget, 'run') as launch:
            cases = [dict(sequence='V1_01_easy'), dict(source='STEREO'), dict(mode='GV'), dict(short=1),
                     dict(repeat=2), dict(repeat=-1), dict(sequence='V1_03_difficult', repeat=1),
                     dict(mode='P_OLD', repeat=1), dict(mode='B', repeat=1)]
            for update in cases:
                args = dict(sequence='V2_03_difficult', mode='P_NEW', source=self.frozen['source'], confirmation=True)
                args.update(update)
                with self.assertRaises(ValueError):
                    real_run.run(**args)
            launch.assert_not_called()

    def test_development_dispatcher_cannot_enter_confirmation(self):
        with mock.patch.object(budget, 'run') as launch:
            with self.assertRaises(RuntimeError):
                real_run.run('V2_03_difficult', 'P_NEW', self.frozen['source'])
            launch.assert_not_called()

    def test_failed_synthetic_artifact_is_preserved_not_retried(self):
        library = Path(self.frozen['synthetic_library'])
        library.write_bytes(b'fake-library')
        scripts = self.root / 'docs/experiments/tools'
        (scripts / 'synthetic').mkdir(parents=True)
        (scripts / 'synthetic/run.py').write_text('# fake numerical wrapper\n')
        inputs = self.root / 'synthetic_inputs/REGULAR_101_1s'
        inputs.mkdir(parents=True)
        inp = dict(confirmation_freeze_sha=self.freeze_sha, input_sha='fake-input')
        (inputs / 'identity.json').write_text(json.dumps(inp))
        identity = dict(phase='confirmation', freeze_sha=self.freeze_sha, input_identity=inp, method='OLD',
                        library_sha=validate.artifacts.sha(library), runner_sha=validate.artifacts.sha(scripts / 'synthetic/run.py'))
        key = hashlib.sha256(json.dumps(identity, sort_keys=True).encode()).hexdigest()[:12]
        out = self.root / 'synthetic_runs' / ('REGULAR_101_OLD_' + key)
        out.mkdir(parents=True)
        (out / 'identity.json').write_text(json.dumps(identity))
        (out / 'run.json').write_text(json.dumps(dict(status='FAILED', full_input_consumed=False)))
        seal = {name: validate.artifacts.sha(out / name) for name in ('identity.json', 'run.json')}
        (out / 'artifacts_sha256.json').write_text(json.dumps(seal))
        before = (out / 'run.json').read_bytes()
        with mock.patch.object(validate, 'HERE', scripts), \
             mock.patch.object(validate, 'check', return_value=(self.frozen, self.freeze_sha)), \
             mock.patch.object(budget, 'run') as launch:
            with self.assertRaisesRegex(RuntimeError, 'Interrupted'):
                validate.synthetic('REGULAR', 101, 1.)
            launch.assert_not_called()
        self.assertEqual((out / 'run.json').read_bytes(), before)


if __name__ == '__main__':
    unittest.main()
