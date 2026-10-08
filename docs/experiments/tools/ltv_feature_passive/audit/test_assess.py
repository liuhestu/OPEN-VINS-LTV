"""Final scientific/completeness boundaries, isolated fabricated evidence only."""
import itertools
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import assess


class FinalAssessment(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.protocol = dict(real_development=['dev1', 'dev2', 'dev3'], real_confirmation=['confirm1', 'confirm2', 'confirm3'],
                             repeat_P_NEW=['confirm2', 'confirm3'], synthetic_scenarios=['REGULAR', 'FAST'],
                             confirmation_seeds=[101, 102], synthetic_confirmation_lifetimes=[.5, 1.])
        self.write(self.root / 'protocol.json', self.protocol)
        self.digest = 'frozen-example'
        self.frozen = dict(method='SEED_HYBRID', source='STEREO_THEN_TEMPORAL', runtime='fake-runtime', synthetic_library=str(self.root/'library.so'))
        (self.root/'library.so').write_bytes(b'fake-library')
        self.library_sha=assess.sha(self.root/'library.so')
        self.runner_sha=assess.sha(Path(assess.__file__).parent/'synthetic/run.py')
        self.manifest = dict(freeze_sha=self.digest, real_metrics={}, synthetic_metrics=[], synthetic_pairs={}, repeat_checks={})
        for sequence in self.protocol['real_development'] + self.protocol['real_confirmation']:
            path = self.root / (sequence + '.json')
            self.write(path, dict(sequence=sequence, engineering={'pass': True},
                                 seed_source_identity={'sequence': sequence, 'freeze_sha': self.digest, 'source': self.frozen['source'], 'runtime': 'fake-runtime'},
                                 coverage_status='SUFFICIENT_COVERAGE', qualifying_improvement=sequence != 'confirm3',
                                 raw_non_degradation=True))
            self.manifest['real_metrics'][sequence] = str(path)
        for index, (scene, seed, life) in enumerate(itertools.product(['REGULAR', 'FAST'], [101, 102], [.5, 1.])):
            folder = self.root / ('synthetic' + str(index))
            folder.mkdir()
            inp=dict(scene=scene, seed=seed, lifetime=life, confirmation_freeze_sha=self.digest)
            old=folder/'old'
            old.mkdir()
            for directory,method in ((folder,'SEED_HYBRID'),(old,'OLD')):
                (directory/'states.npz').write_bytes(b'fake-states')
                self.write(directory/'run.json',dict(method=method,status='COMPLETED_ESTIMATION',full_input_consumed=True,
                    input_identity=inp,library_sha=self.library_sha,runner_sha=self.runner_sha,
                    final_state_sha=assess.sha(directory/'states.npz')))
            self.manifest['synthetic_pairs'][str(folder/'metrics.json')]=str(old)
            self.write(folder / 'metrics.json', dict(method='SEED_HYBRID', status='EVALUATED',
                       diagnostics={'full_input_consumed': True}, baseline={'raw':{'v_RMSE':1.}},
                       seed=dict(accepted_seeds=100, accepted_fraction_opportunity=.2,
                                 reliable_10pct_fraction=.9, reliable_5pct_fraction=.7)))
            self.manifest['synthetic_metrics'].append(str(folder / 'metrics.json'))
        for sequence in self.protocol['repeat_P_NEW']:
            path = self.root / (sequence + '_repeat.json')
            self.write(path, dict(status='PASS', repeated_auxiliary_exact=True, sequence=sequence,
                                 identity={'sequence':sequence,'freeze_sha':self.digest}, repeat_indices=[0,1]))
            self.manifest['repeat_checks'][sequence] = str(path)
        for kind in ('cache_check', 'readonly_check'):
            path = self.root / (kind + '.json')
            self.write(path, {'status': 'PASS'})
            self.manifest[kind] = str(path)
        self.manifest_path = self.root / 'manifest.json'
        for patch in (mock.patch.object(assess, 'check', return_value=(self.frozen, self.digest)),
                      mock.patch.object(assess.budget, 'DOC', self.root),
                      mock.patch.object(assess.budget, 'usage', return_value={'synthetic': 16, 'real': 20})):
            patch.start()
            self.addCleanup(patch.stop)

    @staticmethod
    def write(path, value):
        Path(path).write_text(json.dumps(value))

    def change(self, path, key, value):
        data = json.loads(Path(path).read_text())
        data[key] = value
        self.write(path, data)

    def result(self):
        self.write(self.manifest_path, self.manifest)
        return assess.assess(self.manifest_path)

    def test_exact_scientific_boundaries_pass(self):
        self.assertEqual(self.result()['status'], 'GOAL_ACHIEVED')

    def test_high_accuracy_with_tiny_acceptance_is_not_success(self):
        path = self.manifest['synthetic_metrics'][0]
        data = json.loads(Path(path).read_text())
        data['seed'].update(reliable_10pct_fraction=1., accepted_fraction_opportunity=.01)
        self.write(path, data)
        self.assertEqual(self.result()['status'], 'COMPLETED_WITH_LIMITATIONS')

    def test_low_coverage_or_raw_harm_blocks_scientific_success(self):
        path = self.manifest['real_metrics']['confirm3']
        self.change(path, 'coverage_status', 'INSUFFICIENT_COVERAGE')
        self.assertEqual(self.result()['output_quality'], 'NOT_MET')
        self.change(path, 'coverage_status', 'SUFFICIENT_COVERAGE')
        self.change(path, 'raw_non_degradation', False)
        self.assertEqual(self.result()['status'], 'COMPLETED_WITH_LIMITATIONS')

    def test_failed_bypass_is_blocked_not_negative_algorithm_result(self):
        self.change(self.manifest['real_metrics']['dev1'], 'engineering', {'pass': False})
        self.assertEqual(self.result()['status'], 'BLOCKED_CORRECTNESS')

    def test_missing_or_duplicate_confirmation_scenario_is_rejected(self):
        removed = self.manifest['synthetic_metrics'].pop()
        with self.assertRaises(ValueError):
            self.result()
        self.manifest['synthetic_metrics'].append(self.manifest['synthetic_metrics'][0])
        with self.assertRaises(ValueError):
            self.result()

    def test_failed_old_pair_is_not_complete_evidence(self):
        metric=self.manifest['synthetic_metrics'][0]
        old=Path(self.manifest['synthetic_pairs'][metric])/'run.json'
        self.change(old,'status','FAILED')
        with self.assertRaises(ValueError):self.result()

    def test_repeat_must_belong_to_named_sequence(self):
        path=self.manifest['repeat_checks']['confirm2']
        self.change(path,'sequence','confirm1')
        with self.assertRaises(ValueError):self.result()

    def test_wrong_sequence_file_must_not_count_as_another_confirmation(self):
        self.manifest['real_metrics']['confirm2'] = self.manifest['real_metrics']['confirm1']
        with self.assertRaises(ValueError):
            self.result()


if __name__ == '__main__':
    unittest.main()
