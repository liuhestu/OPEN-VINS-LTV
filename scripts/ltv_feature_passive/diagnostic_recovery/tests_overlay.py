"""Fabricated diagnostic recovery checks; no dataset or ground truth reads."""
import copy
import unittest
import json
from pathlib import Path
import tempfile
from unittest.mock import patch
import merge_overlay
from merge_overlay import merge_rows, lifecycle_counts, verify_binding, sha, PRESERVED, MANAGEMENT


class OverlayTest(unittest.TestCase):
    def fixture(self):
        ns = 1450000000000000123
        original = dict(camera_ns=ns, initialized=True, epoch=1, sequence=1,
                        imu_time=ns*1e-9, cursor=1, available=True, ready_G=False,
                        ready_V=False, mature_features=0, reason='ok', state_features=1,
                        observed_features=1, healthy_updates=1, camera_substeps=1,
                        v_body=[1.,2.,3.], eta_body=[0.,0.,-9.81],
                        current_cam0_ids=[7], current_stereo_ids=[7], retained_ids=[])
        recovered = {k: copy.deepcopy(original[k]) for k in PRESERVED}
        recovered.update(camera_time=ns*1e-9, camera_ns_roundtrip=round(ns*1e-9*1e9),
                         current_cam0_ids=[7], current_stereo_ids=[7],
                         event_type='process', management_accepted_input=True,
                         retained_ids=[7], opportunity_tracks=1, admitted_tracks=1,
                         retired_ids=[], tracks=[], seeds=[{'id':7,'admitted':True}])
        return original, recovered

    def test_roundtrip_recovers_without_changing_outputs(self):
        old, recovered = self.fixture()
        self.assertNotEqual(old['camera_ns'], recovered['camera_ns_roundtrip'])
        result, evidence = merge_rows([old], [recovered])
        self.assertEqual(result[0]['retained_ids'], [7])
        for key in PRESERVED:
            self.assertEqual(result[0][key], old[key])
        self.assertEqual(old['retained_ids'], [])
        self.assertEqual(evidence[0]['cumulative_admissions'], 1)

    def test_raw_output_mismatch_rejected(self):
        old, recovered = self.fixture()
        recovered['v_body'][0] += .001
        with self.assertRaisesRegex(ValueError, 'Raw output mismatch'):
            merge_rows([old], [recovered])

    def test_previously_logged_management_exact(self):
        old, recovered = self.fixture()
        old.update({k:copy.deepcopy(recovered[k]) for k in MANAGEMENT})
        _, evidence = merge_rows([old], [recovered])
        self.assertTrue(evidence[0]['original_guard_success_exact_management'])
        recovered['seeds'][0]['admitted'] = False
        with self.assertRaisesRegex(ValueError, 'Previously logged management differs'):
            merge_rows([old], [recovered])

    def test_missing_event_rejected(self):
        old, _ = self.fixture()
        with self.assertRaisesRegex(ValueError, 'absent from recovery'):
            merge_rows([old], [])

    def test_cumulative_count_rejected(self):
        old, recovered = self.fixture()
        recovered['admitted_tracks'] = 2
        with self.assertRaisesRegex(ValueError, 'cumulative'):
            merge_rows([old], [recovered])

    def test_duplicate_event_rejected(self):
        old, recovered = self.fixture()
        with self.assertRaisesRegex(ValueError, 'duplicate or out of order'):
            merge_rows([old], [recovered, recovered])

    def test_unavailable_internal_pause_keeps_old_core_count(self):
        old, recovered = self.fixture()
        old['available'] = recovered['available'] = False
        recovered.update(management_accepted_input=False, retained_ids=[], seeds=[], admitted_tracks=0)
        result, _ = merge_rows([old], [recovered])
        self.assertEqual(result[0]['state_features'], 1)
        self.assertEqual(result[0]['seeds'], [])

    def test_ttl_and_active_removals_are_distinct(self):
        row = dict(epoch=1, seeds=[{'id':7,'admitted':True}],
                   never_admitted_candidate_ttl_ids=[8,9], admitted_retired_ids=[7])
        counts = lifecycle_counts([row, {**row, 'seeds':[]}])
        self.assertEqual(counts['never_admitted_candidate_ttl_tracks'], 2)
        self.assertEqual(counts['admitted_active_removal_tracks'], 1)

    def test_ttl_cannot_be_admitted(self):
        row = dict(epoch=1, seeds=[{'id':7,'admitted':True}],
                   never_admitted_candidate_ttl_ids=[7], admitted_retired_ids=[])
        with self.assertRaisesRegex(ValueError, 'populations inconsistent'):
            lifecycle_counts([row])

    def test_binding_rejects_changed_recovery_bytes(self):
        with tempfile.TemporaryDirectory() as folder:
            p = Path(folder)
            for name in ('cache.bin', 'recovered.jsonl', 'report.json'):
                (p/name).write_text(name)
            (p/'frozen.json').write_text(json.dumps({'runtime':'frozen'}))
            (p/'binary').write_text('test binary')
            build = {'source_sha256':sha(Path(merge_overlay.__file__).with_name('recover.cpp')),
                     'freeze_sha256':sha(p/'frozen.json'),'runtime':'frozen',
                     'binary_sha256':sha(p/'binary'),'command':['g++','-o',str(p/'binary')],
                     'build':{'exit_code':0}}
            (p/'build.json').write_text(json.dumps(build))
            value = {'recovered_sha256': sha(p/'recovered.jsonl'), 'cache_sha256':sha(p/'cache.bin'),
                     'report_sha256':sha(p/'report.json'), 'buildproof_sha256':sha(p/'build.json'),
                     'buildproof':str(p/'build.json'),'original_run':str(p),'binary_sha':sha(p/'binary'),
                     'attempt':{'id':1,'exit_code':0},'freeze_sha':sha(p/'frozen.json'),'runtime':'frozen'}
            (p/'binding.json').write_text(json.dumps(value))
            with patch.object(merge_overlay,'FROZEN',p/'frozen.json'):
                verify_binding(p/'binding.json', p, p/'recovered.jsonl', p/'report.json')
            (p/'recovered.jsonl').write_text('changed')
            with self.assertRaisesRegex(ValueError, 'binding SHA mismatch'):
                verify_binding(p/'binding.json', p, p/'recovered.jsonl', p/'report.json')


if __name__ == '__main__':
    unittest.main()
