"""Isolated recovery/runtime identity tests; no replay, GT or observer runs."""
import hashlib
import json
import os
from pathlib import Path
import sys
import tempfile
import unittest
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import artifacts
import budget
import resume


class RecoveryIdentity(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.out = Path(self.temp.name)

    def start(self, kind='real'):
        run_id = budget.reserve(kind, ['not-executed'], 'isolated-test', out=self.out)
        process = {'pid': 4321, 'start_ticks': '100', 'boot_id': 'test-boot', 'state': 'S'}
        with budget.locked(self.out):
            budget.append({'event': 'started', 'id': run_id, 'pid': process['pid'],
                           'process': process, 'owner': None}, self.out)
        return run_id, process

    def test_pid_birth_prevents_recycled_pid_false_alive(self):
        original = budget.process_identity(os.getpid())
        self.assertTrue(budget.process_alive(original))
        recycled = dict(original, start_ticks=str(int(original['start_ticks']) + 1))
        self.assertFalse(budget.process_alive(recycled))
        rebooted = dict(original, boot_id='different-boot')
        self.assertFalse(budget.process_alive(rebooted))
        with mock.patch.object(budget, 'process_identity', return_value=dict(original, state='Z')):
            self.assertFalse(budget.process_alive(original))

    def test_interruption_is_terminal_without_refund_or_restart(self):
        self.start()
        with mock.patch.object(budget, 'process_alive', return_value=False), \
             mock.patch.object(budget.subprocess, 'Popen') as launch:
            observed = resume.resume(False, self.out)
            self.assertEqual(observed['pending'][0]['status'], 'INTERRUPTED')
            self.assertFalse(any(e['event'] == 'finished' for e in budget.events(self.out)))
            reconciled = resume.resume(True, self.out)
            self.assertEqual(reconciled['usage']['real'], 1)
            launch.assert_not_called()
        terminal = budget.events(self.out)[-1]
        self.assertEqual(terminal['event'], 'finished')
        self.assertIsNone(terminal['exit_code'])
        self.assertIn('INTERRUPTED', terminal['error'])
        self.assertEqual(resume.resume(True, self.out)['pending'], [])
        self.assertEqual(budget.usage(self.out)['real'], 1)

    def test_live_child_and_unstarted_reservation_not_reconciled(self):
        self.start()
        budget.reserve('real', ['not-launched'], 'must-review', out=self.out)
        with mock.patch.object(budget, 'process_alive', return_value=True):
            result = resume.resume(True, self.out)
        self.assertEqual([x['status'] for x in result['pending']],
                         ['LIVE_CHILD', 'RESERVED_WITHOUT_START_REQUIRES_REVIEW'])
        self.assertFalse(any(e['event'] == 'finished' for e in budget.events(self.out)))
        self.assertEqual(result['usage']['real'], 2)

    def test_candidate_reservation_is_not_orphan_process(self):
        budget.reserve('candidate', [], 'registered-method', out=self.out)
        self.assertEqual(resume.resume(True, self.out)['pending'], [])
        self.assertEqual(budget.usage(self.out)['candidate'], 1)

    def runtime_fixture(self):
        data = {'binaries': {'observer': hashlib.sha256(b'fake-executable').hexdigest()},
                'libraries': {'libobserver.so': hashlib.sha256(b'fake-library').hexdigest()}}
        key = hashlib.sha256(json.dumps(data,sort_keys=True).encode()).hexdigest()
        root = self.out / key
        (root / 'lib').mkdir(parents=True)
        (root / 'observer').write_bytes(b'fake-executable')
        (root / 'lib/libobserver.so').write_bytes(b'fake-library')
        (root / 'manifest.json').write_text(json.dumps(data))
        return root

    def test_mutated_manifest_cannot_replace_content_identity(self):
        root = self.runtime_fixture()
        data = json.loads((root/'manifest.json').read_text())
        data['libraries']['libobserver.so']='0'*64
        (root/'manifest.json').write_text(json.dumps(data))
        with self.assertRaisesRegex(RuntimeError, 'content address'):
            artifacts.verify(root)

    def test_external_mutable_path_rejected_even_if_bytes_match(self):
        root = self.runtime_fixture()
        outside = self.out/'matching.so'
        outside.write_bytes(b'fake-library')
        with mock.patch.object(artifacts,'dependencies',return_value={'libobserver.so':outside}):
            with self.assertRaisesRegex(RuntimeError,'external mutable'):
                artifacts.verify(root)

    def test_frozen_library_corruption_is_rejected(self):
        root = self.runtime_fixture()
        resolved = {'libobserver.so': root / 'lib/libobserver.so'}
        with mock.patch.object(artifacts, 'dependencies', return_value=resolved):
            artifacts.verify(root)
            (root / 'lib/libobserver.so').write_bytes(b'rebuilt-in-place')
            with self.assertRaisesRegex(RuntimeError, 'Frozen library changed'):
                artifacts.verify(root)

    def test_actual_loader_resolution_must_match_frozen_bytes(self):
        root = self.runtime_fixture()
        outside = self.out / 'different.so'
        outside.write_bytes(b'wrong-runtime-library')
        with mock.patch.object(artifacts, 'dependencies', return_value={'libobserver.so': outside}):
            with self.assertRaises(RuntimeError):
                artifacts.verify(root)


if __name__ == '__main__':
    unittest.main()
