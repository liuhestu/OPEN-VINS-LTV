"""Fabricated counterexamples for scope and rejection behavior of passive audit."""
import json
from pathlib import Path
import tempfile
import unittest
from passive_outputs import inspect, compare


class PassiveOutputs(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)

    def fixture(self, name, mode):
        p = self.root / name
        p.mkdir()
        (p / 'effective_options.json').write_text(json.dumps(dict(
            ltv_enable_gravity=False, ltv_enable_velocity=False, ltv_passive_audit_enabled=True,
            ltv_enabled=mode != 'B', ltv_feature_readiness_enabled=mode == 'P_NEW')))
        (p / 'replay.json').write_text(json.dumps(dict(
            mode=mode, camera_packets=1, output_rows=1, actual_G_submissions=0, actual_V_submissions=0,
            complete=True, input_camera_packets=1, imu_consumed=2, input_imu_samples=2)))
        (p / 'trajectory.csv').write_text('timestamp,qx\n1,0\n')
        (p / 'unmatched_camera.csv').write_text('camera_id,timestamp_ns,filename\n')
        (p / 'audit.csv').write_text(
            'camera_ns,initialized,state_time,clones,state_digest,tracker_digest,visual_digest,auxiliary_receipts,gravity_submissions,velocity_submissions\n'
            + '1000000000,1,1,5,' + ','.join(['a' * 40] * 3) + ',0,0,0\n')
        return p

    def test_exact_passive_and_changed_state_detected(self):
        b = self.fixture('B', 'B')
        new = self.fixture('NEW', 'P_NEW')
        self.assertEqual(compare([b, new])['status'], 'PASS')
        audit = new / 'audit.csv'
        audit.write_text(audit.read_text().replace('a' * 40, 'b' * 40, 1))
        with self.assertRaisesRegex(ValueError, 'passive mismatch'):
            compare([b, new])

    def test_actual_injection_rejected_even_config_off(self):
        p = self.fixture('bad', 'P_NEW')
        audit = p / 'audit.csv'
        audit.write_text(audit.read_text().replace(',0,0,0\n', ',0,1,0\n'))
        with self.assertRaisesRegex(ValueError, 'actual injection'):
            inspect(p)

    def test_full_incomplete_and_long_short_rejected(self):
        p = self.fixture('counts', 'B')
        meta = p / 'replay.json'
        data = json.loads(meta.read_text())
        data['imu_consumed'] = 1
        meta.write_text(json.dumps(data))
        with self.assertRaisesRegex(ValueError, 'incomplete'):
            inspect(p)
        data.update(complete=False, short_limit_seconds=10, input_span_seconds=10.01)
        meta.write_text(json.dumps(data))
        with self.assertRaisesRegex(ValueError, 'short input'):
            inspect(p)


if __name__ == '__main__':
    unittest.main()
