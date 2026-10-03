"""Small counterexamples: labels/events identity and bounded physical support."""
import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

path = Path(__file__).resolve().parents[1] / 'synthetic/evaluate.py'
spec = importlib.util.spec_from_file_location('synthetic_evaluate', path)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class EvaluatorContract(unittest.TestCase):
    def test_event_count_order_and_timestamp(self):
        times = module.np.array([0., .05, .1])
        module.verify_camera_events([{'t': t} for t in times], times)
        for events in ([{'t': 0}], [{'t': 0}, {'t': .1}, {'t': .05}],
                       [{'t': 0}, {'t': .05}, {'t': float('nan')}],
                       [{'t': 0}, {'t': .05}, {'t': .100001}]):
            with self.assertRaises(RuntimeError):
                module.verify_camera_events(events, times)

    def test_labels_mutation_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            p = Path(directory)
            (p / 'inputs.npz').write_bytes(b'sensor-only')
            (p / 'labels.npz').write_bytes(b'evaluator-only')
            identity = {key: hashlib.sha256((p / name).read_bytes()).hexdigest()
                        for key, name in [('input_sha', 'inputs.npz'), ('labels_sha', 'labels.npz')]}
            (p / 'identity.json').write_text(json.dumps(identity))
            module.verify_input_identity(p, identity)
            (p / 'labels.npz').write_bytes(b'changed-truth')
            with self.assertRaisesRegex(RuntimeError, 'labels.npz'):
                module.verify_input_identity(p, identity)

    def test_endpoint_has_no_extra_time_support(self):
        t = module.np.array([0., .05, .1])
        self.assertAlmostEqual(module.physical_ready_seconds(t, [True, True, True]), .1)
        self.assertEqual(module.physical_ready_seconds(t, [False, False, True]), 0)
        self.assertAlmostEqual(module.longest_false(t, [False, False, False]), .1)
        self.assertAlmostEqual(module.longest_false(t, [True, False, False]), .05)

    def test_hold_uses_both_camera_sides(self):
        self.assertEqual(module.first_hold([(0, .01), (.05, .01), (.1, .01)], .05), (0., .1))
        self.assertEqual(module.first_hold([(0, .01), (.05, .01), (.05, .1), (.1, .01)], .05), (None, None))
        self.assertEqual(module.first_hold([(0, .01), (.05, .01)], .05), (None, None))


if __name__ == '__main__':
    unittest.main()
