"""Independent scheduler contract tests; use only with main-agent compute slot.

All ledgers live in TemporaryDirectory; never touches the real experiment budget.
The orphan test launches one sleeping child and terminates it during cleanup.
"""
import fcntl
import importlib.util
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import unittest

MODULE = Path(__file__).resolve().parents[1] / 'budget.py'
spec = importlib.util.spec_from_file_location('budget_under_test', MODULE)
budget = importlib.util.module_from_spec(spec)
spec.loader.exec_module(budget)


class BudgetContract(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.out = Path(self.directory.name)

    def test_attempt_limit_and_no_refund(self):
        budget.reserve('real', ['not-started'], 'reserved-is-counted', units=36, out=self.out)
        self.assertEqual(budget.usage(self.out)['real'], 36)
        with self.assertRaises(RuntimeError):
            budget.reserve('real', ['retry'], 'over-budget', out=self.out)
        self.assertEqual(len(budget.events(self.out)), 1)

    def test_short_duration_contract(self):
        for seconds in (None, -1, 0, 10.00001, float('nan'), float('inf')):
            with self.assertRaises(ValueError):
                budget.reserve('short_real', ['unused'], 'short', input_seconds=seconds, out=self.out)
        budget.reserve('short_real', ['unused'], 'short', input_seconds=10, out=self.out)
        self.assertEqual(budget.usage(self.out)['short_real'], 1)

    def test_launch_failure_counts_and_records_terminal(self):
        with self.assertRaises(FileNotFoundError):
            budget.run('real', ['/definitely/missing/binary'], 'failure', out=self.out)
        self.assertEqual(budget.usage(self.out)['real'], 1)
        events = budget.events(self.out)
        self.assertEqual(events[0]['event'], 'reserved')
        self.assertEqual(events[-1]['event'], 'finished')
        self.assertIn('error', events[-1])

    def test_two_slots_block_third_launch_without_charging(self):
        held = []
        try:
            for i in range(2):
                f = (self.out / f'heavy_{i}.lock').open('a')
                fcntl.flock(f, fcntl.LOCK_EX | fcntl.LOCK_NB)
                held.append(f)
            with self.assertRaises(RuntimeError):
                budget.run('real', [sys.executable, '-c', 'pass'], 'third', out=self.out)
            self.assertEqual(budget.usage(self.out)['real'], 0)
        finally:
            for f in held:
                f.close()

    def test_running_child_retains_slot_when_parent_dies(self):
        # The durable PID record alone cannot prevent an already-running child
        # exceeding concurrency after its scheduling parent is killed.
        program = (
            'import sys; from pathlib import Path; '
            f'sys.path.insert(0, {str(MODULE.parent)!r}); import budget; '
            f'budget.run("test", [sys.executable,"-c","import time; time.sleep(30)"],'
            f'"orphan-slot",out=Path({str(self.out)!r}))'
        )
        parent = subprocess.Popen([sys.executable, '-c', program], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        child_pid = None
        try:
            until = time.monotonic() + 10
            while time.monotonic() < until:
                started = [e for e in budget.events(self.out) if e['event'] == 'started']
                if started:
                    child_pid = started[-1]['pid']
                    break
                if parent.poll() is not None:
                    self.fail('scheduler exited before child startup')
                time.sleep(.02)
            self.assertIsNotNone(child_pid, 'startup timeout')
            parent.kill()
            parent.wait(timeout=5)
            os.kill(child_pid, 0)
            available = 0
            for i in range(2):
                with (self.out / f'heavy_{i}.lock').open('a') as f:
                    try:
                        fcntl.flock(f, fcntl.LOCK_EX | fcntl.LOCK_NB)
                        available += 1
                    except BlockingIOError:
                        pass
            self.assertEqual(available, 1, 'orphaned child lost its occupied compute slot')
        finally:
            if parent.poll() is None:
                parent.kill()
                parent.wait(timeout=5)
            if child_pid is not None:
                try:
                    os.kill(child_pid, signal.SIGTERM)
                except ProcessLookupError:
                    pass


if __name__ == '__main__':
    unittest.main()
