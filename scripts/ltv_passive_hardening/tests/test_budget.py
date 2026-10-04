"""Independent temporary-ledger scheduler tests. Execute only with a granted slot."""
from concurrent.futures import ThreadPoolExecutor
import fcntl
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

MODULE = Path(__file__).resolve().parents[1] / 'budget.py'
spec = importlib.util.spec_from_file_location('hardening_budget_under_test', MODULE)
budget = importlib.util.module_from_spec(spec)
spec.loader.exec_module(budget)


class BudgetContract(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.out = Path(self.temp.name) / 'out'
        self.out.mkdir()
        self.doc = Path(self.temp.name) / 'doc'
        self.doc.mkdir()
        for p in (patch.object(budget, 'DOC', self.doc),
                  patch.object(budget.base, 'source_identity', return_value={'fixture':'isolated'})):
            p.start(); self.addCleanup(p.stop)

    def reserve(self, kind, units=1, phase='development'):
        return budget.reserve(kind, ['fabricated-reservation'], {'phase':phase}, units=units,
                              input_seconds=10 if kind=='short_real' else None, out=self.out)

    def test_all_hard_limits(self):
        for kind, maximum in budget.LIMITS.items():
            with self.subTest(kind=kind):
                self.reserve(kind, maximum, 'confirmation')
                self.assertEqual(budget.usage(self.out)[kind], maximum)
                with self.assertRaises(RuntimeError): self.reserve(kind, 1, 'confirmation')
        self.assertEqual(sum(e['event']=='reserved' for e in budget.events(self.out)),5)

    def test_final_capacity_preserved_real40_synthetic24(self):
        for kind, ceiling, reserve in [('real',32,40),('synthetic',56,24)]:
            self.reserve(kind,ceiling)
            for phase in ['development','engineering']:
                with self.assertRaises(RuntimeError):self.reserve(kind,phase=phase)
            self.assertEqual(budget.LIMITS[kind]-budget.usage(self.out)[kind],reserve)
            self.reserve(kind,reserve,'confirmation')
            with self.assertRaises(RuntimeError):self.reserve(kind,phase='confirmation')

    def test_reservation_race_cannot_spend_final_capacity(self):
        self.reserve('real',31)
        def attempt(_):
            try:return self.reserve('real')
            except RuntimeError:return None
        with ThreadPoolExecutor(max_workers=8) as pool: ids=list(pool.map(attempt,range(8)))
        self.assertEqual(sum(i is not None for i in ids),1)
        self.assertEqual(budget.usage(self.out)['real'],32)

    def test_short_duration_bounds(self):
        for seconds in (None,0,-1,10.00001,float('nan'),float('inf')):
            with self.subTest(seconds=seconds), self.assertRaises(ValueError):
                budget.reserve('short_real',['unused'],'bad',input_seconds=seconds,out=self.out)
        self.reserve('short_real')
        self.assertEqual(budget.usage(self.out)['short_real'],1)

    def test_failed_launch_and_nonzero_retry_both_count(self):
        with self.assertRaises(FileNotFoundError):
            budget.run('real',['/definitely/no/program'],'missing',out=self.out)
        result=budget.run('real',[sys.executable,'-c','raise SystemExit(7)'],'failed-retry',out=self.out)
        self.assertEqual(result['exit_code'],7)
        self.assertEqual(budget.usage(self.out)['real'],2)
        es=budget.events(self.out)
        self.assertEqual(len([e for e in es if e['event']=='reserved']),2)
        terminal=[e for e in es if e['event']=='finished']
        self.assertEqual(len(terminal),2)
        self.assertIn('error',terminal[0])
        self.assertEqual(terminal[1]['exit_code'],7)

    def test_two_slots_prevent_third_without_reservation(self):
        held=[]
        try:
            for i in range(2):
                f=(self.out/f'heavy_{i}.lock').open('a')
                fcntl.flock(f,fcntl.LOCK_EX|fcntl.LOCK_NB);held.append(f)
            with self.assertRaises(RuntimeError):
                budget.run('test',[sys.executable,'-c','pass'],'third',out=self.out)
            self.assertEqual(budget.events(self.out),[])
        finally:
            for f in held:f.close()

    def test_confirmation_requires_freeze_and_phase_is_recorded(self):
        with self.assertRaises(RuntimeError):
            budget.run('test',[sys.executable,'-c','pass'],'unfrozen',out=self.out,phase='confirmation')
        self.assertEqual(budget.events(self.out),[])
        # Scheduler presence gate only; scientific freeze validation belongs to dispatcher.
        (self.doc/'frozen_config.json').write_text('{}')
        result=budget.run('test',[sys.executable,'-c','pass'],'frozen-fixture',out=self.out,phase='confirmation')
        self.assertEqual(result['exit_code'],0)
        self.assertEqual(budget.events(self.out)[0]['identity']['phase'],'confirmation')
        with self.assertRaises(ValueError):
            budget.run('test',[sys.executable,'-c','pass'],'invalid',out=self.out,phase='unknown')

    def test_logging_analysis_counts_and_thread_environment(self):
        program='import os; print(os.environ["OPENBLAS_NUM_THREADS"]); print("stderr-marker",file=__import__("sys").stderr)'
        result=budget.run('analysis',[sys.executable,'-c',program],'diagnostic',out=self.out)
        directory=Path(result['directory'])
        self.assertEqual((directory/'stdout.log').read_text().strip(),'1')
        self.assertEqual((directory/'stderr.log').read_text().strip(),'stderr-marker')
        self.assertTrue((directory/'source_sha256.json').is_file())
        self.assertTrue((directory/'command_files_sha256.json').is_file())
        es=budget.events(self.out)
        self.assertEqual([e['event'] for e in es],['reserved','started','finished'])
        self.assertEqual({e['id'] for e in es},{result['id']})
        self.assertIn('start_ticks',es[1]['process'])
        self.assertEqual(sum(budget.usage(self.out).values()),0)


if __name__=='__main__':unittest.main()
