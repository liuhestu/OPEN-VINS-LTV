#!/usr/bin/env python3
"""Relevant native contracts under an isolated install and launcher lifecycle."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time

p = argparse.ArgumentParser()
p.add_argument('install', type=Path)
p.add_argument('output', type=Path)
a = p.parse_args()
a.output.mkdir(exist_ok=False)
registry = Path(os.environ['LTV_COORD_ROOT']) / 'registry' / ('task_' + os.environ['LTV_TASK_NAME'] + '.jsonl')
names = ['test_ltv_landmark_approx', 'test_ltv_alias_fixed', 'test_ltv_core_lifecycle',
         'test_ltv_gv_jacobian', 'test_ltv_gv_covariance', 'test_ltv_fej', 'test_ltv_timing',
         'test_ltv_joint_layout', 'test_ltv_joint_covariance', 'test_ltv_innovation',
         'test_ltv_options', 'test_ltv_landmark_shadow', 'test_ltv_pose_convention',
         'test_ltv_controlled', 'test_ltv_seed_joint', 'test_ltv_full_sensitivity',
         'test_ltv_main_cross', 'test_ltv_native_full_sensitivity']
results = []
for name in names:
    binary = a.install / 'ov_msckf/lib/ov_msckf' / name
    command = [str(binary)]
    if name == 'test_ltv_main_cross':
        command.append(str(a.output / 'main_cross.csv'))
    if name == 'test_ltv_native_full_sensitivity':
        command += [str(a.output / 'native.csv'), 'production']
    def register():
        pid = os.getpid()
        f = Path('/proc/self/stat').read_text().rsplit(') ', 1)[1].split()
        e = dict(event='REGRESSION_EXEC_READY', task=os.environ['LTV_TASK_NAME'], run_id=os.environ['LTV_RUN_ID'],
                 pid=pid, pgid=os.getpgrp(), start_ticks=f[19], command=command, time=time.time())
        with registry.open('a') as s:
            s.write(json.dumps(e) + '\n')
    libraries = subprocess.run(['ldd', str(binary)], capture_output=True, text=True, check=True).stdout
    assert 'not found' not in libraries
    with (a.output / (name + '.log')).open('w') as s:
        result = subprocess.run(command, stdout=s, stderr=subprocess.STDOUT, preexec_fn=register)
    results.append(dict(name=name, exit_code=result.returncode, binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest()))
(a.output / 'checks.json').write_text(json.dumps(results, indent=2) + '\n')
print(json.dumps(results))
raise SystemExit(any(r['exit_code'] != 0 for r in results))
