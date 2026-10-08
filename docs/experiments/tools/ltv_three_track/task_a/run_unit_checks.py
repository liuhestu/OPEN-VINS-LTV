#!/usr/bin/env python3
"""Small native checks relevant to frozen G/V; invoked inside shared launcher."""
import argparse
import json
import os
import time
from pathlib import Path
import subprocess

def registered_run(command, stream, label):
    registry = Path(os.environ['LTV_COORD_ROOT']) / 'registry/task_a.jsonl'
    def register_before_exec():
        # Single-threaded batch parent. Log in the forked child before exec:
        # even an immediately exiting executable cannot lose PID/start identity.
        pid = os.getpid()
        text = Path('/proc/self/stat').read_text()
        fields = text[text.rfind(')') + 2:].split()
        record = {'event': 'PROBE_PRE_EXEC_REGISTERED', 'task': 'a', 'owner': str(os.getuid()),
                  'run_id': os.environ['LTV_RUN_ID'], 'label': label, 'pid': pid,
                  'pgid': os.getpgrp(), 'start_ticks': fields[19], 'command': command,
                  'actual_executable_pre_exec': os.readlink('/proc/self/exe'),
                  'expected_executable_after_exec': command[0], 'registered_at': time.time(),
                  'identity_contract': 'PID/PGID/start_ticks persist across exec; executable changes; no claim monitor saw post-exec'}
        with registry.open('a') as f:
            f.write(json.dumps(record) + '\n')
    child = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT, preexec_fn=register_before_exec)
    # Popen succeeds only after its exec error pipe closes; launch failures raise.
    code = child.wait()
    with registry.open('a') as f:
        f.write(json.dumps({'event': 'PROBE_PROCESS_FINISH', 'task': 'a', 'run_id': os.environ['LTV_RUN_ID'],
                            'label': label, 'pid': child.pid, 'command': command, 'exit_code': code,
                            'finished_at': time.time()}) + '\n')
    return code


p = argparse.ArgumentParser()
p.add_argument('install', type=Path)
p.add_argument('source', type=Path)
p.add_argument('output', type=Path)
a = p.parse_args()
names = ('test_ltv_alias_fixed', 'test_ltv_core_lifecycle', 'test_ltv_options', 'test_ltv_timing', 'test_ltv_pose_convention',
         'test_ltv_readiness', 'test_ltv_fej', 'test_ltv_gv_jacobian', 'test_ltv_gv_covariance',
         'test_ltv_joint_layout', 'test_ltv_joint_covariance', 'test_ltv_huber', 'test_ltv_innovation')
results = []
for name in names:
    command = [str(a.install / 'ov_msckf/lib/ov_msckf' / name)]
    with (a.output / (name + '.log')).open('x') as stream:
        code = registered_run(command, stream, name)
    results.append({'name': name, 'command': command, 'exit_code': code})
    (a.output / 'checks.json').write_text(json.dumps(results, indent=2) + '\n')
    print(name, code, flush=True)
    if code:
        raise SystemExit(code)
fixture = a.source / 'docs/ltv_handover/parity_fixture'
command = [str(a.install / 'ov_msckf/lib/ov_msckf/test_ltv_core_parity'), str(fixture / 'inputs.jsonl'), str(a.output / 'core_matrices.jsonl')]
with (a.output / 'core.log').open('x') as stream:
    code = registered_run(command, stream, 'test_ltv_core_parity')
results.append({'name': 'test_ltv_core_parity', 'command': command, 'exit_code': code})
if code:
    raise SystemExit(code)
command = ['python3', str(fixture / 'compare.py'), str(a.output / 'core_matrices.jsonl'), '--expected',
           str(fixture / 'expected_matrices.jsonl'), '--atol', '0', '--rtol', '0']
with (a.output / 'exact_core_matrices.log').open('x') as stream:
    code = registered_run(command, stream, 'exact_core_matrices')
results.append({'name': 'exact_core_matrices', 'command': command, 'exit_code': code})
(a.output / 'checks.json').write_text(json.dumps(results, indent=2) + '\n')
raise SystemExit(code)
