#!/usr/bin/env python3
"""Small native checks relevant to frozen G/V; invoked inside shared launcher."""
import argparse
import json
import os
import time
from pathlib import Path
import subprocess

def registered_run(command, stream, label):
    child = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT)
    text = Path(f'/proc/{child.pid}/stat').read_text()
    fields = text[text.rfind(')') + 2:].split()
    record = {'event': 'PROBE_PROCESS_REGISTERED', 'task': 'a', 'owner': str(os.getuid()),
              'run_id': os.environ['LTV_RUN_ID'], 'label': label, 'pid': child.pid,
              'pgid': os.getpgid(child.pid), 'start_ticks': fields[19], 'command': command,
              'expected_executable': command[0], 'registered_at': time.time()}
    registry = Path(os.environ['LTV_COORD_ROOT']) / 'registry/task_a.jsonl'
    with registry.open('a') as f:
        f.write(json.dumps(record) + '\n')
    code = child.wait()
    with registry.open('a') as f:
        f.write(json.dumps({**record, 'event': 'PROBE_PROCESS_FINISH', 'exit_code': code, 'finished_at': time.time()}) + '\n')
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
