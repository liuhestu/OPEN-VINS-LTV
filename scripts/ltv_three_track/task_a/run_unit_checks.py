#!/usr/bin/env python3
"""Small native checks relevant to frozen G/V; invoked inside shared launcher."""
import argparse
import json
from pathlib import Path
import subprocess

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
        code = subprocess.run(command, stdout=stream, stderr=subprocess.STDOUT, check=False).returncode
    results.append({'name': name, 'command': command, 'exit_code': code})
    (a.output / 'checks.json').write_text(json.dumps(results, indent=2) + '\n')
    print(name, code, flush=True)
    if code:
        raise SystemExit(code)
fixture = a.source / 'docs/ltv_handover/parity_fixture'
command = [str(a.install / 'ov_msckf/lib/ov_msckf/test_ltv_core_parity'), str(fixture / 'inputs.jsonl'), str(a.output / 'core_matrices.jsonl')]
with (a.output / 'core.log').open('x') as stream:
    code = subprocess.run(command, stdout=stream, stderr=subprocess.STDOUT).returncode
results.append({'name': 'test_ltv_core_parity', 'command': command, 'exit_code': code})
if code:
    raise SystemExit(code)
command = ['python3', str(fixture / 'compare.py'), str(a.output / 'core_matrices.jsonl'), '--expected',
           str(fixture / 'expected_matrices.jsonl'), '--atol', '0', '--rtol', '0']
with (a.output / 'exact_core_matrices.log').open('x') as stream:
    code = subprocess.run(command, stdout=stream, stderr=subprocess.STDOUT).returncode
results.append({'name': 'exact_core_matrices', 'command': command, 'exit_code': code})
(a.output / 'checks.json').write_text(json.dumps(results, indent=2) + '\n')
raise SystemExit(code)
