"""Native B checks under caller's already loaded isolated install and locks."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def run_checks(install, destination):
    destination.mkdir(exist_ok=False)
    assert os.environ['LTV_TASK_NAME'] == 'b'
    registry = Path(os.environ['LTV_COORD_ROOT']) / 'registry/task_b.jsonl'
    names = ['test_ltv_landmark_approx', 'test_ltv_alias_fixed', 'test_ltv_core_lifecycle',
             'test_ltv_gv_jacobian', 'test_ltv_gv_covariance', 'test_ltv_fej',
             'test_ltv_timing', 'test_ltv_joint_layout', 'test_ltv_joint_covariance',
             'test_ltv_innovation', 'test_ltv_options', 'test_ltv_landmark_shadow']
    results = []
    for name in names:
        binary = (install / 'ov_msckf/lib/ov_msckf' / name).resolve()
        assert binary.is_file(), str(binary)
        libraries = subprocess.run(['ldd', str(binary)], capture_output=True, text=True, check=True).stdout
        assert 'not found' not in libraries
        def register():
            pid = os.getpid()
            stat = Path(f'/proc/{pid}/stat').read_text()
            fields = stat[stat.rfind(')') + 2:].split()
            entry = dict(event='NATIVE_TEST_CHILD_EXEC_READY', task='b', run_id=os.environ['LTV_RUN_ID'],
                         pid=pid, pgid=os.getpgid(pid), start_ticks=fields[19], time=time.time(),
                         executable=str(binary), command=[str(binary)])
            with registry.open('a') as stream:
                stream.write(json.dumps(entry) + '\n')
        with (destination / (name + '.log')).open('w') as stream:
            completed = subprocess.run([str(binary)], stdout=stream, stderr=subprocess.STDOUT, preexec_fn=register)
        record = dict(name=name, binary=str(binary), binary_sha256=sha(binary), libraries=libraries,
                      exit_code=completed.returncode)
        results.append(record)
        (destination / 'checks.json').write_text(json.dumps(results, indent=2) + '\n')
        print(name, completed.returncode, flush=True)
        if completed.returncode:
            raise SystemExit(completed.returncode)


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--install', type=Path, required=True)
    p.add_argument('--destination', type=Path, required=True)
    a = p.parse_args()
    run_checks(a.install, a.destination)
