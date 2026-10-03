#!/usr/bin/env python3
"""Run installed Phase 0 tests in sequence, preserving logs and stopping on failure."""
import argparse
import json
from pathlib import Path
import subprocess


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('install', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    for name in ['test_ltv_alias_fixed', 'test_ltv_core_lifecycle', 'test_ltv_gv_jacobian',
                 'test_ltv_gv_covariance', 'test_ltv_fej']:
        # Positional arguments keep filesystem paths out of shell program text.
        command = ['bash', '-c', 'source /opt/ros/humble/setup.bash && source "$1/setup.bash" && ros2 run ov_msckf "$2"',
                   'phase0', str(args.install.resolve()), name]
        with (args.output / (name + '.stdout.log')).open('w') as stdout, (args.output / (name + '.stderr.log')).open('w') as stderr:
            result = subprocess.run(command, stdout=stdout, stderr=stderr)
        (args.output / (name + '.command.json')).write_text(json.dumps(
            dict(command=command, exit_code=result.returncode), indent=2))
        print(name, result.returncode, flush=True)
        if result.returncode:
            raise SystemExit(result.returncode)


if __name__ == '__main__':
    main()
