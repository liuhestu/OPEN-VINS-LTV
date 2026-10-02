#!/usr/bin/env python3
"""Test the actual target alias fix, then replay immutable goldens.

Writes each command, exit code, stdout and stderr into a fresh output directory.
No native acceptance or ROS playback is allowed after a failed gate.
"""
import argparse
import json
from pathlib import Path
import subprocess
import tempfile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    repo = Path(__file__).resolve().parents[3]
    out = args.output or Path(tempfile.mkdtemp(prefix='openvins-ltv-core-gate.'))
    out.mkdir(parents=True, exist_ok=True)
    bundle = repo / 'docs/ltv_handover'
    fixture = bundle / 'parity_fixture'
    print(out, flush=True)

    def run(name, command, expected_exit=0):
        with (out / (name + '.stdout.log')).open('w') as stdout, (out / (name + '.stderr.log')).open('w') as stderr:
            result = subprocess.run([str(x) for x in command], stdout=stdout, stderr=stderr)
        (out / (name + '.command.json')).write_text(json.dumps({
            'command': [str(x) for x in command], 'exit_code': result.returncode,
            'expected_exit': expected_exit}, indent=2))
        print(name, result.returncode, flush=True)
        if result.returncode != expected_exit:
            raise SystemExit(result.returncode or 1)

    flags = ['g++', '-std=c++14', '-O1', '-DNDEBUG', '-Wextra', '-Wpedantic', '-I/usr/include/eigen3']
    run('compile_alias', flags + [Path(__file__).with_name('test_ltv_alias.cpp'), '-o', out / 'alias_probe'])
    run('alias_probe', [out / 'alias_probe'], expected_exit=1)
    core = repo / 'ov_msckf/src'
    run('compile_alias_fixed', flags + ['-I' + str(core), '-I' + str(repo / 'ov_core/src'),
        core / 'ltv/ltv_observer.cpp', Path(__file__).with_name('test_ltv_alias_fixed.cpp'), '-o', out / 'alias_fixed'])
    run('alias_fixed', [out / 'alias_fixed'])
    run('compile_alias_old_core', flags + ['-I' + str(core),
        bundle / 'source_core/vins/src/ltv/ltv_observer.cpp',
        Path(__file__).with_name('test_ltv_alias_fixed.cpp'), '-o', out / 'alias_old_core'])
    run('alias_old_core', [out / 'alias_old_core'], expected_exit=1)
    for name, core in [('source', bundle / 'source_core/vins/src'), ('target', repo / 'ov_msckf/src')]:
        run('compile_' + name, flags + ['-I' + str(core), '-I' + str(repo / 'ov_core/src'),
            core / 'ltv/ltv_observer.cpp', fixture / 'harness.cpp', '-o', out / (name + '_fixture')])
        run(name + '_outputs', [out / (name + '_fixture'), fixture / 'inputs.jsonl', out / (name + '_matrices.jsonl')])
        run('compare_' + name + '_outputs', ['python3', '-B', fixture / 'compare.py', out / (name + '_outputs.stdout.log')])
        run('compare_' + name + '_matrices', ['python3', '-B', fixture / 'compare.py',
            out / (name + '_matrices.jsonl'), '--expected', fixture / 'expected_matrices.jsonl'])


if __name__ == '__main__':
    main()
