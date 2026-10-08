#!/usr/bin/env python3
"""Fresh scalar runner object over explicitly verified, copied same-task dependencies."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shlex
import shutil
import subprocess


def sha(path):
    h = hashlib.sha256()
    with path.open('rb') as f:
        for b in iter(lambda: f.read(4 * 1024 * 1024), b''):
            h.update(b)
    return h.hexdigest()

p = argparse.ArgumentParser()
p.add_argument('parent_artifact', type=Path)
p.add_argument('parent_source', type=Path)
p.add_argument('source', type=Path)
p.add_argument('artifact', type=Path)
p.add_argument('output', type=Path)
a = p.parse_args()
assert not a.artifact.exists(), 'fresh artifact only'
closure = []
for package in ('ov_core', 'ov_init', 'ov_msckf'):
    paths = [a.parent_source / package / 'src', a.parent_source / package / 'cmake']
    files = [p for folder in paths for p in folder.rglob('*') if p.is_file()]
    files += [a.parent_source / package / 'CMakeLists.txt', a.parent_source / package / 'package.xml']
    for parent in files:
        relative = parent.relative_to(a.parent_source)
        current = a.source / relative
        assert current.is_file() and sha(parent) == sha(current), str(relative)
        closure.append({'path': str(relative), 'sha256': sha(parent)})
a.artifact.mkdir(parents=True)
install = a.artifact / 'install'
shutil.copytree(a.parent_artifact / 'install', install, symlinks=False)
replacements = [(str(a.parent_artifact / 'install'), str(install)),
                (str(a.parent_source), str(a.source))]
# Repair/export copied prefix metadata, preserving all binary/library/header bytes.
patched = []
text_suffixes = {'.bash', '.sh', '.zsh', '.dsv', '.ps1', '.py', '.cmake', '.pc', '.xml', '.txt'}
for path in install.rglob('*'):
    if not path.is_file() or path.suffix not in text_suffixes:
        continue
    try:
        original = path.read_text()
    except UnicodeDecodeError:
        continue
    text = original
    for old, new in replacements:
        text = text.replace(old, new)
    if text != original:
        path.write_text(text)
        patched.append(str(path.relative_to(install)))
libs = {}
for package in ('ov_core', 'ov_init', 'ov_msckf'):
    parent = a.parent_artifact / 'install' / package / 'lib' / ('lib' + package + '_lib.so')
    copied = install / package / 'lib' / parent.name
    assert sha(parent) == sha(copied)
    libs[package] = {'parent_path': str(parent), 'new_path': str(copied), 'sha256': sha(copied)}
runners = {}
for target in ('run_ltv_gv_evaluation', 'run_ltv_gv_production'):
    cmake_target = a.parent_artifact / 'build/ov_msckf/tests/ltv/CMakeFiles' / (target + '.dir')
    flags = {}
    for line in (cmake_target / 'flags.make').read_text().splitlines():
        if ' = ' in line:
            k, value = line.split(' = ', 1)
            flags[k] = value
    def rewrite(token):
        for old, new in replacements:
            token = token.replace(old, new)
        return token
    obj = a.artifact / 'build' / (target + '.o')
    obj.parent.mkdir(exist_ok=True)
    cpp = a.source / 'docs/experiments/tools/native/run_ltv_feature_passive.cpp'
    compile_command = ['/usr/bin/c++']
    for key in ('CXX_DEFINES', 'CXX_INCLUDES', 'CXX_FLAGS'):
        compile_command += [rewrite(t) for t in shlex.split(flags[key])]
    compile_command += ['-c', str(cpp), '-o', str(obj)]
    print('compile fresh runner object', target, flush=True)
    subprocess.run(compile_command, check=True)
    old_link = shlex.split((cmake_target / 'link.txt').read_text())
    link, skip = [], False
    binary = install / 'ov_msckf/lib/ov_msckf' / target
    for token in old_link:
        if skip:
            skip = False
            continue
        if token == '-o':
            link += ['-o', str(binary)]
            skip = True
        elif token.endswith('run_ltv_feature_passive.cpp.o'):
            link.append(str(obj))
        elif token.startswith('-Wl,-rpath,'):
            link.append('-Wl,-rpath,' + ':'.join(str(install / pkg / 'lib') for pkg in ('ov_msckf', 'ov_init', 'ov_core')) + ':/opt/ros/humble/lib')
        elif Path(token).name in [Path(v['new_path']).name for v in libs.values()]:
            link.append(next(v['new_path'] for v in libs.values() if Path(v['new_path']).name == Path(token).name))
        else:
            link.append(rewrite(token))
    print('link only copied verified dependencies', target, flush=True)
    subprocess.run(link, check=True)
    ldenv = dict(os.environ)
    ldenv['LD_LIBRARY_PATH'] = ':'.join(str(install / p / 'lib') for p in ('ov_msckf', 'ov_init', 'ov_core')) + ':' + ldenv.get('LD_LIBRARY_PATH', '')
    resolved = subprocess.check_output(['ldd', str(binary)], env=ldenv, text=True)
    assert 'not found' not in resolved
    for package in libs:
        matches = [line for line in resolved.splitlines() if ('lib' + package + '_lib.so =>') in line]
        assert len(matches) == 1 and libs[package]['new_path'] in matches[0], (package, matches)
    runners[target] = {'compile_command': compile_command, 'link_command': link, 'ldd': resolved,
                       'binary_sha256': sha(binary), 'fresh_object_sha256': sha(obj)}
for package, entry in libs.items():
    assert sha(Path(entry['parent_path'])) == sha(Path(entry['new_path'])) == entry['sha256']
identity = {'status': 'PASS_BUILD_DEPENDENCY_CLOSURE', 'parent_artifact': str(a.parent_artifact),
            'new_artifact': str(a.artifact), 'source': str(a.source), 'parent_source': str(a.parent_source),
            'source_closure': closure, 'libraries': libs, 'patched_prefix_metadata': patched,
            'runners': runners, 'qualification': 'NONE; requires new short probe and full exact parity'}
(a.output / 'runner_build_identity.json').write_text(json.dumps(identity, indent=2) + '\n')
(a.artifact / 'runner_build_identity.json').write_text(json.dumps(identity, indent=2) + '\n')
for file in install.rglob('*'):
    if file.is_file():file.chmod(file.stat().st_mode & ~0o222)
print(str(install), flush=True)
