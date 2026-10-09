#!/usr/bin/env python3
"""Run immutable-source tasks under shared resource and per-task artifact locks.

One JSON specification per command. Process starts are sampled from /proc, never
identified solely by PID or executable name. No implicit process termination.
"""
import argparse
import datetime
import fcntl
import hashlib
import json
import os
from pathlib import Path
import subprocess
import threading
import time
import uuid


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def append(path, value):
    with path.open('a') as stream:
        stream.write(json.dumps(value, ensure_ascii=False) + '\n')
        stream.flush()


def process(pid):
    try:
        text = Path(f'/proc/{pid}/stat').read_text()
        fields = text[text.rfind(')') + 2:].split()
        executable = os.readlink(f'/proc/{pid}/exe')
        command = Path(f'/proc/{pid}/cmdline').read_bytes().replace(b'\0', b' ').decode(errors='replace')
        return {'pid': int(pid), 'ppid': int(fields[1]), 'pgid': int(fields[2]),
                'start_ticks': fields[19], 'state': fields[0], 'executable': executable, 'command': command}
    except (OSError, ValueError, IndexError):
        return None


def group(pgid):
    return [p for item in Path('/proc').iterdir() if item.name.isdigit()
            for p in [process(item.name)] if p and p['pgid'] == pgid and p['state'] != 'Z']


def clean_environment(coord, task, root, run_id):
    env = {k:v for k,v in os.environ.items() if not k.startswith('LTV_')}
    for name in ('AMENT_PREFIX_PATH', 'CMAKE_PREFIX_PATH', 'COLCON_PREFIX_PATH', 'LD_LIBRARY_PATH',
                 'PYTHONPATH', 'ROS_PACKAGE_PATH', 'ROS_LOG_DIR', 'ROS_DOMAIN_ID', 'RMW_IMPLEMENTATION',
                 'LTV_JOINT_SHADOW_PATH', 'CCACHE_DIR', 'CCACHE_BASEDIR', 'CMAKE_TOOLCHAIN_FILE'):
        env.pop(name, None)
    env.update(PATH='/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin',
               OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1', MKL_NUM_THREADS='1', NUMEXPR_NUM_THREADS='1',
               CMAKE_BUILD_PARALLEL_LEVEL='1', MAKEFLAGS='-j1', CCACHE_DISABLE='1',
               TMPDIR=str(root / 'tmp' / run_id), ROS_LOG_DIR=str(root / 'log' / run_id / 'ros'),
               LTV_COORD_ROOT=str(coord), LTV_TASK_ROOT=str(root), LTV_RUN_ID=run_id,
               ROS_DOMAIN_ID=str(task['ros_domain_id']), RMW_IMPLEMENTATION='rmw_fastrtps_cpp',
               ROS_LOCALHOST_ONLY='1', LTV_NAMESPACE=task['namespace'], LTV_TASK_NAME=task['namespace'].rsplit('_',1)[-1])
    defaults = coord / 'contracts/empty_colcon_defaults.yaml'
    env['COLCON_DEFAULTS_FILE'] = str(defaults)
    Path(env['TMPDIR']).mkdir(parents=True, exist_ok=False)
    Path(env['ROS_LOG_DIR']).mkdir(parents=True, exist_ok=False)
    return env


def run(spec_path):
    spec = json.loads(spec_path.read_text())
    coord = Path(spec['coord_root']).resolve()
    session = json.loads((coord / 'session.json').read_text())
    task_name = spec['task']
    task = session['tasks'][task_name]
    root, worktree = Path(task['task_root']), Path(task['worktree'])
    run_id = f"{task_name}_{spec['label']}_{datetime.datetime.now(datetime.timezone.utc):%Y%m%dT%H%M%S}_{uuid.uuid4().hex[:8]}"
    registry = coord / 'registry' / f'task_{task_name}.jsonl'
    base = {'owner': str(os.getuid()), 'task': task_name, 'run_id': run_id, 'launcher_pid': os.getpid(),
            'spec_path': str(spec_path.resolve()), 'spec_sha256': sha(spec_path), 'command': spec['command'],
            'launcher_path': str(Path(__file__).resolve()), 'launcher_sha256_at_start': sha(Path(__file__)),
            'label': spec['label'], 'kind': spec.get('kind', 'test'), 'heavy': spec.get('heavy', True),
            'performance': spec.get('performance', False), 'requested_at': time.time()}
    append(registry, {**base, 'event': 'WAITING_RESOURCE'})
    handles = []
    child = None
    pgid = None
    try:
        if spec.get('four_worker_pool'):
            build_gate = (coord / 'locks/build_replay.lock').open('a')
            fcntl.flock(build_gate, fcntl.LOCK_EX if base['kind'] == 'build' else fcntl.LOCK_SH)
            handles.append(build_gate)
            if base['kind'] != 'build':
                while True:
                    memory = Path('/proc/meminfo').read_text().splitlines()
                    available = next(int(line.split()[1]) for line in memory if line.startswith('MemAvailable:'))
                    if available >= 4 * 1024 * 1024:
                        slot = None
                        for number in range(4):
                            candidate = (coord / f'locks/slot_{number}.lock').open('a')
                            try:
                                fcntl.flock(candidate, fcntl.LOCK_EX | fcntl.LOCK_NB)
                                slot = candidate
                                break
                            except BlockingIOError:
                                candidate.close()
                        if slot is not None:
                            handles.append(slot)
                            break
                    time.sleep(1)
        elif base['heavy'] or base['performance']:
            host = (coord / 'locks/host_heavy.lock').open('a')
            fcntl.flock(host, fcntl.LOCK_EX)
            handles.append(host)
        gate = (coord / 'locks/performance_gate.lock').open('a')
        fcntl.flock(gate, fcntl.LOCK_EX if base['performance'] else fcntl.LOCK_SH)
        handles.append(gate)
        artifact = (root / 'task_artifact.lock').open('a')
        fcntl.flock(artifact, fcntl.LOCK_EX)
        handles.append(artifact)
        if base['performance']:
            (coord / 'performance_active.json').write_text(json.dumps(base) + '\n')
        destination = root / 'results' / run_id
        destination.mkdir(parents=True, exist_ok=False)
        env = clean_environment(coord, task, root, run_id)
        for key, value in spec.get('env', {}).items():
            if key in ('HOME', 'home', 'CODEX_HOME', 'AMENT_PREFIX_PATH', 'CMAKE_PREFIX_PATH', 'COLCON_PREFIX_PATH', 'LD_LIBRARY_PATH', 'PYTHONPATH'):
                raise ValueError(f'forbidden environment override {key}')
            env[key] = str(value)
        replacements = {'{RUN_DIR}': str(destination), '{TASK_ROOT}': str(root), '{WORKTREE}': str(worktree), '{COORD_ROOT}': str(coord)}
        def expand(value):
            for key, replacement in replacements.items():
                value = value.replace(key, replacement)
            return value
        command = [expand(arg) for arg in spec['command']]
        affinity = task.get('cpu_affinity')
        if affinity is not None:
            if len(affinity) != 4 or not set(affinity) <= os.sched_getaffinity(0):
                raise ValueError('worker requires four allowed logical CPUs')
            command = ['taskset', '-c', ','.join(map(str, affinity)), *command]
        cwd = Path(expand(spec.get('cwd', '{RUN_DIR}'))).resolve()
        cwd.mkdir(parents=True, exist_ok=True)
        git_oid = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=worktree, text=True).strip()
        diff = subprocess.check_output(['git', 'diff', '--binary', 'HEAD'], cwd=worktree)
        status = subprocess.check_output(['git', 'status', '--short'], cwd=worktree, text=True)
        identities = {'source_oid': spec.get('source_oid', git_oid), 'worktree_head': git_oid,
                      'dirty_state': status, 'dirty_patch_sha256': hashlib.sha256(diff).hexdigest(),
                      'worktree': str(worktree), 'output_dir': str(destination), 'command_expanded': command,
                      'cwd': str(cwd), 'locks': [str(h.name) for h in handles],
                      'contract_sha256': {p.name: sha(p) for p in (coord / 'contracts').iterdir() if p.is_file()},
                      'configuration': spec.get('configuration', {}), 'data': spec.get('data', {}), 'cpu_affinity': affinity,
                      'diagnostic_level': spec.get('diagnostic_level', 'UNSPECIFIED'), 'historical_reuse': bool(spec.get('historical_reuse', False)),
                      'source_snapshot': spec.get('source_snapshot'), 'cache_schema': spec.get('cache_schema'),
                      'source_files_sha256': {expand(p): sha(Path(expand(p))) for p in spec.get('source_files', [])}}
        for category in ('config_paths', 'binary_paths'):
            identities[category] = {expand(p): {'resolved': str(Path(expand(p)).resolve()), 'sha256': sha(Path(expand(p)))} for p in spec.get(category, [])}
        for path in spec.get('binary_paths', []):
            binary = Path(expand(path))
            ldd = subprocess.run(['ldd', str(binary)], env=env, capture_output=True, text=True)
            (destination / (binary.name + '.ldd.txt')).write_text(ldd.stdout + ldd.stderr)
        (destination / 'manifest_start.json').write_text(json.dumps({**base, **identities}, indent=2) + '\n')
        resources = destination / 'resources.json'
        timed = ['/usr/bin/time', '-q', '-f', '{"max_rss_kb":%M,"elapsed_s":%e,"user_s":%U,"system_s":%S}', '-o', str(resources), *command]
        seen, stop = set(), threading.Event()
        started = time.time()
        with (destination / 'stdout_stderr.log').open('w') as output:
            child = subprocess.Popen(timed, cwd=cwd, env=env, stdout=output, stderr=subprocess.STDOUT, start_new_session=True)
            pgid = os.getpgid(child.pid)
            def monitor():
                while not stop.is_set():
                    members = group(pgid)
                    rss_members = []
                    for p in members:
                        try:
                            lines = Path(f"/proc/{p['pid']}/status").read_text().splitlines()
                            memory = {line.split(':')[0]: int(line.split()[1]) for line in lines if line.startswith(('VmRSS:', 'VmHWM:'))}
                            rss_members.append({**p, **memory})
                        except (OSError, ValueError):
                            pass
                    append(destination / 'resource_samples.jsonl', {'time': time.time(), 'members': rss_members, 'sum_rss_kb': sum(m.get('VmRSS', 0) for m in rss_members)})
                    for p in members:
                        identity = (p['pid'], p['start_ticks'], p['executable'], p['command'])
                        if identity not in seen:
                            seen.add(identity)
                            append(registry, {**base, 'event': 'PROCESS_REGISTERED', 'time': time.time(), **p})
                    stop.wait(.2 if time.time() - started < 2 else 2)
            watcher = threading.Thread(target=monitor, daemon=True)
            watcher.start()
            append(registry, {**base, **identities, 'event': 'START', 'pid': child.pid, 'pgid': pgid,
                              'start_identity': process(child.pid), 'started_at': started})
            code = child.wait()
            while group(pgid):
                time.sleep(.5)  # Keep locks until all live group members finish.
            stop.set()
            watcher.join()
        changed = {p: {'before': h, 'after': sha(Path(p))} for p, h in identities['source_files_sha256'].items() if sha(Path(p)) != h}
        binary_changed = {p: {'before': value['sha256'], 'after': sha(Path(p))} for p, value in identities['binary_paths'].items()
                          if base['kind'] != 'build' and sha(Path(p)) != value['sha256']}
        result = {**base, **identities, 'event': 'FINISH', 'started_at': started, 'finished_at': time.time(),
                  'exit_code': code, 'source_mutation': changed, 'binary_mutation': binary_changed,
                  'registered_processes': len(seen), 'resources': json.loads(resources.read_text()) if resources.exists() else None,
                  'validity': 'INVALID_ARTIFACT_MUTATION' if changed or binary_changed else 'EXIT_ZERO' if code == 0 else 'FAILED_RETAINED'}
        (destination / 'manifest_finish.json').write_text(json.dumps(result, indent=2) + '\n')
        append(registry, result)
        print(json.dumps({'run_id': run_id, 'exit_code': code, 'validity': result['validity'], 'output_dir': str(destination)}), flush=True)
        return code or (2 if changed or binary_changed else 0)
    except BaseException as error:
        # An interrupted launcher retains locks until its own process group is
        # terminal. No unverified signal, and no finally-unlock with live children.
        if child is not None:
            while child.poll() is None or (pgid is not None and group(pgid)):
                try:
                    child.wait(timeout=1)
                    time.sleep(.2)
                except (subprocess.TimeoutExpired, KeyboardInterrupt):
                    continue
        append(registry, {**base, 'event': 'LAUNCHER_FAILURE', 'error': repr(error), 'time': time.time()})
        raise
    finally:
        if base['performance']:
            marker = coord / 'performance_active.json'
            if marker.exists() and json.loads(marker.read_text())['run_id'] == run_id:
                marker.unlink()
        for handle in reversed(handles):
            handle.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('spec', type=Path)
    args = parser.parse_args()
    raise SystemExit(run(args.spec))
