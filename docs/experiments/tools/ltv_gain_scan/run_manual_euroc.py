#!/usr/bin/env python3
"""Run an existing EuRoC experiment binary using a user-editable YAML."""
import argparse
from contextlib import ExitStack
import csv
from datetime import datetime, timezone
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import uuid

import yaml

ROOT = Path(__file__).resolve().parents[4]
ARCHIVE = ROOT / 'docs/euroc_final_results_data'
DEFAULT = ROOT / 'config/euroc_final/manual_run.yaml'
BRANCHES = {'OFF': (), 'G': ('G',), 'V': ('V',), 'L': ('L',), 'GVL': ('G', 'V', 'L')}
MODE_KEYS = {'ltv_enabled', 'ltv_enable_gravity', 'ltv_enable_velocity', 'ltv_enable_landmark_approx'}


def read(path):
    return json.loads(path.read_text())


def digest(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def resolve(config, short_override=None):
    if not isinstance(config, dict):
        raise ValueError('YAML must contain a mapping')
    allowed = {'sequence', 'mode', 'weights', 'dataset_root', 'install_root', 'output_root', 'cpu_affinity', 'short_seconds'}
    if set(config) - allowed:
        raise ValueError('Unknown YAML keys: ' + ', '.join(sorted(set(config) - allowed)))
    protocol = read(ARCHIVE / 'protocol.json')
    seq, mode = config.get('sequence'), config.get('mode')
    if seq not in protocol['sequences']:
        raise ValueError('sequence must be one of: ' + ', '.join(protocol['sequences']))
    if mode not in BRANCHES:
        raise ValueError('mode must be OFF, G, V, L or GVL')
    scene = 'MH' if seq.startswith('MH') else 'Vicon'
    weights = dict(zip(('G', 'V', 'L'), protocol['profiles'][scene][mode]))
    overrides = config.get('weights') or {}
    if not isinstance(overrides, dict) or set(overrides) - {'G', 'V', 'L'}:
        raise ValueError('weights accepts only G, V and L')
    for branch, value in overrides.items():
        if value is None:
            continue
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError('weight must be a finite number: ' + branch)
        if (branch in BRANCHES[mode] and value <= 0) or (branch not in BRANCHES[mode] and value != 0):
            raise ValueError('enabled weights must be positive; disabled weights must be 0/null: ' + branch)
        weights[branch] = value
    seconds = config.get('short_seconds', 0) if short_override is None else short_override
    if isinstance(seconds, bool) or not isinstance(seconds, (int, float)) or not math.isfinite(seconds) or not 0 <= seconds <= 40:
        raise ValueError('short_seconds must be 0 (full) or a number in (0,40]')
    cpus = config.get('cpu_affinity', [0, 1])
    if not isinstance(cpus, list) or not cpus or any(type(c) is not int for c in cpus):
        raise ValueError('cpu_affinity must be a nonempty list of CPU numbers')
    if len(set(cpus)) != len(cpus) or not set(cpus) <= os.sched_getaffinity(0):
        raise ValueError('cpu_affinity contains duplicate or unavailable CPUs')
    identity = read(ARCHIVE / 'identity.json')
    install = Path(config.get('install_root') or (Path(identity['artifact']) / 'install')).expanduser().resolve()
    binary = install / 'ov_msckf/lib/ov_msckf/run_ltv_gv_production'
    if not binary.is_file() or not os.access(binary, os.X_OK) or not (install / 'local_setup.bash').is_file():
        raise ValueError('existing experiment installation unavailable: ' + str(install))
    data = Path(config.get('dataset_root', '/home/he/datasets/euroc/ASL')).expanduser().resolve() / seq / 'mav0'
    for name in ['cam0/data.csv', 'cam1/data.csv', 'imu0/data.csv']:
        if not (data / name).is_file():
            raise ValueError('missing ASL sensor input: ' + str(data / name))
    g, v, l = (weights[b] for b in ('G', 'V', 'L'))
    parameters = dict(ltv_enabled=mode != 'OFF', ltv_enable_gravity='G' in BRANCHES[mode],
                      ltv_enable_velocity='V' in BRANCHES[mode], ltv_enable_landmark_approx='L' in BRANCHES[mode],
                      ltv_sigma_gravity_deg=10 / math.sqrt(g or 1), ltv_sigma_velocity_mps=1 / math.sqrt(v or 1),
                      ltv_landmark_approx_sigma_floor_m=.5 / math.sqrt(l or 1),
                      ltv_landmark_approx_information_cap=.01 * (l or 1), ltv_landmark_approx_max_points=2,
                      max_slam=0, num_opencv_threads=1)
    return dict(sequence=seq, scene=scene, mode=mode, native_mode='L_GV' if mode == 'GVL' else mode,
                weights=weights, parameters=parameters, install_root=str(install), binary=str(binary),
                data=str(data), cpu_affinity=cpus, short_seconds=seconds,
                frozen_profile=tuple(weights.values()) == tuple(protocol['profiles'][scene][mode]))


def prepare(config_path, config, selection):
    parent = Path(config.get('output_root', '/tmp/openvins_ltv_manual')).expanduser().resolve()
    name = f"{selection['sequence']}_{selection['mode']}_{datetime.now(timezone.utc):%Y%m%dT%H%M%SZ}_{uuid.uuid4().hex[:8]}"
    output = parent / name
    output.mkdir(parents=True, exist_ok=False)
    folder = output / 'config'
    shutil.copytree(ARCHIVE / 'configs' / selection['scene'] / selection['mode'], folder)
    target = folder / 'estimator_config.yaml'
    target.chmod(target.stat().st_mode | 0o200)
    text = target.read_text()
    for key, value in selection['parameters'].items():
        # The frozen runner validates its managed Passive setup first, then applies
        # the native mode. Preserve the template switches until that mode dispatch.
        if key in MODE_KEYS:
            continue
        value = str(value).lower() if isinstance(value, bool) else str(value)
        pattern = r'^' + re.escape(key) + r':.*$'
        if not re.search(pattern, text, re.M):
            raise ValueError('missing estimator key: ' + key)
        text = re.sub(pattern, key + ': ' + value, text, flags=re.M)
    target.write_text(text)
    shutil.copy2(config_path, output / 'manual_run.yaml')
    selection['binary_sha256'] = digest(Path(selection['binary']))
    selection['generated_config_sha256'] = digest(target)
    (output / 'selection.json').write_text(json.dumps(selection, indent=2) + '\n')
    return output, target


def execute(output, target, selection):
    # Coordinate with the existing experiment launchers without changing their lock files.
    lock_root = Path('/home/he/output/ltv_shared_replay_locks')
    with ExitStack() as stack:
        for name, lock_mode in [('build_replay.lock', fcntl.LOCK_SH), ('performance_gate.lock', fcntl.LOCK_EX)]:
            handle = stack.enter_context((lock_root / name).open('r'))
            try:
                fcntl.flock(handle, lock_mode | fcntl.LOCK_NB)
            except BlockingIOError as error:
                raise RuntimeError('Another experiment holds ' + name + '; retry after it finishes') from error
        env = dict(os.environ)
        for key in list(env):
            if key.startswith(('LTV_', 'ROS_', 'AMENT_', 'COLCON_', 'CCACHE_')) or key in ('CMAKE_PREFIX_PATH', 'LD_LIBRARY_PATH', 'PYTHONPATH'):
                env.pop(key, None)
        env.update(OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1', MKL_NUM_THREADS='1', NUMEXPR_NUM_THREADS='1',
                   ROS_LOG_DIR=str(output / 'ros_logs'))
        # Paths are positional arguments, never interpolated into shell source.
        shell = 'set -e; source /opt/ros/humble/setup.bash; source "$1/local_setup.bash"; shift; exec "$@"'
        command = ['taskset', '-c', ','.join(map(str, selection['cpu_affinity'])), '/usr/bin/time', '-q',
                   '-f', '{"max_rss_kb":%M,"elapsed_s":%e,"user_s":%U,"system_s":%S}',
                   '-o', str(output / 'resources.json'), 'bash', '-c', shell, 'manual-euroc',
                   selection['install_root'], selection['binary'], str(target), selection['data'], str(output), selection['native_mode']]
        if selection['short_seconds']:
            command.append(str(selection['short_seconds']))
        with (output / 'stdout_stderr.log').open('w') as log:
            result = subprocess.run(command, cwd=output, env=env, stdout=log, stderr=subprocess.STDOUT)
        checks = []
        applications = {}
        if result.returncode == 0:
            effective = read(output / 'effective_options.json')
            for key, wanted in selection['parameters'].items():
                actual = effective['max_slam_features' if key == 'max_slam' else key]
                if isinstance(wanted, bool):
                    passed = actual == wanted
                else:
                    passed = math.isclose(actual, wanted, rel_tol=1e-12, abs_tol=1e-15)
                if not passed:
                    checks.append('effective parameter differs: ' + key)
            if read(output / 'threading.json')['actual_opencv_threads'] != 1:
                checks.append('OpenCV is not single threaded')
            replay = read(output / 'replay.json')
            if replay['complete'] != (selection['short_seconds'] == 0):
                checks.append('input interval differs')
            if selection['short_seconds'] == 0 and (replay['camera_packets'] != replay['input_camera_packets'] or
                                                   replay['imu_consumed'] != replay['input_imu_samples']):
                checks.append('full input was not consumed')
            if not math.isclose(replay['short_limit_seconds'], selection['short_seconds'], abs_tol=1e-12):
                checks.append('short duration differs')
            applications = dict(G=replay['actual_G_submissions'], V=replay['actual_V_submissions'], L=0)
            landmark = output / 'landmark_approx.csv'
            if landmark.exists():
                with landmark.open() as stream:
                    applications['L'] = sum(int(r['rows']) > 0 and r['consumed'] == '1' for r in csv.DictReader(stream))
            for branch, count in applications.items():
                if branch not in BRANCHES[selection['mode']] and count:
                    checks.append('disabled branch applied: ' + branch)
            if not (output / 'trajectory.csv').is_file() or replay['output_rows'] <= 0:
                checks.append('no initialized trajectory; try full replay or a different short-check sequence')
        status = dict(exit_code=result.returncode, errors=checks, actual_applications=applications,
                      status='PASS' if result.returncode == 0 and not checks else 'FAIL',
                      scope='Manual replay and effective configuration; not final experiment acceptance')
        (output / 'manual_validation.json').write_text(json.dumps(status, indent=2) + '\n')
        if status['status'] != 'PASS':
            raise RuntimeError('Replay failed; inspect ' + str(output / 'stdout_stderr.log') + ': ' + str(checks))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('yaml_path', nargs='?', type=Path, default=DEFAULT)
    parser.add_argument('--prepare-only', action='store_true', help='Generate config and selection without replay')
    parser.add_argument('--seconds', type=float, help='Override YAML duration: 0=full, (0,40]=short')
    args = parser.parse_args()
    config_path = args.yaml_path.expanduser().resolve()
    config = yaml.safe_load(config_path.read_text())
    selection = resolve(config, args.seconds)
    output, target = prepare(config_path, config, selection)
    print('Output: ' + str(output), flush=True)
    print('Selection: ' + selection['sequence'] + ' / ' + selection['mode'] + ' / ' + str(selection['weights']), flush=True)
    print('Estimator YAML: ' + str(target), flush=True)
    if args.prepare_only:
        return
    execute(output, target, selection)
    print('PASS; trajectory: ' + str(output / 'trajectory.csv'), flush=True)


if __name__ == '__main__':
    try:
        main()
    except (OSError, ValueError, RuntimeError, KeyError, yaml.YAMLError) as error:
        print('ERROR: ' + str(error), file=sys.stderr)
        sys.exit(1)
