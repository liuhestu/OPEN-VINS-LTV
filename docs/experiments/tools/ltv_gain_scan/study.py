#!/usr/bin/env python3
"""UZH reference and independently gated two/four-fold information scans.

Raw archive sensors are converted without trimming. No GT enters the estimator.
The coordinator alone writes indices; launchers own worker/process registries.
"""
import argparse
from concurrent.futures import ThreadPoolExecutor
from decimal import Decimal, ROUND_HALF_EVEN
import hashlib
import json
import csv
import os
from pathlib import Path
import re
import queue
import shutil
import subprocess
import sys
import time
import numpy as np

TOOLS = Path(__file__).resolve().parents[1] / 'ltv_three_track'
sys.path.insert(0, str(TOOLS))
from evaluate_euroc_six import nearest, load_trajectory, aligned_rmse
from run_euroc_six import MODES, FLAGS, verify_output, SEQUENCES
from launcher import sha, process, group
UZH = ['indoor_forward_5', 'indoor_forward_6', 'indoor_forward_7', 'indoor_45_2', 'outdoor_forward_1', 'outdoor_forward_5']
EUROC = [s for s in SEQUENCES if s != 'MH_04_difficult']
REP = ['V2_02_medium', 'V2_03_difficult', 'MH_05_difficult', 'indoor_forward_5', 'outdoor_forward_5']
UZH_TUNE = [seq for seq in UZH if seq != 'indoor_forward_7']
ROOT = Path(__file__).resolve().parents[4]


def write(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False) + '\n')
    temporary.replace(path)


def table_csv(path, rows):
    fields = sorted({k for row in rows for k in row})
    with path.open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fields, lineterminator='\n')
        writer.writeheader()
        writer.writerows(rows)


def family(seq):
    if seq in EUROC:
        return 'euroc'
    return 'uzhfpv_indoor_45' if seq.startswith('indoor_45') else ('uzhfpv_outdoor' if seq.startswith('outdoor') else 'uzhfpv_indoor')


def set_keys(text, values):
    for key, value in values.items():
        line = f'{key}: {value}'
        if re.search(r'^' + key + ':', text, re.M):
            text = re.sub(r'^' + key + r':.*$', line, text, flags=re.M)
        else:
            text += '\n' + line + '\n'
    return text


def share_calibration_evidence(root):
    """Keep published calibration/masks once per family without changing bytes."""
    for family_root in root.iterdir():
        groups = [path for path in family_root.iterdir() if path.is_dir() and path.name != '_shared']
        shared = family_root / '_shared'
        shared.mkdir(exist_ok=True)
        for group_root in groups:
            for path in group_root.iterdir():
                if not path.is_file() or path.name == 'estimator_config.yaml': continue
                target = shared / path.name
                if not target.exists(): shutil.copy2(path, target)
                if path.read_bytes() != target.read_bytes():
                    raise ValueError('calibration evidence differs: ' + str(path))
                path.unlink()
                path.symlink_to(Path('../_shared') / path.name)


def interpolate_gt(gt, times):
    """Positions only; no extrapolation or interpolation across >10 ms gaps."""
    hi = np.searchsorted(gt[:, 0], times, side='left').clip(0, len(gt) - 1)
    lo = np.maximum(0, hi - 1)
    exact = times == gt[hi, 0]
    span = gt[hi, 0] - gt[lo, 0]
    valid = exact | ((span <= .01) & (span > 0) & (times >= gt[0, 0]) & (times <= gt[-1, 0]))
    weight = np.divide(times - gt[lo, 0], span, out=np.zeros(len(times)), where=span > 0)
    positions = gt[lo, 1:4] + weight[:, None] * (gt[hi, 1:4] - gt[lo, 1:4])
    positions[exact] = gt[hi[exact], 1:4]
    return valid, positions


def gate(rows):
    """Strict >10%; decimal comparison avoids classifying exactly 10% as blocked."""
    reasons = []
    for row in rows:
        seq = row['sequence']
        if row.get('status') != 'VALID_FULL_MATCHED':
            reasons.append(f"{seq}: {row.get('reason', row.get('status'))}")
        for name in ('off_ate', 'reference_ate'):
            baseline, value = row.get(name), row.get('ate_rmse_m')
            if baseline is None or value is None or not np.isfinite([baseline, value]).all() or baseline <= 0:
                reasons.append(f'{seq}: unable to screen {name}')
            elif Decimal(str(value)) > Decimal(str(baseline)) * Decimal('1.1'):
                reasons.append(f'{seq}: ATE >10% above {name}')
    if {r['sequence'] for r in rows} != set(REP) or len(rows) != len(REP):
        reasons.append('missing or duplicate representative')
    return {'status': 'BLOCKED' if reasons else 'PASS', 'reasons': reasons}


def prepare(a):
    coord = a.coord.resolve()
    coord.mkdir(parents=True, exist_ok=True)
    if (coord / 'data_identity.json').exists(): raise RuntimeError('already prepared; use run')
    for name in ('locks', 'registry', 'contracts', 'tools', 'specs', 'support'):
        (coord / name).mkdir(exist_ok=True)
    (coord / 'contracts/empty_colcon_defaults.yaml').write_text('{}\n')
    for path in TOOLS.glob('*.py'):
        shutil.copy2(path, coord / 'tools' / path.name)
    shutil.copy2(TOOLS / 'env_exec.sh', coord / 'tools/env_exec.sh')
    cpus = sorted(os.sched_getaffinity(0))
    if len(cpus) < 16:
        raise RuntimeError('four disjoint four-CPU workers require >=16 allowed CPUs')
    tasks = {}
    for number in range(4):
        name = f'w{number}'
        task = coord / name
        task.mkdir(exist_ok=True)
        tasks[name] = dict(task_root=str(task), worktree=str(ROOT), namespace=f'ltv_scan_{name}', ros_domain_id=201 + number,
                           cpu_affinity=cpus[number * 4:number * 4 + 4])
    write(coord / 'session.json', {'tasks': tasks})
    oid = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip()
    source = coord / 'source'
    if not source.exists():
        source.mkdir()
        archive = subprocess.Popen(['git', 'archive', oid], cwd=ROOT, stdout=subprocess.PIPE)
        subprocess.run(['tar', '-x', '-C', str(source)], stdin=archive.stdout, check=True)
        archive.stdout.close()
        assert archive.wait() == 0
    identity = dict(source_oid=oid, source=str(source), artifact=str(coord / ('artifact_retry' if (coord / 'artifact').exists() else 'artifact')), coord_root=str(coord),
                    base_main_oid=oid, origin=subprocess.check_output(['git', 'remote', 'get-url', 'origin'], cwd=ROOT, text=True).strip())
    write(coord / 'identity.json', identity)
    parent = json.loads((ROOT / 'docs/euroc_results_data/identity.json').read_text())
    command = ['bash', str(coord / 'tools/env_exec.sh'), '-', 'python3', str(TOOLS / 'task_a/build_scalar_runner.py'), parent['parent_library_artifact'],
               str(Path(parent['parent_library_artifact']).parents[1] / 'source' / parent['parent_library_source_oid']),
               str(source), identity['artifact'], str(coord / ('build_evidence_retry' if (coord / 'build_evidence').exists() else 'build_evidence'))]
    spec = dict(coord_root=str(coord), task='w0', label='build_retry' if (coord / 'artifact').exists() else 'build', kind='build', four_worker_pool=True,
                command=command, configuration={'fresh_runner': True, 'verified_dependency_closure': True})
    write(coord / 'specs/build.json', spec)
    subprocess.run(['python3', str(coord / 'tools/launcher.py'), str(coord / 'specs/build.json')], check=True)
    # Preserve all UZH physical settings; inherit only the current LTV key block.
    c0 = (source / 'docs/experiments/configs/euroc_c0/estimator_config.yaml').read_text()
    ltv = {m[0]: m[1] for m in re.findall(r'^(ltv_\w+):\s*(.*)$', c0, re.M)}
    for fam in ['euroc', 'uzhfpv_indoor', 'uzhfpv_indoor_45', 'uzhfpv_outdoor']:
        base = source / ('docs/experiments/configs/euroc_c0' if fam == 'euroc' else f'config/ltv_value/{fam}')
        for mode in MODES:
            for alpha in ([1] if mode == 'OFF' else [1, 2, 4]):
                dest = coord / 'configs' / fam / f'{mode}_{alpha}'
                shutil.copytree(base, dest)
                config = dest / 'estimator_config.yaml'
                text = config.read_text()
                if fam != 'euroc':
                    text = set_keys(text, ltv)
                enable_g, enable_v, enable_l = FLAGS[mode][1:]
                text = set_keys(text, dict(max_slam=0, ltv_sigma_gravity_deg=10 / np.sqrt(alpha if enable_g else 1),
                    ltv_sigma_velocity_mps=1 / np.sqrt(alpha if enable_v else 1),
                    ltv_landmark_approx_sigma_floor_m=.5 / np.sqrt(alpha if enable_l else 1),
                    ltv_landmark_approx_information_cap=.01 * (alpha if enable_l else 1), ltv_landmark_approx_max_points=2))
                config.write_text(text)
    inputs = {}
    for seq in UZH + EUROC:
        if seq in EUROC:
            sensor = a.euroc / seq / 'mav0'
            gt = sensor / 'state_groundtruth_estimate0/data.csv'
            original = sensor
        else:
            original = a.uzh / f'{seq}_snapdragon_with_gt'
            sensor = coord / 'inputs' / seq
            sensor.mkdir(parents=True)
            for i, side in enumerate(('left', 'right')):
                dest = sensor / f'cam{i}'
                (dest / 'data').mkdir(parents=True)
                with (dest / 'data.csv').open('w') as stream:
                    stream.write('#timestamp [ns],filename\n')
                    for line in (original / f'{side}_images.txt').read_text().splitlines():
                        if not line or line.startswith('#'): continue
                        _, timestamp, name = line.split()
                        ns = int((Decimal(timestamp) * 10**9).to_integral_value(rounding=ROUND_HALF_EVEN))
                        image = Path(name).name
                        (dest / 'data' / image).symlink_to(original / name)
                        stream.write(f'{ns},{image}\n')
            (sensor / 'imu0').mkdir()
            with (sensor / 'imu0/data.csv').open('w') as stream:
                stream.write('#timestamp [ns],wx,wy,wz,ax,ay,az\n')
                for line in (original / 'imu.txt').read_text().splitlines():
                    if not line or line.startswith('#'): continue
                    _, timestamp, *values = line.split()
                    ns = int((Decimal(timestamp) * 10**9).to_integral_value(rounding=ROUND_HALF_EVEN))
                    stream.write(','.join([str(ns), *values]) + '\n')
            gt = original / 'groundtruth.txt'
        hashes = {}
        for path in sorted(sensor.rglob('*')):
            if path.is_file(): hashes[str(path.relative_to(sensor))] = sha(path)
        write(coord / 'inputs' / f'{seq}_hashes.json', hashes)
        inputs[seq] = dict(root=str(sensor), original=str(original), gt=str(gt), gt_sha256=sha(gt),
                          sensor_hash_manifest=str(coord / 'inputs' / f'{seq}_hashes.json'),
                          sensor_hash_manifest_sha256=sha(coord / 'inputs' / f'{seq}_hashes.json'))
        print('hashed input', seq, len(hashes), flush=True)
    write(coord / 'data_identity.json', inputs)
    for folder in [source, coord / 'configs', coord / 'inputs']:
        for path in folder.rglob('*'):
            if path.is_file() and not path.is_symlink(): path.chmod(path.stat().st_mode & ~0o222)
    write(coord / 'index.json', [])
    write(coord / 'screening.json', [])
    print('prepared', coord, flush=True)


class Study:
    def __init__(self, coord):
        self.coord = coord.resolve()
        self.identity = json.loads((coord / 'identity.json').read_text())
        self.inputs = json.loads((coord / 'data_identity.json').read_text())
        self.index = json.loads((coord / 'index.json').read_text())
        self.screening = json.loads((coord / 'screening.json').read_text())

    def lookup(self, seq, mode, alpha=1):
        return next((r for r in self.index if (r['sequence'], r['mode'], r['alpha']) == (seq, mode, alpha)), None)

    def evaluate(self, row):
        seq = row['sequence']
        result = dict(ate_rmse_m=None, status='FAIL', reason='', samples=0, baseline_support=0, coverage=0.,
                      initialization_delta_s=None, gt_coverage=0.)
        try:
            x = load_trajectory(Path(row['output_dir']) / 'trajectory.csv')
            if seq in EUROC:
                gt = np.loadtxt(self.inputs[seq]['gt'], delimiter=',', comments='#', ndmin=2)[:, :4]
                gt[:, 0] *= 1e-9
                gi = nearest(gt[:, 0], x[:, 0])
                valid = (abs(gt[gi, 0] - x[:, 0]) <= .02) & (x[:, 0] >= gt[0, 0]) & (x[:, 0] <= gt[-1, 0])
                truth = gt[gi, 1:4]
            else:
                gt = np.loadtxt(self.inputs[seq]['gt'], comments='#', ndmin=2)[:, :4]
                valid, truth = interpolate_gt(gt, x[:, 0])
            if not np.isfinite(gt).all() or not np.all(np.diff(gt[:, 0]) > 0):
                raise ValueError('invalid GT positions/times')
            result['gt_coverage'] = float(valid.mean())
            result['estimated_rows'] = len(x)
            support_path = self.coord / 'support' / f'{seq}.npz'
            if row['mode'] == 'OFF':
                if row['exit_code'] != 0 or row['audit']['errors'] or valid.sum() < 3:
                    raise ValueError('invalid OFF runtime/input/flags or insufficient GT support')
                if support_path.exists():
                    frozen = np.load(support_path)
                    assert np.array_equal(frozen['times'], x[valid, 0]) and np.array_equal(frozen['truth'], truth[valid])
                else:
                    np.savez(support_path, times=x[valid, 0], truth=truth[valid], initialization=x[0, 0])
                    support_path.chmod(0o444)
            frozen = np.load(support_path)
            times, truth = frozen['times'], frozen['truth']
            xi = nearest(x[:, 0], times)
            matched = abs(x[xi, 0] - times) <= 1e-6
            result.update(samples=int(matched.sum()), baseline_support=len(times), coverage=float(matched.mean()),
                          initialization_delta_s=float(x[0, 0] - frozen['initialization']))
            if matched.sum() >= 3:
                ate, horn = aligned_rmse(x[xi[matched], 5:8], truth[matched])
                result.update(conditional_ate_rmse_m=ate, svd_horn_difference_m=abs(ate - horn))
                # Diagnostic aligned last-10% positional RMSE, using full support alignment.
                xx, yy = x[xi[matched], 5:8], truth[matched]
                u, _, vt = np.linalg.svd((xx - xx.mean(0)).T @ (yy - yy.mean(0)))
                r = vt.T @ np.diag([1., 1., np.linalg.det(vt.T @ u.T)]) @ u.T
                errors = np.linalg.norm((xx - xx.mean(0)) @ r.T - (yy - yy.mean(0)), axis=1)
                result['tail_position_rmse_m'] = float(np.sqrt(np.mean(errors[-max(1, len(errors)//10):]**2)))
                result['position_error_p95_m'] = float(np.percentile(errors, 95))
                result['position_error_max_m'] = float(errors.max())
            if row['exit_code'] == 0 and not row['audit']['errors'] and matched.all() and len(times) >= 3 and abs(result['initialization_delta_s']) <= 1e-6:
                result.update(status='VALID_FULL_MATCHED', ate_rmse_m=result['conditional_ate_rmse_m'])
            else:
                result['reason'] = 'runtime/input/flags/coverage/initialization mismatch'
        except (OSError, ValueError, KeyError, AssertionError) as error:
            result['reason'] = str(error)
        return result

    def run_one(self, job, worker):
        seq, mode, alpha, short, label = job
        config = self.coord / 'configs' / family(seq) / f'{mode}_{alpha}' / 'estimator_config.yaml'
        install = Path(self.identity['artifact']) / 'install'
        binary = install / 'ov_msckf/lib/ov_msckf/run_ltv_gv_production'
        command = ['bash', str(self.coord / 'tools/env_exec.sh'), str(install), str(binary), str(config), self.inputs[seq]['root'], '{RUN_DIR}', mode]
        if short: command.append(str(short))
        spec = dict(coord_root=str(self.coord), task=f'w{worker}', label=f'{label}_{seq}_{mode}_{alpha}', kind='replay', four_worker_pool=True,
                    source_oid=self.identity['source_oid'], source_snapshot=self.identity['source'], command=command,
                    binary_paths=[str(binary)], config_paths=[str(p) for p in config.parent.iterdir() if p.is_file()],
                    configuration=dict(sequence=seq, mode=mode, alpha=alpha, short_seconds=short), data=self.inputs[seq], diagnostic_level='scalar_ltv_csv')
        path = self.coord / 'specs' / f'{label}_{seq}_{mode}_{alpha}.json'
        write(path, spec)
        subprocess.run(['python3', str(self.coord / 'tools/launcher.py'), str(path)], check=False, stdout=subprocess.DEVNULL)
        events = [json.loads(line) for line in (self.coord / f'registry/task_w{worker}.jsonl').read_text().splitlines()]
        event = next(e for e in reversed(events) if e.get('spec_path') == str(path) and e['event'] in ('FINISH', 'LAUNCHER_FAILURE'))
        out = Path(event.get('output_dir', ''))
        row = dict(sequence=seq, mode=mode, alpha=alpha, run_id=event['run_id'], output_dir=str(out), exit_code=event.get('exit_code'),
                   audit=verify_output(out, mode, not short) if event.get('exit_code') == 0 else {'errors': ['runtime failure']})
        if event.get('source_mutation') or event.get('binary_mutation'):
            row['audit']['errors'].append('artifact mutation')
        if row['exit_code'] == 0:
            options = json.loads((out / 'effective_options.json').read_text())
            threading_path = out / 'threading.json'
            if threading_path.exists():
                row['threading'] = json.loads(threading_path.read_text())
                if row['threading'].get('actual_opencv_threads') != 1:
                    row['audit']['errors'].append('OpenCV thread isolation mismatch')
            else:
                row['audit']['errors'].append('missing post-construction thread verification')
            wanted = dict(ltv_sigma_gravity_deg=10 / np.sqrt(alpha if FLAGS[mode][1] else 1),
                ltv_sigma_velocity_mps=1 / np.sqrt(alpha if FLAGS[mode][2] else 1),
                ltv_landmark_approx_sigma_floor_m=.5 / np.sqrt(alpha if FLAGS[mode][3] else 1),
                ltv_landmark_approx_information_cap=.01 * (alpha if FLAGS[mode][3] else 1), ltv_landmark_approx_max_points=2)
            for key, value in wanted.items():
                if not np.isclose(options[key], value, rtol=1e-14, atol=0): row['audit']['errors'].append('gain mismatch ' + key)
            landmark = out / 'landmark_approx.csv'
            if landmark.exists():
                entries = list(csv.DictReader(landmark.open()))
                information = [float(r['information_budget_used']) for r in entries if 'information_budget_used' in r and r['consumed'] == '1']
                row['landmark_information'] = dict(count=len(information), total=sum(information), maximum=max(information, default=0))
            ltv_csv = out / 'ltv.csv'
            if ltv_csv.exists():
                entries = list(csv.DictReader(ltv_csv.open()))
                row['gate_reason_counts'] = {}
                for entry in entries:
                    for key in ('G_reason', 'V_reason'):
                        if key in entry:
                            reason = key + ':' + entry[key]
                            row['gate_reason_counts'][reason] = row['gate_reason_counts'].get(reason, 0) + 1
        if not short: row.update(self.evaluate(row))
        return row

    def batch(self, jobs, short=False, serial=False):
        # A worker processes its assigned queue serially; no concurrent registry writers per worker.
        finished = queue.Queue()
        def work(worker):
            for job in jobs[worker::(1 if serial else 4)]:
                row = self.run_one(job, worker)
                finished.put(row)
        rows = []
        with ThreadPoolExecutor(max_workers=1 if serial else 4) as pool:
            futures = [pool.submit(work, n) for n in range(1 if serial else 4)]
            while len(rows) < len(jobs):
                try:
                    row = finished.get(timeout=1)
                except queue.Empty:
                    for future in futures:
                        if future.done(): future.result()
                    continue
                rows.append(row)
                if not short:
                    self.index.append(row)
                    write(self.coord / 'index.json', self.index)
                print(json.dumps({k: row.get(k) for k in ('sequence', 'mode', 'alpha', 'status', 'ate_rmse_m', 'reason')}), flush=True)
            for future in futures: future.result()
        return rows

    def jobs(self, sequences, modes, alphas):
        jobs = []
        for seq in sequences:
            off = self.lookup(seq, 'OFF')
            for mode in modes:
                for alpha in alphas:
                    if self.lookup(seq, mode, alpha): continue
                    if seq == 'indoor_forward_7':
                        self.index.append(dict(sequence=seq, mode=mode, alpha=alpha, status='EXCLUDED', reason='User exclusion: LTV never applied in this sequence', ate_rmse_m=None))
                        continue
                    if mode != 'OFF' and (off is None or off['status'] != 'VALID_FULL_MATCHED'):
                        self.index.append(dict(sequence=seq, mode=mode, alpha=alpha, status='SKIPPED', reason='OFF validation failed', ate_rmse_m=None))
                    else:
                        jobs.append((seq, mode, alpha, None, 'full'))
        write(self.coord / 'index.json', self.index)
        return jobs

    def smoke(self):
        seq = UZH[0]
        jobs = [(seq, mode, 1, 40, 'smoke_serial') for mode in MODES] + [(seq, 'L_GV', 4, 40, 'smoke_serial')]
        serial = self.batch(jobs, short=True, serial=True)
        for row in serial:
            load_trajectory(Path(row['output_dir']) / 'trajectory.csv')
        concurrent = self.batch([(s, m, a, t, 'smoke_concurrent') for s, m, a, t, _ in jobs], short=True)
        results = []
        for left, right in zip(serial, sorted(concurrent, key=lambda r: [(j[1], j[2]) for j in jobs].index((r['mode'], r['alpha'])))):
            same = all((Path(left['output_dir']) / name).read_bytes() == (Path(right['output_dir']) / name).read_bytes()
                       for name in ('trajectory.csv', 'replay.json', 'effective_options.json'))
            results.append(dict(mode=left['mode'], alpha=left['alpha'], identical=same,
                                serial=left, concurrent=right))
        write(self.coord / 'smoke.json', results)
        if any(not r['identical'] or r['serial']['audit']['errors'] or r['concurrent']['audit']['errors'] for r in results):
            raise RuntimeError('smoke failed; retained all attempts; do not start full batch')

    def execute(self):
        if not (self.coord / 'smoke.json').exists(): self.smoke()
        else:
            smoke = json.loads((self.coord / 'smoke.json').read_text())
            if len(smoke) != 7 or any(not r['identical'] or r['serial']['audit']['errors'] or r['concurrent']['audit']['errors'] for r in smoke):
                raise RuntimeError('saved smoke gate failed')
        self.batch(self.jobs(UZH, ['OFF'], [1]))
        self.batch(self.jobs(UZH, MODES[1:], [1]))
        self.publish()
        self.batch(self.jobs(REP[:3], ['OFF'], [1]))
        self.batch(self.jobs(REP[:3], MODES[1:], [1]))
        for alpha in [2, 4]:
            self.batch(self.jobs(REP, MODES[1:], [alpha]))
            for mode in MODES[1:]:
                rows = []
                for seq in REP:
                    row = dict(self.lookup(seq, mode, alpha))
                    row['off_ate'] = (self.lookup(seq, 'OFF') or {}).get('ate_rmse_m')
                    row['reference_ate'] = (self.lookup(seq, mode) or {}).get('ate_rmse_m')
                    rows.append(row)
                self.screening = [s for s in self.screening if (s['mode'], s['alpha']) != (mode, alpha)]
                self.screening.append(dict(mode=mode, alpha=alpha, **gate(rows), representatives=rows))
            write(self.coord / 'screening.json', self.screening)
            self.publish()
        passed = [s for s in self.screening if s['status'] == 'PASS']
        remaining = [s for s in EUROC + UZH if s not in REP]
        if passed:
            self.batch(self.jobs([s for s in remaining if s in EUROC], ['OFF'], [1]))
            for mode in MODES[1:]:
                if any(s['mode'] == mode for s in passed):
                    self.batch(self.jobs([s for s in remaining if s in EUROC], [mode], [1]))
        for screen in self.screening:
            if screen['status'] == 'PASS':
                self.batch(self.jobs(remaining, [screen['mode']], [screen['alpha']]))
            else:
                for seq in remaining:
                    if not self.lookup(seq, screen['mode'], screen['alpha']):
                        self.index.append(dict(sequence=seq, mode=screen['mode'], alpha=screen['alpha'], status='BLOCKED',
                                               reason='未启动: representative screening blocked', ate_rmse_m=None))
        # Account for unused EuRoC reference cases as explicit skips.
        for seq in EUROC + UZH:
            for mode in MODES:
                for alpha in ([1] if mode == 'OFF' else [1, 2, 4]):
                    if not self.lookup(seq, mode, alpha):
                        self.index.append(dict(sequence=seq, mode=mode, alpha=alpha, status='EXCLUDED' if seq == 'indoor_forward_7' else 'SKIPPED', reason='User exclusion: LTV never applied' if seq == 'indoor_forward_7' else '未启动: reference not required after screening', ate_rmse_m=None))
        write(self.coord / 'index.json', self.index)
        assert len(self.index) == 256 and len({(r['sequence'], r['mode'], r['alpha']) for r in self.index}) == 256
        live = []
        for path in (self.coord / 'registry').glob('*.jsonl'):
            for line in path.read_text().splitlines():
                event = json.loads(line)
                if event['event'] == 'PROCESS_REGISTERED':
                    current = process(event['pid'])
                    if current and current['state'] != 'Z' and all(current[k] == event[k] for k in ('pgid', 'start_ticks', 'executable')):
                        live.append(current)
        write(self.coord / 'completion_audit.json', dict(planned=256, recorded=len(self.index), owned_live_processes=live,
             status_counts={status: sum(r['status'] == status for r in self.index) for status in sorted({r['status'] for r in self.index})}))
        assert not live
        self.publish()

    def publish(self):
        uzdir = ROOT / 'docs/uzhfpv_results_data'
        tuned = ROOT / 'docs/euroc_tune_results'
        uzdir.mkdir(exist_ok=True)
        tuned.mkdir(exist_ok=True)
        exported = []
        for r in self.index:
            row = dict(r)
            value = r.get('ate_rmse_m')
            for key, mode in [('relative_off_percent', 'OFF'), ('relative_reference_percent', r['mode'])]:
                reference = self.lookup(r['sequence'], mode)
                baseline = reference.get('ate_rmse_m') if reference else None
                row[key] = 100 * (value / baseline - 1) if value is not None and baseline is not None and baseline > 0 else None
            exported.append(row)
        exported.sort(key=lambda r: ((EUROC + UZH).index(r['sequence']), MODES.index(r['mode']), r['alpha']))
        rows = [r for r in exported if r['sequence'] in UZH and r['alpha'] == 1]
        write(uzdir / 'results.json', rows)
        table_csv(uzdir / 'results.csv', rows)
        active_exported = [r for r in exported if r['sequence'] != 'indoor_forward_7']
        write(tuned / 'results.json', active_exported)
        table_csv(tuned / 'results.csv', active_exported)
        write(tuned / 'excluded_results.json', [r for r in exported if r['sequence'] == 'indoor_forward_7'])
        original_path = tuned / 'screening_original.json'
        original = json.loads(original_path.read_text()) if original_path.exists() else []
        seen = {(screen['mode'], screen['alpha']) for screen in original}
        original += [screen for screen in self.screening if (screen['mode'], screen['alpha']) not in seen]
        write(original_path, original)
        self.screening = [dict(mode=screen['mode'], alpha=screen['alpha'],
                              **gate([r for r in screen['representatives'] if r['sequence'] in REP]),
                              representatives=[r for r in screen['representatives'] if r['sequence'] in REP])
                          for screen in self.screening]
        write(tuned / 'screening.json', self.screening)
        table_csv(tuned / 'screening.csv', [{k:v for k,v in s.items() if k != 'representatives'} for s in self.screening])
        for dest in (uzdir, tuned):
            for name in ('identity.json', 'data_identity.json', 'session.json', 'smoke.json', 'completion_audit.json', 'independent_audit.json', 'analysis_identity.json', 'unit_validation.json', 'scope_amendment.json', 'calibration_audit.json', 'final_analysis_identity.json', 'amendment_validation.json', 'opencv_serial_fix_identity.json', 'opencv_serial_fix_validation.json', 'runtime_threading.json', 'raw_archive.json'):
                if (self.coord / name).exists(): shutil.copy2(self.coord / name, dest / name)
            if not (dest / 'configs').exists():
                if dest == tuned: shutil.copytree(self.coord / 'configs', dest / 'configs')
                else:
                    for fam in ('uzhfpv_indoor', 'uzhfpv_indoor_45', 'uzhfpv_outdoor'):
                        for mode in MODES:
                            shutil.copytree(self.coord / 'configs' / fam / f'{mode}_1', dest / 'configs' / fam / f'{mode}_1')
            for path in (self.coord / 'support').glob('*.npz'):
                if dest == tuned or path.stem in UZH:
                    (dest / 'support').mkdir(exist_ok=True)
                    if not (dest / 'support' / path.name).exists(): shutil.copy2(path, dest / 'support' / path.name)
            for seq in (UZH + EUROC if dest == tuned else UZH):
                (dest / 'input_hashes').mkdir(exist_ok=True)
                target = dest / 'input_hashes' / f'{seq}.json'
                if not target.exists(): shutil.copy2(self.inputs[seq]['sensor_hash_manifest'], target)

            manifests = [json.loads(p.read_text()) for p in self.coord.glob('w*/results/*/manifest_finish.json')]
            selected_manifests = manifests if dest == tuned else [m for m in manifests if m['kind'] == 'build' or
                (m.get('configuration', {}).get('sequence') in UZH and
                 (m['configuration'].get('short_seconds') or m['configuration'].get('alpha') == 1))]
            write(dest / 'run_manifest.json', selected_manifests)
            if dest == uzdir:
                write(dest / 'data_identity.json', {seq: value for seq, value in self.inputs.items() if seq in UZH})
            write(dest / 'support_hashes.json', {p.name: sha(p) for p in (self.coord / 'support').glob('*') if dest == tuned or p.stem in UZH})
            shutil.copy2(Path(self.identity['artifact']) / 'runner_build_identity.json', dest / 'runner_build_identity.json')
        header = ['# UZH-FPV 六模式 ATE RMSE', '', '原 LTV 配置；max_slam=0；完整原始流同步回放。位置 SE(3) 无尺度对齐，Horn/SVD 交叉核验。',
                  '保留 UZH 标定、噪声和初始化；秒制 GT 位置插值，不外推、不跨超过 10 ms 的间隙。',
                  '冻结 OFF 初始化后有效 GT 支持。GT 姿态不用于验收。历史 5% 预测 FAIL 和工程结论保留。',
                  '本轮每进程 OpenCV 实际为 4 线程，限制在独立四 CPU 内；构造覆盖问题已在专用入口修正并完成七组字节一致回归。原耗时不作单线程性能验收，见 runtime_threading.json。',
                  '完整输入回放不代表全程都有 GT：ATE 仅覆盖冻结的有效 GT 支持。尾部诊断为该支持最后 10% 的位置 RMSE，并非缺少 GT 的完整输入末尾。', '']
        text = header + self.matrix(UZH, 1, 'ATE (m)') + self.matrix(UZH, 1, '相对 OFF (%)', percent='OFF')
        text += ['', '| 序列 | 模式 | 状态 | 初始化差(s) | GT覆盖 | OFF支持覆盖 | G/V/L实际次数 | 原因 |', '|---|---|---|---:|---:|---:|---|---|']
        for row in rows:
            audit = row.get('audit', {})
            text.append(f"| {row['sequence']} | {row['mode']} | {row['status']} | {row.get('initialization_delta_s')} | {row.get('gt_coverage')} | {row.get('coverage')} | {audit.get('actual_G',0)}/{audit.get('actual_V',0)}/{audit.get('actual_L',0)} | {row.get('reason','')} |")
        text += ['', '## 异常与解释边界', '', '实际应用次数为零的分支没有生效，不能把相同 ATE 当作融合收益或压力测试通过。',
                 '本次所有输入均完整回放；条件 ATE 不替代完整有效结果；未调整参数、代表序列或评估支持。',
                 '构建准备曾有两次失败：未加载 ROS underlay 导致依赖检查退出 1；环境入口缺少执行权限导致退出 126。修正为 bash 入口加载 underlay 后独立构建成功；失败记录保留，失败产物未用于回放。']
        for row in rows:
            audit = row.get('audit', {})
            inactive = [branch for branch, enabled in zip(('G', 'V', 'L'), FLAGS[row['mode']][1:]) if enabled and audit.get('actual_' + branch, 0) == 0]
            if inactive:
                text.append(f"{row['sequence']} / {row['mode']}：{','.join(inactive)} 未实际应用；门控诊断 `{json.dumps(row.get('gate_reason_counts', {}), ensure_ascii=False)}`。")
        text += ['', f"源码 `{self.identity['source_oid']}`；原始输出 `{self.coord}`。", '[结果及诊断](uzhfpv_results_data/results.json) · [运行身份与命令](uzhfpv_results_data/run_manifest.json) · [位置证据及复算](uzhfpv_results_data/evaluation_archive.json)', '']
        (ROOT / 'docs/uzhfpv_results.md').write_text('\n'.join(text))
        text = ['# 代表序列增益扫描', '', '筛选 PASS 只允许扩展，不表示工程验收通过。历史预测 FAIL/工程 STOP 保留。',
                '模式和档位独立判定；相对 OFF 及同模式 1× ATE 升高严格超过 10% 时 BLOCKED。',
                'G 标准差 10°/√α，V 标准差 1 m/s/√α；L 下限 0.5 m/√α、信息上限 0.01α、最多两点。',
                'L 距离/历史项及 Observer Q/V/P0、门控与算法不变；不宣称 Kalman 增益严格翻倍。', '',
                '| 模式 | 档位 | 筛选 | 原因 |', '|---|---:|---|---|']
        for screen in self.screening:
            text.append(f"| {screen['mode']} | {screen['alpha']} | {screen['status']} | {'; '.join(screen['reasons'])} |")
        text += ['', '## 代表序列筛选（排除未参与融合的 indoor_forward_7）', '', '| 序列 | 模式 | 档位 | ATE(m) | 相对OFF | 相对1× | OFF覆盖 | 初始化差(s) | 状态 |', '|---|---|---:|---:|---:|---:|---:|---:|---|']
        for screen in self.screening:
            for row in screen['representatives']:
                def delta(key):
                    baseline = row.get(key)
                    value = row.get('ate_rmse_m')
                    if baseline is None or baseline <= 0 or value is None: return '无法筛选'
                    change = 100 * (value / baseline - 1)
                    cell = f'{change:+.6f}%'
                    return '**' + cell + '**' if change < 0 else cell
                text.append(f"| {row['sequence']} | {screen['mode']} | {screen['alpha']} | {row.get('ate_rmse_m')} | {delta('off_ate')} | {delta('reference_ate')} | {row.get('coverage')} | {row.get('initialization_delta_s')} | {row['status']} |")
        statuses = {status: sum(r['status'] == status for r in self.index) for status in sorted({r['status'] for r in self.index})}
        text += ['', f'原计划最多 256 个独立配置；原始记录 {len(self.index)} 个：`{json.dumps(statuses, ensure_ascii=False)}`。排除 indoor_forward_7 后，增益比较范围为 15 序列、240 个配置；已完成的排除序列结果保存在 excluded_results.json。',
                 'OFF 在各档表中复用同一次新参考回放；BLOCKED/SKIPPED 为明确未启动，不能作为数值结果。',
                 '全部有效组的冻结 OFF 支持覆盖为 100%，初始化差不超过 1 µs；各组 GT 匹配覆盖、尾部误差、实际信息量和门控原因见 results.json。',
                 'UZH OFF 有效 GT 匹配覆盖约 13.38%–70.89%；ATE 只在这部分冻结支持上有效。完整输入未裁剪；尾部诊断指匹配支持最后 10%，不外推到缺少 GT 的输入末尾。']
        for alpha in [1, 2, 4]:
            text += self.matrix(EUROC + UZH_TUNE, alpha, f'{alpha}× ATE (m)')
            text += self.matrix(EUROC + UZH_TUNE, alpha, f'{alpha}× 相对 OFF (%)', percent='OFF')
            if alpha > 1: text += self.matrix(EUROC + UZH_TUNE, alpha, f'{alpha}× 相对同模式 1× (%)', percent='reference')
        inactive = [(r['sequence'], r['mode'], r['alpha']) for r in self.index if r.get('status') == 'VALID_FULL_MATCHED' and r['mode'] != 'OFF' and r['sequence'] != 'indoor_forward_7' and not any(r.get('audit', {}).get('actual_' + branch, 0) for branch in ('G', 'V', 'L'))]
        text += ['', f'有效但全部启用分支实际应用为零的配置：`{inactive}`。筛选门槛不另加融合次数条件；这些配置不构成融合已生效的证据。']
        text += ['', 'indoor_forward_7 按用户要求排除出后续增益测试、代表筛选与收益汇总；其已完成的原始结果及最初六代表筛选证据保留，不追溯伪装成未运行。', '[每序列筛选原因](screening.json) · [全部配置、覆盖、初始化和融合诊断](results.json) · [运行 manifest](run_manifest.json)',
                 '[位置证据及独立复算](evaluation_archive.json) · [归档复算审计](archive_audit.json) · [独立完成审计](independent_audit.json) · [边界测试](unit_validation.json) · [烟测与串行/并发一致性](smoke.json)',
                 '构建准备两次环境入口失败已保留（退出 1/126），修正后独立构建成功；失败产物未用于回放。全量回放与构建尝试分别记录。',
                 f'原始输出 `{self.coord}`；并发耗时仅描述，不作性能验收。', '']
        notes = tuned / 'report_notes.md'
        if notes.exists(): text += ['', notes.read_text().rstrip(), '']
        (tuned / 'report.md').write_text('\n'.join(text))
        for dest in (uzdir, tuned):
            share_calibration_evidence(dest / 'configs')
            write(dest / 'sha256.json', {str(p.relative_to(dest)): sha(p) for p in sorted(dest.rglob('*')) if p.is_file() and p.name != 'sha256.json'})

    def matrix(self, sequences, alpha, title, percent=None):
        text = ['', '## ' + title, '', '| 序列 | ' + ' | '.join(MODES) + ' |', '|---|' + '---:|' * 6]
        for seq in sequences:
            values = []
            for mode in MODES:
                row = self.lookup(seq, mode, 1 if mode == 'OFF' else alpha)
                value = row.get('ate_rmse_m') if row else None
                cell = row.get('status', 'PENDING') if row else 'PENDING'
                if value is not None:
                    if percent:
                        base = self.lookup(seq, 'OFF' if percent == 'OFF' else mode)
                        baseline = base.get('ate_rmse_m') if base else None
                        if baseline is None or baseline <= 0: cell = '无法比较'
                        else:
                            delta = 100 * (value / baseline - 1)
                            cell = f'{delta:+.6f}%'
                            if delta < 0: cell = '**' + cell + '**'
                    else: cell = f'{value:.9f}'
                values.append(cell)
            text.append('| ' + seq + ' | ' + ' | '.join(values) + ' |')
        return text


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('action', choices=['prepare', 'run', 'publish'])
    parser.add_argument('coord', type=Path)
    parser.add_argument('--euroc', type=Path, default=Path('/home/he/datasets/euroc/ASL'))
    parser.add_argument('--uzh', type=Path, default=Path('/home/he/datasets/uzhfpv/archives'))
    args = parser.parse_args()
    if args.action == 'prepare': prepare(args)
    elif args.action == 'run': Study(args.coord).execute()
    else: Study(args.coord).publish()

if __name__ == '__main__':
    main()
