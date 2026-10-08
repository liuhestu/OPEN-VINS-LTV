"""Budget-controlled OFFLINE geometry calibration; never imported by estimator.

This evaluates reconstructed historical GEOM_NOISY inputs. Labels are loaded only
AFTER deterministic selection. It does not replay an observer or edit old outputs.
The parent scheduler must reserve 8160 geometry solves for 40 windows x 2 models x
(100 draws + one mean + one Jacobian validation mean) before invoking this script.
"""
import os
for name in ['OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS']:
    os.environ[name] = '1'
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import numpy as np
ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(ROOT / 'docs/experiments/tools/ltv_standalone_study'))
from truth_model import geometry, truth, rot, ids_at, skew


def serialized_case(poses, Rbc, pc, bearings, covariance=None):
    n = len(poses)
    header = [n, 1, n, n-1, n-1, int(covariance is not None)]
    values = list(header)
    for t, R, p in poses:
        values.extend([t, *R.ravel(), *p])
    values.extend([*Rbc.ravel(), *pc])
    for i, b in enumerate(bearings):
        values.extend([i, 0, *b])
    if covariance is not None:
        values.extend(covariance.ravel())
    return ' '.join(format(float(x), '.17g') for x in values) + '\n'


def noisy_poses(times, scene, Rbc, pc, noise):
    poses = []
    for t, eps in zip(times, noise):
        R, p, *_ = truth(float(t), scene)
        c = p + R @ pc + eps[:3]
        R = R @ Rbc @ rot(eps[3:]) @ Rbc.T
        poses.append((t, R, c - R @ pc))
    return poses


def covariance_input(poses, Rbc, pc, correlated):
    n = len(poses)
    std = np.array([.01]*3 + [np.deg2rad(.1)]*3)
    jitter = np.array([.002]*3 + [np.deg2rad(.02)]*3)
    transform = np.zeros((6*n, 6*n))
    raw = np.zeros_like(transform)
    for j, (tj, Rj, _) in enumerate(poses):
        # Raw variables = world camera-center perturbation, right camera rotation.
        transform[6*j:6*j+3, 6*j+3:6*j+6] = Rbc
        transform[6*j+3:6*j+6, 6*j:6*j+3] = np.eye(3)
        transform[6*j+3:6*j+6, 6*j+3:6*j+6] = Rj @ skew(pc) @ Rbc
        for k, (tk, _, _) in enumerate(poses):
            scale = np.exp(-abs(tj-tk)/.5) if correlated else float(j == k)
            raw[6*j:6*j+6, 6*k:6*k+6] = np.diag(std**2 * scale + (jitter**2 if correlated and j == k else 0))
    cov = np.zeros((8*n+6, 8*n+6))
    cov[:6*n, :6*n] = transform @ raw @ transform.T
    return cov


def summarize(rows):
    result = {}
    for scene in ['REGULAR', 'FAST']:
        selected = [r for r in rows if r['scene'] == scene and r['model'] == 'historical_independent']
        error = np.array([r['old_parallel_error_m'] for r in selected])
        sigma = np.array([r['predicted_sigma_m'] for r in selected])
        relative = np.array([r['old_relative_error'] for r in selected])
        risks = np.array([r['relative_risk'] for r in selected])
        ranks = lambda a: np.argsort(np.argsort(a))
        accepted = risks <= .05
        result[scene] = {'windows': len(selected), 'bias_m': float(error.mean()),
            'RMSE_m': float(np.sqrt(np.mean(error**2))),
            'nominal_95_interval_coverage_UNCALIBRATED': float(np.mean(abs(error) <= 1.96*sigma)),
            'median_interval_width_m': float(np.median(3.92*sigma)),
            'risk_error_rank_correlation': float(np.corrcoef(ranks(risks), ranks(relative))[0, 1]),
            'diagnostic_risk_005_acceptance': float(accepted.mean()),
            'diagnostic_risk_005_reliability': float(np.mean(relative[accepted] <= .1)) if accepted.any() else None,
            'warning': '20 stratified accepted historical windows; not population acceptance or final confirmation'}
    return result


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--legacy', type=Path, default=Path('/home/he/output/ltv_standalone_convergence_v2'))
    p.add_argument('--binary', type=Path, required=True)
    p.add_argument('--out', type=Path, required=True)
    p.add_argument('--prepare-only', action='store_true')
    args = p.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    if (args.out/'cases.txt').exists() or (args.out/'calibration.json').exists():
        raise RuntimeError('Refuse to overwrite existing calibration attempt')
    ledger = json.loads((args.legacy/'ledger.json').read_text())
    windows = []
    sources = []
    for scene in ['REGULAR', 'FAST']:
        entries = [e for e in ledger if e['config']['scene'] == scene and e['config']['initialization'] == 'GEOM_NOISY']
        if len(entries) != 1:
            raise RuntimeError('Ambiguous legacy identity: '+scene)
        e = entries[0]; rd = Path(e['directory'])
        events = json.loads((rd/'seeds.json').read_text())
        # Projection intentionally excludes all GT/error labels during selection.
        candidates = [{'id': int(r['id']), 't': float(r['t']), 'angle': float(r['angle'])} for r in events]
        metrics = json.loads((rd/'metrics.json').read_text())
        inputs = np.load(metrics['input_path'])
        points, Rbc, pc = geometry(scene)
        for block, target in enumerate(range(5, 51, 5)):
            nearby = [r for r in candidates if abs(r['t']-target) <= .5]
            if len(nearby) < 2:
                raise RuntimeError('Fixed time stratum insufficient')
            nearby.sort(key=lambda r: (r['angle'], r['t'], r['id']))
            for quantile in [.25, .75]:
                chosen = nearby[round(quantile*(len(nearby)-1))]
                end = round(chosen['t']*20)
                ticks = np.array([k for k in range(max(0, end-20), end+1)
                                  if chosen['id'] in ids_at(k, 20, e['config']['lifetime'], len(points))])
                if len(ticks) < 3:
                    raise RuntimeError('Historical accepted window missing')
                windows.append(dict(scene=scene, block=block, quantile=quantile, id=chosen['id'],
                    t=chosen['t'], ticks=ticks, bearings=inputs['bearings'][ticks, chosen['id'] % len(points)],
                    priors=inputs['priors'][ticks], Rbc=Rbc, pc=pc, directory=rd))
        sources.append({'id':e['id'], 'identity':e['identity'], 'input_sha':metrics['input_sha'],
                        'seed_log_sha':hashlib.sha256((rd/'seeds.json').read_bytes()).hexdigest()})
    if len(windows) != 40:
        raise RuntimeError('Expected exactly 40 windows')
    # Freeze selection before generating/evaluating any error label.
    selection = [{k:v for k,v in w.items() if k in ['scene','block','quantile','id','t']} for w in windows]
    selection_text = json.dumps({'sources':sources,'windows':selection}, indent=2)
    selection_path = args.out/'window_selection.json'
    if selection_path.exists() and selection_path.read_text() != selection_text:
        raise RuntimeError('Frozen selection mismatch')
    if not selection_path.exists(): selection_path.write_text(selection_text)
    if args.prepare_only:
        print(hashlib.sha256(selection_path.read_bytes()).hexdigest())
        return
    rows = []; case_records = []; count = 0
    input_path = args.out/'cases.txt'
    with input_path.open('w') as stream:
        for scene_no, scene in enumerate(['REGULAR', 'FAST']):
            for block in range(10):
                group = [w for w in windows if w['scene']==scene and w['block']==block]
                first = min(int(w['ticks'][0]) for w in group); last = max(int(w['ticks'][-1]) for w in group)
                size = last-first+1
                for model_no, model in enumerate(['historical_independent','correlated_stress']):
                    rng = np.random.default_rng(70000 + scene_no*1000 + block*10 + model_no)
                    std = np.array([.01]*3 + [np.deg2rad(.1)]*3)
                    noise = rng.normal(size=(100,size,6))*std
                    if model_no:
                        a = np.exp(-.05/.5)
                        for j in range(1,size): noise[:,j] = a*noise[:,j-1]+np.sqrt(1-a*a)*noise[:,j]
                        noise += rng.normal(size=noise.shape)*np.array([.002]*3+[np.deg2rad(.02)]*3)
                    for w in group:
                        times = w['ticks']/20
                        poses = noisy_poses(times,scene,w['Rbc'],w['pc'],w['priors'])
                        covariance = covariance_input(poses,w['Rbc'],w['pc'],bool(model_no))
                        stream.write(serialized_case(poses,w['Rbc'],w['pc'],w['bearings'],covariance)); count += 2
                        case_records.append((w,model,'nominal',None))
                        for draw in range(100):
                            perturbed = noisy_poses(times,scene,w['Rbc'],w['pc'],noise[draw,w['ticks']-first])
                            stream.write(serialized_case(perturbed,w['Rbc'],w['pc'],w['bearings'])); count += 1
                            case_records.append((w,model,'draw',draw))
    if count != 8160: raise RuntimeError('Solve accounting mismatch')
    output_path = args.out/'estimates.jsonl'
    subprocess.run([str(args.binary),str(input_path),str(output_path)],check=True)
    results = [json.loads(line) for line in output_path.read_text().splitlines()]
    if len(results) != len(case_records): raise RuntimeError('Missing batch outputs')
    snapshots = {}
    for index in range(0,len(case_records),101):
        w,model,_,_ = case_records[index]; nominal = results[index]
        if not nominal['valid'] or not nominal['risk_valid']: raise RuntimeError('Legacy accepted input invalid in new module')
        scene=w['scene']; points,_,_=geometry(scene); R,p,*_=truth(w['t'],scene)
        gt=R.T@(points[w['id']%len(points)]-p); ray=(gt-w['pc']);ray/=np.linalg.norm(ray)
        if w['directory'] not in snapshots:
            snapshots[w['directory']]=np.load(w['directory']/'initialization_snapshots_reconstructed.npz')
        snap=snapshots[w['directory']]; ti=np.flatnonzero(abs(snap['t']-w['t'])<1e-10)
        if len(ti)!=1: raise RuntimeError('Historical hook snapshot missing')
        slot=np.flatnonzero(snap['ids'][ti[0]]==w['id'])
        if len(slot)!=1: raise RuntimeError('Historical ID missing')
        old=snap['x_after'][ti[0],3*slot[0]:3*slot[0]+3]
        if np.linalg.norm(old-np.array(nominal['landmark_B']))>1e-8:
            raise RuntimeError('Reconstructed mean differs from saved old hook')
        valid=[r for r in results[index+1:index+101] if r['valid']]
        errors=np.array([r['landmark_B'] for r in valid])-gt
        parallel=errors@ray; olderr=old-gt
        row={'scene':scene,'id':w['id'],'t':w['t'],'model':model,'nposes':len(w['ticks']),
             'old_parallel_error_m':float(olderr@ray),'old_relative_error':float(np.linalg.norm(olderr)/np.linalg.norm(gt)),
             'predicted_sigma_m':nominal['sigma_parallel'],'relative_risk':nominal['relative_parallel_risk'],
             'MC_valid':len(valid),'MC_draws':100,'MC_parallel_bias_m':float(parallel.mean()),
             'MC_parallel_std_m':float(parallel.std(ddof=1)),
             'MC_parallel_RMSE_m':float(np.sqrt(np.mean(parallel**2))),
             'MC_nominal95_coverage_UNCALIBRATED':float(np.mean(abs(parallel)<=1.96*nominal['sigma_parallel'])),
             'MC_noise_shared_with_same_block_features':True,
             'note':'old label belongs to independent model only; correlated result is sensitivity, not historical fit'}
        rows.append(row)
    report={'status':'CALIBRATION_DIAGNOSTIC','geometry_solves_reserved':8160,'windows':40,'draws_per_model_window':100,
            'models':['historical_independent','correlated_stress'],'source':'offline reconstruction, not recorded covariance',
            'rows':rows,'summary':summarize(rows),
            'binary_sha':hashlib.sha256(args.binary.read_bytes()).hexdigest()}
    (args.out/'calibration.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(report['summary'],indent=2))

if __name__ == '__main__': main()
