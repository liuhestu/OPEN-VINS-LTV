"""Auxiliary past-bearing geometry and concrete covariance-feasibility evidence."""
import argparse
from collections import Counter
import csv
import json
from pathlib import Path
import numpy as np
from analyze import paired_group, labels, match_times, angular_stats
from run import sha, write


def covariance_evidence(old, options):
    covariances=[]
    missing_seed_logs=0
    for line in (old/'features.jsonl').open():
        f=json.loads(line)
        missing_seed_logs += 'seeds' not in f
        for seed in f.get('seeds',[]):
            if seed['admitted'] and seed['mean_written']:
                covariances.append(np.array(seed['covariance_B']).reshape(3,3))
    traces=np.array([np.trace(p) for p in covariances])
    eig=[np.linalg.eigvalsh((p+p.T)/2) for p in covariances]
    scale=options['ltv_observer_initial_p_landmark']
    trace_stats=angular_stats(traces)
    trace_stats['above_0_02_m2']=trace_stats.pop('above_0_02_rad')
    return dict(missing_seed_logs=missing_seed_logs,admitted_written_seeds=len(covariances), observer_birth_diagonal=scale,
                observer_birth_trace=3*scale,seed_trace=trace_stats, seed_trace_units="m^2",
                seed_trace_equal_observer_birth_trace=int(np.isclose(traces,3*scale,rtol=1e-12,atol=1e-12).sum()),
                minimum_seed_eigenvalue=float(np.min(eig)) if eig else None, maximum_seed_eigenvalue=float(np.max(eig)) if eig else None)


def assess(out):
    freeze=json.loads((out/'freeze.json').read_text())
    initial=json.loads((out/'summary.json').read_text())
    result={'stage':'auxiliary audit; initial primary result retained','sequences':{},'current_bearing_used_in_fit':False}
    for seq,inp in freeze['inputs'].items():
        old=Path(inp['off_path'])
        features=[json.loads(x) for x in (old/'features.jsonl').read_text().splitlines()]
        phase=labels(features)
        with (out/'past_geometry_audit'/f'{seq}_predictions.csv').open() as f:
            rows=list(csv.DictReader(f))
        # The supplementary fit must leave every original output value unchanged.
        with (out/f'{seq}_predictions.csv').open() as f:
            original=csv.DictReader(f)
            count=0
            for count,(a,b) in enumerate(zip(original,rows),1):
                assert all(a[k]==b[k] for k in a), (seq,count,'primary data changed')
            assert count==len(rows)==initial['sequences'][seq]['rows']
        raw=sorted({int(r['camera_ns']) for r in rows})
        mapping=dict(zip(raw,map(int,match_times(raw,sorted(phase)))))
        source_counts=[]
        for r in rows:
            r['phase']=phase[mapping[int(r['camera_ns'])]]
            latest=float(r['past_fit_latest_time'])
            if np.isfinite(latest):assert latest<float(r['imu_time'])-1e-9
            source_counts.append(int(r['past_fit_observations']))
            # Reuse the primary denominator/paired evaluator, retaining invalid fit samples.
            r['main_valid']=r['past_fit_valid']
            r['main_angle_rad']=r['past_fit_angle_rad']
        mature=np.array([r['past_mature']=='1' for r in rows])
        phases=np.array([r['phase'] for r in rows])
        groups={'primary_mature':mature}
        for name in ['normal','recovery','startup']:
            groups[name]=mature & (phases==name)
        evaluated={name:paired_group(rows,mask) for name,mask in groups.items()}
        result['sequences'][seq]=dict(groups=evaluated,primary_fields_exact=True,
            original_prediction_rows=len(rows),past_source_count=dict(minimum=min(source_counts),maximum=max(source_counts),
            mean=float(np.mean(source_counts))),
            covariance_evidence=covariance_evidence(old,json.loads((old/'effective_options.json').read_text())),
            exact=json.loads((out/'past_geometry_audit'/f'{seq}_exact.json').read_text()),
            predictions_sha=sha(out/'past_geometry_audit'/f'{seq}_predictions.csv'))
    # This scalar counterexample distinguishes the Riccati gain metric from a
    # covariance under a fixed physical discrete observation-noise model.
    p=2.;alpha=.05;measurement_noise=.01
    gain=alpha*p
    metric=p-alpha*p*p
    true_cov=(1-gain)**2*p+gain**2*measurement_noise
    assert not np.isclose(metric,true_cov)
    result['riccati_covariance_witness']=dict(prior=p,alpha=alpha,physical_noise=measurement_noise,
        mean_gain=gain,riccati_metric_after=metric,error_covariance_after=true_cov,
        conclusion='Riccati P is not automatically the covariance of this discrete estimator under fixed physical noise')
    result['missing_contracts']=['cross-time observer/anchor error covariance',
        'cross-point physical error covariance', 'observer-main covariance including shared IMU and main bias supply',
        'observer-visual covariance and per-observation main-consumption provenance',
        'calibrated observer error covariance distinct from its Riccati gain metric']
    result['source_evidence']={str(p):sha(p) for p in [Path('ov_msckf/src/ltv/observer/ltv_observer.cpp'),Path('ov_msckf/src/ltv/observer/ltv_controlled_features.h'),Path('ov_msckf/src/ltv/landmark_adapter/LtvSeedUncertainty.cpp'),Path('ov_msckf/src/ltv/fusion/UpdaterLTV.cpp')]}
    result['model_feasibility']=dict(calibrated_error_covariance_available=False,known_main_visual_cross_blocks_available=False,proven_conservative_bound_available=False,observer_algorithm_change_allowed=False,independence_fallback_allowed=False,required_new_error_maps=['seed/history and calibration dependence','shared IMU/main bias supply','all camera Euler substeps and bearing-dependent H','main visual update and quaternion reset','anchor covariance/lifecycle','shared observation-noise history'])
    result['decision']='STOP_FUSION_CORRELATION_MODEL_UNAVAILABLE'
    result['reason']='Positive shared-anchor prediction evidence is retained. No calibrated joint error model/bound exists; implementing it requires seed augmentation, shared-IMU/main-visual error maps, protected-anchor error storage and retained observation-noise history. This exceeds a minimal fusion change; no unknown cross blocks are set to zero and no independence fallback is run.'
    write(out/'correlation_audit.json',result)
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('out',type=Path)
    args=p.parse_args()
    print(assess(args.out.resolve())['decision'])
