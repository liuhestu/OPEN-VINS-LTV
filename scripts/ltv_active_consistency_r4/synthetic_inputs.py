"""New causal synthetic input adapter; no observer implementation or execution.

Only inputs.npz is estimator input. labels.npz contains evaluator-only identity,
geometry, opportunity and intervention information. Run via the new scheduler.
Historical REGULAR/FAST stress inputs are reused by path+SHA, never regenerated.
"""
import os
for key in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ[key] = '1'
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[2]
OLD = ROOT / 'scripts/ltv_feature_passive/synthetic/generate.py'
spec = importlib.util.spec_from_file_location('previous_synthetic_math', OLD)
math = importlib.util.module_from_spec(spec)
spec.loader.exec_module(math)
CONDITIONS = ('STEREO', 'TEMPORAL', 'STEREO_DROPOUT', 'RECOVERY', 'WRONG_MATCH', 'TIME_ERROR')
G = np.array([0., 0., -9.81])

# Frozen input-only recovery timing qualification, not a success probability.
SUPPLY = {'min_points':15, 'min_history_frames':3, 'min_history_seconds':.10,
          'min_ray_m':.1, 'min_parallax_rad':float(np.deg2rad(.5)),
          'max_condition':1e8, 'max_relative_parallel_risk':.05, 'bearing_sigma_rad':float(np.deg2rad(.05))}


def stereo_supply_qualified(point_body, camera_positions, legal_pair, history_frames, history_seconds):
    if not legal_pair or history_frames < SUPPLY['min_history_frames'] or history_seconds+1e-12 < SUPPLY['min_history_seconds']:
        return False, None
    displacement=np.asarray(point_body)-np.asarray(camera_positions)
    ranges=np.linalg.norm(displacement,axis=1)
    if ranges.shape!=(2,) or not np.isfinite(ranges).all() or np.min(ranges)<SUPPLY['min_ray_m']:
        return False,None
    rays=displacement/ranges[:,None]
    parallax=np.arccos(np.clip(rays[0]@rays[1],-1,1))
    if parallax<SUPPLY['min_parallax_rad']:return False,None
    projectors=np.array([np.eye(3)-np.outer(ray,ray) for ray in rays])
    A=projectors.sum(axis=0);eigen=np.linalg.eigvalsh(A)
    if eigen[0]<=0 or eigen[-1]/eigen[0]>SUPPLY['max_condition']:return False,None
    inverse=np.linalg.inv(A)
    covariance=inverse@(SUPPLY['bearing_sigma_rad']**2*np.sum(ranges[:,None,None]**2*projectors,axis=0))@inverse.T
    risk=float(np.sqrt(max(0.,rays[0]@covariance@rays[0]))/ranges[0])
    return risk<=SUPPLY['max_relative_parallel_risk'],risk


def sufficient_supply(count):
    return count>=SUPPLY['min_points']


def heavy_tail_negative(bearings,rng):
    """Predeclared negative input only; untouched rays retain exact bit values."""
    result=bearings.copy()
    mask=rng.random(bearings.shape[:-1])<.02
    noise=rng.standard_t(3,size=bearings.shape)*np.deg2rad(.5)
    tangent=noise-np.sum(noise*bearings,axis=-1,keepdims=True)*bearings
    changed=bearings[mask]+tangent[mask]
    result[mask]=changed/np.linalg.norm(changed,axis=-1,keepdims=True)
    return result,mask


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def calibration(path):
    # OpenCV's %YAML:1.0 directive is not standard YAML 1.1 syntax.
    text = Path(path).read_text()
    data = yaml.safe_load('\n'.join(line for line in text.splitlines() if not line.startswith('%YAML:')))
    cameras = [data['cam0'], data['cam1']]
    for camera in cameras:
        if camera['camera_model'] != 'pinhole' or camera['distortion_model'] != 'radtan':
            raise ValueError('This adapter requires the actual pinhole/radtan EuRoC calibration')
        matrix = np.asarray(camera['T_imu_cam'], dtype=float)
        if matrix.shape != (4, 4) or not np.allclose(matrix[3], [0, 0, 0, 1]):
            raise ValueError('Invalid camera-to-body transform')
        if not np.allclose(matrix[:3, :3].T @ matrix[:3, :3], np.eye(3), atol=1e-8):
            raise ValueError('Nonorthogonal extrinsic rotation')
    return cameras, np.array([c['T_imu_cam'] for c in cameras])


def visible(x, camera):
    if not np.isfinite(x).all() or x[2] <= .1:
        return False
    a, b = x[:2] / x[2]
    k1, k2, p1, p2 = camera['distortion_coeffs']
    radius = a*a + b*b
    radial = 1 + k1*radius + k2*radius*radius
    distorted = [a*radial + 2*p1*a*b + p2*(radius+2*a*a),
                 b*radial + p1*(radius+2*b*b) + 2*p2*a*b]
    fx, fy, cx, cy = camera['intrinsics']
    u, v = fx*distorted[0]+cx, fy*distorted[1]+cy
    width, height = camera['resolution']
    return 0 <= u < width and 0 <= v < height


def generate(out, calibration_path, scene, seed, lifetime, condition, duration=60., confirmation_sha=None):
    if scene not in ('REGULAR', 'FAST') or condition not in CONDITIONS or lifetime not in (.5, 1.):
        raise ValueError('Unregistered scene/condition/lifetime')
    if seed not in (42, 43, 201, 202, 301, 302) or not 0 < duration <= 60:
        raise ValueError('Unregistered seed or duration')
    if seed >= 200:
        frozen = ROOT / 'docs/ltv/active_landmark_consistency_r4/frozen_config.json'
        if not frozen.exists() or confirmation_sha != sha(frozen):
            raise ValueError('Confirmation input requires the current new-task freeze SHA')
        if seed not in json.loads(frozen.read_text()).get('confirmation_seeds',[]):
            raise ValueError('Confirmation seed is outside the current frozen group')
    out = Path(out)
    if out.exists():
        raise FileExistsError(out)
    cameras, transforms = calibration(calibration_path)
    Rbc, pc = transforms[:, :3, :3], transforms[:, :3, 3]
    streams = np.random.SeedSequence(seed).spawn(5)
    imu_rng, left_rng, right_rng, pose_rng, jitter_rng = [np.random.default_rng(s) for s in streams]
    outlier_rng=np.random.default_rng(np.random.SeedSequence([seed,57031])) if condition=='WRONG_MATCH' else None
    imu_times = np.arange(round(duration*200)+1) / 200
    times = np.arange(round(duration*20)+1) / 20
    ns = np.arange(len(times), dtype=np.int64)*50000000
    imu = np.array([np.r_[math.sample((k-.5)/200, scene)[1], math.sample((k-.5)/200, scene)[2]]
                    for k in range(1, len(imu_times))])
    imu[:, :3] += imu_rng.normal(0, .02, imu[:, :3].shape)
    imu[:, 3:] += imu_rng.normal(0, .002, imu[:, 3:].shape)
    endpoint_rng = np.random.default_rng(np.random.SeedSequence([seed, 200, 2]))
    endpoint_imu = np.array([np.r_[math.sample(t, scene)[1], math.sample(t, scene)[2]] for t in imu_times])
    endpoint_imu[:, :3] += endpoint_rng.normal(0, .02, endpoint_imu[:, :3].shape)
    endpoint_imu[:, 3:] += endpoint_rng.normal(0, .002, endpoint_imu[:, 3:].shape)
    std = np.array([.01]*3 + [np.deg2rad(.1)]*3)
    jitter = np.array([.002]*3 + [np.deg2rad(.02)]*3)
    eps = pose_rng.normal(size=(len(times), 6))*std
    correlation = np.exp(-.05/.5)
    for k in range(1, len(times)):
        eps[k] = correlation*eps[k-1] + np.sqrt(1-correlation**2)*eps[k]
    eps += jitter_rng.normal(size=eps.shape)*jitter
    prior_R, prior_p, true_R, true_p = [], [], [], []
    offsets, obs_ids, obs_cams, obs_ns, bearings = [0], [], [], [], []
    physical_ids, wrong_labels, timing_labels, outlier_labels = [], [], [], []
    stereo_opportunities, temporal_opportunities, birth_ids = [], [], []
    world, histories, first_visible = {}, {}, set()
    phase, visible_left_count, visible_stereo_count = [], [], []
    supply_points, supply_counts, supply_mask, supply_starts = [], [], [], []
    for tick, t in enumerate(times):
        R, p, *_ = math.truth(t, scene)
        true_R.append(R); true_p.append(p)
        center = p + R@pc[0] + eps[tick, :3]
        noisy_R = R@Rbc[0]@math.rot(eps[tick, 3:])@Rbc[0].T
        prior_R.append(noisy_R); prior_p.append(center-noisy_R@pc[0])
        ids = math.ids_at(tick, 20, lifetime, 30)
        for feature in ids:
            feature = int(feature)
            if feature not in world:
                slot = feature % 30
                depth = 2.0 + 2.0*(slot % 3)/2
                local = depth*np.array([.12*(slot % 6-2.5), .10*(slot//6-2), 1.])
                world[feature] = p+R@(pc[0]+Rbc[0]@local)
        ell = np.array([(world[int(i)]-p)@R for i in ids])
        camera_points = np.array([(ell-pc[c])@Rbc[c] for c in range(2)])
        unit = camera_points/np.linalg.norm(camera_points, axis=2, keepdims=True)
        noisy = np.array([math.tangent_noise(unit[0], left_rng, np.deg2rad(.05)),
                          math.tangent_noise(unit[1], right_rng, np.deg2rad(.05))])
        outlier_mask=np.zeros((2,len(ids)),dtype=bool)
        if outlier_rng is not None:
            noisy,outlier_mask=heavy_tail_negative(noisy,outlier_rng)
        masks = np.array([[visible(x, cameras[c]) for x in camera_points[c]] for c in range(2)])
        weak = condition == 'RECOVERY' and 15 <= t < 30
        right_missing = condition == 'TEMPORAL' or (condition == 'STEREO_DROPOUT' and 15 <= t < 30)
        if weak:
            masks[:] = False
        if right_missing:
            masks[1] = False
        staged = condition in ('RECOVERY', 'STEREO_DROPOUT')
        phase.append((1 if 15 <= t < 30 else (2 if t >= 30 else 0)) if staged else 0)
        current = set()
        stereo = set()
        qualified=set()
        for slot, feature in enumerate(ids):
            feature = int(feature)
            if masks[0, slot]:
                current.add(feature)
                if feature not in first_visible:
                    first_visible.add(feature); birth_ids.append([tick, feature])
                previous = histories.get(feature)
                count = previous[2]+1 if previous and previous[0] == tick-1 else 1
                start = previous[1] if count > 1 else tick
                histories[feature] = tick, start, count
                # Opportunity is causal input support, before any risk/solver/capacity.
                if count >= 3 and tick-start >= 2:
                    temporal_opportunities.append([tick, feature])
            for camera in range(2):
                if not masks[camera, slot]:
                    continue
                source_slot = (slot+1) % 30 if condition == 'WRONG_MATCH' and camera == 1 and slot % 10 == 0 else slot
                time_error = condition == 'TIME_ERROR' and camera == 1 and slot % 10 == 0
                obs_ids.append(feature); obs_cams.append(camera)
                obs_ns.append(int(ns[tick]) - (50000000 if time_error else 0))
                bearings.append(noisy[camera, source_slot])
                physical_ids.append(int(ids[source_slot]))
                wrong_labels.append(source_slot != slot); timing_labels.append(time_error)
                outlier_labels.append(bool(outlier_mask[camera,source_slot]))
                if camera == 1 and masks[0, slot] and not time_error:
                    stereo.add(feature)
                    stereo_opportunities.append([tick, feature])
                    previous=histories[feature]
                    good,risk=stereo_supply_qualified(ell[slot],pc,source_slot==slot,previous[2],(tick-previous[1])/20)
                    if good:
                        qualified.add(feature);supply_points.append([tick,feature])
        visible_left_count.append(len(current)); visible_stereo_count.append(len(stereo))
        supply_counts.append(len(qualified));enough=sufficient_supply(len(qualified))
        supply_starts.append((supply_starts[-1] if supply_mask and supply_mask[-1] else float(t)) if enough else -1.)
        supply_mask.append(enough)
        offsets.append(len(obs_ids))
    # No condition name, phase, lifespan, physical association or truth in this payload.
    inputs = dict(schema_version=np.array(2), imu_times=imu_times, imu=imu,
                  imu_sample_times=imu_times, imu_samples=endpoint_imu, camera_times=times, camera_ns=ns,
                  observation_offsets=np.array(offsets, np.int64), feature_ids=np.array(obs_ids, np.int64),
                  camera_ids=np.array(obs_cams, np.int32), actual_ns=np.array(obs_ns, np.int64),
                  bearings=np.array(bearings).reshape(-1, 3), prior_R_WB=np.array(prior_R), prior_p_WB=np.array(prior_p),
                  R_BC=Rbc, p_BC=pc, pose_model_std=std, pose_model_jitter=jitter,
                  pose_correlation_seconds=np.array(.5), bearing_sigma_rad=np.array(np.deg2rad(.05)))
    state = np.array([np.r_[math.truth(t, scene)[0].T@math.truth(t, scene)[2],
                           math.truth(t, scene)[0].T@G] for t in imu_times])
    labels = dict(times=imu_times, velocity_gravity_body=state, camera_times=times, R_WB=np.array(true_R), p_WB=np.array(true_p),
                  point_ids=np.array(list(world), np.int64), points_W=np.array(list(world.values())),
                  observation_physical_ids=np.array(physical_ids, np.int64), wrong_match=np.array(wrong_labels),
                  wrong_time=np.array(timing_labels), phase=np.array(phase), pose_noise_camera=eps,
                  stereo_opportunities=np.array(stereo_opportunities, np.int64).reshape(-1, 2),
                  temporal_opportunities=np.array(temporal_opportunities, np.int64).reshape(-1, 2),
                  births=np.array(birth_ids, np.int64).reshape(-1, 2), visible_left_count=np.array(visible_left_count),
                  visible_stereo_count=np.array(visible_stereo_count),
                  reliable_supply_feature_ids=np.array(supply_points,np.int64).reshape(-1,2),
                  reliable_supply_count=np.array(supply_counts),sufficient_reliable_supply=np.array(supply_mask),
                  sufficient_reliable_supply_run_start=np.array(supply_starts))
    if condition=='WRONG_MATCH':labels['heavy_tail_outlier']=np.array(outlier_labels,dtype=bool)
    out.mkdir(parents=True)
    np.savez_compressed(out/'inputs.npz', **inputs)
    np.savez_compressed(out/'labels.npz', **labels)
    identity = dict(scene=scene, geometry_name='EUROC_CALIBRATED_NEAR_2_4M', seed=seed, lifetime=lifetime, condition=condition, duration=duration,
                    schema_version=2, calibration=str(Path(calibration_path).resolve()), calibration_sha=sha(calibration_path),
                    generator_sha=sha(__file__), mathematical_functions_sha=sha(OLD),
                    truth_model_sha=sha(math.sample.__code__.co_filename), input_sha=sha(out/'inputs.npz'), labels_sha=sha(out/'labels.npz'),
                    confirmation_sha=confirmation_sha, negative_control=condition in ('WRONG_MATCH', 'TIME_ERROR'),
                    reliable_supply_contract={**SUPPLY,'source':'Input-only true stereo geometry and fixed noise model; same-ID synchronous lawful observations, causal history; not a probability guarantee, no candidate seed/output error/readiness used'},
                    opportunity_contract={'stereo':'current same-ID legal cam0+cam1, equal integer timestamp, before solver/risk/history/capacity',
                                          'temporal':'current cam0 with >=3 consecutive frames spanning >=0.10s; before solver/parallax/risk/capacity',
                                          'birth':'first actual cam0 visibility of ID; both sources also report all-birth denominator'},
                    estimator_payload='inputs.npz only; labels and identity condition/lifetime fields evaluator/dispatcher only',
                    adapter_imu_contract='imu_sample_times/imu_samples are 200Hz endpoint measurements, independent SeedSequence([seed,200,2]); legacy imu is midpoint and cannot be treated as endpoint or reused across numerical identities',
                    old_pressure_regression='Reuse historical input path and exact SHA; not regenerated by this adapter')
    if condition=='WRONG_MATCH':
        identity['negative_noise_profile']={'name':'bad_match_and_heavy_tail','retains_gaussian_sigma_deg':.05,
                                           'probability_per_camera_point':.02,'student_t_df':3,'tangent_scale_deg':.5,
                                           'independent_rng':'SeedSequence([seed,57031])',
                                           'label':'heavy_tail_outlier maps actual source_slot after wrong-ID association',
                                           'estimator_sigma_unchanged':True,'normal_condition_inputs_unchanged':True}
    (out/'identity.json').write_text(json.dumps(identity, indent=2))
    return identity


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', required=True)
    parser.add_argument('--calibration', required=True, dest='calibration_path')
    parser.add_argument('--scene', choices=['REGULAR', 'FAST'], required=True)
    parser.add_argument('--seed', type=int, required=True)
    parser.add_argument('--lifetime', type=float, required=True)
    parser.add_argument('--condition', choices=CONDITIONS, required=True)
    parser.add_argument('--duration', type=float, default=60.)
    parser.add_argument('--confirmation-sha')
    print(json.dumps(generate(**vars(parser.parse_args())), indent=2))
