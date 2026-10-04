"""Bridge frozen historical far-scene midpoint measurements to timestamped Adapter input.

Original archives are copied verbatim. This is a new discretization identity,
not reproduction of the historical direct interval-midpoint propagation.
"""
import argparse
import hashlib
import json
import shutil
from pathlib import Path
import numpy as np

PROFILES = {'pressure_REGULAR_101_05': 'REGULAR_101_0.5s', 'historical_FAST_101_1': 'FAST_101_1s'}
HISTORICAL_ROOT = Path('/home/he/output/ltv_feature_readiness_passive/synthetic_inputs')


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def bridge(source, out):
    source, out = Path(source), Path(out)
    if out.exists():
        raise FileExistsError(out)
    identity = json.loads((source / 'identity.json').read_text())
    for name, key in [('inputs.npz', 'input_sha'), ('labels.npz', 'labels_sha')]:
        if sha(source / name) != identity[key]:
            raise ValueError('Historical archive SHA mismatch: ' + name)
    with np.load(source / 'inputs.npz', allow_pickle=False) as archive:
        old = {key: archive[key] for key in archive.files}
    with np.load(source / 'labels.npz', allow_pickle=False) as archive:
        truth = {key: archive[key] for key in archive.files}
    if identity['imu_hz'] != 200 or identity['camera_hz'] != 20:
        raise ValueError('Unrecognized historical sampling contract')
    times = old['imu_times']; camera_times = old['camera_times']; ids = old['feature_ids']
    if len(times) < 2 or not np.array_equal(times, np.arange(len(times)) / 200):
        raise ValueError('Historical integration boundary grid mismatch')
    if not np.array_equal(camera_times, np.arange(len(camera_times)) / 20) or camera_times[-1] != times[-1]:
        raise ValueError('Historical camera grid mismatch')
    if old['imu'].shape != (len(times)-1, 6) or ids.ndim != 2 or len(ids) != len(camera_times):
        raise ValueError('Historical dimensions mismatch')
    if not np.array_equal(truth['times'], times) or not np.array_equal(truth['camera_times'], camera_times):
        raise ValueError('Historical truth timeline mismatch')
    if not np.array_equal(truth['feature_ids'], ids):
        raise ValueError('Historical truth identity mismatch')
    n = ids.shape[1]
    if old['bearings_left'].shape != (len(camera_times), n, 3) or old['bearings_right'].shape != old['bearings_left'].shape:
        raise ValueError('Historical bearing dimensions mismatch')
    camera_ns = np.rint(camera_times * 1e9).astype(np.int64)
    # Keep every original measurement at its actual generation time, unchanged.
    midpoints = (np.arange(len(old['imu'])) + .5) / 200
    sample_times = np.r_[-.0025, midpoints, times[-1] + .0025]
    samples = np.vstack([old['imu'][0], old['imu'], old['imu'][-1]])
    csr_ids = np.repeat(ids, 2, axis=1).reshape(-1)
    bearings = np.stack([old['bearings_left'], old['bearings_right']], axis=2).reshape(-1, 3)
    payload = dict(old)
    payload.update(schema_version=np.array(2), historical_feature_ids=ids, feature_ids=csr_ids,
                   imu_sample_times=sample_times, imu_samples=samples, camera_ns=camera_ns,
                   observation_offsets=np.arange(len(camera_times)+1, dtype=np.int64)*n*2,
                   camera_ids=np.tile([0, 1], len(camera_times)*n),
                   actual_ns=np.repeat(camera_ns, n*2), bearings=bearings)
    point_map = {}; births = []; stereo = []; temporal = []; consecutive = {}; first_time = {}
    previous_ids = set()
    for k, row in enumerate(ids):
        visible = set(map(int, row))
        if len(visible) != n:
            raise ValueError('Duplicate historical camera ID')
        for slot, value in enumerate(row):
            fid = int(value); physical = int(truth['physical_indices'][k, slot])
            point = truth['points_W'][physical]
            if fid in point_map and not np.array_equal(point_map[fid], point):
                raise ValueError('Historical ID changed physical association')
            if fid not in point_map:
                point_map[fid] = point; births.append([k, fid])
            stereo.append([k, fid])
            if fid not in previous_ids:
                consecutive[fid] = 0; first_time[fid] = camera_times[k]
            consecutive[fid] += 1
            if consecutive[fid] >= 3 and camera_times[k] - first_time[fid] >= .1 - 1e-12:
                temporal.append([k, fid])
        previous_ids = visible
    # Evaluator-only mapping; no truth or historical lifecycle labels in payload.
    labels = dict(truth)
    labels.update(velocity_gravity_body=truth['state_body'][:, -6:], point_ids=np.array(list(point_map), np.int64),
                  points_W=np.array(list(point_map.values())), historical_points_W=truth['points_W'],
                  observation_physical_ids=np.repeat(truth['physical_indices'], 2, axis=1).reshape(-1),
                  wrong_match=np.zeros(len(csr_ids), bool), wrong_time=np.zeros(len(csr_ids), bool),
                  phase=np.zeros(len(camera_times), np.int64), stereo_opportunities=np.asarray(stereo, np.int64).reshape(-1, 2),
                  temporal_opportunities=np.asarray(temporal, np.int64).reshape(-1, 2), births=np.asarray(births, np.int64).reshape(-1, 2),
                  visible_left_count=np.full(len(camera_times), n), visible_stereo_count=np.full(len(camera_times), n))
    out.mkdir(parents=True)
    for name in ['inputs.npz', 'labels.npz', 'identity.json']:
        shutil.copyfile(source / name, out / ('historical_' + name))
    np.savez_compressed(out / 'inputs.npz', **payload)
    np.savez_compressed(out / 'labels.npz', **labels)
    pressure = identity['scene'] == 'REGULAR' and identity['lifetime'] == .5
    result = dict(identity)
    result.update(schema_version=2, historical_input_sha=identity['input_sha'], historical_labels_sha=identity['labels_sha'],
                  historical_identity_sha=sha(source/'identity.json'), historical_source=str(source.resolve()),
                  input_sha=sha(out/'inputs.npz'), labels_sha=sha(out/'labels.npz'), bridge_sha=sha(__file__),
                  geometry_name='HISTORICAL_FAR_ORIGINAL_UNCHANGED', condition='PRESSURE' if pressure else 'HISTORICAL_FAST',
                  negative_control=pressure, unseen_confirmation=False,
                  original_feature_ids_mapping='historical_feature_ids[k,j] == feature_ids[offset[k]+2*j:offset[k]+2*j+2]; cam0/cam1 alternating',
                  original_bearing_mapping='bearings[offset[k]+2*j+c] equals original bearings_left/right[k,j] bit-for-bit',
                  adapter_imu_contract='Original midpoint sample values at (j+.5)/200 seconds; two declared constant boundary guards; Adapter interpolates timestamped samples',
                  boundary_augmentation={'sample_times': [float(sample_times[0]), float(sample_times[-1])],
                                         'values': 'copies of first/last original IMU row; no truth-derived replacement/no new noise'},
                  numerical_identity='NEW: linear sample interpolation and Adapter subinterval propagation; not historical direct interval-midpoint reproduction',
                  opportunity_contract={'stereo': 'all original legal synchronous two-view IDs before risk/history/capacity',
                                        'temporal': 'same ID cam0 >=3 consecutive frames spanning .10s before risk/capacity'},
                  labels_mapping='Original truth archive copied byte-for-byte; mapped truth v/eta are final six state columns; physical point by saved physical_indices',
                  recovery_supply='Not defined for historical input; no recovery claim',
                  estimator_payload='inputs.npz only; historical labels and identity are provenance/evaluator only')
    (out/'identity.json').write_text(json.dumps(result, indent=2)+'\n')
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    origin = parser.add_mutually_exclusive_group(required=True)
    origin.add_argument('--source')
    origin.add_argument('--profile', choices=list(PROFILES))
    parser.add_argument('--out', required=True)
    args = parser.parse_args()
    source = args.source if args.source else HISTORICAL_ROOT / PROFILES[args.profile]
    print(json.dumps(bridge(source, args.out), indent=2))
