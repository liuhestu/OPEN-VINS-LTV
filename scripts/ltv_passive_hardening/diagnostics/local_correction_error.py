"""Offline, same-event pre/post error decomposition; no policy or observer execution."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np


def digest(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def run(evaluation, prediction, features, receipts, out):
    out = Path(out)
    if out.exists():
        raise FileExistsError(out)
    with np.load(evaluation, allow_pickle=False) as a:
        curves = {k: a[k] for k in a.files}
    estimates = [json.loads(s) for s in Path(prediction).read_text().splitlines()]
    logs = [json.loads(s) for s in Path(features).read_text().splitlines()]
    for rows in (estimates, logs):
        if len({r['camera_ns'] for r in rows}) != len(rows):
            raise ValueError('Ambiguous integer camera identity')
    estimates = {r['camera_ns']: r for r in estimates}
    logs = {r['camera_ns']: r for r in logs}
    keys = curves['camera_ns'].tolist()
    if len(set(keys)) != len(keys):
        raise ValueError('Ambiguous evaluator identity')
    origin = float(curves['time'][0])
    result = {'events': [], 'neighborhood_half_width_s': .5,
              'scope': 'Reconstructed prior and actual posterior against the SAME current reference; no per-landmark causal attribution, policy execution or GT feedback.'}
    for ns in receipts:
        i = keys.index(ns)
        center = float(curves['time'][i])
        rows = []
        for k, time in enumerate(curves['time']):
            if not center-.5 <= time <= center+.5:
                continue
            key = keys[k]
            if key not in estimates:
                raise ValueError('Missing reconstruction in requested neighborhood')
            d, f = estimates[key], logs[key]
            if d['epoch'] != f['epoch'] or float(time) != f['imu_time']:
                raise ValueError('Epoch/physical-time identity mismatch')
            if not f['raw_current']:
                raise ValueError('No current state')
            row = {'camera_ns': key, 'time': float(time), 'relative_initialized_s': float(time-origin),
                   'ready_G': bool(f['ready_G']), 'ready_V': bool(f['ready_V']), 'quantities': {}}
            for name, pred_key, delta_key in [('v', 'predicted_v', 'delta_v'), ('eta', 'predicted_eta', 'delta_eta')]:
                actual = curves['P_NEW_'+name][k]
                predicted, delta = np.array(d[pred_key]), np.array(d[delta_key])
                np.testing.assert_allclose(predicted+delta, actual, rtol=0, atol=1e-12)
                reference = curves['reference_'+name][k]
                if not np.isfinite(reference).all():
                    row['quantities'][name] = {'reference_valid': False}
                    continue
                before, after = predicted-reference, actual-reference
                cross, square = float(2*before@delta), float(delta@delta)
                change = float(after@after-before@before)
                if abs(change-cross-square) > 1e-12:
                    raise ValueError('Squared-error decomposition failed')
                previous = float(curves['P_NEW_error_'+name][k-1]) if k else None
                q = {'reference_valid': True, 'previous_post_error_norm': previous,
                     'pre_error_norm': float(np.linalg.norm(before)), 'post_error_norm': float(np.linalg.norm(after)),
                     'OpenVINS_error_norm': float(curves['OpenVINS_error_'+name][k]),
                     'pre_error_vector': before.tolist(), 'post_error_vector': after.tolist(), 'delta': delta.tolist(),
                     'delta_norm': float(np.linalg.norm(delta)), 'squared_error_change': change,
                     'twice_pre_error_dot_delta': cross, 'squared_delta_norm': square,
                     'same_time_correction_error_norm_change': float(np.linalg.norm(after)-np.linalg.norm(before))}
                q['previous_to_prediction_error_norm_change'] = q['pre_error_norm']-previous if previous is not None else None
                row['quantities'][name] = q
            rows.append(row)
        selected = [r for r in rows if r['camera_ns'] == ns]
        if len(selected) != 1:
            raise ValueError('Target is not present exactly once')
        result['events'].append({'target_camera_ns': ns, 'target': selected[0], 'neighborhood': rows})
    result['identity'] = {str(p): digest(p) for p in (evaluation, prediction, features, __file__)}
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open('x') as f:
        json.dump(result, f, indent=2, allow_nan=False)
        f.write('\n')
    return result


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    for key in ('evaluation', 'prediction', 'features', 'out'):
        p.add_argument('--'+key, required=True)
    p.add_argument('--receipts', nargs='+', type=int, required=True)
    r = run(**vars(p.parse_args()))
    print(json.dumps([e['target'] for e in r['events']], indent=2))
