"""Recover management diagnostics only; never re-evaluate GT or raw outputs.

The frozen observer/evaluator are not edited. Recovery rows come from exact
full-cache replay. Original feature rows and evaluation evidence remain intact.
"""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import shutil
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import real_evaluate
FROZEN = Path(__file__).resolve().parents[5] / 'docs/ltv/feature_readiness_passive/frozen_config.json'

MANAGEMENT = ('retained_ids', 'opportunity_tracks', 'admitted_tracks',
              'retired_ids', 'seeds', 'tracks')
PRESERVED = ('epoch', 'sequence', 'imu_time', 'cursor', 'available', 'ready_G',
             'ready_V', 'mature_features', 'reason', 'state_features',
             'observed_features', 'healthy_updates', 'camera_substeps',
             'v_body', 'eta_body')


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read_rows(path):
    return [json.loads(line) for line in Path(path).read_text().splitlines() if line]


def lifecycle_counts(recovered):
    """Separate unadmitted expiry from removal of a once-admitted state slot."""
    ttl, removed, admitted = set(), set(), set()
    previous = {}
    for row in recovered:
        epoch = row['epoch']
        for seed in row['seeds']:
            if seed.get('admitted'):
                admitted.add((epoch, int(seed['id'])))
        current_ttl = set(row['never_admitted_candidate_ttl_ids'])
        current_removed = set(row['admitted_retired_ids'])
        old_ttl, old_removed = previous.get(epoch, (set(), set()))
        if not old_ttl <= current_ttl or not old_removed <= current_removed:
            raise ValueError('Cumulative retirement sets decreased within observer epoch')
        previous[epoch] = current_ttl, current_removed
        ttl.update((epoch, int(i)) for i in current_ttl)
        removed.update((epoch, int(i)) for i in current_removed)
        if ttl & admitted or ttl & removed or not removed <= admitted:
            raise ValueError('Candidate TTL and admitted retirement populations inconsistent')
    return {'never_admitted_candidate_ttl_tracks': len(ttl),
            'admitted_active_removal_tracks': len(removed),
            'retirement_count_contract': 'Unique (epoch,id); candidate TTL never occupied a core slot; admitted removals previously occupied a core slot. Observer reset is not counted as ordinary active retirement.'}


def merge_rows(original, recovered):
    lookup = {}
    for index, row in enumerate(original):
        key = (int(row['camera_ns']) * 1e-9, row['sequence'], row['epoch'])
        if key in lookup:
            raise ValueError('Ambiguous original physical camera/sequence/epoch key')
        lookup[key] = index
    result = copy.deepcopy(original)
    matched = set()
    admitted = {}
    last_index = -1
    evidence = []
    for row in recovered:
        key = (row['camera_time'], row['sequence'], row['epoch'])
        if key not in lookup:
            raise ValueError('Recovery event has no exact physical camera/sequence/epoch match')
        index = lookup[key]
        if index <= last_index:
            raise ValueError('Recovery camera events are duplicate or out of order')
        last_index = index
        old = original[index]
        for field in PRESERVED:
            if row[field] != old[field]:
                raise ValueError(f'Raw output mismatch at row {index}: {field}')
        original_guard_succeeded = 'admitted_tracks' in old
        if original_guard_succeeded:
            for field in MANAGEMENT:
                if old[field] != row[field]:
                    raise ValueError(f'Previously logged management differs at row {index}: {field}')
        if row['event_type'] == 'pause':
            evidence.append({'row': index, 'event': 'pause', 'management_replaced': False})
            matched.add(index)
            continue
        if row['event_type'] != 'process':
            raise ValueError('Unknown recovery event type')
        if not row['available'] or not row['management_accepted_input']:
            if any(s.get('admitted') for s in row['seeds']):
                raise ValueError('Unavailable/rejected frame has admitted seed')
            for field in MANAGEMENT:
                result[index][field] = copy.deepcopy(row[field])
            matched.add(index)
            evidence.append({'row': index, 'event': 'process_unavailable', 'management_replaced': True})
            continue
        for field in ('current_cam0_ids', 'current_stereo_ids'):
            if row[field] != old[field]:
                raise ValueError(f'Recovered causal frontend identities differ: {field}')
        epoch_admitted = admitted.setdefault(row['epoch'], set())
        for seed in row['seeds']:
            if seed.get('admitted'):
                feature_id = int(seed['id'])
                if feature_id in epoch_admitted:
                    raise ValueError('Duplicate recovered admission')
                epoch_admitted.add(feature_id)
        if len(epoch_admitted) != row['admitted_tracks']:
            raise ValueError('Recovered admissions do not equal cumulative manager counter')
        retained = set(row['retained_ids'])
        if len(retained) != len(row['retained_ids']) or not retained <= epoch_admitted:
            raise ValueError('Retained identities are not unique admitted identities')
        if len(retained) != old['state_features']:
            raise ValueError('Retained identities disagree with immutable core active count')
        visible = retained & set(old['current_cam0_ids'])
        if not 0 <= old['mature_features'] <= len(visible) <= len(retained):
            raise ValueError('Mature/visible/retained counts inconsistent')
        for field in MANAGEMENT:
            result[index][field] = copy.deepcopy(row[field])
        # All fields not explicitly permitted above must retain original values.
        for field in set(old) - set(MANAGEMENT):
            if result[index][field] != old[field]:
                raise AssertionError('Overlay changed a protected field')
        matched.add(index)
        evidence.append({'row': index, 'event': 'process', 'management_replaced': True,
                         'original_guard_success_exact_management': original_guard_succeeded,
                         'original_camera_ns': old['camera_ns'],
                         'recovered_camera_ns_roundtrip': row['camera_ns_roundtrip'],
                         'cumulative_admissions': len(epoch_admitted)})
    missing = [i for i, r in enumerate(original)
               if r.get('initialized') and r.get('available') and i not in matched]
    if missing:
        raise ValueError(f'Initialized available packets absent from recovery: {missing[:10]}')
    return result, evidence


def verify_binding(binding, run, recovered, provenance):
    value = json.loads(Path(binding).read_text())
    for field, path in (('recovered_sha256', recovered), ('cache_sha256', Path(run) / 'cache.bin'),
                        ('report_sha256', provenance)):
        if value[field] != sha(path):
            raise ValueError(f'Recovery binding SHA mismatch: {field}')
    if value['attempt'].get('exit_code') != 0:
        raise ValueError('Recovery attempt did not exit successfully')
    buildpath = Path(value['buildproof'])
    if sha(buildpath) != value['buildproof_sha256']:
        raise ValueError('Recovery buildproof SHA mismatch')
    build = json.loads(buildpath.read_text())
    frozen = json.loads(FROZEN.read_text())
    if sha(FROZEN) != value['freeze_sha'] or build['freeze_sha256'] != value['freeze_sha']:
        raise ValueError('Recovery freeze SHA mismatch')
    if build['runtime'] != value['runtime'] or frozen['runtime'] != value['runtime']:
        raise ValueError('Recovery runtime differs from frozen runtime')
    if build['source_sha256'] != sha(Path(__file__).with_name('recover.cpp')):
        raise ValueError('Recovery source differs from build proof')
    binary = Path(build['command'][build['command'].index('-o') + 1])
    if build['binary_sha256'] != value['binary_sha'] or sha(binary) != value['binary_sha']:
        raise ValueError('Recovery binary SHA mismatch')
    if build['build']['exit_code'] != 0:
        raise ValueError('Recovery binary build failed')
    if Path(value['original_run']).resolve() != Path(run).resolve():
        raise ValueError('Binding belongs to another original run')
    return value


def recover(run, metrics, recovered, provenance, binding, out):
    run, metrics, recovered, provenance, binding, out = map(Path, (run, metrics, recovered, provenance, binding, out))
    if out.exists():
        raise FileExistsError(out)
    identity = json.loads((run / 'identity.json').read_text())
    before = json.loads(metrics.read_text())
    expected = before['provenance']['run_files']['P_NEW']['features.jsonl']
    if expected != sha(run / 'features.jsonl'):
        raise ValueError('Original metrics/features identity mismatch')
    if before['seed_source_identity'] != identity:
        raise ValueError('Original metrics/run identity mismatch')
    if before['provenance']['evaluator_sha'] != sha(real_evaluate.__file__):
        raise ValueError('Evaluator differs from original frozen evaluation')
    binding_value = verify_binding(binding, run, recovered, provenance)
    recovered_rows = read_rows(recovered)
    proof = json.loads(provenance.read_text())
    if (proof.get('status') != 'PASS' or proof.get('comparison') != 'exact_frozen_cache_non_wallclock'
            or proof.get('compared_events') != len(recovered_rows)):
        raise ValueError('Recovery lacks successful complete exact-cache replay proof')
    overlay, evidence = merge_rows(read_rows(run / 'features.jsonl'), recovered_rows)
    management = real_evaluate.manager_metrics(overlay, identity['source'])
    management.update(lifecycle_counts(recovered_rows))
    after = copy.deepcopy(before)
    after['feature_management'] = management
    after['tables']['seed_reliability_available_evidence'] = {
        **management, 'reliability_claim': 'UNAVAILABLE_REAL_LANDMARK_GT'}
    after['tables']['feature_management']['management'] = management
    record = {'status': 'MANAGEMENT_DIAGNOSTICS_RECOVERED',
              'reason': 'Original integer camera timestamp equality lost diagnostics after double roundtrip',
              'original_run': str(run), 'original_metrics': str(metrics),
              'recovered_rows': str(recovered), 'recovery_provenance': str(provenance),
              'binding': binding_value,
              'sha': {str(p): sha(p) for p in (metrics, recovered, provenance, binding, run / 'features.jsonl', Path(__file__), Path(real_evaluate.__file__))},
              'raw_outputs_ready_reference_errors_thresholds_preserved': True,
              'GT_reread': False, 'matches': evidence}
    after['diagnostic_recovery'] = {k: v for k, v in record.items() if k != 'matches'}
    out.mkdir(parents=True)
    (out / 'overlay_features.jsonl').write_text(''.join(json.dumps(r, allow_nan=False) + '\n' for r in overlay))
    (out / 'metrics.json').write_text(json.dumps(after, indent=2, allow_nan=False))
    (out / 'recovery_audit.json').write_text(json.dumps(record, indent=2, allow_nan=False))
    shutil.copyfile(metrics.parent / 'error_curves.npz', out / 'error_curves.npz')
    if sha(metrics.parent / 'error_curves.npz') != sha(out / 'error_curves.npz'):
        raise AssertionError('Error curve copy differs')
    return {'out': str(out), 'admitted_tracks': management['admitted_tracks'], 'matched_events': len(evidence)}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('run', 'metrics', 'recovered', 'provenance', 'binding', 'out'):
        parser.add_argument('--' + name, required=True)
    print(json.dumps(recover(**vars(parser.parse_args()))))
