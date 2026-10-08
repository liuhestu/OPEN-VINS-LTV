#!/usr/bin/env python3
"""Rehash real sensor/image/GT inputs; no estimator or truth-based selection."""
import argparse
import hashlib
import json
from pathlib import Path


def sha(path):
    h = hashlib.sha256()
    with path.open('rb') as f:
        for block in iter(lambda: f.read(4 * 1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()

p = argparse.ArgumentParser()
p.add_argument('source', type=Path)
p.add_argument('data_root', type=Path)
p.add_argument('output', type=Path)
a = p.parse_args()
prior = json.loads((a.source / 'docs/ltv/landmark_validation/freeze.json').read_text())['inputs']
results = {}
for sequence in ('V2_02_medium', 'V2_03_difficult', 'V1_01_easy', 'V1_03_difficult'):
    root = a.data_root / sequence / 'mav0'
    images, counts = {}, {}
    sensor = {}
    for camera in ('cam0', 'cam1'):
        path = root / camera / 'data.csv'
        sensor[camera] = {'path': str(path), 'sha256': sha(path)}
        counts[camera] = 0
        for line in path.read_text().splitlines():
            if line and not line.startswith('#'):
                _, name = line.split(',', 1)
                relative = str(Path(camera) / 'data' / name.strip())
                images[relative] = sha(root / relative)
                counts[camera] += 1
    for kind in ('imu0', 'state_groundtruth_estimate0'):
        path = root / kind / 'data.csv'
        sensor[kind] = {'path': str(path), 'sha256': sha(path)}
    aggregate = hashlib.sha256(json.dumps(images, sort_keys=True, separators=(',', ':')).encode()).hexdigest()
    if sequence in prior:
        expected = prior[sequence]
        assert aggregate == expected['images_aggregate_sha'], 'dev image identity mismatch'
        for kind in ('cam0', 'cam1', 'imu0'):
            assert sensor[kind]['sha256'] == expected['sensors'][kind]['sha256'], (sequence, kind)
    results[sequence] = {'root': str(root), 'sensors': sensor, 'image_files': counts, 'images_aggregate_sha': aggregate,
                         'historical_expected_verified': sequence in prior}
    (a.output / ('images_' + sequence + '.json')).write_text(json.dumps(images, sort_keys=True) + '\n')
    print(sequence, len(images), aggregate, flush=True)
with (a.output / 'data_identity.json').open('x') as f:
    json.dump({'status': 'PASS', 'schema': 'sensor_csv_sha256+sorted_relative_image_sha256_aggregate_v1',
               'sequences': results, 'history': 'dev independently rehashed against old source freeze; holdout new hash'}, f, indent=2)
    f.write('\n')
