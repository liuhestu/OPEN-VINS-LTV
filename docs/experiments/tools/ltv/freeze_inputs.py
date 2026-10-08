#!/usr/bin/env python3
"""Read-only manifest of every ASL input and official state GT, before final evaluation."""
import hashlib,json,pathlib
from run_sequence import OUT,ASL,sha
m=json.loads((OUT/'experiment_manifest.json').read_text());result={}
for sequence in m['final_sequences']:
    base=ASL/sequence/'mav0';files={}
    for component in ['imu0','cam0','cam1','state_groundtruth_estimate0']:
        for p in sorted((base/component).rglob('*')):
            if p.is_file() and (p.suffix in ['.csv','.png','.yaml']):files[str(p.relative_to(base))]=sha(p)
    assert 'imu0/data.csv' in files and 'state_groundtruth_estimate0/data.csv' in files
    digest=hashlib.sha256(json.dumps(files,sort_keys=True).encode()).hexdigest()
    result[sequence]=dict(root=str(base),files=files,sha256=digest)
    print(sequence,len(files),digest,flush=True)
path=OUT/'input_manifest.json'
if path.exists():
    assert json.loads(path.read_text())==result,'Input changes detected; refusing overwrite'
else:path.write_text(json.dumps(result,indent=2))
