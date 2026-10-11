"""Repeat determinism is exact reproducibility, never independent statistical samples."""
import argparse
import hashlib
import json
from pathlib import Path


def sha(path):
    h=hashlib.sha256()
    with path.open('rb') as f:
        for b in iter(lambda:f.read(1024*1024),b''):h.update(b)
    return h.hexdigest()


def main(a,b,out):
    files=['trajectory.csv','audit.csv','lifecycle_identity.jsonl','algorithm_visual_identity.jsonl',
           'observer_identity.jsonl','effective_options.json','replay.json','fusion.jsonl',
           'unmatched_camera.csv','landmark_approx.csv','landmark_approx.csv.points.csv']
    result={'status':'PASS_EXACT_REPEAT','interpretation':'identical deterministic input, not independent sample','files':{}}
    for name in files:
        x,y=sha(a/name),sha(b/name)
        result['files'][name]={'first_sha256':x,'repeat_sha256':y,'exact':x==y}
        assert x==y,name
    with out.open('x') as f:json.dump(result,f,indent=2);f.write('\n')
    print(result['status'],len(files))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('first',type=Path);p.add_argument('repeat',type=Path);p.add_argument('out',type=Path);a=p.parse_args();main(a.first,a.repeat,a.out)
