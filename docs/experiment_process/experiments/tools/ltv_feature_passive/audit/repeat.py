"""Exact repeated P_NEW replay audit, including auxiliary outputs and complete cache."""
import argparse
import json
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from artifacts import sha
from audit.passive_outputs import compare

def repeat(first,second):
    paths=[Path(first),Path(second)]
    identities=[json.loads((p/'identity.json').read_text()) for p in paths]
    for d in identities:
        if d['mode']!='P_NEW' or d.get('phase')!='confirmation' or not d.get('freeze_sha'):
            raise ValueError('Expected frozen confirmation P_NEW')
    if [d.get('repeat') for d in identities]!=[0,1]:raise ValueError('Expected prescribed original and repeat1')
    if {k:v for k,v in identities[0].items() if k!='repeat'}!={k:v for k,v in identities[1].items() if k!='repeat'}:
        raise ValueError('Repeated input/runtime/configuration identity differs')
    result=compare(paths);digests={}
    for name in ('features.jsonl','cache.bin'):
        values=[sha(p/name) for p in paths]
        if values[0]!=values[1]:raise RuntimeError('Repeated auxiliary output differs: '+name)
        digests[name]=values[0]
    result.update(repeated_auxiliary_exact=True,auxiliary_sha256=digests,paths=[str(p) for p in paths],
                  sequence=identities[0]['sequence'],identity=identities[0],repeat_indices=[0,1])
    return result

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('first',type=Path);p.add_argument('second',type=Path)
    a=p.parse_args();print(json.dumps(repeat(a.first,a.second),indent=2))
