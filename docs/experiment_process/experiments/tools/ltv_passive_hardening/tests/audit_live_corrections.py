"""Native live actual-correction fields; structural/causal audit, no GT."""
import collections,json,sys
from pathlib import Path
allow_legacy_duplicate='--allow-identical-duplicate-keys' in sys.argv
p=Path(sys.argv[1]);n=positive=empty=0;last={};dup=collections.Counter()
def pairs(items):
 r={}
 for k,v in items:
  if k in r:
   assert allow_legacy_duplicate,('duplicate key forbidden',k)
   assert r[k]==v,('conflicting duplicate key',k)
   dup[k]+=1
  r[k]=v
 return r
for line in (p/'features.jsonl').open():
 r=json.loads(line,object_pairs_hook=pairs);n+=1
 assert 'corrected_ids' in r and 'camera_substeps' in r
 ids=r['corrected_ids'];steps=r['camera_substeps']
 assert isinstance(steps,int) and steps>=0 and len(ids)==len(set(ids))
 if r['available'] and r['raw_current'] and steps>0:
  assert len(ids)==r['observed_features']
  assert set(ids)<=set(r['retained_ids']) and set(ids)<=set(r['current_cam0_ids'])
  positive+=bool(ids)
 else:assert not ids;empty+=1
 if r['management_present']:
  key=r['epoch']; prev=last.get(key,0); count=r['actual_corrections']
  assert count>=prev and count-prev<=1
  if count>prev:assert r['raw_current'] and steps>0 and len(ids)>=15
  last[key]=count
assert positive>0 and empty>0
result={'status':'PASS_ENGINEERING_ONLY','receipts':n,'nonempty_correction_events':positive,'no_current_correction_events':empty,'last_actual_corrections_per_epoch':last,'identical_duplicate_keys':dict(dup),'limitation':'IDs describe actual accepted camera correction input, not nonzero gain or accuracy for each ray; no GT used.'}
Path(sys.argv[2]).write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
