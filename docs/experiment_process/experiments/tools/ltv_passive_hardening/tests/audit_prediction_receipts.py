"""Current-record float association validation only; no general nanosecond claim."""
import json
from pathlib import Path
p=Path('/home/he/output/ltv_passive_hardening_v2/real_development/R3B_grace_V2_03_01/P_NEW/features.jsonl');r=[json.loads(l) for l in p.read_text().splitlines()];keys=[int(x['camera_ns']) for x in r];times=[k*1e-9 for k in keys];assert len(keys)==len(set(keys))==len(set(times))
d=Path('/home/he/output/ltv_passive_hardening_v2/diagnostics/R3B_V2_03_prediction_01');rows=[json.loads(l) for l in (d/'camera_diagnostics.jsonl').read_text().splitlines()];lookup={x['camera_ns']:x for x in r}
for x in rows:
 assert x['camera_ns'] in lookup and x['time']==x['camera_ns']*1e-9
 assert lookup[x['camera_ns']]['epoch']==x['epoch'] and lookup[x['camera_ns']]['raw_current']
assert len(rows)==1805
out={'status':'PASS_CURRENT_DATA_ONLY','original_receipts':len(r),'integer_ns_unique':True,'forward_float_keys_unique':True,'diagnostic_rows_matched':len(rows),'limitation':'Does not claim inverse float->ns identity or safe joining for arbitrary sub-ULP nanosecond inputs.'};(d/'independent_receipt_audit.json').write_text(json.dumps(out,indent=2)+'\n');print(json.dumps(out))
