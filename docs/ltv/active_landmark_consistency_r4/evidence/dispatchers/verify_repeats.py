import json,sys
from pathlib import Path
ROOT=Path('/home/he/open_vins_ltv_ws/src/open_vins_ltv');OUT=Path('/home/he/output/ltv_active_landmark_consistency_r4')
sys.path.insert(0,str(ROOT/'scripts/ltv_active_consistency_r4'))
import artifacts,budget
f=json.loads((budget.DOC/'frozen_config.json').read_text());fsha=artifacts.sha(budget.DOC/'frozen_config.json');summary=json.loads((OUT/'CONF_REPEAT_C02_01_confirmation_jobs.json').read_text())
assert summary['complete'] and summary['freeze_sha']==fsha and all(x['exit_code']==0 for x in summary['results'])
assert len(summary['results'])==2 and {x['sequence'] for x in summary['results']}==set(f['representative_repeats'])
rows=[]
for seq in f['representative_repeats']:
 a=OUT/'real_confirmation'/('CONF_ON_C02_01_ON_'+seq)/'P_NEW';b=OUT/'real_confirmation'/('CONF_REPEAT_C02_01_ON_'+seq)/'P_NEW'
 ia=json.loads((a/'identity.json').read_text());ib=json.loads((b/'identity.json').read_text())
 for k in ('runtime','manifest_sha','config_sha','inputs','source_sha','active_config','freeze_sha'):assert ia[k]==ib[k],k
 proof={}
 for name in ('cache.bin','audit.csv','trajectory.csv'):
  proof[name]=dict(original=artifacts.sha(a/name),repeat=artifacts.sha(b/name));assert proof[name]['original']==proof[name]['repeat'],(seq,name)
 rows.append(dict(sequence=seq,status='PASS_FULL_NATIVE_EXACT_REPEAT',files=proof,scope='All cache input/events and complete non-wallclock x/P/slot/ready/active evidence bytes, plus main audit/trajectory'))
result=dict(status='PASS_TWO_FULL_NATIVE_REPEATS',freeze_sha=fsha,rows=rows)
p=OUT/'formal_repeats_exact.json';assert not p.exists();p.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
