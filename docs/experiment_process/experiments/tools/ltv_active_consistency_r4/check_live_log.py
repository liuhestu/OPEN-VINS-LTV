"""Audit emitted consistency decisions, counters and actual sent observation count."""
import argparse,collections,json,math
from pathlib import Path

def check(path):
 def unique(pairs):
  d={}
  for k,v in pairs:
   if k in d:raise ValueError('duplicate JSON key '+k)
   d[k]=v
  return d
 rows=[json.loads(l,object_pairs_hook=unique,parse_constant=lambda x: (_ for _ in ()).throw(ValueError(x))) for l in Path(path).read_text().splitlines()]
 if not rows:raise ValueError('empty live log')
 totals=collections.Counter();reasons=collections.Counter();last={};rejects={};active=[]
 for r in rows:
  if not r.get('management_present'):continue
  assert r['active_consistency_schema']=='ACTIVE_CONSISTENCY_V1'
  ds=r['active_consistency'];ids=[d['feature_id'] for d in ds];assert len(ids)==len(set(ids))
  assert len(ds)<=r['consistency_active_count']
  assert r['consistency_evaluable_count']==sum(bool(d['evaluable']) for d in ds)
  assert r['consistency_pass_count']==sum(bool(d['evaluable'] and d['consistency_pass']) for d in ds)
  assert r['consistency_skip_count']==sum(d['action']=='SKIP' for d in ds)
  assert r['consistency_retire_count']==sum(d['action']=='RETIRE' for d in ds)
  assert r['consistency_evaluable_count']==r['consistency_pass_count']+r['consistency_skip_count']+r['consistency_retire_count']
  active.append(r['consistency_active_count'])
  for d in ds:
   assert d['timestamp']==r['imu_time'];reasons[d['reason']]+=1
   key=(r['epoch'],d['feature_id']);prior=last.get(key,0)
   if d['evaluable']:
    assert d['history_max_residual_rad'] is not None
    expected=0 if d['consistency_pass'] else prior+1
    assert d['consistency_fail_count']==expected,(key,prior,d)
    if not d['consistency_pass']:
     assert d['action'] in ('SKIP','RETIRE')
     assert d['feature_id'] not in r['corrected_ids']
     assert (d['action']=='RETIRE')==(expected>=2)
     assert (d['feature_id'] in r['retained_ids'])==(d['action']=='SKIP')
     if d['action']=='RETIRE':
      matching=[e for e in r['retirement_events'] if e['id']==d['feature_id'] and e['kind']=='ACTIVE_RETIRED' and e['record']['reason']=='active_consistency']
      assert len(matching)==1
   else:
    assert d['consistency_pass'] and d['action']=='UPDATE'
    assert d['consistency_fail_count']==prior
   last[key]=d['consistency_fail_count']
   expected_rejects=rejects.get(key,0)+int(bool(d['evaluable'] and not d['consistency_pass']))
   assert d['consistency_reject_count']==expected_rejects
   rejects[key]=expected_rejects
  for k in ('active','evaluable','pass','skip','retire'):totals[k]+=r['consistency_'+k+'_count']
  totals['diagnostics']+=len(ds);totals['sent']+=r['observations_sent_to_ltv'];totals['frames']+=1
 if not totals['frames']:raise ValueError('no consistency management frames')
 return dict(status='PASS_LIVE_COUNTERS_AND_LIFECYCLE',totals=dict(totals),reasons=dict(reasons),
  evaluable_fraction=totals['evaluable']/totals['diagnostics'] if totals['diagnostics'] else None,
  reject_fraction=(totals['skip']+totals['retire'])/totals['evaluable'] if totals['evaluable'] else None,
  min_active=min(active) if active else None,
  limitations=['active denominator includes coasting slots; diagnostics require a current visible valid observation',
  'actual absent rejected observations are verified by native cache/core replay, not inferred from aggregate sent count'])
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('path');p.add_argument('--out');a=p.parse_args();r=check(a.path)
 if a.out:Path(a.out).write_text(json.dumps(r,indent=2)+'\n')
 print(json.dumps(r,indent=2))
