"""Independent offline lifecycle conservation audit; does not change evaluation."""
import collections,json,sys
from pathlib import Path
p=Path(sys.argv[1]); births={}; retired=set(); ttl=set(); totals=collections.defaultdict(collections.Counter); sources=collections.Counter(); n=0
for line in (p/'features.jsonl').open():
 r=json.loads(line);n+=1
 if not r['management_present']:continue
 e=r['epoch']; c=totals[e]
 assert r['bounded_management_schema']=='BOUNDED_V1' and r['retirement_ids_scope']=='CURRENT_EVENT'
 active=[]; expired=[]
 for s in r['seeds']:
  if s['admitted']:
   key=(e,s['id']);assert key not in births and s['mean_written']; births[key]=s['source'];sources[s['source']]+=1;c['births']+=1
 for ev in r['retirement_events']:
  key=(e,ev['id']);rec=ev['record'];assert ev['epoch']==e and rec['id']==ev['id'] and rec['phase']=='RETIRED'
  assert ev['time']==r['target_imu_time']
  tracks=[t for t in r['tracks'] if t['id']==ev['id']];assert len(tracks)==1 and tracks[0]==rec
  if ev['kind']=='ACTIVE_RETIRED':
   assert key in births and key not in retired and rec['entered']>=0 and rec['seed_written'];retired.add(key);active.append(ev['id']);c['active']+=1
  else:
   assert ev['kind']=='CANDIDATE_TTL' and key not in births and key not in ttl and rec['entered']<0;ttl.add(key);expired.append(ev['id']);c['ttl']+=1
 assert sorted(active)==sorted(r['admitted_retired_ids'])==sorted(r['retired_ids'])
 assert sorted(expired)==sorted(r['never_admitted_candidate_ttl_ids'])
 c['rejections']+=len(r['identity_guard_rejected_ids'])
 assert c['births']==r['admitted_tracks'] and c['active']==r['active_retired_total'] and c['ttl']==r['candidate_ttl_total'] and c['rejections']==r['identity_guard_rejections_total']
 assert set(r['retained_ids'])=={i for epoch,i in births if epoch==e and (epoch,i) not in retired}
result={'status':'PASS','receipts':n,'epochs':dict(totals),'admitted_sources':dict(sources),'births':len(births),'active_retired':len(retired),'candidate_ttl':len(ttl)}
Path(sys.argv[2]).write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
