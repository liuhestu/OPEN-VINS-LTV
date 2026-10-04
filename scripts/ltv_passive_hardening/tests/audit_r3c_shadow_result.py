"""Independent saved-shadow audit: fixed scientific metrics, no policy generation."""
import sys,json,hashlib
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from real_evaluate import episodes,summarize,physical
root=Path('/home/he/output/ltv_passive_hardening_v2');d=root/'diagnostics/R3C_shadow_01'
evaldir=root/'real_development/R3B_grace_V2_03_01/evaluation_v1';a=dict(np.load(evaldir/'error_curves.npz'))
r=[json.loads(s) for s in (d/'policy.jsonl').read_text().splitlines()];lookup={x['camera_ns']:x for x in r};assert len(lookup)==len(r)
m=json.loads((evaldir/'metrics.json').read_text());fs,fl=physical.read_features(Path(m['provenance']['runs']['P_NEW'])/'features.jsonl')
assert len(r)==len(fs)
p=json.loads((d/'policy_provenance.json').read_text());sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
assert p['policy_sha']==sha(d/'policy.jsonl')
assert p['script_sha']==sha(Path(__file__).parents[1]/'diagnostics/r3c_shadow_audit.py')
# Recheck saved decisions and counters independently from recorded window measures.
g=v=0
for z,f in zip(r,fs):
 assert z['camera_ns']==f['camera_ns'] and z['time']==f['target_imu_time']
 if z['clear_reason']:
  g=v=0;assert not z['ready_G'] and not z['ready_V']
 else:
  b=z['budgets'];G=z['complete'] and b['angle_mean']<=.02 and b['G']['net']<=1 and b['G']['activity']<=2 and b['G']['impulse']<=1
  V=G and b['V']['net']<=.5 and b['V']['activity']<=1 and b['V']['impulse']<=.5
  g=min(g+1,20) if G else 0;v=min(v+1,20) if V else 0
  assert z['confirm_G']==g and z['confirm_V']==v and z['ready_G']==(g==20) and z['ready_V']==(v==20)
  assert z['complete']==(z['time']-z['anchor']>=1)
  assert f['raw_current'] and f['observer_valid'] and f['pool_ready'] and min(f['observed_features'],f['mature_features'])>=15
  assert f['prediction_angle_p95_rad']<=.04 and 8<=np.linalg.norm(f['eta_body'])<=11.5
selected=[lookup[int(k)] for k in a['camera_ns']]
assert all(x['time']==t for x,t in zip(selected,a['time']))
G=np.array([x['ready_G'] for x in selected]);V=np.array([x['ready_V'] for x in selected]);epoch=a['P_NEW_epoch']
summary={};comparisons={}
for label,mask in [('G',G),('V',V),('joint',G&V)]:
 q='eta' if label=='G' else 'v';valid=np.isfinite(a['reference_'+q]).all(axis=1)
 eps=episodes(a['time'],mask,epoch)
 summary[label]={'packets':int(mask.sum()),'fraction':float(mask.mean()),'reference_seconds':float(a['weights'][mask&valid].sum()),'reference_missing_packets':int(sum(mask&~valid)),'longest_episode_s':max((e['duration'] for e in eps),default=0),'episodes':eps}
 comparisons[label]={}
 for method in ('P_NEW','P_PREV','OpenVINS'):
  err={k[len(method+'_error_'):]:a[k] for k in a if k.startswith(method+'_error_')}
  comparisons[label][method]=summarize(err,mask,a['weights'])
threshold=json.loads((Path(__file__).parents[3]/'docs/ltv/passive_hardening_v2/acceptance.json').read_text())['each_valid_sequence']
decisions={'coverage_G':summary['G']['fraction']>=.2 and summary['G']['reference_seconds']>=10,'coverage_V':summary['V']['fraction']>=.2 and summary['V']['reference_seconds']>=10,'joint5s':summary['joint']['longest_episode_s']>=5,'G_error':comparisons['G']['P_NEW']['g_angle_deg_RMSE']<=1 and comparisons['G']['P_NEW']['g_angle_deg_P95']<=2,'V_error':comparisons['V']['P_NEW']['v_RMSE']<=.2 and comparisons['V']['P_NEW']['v_P95']<=.5,'G_severe':comparisons['G']['P_NEW']['severe_G_fraction']<=.01,'V_severe':comparisons['V']['P_NEW']['severe_V_fraction']<=.01}
targets=[]
for ns in (1413394964105760512,1413394966255760384):
 z=lookup[ns];j=next(i for i,x in enumerate(r) if x['camera_ns']==ns);lastclear=next(x for x in reversed(r[:j+1]) if x['clear_reason'])
 targets.append({'target':z,'last_clear':lastclear,'last_clear_original_angle':fl[lastclear['camera_ns']]['prediction_angle_p95_rad'],'support_age_s':z['time']-z['anchor']})
result={'status':'SHADOW_ONLY_NOT_IMPLEMENTATION_ACCEPTANCE','decisions':decisions,'summary':summary,'same_support_errors':comparisons,'targets':targets,'provenance':{str(p):sha(p) for p in (Path(__file__),d/'policy.jsonl',d/'policy_provenance.json',evaldir/'error_curves.npz')},'limitations':['No observer replay or new policy masks.','Counter audit uses recorded net/activity statistics; it is not an independent reconstruction of every transported delta.','Prior 1.5 median cadence episode rule unchanged.','Passing this single saved policy would not establish all11, recovery, resource, seed or implementation acceptance.']}
(d/'independent_scientific_audit.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
print(json.dumps({'decisions':decisions,'summary':{k:{n:v for n,v in x.items() if n!='episodes'} for k,x in summary.items()},'ready_errors':{k:v['P_NEW'] for k,v in comparisons.items()},'targets':[{ 'camera_ns':x['target']['camera_ns'],'ready_G':x['target']['ready_G'],'ready_V':x['target']['ready_V'],'support_age_s':x['support_age_s'],'last_clear_time':x['last_clear']['relative_time'],'last_clear_angle':x['last_clear_original_angle']} for x in targets]},indent=2))
