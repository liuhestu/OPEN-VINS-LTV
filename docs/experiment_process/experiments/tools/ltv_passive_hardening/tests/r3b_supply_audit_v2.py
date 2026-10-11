import json,math,collections
from pathlib import Path
p=Path('/home/he/output/ltv_passive_hardening_v2/real_development/R3B_grace_V2_03_01')
rows=[json.loads(l) for l in open(p/'P_NEW/features.jsonl')];rows=[r for r in rows if r['initialized']]
t0=rows[0]['imu_time']
def episodes(mask):
 out=[];start=None
 for k,good in enumerate(mask):
  if start is not None and (not good or rows[k]['epoch']!=rows[k-1]['epoch'] or rows[k]['imu_time']-rows[k-1]['imu_time']>.075):
   j=k-1;out.append((start,j));start=None
  if good and start is None:start=k
 if start is not None:out.append((start,len(rows)-1))
 return out
def desc(pair):
 a,b=pair;return dict(start=rows[a]['imu_time']-t0,end=rows[b]['imu_time']-t0,duration=rows[b]['imu_time']-rows[a]['imu_time'],packets=b-a+1)
flags={k:[] for k in ['current','observed15','mature15','pool','hard','strict','joint']}
def finite(v):return isinstance(v,(int,float)) and math.isfinite(v)
for r in rows:
 current=bool(r.get('raw_current'));obs=r.get('observed_features',0)>=15;mat=r.get('mature_features',0)>=15
 eta=r.get('eta_body');norm=math.sqrt(sum(x*x for x in eta)) if eta else -1
 rates=[r.get('prediction_angle_p95_rad'),r.get('velocity_correction_rate'),r.get('gravity_correction_rate')]
 diag=r.get('correction_diagnostics_valid') and all(finite(x) and x>=0 for x in rates)
 hard=current and obs and mat and r.get('pool_ready') and diag and 8<=norm<=11.5 and r.get('actual_corrections',0)>=20
 strict=bool(hard and rates[0]<=.02 and rates[1]<=.5 and rates[2]<=1)
 for k,v in zip(flags,[current,current and obs,current and mat,current and bool(r.get('pool_ready')),bool(hard),strict,bool(r.get('ready_G') and r.get('ready_V'))]):flags[k].append(v)
summary={k:{'packets':sum(v),'longest':max((desc(e)['duration'] for e in episodes(v)),default=0),'episodes_ge5':[desc(e) for e in episodes(v) if desc(e)['duration']>=5]} for k,v in flags.items()}
# Every current-state interval containing at least five seconds: reason histogram.
windows=[]
for a,b in episodes(flags['current']):
 if rows[b]['imu_time']-rows[a]['imu_time']<5:continue
 rr=rows[a:b+1];reason=collections.Counter(r['reason'] for r in rr if not(r.get('ready_G') and r.get('ready_V')))
 windows.append({**desc((a,b)),'nonjoint_reasons':dict(reason)})
# Hard-support windows that could possibly sustain a five-second output under any soft rule.
hardwindows=[]
for a,b in episodes(flags['hard']):
 if rows[b]['imu_time']-rows[a]['imu_time']<5:continue
 bad=[r for r in rows[a:b+1] if r['prediction_angle_p95_rad']>.02 or r['velocity_correction_rate']>.5 or r['gravity_correction_rate']>1]
 hardwindows.append({**desc((a,b)),'strict_bad_packets':len(bad),'prediction_over':sum(r['prediction_angle_p95_rad']>.02 for r in bad),'v_rate_over':sum(r['velocity_correction_rate']>.5 for r in bad),'g_rate_over':sum(r['gravity_correction_rate']>1 for r in bad),'bad_packets':[{'t':r['imu_time']-t0,'reason':r['reason'],'observed':r['observed_features'],'mature':r['mature_features'],'prediction':r['prediction_angle_p95_rad'],'v_rate':r['velocity_correction_rate'],'g_rate':r['gravity_correction_rate']} for r in bad]})
supply=[]
for k,r in enumerate(rows):
 if not flags['current'][k] or flags['pool'][k]:continue
 tracks=r.get('tracks',[]);born=[s for s in r.get('seeds',[]) if s.get('admitted')]
 supply.append({'t':r['imu_time']-t0,'raw_frontend':len(set(r.get('current_cam0_ids',[]))),'state':r['state_features'],'observed':r['observed_features'],'mature':r['mature_features'],'eligible':r.get('eligible_seeds'),'born':len(born),'retired':len(r.get('retired_ids',[])),'coasting':sum(x['phase']=='COASTING' for x in tracks),'capacity_ready':sum(x.get('reason')=='capacity' and x.get('last_seen')==r['imu_time'] and x['id'] in r.get('current_cam0_ids',[]) for x in tracks)})
shortage={'packets':len(supply),'observed_below15':sum(x['observed']<15 for x in supply),'maturity_only':sum(x['observed']>=15 and x['mature']<15 for x in supply),'frontend_below15':sum(x['raw_frontend']<15 for x in supply),'state_full30':sum(x['state']==30 for x in supply),'capacity_ready_present':sum(x['capacity_ready']>0 for x in supply),'rows':supply}
# Log actual retirement reasons and early exits from already-ready joint episodes.
retire=collections.Counter(ev['record']['reason'] for r in rows for ev in r.get('retirement_events',[]) if ev['kind']=='ACTIVE_RETIRED')
exits=[]
for k in range(1,len(rows)):
 if flags['joint'][k-1] and not flags['joint'][k]:
  r=rows[k];exits.append({'t':r['imu_time']-t0,'reason':r['reason'],'observed':r['observed_features'],'mature':r['mature_features'],'prediction':r.get('prediction_angle_p95_rad'),'v_rate':r.get('velocity_correction_rate'),'g_rate':r.get('gravity_correction_rate'),'retired_ids':r.get('retired_ids',[]),'born':sum(s.get('admitted',False) for s in r.get('seeds',[]))})
out=dict(t0=t0,summary=summary,current_windows=windows,hard_windows=hardwindows,pool_shortages=shortage,active_retirement_reasons=dict(retire),joint_exits=exits)
(p/'supply_break_audit_v2.json').write_text(json.dumps(out,indent=2))
print(json.dumps({'summary':summary,'hard_windows':len(hardwindows),'shortages':{k:v for k,v in shortage.items() if k!='rows'},'joint_exit_reasons':dict(collections.Counter(x['reason'] for x in exits)),'retirement_reasons':dict(retire)}))

import numpy as np
with np.load(p/'evaluation_v1/error_curves.npz') as z:curves={k:z[k] for k in z.files}
lookup={int(ns):i for i,ns in enumerate(curves['camera_ns'])}
windows=[]
for a,b in episodes(flags['pool']):
 if rows[b]['imu_time']-rows[a]['imu_time']<5:continue
 rr=rows[a:b+1];idx=[lookup[r['camera_ns']] for r in rr]
 item=desc((a,b));item['nonjoint_reasons']=dict(collections.Counter(r['reason'] for r in rr if not(r['ready_G'] and r['ready_V'])))
 item['raw_errors']={}
 for metric in ['v','eta','g_angle_deg']:
  values=curves['P_NEW_error_'+metric][idx];values=values[np.isfinite(values)]
  item['raw_errors'][metric]={'finite':len(values),'RMSE':float(np.sqrt(np.mean(values**2))) if len(values) else None,'max':float(np.max(values)) if len(values) else None}
 bad=[r for r in rr if r['prediction_angle_p95_rad']>.02]
 item['prediction_bad_all_observed_mature']=sum(r['observed_features']==r['mature_features'] for r in bad)
 item['prediction_bad_total']=len(bad)
 item['births_at_prediction_bad']=sum(sum(s.get('admitted',False) for s in r.get('seeds',[])) for r in bad)
 windows.append(item)
visible_retirement=[]
for r in rows:
 vis=set(r.get('current_cam0_ids',[]))
 for ev in r.get('retirement_events',[]):
  if ev['kind']=='ACTIVE_RETIRED' and ev['id'] in vis:visible_retirement.append({'t':r['imu_time']-t0,'id':ev['id'],'record':ev['record']})
specific=[]
for r in rows:
 t=r['imu_time']-t0
 if 9.8<=t<=10.16:
  retained=set(r['retained_ids']);ages=[r['imu_time']-x['entered'] for x in r['tracks'] if x['id'] in retained and x['entered']>=0]
  specific.append({'t':t,'prediction':r['prediction_angle_p95_rad'],'observed':r['observed_features'],'mature':r['mature_features'],'youngest_residence':min(ages) if ages else None,'births':sum(s.get('admitted',False) for s in r.get('seeds',[]))})
report={'pool_windows':windows,'current_visible_active_retirements':visible_retirement,'all_mature_prediction_break':specific,'pool_shortages_corrected_counts':{k:v for k,v in shortage.items() if k!='rows'}}
(p/'supply_and_error_detail.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report))
