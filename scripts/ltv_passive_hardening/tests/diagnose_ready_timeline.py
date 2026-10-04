"""Offline cause diagnosis only: no policy edits, GT gating or observer execution."""
import argparse,json,hashlib,sys,collections
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from real_evaluate import episodes
import numpy as np
p=argparse.ArgumentParser();p.add_argument('evaluation');p.add_argument('--output-name',default='ready_timeline_diagnosis.json');args=p.parse_args();d=Path(args.evaluation);m=json.loads((d/'metrics.json').read_text());a=dict(np.load(d/'error_curves.npz'));path=Path(m['provenance']['runs']['P_NEW'])/'features.jsonl';lookup={r['camera_ns']:r for r in map(json.loads,path.read_text().splitlines())};rows=[lookup[int(k)] for k in a['camera_ns']];t=a['time'];epoch=a['P_NEW_epoch'];cad=float(np.median(np.diff(t)));hard=[];soft=[]
for r in rows:
 h=[];s=[]
 if not r['raw_current'] or not r['observer_valid']:h.append('not_current_valid')
 if not r['pool_ready']:h.append('pool_not_mature')
 if r['actual_corrections']<20:h.append('fewer_than_20_actual_corrections')
 if not r['correction_diagnostics_valid']:h.append('diagnostics_invalid')
 if r['eta_body'] is None or not 8<=np.linalg.norm(r['eta_body'])<=11.5:h.append('gravity_norm')
 for q,lim in [('prediction_angle_p95_rad',.02),('velocity_correction_rate',.5),('gravity_correction_rate',1.)]:
  if r[q] is None or not np.isfinite(r[q]) or r[q]<0 or r[q]>lim:s.append(q)
 hard.append(h);soft.append(s)
instant=np.array([not h and not s for h,s in zip(hard,soft)]);actual=a['P_NEW_ready_G']&a['P_NEW_ready_V'];raw=a['P_NEW_finite_current'];N=len(t)
def summary(mask):
 e=episodes(t,mask,epoch);return {'packets':int(np.sum(mask)),'longest_seconds':max((x['duration'] for x in e),default=0),'episodes':e}
def errors(i,j):
 out={}
 for k in ['v','g_angle_deg']:
  v=a['P_NEW_error_'+k][i:j+1];v=v[np.isfinite(v)];out[k+'_max']=float(max(v)) if len(v) else None
 return out
bad=[];i=0
while i<N:
 if instant[i]:i+=1;continue
 j=i
 while j+1<N and not instant[j+1] and t[j+1]-t[j]<=1.5*cad and epoch[j+1]==epoch[i]:j+=1
 hs=sorted(set(x for v in hard[i:j+1] for x in v));ss=sorted(set(x for v in soft[i:j+1] for x in v));isolated=i>0 and j+1<N and instant[i-1] and instant[j+1] and t[i]-t[i-1]<=1.5*cad and t[j+1]-t[j]<=1.5*cad
 bad.append(dict(start=float(t[i]),end=float(t[j]),samples=j-i+1,seconds=float(t[j]-t[i]),hard=hs,soft=ss,isolated_soft_1_or_2_frames=bool(not hs and j-i+1<=2 and isolated),raw_errors=errors(i,j)))
 i=j+1
s={k:summary(v) for k,v in [('all_initialized',np.ones(N,bool)),('legal_current',raw),('instant_all_existing_health_without_20_ready_confirmation',instant),('actual_joint',actual)]};longest=max(s['actual_joint']['episodes'],key=lambda x:x['duration']);near=[b for b in bad if b['end']>=longest['start']-5 and b['start']<=longest['end']+5];gapindices=np.flatnonzero(np.diff(t)>1.5*cad)+1;gaps=[{'before':float(t[i-1]),'after':float(t[i]),'seconds':float(t[i]-t[i-1]),'near_longest':bool(longest['start']-5<=t[i]<=longest['end']+5)} for i in gapindices]
assert not np.any(actual&~instant),'actual ready must satisfy instantaneous necessary conditions'
frame_failures=[dict(time=float(t[i]),relative_time=float(t[i]-t[0]),camera_ns=int(a['camera_ns'][i]),hard=hard[i],soft=soft[i],observed_features=rows[i]['observed_features'],mature_features=rows[i]['mature_features'],prediction_angle_p95_rad=rows[i]['prediction_angle_p95_rad'],velocity_correction_rate=rows[i]['velocity_correction_rate'],gravity_correction_rate=rows[i]['gravity_correction_rate'],raw_errors=errors(i,i)) for i in range(N) if not instant[i]]
result={'frame_failures':frame_failures,'purpose':'DIAGNOSTIC_ONLY_NOT_A_NEW_CANDIDATE','sequence':m['sequence'],'status_unchanged':m['status'],'gap_seconds_limit':1.5*cad,'timeline_origin':float(t[0]),'supports':s,'failures':bad,'failures_near_longest_actual_joint':near,'input_gaps':gaps,'isolated_soft_1_or_2_frame_failures':sum(b['isolated_soft_1_or_2_frames'] for b in bad),'instant_healthy_but_not_joint_packets':int(np.sum(instant&~actual)),'provenance':{'log_sha':hashlib.sha256(path.read_bytes()).hexdigest(),'curves_sha':hashlib.sha256((d/'error_curves.npz').read_bytes()).hexdigest(),'script_sha':hashlib.sha256(Path(__file__).read_bytes()).hexdigest()},'limits':['Instant health removes only twenty-ready-frame confirmation; min20 actual corrections, all physical/pool/current/diagnostics/rate criteria remain.','Hard/soft are descriptive categories, not changed rejection policy.','Saved raw errors describe every failing interval; no GT-based masking or filtering.','Input gap rule remains the original 1.5 median cadence; sparse camera inputs can fragment healthy intervals.']}
(d/args.output_name).write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({'longest_seconds':{k:v['longest_seconds'] for k,v in s.items()},'isolated_soft_failures':result['isolated_soft_1_or_2_frame_failures'],'healthy_not_ready_packets':result['instant_healthy_but_not_joint_packets']},indent=2))
