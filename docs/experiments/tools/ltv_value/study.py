"""Resumable fixed study stages. Stage advancement is explicit and evidenced."""
import argparse,datetime,json
import numpy as np
from common import *
from run import run,identity
from analyze import selected,summarize,cache,freeze_strata,qualification
from evaluate import evaluate
from audit import audit

def safe_run(seq,mode,candidate='C0',stage='final',repeat=False):
 r=run(seq,mode,candidate,stage=stage,repeat=repeat)
 audit(r)
 if r['status']!='complete':raise RuntimeError('execution failure requires diagnosis: '+r['path'])
 return r

def diagnostic():
 for seq in EUROC+DEV:
  records={m:safe_run(seq,m,stage='passive') for m in ['B','P']}
  exact={f:sha(Path(records['B']['path'])/f)==sha(Path(records['P']['path'])/f) for f in ['trajectory.csv','audit.csv']}
  if not all(exact.values()):raise RuntimeError('Passive corruption '+seq)
 freeze_strata();results=[summarize(selected(s)) for s in EUROC+DEV]
 q=qualification([x for x in results if x['sequence'] in DEV]);print({k:v['qualified'] for k,v in q.items()},flush=True)

def freeze():
 manifest=read(OUT/'experiment_manifest.json')
 if manifest.get('weights_frozen'):print('already frozen',manifest['selected_candidate']);return
 q=read(OUT/'qualification.json');allowed=[]
 if q['G']['qualified']:allowed.append('C1')
 if q['V']['qualified']:allowed.append('C2')
 if q['G']['qualified'] and q['V']['qualified']:allowed.append('C3')
 results={}
 for candidate in ['C0']+allowed:
  results[candidate]={}
  for seq in DEV:
   r=safe_run(seq,'GV',candidate,stage='candidate');results[candidate][seq]={'metrics':evaluate(r),'mechanism':summarize(r),'run':r['id']}
 decisions={};eligible=[]
 for candidate in allowed:
  reasons=[];gains=[];target=['G'] if candidate=='C1' else ['V'] if candidate=='C2' else ['G','V']
  wins={b:0 for b in target}
  for seq in DEV:
   old=results['C0'][seq];new=results[candidate][seq]
   if new['metrics']['status']!='VALID' or new['metrics'].get('localization_status')!='VALID': reasons.append(seq+':invalid_localization');continue
   if new['metrics']['coverage']<old['metrics']['coverage']:reasons.append(seq+':coverage_decreased')
   for branch in ['G','V']:
    a=old['mechanism']['strata']['all'][branch]['actual_total'];b=new['mechanism']['strata']['all'][branch]['actual_total']
    if b['method_p95']>1.1*a['method_p95']:reasons.append(seq+':'+branch+'_P95_harm')
    if branch in target:
     gain=1-b['method_rmse']/max(a['method_rmse'],1e-12);gains.append(gain);wins[branch]+=gain>0
  if any(v<2 for v in wins.values()):reasons.append('less_than_2_of_3_target_improved')
  mean=lambda c:np.mean([results[c][s]['metrics'].get('ate_rmse_m',float('inf')) for s in DEV])
  if mean(candidate)>mean('C0'):reasons.append('ATE_worse_than_C0')
  score=float(np.mean(gains)) if gains else -1.
  decisions[candidate]={'rejected_reasons':reasons,'mean_target_relative_gain':score,'target_wins':wins}
  if not reasons:eligible.append((len(target),-score,mean(candidate),candidate))
 chosen=sorted(eligible)[0][-1] if eligible else 'C0'
 manifest.update(weights_frozen=True,selected_candidate=chosen,validation_unsealed=True,frozen_at=datetime.datetime.now(datetime.timezone.utc).isoformat(),frozen_identity=identity(),candidate_decisions=decisions,extra_candidates=allowed)
 write(OUT/'candidate_results.json',results);write(OUT/'weight_freeze.json',{'selected':chosen,'qualification':q,'decisions':decisions,'extra_candidates':allowed,'timestamp':manifest['frozen_at']});write(OUT/'experiment_manifest.json',manifest)
 print('FROZEN',chosen,'extra candidates',allowed,flush=True)

def final():
 manifest=read(OUT/'experiment_manifest.json');candidate=manifest['selected_candidate']
 paths={}
 for seq in SEQUENCES:
  paths[seq]={}
  for mode in MODES:
   # Sigma is inactive in B/P; prior C0 records remain immutable diagnostic evidence.
   if mode in ['B','P']:
    found=[r for r in latest_runs().values() if r['sequence']==seq and r['mode']==mode and not r['short'] and r['status']=='complete']
    r=found[0] if found else safe_run(seq,mode,candidate)
   else:r=safe_run(seq,mode,candidate)
   paths[seq][mode]=r['id']
   audit(r)
  for f in ['trajectory.csv','audit.csv']:
   records=latest_runs();assert sha(Path(records[paths[seq]['B']]['path'])/f)==sha(Path(records[paths[seq]['P']]['path'])/f),(seq,f)
  write(OUT/'final_paths.json',paths)
  for mode in MODES:
   r=latest_runs()[paths[seq][mode]];evaluate(r);summarize(r)
  print('FINAL COMPLETE',seq,flush=True)

def repeats():
 paths=read(OUT/'final_paths.json');records=latest_runs();candidate=read(OUT/'experiment_manifest.json')['selected_candidate']
 scores=[];failures=[]
 for seq in VALIDATION:
  b=evaluate(records[paths[seq]['B']]);v=evaluate(records[paths[seq]['GV']])
  if b['status']!='VALID' or v['status']!='VALID' or b.get('localization_status')!='VALID' or v.get('localization_status')!='VALID':failures.append(seq)
  else:scores.append((1-v['ate_rmse_m']/b['ate_rmse_m'],seq))
 order=['V1_01_easy']
 if scores:order.append(max(scores)[1])
 if failures:order.append(failures[0])
 elif scores:
  order.extend(s for _,s in sorted(scores) if s not in order)
 order=list(dict.fromkeys(order))
 for seq in VALIDATION:
  if len(order)>=3:break
  if seq not in order:order.append(seq)
 order=order[:3];write(OUT/'repeat_sequences.json',order);result={}
 for seq in order:
  result[seq]={}
  for mode in ['B','G','V','GV']:
   found=[r for r in latest_runs().values() if r['sequence']==seq and r['mode']==mode and r['stage']=='repeat' and r['status']=='complete']
   r=found[0] if found else safe_run(seq,mode,candidate,stage='repeat',repeat=True)
   first=records[paths[seq][mode]];m=evaluate(r);baseline=evaluate(first)
   result[seq][mode]={'first':first['id'],'repeat':r['id'],'trajectory_exact':sha(Path(first['path'])/'trajectory.csv')==sha(Path(r['path'])/'trajectory.csv'),'audit_exact':sha(Path(first['path'])/'audit.csv')==sha(Path(r['path'])/'audit.csv'),'first_ate':baseline.get('ate_rmse_m'),'repeat_ate':m.get('ate_rmse_m')}
   write(OUT/'repeat_summary.json',result)
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('stage',choices=['diagnostic','freeze','final','repeats']);args=p.parse_args()
 globals()[args.stage]()
