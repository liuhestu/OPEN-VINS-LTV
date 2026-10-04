"""Read saved local-kernel result; independently recompute algebraic contributions."""
import json,sys,hashlib
from pathlib import Path
import numpy as np
root=Path('/home/he/output/ltv_passive_hardening_v2');d=root/'diagnostics/R3B_local_contributions_01'
rows=[json.loads(s) for s in (d/'events.jsonl').read_text().splitlines()];summary=json.loads((d/'summary.json').read_text());assert summary['status']=='ALIGNED_LOCAL_RECONSTRUCTION' and len(rows)==42
features=root/'real_development/R3B_grace_V2_03_01/P_NEW/features.jsonl';logs=[json.loads(s) for s in features.read_text().splitlines()];lookup={r['camera_ns']:r for r in logs};assert len(lookup)==len(logs)
assert len({r['camera_ns']*1e-9 for r in logs})==len(logs)
pointreport=json.loads((root/'diagnostics/R3B_point_pre_post_01/summary.json').read_text());points={}
for w in pointreport['windows'].values():
 for p in w['packets_detail']:points[p['camera_ns']]=p
maxsum=maxblock=maxdirection=0.;targets=[]
for r in rows:
 ns=r['camera_ns'];f=lookup[ns];assert ns*1e-9==r['time'] and f['target_imu_time']==r['time'] and f['epoch']==r['epoch']
 assert len(set(r['slots']))==len(r['slots']) and set(r['slots'])==set(f['retained_ids'])
 assert len(set(r['point_ids']))==len(r['point_ids']) and set(r['point_ids'])==set(f['corrected_ids'])
 assert r['substeps']==r['recorded_substeps']==f['camera_substeps']
 assert max(r['x_relative_error'],r['P_relative_error'])<=1e-9 and r['delta_absolute_error']<=1e-9
 terms=np.array(r['point_shared_contributions']);total=np.array(r['total_shared_delta']);check=np.zeros_like(terms)
 for s in r['substep_details']:
  gain=np.array(s['shared_gain']);innovation=np.array(s['projection_innovation']);st=np.array(s['point_shared_terms'])
  for j in range(len(st)):
   recompute=gain[:,3*j:3*j+3]@innovation[3*j:3*j+3]
   maxblock=max(maxblock,float(np.max(abs(st[j]-recompute))));check[j]+=recompute
  maxsum=max(maxsum,float(np.max(abs(gain@innovation-np.array(s['delta_shared'])))))
 maxsum=max(maxsum,float(np.max(abs(check-terms))),float(np.max(abs(terms.sum(axis=0)-total))))
 change=np.array(r['reconstructed_post_x'])[-6:]-np.array(r['after_lifecycle_x'])[-6:]
 maxsum=max(maxsum,float(np.max(abs(change-total))))
 dv=total[:3];norm=float(np.linalg.norm(dv));axis=dv/norm
 fractions=terms[:,:3]@axis/norm
 maxdirection=max(maxdirection,abs(float(fractions.sum())-1))
 if ns in (1413394964105760512,1413394966255760384):
  angles={p['id']:p for p in points[ns]['points']};largest=max((p for p in angles.values() if p['pre']['angle_rad'] is not None),key=lambda p:p['pre']['angle_rad'])
  ranked=[]
  for j in np.argsort(-np.linalg.norm(terms[:,:3],axis=1)):
   fid=r['point_ids'][j];ranked.append({'id':fid,'delta_v_norm':float(np.linalg.norm(terms[j,:3])),'projection_along_total_delta_v':float(terms[j,:3]@axis),'fraction_along_total':float(fractions[j]),'rest_fraction':float(1-fractions[j]),'pre_angle_rad':angles[fid]['pre']['angle_rad'],'post_angle_rad':angles[fid]['post']['angle_rad'],'delta_shared':terms[j].tolist()})
  targets.append({'camera_ns':ns,'delta_v_norm':norm,'total_shared_delta':total.tolist(),'largest_pre_angle_id':largest['id'],'largest_pre_angle_rad':largest['pre']['angle_rad'],'points_by_v_contribution_norm':ranked})
assert max(maxsum,maxblock,maxdirection)<=1e-9
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
result={'status':'PASS_LOCAL_ALGEBRA_ONLY','frames':42,'max_point_block_product_error':maxblock,'max_sum_telescope_error':maxsum,'max_projected_fraction_sum_error':maxdirection,'max_x_relative_error':max(r['x_relative_error'] for r in rows),'max_P_relative_error':max(r['P_relative_error'] for r in rows),'max_delta_absolute_error':max(r['delta_absolute_error'] for r in rows),'spectral_floor_clamped_values':sum(r['spectral_protection']['floor_clamped_values'] for r in rows),'targets':targets,'provenance':{str(p):sha(p) for p in (Path(__file__),d/'events.jsonl',d/'summary.json',features)},'limits':['Only42 events in two fixed windows; each resets to actual preceding cached x/P.','Actual-path additive terms, not deleting-point counterfactual.','This audit checks serialized slot identity sets and corrected IDs; final x/P numerical oracle was evaluated by source reconstruction.','Current float-time key uniqueness verified; not arbitrary nanosecond mapping assurance.']}
(d/'independent_contribution_audit.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps({**{k:v for k,v in result.items() if k not in ('targets','provenance')},'targets':[{**{k:v for k,v in t.items() if k!='points_by_v_contribution_norm'},'top3':t['points_by_v_contribution_norm'][:3]} for t in targets]},indent=2))
