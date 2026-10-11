"""Exact event neighborhood audit; no policy simulation or observer replay."""
import json, hashlib, sys
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from real_evaluate import physical
root=Path('/home/he/output/ltv_passive_hardening_v2')
d=root/'real_development/R3B_grace_V2_03_01/evaluation_v1'
p=root/'real_development/R3_history_compat_V2_03_01/evaluation_v1'
m=json.loads((d/'metrics.json').read_text());a=dict(np.load(d/'error_curves.npz'));b=dict(np.load(p/'error_curves.npz'))
assert np.array_equal(a['camera_ns'],b['camera_ns']) and np.array_equal(a['time'],b['time'])
for q in ('v','eta'):
 assert np.array_equal(a['P_NEW_'+q],a['P_PREV_'+q],equal_nan=True)
 assert np.array_equal(a['P_NEW_'+q],b['P_NEW_'+q],equal_nan=True)
rows,lookup=physical.read_features(Path(m['provenance']['runs']['P_NEW'])/'features.jsonl')
gtpath=Path(m['reference']['path']);gt=physical.load_gt('V2_03_difficult',gtpath)
ref=physical.physical_reference(gt,a['time'])
for q in ('v','eta'):assert np.array_equal(ref[q],a['reference_'+q],equal_nan=True)
t,pgt,qgt,vgt,g=gt
origin=int(a['camera_ns'][0]);rel=a['camera_ns']-origin
keys=['ready_G','ready_V','joint_ready','reason','availability_state','raw_current','pool_ready','observer_valid','observed_features','mature_features','prediction_angle_p95_rad','velocity_correction_rate','gravity_correction_rate','grace_G','grace_V','actual_corrections','epoch','sequence']
selected=[]
for i in np.flatnonzero((rel>=76450000000)&(rel<=79150000000)):
 ns=int(a['camera_ns'][i]);r=lookup[ns];u=float(a['time'][i]);assert r['target_imu_time']==u
 k=int(np.searchsorted(t,u).clip(1,len(t)-1));lo=k-1
 item={'camera_ns':ns,'physical_time':u,'relative_camera_seconds':int(rel[i])/1e9,'relative_physical_seconds':u-float(a['time'][0]),'index':int(i),'previous_dt':u-float(a['time'][i-1]),'health':{k:r.get(k) for k in keys},'errors':{name:{q:float(a[name+'_error_'+q][i]) for q in ('v','eta','g_angle_deg')} for name in ('P_NEW','P_PREV','OpenVINS')},'v_body':{name:a[name+'_v'][i].tolist() for name in ('P_NEW','P_PREV','OpenVINS')},'reference_v_body':ref['v'][i].tolist(),'reference_valid':bool(ref['pose_valid'][i]),'gt_bracket':{'lo_index':lo,'hi_index':k,'time':[float(t[lo]),float(t[k])],'gap_seconds':float(t[k]-t[lo]),'fraction':float((u-t[lo])/(t[k]-t[lo])),'v_world':vgt[[lo,k]].tolist(),'quaternion_xyzw':qgt[[lo,k]].tolist(),'velocity_endpoint_change_norm':float(np.linalg.norm(vgt[k]-vgt[lo]))}}
 selected.append(item)
anchors=[]
# Display labels resolve to these explicit observed receipts; all joins below are exact IDs.
for offset,receipt in ((76750000000,1413394964105760512),(78900000000,1413394966255760384)):
 ix=np.flatnonzero(a['camera_ns']==receipt);assert len(ix)==1, 'Exact receipt absent'
 i=int(ix[0]);anchors.append({'requested_relative_seconds':offset/1e9,'camera_ns':int(a['camera_ns'][i]),'previous_camera_ns':int(a['camera_ns'][i-1]),'previous_relative_seconds':int(rel[i-1])/1e9,'previous_ltv_v_error':float(a['P_NEW_error_v'][i-1]),'ltv_v_error':float(a['P_NEW_error_v'][i]),'openvins_v_error':float(a['OpenVINS_error_v'][i])})
out=root/'diagnostics/R3B_exact_local_error_01';out.mkdir(exist_ok=False)
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
result={'anchors':anchors,'whole_initialized_raw_equal_P_PREV_R3_history_compat_R3B':True,'reference_recomputed_exact':True,'initialized_events':len(rel),'neighborhood':selected,'provenance':{str(x):sha(x) for x in (Path(__file__),d/'error_curves.npz',p/'error_curves.npz',d/'metrics.json',gtpath)},'limitations':['Official EuRoC world velocity is linearly interpolated and quaternion SLERP rotates it into body frame. Valid support does not independently certify physical GT accuracy.','C0 denotes unchanged observer gains, not a demonstrated equality with the old zero-seeded P_OLD baseline.','No candidate ready masks or counterfactual observer outputs generated.']}
(out/'audit.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
print(json.dumps({'output':str(out),'anchors':anchors,'rows':len(selected),'valid_reference_rows':sum(r['reference_valid'] for r in selected),'max_gt_bracket_gap':max(r['gt_bracket']['gap_seconds'] for r in selected),'max_gt_endpoint_v_change':max(r['gt_bracket']['velocity_endpoint_change_norm'] for r in selected)},indent=2))
