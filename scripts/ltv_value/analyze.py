"""Frozen value-study mechanism analysis. Held-out errors cannot be read before weight freeze."""
import argparse,gzip,json,itertools
import numpy as np
from scipy.ndimage import uniform_filter1d
from reference import *
from statistics import paired,dependence,windows

def guard(seq):
 if seq in VALIDATION and not read(OUT/'experiment_manifest.json').get('weights_frozen'):
  raise RuntimeError('validation effects remain sealed')
def cache(run):
 seq=run['sequence'];guard(seq);p=Path(run['path']);dest=p/'analysis_arrays.npz'
 if dest.exists(): return dict(np.load(dest))
 values={k:[] for k in ['t','camera_time','available','valid','velocity_valid','gravity_valid','visual_rows','features','observed','prior','posterior','fej','ltv_v','ltv_g','shadow_visual','shadow_G','shadow_V','shadow_GV','G_accepted','V_accepted','GV_accepted','observer_seconds','msckf_seconds_without_diagnostics','diagnostic_compute_seconds']}
 with gzip.open(p/'value.jsonl.gz','rt') as f:
  for line in f:
   x=json.loads(line)
   for k in values:values[k].append(x['imu_time'] if k=='t' else np.asarray(x[k]).squeeze())
 a={k:np.array(v) for k,v in values.items()};ref=reference(seq,a['t'])
 for k,v in ref.items():a['gt_'+k]=v
 for w in [.05,.2]:a[f'gt_v_body_{w}']=reference(seq,a['t'],w)['v_body']
 for name in ['prior','posterior','shadow_visual','shadow_G','shadow_V','shadow_GV']:
  v,g=body_estimate(a[name]);a[name+'_v']=v;a[name+'_g']=g
  a[name+'_V_error']=np.linalg.norm(v-ref['v_body'],axis=1)
  a[name+'_G_error']=angle(g,ref['g_body'])
 for branch,key in [('V','v'),('G','g')]:
  a['ltv_'+branch+'_error']=np.linalg.norm(a['ltv_v']-ref['v_body'],axis=1) if branch=='V' else angle(a['ltv_g'],ref['g_body'])
  mask=a['available'].astype(bool)&a['valid'].astype(bool)&a['velocity_valid' if branch=='V' else 'gravity_valid'].astype(bool)
  a['ltv_'+branch+'_error'][~mask]=np.nan
 actual='shadow_visual' if run['mode'] in ['B','P'] else 'shadow_'+run['mode']
 err=np.linalg.norm(a['posterior']-a[actual],axis=1)
 relative=err/np.maximum(1,np.linalg.norm(a['posterior'],axis=1))
 if relative.max()>1e-9: raise RuntimeError('shadow actual mismatch '+str(relative.max()))
 a['shadow_actual_relative_error']=relative
 # Motion labels derive only from raw IMU and baseline visual rows.
 inp=read(OUT/'inputs.json')[seq];imu=np.loadtxt(Path(inp['replay_root'])/'imu0/data.csv',delimiter=',',comments='#');imu[:,0]*=1e-9
 n=max(1,int(round(1/np.median(np.diff(imu[:,0])))))
 g=9.81 if seq in EUROC else 9.8065
 omega=np.sqrt(uniform_filter1d(np.sum(imu[:,1:4]**2,axis=1),n,mode='nearest'))
 force=np.sqrt(uniform_filter1d((np.linalg.norm(imu[:,4:7],axis=1)-g)**2,n,mode='nearest'))
 a['omega_rms']=np.interp(a['t'],imu[:,0],omega);a['force_rms']=np.interp(a['t'],imu[:,0],force)
 np.savez_compressed(dest,**a);return a

def selected(seq,mode='P',candidate='C0'):
 records=[r for r in latest_runs().values() if r['sequence']==seq and r['mode']==mode and r['candidate']==candidate and not r['short'] and r['status']=='complete']
 if not records:raise ValueError('no completed run '+seq+' '+mode)
 return records[0]
def freeze_strata():
 if (OUT/'strata.json').exists():return read(OUT/'strata.json')
 result={}
 for name,seqs in [('euroc',EUROC),('uzh',DEV)]:
  arrays=[cache(selected(s)) for s in seqs]
  # Each sequence contributes its own distribution; pooled timestamps, thresholds fixed before validation.
  omega=np.concatenate([a['omega_rms'][a['gt_pose_valid']] for a in arrays]);force=np.concatenate([a['force_rms'][a['gt_pose_valid']] for a in arrays])
  visual=np.concatenate([a['visual_rows'][(a['visual_rows']>0)&a['gt_pose_valid']] for a in arrays])
  result[name]={'omega_p75':float(np.quantile(omega,.75)),'force_p75':float(np.quantile(force,.75)),'visual_positive_p25':float(np.quantile(visual,.25)) if len(visual) else 0.,'sequences':seqs}
 write(OUT/'strata.json',result);return result

def labels(a,seq):
 threshold=read(OUT/'strata.json')['euroc' if seq in EUROC else 'uzh']
 # The mask timeline always follows Passive (byte-identical to B), even in active modes.
 baseline=cache(selected(seq));j=np.searchsorted(baseline['t'],a['t']).clip(0,len(baseline['t'])-1)
 same=np.abs(baseline['t'][j]-a['t'])<=1e-6;vr=baseline['visual_rows'][j]
 masks={'all':same,'high_rotation':same&(a['omega_rms']>=threshold['omega_p75']),'high_force':same&(a['force_rms']>=threshold['force_p75']),'zero_visual':same&(vr==0),'weak_visual':same&(vr>0)&(vr<=threshold['visual_positive_p25'])}
 for x,y in itertools.combinations(['high_rotation','high_force','zero_visual','weak_visual'],2):
  if {x,y}=={'zero_visual','weak_visual'}:continue
  masks[x+'__'+y]=masks[x]&masks[y]
 return masks

def summarize(run):
 a=cache(run);seq=run['sequence'];masks=labels(a,seq)
 result={'sequence':seq,'mode':run['mode'],'candidate':run['candidate'],'events':len(a['t']),'pose_reference_fraction':float(a['gt_pose_valid'].mean()),'velocity_reference_fraction':float(a['gt_v_valid'].mean()),'shadow_native_max_relative_error':float(a['shadow_actual_relative_error'].max()),'strata':{},'correlation':{},'intervals':{},'timing':{k:float(a[k].sum()) for k in ['observer_seconds','msckf_seconds_without_diagnostics','diagnostic_compute_seconds']}}
 for name,m in masks.items():
  item={}
  for branch in ['G','V']:
   own=paired(a['t'][m],a['prior_'+branch+'_error'][m],a['ltv_'+branch+'_error'][m])
   incremental=paired(a['t'][m],a['shadow_visual_'+branch+'_error'][m],a['shadow_'+branch+'_'+branch+'_error'][m])
   dt=float(np.median(np.diff(a['t'])))
   own['seconds']=own['n']*dt;incremental['seconds']=incremental['n']*dt
   own['support_total_events']=int(m.sum());own['effective_fraction']=float(own['n']/max(1,m.sum()))
   item[branch]={'own':own,'incremental':incremental,'GV_incremental':paired(a['t'][m],a['shadow_visual_'+branch+'_error'][m],a['shadow_GV_'+branch+'_error'][m]),'actual_total':paired(a['t'][m],a['prior_'+branch+'_error'][m],a['posterior_'+branch+'_error'][m])}
   if branch=='V':
    sensitivity={}
    for w in [.05,.1,.2]:
     gt=a['gt_v_body'] if w==.1 else a[f'gt_v_body_{w}'];ok=m&np.isfinite(a['ltv_V_error'])
     sensitivity[str(w)]={'own':paired(a['t'][ok],np.linalg.norm(a['prior_v'][ok]-gt[ok],axis=1),np.linalg.norm(a['ltv_v'][ok]-gt[ok],axis=1)), 'incremental':paired(a['t'][m],np.linalg.norm(a['shadow_visual_v'][m]-gt[m],axis=1),np.linalg.norm(a['shadow_V_v'][m]-gt[m],axis=1))}
    item[branch]['sensitivity']=sensitivity
  result['strata'][name]=item
 for branch in ['G','V']:
  ok=np.isfinite(a['ltv_'+branch+'_error']);gt=a['gt_g_body'] if branch=='G' else a['gt_v_body']
  if branch=='V':e=a['prior_v'][ok]-gt[ok];f=a['ltv_v'][ok]-gt[ok]
  else:e=tangent_error(a['prior_g'][ok],gt[ok]);f=tangent_error(a['ltv_g'][ok],gt[ok])
  result['correlation'][branch]=dependence(e,f)
  result['intervals'][branch]=windows(a['t'],a['prior_'+branch+'_error'],a['ltv_'+branch+'_error'])
 write(Path(run['path'])/'mechanism.json',result);return result

def qualification(results):
 """Predeclared same-stratum two-of-three development gate; never ATE-driven."""
 out={}
 for branch in ['G','V']:
  by_stratum={}
  for name in results[0]['strata']:
   passes=[];details={}
   for r in results:
    d=r['strata'][name][branch];inc=d['incremental'];own=d['own'];reasons=[]
    def improves(z):return z.get('seconds',0)>=5 and z.get('improvement') is not None and z['improvement']>=.05 and z['ci95'][0]>0
    win=improves(own) or improves(inc)
    if not win:reasons.append('no_stable_5pct_advantage')
    if inc.get('method_p95',float('inf'))>1.1*inc.get('baseline_p95',0):reasons.append('shadow_p95_harm')
    if branch=='V':
     which='own' if improves(own) else 'incremental'
     if any(z[which].get('improvement') is None or z[which]['improvement']<=0 for z in d['sensitivity'].values()):reasons.append('velocity_derivative_sensitive')
    passed=not reasons
    if passed:passes.append(r['sequence'])
    details[r['sequence']]={'pass':passed,'reasons':reasons}
   by_stratum[name]={'passes':passes,'details':details}
  out[branch]={'qualified':any(len(d['passes'])>=2 for d in by_stratum.values()),'strata':by_stratum}
 write(OUT/'qualification.json',out);return out
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--development',action='store_true');a=p.parse_args()
 if a.development:
  freeze_strata();results=[summarize(selected(s)) for s in EUROC+DEV]
  qualification([r for r in results if r['sequence'] in DEV]);print(json.dumps(read(OUT/'qualification.json'),indent=2))
