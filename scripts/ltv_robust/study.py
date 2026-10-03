"""Bounded, separately frozen all-EuRoC in-sample robust-fusion comparison."""
import concurrent.futures,hashlib,json,os,re,shutil,subprocess,sys,time
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[2]
OUT=Path('/home/he/output/openvins_ltv_robust_fusion_20261003')
OLD=Path('/home/he/output/openvins_ltv_value_study_20261003')
ASL=Path('/home/he/datasets/euroc/ASL')
sys.path.insert(0,str(ROOT/'scripts/ltv'))
from evaluate_sequence import evaluate,self_test,nearest
from audit_run import audit
SEQS=sorted(p.name for p in ASL.iterdir() if p.is_dir() and p.name!='MH_04_difficult')
CANDIDATES={'B':(False,False,1),'raw':(False,False,1),'Q':(True,False,1),'H':(False,True,1),'QH':(True,True,1),'QH_strong':(True,True,.5),'QH_weak':(True,True,2)}
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def write(p,x):p.write_text(json.dumps(x,indent=2))
def identity():
 return {str(p):sha(p) for p in [OUT/'install/ov_msckf/lib/ov_msckf/run_ltv_euroc',OUT/'install/ov_msckf/lib/libov_msckf_lib.so',OLD/'install/ov_core/lib/libov_core_lib.so',OLD/'install/ov_init/lib/libov_init_lib.so',Path(__file__),ROOT/'scripts/ltv/evaluate_sequence.py',ROOT/'scripts/ltv/audit_run.py']}
def prepare():
 self_test();OUT.mkdir(exist_ok=True)
 manifest={'scope':'10 EuRoC sequences, MH04 excluded per prior user instruction; in-sample selection, no held-out claim','sequences':SEQS,'candidates':CANDIDATES,'selection':'minimum mean per-sequence relative ATE increase vs B; all 10 engineering valid; all candidates reported','quality':{'minimum_features':25,'eta_error':.2,'innovation':.03,'velocity_disagreement':.5},'huber_delta':2,'original_nis_gate':True,'full_start_limit':80,'workers':4,'identity':identity(),'source_head':subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),'inputs':{s:{str(p):sha(p) for p in (ASL/s/'mav0').glob('*/data.csv')} for s in SEQS},'base_config':{str(p):sha(p) for p in (ROOT/'config/ltv_euroc').glob('*.yaml')}}
 p=OUT/'protocol.json'
 if p.exists():
  old=json.loads(p.read_text());assert old==manifest,'frozen protocol changed'
 else:write(p,manifest)
 return manifest

def run(seq,name,repeat=False):
 p=OUT/'runs'/(seq+'-'+name+('-repeat' if repeat else ''));m=p/'manifest.json'
 if m.exists():
  r=json.loads(m.read_text());assert r['status']=='complete','previous failure/running attempt must be inspected'
  return r
 p.mkdir(parents=True,exist_ok=False)
 q,h,k=CANDIDATES[name];enabled=name!='B'
 values={'ltv_enabled':enabled,'ltv_enable_gravity':enabled,'ltv_enable_velocity':enabled,'ltv_enable_quality_gate':q,'ltv_enable_huber':h,'ltv_huber_delta':2.,'ltv_quality_eta_error':.2,'ltv_quality_innovation':.03,'ltv_quality_min_features':25,'ltv_quality_velocity_disagreement':.5,'ltv_sigma_gravity_deg':10*k,'ltv_sigma_velocity_mps':k,'ltv_log_path':str(p/'ltv.csv'),'ltv_value_diagnostics_enabled':False}
 cfg=(ROOT/'config/ltv_euroc/estimator_config.yaml').read_text()
 for key,value in values.items():
  line=key+': '+json.dumps(value);cfg=re.sub(r'^'+key+r':.*$',line,cfg,flags=re.M) if re.search(r'^'+key+':',cfg,re.M) else cfg+'\n'+line+'\n'
 (p/'estimator_config.yaml').write_text(cfg)
 for file in ['kalibr_imu_chain.yaml','kalibr_imucam_chain.yaml']:shutil.copyfile(ROOT/'config/ltv_euroc'/file,p/file)
 command=['bash','-c','source /opt/ros/humble/setup.bash && source "$1/install/setup.bash" && source "$2/install/setup.bash" && exec "$2/install/ov_msckf/lib/ov_msckf/run_ltv_euroc" "$3" "$4" "$5" 0','robust',str(OLD),str(OUT),str(p/'estimator_config.yaml'),str(ASL/seq/'mav0'),str(p)]
 r={'sequence':seq,'candidate':name,'mode':'GV' if enabled else 'B','path':str(p),'command':command,'status':'running','full':True,'config_sha':sha(p/'estimator_config.yaml'),'repeat':repeat};write(m,r)
 print('START',seq,name,repeat,flush=True);start=time.monotonic()
 with (p/'stdout.log').open('w') as o,(p/'stderr.log').open('w') as e:
  proc=subprocess.Popen(command,stdout=o,stderr=e);r['pid']=proc.pid;write(m,r);code=proc.wait()
 r.update(exit_code=code,runtime_seconds=time.monotonic()-start,status='failed');write(m,r)
 if code:raise RuntimeError('replay failed '+str(p))
 replay=json.loads((p/'replay.json').read_text());assert replay['complete'] and replay['output_rows']>=3
 effective=json.loads((p/'effective_options.json').read_text())
 assert all(effective[key]==v for key,v in values.items() if key in effective)
 assert effective['calib_camimu_dt']==0
 r['status']='complete';write(m,r)
 try:
  a=audit(p)
  if enabled:
   x=np.genfromtxt(p/'ltv.csv',delimiter=',',names=True,dtype=None,encoding='utf8')
   for branch,variance in [('G',np.deg2rad(10*k)**2),('V',k*k)]:
    mask=x[branch+'_rows']>0;w=variance/x[branch+'_variance'][mask]
    a[branch]['huber_weight']={'n':len(w),'below_one':int(sum(w<1-1e-12)),'min':float(w.min()) if len(w) else None,'mean':float(w.mean()) if len(w) else None}
  write(p/'audit_results.json',a)
 except Exception:
  r['status']='failed_audit';write(m,r);raise
 print('END',seq,name,round(r['runtime_seconds'],2),flush=True);return r

def main():
 protocol=prepare()
 jobs=[(s,n) for n in CANDIDATES for s in SEQS]
 with concurrent.futures.ThreadPoolExecutor(max_workers=4) as pool:
  for offset in range(0,len(jobs),4):
   futures=[pool.submit(run,*job) for job in jobs[offset:offset+4]]
   for future in concurrent.futures.as_completed(futures):future.result()
 results={s:evaluate(s,{n:str(OUT/'runs'/(s+'-'+n)) for n in CANDIDATES}) for s in SEQS}
 write(OUT/'metrics.json',results)
 scores={}
 for n in CANDIDATES:
  if n=='B':continue
  if all(results[s]['modes'][n]['status']=='VALID' and results[s]['modes']['B']['status']=='VALID' for s in SEQS):
   change=[100*(results[s]['modes'][n]['ate_rmse_m']/results[s]['modes']['B']['ate_rmse_m']-1) for s in SEQS]
   scores[n]={'mean_relative_ATE_percent':float(np.mean(change)),'median_relative_ATE_percent':float(np.median(change)),'worst_relative_ATE_percent':max(change),'per_sequence_percent':dict(zip(SEQS,change))}
 if not scores:raise RuntimeError('no eligible candidate')
 best=min(scores,key=lambda n:scores[n]['mean_relative_ATE_percent']);write(OUT/'selection.json',{'best':best,'scores':scores,'in_sample':True})
 with concurrent.futures.ThreadPoolExecutor(max_workers=4) as pool:
  for r in pool.map(lambda s:run(s,best,True),SEQS):
   p=Path(r['path']);original=OUT/'runs'/(r['sequence']+'-'+best)
   matches={n:sha(p/n)==sha(original/n) for n in ['trajectory.csv','audit.csv']};write(p/'repeat_check.json',matches);assert all(matches.values())
 print('COMPLETE',best,flush=True)
if __name__=='__main__':main()
