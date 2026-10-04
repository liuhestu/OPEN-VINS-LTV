"""Two-worker frozen native confirmation; each child reserves its own real attempt."""
import argparse,concurrent.futures,json,subprocess,sys
from pathlib import Path
import budget,artifacts

def main(prefix):
 frozen=json.loads((budget.DOC/'frozen_config.json').read_text())
 summary=budget.OUT/(prefix+'_confirmation_jobs.json')
 if summary.exists():raise FileExistsError('existing supervisor evidence; do not restart')
 jobs=[]
 for sequence in frozen['sequences']:
  name=prefix+'_'+sequence
  if (budget.OUT/'real_confirmation'/name).exists():raise FileExistsError('existing child attempt '+name)
  jobs.append((sequence,name))
 def run(job):
  sequence,name=job
  command=[sys.executable,str(Path(__file__).with_name('real_run.py')),name,sequence,
   '--phase','confirmation','--enabled','--angle',str(frozen['angle']),
   '--config',frozen['base_config'],'--runtime',frozen['runtime']]
  logs=budget.OUT/'confirmation_supervisor';logs.mkdir(exist_ok=True)
  with (logs/(name+'.stdout')).open('w') as out,(logs/(name+'.stderr')).open('w') as err:
   code=subprocess.run(command,stdout=out,stderr=err).returncode
  return dict(sequence=sequence,name=name,command=command,exit_code=code)
 results=[]
 with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
  futures=[pool.submit(run,j) for j in jobs]
  for future in concurrent.futures.as_completed(futures):
   results.append(future.result())
   summary.write_text(json.dumps(dict(freeze_sha=artifacts.sha(budget.DOC/'frozen_config.json'),results=results,complete=len(results)==len(jobs)),indent=2)+'\n')
   print(json.dumps(results[-1]),flush=True)
 if any(r['exit_code'] for r in results):raise RuntimeError('failed confirmation preserved; no automatic retries')
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('prefix');main(p.parse_args().prefix)
