"""Offline unchanged seed/recovery/accuracy evaluator with R4 provenance."""
import argparse,importlib.util,json
from pathlib import Path
import budget
spec=importlib.util.spec_from_file_location('_r4_frozen_synthetic_evaluation',budget.ROOT/'scripts/ltv_passive_hardening/synthetic_evaluate.py')
base=importlib.util.module_from_spec(spec);spec.loader.exec_module(base)
base.DOC=budget.DOC
if __name__=='__main__':
 p=argparse.ArgumentParser()
 for name in ('input-dir','run-dir','out'):p.add_argument('--'+name,required=True)
 p.add_argument('--baseline');a=p.parse_args();r=base.evaluate(**vars(a))
 r['provenance']['R4_evaluator_wrapper_sha']=base.sha(__file__)
 # The original evaluator writes metrics.json; add the wrapper identity explicitly.
 files=list(Path(a.out).glob('*.json'))
 for path in files:
  value=json.loads(path.read_text())
  if isinstance(value,dict) and 'provenance' in value:
   value['provenance']['R4_evaluator_wrapper_sha']=base.sha(__file__)
   path.write_text(json.dumps(value,indent=2,allow_nan=False)+'\n')
 print(json.dumps({'status':r['status'],'output':a.out}))
