"""Use unchanged physical reference/threshold evaluator with R4-only config controls."""
import argparse,importlib.util,json,sys
from pathlib import Path
import budget
ROOT=budget.ROOT
spec=importlib.util.spec_from_file_location('_r4_prior_real_evaluator',ROOT/'scripts/ltv_passive_hardening/real_evaluate.py')
base=importlib.util.module_from_spec(spec);spec.loader.exec_module(base)
base.DOC=budget.DOC
# Original physical health and all11 contracts remain identical.
original_protocol=json.loads((ROOT/'docs/ltv/passive_hardening_v2/protocol.json').read_text())
base.PROTOCOL=original_protocol
base.CONTRACT=json.loads((budget.DOC/'acceptance.json').read_text())
ACTIVE={'ltv_active_consistency_enabled','ltv_active_consistency_min_history_frames',
 'ltv_active_consistency_min_history_span_s','ltv_active_consistency_max_history_residual_rad',
 'ltv_active_consistency_max_holdout_residual_rad','ltv_active_consistency_retire_after_consecutive_failures'}

def config_equivalence(old_path,new_path):
 old=base.yaml_values(old_path);new=base.yaml_values(new_path)
 if old.get('ltv_active_consistency_enabled',False):raise ValueError('previous method already enabled active consistency')
 if {k:v for k,v in old.items() if k not in ACTIVE}!={k:v for k,v in new.items() if k not in ACTIVE}:
  raise ValueError('Non-R4 configuration changed')
 return dict(old_path=str(old_path),new_path=str(new_path),old_sha=base.sha(old_path),new_sha=base.sha(new_path),
  non_R4_yaml_exact=True,active_controls={k:dict(previous=old.get(k),new=new.get(k)) for k in sorted(ACTIVE)})
base.config_equivalence=config_equivalence

def alias(original,destination,mode,config):
 """New read-only evidence view with explicit identity; original files untouched."""
 original=Path(original).resolve();destination=Path(destination);config=Path(config).resolve()
 if destination.exists():raise FileExistsError(destination)
 identity=base.load(base.identity_path(original))
 # Validate original config tree before constructing a view.
 for name,digest in base.config_identity(identity).items():
  if base.sha(config.parent/name)!=digest:raise ValueError('original configuration identity mismatch')
 destination.mkdir(parents=True)
 for p in original.iterdir():
  if p.name!='identity.json':(destination/p.name).symlink_to(p)
 identity.update(mode=mode,config_path=str(config),alias_origin=str(original),alias_origin_identity_sha=base.sha(base.identity_path(original)))
 (destination/'identity.json').write_text(json.dumps(identity,indent=2)+'\n')
 return destination
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--sequence',required=True);p.add_argument('--new',required=True);p.add_argument('--baseline',required=True);p.add_argument('--previous',required=True);p.add_argument('--out',required=True)
 a=p.parse_args();r=base.evaluate(**vars(a))
 r['provenance']['R4_evaluator_wrapper_sha']=base.sha(__file__)
 (Path(a.out)/'metrics.json').write_text(json.dumps(r,indent=2,allow_nan=False)+'\n')
 print(json.dumps({'status':r['status'],'decisions':r['decisions']},indent=2))
