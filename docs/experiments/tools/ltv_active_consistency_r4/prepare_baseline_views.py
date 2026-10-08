"""Prepare provenance-bound B-only views; never relabel historical P_NEW as R3B."""
import argparse,csv,hashlib,json
from pathlib import Path
SEQUENCES=['MH_01_easy','MH_02_easy','MH_03_medium','MH_04_difficult','MH_05_difficult','V1_01_easy','V1_02_medium','V1_03_difficult','V2_01_easy','V2_02_medium','V2_03_difficult']
def sha(p):
 h=hashlib.sha256()
 with Path(p).open('rb') as f:
  for b in iter(lambda:f.read(1<<20),b''):h.update(b)
 return h.hexdigest()
def load(p):return json.loads(Path(p).read_text())
def prepare(manifest,views,out):
 manifest=Path(manifest).resolve();old=manifest.parent;views=Path(views).resolve();out=Path(out)
 if out.exists():raise FileExistsError(out)
 m=load(manifest);configs={}
 for p in sorted((old/'mode_configs').rglob('estimator_config.yaml')):configs.setdefault(sha(p),[]).append(p)
 identities=list((old/'real_runs').glob('*/identity.json'));results={}
 for seq in SEQUENCES:
  item={'sequence':seq,'status':'UNVERIFIED','candidates':[]};results[seq]=item
  try:
   metricpath=m['real_metrics'].get(seq)
   if metricpath is None:item.update(status='MISSING_METRIC',reason='No sequence entry in named recovered final manifest');continue
   metric=load(metricpath);expected=metric['provenance']['run_files']['B'];item.update(metrics=metricpath,metrics_sha=sha(metricpath))
   for p in identities:
    ident=load(p)
    if ident.get('sequence')==seq and ident.get('mode')=='B':
     item['candidates'].append(dict(path=str(p.parent),identity_sha=sha(p),short_seconds=ident.get('short_seconds'),metric_identity_matches=sha(p)==expected['identity.json']))
   matching=[x for x in item['candidates'] if x['metric_identity_matches']]
   if len(matching)!=1:item.update(status='MISSING_B' if not matching else 'AMBIGUOUS_B',reason='Expected exactly one original B identity bound by metrics');continue
   origin=Path(matching[0]['path']);ident=load(origin/'identity.json')
   if ident.get('short_seconds') is not None:raise ValueError('Selected original baseline is short')
   fileproof={}
   for name,digest in expected.items():
    actual=sha(origin/name)
    if actual!=digest:raise ValueError('Changed baseline file '+name)
    fileproof[name]=actual
   replay=load(origin/'replay.json')
   if not replay['complete'] or replay['camera_packets']!=replay['input_camera_packets'] or replay['actual_G_submissions'] or replay['actual_V_submissions']:raise ValueError('Incomplete/injecting original baseline')
   if not metric['engineering']['complete_input']['B'] or not metric['engineering']['main_audit_equal']['B']:raise ValueError('Original baseline engineering checks absent/failed')
   with (origin/'audit.csv').open() as f:
    audit=list(csv.DictReader(f))
   if len(audit)!=replay['camera_packets'] or len({r['camera_ns'] for r in audit})!=len(audit):raise ValueError('Native audit receipt count/uniqueness')
   if any(int(r['gravity_submissions']) or int(r['velocity_submissions']) for r in audit):raise ValueError('Native audit injection')
   sensors={}
   for p,digest in ident['sensor_csv_sha'].items():
    if sha(p)!=digest:raise ValueError('Changed original sensor CSV '+p)
    sensors[p]=digest
   tree=ident['config_tree_sha'];paths=configs.get(tree['estimator_config.yaml'],[]);item['matching_estimator_paths']=[str(p) for p in paths]
   exact=[p for p in paths if all((p.parent/n).is_file() and sha(p.parent/n)==d for n,d in tree.items())]
   if len(exact)!=1:item.update(status='CONFIG_MISSING_OR_AMBIGUOUS',reason='Expected exactly one full original config tree',matching_trees=[str(p) for p in exact]);continue
   config=exact[0];runtime=Path(ident['runtime'])
   if sha(runtime/'manifest.json')!=ident['runtime_manifest_sha']:raise ValueError('Runtime manifest changed')
   if sha(runtime/'lib/libov_msckf_lib.so')!=ident['library_sha']:raise ValueError('Original runtime library changed')
   destination=views/seq/'B'
   if destination.exists():raise FileExistsError('Refuse overwrite view '+str(destination))
   destination.mkdir(parents=True)
   for p in origin.iterdir():
    if p.name!='identity.json':(destination/p.name).symlink_to(p)
   ident.update(mode='B',config_path=str(config.resolve()),alias_origin=str(origin),alias_origin_identity_sha=sha(origin/'identity.json'))
   (destination/'identity.json').write_text(json.dumps(ident,indent=2)+'\n')
   item.update(status='VERIFIED_B_VIEW',origin=str(origin),view=str(destination),view_identity_sha=sha(destination/'identity.json'),config_path=str(config),config_tree_sha=tree,sensor_csv_sha=sensors,files=fileproof,native_audit_rows=len(audit),complete_input=True,actual_injection=False,runtime_manifest_sha=ident['runtime_manifest_sha'])
  except Exception as e:item.update(status='INVALID_OR_UNAVAILABLE',reason=str(e))
 result=dict(scope='ORIGINAL_B_ONLY_NO_R3B_PREDECESSOR_RELABEL',manifest=str(manifest),manifest_sha=sha(manifest),tool_sha=sha(__file__),sequences=results,verified=sum(x['status']=='VERIFIED_B_VIEW' for x in results.values()))
 out.parent.mkdir(parents=True,exist_ok=True);out.write_text(json.dumps(result,indent=2)+'\n')
 print(json.dumps({k:v['status'] for k,v in results.items()}))
if __name__=='__main__':
 p=argparse.ArgumentParser()
 for k in ('manifest','views','out'):p.add_argument('--'+k,required=True)
 prepare(**vars(p.parse_args()))
