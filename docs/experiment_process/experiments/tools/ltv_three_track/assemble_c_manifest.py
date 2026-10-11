#!/usr/bin/env python3
"""Preserve all task C attempts/queued/failures, not only successful evidence."""
import argparse
import csv
import hashlib
import json
from pathlib import Path

p=argparse.ArgumentParser();p.add_argument('coord',type=Path);p.add_argument('output',type=Path);a=p.parse_args()
if a.output.exists():raise RuntimeError('refusing manifest overwrite')
a.output.mkdir();runs={}
for line in (a.coord/'registry/task_c.jsonl').read_text().splitlines():
 e=json.loads(line);runs.setdefault(e['run_id'],[]).append(e)
rows=[]
for rid,events in runs.items():
 starts=[e for e in events if e['event']=='START'];finish=[e for e in events if e['event']=='FINISH'];fail=[e for e in events if e['event']=='LAUNCHER_FAILURE'];base=finish[-1] if finish else starts[-1] if starts else events[0]
 status=base.get('validity','LAUNCHER_FAILURE' if fail else 'RUNNING' if starts else 'WAITING_RESOURCE')
 rows.append({'task':'c','run_id':rid,'label':base.get('label'),'status':status,'exit_code':base.get('exit_code'),'source_oid':base.get('source_oid'),'worktree_head':base.get('worktree_head'),'dirty_state':base.get('dirty_state'),'dirty_patch_sha256':base.get('dirty_patch_sha256'),'source_snapshot':base.get('source_snapshot'),'binary_identity':json.dumps(base.get('binary_paths',{}),sort_keys=True),'config_identity':json.dumps(base.get('config_paths',{}),sort_keys=True),'data':json.dumps(base.get('data',{}),sort_keys=True),'configuration':json.dumps(base.get('configuration',{}),sort_keys=True),'pid':starts[-1].get('pid') if starts else None,'pgid':starts[-1].get('pgid') if starts else None,'process_start_identity':json.dumps(starts[-1].get('start_identity')) if starts else None,'started_at':base.get('started_at'),'finished_at':base.get('finished_at'),'diagnostic_level':base.get('diagnostic_level'),'historical_reuse':base.get('historical_reuse',False),'output_dir':base.get('output_dir'),'resources':json.dumps(base.get('resources',{}),sort_keys=True),'lock_order':json.dumps(base.get('locks',[])),'launcher_failure':json.dumps(fail[-1]) if fail else None})
with (a.output/'run_manifest.csv').open('w',newline='') as stream:
 writer=csv.DictWriter(stream,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
(a.output/'registry_task_c.jsonl').write_bytes((a.coord/'registry/task_c.jsonl').read_bytes())
(a.output/'source.json').write_text(json.dumps({'coord':str(a.coord),'registry_sha256':hashlib.sha256((a.coord/'registry/task_c.jsonl').read_bytes()).hexdigest(),'runs':len(rows),'all_attempts_retained':True},indent=2)+'\n')
print(json.dumps({'runs':len(rows),'all_attempts_retained':True}))
