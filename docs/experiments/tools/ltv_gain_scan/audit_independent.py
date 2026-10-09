#!/usr/bin/env python3
"""Recompute independent-grid identities, gates, ATE, input hashes and reuse."""
import argparse
import json
from pathlib import Path
import numpy as np
from independent import Independent, TIERS, NEW, EXTRA, REP, EUROC, ROOT, assess, key, weights, sha, write


def audit(coord):
    trial=Independent(coord);errors=[]
    expected={(s,'OFF',0,0) for s in EUROC}
    expected|={(s,'G',a,0) for s in EUROC for a in TIERS}
    expected|={(s,'V',0,a) for s in EUROC for a in TIERS}
    expected|={(s,'GV',g,v) for s in EUROC for g in TIERS for v in TIERS}
    if (coord/'extra_single_plan.json').exists():
        expected|={(s,'G',a,0) for s in EUROC for a in EXTRA}
        expected|={(s,'V',0,a) for s in EUROC for a in EXTRA}
    observed=[key(r) for r in trial.index]
    if len(observed)!=len(set(observed)) or set(observed)!=expected:errors.append('duplicate/missing/unexpected identity')
    historical=[r for r in trial.index if r['historical_reuse']]
    prior=json.loads((ROOT/'docs/euroc_tune_results/results.json').read_text())
    if len(historical)!=100:errors.append('historical identity count')
    for row in historical:
        old=next((r for r in prior if r['run_id']==row['run_id']),None)
        if old is None or old['ate_rmse_m']!=row['ate_rmse_m'] or old['output_dir']!=row['output_dir']:errors.append('historical result rewritten')
    for seq,item in trial.inputs.items():
        manifest=coord/'support'/f'{seq}_input_hashes.json'
        for name,h in json.loads(manifest.read_text()).items():
            if sha(Path(item['root'])/name)!=h:errors.append('sensor changed '+seq+'/'+name)
        if sha(item['gt'])!=item['gt_sha256']:errors.append('GT changed '+seq)
    new=[r for r in trial.index if not r['historical_reuse'] and 'run_id' in r]
    dirs=[r['output_dir'] for r in new]
    if len(dirs)!=len(set(dirs)):errors.append('overlapping output')
    for row in new:
        out=Path(row['output_dir']);manifest=json.loads((out/'manifest_finish.json').read_text())
        options=json.loads((out/'effective_options.json').read_text());thread=json.loads((out/'threading.json').read_text())
        if thread['actual_opencv_threads']!=1:errors.append('not single threaded')
        for name,value in weights(row['mode'],row['alpha_gravity'],row['alpha_velocity']).items():
            if not np.isclose(options[name],value,rtol=1e-14,atol=0):errors.append('independent weight mismatch')
        for category in ['binary_paths','config_paths']:
            for path,value in manifest[category].items():
                if sha(path)!=value['sha256']:errors.append('artifact changed '+path)
        if manifest['source_mutation'] or manifest['binary_mutation']:errors.append('artifact mutation')
        evaluated=trial.evaluate(row)
        if evaluated['status']!=row['status']:errors.append('evaluation status mismatch '+row['run_id'])
        if row['status']=='VALID_FULL_MATCHED' and not np.isclose(evaluated['ate_rmse_m'],row['ate_rmse_m'],rtol=1e-12,atol=0):errors.append('ATE mismatch '+row['run_id'])
    for screen in trial.screening:
        seqs=EUROC if screen['stage'] in ['single_eligibility','extra_single_full_check'] else REP
        rows=[trial.comparison(s,screen['mode'],screen['alpha_gravity'],screen['alpha_velocity']) for s in seqs]
        if assess(rows,seqs)['status']!=screen['status']:errors.append('screen decision mismatch')
        if screen['stage']=='representative' and screen['status']=='BLOCKED':
            for seq in EUROC:
                r=trial.lookup(seq,screen['mode'],screen['alpha_gravity'],screen['alpha_velocity'])
                if seq not in REP and not r['historical_reuse'] and 'run_id' in r:errors.append('blocked case expanded')
    grid=json.loads((coord/'eligible_grid.json').read_text())
    for row in new:
        if row['mode']=='GV' and (row['alpha_gravity'] not in grid['gravity'] or row['alpha_velocity'] not in grid['velocity']):errors.append('unsafe branch combined')
    manifests=[json.loads(p.read_text()) for p in coord.glob('w*/results/*/manifest_finish.json')]
    timeline=sorted([(m['started_at'],1) for m in manifests]+[(m['finished_at'],-1) for m in manifests]);current=peak=0
    for _,change in timeline:current+=change;peak=max(peak,current)
    limit=len(json.loads((coord/'session.json').read_text())['tasks'])
    if peak>limit:errors.append('more processes than declared pool')
    sessions=json.loads((coord/'session.json').read_text())['tasks'];cpus=[c for t in sessions.values() for c in t['cpu_affinity']]
    if len(set(cpus))!=16 or len({t['ros_domain_id'] for t in sessions.values()})!=limit:errors.append('worker isolation mismatch')
    result=dict(status='FAIL' if errors else 'PASS',errors=errors,expected=len(expected),recorded=len(trial.index),historical=len(historical),new_full=len(new),maximum_concurrent=peak,sensor_hashes_reverified=True,position_ATE_recomputed=len(new),new_opencv_threads=1,historical_threads=4,status_counts={s:sum(r['status']==s for r in trial.index) for s in sorted({r['status'] for r in trial.index})})
    write(coord/'independent_audit.json',result);print(json.dumps(result))
    if errors:raise SystemExit(1)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('coord',type=Path);audit(p.parse_args().coord)
