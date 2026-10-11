"""Run one explicit real Passive identity through the shared ledger (no evaluation GT)."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import shutil
import sys
import budget
from prepare import prepare
import artifacts

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()

def mode_config(source,family,mode):
    base=budget.OUT/'configs'/source/family
    dest=budget.OUT/'mode_configs'/source/family/mode
    if not dest.exists():
        shutil.copytree(base,dest)
        p=dest/'estimator_config.yaml';s=p.read_text()
        values={'ltv_enabled':str(mode!='B').lower(),'ltv_feature_readiness_enabled':str(mode=='P_NEW').lower(),
                'ltv_enable_gravity':'false','ltv_enable_velocity':'false','ltv_value_diagnostics_enabled':'false'}
        for k,v in values.items():
            if re.search(r'^'+k+r':',s,re.M):s=re.sub(r'^'+k+r':.*$',k+': '+v,s,flags=re.M)
            else:s+='\n'+k+': '+v+'\n'
        p.write_text(s)
    return dest/'estimator_config.yaml'

def run(sequence, mode, source='STEREO', short=None, repeat=0, confirmation=False):
    p=prepare();protocol=json.loads((budget.DOC/'protocol.json').read_text())
    frozen=None;freeze_sha=None
    if confirmation:
        from freeze import check
        frozen,freeze_sha=check()
        if sequence not in protocol['real_confirmation'] or source!=frozen['source'] or short is not None:
            raise ValueError('Outside frozen confirmation identity')
        if repeat not in (0,1) or (repeat==1 and (sequence not in protocol['repeat_P_NEW'] or mode!='P_NEW')):
            raise ValueError('Unregistered confirmation repeat')
    elif sequence not in protocol['real_development']:
        raise RuntimeError('Use future frozen validation dispatcher for confirmation; development runner cannot access it')
    if source not in ['STEREO','TEMPORAL_POSE','STEREO_THEN_TEMPORAL'] or mode not in ['B','P_OLD','P_NEW']:raise ValueError('Invalid registered method/mode')
    if short is not None and not 0<short<=10:raise ValueError('Short input <=10 seconds')
    runtime=Path(frozen['runtime']) if frozen else artifacts.snapshot()
    binary=runtime/'run_ltv_feature_passive'
    config=mode_config(source,'ltv_euroc' if sequence.startswith('V') else 'uzhfpv_indoor',mode)
    identity={'sequence':sequence,'mode':mode,'source':source,'short_seconds':short,'repeat':repeat,
              'binary_sha':sha(binary),'library_sha':sha(runtime/'lib/libov_msckf_lib.so'),
              'runtime':str(runtime),'runtime_manifest_sha':sha(runtime/'manifest.json'),
              'config_tree_sha':artifacts.tree(config.parent),'config_sha':sha(config),'sensor_csv_sha':p['sensors'][sequence]['csv_sha256'],'phase':'confirmation' if confirmation else 'development'}
    if confirmation:identity['freeze_sha']=freeze_sha
    key=hashlib.sha256(json.dumps(identity,sort_keys=True).encode()).hexdigest()[:12]
    out=budget.OUT/'real_runs'/f'{sequence}_{mode}_{key}'
    if out.exists():
        if not confirmation:raise RuntimeError('Existing development directory preserved; inspect before reuse/retry')
        if json.loads((out/'identity.json').read_text())!=identity:raise RuntimeError('Identity mismatch')
        seal=json.loads((out/'artifacts_sha256.json').read_text())
        for name,digest in seal.items():
            if artifacts.sha(out/name)!=digest:raise RuntimeError('Saved artifact changed')
        from audit.passive_outputs import inspect
        if not inspect(out)['complete']:raise RuntimeError('Incomplete confirmation result')
        return out,config
    if not confirmation and short is None and budget.usage()['real']>=24:raise RuntimeError('Preserve eleven confirmation attempts and one full cache parity')
    out.mkdir(parents=True)
    (out/'identity.json').write_text(json.dumps(identity,indent=2)+'\n')
    command=artifacts.command(runtime,'run_ltv_feature_passive')+[str(config),p['sensors'][sequence]['root'],str(out),mode]
    if short is not None:command.append(str(short))
    result=budget.run('short_real' if short is not None else 'real',command,json.dumps(identity,sort_keys=True),input_seconds=short)
    print(json.dumps(dict(result,output=str(out),config=str(config))),flush=True)
    if result['exit_code']:raise RuntimeError('Preserved failed real attempt; inspect logs before explicit retry')
    (out/'attempt.json').write_text(json.dumps(result,indent=2)+'\n')
    (out/'artifacts_sha256.json').write_text(json.dumps(artifacts.tree(out),indent=2)+'\n')
    return out,config

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('sequence');p.add_argument('mode',choices=['B','P_OLD','P_NEW']);p.add_argument('--source',default='STEREO');p.add_argument('--short',type=float);p.add_argument('--repeat',type=int,default=0);a=p.parse_args();run(a.sequence,a.mode,a.source,a.short,a.repeat)
