"""Capped native OFF parity and integrated cold/health log check; no GT read."""
import argparse,json,shutil
import budget
from legacy import artifacts,passive_outputs
from check_live_receipts import check
from check_hardened_log import check as check_hardened

def main():
    parser=argparse.ArgumentParser();parser.add_argument('name');parser.add_argument('--hardened-only',action='store_true');parser.add_argument('--r3',action='store_true');args=parser.parse_args()
    root=budget.OUT/args.name
    if root.exists():raise FileExistsError('A new counted name is required for retry')
    root.mkdir()
    runtime=artifacts.snapshot()
    (root/'runtime.txt').write_text(str(runtime)+'\n')
    source=budget.OUT/'logger_short/config'
    previous=budget.OUT/'logger_short/P_PREV'
    inputs=json.loads((budget.OUT/'input_metadata.json').read_text())['sequences']['V1_01_easy']
    results={}
    for label,enabled in ([('HARDENED',True)] if args.hardened_only else [('OFF',False),('HARDENED',True)]):
        config=root/('config_'+label);shutil.copytree(source,config)
        with (config/'estimator_config.yaml').open('a') as f:
            f.write('\nltv_passive_hardening_enabled: '+str(enabled).lower()+'\n')
            if enabled and args.r3:f.write('ltv_hardening_initial_warmup: true\nltv_hardening_preserve_constrained_state: true\n')
        out=root/label;out.mkdir()
        identity={'stage':'engineering_integrated_prefix','label':label,'runtime':str(runtime),
                  'runtime_manifest_sha':artifacts.sha(runtime/'manifest.json'),
                  'config_sha':artifacts.tree(config),'inputs':inputs,'input_seconds':10}
        (out/'identity.json').write_text(json.dumps(identity,indent=2))
        command=artifacts.command(runtime,'run_ltv_feature_passive')+[str(config/'estimator_config.yaml'),inputs['root'],str(out),'P_NEW','10']
        r=budget.run('short_real',command,identity,input_seconds=10,phase='engineering')
        if r['exit_code']:raise RuntimeError('Native prefix failed: '+str(r))
        parity=passive_outputs.compare([previous,out])
        receipt=check(out)
        if not enabled:
            assert artifacts.sha(previous/'cache.bin')==artifacts.sha(out/'cache.bin'),'OFF full cache differs'
        else:
            receipt.update(check_hardened(out))
        command=artifacts.command(runtime,'replay_ltv_feature_cache')+[str(config/'estimator_config.yaml'),str(out/'cache.bin'),str(out/'cache_parity.json')]
        replay=budget.run('test',command,{'stage':'short_cache_exact','source_attempt':r,'identity':identity},phase='engineering')
        if replay['exit_code']:raise RuntimeError('Short cache replay failed: '+str(replay))
        results[label]={'attempt':r,'cache_attempt':replay,'passive':parity,'receipts':receipt}
        (root/'result.json').write_text(json.dumps(results,indent=2)+'\n')
    print(json.dumps(results,indent=2))
if __name__=='__main__':main()
