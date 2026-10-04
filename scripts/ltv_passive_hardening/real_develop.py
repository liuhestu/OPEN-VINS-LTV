"""One registered full native development replay, sensor-only estimator input."""
import argparse,json,shutil
from pathlib import Path
import budget
from legacy import artifacts,passive_outputs
from check_hardened_log import check

def main():
    p=argparse.ArgumentParser();p.add_argument('name');p.add_argument('sequence',choices=['V1_01_easy','V2_02_medium','V2_03_difficult']);p.add_argument('--initial-warmup',action='store_true');a=p.parse_args()
    root=budget.OUT/'real_development'/a.name
    if root.exists():raise FileExistsError('Preserve attempts; use an explicit new identity for retry')
    root.mkdir(parents=True)
    runtime=artifacts.snapshot();config=root/'config';shutil.copytree(budget.OUT/'logger_short/config',config)
    with (config/'estimator_config.yaml').open('a') as f:
        f.write('\nltv_passive_hardening_enabled: true\n')
        if a.initial_warmup:f.write('ltv_hardening_initial_warmup: true\n')
    inputs=json.loads((budget.OUT/'input_metadata.json').read_text())['sequences'][a.sequence]
    identity={'candidate':'R2_BOUNDED_INITIAL_ZERO_WARMUP_C0' if a.initial_warmup else 'R1_DELAYED_HEALTH_C0_ZERO_START','revision':'bounded_initial_warmup' if a.initial_warmup else 'startup_fault_classification_fix',
              'sequence':a.sequence,'runtime':str(runtime),'manifest_sha':artifacts.sha(runtime/'manifest.json'),
              'source_sha':budget.source_identity(),'config_sha':artifacts.tree(config),'inputs':inputs,'development':True}
    (root/'identity.json').write_text(json.dumps(identity,indent=2)+'\n')
    out=root/'P_NEW';out.mkdir()
    command=artifacts.command(runtime,'run_ltv_feature_passive')+[str(config/'estimator_config.yaml'),inputs['root'],str(out),'P_NEW']
    result=budget.run('real',command,identity)
    (root/'attempt.json').write_text(json.dumps(result,indent=2)+'\n')
    if result['exit_code']:raise RuntimeError('Preserved failed full native attempt: '+str(result))
    engineering={'passive':passive_outputs.inspect(out),'log':check(out),'precision':'NOT_YET_EVALUATED'}
    assert engineering['passive']['complete']
    (root/'engineering.json').write_text(json.dumps(engineering,indent=2)+'\n')
    print(json.dumps({'output':str(root),'attempt':result,'engineering':engineering},indent=2))
if __name__=='__main__':main()
