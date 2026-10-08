"""Actual <=10s previous/new live runs, with identical numerical configuration."""
import json,shutil
from pathlib import Path
import budget
from legacy import artifacts,passive_outputs
from check_live_receipts import check

def main():
    base=budget.OUT/'logger_short'
    if base.exists():raise FileExistsError('No implicit retry')
    base.mkdir()
    prior=budget.ROOT/'docs/ltv/feature_readiness_passive/frozen_config.json'
    old=json.loads(prior.read_text())
    runtimes={'P_PREV':Path(old['runtime']),'P_LOGGED':artifacts.snapshot()}
    origin=Path('/home/he/output/ltv_feature_readiness_passive/mode_configs/STEREO_THEN_TEMPORAL/ltv_euroc/P_NEW')
    config=base/'config';shutil.copytree(origin,config)
    inputs=json.loads((budget.OUT/'input_metadata.json').read_text())['sequences']['V1_01_easy']
    dirs=[];records={}
    for label,runtime in runtimes.items():
        artifacts.verify(runtime)
        dest=base/label;dest.mkdir();dirs.append(dest)
        identity={'stage':'logger_only_no_observer_changes','label':label,'runtime':str(runtime),
                  'runtime_manifest_sha':artifacts.sha(runtime/'manifest.json'),'config_sha':artifacts.tree(config),
                  'inputs':inputs,'short_seconds':10}
        (dest/'identity.json').write_text(json.dumps(identity,indent=2)+'\n')
        command=artifacts.command(runtime,'run_ltv_feature_passive')+[str(config/'estimator_config.yaml'),inputs['root'],str(dest),'P_NEW','10']
        result=budget.run('short_real',command,identity,input_seconds=10)
        records[label]=dict(identity,attempt=result)
        (dest/'attempt.json').write_text(json.dumps(result,indent=2)+'\n')
        if result['exit_code']:raise RuntimeError('Preserved failed short input; inspect before new counted attempt')
    parity=passive_outputs.compare(dirs)
    if artifacts.sha(dirs[0]/'cache.bin')!=artifacts.sha(dirs[1]/'cache.bin'):
        raise RuntimeError('Logger-only run altered cache numerical evidence')
    receipt=check(dirs[1])
    result={'status':'PASS','runs':records,'passive':parity,'live_receipts':receipt,'full_numeric_cache_byte_identical':True}
    (base/'result.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
if __name__=='__main__':main()
