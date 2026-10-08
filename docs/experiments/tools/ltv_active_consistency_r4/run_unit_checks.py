"""Execute integration checks with assertions enabled by the CMake test targets."""
import json,subprocess
from pathlib import Path
import budget
names=['test_ltv_active_consistency','test_ltv_active_lifecycle','test_ltv_feature_pipeline',
 'test_ltv_manager_bounded','test_ltv_adapter_hardened','test_ltv_controlled',
 'test_ltv_readiness','test_ltv_history_competition']
root=budget.OUT/'integration_checks'
root.mkdir(exist_ok=False)
results=[]
for name in names:
 command=[str(budget.OUT/'build/tests/ltv'/name)]
 with (root/(name+'.stdout')).open('w') as out,(root/(name+'.stderr')).open('w') as err:
  code=subprocess.run(command,stdout=out,stderr=err).returncode
 results.append(dict(name=name,command=command,exit_code=code))
(root/'results.json').write_text(json.dumps(results,indent=2)+'\n')
if any(r['exit_code'] for r in results):raise RuntimeError('integration failure; outputs preserved')
print(json.dumps({'status':'PASS_INTEGRATION_TESTS','results':results}))
