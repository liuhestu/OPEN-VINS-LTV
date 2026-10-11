"""Verify initial tracked baseline except the explicitly authorized bridge/core files."""
import json
from pathlib import Path
import budget
from artifacts import sha

ALLOWED={
 'ov_msckf/cmake/ROS1.cmake','ov_msckf/cmake/ROS2.cmake',
 'ov_msckf/src/core/VioManager.cpp','ov_msckf/src/core/VioManager.h',
 'ov_msckf/src/ltv/LtvAdapter.cpp','ov_msckf/src/ltv/LtvAdapter.h',
 'ov_msckf/src/ltv/LtvOptions.h','ov_msckf/src/ltv/ltv_observer.cpp',
 'ov_msckf/src/ltv/ltv_observer.h','ov_msckf/tests/ltv/CMakeLists.txt'}

def verify():
    baseline=budget.OUT/'baseline_sha256.json'
    start=json.loads((budget.DOC/'evidence/start.json').read_text())
    if sha(baseline)!=start['baseline_manifest_sha256']:raise RuntimeError('Initial manifest changed')
    data=json.loads(baseline.read_text());changes={};violations=[]
    for name,digest in data.items():
        path=budget.ROOT/name
        current=sha(path) if path.is_file() else None
        if current!=digest:
            changes[name]={'initial':digest,'current':current}
            if name not in ALLOWED:violations.append(name)
    if violations:raise RuntimeError('Read-only baseline changed: '+str(violations))
    return {'status':'PASS','baseline_files':len(data),'unchanged_files':len(data)-len(changes),
            'authorized_changes':changes,'historical_experiments_repeated':False}

if __name__=='__main__':print(json.dumps(verify(),indent=2))
