"""Content-addressed executable/library snapshots for reproducible real replay."""
import hashlib
import json
from pathlib import Path
import os
import re
import shutil
import subprocess
import budget

def sha(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as f:
        for block in iter(lambda:f.read(1<<20),b''):h.update(block)
    return h.hexdigest()

def tree(path):return {str(p.relative_to(path)):sha(p) for p in sorted(Path(path).rglob('*')) if p.is_file()}

def dependencies(binary,env=None):
    result={}
    text=subprocess.check_output(['ldd',str(binary)],text=True,env=env)
    if 'not found' in text:raise RuntimeError('Missing runtime library')
    for line in text.splitlines():
        match=re.match(r'\s*(\S+) => (/\S+) ',line)
        if match:result[match[1]]=Path(match[2])
        else:
            match=re.match(r'\s*(/\S+) ',line)
            if match:result[Path(match[1]).name]=Path(match[1])
    return result

def snapshot():
    binaries={name:budget.OUT/'build/tests/ltv'/name for name in ('run_ltv_feature_passive','replay_ltv_feature_cache')}
    libs={}
    for binary in binaries.values():
        for name,path in dependencies(binary).items():
            if name in libs and sha(libs[name])!=sha(path):raise RuntimeError('Conflicting dependency versions')
            libs[name]=path
    identity={'binaries':{name:sha(p) for name,p in binaries.items()},'libraries':{name:sha(p) for name,p in libs.items()}}
    key=hashlib.sha256(json.dumps(identity,sort_keys=True).encode()).hexdigest()
    root=budget.OUT/'runtime'/key
    if not root.exists():
        root.mkdir(parents=True);(root/'lib').mkdir()
        for name,p in binaries.items():shutil.copy2(p,root/name)
        for name,p in libs.items():shutil.copy2(p,root/'lib'/name)
        (root/'manifest.json').write_text(json.dumps(identity,indent=2)+'\n')
    verify(root)
    return root

def verify(root):
    root=Path(root);data=json.loads((root/'manifest.json').read_text())
    key=hashlib.sha256(json.dumps(data,sort_keys=True).encode()).hexdigest()
    if root.name!=key:raise RuntimeError('Runtime manifest differs from content address')
    for name,digest in data['binaries'].items():
        if sha(root/name)!=digest:raise RuntimeError('Frozen executable changed')
    for name,digest in data['libraries'].items():
        if sha(root/'lib'/name)!=digest:raise RuntimeError('Frozen library changed')
    env=dict(os.environ,LD_LIBRARY_PATH=str(root/'lib'))
    for name in data['binaries']:
        for lib,path in dependencies(root/name,env).items():
            if sha(path)!=data['libraries'][lib]:raise RuntimeError('Runtime resolution differs from snapshot')
            if not lib.startswith('ld-linux') and path.parent.resolve()!=(root/'lib').resolve():
                raise RuntimeError('Runtime resolved an external mutable library: '+str(path))
    return data

def command(root,name):return ['/usr/bin/env','LD_LIBRARY_PATH='+str(Path(root)/'lib'),str(Path(root)/name)]

if __name__=='__main__':print(snapshot())
