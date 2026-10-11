"""Final closeout audit, no simulation and no mutation of historical evidence."""
from closeout_review import DEFAULT, DEST, MATRIX, audit, file_sha
from run_study import ledger,code_identity,write,HERE
import json
from pathlib import Path
import re
import subprocess
import numpy as np


def main():
    audit(DEFAULT)
    es=ledger(DEFAULT);assert len(es)==64
    summary=json.loads((DEST/'evidence/summary.json').read_text())
    assert summary['status']=='COMPLETED_WITH_RESULTS' and not summary['missing']
    assert summary['historical_C_star']=='C6' and not summary['selection_repeated']
    assert json.loads((DEFAULT/'selection.json').read_text())['C_star']=='C6'
    numerical={};source={}
    for label,c in MATRIX.items():
        e=next(e for e in es if e['id']==summary['completed'][label]);rd=Path(e['directory'])
        assert e['config']==c and e['status']=='COMPLETED' and e['exit_code']==0
        assert json.loads((rd/'code.json').read_text())==code_identity()
        for name in ['trace.npz','camera.npz']:
            with np.load(rd/name) as z:
                counts=np.sum(z['ids']>=0,axis=1)
                for x,n in zip(z['x'],counts):
                    assert np.isfinite(x[:3*n+6]).all()
        with np.load(rd/'matrices.npz') as z:
            eigen=z['eigen'];symmetry=z['symmetry']
            assert np.all(eigen[:,0]>=-1e-10*np.maximum(1,np.abs(eigen[:,1])))
            assert np.max(symmetry)<1e-8
            for P,ids in zip(z['P'],z['ids']):
                d=3*np.sum(ids>=0)+6;assert np.isfinite(P[:d,:d]).all()
            numerical[label]={'finite_active_states_and_P':True,'P_min':float(eigen[:,0].min()),'symmetry_max':float(symmetry.max()),'pass':True}
        metric=json.loads((rd/'metrics.json').read_text());assert metric['input_sha']==file_sha(metric['input_path'])
        for p in rd.iterdir():
            if p.is_file():source[str(p)]=file_sha(p)
        source[metric['input_path']]=metric['input_sha']
    for name in ['report.md','conclusions.md']:
        p=DEST/name;width=None
        for line in p.read_text().splitlines():
            if line.startswith('|'):
                n=line.count('|');assert width is None or width==n,(name,line);width=n
            else:width=None
        for link in re.findall(r'\]\(([^)]+)\)',p.read_text()):
            if not link.startswith(('https:','http:')):assert (p.parent/link).exists(),link
    test=subprocess.run(['python3',str(HERE/'test_closeout_review.py')],capture_output=True,text=True)
    assert test.returncode==0
    for p in HERE.glob('*closeout*.py'):source[str(p)]=file_sha(p)
    write(DEST/'evidence/new_run_sources_sha256.json',source)
    write(DEST/'evidence/final_validation.json',{'status':'PASS','runs':64,'new_runs':6,'failed_new_runs':0,'numerical':numerical,'test_command':test.args,'test_exit_code':test.returncode,'test_output':test.stderr,'links_and_table_widths':True,'historical_artifacts_unchanged':True,'historical_ledger_prefix_unchanged':True,'historical_selection_unchanged':True})
    write(DEST/'evidence/artifacts_sha256.json',{str(p.relative_to(DEST)):file_sha(p) for p in sorted(DEST.rglob('*')) if p.is_file() and p.name!='artifacts_sha256.json'})
    print('PASS: six identities, finite states/P, spectral boundaries, inputs, history, links, tests; 64/64. STOP.')

if __name__=='__main__':main()
