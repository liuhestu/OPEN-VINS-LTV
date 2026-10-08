"""Read-only scientific-output audit; writes only compact delivery evidence."""
import hashlib
import inspect
import json
from pathlib import Path
from collections import Counter
from run_study import DEFAULT, DOC, ROOT, code_identity, frozen_check, gains, digest, write
from diagnostics import combined_settling


def audit(out=DEFAULT, doc=DOC):
    evidence=doc/'evidence'
    entries=json.loads((out/'ledger.json').read_text())
    summary=json.loads((evidence/'summary.json').read_text())
    assert summary['status']=='COMPLETED' and not summary['missing']
    assert len(entries)<=64 and len(entries)==summary['runs_used']
    assert all(e['status']=='COMPLETED' and e['exit_code']==0 for e in entries)
    limits={'fixed':14,'discretization':4,'gains':19,'lifecycle':11,'initialization':8,'diagnostic':8}
    counts=Counter(e['stage'] for e in entries)
    assert all(counts[k]<=v for k,v in limits.items())
    current=code_identity()
    assert json.loads((out/'verification.json').read_text())['code']==current
    for e in entries:
        rd=Path(e['directory'])
        assert json.loads((rd/'config.json').read_text())==e['config']
        assert json.loads((rd/'code.json').read_text())==current
        for file in ['trace.npz','camera.npz','matrices.npz','metrics.json']:
            assert (rd/file).is_file()
    contract=json.loads((out/'selection_contract.json').read_text())
    assert contract['selector_source_sha256']==digest(inspect.getsource(gains).encode())
    assert contract['combined_settling_source_sha256']==digest(inspect.getsource(combined_settling).encode())
    assert all(v['pass'] for v in json.loads((evidence/'reference_checks.json').read_text()))
    assert all(json.loads((evidence/'full_parity.json').read_text()).values())
    parity=json.loads((evidence/'initialization_covariance_parity.json').read_text())
    assert len(parity)==8 and all(v['pass'] for v in parity)
    snapshots=json.loads((evidence/'initialization_snapshots.json').read_text())
    assert len(snapshots)==8 and all(v['unique_once'] and v['vg_means_unchanged_at_hook'] for v in snapshots.values())
    # Markdown table row width and every local link in the delivered main documents.
    import re
    for name in ['report.md','conclusions.md']:
        width=None
        for line in (doc/name).read_text().splitlines():
            if line.startswith('|'):
                count=line.count('|')
                if width is None: width=count
                assert width==count,(name,line)
            else: width=None
        for link in re.findall(r'\]\(([^)]+)\)',(doc/name).read_text()):
            if not link.startswith(('http:','https:')):
                assert (doc/link.split('#')[0]).exists(),link
    integrity=frozen_check()
    assert not integrity['changed']
    write(evidence/'integrity.json',integrity)
    result={'status':'PASS','runs':len(entries),'stage_counts':dict(counts),'numerical_code_identity_unchanged':True,'frozen_selector_unchanged':True,'all_initialization_covariance_checks':True,'report_tables_and_local_links':True,'authorized_frozen_files_unchanged':integrity['checked'],'no_new_simulation':True}
    write(evidence/'delivery_audit.json',result)
    paths=[p for p in (ROOT/'docs/experiments/tools/ltv_standalone_study').rglob('*') if p.is_file() and '__pycache__' not in p.parts]
    paths += [out/'core.so',out/'experimental.so']
    paths += [p for p in doc.rglob('*') if p.is_file() and p.name!='delivery_sha256.json']
    write(evidence/'delivery_sha256.json',{str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(paths)})
    print(json.dumps(result,indent=2))


if __name__=='__main__':
    audit()
