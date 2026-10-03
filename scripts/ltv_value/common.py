"""Separate value-study identity and append-only evidence; never imports the old runner."""
from pathlib import Path
import hashlib, json
ROOT = Path(__file__).resolve().parents[2]
OUT = Path('/home/he/output/openvins_ltv_value_study_20261003')
EUROC = ['V1_01_easy', 'V2_02_medium', 'V2_03_difficult']
DEV = ['indoor_forward_3', 'indoor_45_2', 'outdoor_forward_1']
VALIDATION = ['indoor_forward_6', 'indoor_45_14', 'outdoor_forward_5']
SEQUENCES = EUROC + DEV + VALIDATION
MODES = ['B','P','G','V','GV']
WEIGHTS = {'C0': (10.,1.), 'C1':(5.,1.), 'C2':(10.,.5), 'C3':(5.,.5)}
def sha(path):
    h=hashlib.sha256()
    with open(path,'rb') as f:
        for chunk in iter(lambda:f.read(1024*1024),b''): h.update(chunk)
    return h.hexdigest()
def write(path, value):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    tmp=path.with_suffix(path.suffix+'.tmp')
    tmp.write_text(json.dumps(value,indent=2,allow_nan=False));tmp.replace(path)
def read(path): return json.loads(Path(path).read_text())
def source_identity():
    return {str(p.relative_to(ROOT)):sha(p) for folder in ['ov_core','ov_init','ov_msckf','config/ltv_value']
            for p in sorted((ROOT/folder).rglob('*')) if p.is_file() and p.suffix in ['.cpp','.h','.cmake','.txt','.yaml','.png']}
def latest_runs():
    return {r['id']:r for r in map(json.loads,(OUT/'runs.jsonl').read_text().splitlines())}
