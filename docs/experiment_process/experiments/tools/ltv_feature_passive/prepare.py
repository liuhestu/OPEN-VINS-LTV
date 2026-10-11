"""Create new-study config copies and read-only sensor manifests; no replay or GT."""
from pathlib import Path
import hashlib
import json
import re
import shutil
from budget import ROOT, DOC, OUT, usage

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()

def prepare():
    protocol=json.loads((DOC/'protocol.json').read_text())
    start=json.loads((DOC/'evidence/start.json').read_text())
    assert sha(DOC/'protocol.json')==start['protocol_sha256']
    assert sha(DOC/'acceptance.json')==start['acceptance_sha256']
    assert sha(DOC/'goal.md')==protocol['goal_sha256']
    manifests={}
    for sequence in protocol['real_development']+protocol['real_confirmation']:
        if sequence=='V1_03_difficult':
            root=Path('/home/he/datasets/euroc/ASL')/sequence/'mav0'
        else:root=Path('/home/he/output/openvins_ltv_value_study_20261003/inputs')/sequence
        # Only sensor CSV metadata; no GT or image scans for preparation.
        files=[root/'cam0/data.csv',root/'cam1/data.csv',root/'imu0/data.csv']
        if not all(p.is_file() for p in files):raise FileNotFoundError(sequence)
        manifests[sequence]={'root':str(root),'csv_sha256':{str(p):sha(p) for p in files},
                             'split':'development' if sequence in protocol['real_development'] else 'confirmation'}
    for family in ['ltv_euroc','uzhfpv_indoor']:
        source=ROOT/'config/ltv_value'/family
        for seed_source in ['TEMPORAL_POSE','STEREO','STEREO_THEN_TEMPORAL']:
            dest=OUT/'configs'/seed_source/family
            if not dest.exists():
                shutil.copytree(source,dest)
                p=dest/'estimator_config.yaml';s=p.read_text()
                for k,v in {'ltv_feature_seed_source':f'"{seed_source}"','ltv_feature_readiness_enabled':'false',
                            'ltv_feature_apply_seed':'true','ltv_passive_audit_enabled':'true',
                            'ltv_feature_bearing_sigma_rad':'0.0008726646259971648'}.items():
                    if re.search(r'^'+k+r':',s,re.M):s=re.sub(r'^'+k+r':.*$',k+': '+v,s,flags=re.M)
                    else:s+='\n'+k+': '+v+'\n'
                p.write_text(s)
    value={'protocol_sha':sha(DOC/'protocol.json'),'sensors':manifests,'usage':usage(),
           'configs_sha256':{str(p):sha(p) for p in sorted((OUT/'configs').rglob('*')) if p.is_file()}}
    path=OUT/'preparation.json'
    if path.exists():
        old=json.loads(path.read_text());assert old['sensors']==value['sensors']
        assert all(value['configs_sha256'].get(k)==v for k,v in old['configs_sha256'].items())
        added={k:v for k,v in value['configs_sha256'].items() if k not in old['configs_sha256']}
        if added:
            extension=OUT/'preparation_candidate5.json'
            content={'protocol_sha':value['protocol_sha'],'additional_configs_sha256':added}
            if extension.exists():assert json.loads(extension.read_text())==content
            else:extension.write_text(json.dumps(content,indent=2)+'\n')
    else:path.write_text(json.dumps(value,indent=2)+'\n')
    return value

if __name__=='__main__':print(json.dumps(prepare(),indent=2))
