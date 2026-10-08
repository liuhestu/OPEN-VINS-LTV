"""Metadata-only input audit; no candidate errors or GT labels evaluated."""
import csv,json
from pathlib import Path
import budget
from legacy import artifacts

def metadata(path,images=False):
    stamps=[];missing=[]
    with path.open() as f:
        for row in csv.reader(f):
            if not row or row[0].startswith('#'):continue
            stamp=int(row[0]);stamps.append(stamp)
            if images and not (path.parent/'data'/row[1]).is_file():missing.append(row[1])
    if not stamps or any(b<=a for a,b in zip(stamps,stamps[1:])):raise ValueError(str(path)+' time order')
    if missing:raise FileNotFoundError(str(path)+' '+repr(missing[:3]))
    return {'path':str(path),'sha256':artifacts.sha(path),'rows':len(stamps),'first_ns':stamps[0],
            'last_ns':stamps[-1],'duration_s':(stamps[-1]-stamps[0])*1e-9,'missing_images':len(missing)}

def audit():
    protocol=json.loads((budget.DOC/'protocol.json').read_text());result={}
    for seq in protocol['real_final']:
        root=Path('/home/he/datasets/euroc/ASL')/seq/'mav0'
        result[seq]={'root':str(root),'sensors':{name:metadata(root/name/'data.csv',name.startswith('cam')) for name in ('cam0','cam1','imu0')},
                     'reference_metadata_only':metadata(root/'state_groundtruth_estimate0/data.csv')}
    doc={'status':'PASS','sequences':result,'GT_labels_evaluated':False,'image_content_hash_scope':'Sensor index CSV SHA plus complete referenced image existence; native runs freeze decoded input identity separately.'}
    output=budget.OUT/'input_metadata.json'
    if output.exists():raise FileExistsError(output)
    output.write_text(json.dumps(doc,indent=2)+'\n')
    print(json.dumps({'status':'PASS','sequences':len(result),'input_manifest':str(output)},indent=2))
if __name__=='__main__':audit()
