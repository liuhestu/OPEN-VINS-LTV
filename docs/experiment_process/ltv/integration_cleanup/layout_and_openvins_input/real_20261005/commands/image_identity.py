import hashlib,json,sys
from pathlib import Path
OUT=Path(__file__).resolve().parent;p=OUT/'real_inputs.json';inputs=json.loads(p.read_text());manifest={}
checking='--check' in sys.argv
if not checking and (OUT/'images_identity.json').exists():raise FileExistsError('Image identity already registered')
for seq,case in inputs.items():
 root=Path(case['root']);images={}
 for camera in ('cam0','cam1'):
  for line in (root/camera/'data.csv').read_text().splitlines():
   if not line or line.startswith('#'):continue
   _,filename=line.split(',',1);relative=str(Path(camera)/'data'/filename.strip());images[relative]=hashlib.sha256((root/relative).read_bytes()).hexdigest()
 identity=hashlib.sha256(json.dumps(images,sort_keys=True,separators=(',',':')).encode()).hexdigest()
 manifest[seq]=dict(root=str(root),files=images,aggregate_sha=identity)
 if checking:assert case['images_aggregate_sha']==identity
 else:case['image_files']=len(images);case['images_aggregate_sha']=identity
 print(seq,len(images),identity,flush=True)
if checking:(OUT/'images_unchanged_after_replay.json').write_text(json.dumps({'status':'PASS_IMAGE_IDENTITIES_UNCHANGED','aggregates':{k:v['aggregate_sha'] for k,v in manifest.items()}},indent=2)+'\n')
else:
 (OUT/'images_identity.json').write_text(json.dumps(manifest,indent=2)+'\n');p.write_text(json.dumps(inputs,indent=2)+'\n')
