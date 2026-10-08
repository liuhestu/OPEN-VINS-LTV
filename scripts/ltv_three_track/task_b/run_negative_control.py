"""Light native control against own frozen libraries; no candidate/ROS rebuild."""
import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import shlex
import subprocess


def main(artifact, source, diagnostic, output):
    output.mkdir(exist_ok=False)
    with (diagnostic / 'landmark_approx.csv').open() as f:
        lookup={round(float(x['camera_time'])*1e9):int(x['rows']) for x in csv.DictReader(f) if int(x['rows'])>0}
    cases=[]
    for line in (diagnostic/'matrices.jsonl').open():
        r=json.loads(line);t=int(r['camera_ns']);key=min(lookup,key=lambda x:abs(x-t))
        if abs(key-t)>1000:continue
        H=r['joint_H'];rows=len(H)-lookup[key]
        if rows<12:continue
        columns=[];ids=[];sizes=[];offset=0
        for id,size in zip(r['joint_ids'],r['joint_sizes']):
            if id>=15 and any(H[i][j]!=0 for i in range(rows) for j in range(offset,offset+size)):
                assert size==6;ids.append(id);sizes.append(size);columns.extend(range(offset,offset+size))
            offset+=size
        assert columns
        cases.append((r,rows,columns,ids,sizes))
        if len(cases)==5:break
    assert len(cases)==5
    data=output/'real_visual_inputs.txt'
    with data.open('x') as f:
        f.write('5\n')
        for r,rows,columns,ids,sizes in cases:
            f.write(f"{len(r['prior_P'])} {rows} {len(columns)} {len(ids)} "+' '.join(f'{i} {s}' for i,s in zip(ids,sizes))+'\n')
            blocks=[r['prior_imu'],r['prior_fej'],r['prior_P'],[[r['joint_H'][i][j] for j in columns] for i in range(rows)], [x[:rows] for x in r['joint_R'][:rows]],r['joint_res'][:rows]]
            for block in blocks:
                f.write(' '.join(format(v,'.17g') for row in block for v in row)+'\n')
    obj=output/'control.o';binary=output/'control'
    cmake=artifact/'build/ov_msckf/CMakeFiles/test_ltv_landmark_approx.dir'
    flags={k:v for k,v in (x.split(' = ',1) for x in (cmake/'flags.make').read_text().splitlines() if ' = ' in x)}
    compile_cmd=['/usr/bin/c++',*shlex.split(flags['CXX_DEFINES']),*shlex.split(flags['CXX_INCLUDES']),*shlex.split(flags['CXX_FLAGS']),'-c',str(source),'-o',str(obj)]
    subprocess.run(compile_cmd,check=True)
    link=shlex.split((cmake/'link.txt').read_text());link=[str(obj) if x.endswith('.cpp.o') else x for x in link]
    at=link.index('-o');link[at+1]=str(binary)
    subprocess.run(link,cwd=artifact/'build/ov_msckf',check=True)
    with (output/'control.log').open('x') as f:
        subprocess.run([str(binary),str(data)],stdout=f,stderr=subprocess.STDOUT,check=True)
    (output/'identity.json').write_text(json.dumps({'native_libraries_artifact':str(artifact),'source':str(source),'source_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),'compile':compile_cmd,'link':link,'cases':'first five chronological real joint captures with >=12 visual rows; cloned means are controlled fixture, actual priorP/H/R/res retained; no calibration claim'},indent=2)+'\n')
    print((output/'control.log').read_text())


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('artifact',type=Path);p.add_argument('source',type=Path);p.add_argument('diagnostic',type=Path);p.add_argument('output',type=Path);a=p.parse_args();main(a.artifact,a.source,a.diagnostic,a.output)
