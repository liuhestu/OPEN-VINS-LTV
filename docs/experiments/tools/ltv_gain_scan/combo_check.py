#!/usr/bin/env python3
"""Three bounded additions to V49; retain matched OFF support and native math."""
import argparse
import csv
import json
from pathlib import Path
import shutil
import subprocess
import numpy as np
from independent import Independent, ROOT, TOOLS, EUROC, REP, assess, write, set_keys, table_csv, sha, process
from run_euroc_six import verify_output

PROFILES={'GV':(2,49,0),'VL':(0,49,6),'L_GV':(2,49,6)}
DEST=ROOT/'docs/euroc_tune_results/v49_combinations'


def prepare(coord,parent):
    if coord.exists():raise ValueError('new coordination directory required')
    coord.mkdir(parents=True)
    for n in ['registry','contracts','tools','specs','support','configs','locks']:(coord/n).mkdir()
    tasks=json.loads((parent/'session.json').read_text())['tasks']
    for n,t in tasks.items():t.update(task_root=str(coord/n),namespace=f'ltv_combo_{coord.name}_{n}');(coord/n).mkdir()
    write(coord/'session.json',{'tasks':tasks})
    shared=Path('/home/he/output/ltv_shared_replay_locks')
    for n in ['build_replay.lock','performance_gate.lock']+[f'slot_{i}.lock' for i in range(8)]:(coord/'locks'/n).symlink_to(shared/n)
    for f in TOOLS.glob('*.py'):shutil.copy2(f,coord/'tools'/f.name)
    shutil.copy2(TOOLS/'env_exec.sh',coord/'tools/env_exec.sh');(coord/'contracts/empty_colcon_defaults.yaml').write_text('{}\n')
    oid=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip();source=coord/'source';source.mkdir()
    archive=subprocess.Popen(['git','archive',oid],cwd=ROOT,stdout=subprocess.PIPE);subprocess.run(['tar','-x','-C',str(source)],stdin=archive.stdout,check=True);archive.stdout.close();assert archive.wait()==0
    ancestor=json.loads((ROOT/'docs/euroc_results_data/identity.json').read_text())
    spec=dict(coord_root=str(coord),task='w0',label='fresh_vl_driver_build',kind='build',four_worker_pool=True,worker_pool_size=8,source_oid=oid,source_snapshot=str(source),command=['bash',str(coord/'tools/env_exec.sh'),'-','python3',str(TOOLS/'task_a/build_scalar_runner.py'),ancestor['parent_library_artifact'],str(Path(ancestor['parent_library_artifact']).parents[1]/'source'/ancestor['parent_library_source_oid']),str(source),str(coord/'artifact'),str(coord/'build_evidence')])
    write(coord/'specs/build.json',spec);subprocess.run(['python3',str(coord/'tools/launcher.py'),str(coord/'specs/build.json')],check=True)
    identity=dict(source_oid=oid,source=str(source),artifact=str(coord/'artifact'),coord_root=str(coord));write(coord/'identity.json',identity)
    inputs=json.loads((parent/'data_identity.json').read_text());write(coord/'data_identity.json',inputs)
    for seq,item in inputs.items():
        assert sha(item['gt'])==item['gt_sha256'];manifest=parent/'support'/f'{seq}_input_hashes.json'
        for n,h in json.loads(manifest.read_text()).items():assert sha(Path(item['root'])/n)==h
        shutil.copy2(manifest,coord/'support'/manifest.name);shutil.copy2(parent/'support'/f'{seq}.npz',coord/'support'/f'{seq}.npz')
    prior=json.loads((parent/'index.json').read_text());refs=[dict(r,historical_reuse=True) for r in prior if r['mode']=='OFF' or (r['mode']=='V' and r['alpha_velocity'] in [1,49])];assert len(refs)==30
    write(coord/'index.json',refs);write(coord/'screening.json',[])
    for mode,(g,v,l) in PROFILES.items():
        dest=coord/'configs'/mode;shutil.copytree(parent/'configs/V_G0_V49',dest);p=dest/'estimator_config.yaml';p.chmod(p.stat().st_mode|0o200)
        p.write_text(set_keys(p.read_text(),dict(ltv_sigma_gravity_deg=10/np.sqrt(g or 1),ltv_sigma_velocity_mps=1/np.sqrt(v),ltv_landmark_approx_sigma_floor_m=.5/np.sqrt(l or 1),ltv_landmark_approx_information_cap=.01*(l or 1),ltv_landmark_approx_max_points=2,ltv_enable_gravity='false',ltv_enable_velocity='false',ltv_enable_landmark_approx='false',max_slam=0,num_opencv_threads=1)));p.chmod(0o444)
    write(coord/'contracts/protocol.json',dict(profiles=PROFILES,sequences=EUROC,representatives=REP,gate='strict >10% relative OFF or V49 blocks extension',position_support='frozen OFF, initialization <=1us, coverage100%',threads=1,workers=8))
    write(coord/'analysis_identity.json',{str(f):sha(f) for f in [Path(__file__).resolve(),ROOT/'docs/experiments/tools/ltv_gain_scan/independent.py',TOOLS/'launcher.py']});print('prepared',coord,flush=True)


class Combinations(Independent):
    def run_one(self,job,worker):
        seq,mode,g,v,short,phase=job;l=PROFILES[mode][2];config=self.coord/'configs'/mode/'estimator_config.yaml';install=Path(self.identity['artifact'])/'install';binary=install/'ov_msckf/lib/ov_msckf/run_ltv_gv_production'
        cmd=['bash',str(self.coord/'tools/env_exec.sh'),str(install),str(binary),str(config),self.inputs[seq]['root'],'{RUN_DIR}',mode]
        if short:cmd.append(str(short))
        spec=dict(coord_root=str(self.coord),task=f'w{worker}',label=f'{phase}_{seq}_{mode}_G{g}_V{v}_L{l}',kind='replay',four_worker_pool=True,worker_pool_size=8,source_oid=self.identity['source_oid'],source_snapshot=self.identity['source'],command=cmd,binary_paths=[str(binary)],config_paths=[str(f) for f in config.parent.iterdir() if f.is_file()],configuration=dict(sequence=seq,mode=mode,gravity=g,velocity=v,landmark=l,short_seconds=short,phase=phase),data=self.inputs[seq],diagnostic_level='scalar_ltv_csv')
        path=self.coord/'specs'/f'{phase}_{seq}_{mode}.json';write(path,spec);subprocess.run(['python3',str(self.coord/'tools/launcher.py'),str(path)],stdout=subprocess.DEVNULL,check=False)
        events=[json.loads(s) for s in (self.coord/f'registry/task_w{worker}.jsonl').read_text().splitlines()];m=next(e for e in reversed(events) if e.get('spec_path')==str(path) and e['event'] in ['FINISH','LAUNCHER_FAILURE']);out=Path(m.get('output_dir',''))
        row=dict(sequence=seq,mode=mode,alpha_gravity=g,alpha_velocity=v,alpha_landmark=l,historical_reuse=False,run_id=m['run_id'],output_dir=str(out),exit_code=m.get('exit_code'),audit=verify_output(out,mode,not short) if m.get('exit_code')==0 else {'errors':['runtime failure']})
        if row['exit_code']==0:
            opt=json.loads((out/'effective_options.json').read_text());thread=json.loads((out/'threading.json').read_text());row['threading']=thread
            expected=dict(ltv_sigma_gravity_deg=10/np.sqrt(g or 1),ltv_sigma_velocity_mps=1/np.sqrt(v),ltv_landmark_approx_sigma_floor_m=.5/np.sqrt(l or 1),ltv_landmark_approx_information_cap=.01*(l or 1),ltv_landmark_approx_max_points=2)
            for name,value in expected.items():
                if not np.isclose(opt[name],value,rtol=1e-14,atol=0):row['audit']['errors'].append('weight mismatch')
            if thread['actual_opencv_threads']!=1:row['audit']['errors'].append('thread mismatch')
            if (out/'landmark_approx.csv').exists():
                entries=list(csv.DictReader((out/'landmark_approx.csv').open()));active=[e for e in entries if e['consumed']=='1' and int(e['rows'])>0];row['landmark_information_total']=sum(float(e['information_budget_used']) for e in active)
        if not short:row.update(self.evaluate(row))
        return row

    def execute_trial(self):
        jobs=lambda seqs,mode,phase:[(s,mode,*PROFILES[mode][:2],None,phase) for s in seqs if not self.lookup(s,mode,*PROFILES[mode][:2])]
        if not (self.coord/'smoke.json').exists():write(self.coord/'smoke.json',self.batch([(REP[0],mode,*a[:2],40,'smoke') for mode,a in PROFILES.items()]))
        if any(r['exit_code'] or r['audit']['errors'] for r in json.loads((self.coord/'smoke.json').read_text())):raise RuntimeError('smoke failed')
        self.batch([j for mode in PROFILES for j in jobs(REP,mode,'representative')]);extending=[]
        for mode,(g,v,l) in PROFILES.items():
            rows=[]
            for seq in REP:
                r=dict(self.lookup(seq,mode,g,v));r['off_ate']=self.lookup(seq,'OFF')['ate_rmse_m'];r['reference_ate']=self.lookup(seq,'V',0,49)['ate_rmse_m'];rows.append(r)
            result=assess(rows,REP);self.screening.append(dict(mode=mode,**result,sequences=rows))
            if result['status']=='PASS':extending+=jobs([s for s in EUROC if s not in REP],mode,'full')
            else:self.block(EUROC,mode,g,v,'combination representative gate blocked')
        write(self.coord/'screening.json',self.screening);self.publish_trial();self.batch(extending);self.audit_trial();self.publish_trial()

    def audit_trial(self):
        errors=[];new=[r for r in self.index if not r['historical_reuse']]
        if len(self.index)!=60 or len({(r['sequence'],r['mode'],r['alpha_gravity'],r['alpha_velocity']) for r in self.index})!=60:errors.append('identity count')
        for r in new:
            if 'output_dir' not in r:continue
            out=Path(r['output_dir']);m=json.loads((out/'manifest_finish.json').read_text());check=self.evaluate(r)
            if verify_output(out,r['mode'],True)['errors']:errors.append('mode/input/receipt mismatch')
            opt=json.loads((out/'effective_options.json').read_text());g,v,l=PROFILES[r['mode']]
            expected={'ltv_sigma_gravity_deg':10/np.sqrt(g or 1),'ltv_sigma_velocity_mps':1/np.sqrt(v),'ltv_landmark_approx_sigma_floor_m':.5/np.sqrt(l or 1),'ltv_landmark_approx_information_cap':.01*(l or 1)}
            if any(not np.isclose(opt[n],x,rtol=1e-14,atol=0) for n,x in expected.items()):errors.append('parameter mismatch')
            if json.loads((out/'threading.json').read_text())['actual_opencv_threads']!=1:errors.append('thread mismatch')
            if check['status']!=r['status'] or (r['status']=='VALID_FULL_MATCHED' and not np.isclose(check['ate_rmse_m'],r['ate_rmse_m'],rtol=1e-12,atol=0)):errors.append('ATE mismatch')
            for category in ['binary_paths','config_paths']:
                for path,obj in m[category].items():
                    if sha(path)!=obj['sha256']:errors.append('artifact mutation')
            if m['source_mutation'] or m['binary_mutation']:errors.append('artifact mutation')
        for screen in self.screening:
            mode=screen['mode'];g,v,l=PROFILES[mode];comparisons=[]
            for seq in REP:
                row=dict(self.lookup(seq,mode,g,v));row['off_ate']=self.lookup(seq,'OFF')['ate_rmse_m'];row['reference_ate']=self.lookup(seq,'V',0,49)['ate_rmse_m'];comparisons.append(row)
            if assess(comparisons,REP)['status']!=screen['status']:errors.append('screen mismatch')
            if screen['status']=='BLOCKED' and any(r['mode']==mode and r['sequence'] not in REP and 'run_id' in r for r in new):errors.append('blocked extension launched')
        ms=[json.loads(p.read_text()) for p in self.coord.glob('w*/results/*/manifest_finish.json')];replay=[m for m in ms if m['kind']=='replay'];current=peak=0
        for _,delta in sorted([(m['started_at'],1) for m in replay]+[(m['finished_at'],-1) for m in replay]):current+=delta;peak=max(peak,current)
        if peak>8:errors.append('pool limit')
        if any(b['started_at']<r['finished_at'] and r['started_at']<b['finished_at'] for b in ms if b['kind']=='build' for r in replay):errors.append('build overlapped replay')
        live=[]
        for f in (self.coord/'registry').glob('*.jsonl'):
            for line in f.read_text().splitlines():
                e=json.loads(line)
                if e['event']=='PROCESS_REGISTERED':
                    q=process(e['pid'])
                    if q and q['state']!='Z' and all(q[k]==e[k] for k in ['pgid','start_ticks','executable']):live.append(q)
        if live:errors.append('owned process remains')
        write(self.coord/'independent_audit.json',dict(status='FAIL' if errors else 'PASS',errors=errors,recorded=len(self.index),new_full_recomputed=sum('run_id' in r for r in new),maximum_concurrent=peak,owned_live_processes=live))
        if errors:raise RuntimeError(errors)

    def publish_trial(self):
        DEST.mkdir(parents=True,exist_ok=True);rows=[]
        for seq in EUROC:
            for mode,(g,v,l) in PROFILES.items():
                r=dict(self.lookup(seq,mode,g,v) or dict(sequence=seq,mode=mode,alpha_gravity=g,alpha_velocity=v,alpha_landmark=l,status='PENDING',ate_rmse_m=None));x=r['ate_rmse_m'];off=self.lookup(seq,'OFF')['ate_rmse_m'];baseline=self.lookup(seq,'V',0,49)['ate_rmse_m'];r.update(off_ate=off,v49_ate=baseline,relative_off_percent=100*(x/off-1) if x is not None else None,relative_v49_percent=100*(x/baseline-1) if x is not None else None);rows.append(r)
        write(DEST/'results.json',rows);table_csv(DEST/'results.csv',rows);write(DEST/'screening.json',self.screening)
        for name in ['identity.json','analysis_identity.json','independent_audit.json','smoke.json','session.json']:
            if (self.coord/name).exists():shutil.copy2(self.coord/name,DEST/name)
        shutil.copy2(self.coord/'contracts/protocol.json',DEST/'protocol.json');shutil.copy2(Path(self.identity['artifact'])/'runner_build_identity.json',DEST/'runner_build_identity.json')
        if not (DEST/'configs').exists():shutil.copytree(self.coord/'configs',DEST/'configs')
        write(DEST/'run_manifest.json',[json.loads(f.read_text()) for f in self.coord.glob('w*/results/*/manifest_finish.json')])
        pct=lambda x:'—' if x is None else '**'+f'{x:+.6f}%'+ '**' if x<0 else f'{x:+.6f}%'
        text=['## V49 加入 G/L 的组合验证','', '三组为 G2/V49、V49/L6、G2/V49/L6。原生SLAM关闭，算法与门控不变。VL为显式实验入口，仅启用velocity与landmark，原六模式批次不变。代表筛选对OFF和V49严格超过10%阻止扩展；新回放实际单线程、最多八进程隔离。','', '| 序列 | 参数 | V49 ATE(m) | 本组ATE(m) | 相对OFF | 相对V49 | 状态 | G/V/L应用帧 |','|---|---|---:|---:|---:|---:|---|---|']
        for r in rows:
            g,v,l=PROFILES[r['mode']];name=f'G{g}/V{v}/L{l}';x=r['ate_rmse_m'];value='—' if x is None else f'{x:.9f}'
            if x is not None and r['relative_v49_percent']<0:name='**'+name+'**';value='**'+value+'**'
            a=r.get('audit',{});text.append(f"| {r['sequence']} | {name} | {r['v49_ate']:.9f} | {value} | {pct(r['relative_off_percent'])} | {pct(r['relative_v49_percent'])} | {r['status']} | {a.get('actual_G',0)}/{a.get('actual_V',0)}/{a.get('actual_L',0)} |")
        text+=['', '| 参数 | 平均相对OFF | 平均相对V49 | 中位数相对V49 | 比V49改善/退化 | 最差相对V49 |','|---|---:|---:|---:|---:|---:|']
        for mode,(g,v,l) in PROFILES.items():
            a=[r for r in rows if r['mode']==mode]
            if any(r['status']!='VALID_FULL_MATCHED' for r in a):continue
            d=[r['relative_v49_percent'] for r in a];name=f'G{g}/V{v}/L{l}'
            if np.mean(d)<0:name='**'+name+'**'
            text.append(f'| {name} | {pct(float(np.mean([r["relative_off_percent"] for r in a])))} | {pct(float(np.mean(d)))} | {pct(float(np.median(d)))} | {sum(v<0 for v in d)}/{sum(v>0 for v in d)} | {max(d):+.6f}% |')
        notes=DEST/'analysis_notes.md'
        if notes.exists():text+=['',notes.read_text().rstrip()]
        text+=['',f'原始目录 `{self.coord}`。','[结果](euroc_tune_results/v49_combinations/results.json) · [筛选](euroc_tune_results/v49_combinations/screening.json) · [审计](euroc_tune_results/v49_combinations/independent_audit.json)','']
        section='\n'.join(text);(DEST/'section.md').write_text(section);report=ROOT/'docs/euroc_tune_results.md';old=report.read_text();marker='\n## V49 加入 G/L 的组合验证'
        if marker in old:old=old[:old.index(marker)]
        report.write_text(old.rstrip()+'\n\n'+section);write(DEST/'sha256.json',{str(f.relative_to(DEST)):sha(f) for f in sorted(DEST.rglob('*')) if f.is_file() and f.name!='sha256.json'})


def main():
    p=argparse.ArgumentParser();p.add_argument('action',choices=['prepare','run','publish']);p.add_argument('coord',type=Path);p.add_argument('--parent',type=Path);a=p.parse_args()
    if a.action=='prepare':prepare(a.coord,a.parent)
    elif a.action=='run':Combinations(a.coord).execute_trial()
    else:Combinations(a.coord).publish_trial()

if __name__=='__main__':main()
