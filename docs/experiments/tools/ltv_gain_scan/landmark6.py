#!/usr/bin/env python3
"""Landmark-only 6x extension; reuse L1/L2/L4 and frozen OFF, no algorithm edits."""
import argparse
import json
from pathlib import Path
import shutil
import subprocess
import numpy as np
from study import Study, ROOT, TOOLS, set_keys, write, table_csv, sha, process
from independent import EUROC, REP, assess

DEST=ROOT/'docs/euroc_tune_results/landmark'


def prepare(coord,parent):
    if coord.exists():raise ValueError('new coordination directory required')
    coord.mkdir(parents=True)
    for name in ['registry','contracts','tools','specs','support','configs','locks']:(coord/name).mkdir()
    identity=json.loads((parent/'identity.json').read_text());identity['coord_root']=str(coord);write(coord/'identity.json',identity)
    binary=Path(identity['artifact'])/'install/ov_msckf/lib/ov_msckf/run_ltv_gv_production';assert sha(binary)==identity['binary_sha256']
    delta=subprocess.check_output(['git','diff',identity['source_oid'],'HEAD','--','ov_core/src','ov_init/src','ov_msckf/src','config','docs/experiments/tools/native/run_ltv_feature_passive.cpp'],cwd=ROOT);assert not delta
    tasks=json.loads((parent/'session.json').read_text())['tasks']
    for n,t in tasks.items():t.update(task_root=str(coord/n),namespace=f'ltv_l6_{coord.name}_{n}');(coord/n).mkdir()
    write(coord/'session.json',{'tasks':tasks})
    shared=Path('/home/he/output/ltv_shared_replay_locks')
    for name in ['build_replay.lock','performance_gate.lock']+[f'slot_{i}.lock' for i in range(8)]:(coord/'locks'/name).symlink_to(shared/name)
    for f in TOOLS.glob('*.py'):shutil.copy2(f,coord/'tools'/f.name)
    shutil.copy2(TOOLS/'env_exec.sh',coord/'tools/env_exec.sh');(coord/'contracts/empty_colcon_defaults.yaml').write_text('{}\n')
    write(coord/'contracts/protocol.json',dict(mode='L',new_alpha=6,reused_alphas=[1,2,4],gravity=False,velocity=False,native_slam=0,sigma_floor_m=.5/np.sqrt(6),information_cap=.06,max_points=2,sequences=EUROC,representatives=REP,gate='strict >10% vs OFF or L1 blocks extension; initialization<=1us; frozen support 100%',threads=1,workers=8))
    inputs=json.loads((parent/'data_identity.json').read_text());write(coord/'data_identity.json',inputs)
    for seq,item in inputs.items():
        assert sha(item['gt'])==item['gt_sha256']
        manifest=parent/'support'/f'{seq}_input_hashes.json'
        for name,h in json.loads(manifest.read_text()).items():assert sha(Path(item['root'])/name)==h
        shutil.copy2(manifest,coord/'support'/manifest.name);shutil.copy2(parent/'support'/f'{seq}.npz',coord/'support'/f'{seq}.npz')
    rows=[dict(r,historical_reuse=True) for r in json.loads((ROOT/'docs/euroc_tune_results/results.json').read_text()) if r['sequence'] in EUROC and (r['mode']=='OFF' or r['mode']=='L')]
    assert len(rows)==40;write(coord/'index.json',rows);write(coord/'screening.json',[])
    base=ROOT/'docs/euroc_tune_results/configs/euroc/L_1';dest=coord/'configs/euroc/L_6';shutil.copytree(base,dest,symlinks=False)
    config=dest/'estimator_config.yaml';config.chmod(config.stat().st_mode|0o200);config.write_text(set_keys(config.read_text(),dict(ltv_landmark_approx_sigma_floor_m=.5/np.sqrt(6),ltv_landmark_approx_information_cap=.06,ltv_landmark_approx_max_points=2,num_opencv_threads=1)));config.chmod(0o444)
    write(coord/'analysis_identity.json',{str(f):sha(f) for f in [Path(__file__).resolve(),ROOT/'docs/experiments/tools/ltv_gain_scan/study.py',TOOLS/'launcher.py']})
    print('prepared',coord,flush=True)


class Landmark6(Study):
    def comparisons(self):
        rows=[]
        for seq in REP:
            row=dict(self.lookup(seq,'L',6));row['off_ate']=self.lookup(seq,'OFF')['ate_rmse_m'];row['reference_ate']=self.lookup(seq,'L',1)['ate_rmse_m'];rows.append(row)
        return rows

    def execute_trial(self):
        if not (self.coord/'smoke.json').exists():write(self.coord/'smoke.json',self.batch([(REP[0],'L',6,40,'smoke')],short=True))
        if any(r['exit_code'] or r['audit']['errors'] for r in json.loads((self.coord/'smoke.json').read_text())):raise RuntimeError('smoke failed')
        jobs=lambda seqs:[(s,'L',6,None,'full') for s in seqs if not self.lookup(s,'L',6)]
        self.batch(jobs(REP));result=assess(self.comparisons(),REP);self.screening=[dict(mode='L',alpha=6,**result,sequences=self.comparisons())];write(self.coord/'screening.json',self.screening)
        self.publish_trial()
        if result['status']=='PASS':self.batch(jobs([s for s in EUROC if s not in REP]))
        else:
            for seq in EUROC:
                if not self.lookup(seq,'L',6):self.index.append(dict(sequence=seq,mode='L',alpha=6,historical_reuse=False,status='BLOCKED',reason='representative gate blocked',ate_rmse_m=None))
            write(self.coord/'index.json',self.index)
        self.audit_trial();self.publish_trial()

    def audit_trial(self):
        errors=[];new=[r for r in self.index if r['mode']=='L' and r['alpha']==6]
        if len(self.index)!=50 or len({(r['sequence'],r['mode'],r['alpha']) for r in self.index})!=50:errors.append('identity count')
        for row in new:
            if 'output_dir' not in row:continue
            out=Path(row['output_dir']);m=json.loads((out/'manifest_finish.json').read_text());opt=json.loads((out/'effective_options.json').read_text())
            if opt['ltv_enable_gravity'] or opt['ltv_enable_velocity'] or not opt['ltv_enable_landmark_approx']:errors.append('mode mismatch')
            if not np.isclose(opt['ltv_landmark_approx_sigma_floor_m'],.5/np.sqrt(6),rtol=1e-14) or opt['ltv_landmark_approx_information_cap']!=.06 or opt['ltv_landmark_approx_max_points']!=2:errors.append('landmark budget mismatch')
            if json.loads((out/'threading.json').read_text())['actual_opencv_threads']!=1:errors.append('thread mismatch')
            for category in ['config_paths','binary_paths']:
                for path,item in m[category].items():
                    if sha(path)!=item['sha256']:errors.append('artifact changed')
            check=self.evaluate(row)
            if check['status']!=row['status'] or (row['status']=='VALID_FULL_MATCHED' and not np.isclose(check['ate_rmse_m'],row['ate_rmse_m'],rtol=1e-12,atol=0)):errors.append('ATE mismatch')
        if assess(self.comparisons(),REP)['status']!=self.screening[0]['status']:errors.append('screen mismatch')
        live=[]
        for f in (self.coord/'registry').glob('*.jsonl'):
            for line in f.read_text().splitlines():
                e=json.loads(line)
                if e['event']=='PROCESS_REGISTERED':
                    q=process(e['pid'])
                    if q and q['state']!='Z' and all(q[k]==e[k] for k in ['pgid','start_ticks','executable']):live.append(q)
        if live:errors.append('owned process remains')
        write(self.coord/'independent_audit.json',dict(status='FAIL' if errors else 'PASS',errors=errors,recorded=len(self.index),historical=40,new_full_recomputed=sum('run_id' in r for r in new),owned_live_processes=live))
        if errors:raise RuntimeError(errors)

    def publish_trial(self):
        DEST.mkdir(parents=True,exist_ok=True);rows=[]
        for seq in EUROC:
            for alpha in [1,2,4,6]:
                r=dict(self.lookup(seq,'L',alpha) or dict(sequence=seq,mode='L',alpha=alpha,status='PENDING',ate_rmse_m=None));x=r['ate_rmse_m']
                for key,a in [('relative_off_percent',None),('relative_l1_percent',1),('relative_l4_percent',4)]:
                    b=self.lookup(seq,'OFF' if a is None else 'L',1 if a is None else a)['ate_rmse_m'];r[key]=100*(x/b-1) if x is not None else None
                rows.append(r)
        write(DEST/'results.json',rows);table_csv(DEST/'results.csv',rows);write(DEST/'screening.json',self.screening)
        for name in ['identity.json','analysis_identity.json','independent_audit.json','smoke.json','session.json']:
            if (self.coord/name).exists():shutil.copy2(self.coord/name,DEST/name)
        shutil.copy2(self.coord/'contracts/protocol.json',DEST/'protocol.json')
        if not (DEST/'config').exists():shutil.copytree(self.coord/'configs/euroc/L_6',DEST/'config')
        write(DEST/'run_manifest.json',[json.loads(f.read_text()) for f in self.coord.glob('w*/results/*/manifest_finish.json')])
        text=['## Landmark 独立档位 L1/L2/L4/L6','', 'G/V关闭，原生SLAM关闭；复用L1/L2/L4，仅新增L6。标准差下限为0.5/√α m，信息上限0.01α，最多两点；距离/历史项、选择、门控和更新顺序不变。新回放单线程，历史为四线程，不混用耗时作性能验收。', '', '| 序列 | OFF ATE(m) | L1 ATE(m) | L2 ATE(m) | L4 ATE(m) | L6 ATE(m) | L6 vs OFF | L6 vs L1 | L6 vs L4 | 状态 | L6应用次数 |','|---|---:|---:|---:|---:|---:|---:|---:|---:|---|---:|']
        for seq in EUROC:
            a=[r for r in rows if r['sequence']==seq];off=self.lookup(seq,'OFF')['ate_rmse_m'];cells=[]
            for r in a:
                x=r['ate_rmse_m'];value='—' if x is None else f'{x:.9f}';cells.append('**'+value+'**' if x is not None and x<off else value)
            r=a[-1]
            def pct(v):return '—' if v is None else ('**'+f'{v:+.6f}%'+ '**' if v<0 else f'{v:+.6f}%')
            text.append('| '+seq+f' | {off:.9f} | '+' | '.join(cells)+' | '+' | '.join(pct(r[k]) for k in ['relative_off_percent','relative_l1_percent','relative_l4_percent'])+f" | {r['status']} | {r.get('audit',{}).get('actual_L',0)} |")
        text+=['','### Landmark 收益与实际信息量','', '| 参数 | 平均相对OFF | 中位数 | 改善/退化 | 最差变化 | 总应用帧 | 平均每应用帧信息量 |','|---|---:|---:|---:|---:|---:|---:|']
        for alpha in [1,2,4,6]:
            a=[r for r in rows if r['alpha']==alpha]
            if any(r['status']!='VALID_FULL_MATCHED' for r in a):continue
            d=[r['relative_off_percent'] for r in a];infos=[r.get('landmark_information',{}) for r in a];count=sum(r.get('audit',{}).get('actual_L',0) for r in a);amount=sum(i.get('total',0) for i in infos);name=f'L{alpha}';mean=float(np.mean(d))
            if mean<0:name='**'+name+'**'
            text.append(f'| {name} | {mean:+.6f}% | {np.median(d):+.6f}% | {sum(x<0 for x in d)}/{sum(x>0 for x in d)} | {max(d):+.6f}% | {sum(r.get("audit",{}).get("actual_L",0) for r in a)} | '+(f'{amount/count:.8f}' if count else '未记录')+' |')
        notes=DEST/'analysis_notes.md'
        if notes.exists():text+=['',notes.read_text().rstrip()]
        text+=['',f'原始目录 `{self.coord}`。', '[结构化结果](euroc_tune_results/landmark/results.json) · [筛选](euroc_tune_results/landmark/screening.json) · [审计](euroc_tune_results/landmark/independent_audit.json)','']
        section='\n'.join(text);(DEST/'section.md').write_text(section);report=ROOT/'docs/euroc_tune_results.md';old=report.read_text();marker='\n## Landmark 独立档位 L1/L2/L4/L6'
        if marker in old:old=old[:old.index(marker)]
        report.write_text(old.rstrip()+'\n\n'+section)
        write(DEST/'sha256.json',{str(f.relative_to(DEST)):sha(f) for f in sorted(DEST.rglob('*')) if f.is_file() and f.name!='sha256.json'})


def main():
    p=argparse.ArgumentParser();p.add_argument('action',choices=['prepare','run','publish']);p.add_argument('coord',type=Path);p.add_argument('--parent',type=Path);a=p.parse_args()
    if a.action=='prepare':prepare(a.coord,a.parent)
    elif a.action=='run':Landmark6(a.coord).execute_trial()
    else:Landmark6(a.coord).publish_trial()

if __name__=='__main__':main()
