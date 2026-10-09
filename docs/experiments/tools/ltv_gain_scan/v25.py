#!/usr/bin/env python3
"""One velocity-only extension, with original frozen OFF supports and risk gate."""
import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import numpy as np
from independent import Independent, ROOT, TOOLS, EUROC, REP, assess, set_keys, write, sha, weights, table_csv, process

DEST=ROOT/'docs/euroc_tune_results/v25'


def prepare(coord,parent,tiers=(25,),reference_tiers=(1,16)):
    if coord.exists():raise ValueError('new coordination directory required')
    coord.mkdir(parents=True)
    for name in ['registry','contracts','tools','specs','support','configs','locks']: (coord/name).mkdir()
    identity=json.loads((parent/'identity.json').read_text());identity['coord_root']=str(coord);write(coord/'identity.json',identity)
    binary=Path(identity['artifact'])/'install/ov_msckf/lib/ov_msckf/run_ltv_gv_production'
    assert sha(binary)==identity['binary_sha256']
    delta=subprocess.check_output(['git','diff',identity['source_oid'],'HEAD','--','ov_core/src','ov_init/src','ov_msckf/src','config','docs/experiments/tools/native/run_ltv_feature_passive.cpp'],cwd=ROOT)
    assert not delta
    tasks=json.loads((parent/'session.json').read_text())['tasks']
    for n,t in tasks.items():
        t.update(task_root=str(coord/n),namespace=f'ltv_velocity_{coord.name}_{n}');(coord/n).mkdir()
    write(coord/'session.json',{'tasks':tasks})
    shared=Path('/home/he/output/ltv_shared_replay_locks')
    for name in ['build_replay.lock','performance_gate.lock']+[f'slot_{i}.lock' for i in range(8)]: (coord/'locks'/name).symlink_to(shared/name)
    for f in TOOLS.glob('*.py'):shutil.copy2(f,coord/'tools'/f.name)
    shutil.copy2(TOOLS/'env_exec.sh',coord/'tools/env_exec.sh')
    (coord/'contracts/empty_colcon_defaults.yaml').write_text('{}\n')
    write(coord/'contracts/protocol.json',dict(mode='V',alpha_gravity=0,alpha_velocity=list(tiers),landmark=False,velocity_sigma_mps={a:1/np.sqrt(a) for a in tiers},sequences=EUROC,representatives=REP,gate='strict >10% vs OFF or V1 blocks extension; initialization<=1us; frozen support 100%',reference_v16='comparison only; reused',threads=1,workers=8))
    inputs=json.loads((parent/'data_identity.json').read_text());write(coord/'data_identity.json',inputs)
    for seq,item in inputs.items():
        if sha(item['gt'])!=item['gt_sha256']:raise ValueError('GT changed')
        manifest=parent/'support'/f'{seq}_input_hashes.json'
        for name,h in json.loads(manifest.read_text()).items():
            if sha(Path(item['root'])/name)!=h:raise ValueError('sensor changed '+seq)
        shutil.copy2(manifest,coord/'support'/manifest.name);shutil.copy2(parent/'support'/f'{seq}.npz',coord/'support'/f'{seq}.npz')
    prior=json.loads((parent/'index.json').read_text());refs=[dict(r,historical_reuse=True) for r in prior if r['mode']=='OFF' or (r['mode']=='V' and r['alpha_velocity'] in reference_tiers)]
    assert len(refs)==10*(1+len(reference_tiers))
    write(coord/'index.json',refs);write(coord/'screening.json',[])
    base=parent/'configs/V_G0_V16'
    if not base.exists():base=parent/'configs/V_G0_V25'
    for alpha in tiers:
        dest=coord/f'configs/V_G0_V{alpha}';shutil.copytree(base,dest)
        config=dest/'estimator_config.yaml';config.chmod(config.stat().st_mode|0o200);config.write_text(set_keys(config.read_text(),weights('V',0,alpha)));config.chmod(0o444)
    for f in (coord/'support').iterdir():f.chmod(0o444)
    write(coord/'analysis_identity.json',{str(f):sha(f) for f in [Path(__file__).resolve(),ROOT/'docs/experiments/tools/ltv_gain_scan/independent.py',TOOLS/'launcher.py']})
    print('prepared',coord,flush=True)


class V25(Independent):
    def execute_trial(self):
        if not (self.coord/'smoke.json').exists():write(self.coord/'smoke.json',self.batch([(REP[0],'V',0,25,40,'smoke')]))
        smoke=json.loads((self.coord/'smoke.json').read_text())
        if any(r['exit_code'] or r['audit']['errors'] for r in smoke):raise RuntimeError('smoke failed')
        self.batch(self.jobs(REP,'V',0,25,'representative'))
        result=self.screen(REP,'V',0,25,'representative');self.publish_trial()
        if result['status']=='PASS':self.batch(self.jobs([s for s in EUROC if s not in REP],'V',0,25,'full'))
        else:self.block(EUROC,'V',0,25,'V25 representative screening blocked')
        live=[]
        for path in (self.coord/'registry').glob('*.jsonl'):
            for line in path.read_text().splitlines():
                e=json.loads(line)
                if e['event']=='PROCESS_REGISTERED':
                    now=process(e['pid'])
                    if now and now['state']!='Z' and all(now[k]==e[k] for k in ['pgid','start_ticks','executable']):live.append(now)
        write(self.coord/'completion_audit.json',dict(expected=40,recorded=len(self.index),historical=30,new_full=sum(not r['historical_reuse'] and 'run_id' in r for r in self.index),owned_live_processes=live))
        assert len(self.index)==40 and not live
        self.audit_trial();self.publish_trial()

    def audit_trial(self):
        errors=[];rows=[r for r in self.index if not r['historical_reuse']]
        for row in rows:
            if 'output_dir' not in row:continue
            out=Path(row['output_dir']);m=json.loads((out/'manifest_finish.json').read_text());opt=json.loads((out/'effective_options.json').read_text());threads=json.loads((out/'threading.json').read_text())
            if opt['ltv_enable_gravity'] or opt['ltv_enable_landmark_approx'] or not opt['ltv_enable_velocity']:errors.append('mode mismatch')
            if not np.isclose(opt['ltv_sigma_velocity_mps'],.2,rtol=1e-14,atol=0) or threads['actual_opencv_threads']!=1:errors.append('weight/thread mismatch')
            for category in ['config_paths','binary_paths']:
                for path,item in m[category].items():
                    if sha(path)!=item['sha256']:errors.append('artifact changed')
            check=self.evaluate(row)
            if check['status']!=row['status'] or (row['status']=='VALID_FULL_MATCHED' and not np.isclose(check['ate_rmse_m'],row['ate_rmse_m'],rtol=1e-12,atol=0)):errors.append('ATE mismatch')
        for screen in self.screening:
            result=assess([self.comparison(s,'V',0,25) for s in REP],REP)
            if result['status']!=screen['status']:errors.append('screen mismatch')
            if screen['status']=='BLOCKED' and any(r['sequence'] not in REP and 'run_id' in r for r in rows):errors.append('blocked extension launched')
        ms=[json.loads(p.read_text()) for p in self.coord.glob('w*/results/*/manifest_finish.json')];n=peak=0
        for _,delta in sorted([(m['started_at'],1) for m in ms]+[(m['finished_at'],-1) for m in ms]):n+=delta;peak=max(peak,n)
        if peak>8:errors.append('pool overrun')
        write(self.coord/'independent_audit.json',dict(status='FAIL' if errors else 'PASS',errors=errors,new_full_recomputed=sum('run_id' in r for r in rows),maximum_concurrent=peak,artifact_weights_threads_verified=True))
        if errors:raise RuntimeError(errors)

    def publish_trial(self):
        DEST.mkdir(parents=True,exist_ok=True)
        rows=[]
        for seq in EUROC:
            row=self.lookup(seq,'V',0,25)
            r=dict(row or dict(sequence=seq,mode='V',alpha_gravity=0,alpha_velocity=25,status='PENDING',ate_rmse_m=None))
            x=r.get('ate_rmse_m');off=self.lookup(seq,'OFF')['ate_rmse_m'];v16=self.lookup(seq,'V',0,16)['ate_rmse_m']
            r['off_ate']=off;r['v16_ate']=v16;r['relative_off_percent']=100*(x/off-1) if x is not None else None;r['relative_v16_percent']=100*(x/v16-1) if x is not None else None;rows.append(r)
        write(DEST/'results.json',rows);table_csv(DEST/'results.csv',rows);write(DEST/'screening.json',self.screening)
        for name in ['identity.json','analysis_identity.json','completion_audit.json','independent_audit.json','smoke.json','session.json']:
            if (self.coord/name).exists():shutil.copy2(self.coord/name,DEST/name)
        shutil.copy2(self.coord/'contracts/protocol.json',DEST/'protocol.json')
        if not (DEST/'config').exists():shutil.copytree(self.coord/'configs/V_G0_V25',DEST/'config')
        write(DEST/'run_manifest.json',[json.loads(f.read_text()) for f in self.coord.glob('w*/results/*/manifest_finish.json')])
        text=['## V25 单分支追加测试','', 'G/landmark关闭；信息权重25×，速度标准差0.2 m/s。保持原参数与门控，原冻结OFF支持，八进程隔离、实际单线程。代表筛选相对OFF/V1严格超过10%阻止扩展；V16用于收益对照。','', '| 序列 | OFF ATE(m) | V16 ATE(m) | V25 ATE(m) | 相对OFF | 相对V16 | 状态 | G/V/L实际次数 |','|---|---:|---:|---:|---:|---:|---|---|']
        for r in rows:
            def pct(v):return '—' if v is None else ('**'+f'{v:+.6f}%'+ '**' if v<0 else f'{v:+.6f}%')
            x=r['ate_rmse_m'];ate='—' if x is None else f'{x:.9f}'
            if x is not None and r['relative_off_percent']<0:ate='**'+ate+'**'
            audit=r.get('audit',{});text.append(f"| {r['sequence']} | {r['off_ate']:.9f} | {r['v16_ate']:.9f} | {ate} | {pct(r['relative_off_percent'])} | {pct(r['relative_v16_percent'])} | {r['status']} | {audit.get('actual_G',0)}/{audit.get('actual_V',0)}/{audit.get('actual_L',0)} |")
        valid=[r for r in rows if r['status']=='VALID_FULL_MATCHED']
        if len(valid)==10:
            delta=[r['relative_off_percent'] for r in valid];extra=[r['relative_v16_percent'] for r in valid]
            text+=['',f'V25 相对 OFF 的每序列等权平均变化为 {np.mean(delta):+.6f}%，中位数 {np.median(delta):+.6f}%；改善/退化为 {sum(v<0 for v in delta)}/{sum(v>0 for v in delta)}，最差退化 {max(delta):+.6f}%。相对 V16 的平均变化为 {np.mean(extra):+.6f}%，改善/退化为 {sum(v<0 for v in extra)}/{sum(v>0 for v in extra)}。',
                   '这是固定数据集探索结果，不能据此推断无限增大权重会持续改善；历史预测FAIL与工程结论保留。']
        else:text+=['',f'当前完整有效 {len(valid)}/10；失败、阻止和未启动不作为完整ATE。']
        text+=['',f'原始目录 `{self.coord}`。', '[结构化结果](euroc_tune_results/v25/results.json) · [筛选](euroc_tune_results/v25/screening.json) · [审计](euroc_tune_results/v25/independent_audit.json)','']
        notes=DEST/'analysis_notes.md'
        if notes.exists():text+=['',notes.read_text().rstrip(),'']
        report=ROOT/'docs/euroc_tune_results.md';old=report.read_text();marker='\n## V25 单分支追加测试'
        if marker in old:old=old[:old.index(marker)]
        section='\n'.join(text)
        (DEST/'section.md').write_text(section)
        landmark_section=ROOT/'docs/euroc_tune_results/landmark/section.md'
        suffix='\n\n'+landmark_section.read_text() if landmark_section.exists() else ''
        report.write_text(old.rstrip()+'\n\n'+section+suffix)
        write(DEST/'sha256.json',{str(f.relative_to(DEST)):sha(f) for f in sorted(DEST.rglob('*')) if f.is_file() and f.name!='sha256.json'})


def main():
    p=argparse.ArgumentParser();p.add_argument('action',choices=['prepare','run','publish']);p.add_argument('coord',type=Path);p.add_argument('--parent',type=Path);a=p.parse_args()
    if a.action=='prepare':prepare(a.coord,a.parent)
    elif a.action=='run':V25(a.coord).execute_trial()
    else:V25(a.coord).publish_trial()

if __name__=='__main__':main()
