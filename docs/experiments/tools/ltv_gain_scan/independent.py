#!/usr/bin/env python3
"""EuRoC independent G/V information weights; historical results are never rerun."""
import argparse
from concurrent.futures import ThreadPoolExecutor
from decimal import Decimal
import hashlib
import json
import os
from pathlib import Path
import queue
import shutil
import subprocess
import time
import numpy as np
from study import Study, ROOT, TOOLS, EUROC, set_keys, write, table_csv, sha, process, share_calibration_evidence
from run_euroc_six import verify_output

TIERS = [1, 2, 3, 4, 5, 6, 8]
NEW = [3, 5, 6, 8]
EXTRA = [10, 16]
REP = ['V2_02_medium', 'V2_03_difficult', 'MH_05_difficult']
DEST = ROOT / 'docs/euroc_tune_results/gv_independent'
REPORT = ROOT / 'docs/euroc_tune_results.md'


def key(row):
    return row['sequence'], row['mode'], row['alpha_gravity'], row['alpha_velocity']


def label(mode, g, v):
    return f'{mode}_G{g}_V{v}'


def weights(mode, g, v):
    if mode not in ['OFF', 'G', 'V', 'GV']:
        raise ValueError('unsupported mode')
    if (g > 0) != (mode in ['G', 'GV']) or (v > 0) != (mode in ['V', 'GV']):
        raise ValueError('disabled branch must have zero weight identity')
    allowed = TIERS if mode == 'GV' else TIERS + EXTRA
    allowed_velocity = allowed + ([25,36,49] if mode == 'V' else [])
    if g not in [0] + allowed or v not in [0] + allowed_velocity:
        raise ValueError('outside declared grid')
    return dict(ltv_sigma_gravity_deg=10 / np.sqrt(g or 1), ltv_sigma_velocity_mps=1 / np.sqrt(v or 1))


def assess(rows, expected):
    reasons = []
    if len(rows) != len(expected) or {r['sequence'] for r in rows} != set(expected):
        reasons.append('missing or duplicate sequence')
    for r in rows:
        dt = r.get('initialization_delta_s')
        if r.get('status') != 'VALID_FULL_MATCHED' or r.get('coverage') != 1 or dt is None or not np.isfinite(dt) or abs(dt) > 1e-6:
            reasons.append(r['sequence'] + ': invalid runtime/support/initialization')
        for name in ['off_ate', 'reference_ate']:
            b, x = r.get(name), r.get('ate_rmse_m')
            if b is None or x is None or not np.isfinite([b, x]).all() or b <= 0:
                reasons.append(r['sequence'] + ': invalid ' + name)
            elif Decimal(str(x)) > Decimal(str(b)) * Decimal('1.1'):
                reasons.append(r['sequence'] + ': >10% above ' + name)
    return dict(status='BLOCKED' if reasons else 'PASS', reasons=reasons)


def prepare(coord):
    coord = coord.resolve()
    if coord.exists():
        raise ValueError('use a new coordination directory')
    coord.mkdir(parents=True)
    for name in ['registry', 'contracts', 'tools', 'specs', 'support', 'configs', 'locks']:
        (coord / name).mkdir()
    old = ROOT / 'docs/euroc_tune_results'
    identity = json.loads((old / 'opencv_serial_fix_identity.json').read_text())
    binary = Path(identity['artifact']) / 'install/ov_msckf/lib/ov_msckf/run_ltv_gv_production'
    if not binary.is_file(): raise ValueError('verified single-thread artifact missing')
    delta = subprocess.check_output(['git', 'diff', identity['source_oid'], 'HEAD', '--', 'ov_core/src', 'ov_init/src', 'ov_msckf/src', 'config', 'docs/experiments/tools/native/run_ltv_feature_passive.cpp'], cwd=ROOT)
    if delta: raise ValueError('estimator/driver differs from verified artifact source')
    identity.update(coord_root=str(coord), binary_sha256=sha(binary), analysis_source_oid=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip())
    write(coord / 'identity.json', identity)
    cpus = sorted(os.sched_getaffinity(0))
    if len(cpus) < 16: raise ValueError('need 16 allowed CPUs for four disjoint workers')
    session = {'tasks': {f'w{i}': dict(task_root=str(coord/f'w{i}'), worktree=str(ROOT), namespace=f'ltv_independent_{coord.name}_w{i}', ros_domain_id=201+i, cpu_affinity=cpus[4*i:4*i+4]) for i in range(4)}}
    for t in session['tasks'].values(): Path(t['task_root']).mkdir()
    write(coord/'session.json', session)
    shared = Path('/home/he/output/ltv_shared_replay_locks');shared.mkdir(parents=True,exist_ok=True)
    for name in ['build_replay.lock','performance_gate.lock'] + [f'slot_{i}.lock' for i in range(4)]:
        (coord/'locks'/name).symlink_to(shared/name)
    for f in TOOLS.glob('*.py'): shutil.copy2(f,coord/'tools'/f.name)
    shutil.copy2(TOOLS/'env_exec.sh',coord/'tools/env_exec.sh')
    (coord/'contracts/empty_colcon_defaults.yaml').write_text('{}\n')
    write(coord/'contracts/protocol.json',dict(tiers=TIERS,new_tiers=NEW,representatives=REP,sequences=EUROC,gate='strict >10% vs OFF and same-mode 1x; 100% frozen support; initialization <=1us',single_eligibility='all ten completed sequences must pass before combination',new_full_maximum=540,historical_reuse=100,opencv_threads_new=1,opencv_threads_historical=4,performance_comparison=False))
    data=json.loads((old/'data_identity.json').read_text()); inputs={s:data[s] for s in EUROC}
    for seq,item in inputs.items():
        manifest=old/'input_hashes'/f'{seq}.json'
        for name,h in json.loads(manifest.read_text()).items():
            if sha(Path(item['root'])/name)!=h:raise ValueError('input changed '+seq+'/'+name)
        if sha(item['gt'])!=item['gt_sha256']:raise ValueError('GT changed '+seq)
        shutil.copy2(old/'support'/f'{seq}.npz',coord/'support'/f'{seq}.npz')
        shutil.copy2(manifest,coord/'support'/f'{seq}_input_hashes.json')
    write(coord/'data_identity.json',inputs)
    rows=[]
    for original in json.loads((old/'results.json').read_text()):
        if original['sequence'] not in EUROC:continue
        mode,alpha=original['mode'],original['alpha']
        if mode not in ['OFF','G','V','GV'] or alpha not in [1,2,4]:continue
        row=dict(original,alpha_gravity=alpha if mode in ['G','GV'] else 0,alpha_velocity=alpha if mode in ['V','GV'] else 0,historical_reuse=True,historical_result_sha256=sha(old/'results.json'))
        rows.append(row)
    assert len(rows)==100 and len({key(r) for r in rows})==100
    write(coord/'index.json',rows);write(coord/'screening.json',[])
    base=old/'configs/euroc/OFF_1'
    for mode in ['G','V','GV']:
        pairs=[(a,0) for a in TIERS] if mode=='G' else [(0,a) for a in TIERS] if mode=='V' else [(g,v) for g in TIERS for v in TIERS]
        for g,v in pairs:
            dest=coord/'configs'/label(mode,g,v);dest.mkdir()
            for f in base.iterdir():
                if f.is_file():shutil.copy2(f,dest/f.name)
            config = dest/'estimator_config.yaml'
            config.chmod(config.stat().st_mode | 0o200)
            text=config.read_text()
            text=set_keys(text,{**weights(mode,g,v),'num_opencv_threads':1,'ltv_enable_gravity':'false','ltv_enable_velocity':'false','ltv_enable_landmark_approx':'false','max_slam':0})
            (dest/'estimator_config.yaml').write_text(text)
    for folder in [coord/'configs',coord/'support']:
        for f in folder.rglob('*'):
            if f.is_file():f.chmod(f.stat().st_mode & ~0o222)
    write(coord/'analysis_identity.json',{str(p):sha(p) for p in [Path(__file__).resolve(),ROOT/'docs/experiments/tools/ltv_gain_scan/study.py',TOOLS/'launcher.py']})
    print('prepared',coord,flush=True)


class Independent(Study):
    def lookup(self,seq,mode,g=0,v=0):
        return next((r for r in self.index if key(r)==(seq,mode,g,v)),None)

    def comparison(self,seq,mode,g,v):
        row=dict(self.lookup(seq,mode,g,v) or dict(sequence=seq,status='MISSING'))
        off=self.lookup(seq,'OFF');ref=self.lookup(seq,mode,1 if mode in ['G','GV'] else 0,1 if mode in ['V','GV'] else 0)
        row['off_ate']=off.get('ate_rmse_m') if off else None;row['reference_ate']=ref.get('ate_rmse_m') if ref else None
        return row

    def run_one(self,job,worker):
        seq,mode,g,v,short,phase=job
        config=self.coord/'configs'/label(mode,g,v)/'estimator_config.yaml'
        install=Path(self.identity['artifact'])/'install';binary=install/'ov_msckf/lib/ov_msckf/run_ltv_gv_production'
        command=['bash',str(self.coord/'tools/env_exec.sh'),str(install),str(binary),str(config),self.inputs[seq]['root'],'{RUN_DIR}',mode]
        if short:command.append(str(short))
        spec=dict(coord_root=str(self.coord),task=f'w{worker}',label=f'{phase}_{seq}_{label(mode,g,v)}',kind='replay',four_worker_pool=True,worker_pool_size=len(json.loads((self.coord/'session.json').read_text())['tasks']),source_oid=self.identity['source_oid'],source_snapshot=self.identity['source'],command=command,binary_paths=[str(binary)],config_paths=[str(f) for f in config.parent.iterdir() if f.is_file()],configuration=dict(sequence=seq,mode=mode,alpha_gravity=g,alpha_velocity=v,short_seconds=short,phase=phase),data=self.inputs[seq],diagnostic_level='scalar_ltv_csv')
        path=self.coord/'specs'/f'{phase}_{seq}_{label(mode,g,v)}.json';write(path,spec)
        subprocess.run(['python3',str(self.coord/'tools/launcher.py'),str(path)],stdout=subprocess.DEVNULL,check=False)
        events=[json.loads(l) for l in (self.coord/f'registry/task_w{worker}.jsonl').read_text().splitlines()]
        event=next(e for e in reversed(events) if e.get('spec_path')==str(path) and e['event'] in ['FINISH','LAUNCHER_FAILURE'])
        out=Path(event.get('output_dir',''))
        row=dict(sequence=seq,mode=mode,alpha_gravity=g,alpha_velocity=v,historical_reuse=False,phase=phase,run_id=event['run_id'],output_dir=str(out),exit_code=event.get('exit_code'),audit=verify_output(out,mode,not short) if event.get('exit_code')==0 else {'errors':['runtime failure']})
        if row['exit_code']==0:
            options=json.loads((out/'effective_options.json').read_text());thread=json.loads((out/'threading.json').read_text());row['threading']=thread
            for name,wanted in weights(mode,g,v).items():
                if not np.isclose(options[name],wanted,rtol=1e-14,atol=0):row['audit']['errors'].append('weight mismatch '+name)
            if thread.get('actual_opencv_threads')!=1:row['audit']['errors'].append('thread mismatch')
            if event.get('source_mutation') or event.get('binary_mutation'):row['audit']['errors'].append('artifact mutation')
        if not short:row.update(self.evaluate(row))
        return row

    def batch(self,jobs):
        jobs=list(jobs)
        if not jobs:return []
        work=queue.Queue();done=queue.Queue()
        for job in jobs:work.put(job)
        def worker(number):
            while True:
                try:job=work.get_nowait()
                except queue.Empty:return
                row=self.run_one(job,number);done.put(row)
        rows=[]
        worker_count = len(json.loads((self.coord/'session.json').read_text())['tasks'])
        with ThreadPoolExecutor(max_workers=worker_count) as pool:
            futures=[pool.submit(worker,i) for i in range(worker_count)]
            while len(rows)<len(jobs):
                try:row=done.get(timeout=1)
                except queue.Empty:
                    for f in futures:
                        if f.done():f.result()
                    continue
                rows.append(row)
                if not jobs[0][4]:self.index.append(row);write(self.coord/'index.json',self.index)
                print(json.dumps({k:row.get(k) for k in ['sequence','mode','alpha_gravity','alpha_velocity','status','ate_rmse_m']}) ,flush=True)
            for f in futures:f.result()
        return rows

    def jobs(self,seqs,mode,g,v,phase):
        return [(s,mode,g,v,None,phase) for s in seqs if self.lookup(s,mode,g,v) is None]

    def screen(self,seqs,mode,g,v,stage):
        rows=[self.comparison(s,mode,g,v) for s in seqs]
        result=dict(mode=mode,alpha_gravity=g,alpha_velocity=v,stage=stage,**assess(rows,seqs),sequences=rows)
        self.screening=[s for s in self.screening if (s['mode'],s['alpha_gravity'],s['alpha_velocity'],s['stage'])!=(mode,g,v,stage)]+[result]
        write(self.coord/'screening.json',self.screening);return result

    def block(self,seqs,mode,g,v,reason):
        for seq in seqs:
            if self.lookup(seq,mode,g,v) is None:self.index.append(dict(sequence=seq,mode=mode,alpha_gravity=g,alpha_velocity=v,status='BLOCKED',reason=reason,ate_rmse_m=None,historical_reuse=False))
        write(self.coord/'index.json',self.index)

    def execute(self):
        if (self.coord/'isolation_8_verified.json').exists():
            isolation=json.loads((self.coord/'isolation_8_verified.json').read_text())
            if isolation['status']!='PASS':raise RuntimeError('eight-worker isolation check failed')
        if not (self.coord/'smoke.json').exists():
            rows=self.batch([(REP[0],'G',3,0,40,'smoke'),(REP[0],'V',0,5,40,'smoke'),(REP[0],'GV',5,6,40,'smoke')])
            write(self.coord/'smoke.json',rows)
        smoke=json.loads((self.coord/'smoke.json').read_text())
        if len(smoke)!=3 or any(r['exit_code'] or r['audit']['errors'] for r in smoke):raise RuntimeError('smoke failed')
        self.batch([j for mode in ['G','V'] for a in NEW for j in self.jobs(REP,mode,a if mode=='G' else 0,a if mode=='V' else 0,'single_representative')])
        extending=[]
        for mode in ['G','V']:
            for a in NEW:
                g,v=(a,0) if mode=='G' else (0,a)
                gate=self.screen(REP,mode,g,v,'representative')
                if gate['status']=='PASS':extending+=self.jobs([s for s in EUROC if s not in REP],mode,g,v,'single_full')
                else:self.block(EUROC,mode,g,v,'single representative gate blocked')
        self.publish();self.batch(extending)
        safe={mode:[] for mode in ['G','V']}
        for mode in safe:
            for a in TIERS:
                g,v=(a,0) if mode=='G' else (0,a)
                if self.screen(EUROC,mode,g,v,'single_eligibility')['status']=='PASS':safe[mode].append(a)
        candidates=[(g,v) for g in safe['G'] for v in safe['V']]
        write(self.coord/'eligible_grid.json',dict(gravity=safe['G'],velocity=safe['V'],pairs=candidates))
        self.batch([j for g,v in candidates for j in self.jobs(REP,'GV',g,v,'joint_representative')]);self.publish()
        extending=[]
        for g in TIERS:
            for v in TIERS:
                if (g,v) not in candidates:self.block(EUROC,'GV',g,v,'single branch ineligible');continue
                result=self.screen(REP,'GV',g,v,'representative')
                if result['status']=='PASS':extending+=self.jobs([s for s in EUROC if s not in REP],'GV',g,v,'joint_full')
                else:self.block(EUROC,'GV',g,v,'joint representative gate blocked')
        self.publish();self.batch(extending)
        expected=680 if (self.coord/'extra_single_plan.json').exists() else 640
        assert len(self.index)==expected and len({key(r) for r in self.index})==expected
        live=[]
        for f in (self.coord/'registry').glob('*.jsonl'):
            for line in f.read_text().splitlines():
                e=json.loads(line)
                if e['event']=='PROCESS_REGISTERED':
                    now=process(e['pid'])
                    if now and now['state']!='Z' and all(now[k]==e[k] for k in ['pgid','start_ticks','executable']):live.append(now)
        write(self.coord/'completion_audit.json',dict(recorded=len(self.index),historical=sum(r['historical_reuse'] for r in self.index),new_full=sum('run_id' in r and not r['historical_reuse'] for r in self.index),owned_live_processes=live))
        if live:raise RuntimeError('owned process remains')
        self.publish()

    def extra(self):
        # Explicit user amendment: high single weights only, no larger GV grid.
        if not (self.coord/'completion_audit.json').exists():
            raise RuntimeError('finish the seven-tier stage first')
        write(self.coord/'extra_single_plan.json',dict(tiers=EXTRA,modes=['G','V'],representatives=REP,maximum_new_full=40,joint_grid_unchanged=True))
        for mode in ['G','V']:
            for a in EXTRA:
                g,v=(a,0) if mode=='G' else (0,a)
                dest=self.coord/'configs'/label(mode,g,v)
                if not dest.exists():
                    shutil.copytree(self.coord/'configs'/label(mode,1 if mode=='G' else 0,1 if mode=='V' else 0),dest)
                    config=dest/'estimator_config.yaml';config.chmod(config.stat().st_mode|0o200)
                    config.write_text(set_keys(config.read_text(),weights(mode,g,v)))
                    for f in dest.iterdir():
                        if f.is_file():f.chmod(f.stat().st_mode & ~0o222)
        self.batch([j for mode in ['G','V'] for a in EXTRA for j in self.jobs(REP,mode,a if mode=='G' else 0,a if mode=='V' else 0,'extra_single_representative')])
        extending=[]
        for mode in ['G','V']:
            for a in EXTRA:
                g,v=(a,0) if mode=='G' else (0,a)
                if self.screen(REP,mode,g,v,'representative')['status']=='PASS':
                    extending+=self.jobs([s for s in EUROC if s not in REP],mode,g,v,'extra_single_full')
                else:self.block(EUROC,mode,g,v,'extra single representative gate blocked')
        self.publish();self.batch(extending)
        for mode in ['G','V']:
            for a in EXTRA:
                self.screen(EUROC,mode,a if mode=='G' else 0,a if mode=='V' else 0,'extra_single_full_check')
        assert len(self.index)==680 and len({key(r) for r in self.index})==680
        write(self.coord/'extra_completion_audit.json',dict(recorded=680,historical=100,new_full=sum(not r['historical_reuse'] and 'run_id' in r for r in self.index),joint_grid_unchanged=True))
        self.publish()

    def publish(self):
        DEST.mkdir(parents=True,exist_ok=True)
        single_tiers = TIERS + EXTRA if (self.coord/'extra_single_plan.json').exists() else TIERS
        planned=640+(40 if len(single_tiers)>7 else 0)
        rows=[]
        for r in self.index:
            row=dict(r);x=r.get('ate_rmse_m');off=self.lookup(r['sequence'],'OFF')['ate_rmse_m']
            row['relative_off_percent']=100*(x/off-1) if x is not None else None
            rows.append(row)
        rows.sort(key=lambda r:(EUROC.index(r['sequence']),r['mode'],r['alpha_gravity'],r['alpha_velocity']))
        write(DEST/'results.json',rows);table_csv(DEST/'results.csv',rows);write(DEST/'screening.json',self.screening)
        for name in ['identity.json','data_identity.json','analysis_identity.json','completion_audit.json','independent_audit.json','smoke.json','eligible_grid.json','session.json','session_four_workers.json','eight_worker_switch.json','eight_worker_recovery.json','isolation_8_verified.json','eight_worker_analysis_identity.json','protocol_initial.json','extra_request.json','extra_single_plan.json','extra_completion_audit.json']:
            if (self.coord/name).exists():shutil.copy2(self.coord/name,DEST/name)
        shutil.copy2(self.coord/'contracts/protocol.json',DEST/'protocol.json')
        (DEST/'configs').mkdir(exist_ok=True)
        for source in (self.coord/'configs').iterdir():
            target=DEST/'configs'/source.name
            if not target.exists():shutil.copytree(source,target)
            else:
                for f in source.iterdir():
                    if f.is_file() and sha(f)!=sha(target/f.name):raise ValueError('published configuration differs')
        if not (DEST/'support').exists():shutil.copytree(self.coord/'support',DEST/'support')
        manifests=[json.loads(p.read_text()) for p in self.coord.glob('w*/results/*/manifest_finish.json')];write(DEST/'run_manifest.json',manifests)
        text=['# EuRoC G/V 权重实验：七档网格与追加高档单分支','', '排除 MH_04；G/V 权重独立；候选为 1、2、3、4、5、6、8。原始残差、Jacobian、Observer、门控与更新顺序不变，landmark 关闭。',
              '1/2/4 档与原对角 GV 复用历史数据，不重复回放；新档实际 OpenCV 单线程，历史为四线程。位置支持相同，七组先前线程修正烟测字节一致；不混用耗时做性能比较。',
              '筛选采用相对 OFF/同模式 1× ATE 严格升高超过 10% 阻止扩展；PASS 仅允许扩展，历史预测 FAIL、工程结论与相关性缺口保留。',
              f'当前登记 {len(rows)}/{planned} 个配置，历史复用 {sum(r["historical_reuse"] for r in rows)}；新完整回放 {sum("run_id" in r and not r["historical_reuse"] for r in rows)}。','',
              '| 信息权重 | '+' | '.join(f'{a}×' for a in single_tiers)+' |','|---|'+'---:|'*len(single_tiers),
              '| G 标准差(°) | '+' | '.join(f'{10/np.sqrt(a):.6f}' for a in single_tiers)+' |',
              '| V 标准差(m/s) | '+' | '.join(f'{1/np.sqrt(a):.6f}' for a in single_tiers)+' |']
        def cell(seq,mode,g,v):
            r=self.lookup(seq,mode,g,v)
            if r is None:return 'PENDING'
            x=r.get('ate_rmse_m')
            if x is None:return r['status']
            b=self.lookup(seq,'OFF')['ate_rmse_m'];delta=100*(x/b-1);value=f'{x:.9f} ({delta:+.6f}%)'
            return '**'+value+'**' if delta<0 else value
        for mode in ['G','V']:
            text+=['','## '+mode+' 单分支档位','', 'ATE(m)，括号为相对 OFF；改善值加粗。','', '| 序列 | OFF | '+' | '.join(f'{a}×' for a in single_tiers)+' |','|---|'+'---:|'*(len(single_tiers)+1)]
            for seq in EUROC:text.append('| '+seq+' | '+f'{self.lookup(seq,"OFF")["ate_rmse_m"]:.9f}'+' | '+' | '.join(cell(seq,mode,a if mode=='G' else 0,a if mode=='V' else 0) for a in single_tiers)+' |')
        if len(single_tiers)>7:
            text+=['', '用户追加 10×/16× G/V 单分支，最多 40 次新回放；以下 GV 网格仍限于原七档，没有自动扩展到九档组合。']
        text+=['','## GV 非对称权重网格','', '行为 G 信息权重、列为 V 信息权重；每格 ATE(m) 与相对 OFF 百分比。BLOCKED 表示未扩展，已完成代表序列的实测值仍保留。']
        for seq in EUROC:
            text+=['','### '+seq,'','| G＼V | '+' | '.join(f'{a}×' for a in TIERS)+' |','|---|'+'---:|'*7]
            for g in TIERS:text.append('| '+str(g)+'× | '+' | '.join(cell(seq,'GV',g,v) for v in TIERS)+' |')
        text+=['','## 描述性分析','', '下表对十条序列的相对 OFF 变化等权平均；负值为改善，不能解释为合并轨迹 RMSE。只对十条均有效的配置作汇总。','', '| 配置参数 | 平均变化 | 中位数 | 改善/退化/相同 | 最差变化 | 相对对应 V 平均变化 |','|---|---:|---:|---:|---:|---:|']
        summaries=[]
        for mode in ['G','V','GV']:
            pairs=[(a,0) for a in single_tiers] if mode=='G' else [(0,a) for a in single_tiers] if mode=='V' else [(g,v) for g in TIERS for v in TIERS]
            for g,v in pairs:
                a=[self.lookup(s,mode,g,v) for s in EUROC]
                if any(r is None or r.get('status')!='VALID_FULL_MATCHED' for r in a):continue
                d=[100*(r['ate_rmse_m']/self.lookup(r['sequence'],'OFF')['ate_rmse_m']-1) for r in a]
                dv=[100*(r['ate_rmse_m']/self.lookup(r['sequence'],'V',0,v)['ate_rmse_m']-1) for r in a] if mode=='GV' else []
                summary=dict(mode=mode,alpha_gravity=g,alpha_velocity=v,mean_relative_off_percent=float(np.mean(d)),median_relative_off_percent=float(np.median(d)),improved=sum(x<0 for x in d),worse=sum(x>0 for x in d),equal=sum(x==0 for x in d),worst_percent=max(d),mean_vs_corresponding_v=float(np.mean(dv)) if dv else None)
                summaries.append(summary)
                name=label(mode,g,v);mean=summary['mean_relative_off_percent'];bold=lambda t:'**'+t+'**' if mean<0 else t
                text.append(f'| {bold(name)} | {bold(f"{mean:+.6f}%")} | {summary["median_relative_off_percent"]:+.6f}% | {summary["improved"]}/{summary["worse"]}/{summary["equal"]} | {max(d):+.6f}% | '+(f'{np.mean(dv):+.6f}%' if dv else '—')+' |')
        write(DEST/'summary.json',summaries)
        for mode in ['G','V','GV']:
            options=[r for r in summaries if r['mode']==mode]
            if options:
                best=min(options,key=lambda r:r['mean_relative_off_percent']);text+=['',f'{mode} 当前完整有效配置中，等权平均 ATE 最好的参数为 **{label(mode,best["alpha_gravity"],best["alpha_velocity"])}**，平均变化 {best["mean_relative_off_percent"]:+.6f}%，改善 {best["improved"]}/10 条。这是本数据集扫描结果，不是独立留出验证或生产推荐。']
        notes=DEST/'analysis_notes.md'
        if notes.exists():text+=['',notes.read_text().rstrip()]
        text+=['','## 身份与证据','',f'新回放源码 `{self.identity["source_oid"]}`；原始记录 `{self.coord}`。', '[全部配置](euroc_tune_results/gv_independent/results.json) · [筛选决定](euroc_tune_results/gv_independent/screening.json) · [审计](euroc_tune_results/gv_independent/independent_audit.json) · [协议](euroc_tune_results/gv_independent/protocol.json)','']
        v25_section=ROOT/'docs/euroc_tune_results/v25/section.md'
        if v25_section.exists():text+=['',v25_section.read_text().rstrip(),'']
        landmark_section=ROOT/'docs/euroc_tune_results/landmark/section.md'
        if landmark_section.exists():text+=['',landmark_section.read_text().rstrip(),'']
        high_section=ROOT/'docs/euroc_tune_results/v36_v49/section.md'
        if high_section.exists():text+=['',high_section.read_text().rstrip(),'']
        combination_section=ROOT/'docs/euroc_tune_results/v49_combinations/section.md'
        if combination_section.exists():text+=['',combination_section.read_text().rstrip(),'']
        REPORT.write_text('\n'.join(text));write(DEST/'sha256.json',{str(f.relative_to(DEST)):sha(f) for f in sorted(DEST.rglob('*')) if f.is_file() and f.name!='sha256.json'})


def main():
    p=argparse.ArgumentParser();p.add_argument('action',choices=['prepare','run','publish','extra']);p.add_argument('coord',type=Path);a=p.parse_args()
    if a.action=='prepare':prepare(a.coord)
    elif a.action=='run':Independent(a.coord).execute()
    elif a.action=='extra':Independent(a.coord).extra()
    else:Independent(a.coord).publish()

if __name__=='__main__':main()
