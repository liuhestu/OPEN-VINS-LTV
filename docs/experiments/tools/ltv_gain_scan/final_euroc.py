#!/usr/bin/env python3
"""Frozen scene/mode confirmation; new matched trajectories and isolated serial costs."""
import argparse
import csv
import json
from pathlib import Path
import shutil
import subprocess
import numpy as np
from scipy.spatial.transform import Rotation
from independent import Independent, ROOT, TOOLS, EUROC, write, set_keys, table_csv, sha, process
from evaluate_euroc_six import nearest, aligned_rmse, load_trajectory
from run_euroc_six import verify_output

MODES=['OFF','G','V','L','GVL']
PROFILES={'MH':{'OFF':(0,0,0),'G':(16,0,0),'V':(0,49,0),'L':(0,0,6),'GVL':(2,49,6)},'Vicon':{'OFF':(0,0,0),'G':(1,0,0),'V':(0,6,0),'L':(0,0,6),'GVL':(1,1,1)}}
DEST=ROOT/'docs/euroc_final_results_data'
REPORT=ROOT/'docs/euroc_final_results.md'


def scene(seq):return 'MH' if seq.startswith('MH') else 'Vicon'

def prepare(coord,parent):
    if coord.exists():raise ValueError('new directory required')
    coord.mkdir(parents=True)
    for n in ['registry','contracts','tools','specs','support','configs','locks']:(coord/n).mkdir()
    identity=json.loads((parent/'identity.json').read_text());identity['coord_root']=str(coord);write(coord/'identity.json',identity)
    binary=Path(identity['artifact'])/'install/ov_msckf/lib/ov_msckf/run_ltv_gv_production';assert binary.is_file()
    delta=subprocess.check_output(['git','diff',identity['source_oid'],'HEAD','--','ov_core/src','ov_init/src','ov_msckf/src','config','docs/experiments/tools/native'],cwd=ROOT);assert not delta
    tasks=json.loads((parent/'session.json').read_text())['tasks']
    for n,t in tasks.items():t.update(task_root=str(coord/n),namespace=f'ltv_final_{coord.name}_{n}');(coord/n).mkdir()
    write(coord/'session.json',{'tasks':tasks})
    shared=Path('/home/he/output/ltv_shared_replay_locks')
    for n in ['build_replay.lock','performance_gate.lock']+[f'slot_{i}.lock' for i in range(8)]:(coord/'locks'/n).symlink_to(shared/n)
    for f in TOOLS.glob('*.py'):shutil.copy2(f,coord/'tools'/f.name)
    shutil.copy2(TOOLS/'env_exec.sh',coord/'tools/env_exec.sh');(coord/'contracts/empty_colcon_defaults.yaml').write_text('{}\n')
    write(coord/'contracts/protocol.json',dict(profiles=PROFILES,sequences=EUROC,excluded={'MH_04_difficult':'User previously excluded baseline divergence attributed to initial stationary segment; not in selection scope'},mode_order=MODES,accuracy='50 fresh full runs; OFF first; fixed support; no selection after freeze',performance='50 separate full serial runs, exclusive performance lock, w0 fixed two-CPU affinity, same diagnostics; one paired run per sequence/mode; no independent timing repeats',metrics='SE3 no scale ATE; body translation and rotation RPE at 1/5s, nearest endpoint <=25ms; OFF fixed10s windows; position p95/p99/max and last10% RMSE',gt_orientation_note='V1_01 GT orientation provenance not verified as corrected; attitude-dependent RPE descriptive only, not acceptance',threads=1,workers_accuracy=8,workers_performance=1,acceptance='historical thresholds preserved; selected-scene/mode confirmation, not strict same-weight ablation or held-out validation'))
    shutil.copy2(ROOT/'docs/ltv/three_track_validation/contracts/acceptance.yaml',coord/'contracts/historical_acceptance.json')
    inputs=json.loads((parent/'data_identity.json').read_text());write(coord/'data_identity.json',inputs)
    for seq,item in inputs.items():
        assert sha(item['gt'])==item['gt_sha256']
        manifest=parent/'support'/f'{seq}_input_hashes.json'
        for n,h in json.loads(manifest.read_text()).items():assert sha(Path(item['root'])/n)==h
        shutil.copy2(manifest,coord/'support'/manifest.name)
    base=parent/'configs/GV'
    for family,profiles in PROFILES.items():
        for mode,(g,v,l) in profiles.items():
            dest=coord/'configs'/family/mode;shutil.copytree(base,dest);p=dest/'estimator_config.yaml';p.chmod(p.stat().st_mode|0o200)
            p.write_text(set_keys(p.read_text(),dict(ltv_sigma_gravity_deg=10/np.sqrt(g or 1),ltv_sigma_velocity_mps=1/np.sqrt(v or 1),ltv_landmark_approx_sigma_floor_m=.5/np.sqrt(l or 1),ltv_landmark_approx_information_cap=.01*(l or 1),ltv_landmark_approx_max_points=2,ltv_enable_gravity='false',ltv_enable_velocity='false',ltv_enable_landmark_approx='false',max_slam=0,num_opencv_threads=1)));p.chmod(0o444)
    write(coord/'index.json',[]);write(coord/'screening.json',[]);write(coord/'performance_index.json',[])
    write(coord/'analysis_identity.json',{str(f):sha(f) for f in [Path(__file__).resolve(),ROOT/'docs/experiments/tools/ltv_gain_scan/independent.py',TOOLS/'launcher.py']});print('prepared',coord,flush=True)


class Final(Independent):
    def lookup_mode(self,seq,mode):return next((r for r in self.index if r['sequence']==seq and r['mode']==mode),None)

    def run_one(self,job,worker):
        seq,mode,g,v,short,phase=job;family=scene(seq);g,v,l=PROFILES[family][mode];native='L_GV' if mode=='GVL' else mode;config=self.coord/'configs'/family/mode/'estimator_config.yaml';install=Path(self.identity['artifact'])/'install';binary=install/'ov_msckf/lib/ov_msckf/run_ltv_gv_production'
        cmd=['bash',str(self.coord/'tools/env_exec.sh'),str(install),str(binary),str(config),self.inputs[seq]['root'],'{RUN_DIR}',native]
        if short:cmd.append(str(short))
        spec=dict(coord_root=str(self.coord),task=f'w{worker}',label=f'{phase}_{seq}_{mode}',kind='replay',four_worker_pool=True,worker_pool_size=8,performance=phase=='serial_performance',source_oid=self.identity['source_oid'],source_snapshot=self.identity['source'],command=cmd,binary_paths=[str(binary)],config_paths=[str(f) for f in config.parent.iterdir() if f.is_file()],configuration=dict(sequence=seq,scene=family,mode=mode,native_mode=native,gravity=g,velocity=v,landmark=l,short_seconds=short,phase=phase),data=self.inputs[seq],diagnostic_level='scalar_ltv_csv')
        path=self.coord/'specs'/f'{phase}_{seq}_{mode}.json';write(path,spec);subprocess.run(['python3',str(self.coord/'tools/launcher.py'),str(path)],stdout=subprocess.DEVNULL,check=False)
        events=[json.loads(s) for s in (self.coord/f'registry/task_w{worker}.jsonl').read_text().splitlines()];m=next(e for e in reversed(events) if e.get('spec_path')==str(path) and e['event'] in ['FINISH','LAUNCHER_FAILURE']);out=Path(m.get('output_dir',''))
        row=dict(sequence=seq,scene=family,mode=mode,native_mode=native,alpha_gravity=g,alpha_velocity=v,alpha_landmark=l,historical_reuse=False,run_id=m['run_id'],output_dir=str(out),exit_code=m.get('exit_code'),phase=phase,audit=verify_output(out,native,not short) if m.get('exit_code')==0 else {'errors':['runtime failure']})
        if row['exit_code']==0:
            opt=json.loads((out/'effective_options.json').read_text());thread=json.loads((out/'threading.json').read_text());row['threading']=thread
            expected=dict(ltv_sigma_gravity_deg=10/np.sqrt(g or 1),ltv_sigma_velocity_mps=1/np.sqrt(v or 1),ltv_landmark_approx_sigma_floor_m=.5/np.sqrt(l or 1),ltv_landmark_approx_information_cap=.01*(l or 1),ltv_landmark_approx_max_points=2)
            if any(not np.isclose(opt[n],x,rtol=1e-14,atol=0) for n,x in expected.items()):row['audit']['errors'].append('weight mismatch')
            if thread['actual_opencv_threads']!=1:row['audit']['errors'].append('thread mismatch')
        if not short:row.update(self.evaluate(row))
        return row

    def execute(self):
        if not (self.coord/'smoke.json').exists():
            jobs=[(seq,mode,*PROFILES[scene(seq)][mode][:2],40,'smoke') for seq in ['MH_05_difficult','V2_02_medium'] for mode in MODES]
            write(self.coord/'smoke.json',self.batch(jobs))
        if any(r['exit_code'] or r['audit']['errors'] for r in json.loads((self.coord/'smoke.json').read_text())):raise RuntimeError('smoke failed')
        self.batch([(s,'OFF',0,0,None,'accuracy') for s in EUROC if not self.lookup_mode(s,'OFF')])
        jobs=[]
        for seq in EUROC:
            for mode in MODES[1:]:
                if self.lookup_mode(seq,mode):continue
                if self.lookup_mode(seq,'OFF')['status']!='VALID_FULL_MATCHED':self.index.append(dict(sequence=seq,mode=mode,status='SKIPPED',reason='fresh OFF invalid',ate_rmse_m=None))
                else:jobs.append((seq,mode,*PROFILES[scene(seq)][mode][:2],None,'accuracy'))
        self.batch(jobs);write(self.coord/'index.json',self.index);self.publish()
        performance=json.loads((self.coord/'performance_index.json').read_text())
        # Full runs, serial, alternate forward/reverse mode order by sequence.
        for i,seq in enumerate(EUROC):
            for mode in (MODES if i%2==0 else list(reversed(MODES))):
                if any(r['sequence']==seq and r['mode']==mode for r in performance):continue
                if self.lookup_mode(seq,mode)['status']!='VALID_FULL_MATCHED':continue
                row=self.run_one((seq,mode,*PROFILES[scene(seq)][mode][:2],None,'serial_performance'),0)
                if row['exit_code']==0:
                    d=Path(row['output_dir']);timing=np.loadtxt(d/'frame_processing.csv',delimiter=',',skiprows=1,ndmin=2)[:,1];resources=json.loads((d/'resources.json').read_text());row['cost']=dict(camera_api_p50_ms=float(np.quantile(timing,.5)),camera_api_p95_ms=float(np.quantile(timing,.95)),camera_api_p99_ms=float(np.quantile(timing,.99)),camera_api_max_ms=float(timing.max()),full_wall_s=resources['elapsed_s'],full_cpu_s=resources['user_s']+resources['system_s'],peak_rss_kb=resources['max_rss_kb'],timing_samples=len(timing))
                    first=Path(self.lookup_mode(seq,mode)['output_dir']);row['trajectory_identical_to_parallel']=(first/'trajectory.csv').read_bytes()==(d/'trajectory.csv').read_bytes()
                performance.append(row);write(self.coord/'performance_index.json',performance);print(json.dumps({'serial_done':len(performance),'sequence':seq,'mode':mode,'status':row['status']}),flush=True)
                self.publish()
        self.evaluate_metrics();self.audit();self.publish()

    def evaluate_metrics(self):
        metrics=[];windows=[]
        for seq in EUROC:
            baseline=self.lookup_mode(seq,'OFF')
            if baseline['status']!='VALID_FULL_MATCHED':continue
            frozen=np.load(self.coord/'support'/f'{seq}.npz');times,truth=frozen['times'],frozen['truth'];gt=np.loadtxt(self.inputs[seq]['gt'],delimiter=',',comments='#',ndmin=2);gi=nearest(gt[:,0]*1e-9,times);rg=Rotation.from_quat(gt[gi][:,[5,6,7,4]]);bins=np.floor((times-frozen['initialization'])/10).astype(int)
            for mode in MODES:
                row=self.lookup_mode(seq,mode)
                if row['status']!='VALID_FULL_MATCHED':continue
                x=load_trajectory(Path(row['output_dir'])/'trajectory.csv');xi=nearest(x[:,0],times);est=x[xi];xx,yy=est[:,5:8],truth;u,_,vt=np.linalg.svd((xx-xx.mean(0)).T@(yy-yy.mean(0)));r=vt.T@np.diag([1.,1.,np.linalg.det(vt.T@u.T)])@u.T;aligned=(xx-xx.mean(0))@r.T+yy.mean(0);err=np.linalg.norm(aligned-yy,axis=1);re=Rotation.from_quat(est[:,1:5])
                item=dict(sequence=seq,scene=scene(seq),mode=mode,ate_rmse_m=row['ate_rmse_m'],position_p95_m=float(np.quantile(err,.95)),position_p99_m=float(np.quantile(err,.99)),position_max_m=float(err.max()),support_last10pct_rmse_m=float(np.sqrt(np.mean(err[-max(1,len(err)//10):]**2))),samples=len(err),rotation_truth_status='UNVERIFIED_V1_01_ORIENTATION' if seq=='V1_01_easy' else 'existing_frozen_GT')
                for dt in [1,5]:
                    end=nearest(times,times+dt);ok=np.abs(times[end]-(times+dt))<=.025;a=np.flatnonzero(ok);b=end[ok]
                    de=re[a].inv().apply(est[b,5:8]-est[a,5:8]);dg=rg[a].inv().apply(truth[b]-truth[a]);rr=((rg[a].inv()*rg[b]).inv()*(re[a].inv()*re[b])).magnitude()
                    item[f'rpe_translation_{dt}s_m']=float(np.sqrt(np.mean(np.linalg.norm(de-dg,axis=1)**2))) if len(a) else None;item[f'rpe_rotation_{dt}s_deg']=float(np.sqrt(np.mean(np.degrees(rr)**2))) if len(a) else None;item[f'rpe_samples_{dt}s']=len(a)
                metrics.append(item)
                for bucket in np.unique(bins):
                    mask=bins==bucket;windows.append(dict(sequence=seq,mode=mode,bin=int(bucket),start_s=float(frozen['initialization']+bucket*10),samples=int(mask.sum()),ate_rmse_m=float(np.sqrt(np.mean(err[mask]**2)))))
        write(self.coord/'metrics.json',metrics);write(self.coord/'local_windows.json',windows)

    def audit(self):
        errors=[];performance=json.loads((self.coord/'performance_index.json').read_text())
        if len(self.index)!=50 or len({(r['sequence'],r['mode']) for r in self.index})!=50:errors.append('accuracy identity count')
        for row in self.index+performance:
            if 'output_dir' not in row:continue
            d=Path(row['output_dir']);m=json.loads((d/'manifest_finish.json').read_text())
            if verify_output(d,row['native_mode'],True)['errors']:errors.append('input/flags/receipts')
            check=self.evaluate(row)
            if check['status']!=row['status'] or (row['status']=='VALID_FULL_MATCHED' and not np.isclose(check['ate_rmse_m'],row['ate_rmse_m'],rtol=1e-12,atol=0)):errors.append('ATE mismatch')
            for category in ['binary_paths','config_paths']:
                for p,value in m[category].items():
                    if sha(p)!=value['sha256']:errors.append('artifact mutation')
            if row['phase']=='serial_performance' and not row.get('trajectory_identical_to_parallel'):errors.append('serial/parallel trajectory changed')
        ms=[json.loads(p.read_text()) for p in self.coord.glob('w*/results/*/manifest_finish.json')];serial=[m for m in ms if m.get('configuration',{}).get('phase')=='serial_performance']
        for s in serial:
            if not s['performance'] or s['cpu_affinity']!=[0,1]:errors.append('serial measurement not fixed/exclusive')
            if any(o['run_id']!=s['run_id'] and o['started_at']<s['finished_at'] and s['started_at']<o['finished_at'] for o in ms):errors.append('serial workload overlap')
        live=[]
        for f in (self.coord/'registry').glob('*.jsonl'):
            for line in f.read_text().splitlines():
                e=json.loads(line)
                if e['event']=='PROCESS_REGISTERED':
                    now=process(e['pid'])
                    if now and now['state']!='Z' and all(now[k]==e[k] for k in ['pgid','start_ticks','executable']):live.append(now)
        if live:errors.append('owned process remains')
        write(self.coord/'independent_audit.json',dict(status='FAIL' if errors else 'PASS',errors=errors,new_accuracy=len(self.index),serial_measurements=len(performance),owned_live_processes=live,qualification='metric confirmation, not new engineering acceptance'))
        if errors:raise RuntimeError(errors)

    def publish(self):
        DEST.mkdir(parents=True,exist_ok=True);rows=[]
        for row in self.index:
            r=dict(row);off=self.lookup_mode(r['sequence'],'OFF');a,b=r.get('ate_rmse_m'),off.get('ate_rmse_m') if off else None;r['relative_off_percent']=100*(a/b-1) if a is not None and b else None;rows.append(r)
        write(DEST/'results.json',rows);table_csv(DEST/'results.csv',rows)
        for name in ['identity.json','data_identity.json','analysis_identity.json','session.json','smoke.json','performance_index.json','metrics.json','local_windows.json','independent_audit.json']:
            if (self.coord/name).exists():shutil.copy2(self.coord/name,DEST/name)
        shutil.copy2(self.coord/'contracts/protocol.json',DEST/'protocol.json');shutil.copy2(self.coord/'contracts/historical_acceptance.json',DEST/'historical_acceptance.json')
        if not (DEST/'configs').exists():shutil.copytree(self.coord/'configs',DEST/'configs')
        if not (DEST/'support').exists():shutil.copytree(self.coord/'support',DEST/'support')
        write(DEST/'run_manifest.json',[json.loads(p.read_text()) for p in self.coord.glob('w*/results/*/manifest_finish.json')])
        text=['# EuRoC 最终冻结配置确认','', '按场景和模式调参后的性能比较；这些序列已经参与选参，重跑属于冻结确认，不是独立留出验证或严格同权重消融。OFF为纯OpenVINS，ON内部Observer参数一致，max_slam=0。MH_04此前因基线发散被用户排除，未纳入主表或本轮重跑。', '', '| 场景 | 模式 | G信息权重 | V信息权重 | L档位 |','|---|---|---:|---:|---:|']
        for family,profiles in PROFILES.items():
            for mode,(g,v,l) in profiles.items():text.append(f'| {family} | {mode} | {g or "关闭"} | {v or "关闭"} | {l or "关闭"} |')
        text+=['','## ATE RMSE（m）','', '| 序列 | OFF | G | V | L | GVL |','|---|---:|---:|---:|---:|---:|']
        def cell(seq,mode,percent=False):
            r=next((r for r in rows if r['sequence']==seq and r['mode']==mode),None)
            if r is None:return 'PENDING'
            x=r.get('relative_off_percent') if percent else r.get('ate_rmse_m')
            if x is None:return r['status']
            value=f'{x:+.6f}%' if percent else f'{x:.9f}'
            return '**'+value+'**' if mode!='OFF' and r.get('relative_off_percent',0)<0 else value
        for seq in EUROC:text.append('| '+seq+' | '+' | '.join(cell(seq,m) for m in MODES)+' |')
        text+=['','## 相对 OFF 百分比变化','', '变化=(配置−OFF)/OFF×100%；负值改善并加粗。','', '| 序列 | OFF | G | V | L | GVL |','|---|---:|---:|---:|---:|---:|---:|']
        for seq in EUROC:text.append('| '+seq+' | '+' | '.join(cell(seq,m,True) for m in MODES)+' |')
        text+=['','## 初始化、覆盖与实际更新','', '| 序列 | 模式 | 状态 | OFF支持覆盖 | GT覆盖 | G/V/L应用帧 |','|---|---|---|---:|---:|---|']
        for r in rows:
            a=r.get('audit',{});text.append(f"| {r['sequence']} | {r['mode']} | {r['status']} | {r.get('coverage')} | {r.get('gt_coverage')} | {a.get('actual_G',0)}/{a.get('actual_V',0)}/{a.get('actual_L',0)} |")
        if (self.coord/'metrics.json').exists():
            metrics=json.loads((self.coord/'metrics.json').read_text());text+=['','## RPE 与尾部位置误差','', 'RPE采用机体相对位姿，1/5秒端点匹配容差25ms；V1_01姿态GT版本未核实为重估版本，其依赖姿态的RPE仅描述，不用于验收。局部误差为OFF初始化起点的固定10秒窗口，使用完整对齐，不逐窗口重新对齐。末尾10%RMSE与误差分布尾部是不同指标。','', '| 序列 | 模式 | RPE-T 1s(m) | RPE-T 5s(m) | RPE-R 1s(°) | RPE-R 5s(°) | 位置P95(m) | P99(m) | 最大(m) | 支持末10%RMSE(m) |','|---|---|---:|---:|---:|---:|---:|---:|---:|---:|']
            for r in metrics:text.append('| '+r['sequence']+' | '+r['mode']+' | '+' | '.join('NOT_EVALUATED' if r.get(k) is None else f'{r[k]:.6f}' for k in ['rpe_translation_1s_m','rpe_translation_5s_m','rpe_rotation_1s_deg','rpe_rotation_5s_deg','position_p95_m','position_p99_m','position_max_m','support_last10pct_rmse_m'])+' |')
        performance=json.loads((self.coord/'performance_index.json').read_text());text+=['','## 串行运行开销','', f'已测 {len(performance)}/50 组；每组完整输入、独占性能锁、同一w0两逻辑CPU、相同诊断等级。仅一次配对运行，未测独立重复计时；顺序按序列交替正反。camera_api仅覆盖feed_camera调用，不能代替包含IMU更新和I/O的整段CPU/墙钟时间。并行回放计时不用于此表。','', '| 序列 | 模式 | 相机API P50(ms) | P95(ms) | P99(ms) | 全段CPU(s) | 墙钟(s) | RSS(MiB) | 串并行轨迹一致 |','|---|---|---:|---:|---:|---:|---:|---:|---|']
        for r in performance:
            c=r.get('cost',{});text.append('| '+r['sequence']+' | '+r['mode']+' | '+' | '.join('NOT_EVALUATED' if k not in c else f'{c[k]:.6f}' for k in ['camera_api_p50_ms','camera_api_p95_ms','camera_api_p99_ms','full_cpu_s','full_wall_s'])+' | '+(f'{c["peak_rss_kb"]/1024:.3f}' if 'peak_rss_kb' in c else 'NOT_EVALUATED')+f" | {r.get('trajectory_identical_to_parallel')} |")
        notes=DEST/'analysis_notes.md'
        if notes.exists():text+=['',notes.read_text().rstrip()]
        text+=['','## 证据与验收边界','', '历史5%预测FAIL和工程结论保留。此表未完成完整主P/Observer状态与生命周期的冻结确定性回归，相关性近似和误差协方差一致性缺口仍在，不以ATE确认授予生产资格。原历史阈值原样附存，没有用调参筛选的10%替代真实验收标准。',f'源码 `{self.identity["source_oid"]}`；原始目录 `{self.coord}`。','[结果](euroc_final_results_data/results.json) · [RPE/尾部](euroc_final_results_data/metrics.json) · [局部窗口](euroc_final_results_data/local_windows.json) · [串行开销](euroc_final_results_data/performance_index.json) · [审计](euroc_final_results_data/independent_audit.json) · [冻结协议](euroc_final_results_data/protocol.json)','']
        REPORT.write_text('\n'.join(text));write(DEST/'sha256.json',{str(f.relative_to(DEST)):sha(f) for f in sorted(DEST.rglob('*')) if f.is_file() and f.name!='sha256.json'})


def main():
    p=argparse.ArgumentParser();p.add_argument('action',choices=['prepare','run','publish']);p.add_argument('coord',type=Path);p.add_argument('--parent',type=Path);a=p.parse_args()
    if a.action=='prepare':prepare(a.coord,a.parent)
    elif a.action=='run':Final(a.coord).execute()
    else:Final(a.coord).publish()

if __name__=='__main__':main()
