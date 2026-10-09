#!/usr/bin/env python3
"""V36/V49 velocity-only extension; independently screened against OFF and V1."""
import argparse
import json
from pathlib import Path
import shutil
import numpy as np
from independent import Independent, ROOT, EUROC, REP, assess, write, table_csv, sha, process, weights
from v25 import prepare as prepare_shared

TIERS=[36,49]
DEST=ROOT/'docs/euroc_tune_results/v36_v49'

class VelocityHigh(Independent):
    def execute_trial(self):
        if not (self.coord/'smoke.json').exists():write(self.coord/'smoke.json',self.batch([(REP[0],'V',0,a,40,'smoke') for a in TIERS]))
        if any(r['exit_code'] or r['audit']['errors'] for r in json.loads((self.coord/'smoke.json').read_text())):raise RuntimeError('smoke failed')
        self.batch([j for a in TIERS for j in self.jobs(REP,'V',0,a,'representative')])
        extending=[]
        for a in TIERS:
            result=self.screen(REP,'V',0,a,'representative')
            if result['status']=='PASS':extending+=self.jobs([s for s in EUROC if s not in REP],'V',0,a,'full')
            else:self.block(EUROC,'V',0,a,'representative gate blocked')
        self.publish_trial();self.batch(extending)
        self.audit_trial();self.publish_trial()

    def audit_trial(self):
        errors=[];new=[r for r in self.index if r['mode']=='V' and r['alpha_velocity'] in TIERS]
        if len(self.index)!=60 or len({(r['sequence'],r['mode'],r['alpha_gravity'],r['alpha_velocity']) for r in self.index})!=60:errors.append('identity count')
        for row in new:
            if 'output_dir' not in row:continue
            out=Path(row['output_dir']);m=json.loads((out/'manifest_finish.json').read_text());opt=json.loads((out/'effective_options.json').read_text())
            if opt['ltv_enable_gravity'] or opt['ltv_enable_landmark_approx'] or not opt['ltv_enable_velocity']:errors.append('mode mismatch')
            if not np.isclose(opt['ltv_sigma_velocity_mps'],weights('V',0,row['alpha_velocity'])['ltv_sigma_velocity_mps'],rtol=1e-14,atol=0):errors.append('weight mismatch')
            if json.loads((out/'threading.json').read_text())['actual_opencv_threads']!=1:errors.append('thread mismatch')
            for category in ['config_paths','binary_paths']:
                for path,item in m[category].items():
                    if sha(path)!=item['sha256']:errors.append('artifact changed')
            check=self.evaluate(row)
            if check['status']!=row['status'] or (row['status']=='VALID_FULL_MATCHED' and not np.isclose(check['ate_rmse_m'],row['ate_rmse_m'],rtol=1e-12,atol=0)):errors.append('ATE mismatch')
        for screen in self.screening:
            a=screen['alpha_velocity'];result=assess([self.comparison(s,'V',0,a) for s in REP],REP)
            if result['status']!=screen['status']:errors.append('screen mismatch')
            if screen['status']=='BLOCKED' and any(r['alpha_velocity']==a and r['sequence'] not in REP and 'run_id' in r for r in new):errors.append('blocked extension launched')
        ms=[json.loads(p.read_text()) for p in self.coord.glob('w*/results/*/manifest_finish.json')];n=peak=0
        for _,d in sorted([(m['started_at'],1) for m in ms]+[(m['finished_at'],-1) for m in ms]):n+=d;peak=max(peak,n)
        if peak>8:errors.append('pool limit')
        live=[]
        for f in (self.coord/'registry').glob('*.jsonl'):
            for line in f.read_text().splitlines():
                e=json.loads(line)
                if e['event']=='PROCESS_REGISTERED':
                    q=process(e['pid'])
                    if q and q['state']!='Z' and all(q[k]==e[k] for k in ['pgid','start_ticks','executable']):live.append(q)
        if live:errors.append('owned process remains')
        write(self.coord/'independent_audit.json',dict(status='FAIL' if errors else 'PASS',errors=errors,recorded=len(self.index),historical=40,new_full_recomputed=sum('run_id' in r for r in new),maximum_concurrent=peak,owned_live_processes=live))
        if errors:raise RuntimeError(errors)

    def publish_trial(self):
        DEST.mkdir(parents=True,exist_ok=True);rows=[]
        for seq in EUROC:
            for a in TIERS:
                row=dict(self.lookup(seq,'V',0,a) or dict(sequence=seq,mode='V',alpha_gravity=0,alpha_velocity=a,status='PENDING',ate_rmse_m=None));x=row['ate_rmse_m']
                for name,ref in [('off_ate',None),('v16_ate',16),('v25_ate',25)]:
                    b=self.lookup(seq,'OFF' if ref is None else 'V',0,0 if ref is None else ref)['ate_rmse_m'];row[name]=b;row['relative_'+name+'_percent']=100*(x/b-1) if x is not None else None
                if 'output_dir' in row:
                    import csv
                    entries=list(csv.DictReader((Path(row['output_dir'])/'ltv.csv').open()));counts={}
                    for entry in entries:
                        reason=entry.get('V_reason','');counts[reason]=counts.get(reason,0)+1
                    row['velocity_reason_counts']=counts;eligible=sum(n for k,n in counts.items() if k=='accepted' or 'nis' in k.lower());rejected=sum(n for k,n in counts.items() if 'nis' in k.lower());row['nis_rejection_fraction_of_tested']=rejected/eligible if eligible else None
                rows.append(row)
        write(DEST/'results.json',rows);table_csv(DEST/'results.csv',rows);write(DEST/'screening.json',self.screening)
        for name in ['identity.json','analysis_identity.json','independent_audit.json','smoke.json','session.json']:
            if (self.coord/name).exists():shutil.copy2(self.coord/name,DEST/name)
        shutil.copy2(self.coord/'contracts/protocol.json',DEST/'protocol.json')
        for a in TIERS:
            if not (DEST/f'config_v{a}').exists():shutil.copytree(self.coord/f'configs/V_G0_V{a}',DEST/f'config_v{a}')
        write(DEST/'run_manifest.json',[json.loads(f.read_text()) for f in self.coord.glob('w*/results/*/manifest_finish.json')])
        def pct(v):return '—' if v is None else ('**'+f'{v:+.6f}%'+ '**' if v<0 else f'{v:+.6f}%')
        text=['## V36/V49 单分支追加测试','', 'G/landmark关闭；V36标准差1/6 m/s，V49标准差1/7 m/s。原算法和门控不变，原冻结OFF支持，八进程隔离、实际单线程。相对OFF/V1严格超过10%阻止扩展，V25用于收益对照。','', '| 序列 | 档位 | OFF ATE(m) | V25 ATE(m) | 本档ATE(m) | 相对OFF | 相对V25 | 状态 | V应用帧 |','|---|---:|---:|---:|---:|---:|---:|---|---:|']
        for r in rows:
            x=r['ate_rmse_m'];value='—' if x is None else f'{x:.9f}'
            if x is not None and r['relative_off_ate_percent']<0:value='**'+value+'**'
            text.append(f"| {r['sequence']} | V{r['alpha_velocity']} | {r['off_ate']:.9f} | {r['v25_ate']:.9f} | {value} | {pct(r['relative_off_ate_percent'])} | {pct(r['relative_v25_ate_percent'])} | {r['status']} | {r.get('audit',{}).get('actual_V',0)} |")
        text+=['','| 参数 | 平均相对OFF | 中位数 | 改善/退化 | 最差变化 | 平均相对V25 |','|---|---:|---:|---:|---:|---:|']
        for a in TIERS:
            data=[r for r in rows if r['alpha_velocity']==a and r['status']=='VALID_FULL_MATCHED']
            if len(data)!=10:continue
            d=[r['relative_off_ate_percent'] for r in data];extra=[r['relative_v25_ate_percent'] for r in data];text.append(f'| **V{a}** | {pct(float(np.mean(d)))} | {pct(float(np.median(d)))} | {sum(x<0 for x in d)}/{sum(x>0 for x in d)} | {max(d):+.6f}% | {pct(float(np.mean(extra)))} |')
        notes=DEST/'analysis_notes.md'
        if notes.exists():text+=['',notes.read_text().rstrip()]
        text+=['',f'原始目录 `{self.coord}`。', '[结构化结果与NIS原因](euroc_tune_results/v36_v49/results.json) · [筛选](euroc_tune_results/v36_v49/screening.json) · [审计](euroc_tune_results/v36_v49/independent_audit.json)','']
        section='\n'.join(text);(DEST/'section.md').write_text(section);report=ROOT/'docs/euroc_tune_results.md';old=report.read_text();marker='\n## V36/V49 单分支追加测试'
        if marker in old:old=old[:old.index(marker)]
        report.write_text(old.rstrip()+'\n\n'+section);write(DEST/'sha256.json',{str(f.relative_to(DEST)):sha(f) for f in sorted(DEST.rglob('*')) if f.is_file() and f.name!='sha256.json'})


def main():
    p=argparse.ArgumentParser();p.add_argument('action',choices=['prepare','run','publish']);p.add_argument('coord',type=Path);p.add_argument('--parent',type=Path);a=p.parse_args()
    if a.action=='prepare':prepare_shared(a.coord,a.parent,TIERS,(1,16,25))
    elif a.action=='run':VelocityHigh(a.coord).execute_trial()
    else:VelocityHigh(a.coord).publish_trial()

if __name__=='__main__':main()
