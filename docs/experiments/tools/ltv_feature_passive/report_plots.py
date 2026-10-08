"""Plot saved evaluator evidence and export four descriptive tables.

Manifest: {"phase":"development"|"confirmation", "real_metrics":
{"sequence":"/absolute/path/metrics.json"}, "synthetic_metrics":[".../metrics.json"]}.
No GT/input/observer code is imported, no metrics are re-evaluated, and no final
scientific success state is inferred. The main scheduler must budget execution.
"""
import os
for key in ['OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS']:os.environ[key]='1'
import argparse
import csv
from collections import Counter
import hashlib
import json
from pathlib import Path
import re
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
matplotlib.rcParams.update({'svg.hashsalt':'ltv_feature_readiness_passive','font.size':9,'axes.grid':True,'grid.alpha':.2})
COLORS={'P_OLD':'#6b7280','P_NEW':'#146c94','OpenVINS':'#9c528b','OLD':'#6b7280','SEL':'#b65f22',
        'SEED_TEMPORAL':'#27856a','SEED_STEREO':'#146c94','SEED_HYBRID':'#9c528b'}

def digest(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def slug(value):return re.sub('[^A-Za-z0-9_.-]+','_',str(value))
def load_arrays(path):
    with np.load(path,allow_pickle=False) as z:return {key:z[key] for key in z.files}
def number(value):
    if value is None:return 'N/A'
    if isinstance(value,(bool,np.bool_)):return str(bool(value))
    if isinstance(value,(float,np.floating)):return f'{value:.6g}' if np.isfinite(value) else 'N/A'
    if isinstance(value,(dict,list)):return json.dumps(value,sort_keys=True)
    return str(value)

def save_figure(fig,path,artifacts):
    fig.tight_layout()
    for suffix in ['.png','.svg']:
        destination=Path(str(path)+suffix)
        if destination.exists():raise FileExistsError(f'Duplicate figure destination: {destination}')
        if suffix=='.png':fig.savefig(destination,dpi=160,bbox_inches='tight')
        else:
            fig.savefig(destination,metadata={'Date':None},bbox_inches='tight')
            destination.write_text('\n'.join(line.rstrip() for line in destination.read_text().splitlines())+'\n')
        artifacts.append(destination.name)
    plt.close(fig)

def real_figures(sequence,metrics,arrays,phase,out,artifacts):
    t=arrays['time']-arrays['time'][0]
    panels=[('v','Velocity error (m/s)','raw_common'),('eta','Gravity vector error (m/s²)','raw_common'),
            ('v','Velocity error on NEW V-ready support (m/s)','NEW_ready_V'),
            ('eta','Gravity error on NEW G-ready support (m/s²)','NEW_ready_G')]
    fig,axes=plt.subplots(2,2,figsize=(12,7),sharex=True)
    for ax,(key,label,support) in zip(axes.flat,panels):
        mask=arrays['support_'+support]
        for method in ['P_OLD','P_NEW','OpenVINS']:
            values=arrays[method+'_'+key].copy()
            # Raw errors retain each available reference-supported output. The
            # lower panels alone restrict to the identical recorded NEW support.
            if support!='raw_common':values[~mask]=np.nan
            ax.plot(t,values,color=COLORS[method],lw=.85,label=method)
        ax.set_ylabel(label);ax.set_xlabel('Time since first initialized packet (s)')
        if not mask.any():ax.text(.5,.5,'No evaluated support',transform=ax.transAxes,ha='center')
    axes[0,0].legend(ncol=3,fontsize=8)
    fig.suptitle(f'{sequence} | {phase} | saved status: {metrics["status"]}',y=1.02)
    save_figure(fig,out/(slug(sequence)+'_real_errors'),artifacts)
    fig,axes=plt.subplots(4,1,figsize=(12,8),sharex=True)
    for method in ['P_OLD','P_NEW','OpenVINS']:
        axes[0].plot(t,arrays[method+'_g_angle_deg'],color=COLORS[method],lw=.8,label=method)
        axes[1].plot(t,arrays[method+'_g_magnitude'],color=COLORS[method],lw=.8)
    axes[0].set_ylabel('Gravity direction error (deg)');axes[0].legend(ncol=3,fontsize=8)
    axes[1].set_ylabel('Gravity magnitude error (m/s²)')
    for index,(mask,label,color) in enumerate([('NEW_ready_G','NEW G-ready common support','#27856a'),
                                               ('NEW_ready_V','NEW V-ready common support','#146c94')]):
        axes[2].step(t,arrays['support_'+mask].astype(float)+1.2*index,where='post',lw=.75,label=label,color=color)
    axes[2].set_yticks([0,1.2],['G support','V support']);axes[2].legend(loc='upper right',fontsize=8)
    axes[3].step(t,np.isfinite(arrays['reference_v_body']).all(axis=1).astype(float),where='post',label='Velocity reference',lw=.8)
    axes[3].step(t,np.isfinite(arrays['reference_eta_body']).all(axis=1).astype(float)+1.2,where='post',label='Gravity reference',lw=.8)
    axes[3].set_yticks([0,1.2],['V reference','G reference']);axes[3].legend(loc='upper right',fontsize=8)
    axes[3].set_xlabel('Time since first initialized packet (s)')
    fig.suptitle(f'{sequence} | {phase} | support is saved evaluator support; missing GT is not zero error',y=1.01)
    save_figure(fig,out/(slug(sequence)+'_gravity_support'),artifacts)
    fig,axes=plt.subplots(3,2,figsize=(12,8),sharex=True)
    for axis in range(3):
        for column,key in enumerate(['v_components','eta_components']):
            for method in ['P_OLD','P_NEW','OpenVINS']:
                axes[axis,column].plot(t,arrays[method+'_'+key][:,axis],color=COLORS[method],lw=.75,label=method)
            axes[axis,column].set_ylabel(('Velocity' if column==0 else 'Gravity')+f' {"xyz"[axis]} error '+('(m/s)' if column==0 else '(m/s²)'))
    axes[0,0].legend(ncol=3,fontsize=8);axes[2,0].set_xlabel('Elapsed physical time (s)');axes[2,1].set_xlabel('Elapsed physical time (s)')
    fig.suptitle(f'{sequence} | {phase} | body-frame component errors, no fitted alignment',y=1.01)
    save_figure(fig,out/(slug(sequence)+'_components'),artifacts)

def synthetic_figures(group,phase,out,artifacts):
    identity=group[0]['run']['input_identity']
    label=f'{identity["scene"]} life={identity["lifetime"]}s seed={identity["seed"]}'
    basename=f'{identity["scene"]}_life{identity["lifetime"]}_seed{identity["seed"]}_{group[0]["metrics"]["input_sha"][:8]}'
    fig,axes=plt.subplots(4,1,figsize=(12,9),sharex=True)
    for record_index,record in enumerate(group):
        method=record['metrics']['method'];a=record['curves'];color=COLORS.get(method)
        for ax,key,unit in zip(axes[:3],['velocity','eta','gravity_angle_deg'],['Velocity error (m/s)','Gravity vector error (m/s²)','Gravity direction error (deg)']):
            ax.plot(a['t'],a[key],label=method,color=color,lw=.8);ax.set_ylabel(unit)
        events=record['events'];t=np.array([e['t'] for e in events]);ready=np.array([bool(e['ready_G']) and bool(e['ready_V']) for e in events])
        axes[3].step(t,ready.astype(float)+1.2*record_index,label=method,where='post',color=color,lw=.8)
    axes[0].legend(ncol=min(4,len(group)),fontsize=8)
    axes[3].set_yticks([1.2*i for i in range(len(group))],[r['metrics']['method'] for r in group]);axes[3].set_ylabel('Joint ready');axes[3].set_xlabel('Physical time (s)')
    fig.suptitle(f'{label} | {phase} | complete saved timeline',y=1.01)
    save_figure(fig,out/(slug(basename)+'_synthetic_errors'),artifacts)
    for record in group:
        seeds=record['seeds'];method=record['metrics']['method']
        fig,axes=plt.subplots(2,2,figsize=(11,7))
        if seeds:
            parallel=np.array([s['parallel_error_m'] for s in seeds]);transverse=np.array([s['transverse_error_m'] for s in seeds])
            risk=np.array([s['risk'] for s in seeds]);relative=np.array([s['relative_error'] for s in seeds]);wait=np.array([s['waiting_seconds'] for s in seeds])
            reliable=np.array([s['reliable_10pct'] for s in seeds]);colors=np.where(reliable,'#27856a','#bd4b40')
            axes[0,0].hist(parallel,bins=40,color='#146c94',alpha=.8);axes[0,0].axvline(0,color='black',lw=.7)
            axes[0,1].hist(transverse,bins=40,color='#9c528b',alpha=.8)
            axes[1,0].scatter(risk,relative,c=colors,s=8,alpha=.4,rasterized=True)
            axes[1,0].axhline(.1,color='#bd4b40',ls='--',lw=.8,label='10% reliability criterion')
            axes[1,0].set_xscale('symlog',linthresh=1e-4);axes[1,0].set_yscale('symlog',linthresh=1e-4);axes[1,0].legend(fontsize=8)
            axes[1,1].scatter(wait,relative,c=colors,s=8,alpha=.4,rasterized=True);axes[1,1].axhline(.1,color='#bd4b40',ls='--',lw=.8)
        else:
            for ax in axes.flat:ax.text(.5,.5,'No accepted seed (not a reliability success)',ha='center',transform=ax.transAxes,fontsize=8)
        axes[0,0].set_xlabel('Signed along-ray seed error (m)');axes[0,0].set_ylabel('Accepted seeds')
        axes[0,1].set_xlabel('Transverse seed error (m)');axes[0,1].set_ylabel('Accepted seeds')
        axes[1,0].set_xlabel('Predicted relative directional risk (not a probability)');axes[1,0].set_ylabel('True relative 3D error')
        axes[1,1].set_xlabel('Causal wait before admission (s)');axes[1,1].set_ylabel('True relative 3D error')
        fig.suptitle(f'{label} {method} | {phase} | synthetic accepted-seed evaluation only',y=1.01)
        save_figure(fig,out/(slug(basename+'_'+method)+'_seed_quality'),artifacts)

def seed_summary(seeds):
    if not seeds:return {'parallel_bias_m':None,'parallel_RMSE_m':None,'transverse_RMSE_m':None,'waiting_median_s':None}
    p=np.array([r['parallel_error_m'] for r in seeds]);l=np.array([r['transverse_error_m'] for r in seeds])
    return {'parallel_bias_m':float(p.mean()),'parallel_RMSE_m':float(np.sqrt(np.mean(p*p))),
            'transverse_RMSE_m':float(np.sqrt(np.mean(l*l))),'waiting_median_s':float(np.median([r['waiting_seconds'] for r in seeds]))}

def export_table(out,name,rows):
    (out/(name+'.json')).write_text(json.dumps(rows,indent=2,allow_nan=False))
    columns=[]
    for row in rows:
        for key in row:
            if key not in columns:columns.append(key)
    with (out/(name+'.csv')).open('w',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=columns);writer.writeheader()
        for row in rows:writer.writerow({key:number(row.get(key)) for key in columns})
    visible=[key for key in columns if key not in ('metrics_path','input_sha')]
    def escape(value):return number(value).replace('|','\\|').replace('\n',' ')
    markdown='| '+' | '.join(visible)+' |\n| '+' | '.join(['---']*len(visible))+' |\n'
    markdown+=''.join('| '+' | '.join(escape(row.get(key)) for key in visible)+' |\n' for row in rows)
    if not rows:markdown='No supplied records; no result inferred.\n'
    (out/(name+'.md')).write_text(markdown)
    return markdown

def build(manifest_path,out):
    manifest_path=Path(manifest_path);manifest=json.loads(manifest_path.read_text());out=Path(out)
    phase=manifest.get('phase')
    if not isinstance(phase,str) or not phase.strip():raise ValueError('Explicit manifest phase required')
    if not isinstance(manifest.get('real_metrics',{}),dict) or not isinstance(manifest.get('synthetic_metrics',[]),list):raise ValueError('Invalid manifest schema')
    out.mkdir(parents=True,exist_ok=False)
    artifacts=[];sources={};tables={key:[] for key in ['01_seed_reliability','02_feature_management','03_outputs','04_passive_engineering']}
    def read_json(path):
        path=Path(path).resolve();sources[str(path)]=digest(path);return json.loads(path.read_text())
    def read_curves(path):
        path=Path(path).resolve();sources[str(path)]=digest(path);return load_arrays(path)
    real=[];synthetic=[];seen=set()
    for sequence,path in manifest.get('real_metrics',{}).items():
        path=Path(path).resolve();m=read_json(path)
        if m['sequence']!=sequence:raise ValueError('Manifest sequence and metric identity differ')
        a=read_curves(path.parent/'error_curves.npz');real_figures(sequence,m,a,phase,out,artifacts);real.append(m)
        pop=m['feature_management'];coverage=m['coverage']['P_NEW'];identity={'phase':phase,'domain':'real','sequence':sequence,'method':m['seed_source_identity']['source'],'metrics_path':str(path)}
        tables['01_seed_reliability'].append({**identity,'opportunities':pop['opportunity_tracks'],'accepted':pop['seed_writes'],
            'acceptance_fraction':pop['admission_fraction_opportunity'],'seed_sources':pop['source_admissions'],'reliable_10pct_fraction':None,'parallel_RMSE_m':None,
            'risk_median':pop['relative_risk_median'],'waiting_median_s':pop['waiting_median_s'],'reliability_scope':'No real landmark GT'})
        tables['02_feature_management'].append({**identity,'candidates':pop['candidate_tracks'],'admitted':pop['admitted_tracks'],
            'retired':pop['retired_tracks'],'mean_active':pop['mean_active'],'mean_mature':pop['mean_mature'],
            'ready_G_fraction':coverage['ready_G_fraction'],'ready_V_fraction':coverage['ready_V_fraction'],
            'ready_G_seconds':coverage['ready_G_reference_seconds'],'ready_V_seconds':coverage['ready_V_reference_seconds'],
            'longest_G_gap_s':coverage['ready_G_longest_gap_s'],'longest_V_gap_s':coverage['ready_V_longest_gap_s'],'epoch_transitions':pop['epoch_transitions']})
        for support,methods in m['comparison'].items():
            for method,values in methods.items():
                tables['03_outputs'].append({**identity,'method':method,'support':support,'v_RMSE':values['v_RMSE'],'eta_RMSE':values['eta_RMSE'],
                    'gravity_angle_RMSE_deg':values['g_angle_deg_RMSE'],'v_supported_seconds':values['v_seconds'],'eta_supported_seconds':values['eta_seconds'],
                    'wide_v_accuracy':values['wide_v_accuracy'],'wide_g_accuracy':values['wide_g_accuracy']})
        e=m['engineering'];tables['04_passive_engineering'].append({**identity,'saved_status':m['status'],'main_audit_equal':all(e['main_audit_equal'].values()),
            'trajectory_exact_equal':all(e['trajectory_exact_equal'].values()),'any_injection':any(e['any_actual_injection'].values()),
            'all_input_consumed':all(e['complete_input'].values()),'engineering_pass':e['pass'],'scope':'Actual B/P_OLD/P_NEW main VIO comparison'})
    for path in manifest.get('synthetic_metrics',[]):
        path=Path(path).resolve()
        if path in seen:raise ValueError('Duplicate synthetic metric path')
        seen.add(path);m=read_json(path);run=read_json(path.parent/'run.json');seeds=read_json(path.parent/'seed_evaluation.json')
        curves=read_curves(path.parent/'error_curves.npz');event_path=path.parent/'events.jsonl';sources[str(event_path)]=digest(event_path)
        events=[json.loads(line) for line in event_path.read_text().splitlines() if line]
        record={'metrics':m,'run':run,'seeds':seeds,'curves':curves,'events':events};synthetic.append(record)
        config=run['input_identity'];identity={'phase':phase,'domain':'synthetic','sequence':config['scene'],'lifetime_s':config['lifetime'],
            'seed':config['seed'],'method':m['method'],'input_sha':m['input_sha'],'metrics_path':str(path)}
        s=m['seed'];c=m['coverage'];life=m['lifecycle'];stats=seed_summary(seeds)
        tables['01_seed_reliability'].append({**identity,'opportunities':s['opportunity_tracks'],'accepted':s['accepted_seeds'],
            'acceptance_fraction':s['accepted_fraction_opportunity'],'seed_sources':dict(Counter(r['source'] for r in seeds)),'reliable_10pct_fraction':s['reliable_10pct_fraction'],
            'reliable_5pct_fraction':s['reliable_5pct_fraction'],'minimum_100_accepted':s['minimum_100_accepted'],**stats,'reliability_scope':'Synthetic accepted seeds only'})
        tables['02_feature_management'].append({**identity,'candidates':s['all_birth_tracks'],'admitted':life['admitted_tracks'],
            'mean_active':life['mean_active'],'mean_mature':life['mean_mature_visible'],'ready_G_fraction':c['ready_G_fraction'],
            'ready_V_fraction':c['ready_V_fraction'],'ready_G_seconds':c['ready_G_seconds'],'ready_V_seconds':c['ready_V_seconds'],
            'longest_G_gap_s':c['longest_G_gap_s'],'longest_V_gap_s':c['longest_V_gap_s'],
            'completed_tracks':life['completed_observation_tracks'],'right_censored_tracks':life['right_censored_tracks']})
        for support,values in [('raw',m['raw']),('method_ready_G',m['ready_G']),('method_ready_V',m['ready_V'])]:
            tables['03_outputs'].append({**identity,'support':support,'v_RMSE':values['v_RMSE'],'eta_RMSE':values['eta_RMSE'],
                'gravity_angle_RMSE_deg':values['g_angle_RMSE_deg'],'samples':values['samples'],'wide_v_accuracy':values['wide_v_accuracy'],'wide_g_accuracy':values['wide_g_accuracy']})
        for support,values in m.get('baseline',{}).items():
            tables['03_outputs'].append({**identity,'method':'OLD_same_input','support':support,'v_RMSE':values['v_RMSE'],'eta_RMSE':values['eta_RMSE'],
                'gravity_angle_RMSE_deg':values['g_angle_RMSE_deg'],'samples':values['samples']})
        d=m['diagnostics'];tables['04_passive_engineering'].append({**identity,'saved_status':m['status'],'any_injection':bool(d['injection_G'] or d['injection_V']),
            'all_input_consumed':d['full_input_consumed'],'elapsed_seconds':d['elapsed_seconds'],'main_audit_equal':None,'scope':'Synthetic core only; no real bypass claim'})
    groups={}
    for record in synthetic:groups.setdefault(record['metrics']['input_sha'],[]).append(record)
    for group in groups.values():synthetic_figures(group,phase,out,artifacts)
    if synthetic:
        fig,axes=plt.subplots(2,1,figsize=(max(10,len(synthetic)*.5),7),sharex=True)
        labels=[f'{r["run"]["input_identity"]["scene"]}\nL{r["run"]["input_identity"]["lifetime"]} s{r["run"]["input_identity"]["seed"]}\n{r["metrics"]["method"]}' for r in synthetic]
        acceptance=[r['metrics']['seed']['accepted_fraction_opportunity'] for r in synthetic]
        reliability=[r['metrics']['seed']['reliable_10pct_fraction'] for r in synthetic]
        axes[0].bar(np.arange(len(synthetic)),acceptance,color='#146c94');axes[0].axhline(.2,ls='--',color='#bd4b40',label='20% task target');axes[0].set_ylabel('Accepted / opportunities');axes[0].legend()
        axes[1].bar(np.arange(len(synthetic)),[v if v is not None else np.nan for v in reliability],color='#27856a');axes[1].axhline(.9,ls='--',color='#bd4b40',label='90% task target');axes[1].set_ylabel('Reliable / accepted');axes[1].legend()
        for j,r in enumerate(synthetic):
            axes[1].text(j,.03,f'n={r["metrics"]["seed"]["accepted_seeds"]}',ha='center',rotation=90,fontsize=7)
            if reliability[j] is None:axes[1].text(j,.5,'N/A',ha='center',fontsize=8)
        axes[1].set_xticks(np.arange(len(synthetic)),labels,rotation=65,ha='right');axes[0].set_ylim(0,1.05);axes[1].set_ylim(0,1.05)
        fig.suptitle(f'{phase} | descriptive synthetic screening results; targets shown without automatic final decision',y=1.01)
        save_figure(fig,out/'synthetic_acceptance_reliability',artifacts)
    markdown=[f'# Saved evidence tables — {phase}\n',
        'These artifacts reproduce supplied evaluator evidence. They do not select parameters or infer final task success. N/A is not zero; real landmark reliability has no GT. All curves retain the complete saved time axis.\n']
    for key,rows in tables.items():
        body=export_table(out,key,rows);markdown.extend([f'\n## {key}\n',body])
    (out/'tables.md').write_text('\n'.join(markdown))
    (out/'tables.json').write_text(json.dumps(tables,indent=2,allow_nan=False))
    provenance={'phase':phase,'status':'ARTIFACTS_WRITTEN_NO_SUCCESS_INFERENCE','manifest':str(manifest_path.resolve()),
        'manifest_sha':digest(manifest_path),'plotter_sha':digest(__file__),'sources':sources,'figures':artifacts,
        'tables':list(tables),'real_sequences':list(manifest.get('real_metrics',{})),'synthetic_runs':len(synthetic),
        'GT_reread':False,'metric_or_threshold_changes':False}
    (out/'artifact_manifest.json').write_text(json.dumps(provenance,indent=2))
    return provenance

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--manifest',type=Path,required=True);parser.add_argument('--out',type=Path,required=True)
    args=parser.parse_args();result=build(args.manifest,args.out);print(json.dumps({'status':result['status'],'phase':result['phase'],'figures':len(result['figures']),'tables':result['tables']},indent=2))
