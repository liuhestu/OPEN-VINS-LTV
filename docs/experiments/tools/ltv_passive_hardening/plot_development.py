"""Full development timelines, including startup and all unavailable intervals."""
import os
os.environ['MPLBACKEND']='Agg'
os.environ['OPENBLAS_NUM_THREADS']='1'
import argparse,hashlib,json
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt

def plot(previous,new,out,title):
    previous,new,out=map(Path,(previous,new,out));out.mkdir(parents=True,exist_ok=False)
    data={label:np.load(path/'error_curves.npz') for label,path in [('P_PREV',previous),('R1',new)]}
    metadata=json.loads((new/'metrics.json').read_text())
    fig,axes=plt.subplots(4,1,figsize=(12,10),sharex=True)
    for label,d in data.items():
        for ax,key,name in zip(axes[:3],['v','eta','angle'],['velocity error (m/s)','gravity vector error (m/s²)','gravity direction error (deg)']):
            ax.plot(d['t'],np.where(d['raw'],d[key],np.nan),label=label,lw=1)
            ax.set_ylabel(name);ax.grid(alpha=.25);ax.set_yscale('symlog',linthresh=.1)
        axes[3].step(d['t'],d['ready_G']&d['ready_V'],where='post',label=label+' joint ready',lw=1)
    d=data['R1']
    axes[3].fill_between(d['t'],0,1,where=~d['raw'],step='post',alpha=.18,color='black',label='R1 raw unavailable')
    for event in metadata.get('bootstrap_events',[]):
        for ax in axes:ax.axvline(event['t'],color='purple',alpha=.4,ls=':')
    axes[0].legend();axes[3].legend(loc='upper right',ncol=3);axes[3].set_ylabel('ready / unavailable');axes[3].set_xlabel('physical time (s)')
    axes[3].set_ylim(-.05,1.2);axes[3].set_xlim(float(d['t'][0]),float(d['t'][-1]));fig.suptitle(title+' — development, not final acceptance');fig.tight_layout()
    fig.savefig(out/'timeline.png',dpi=160);fig.savefig(out/'timeline.svg');plt.close(fig)
    provenance={'title':title,'inputs':{str(p/'error_curves.npz'):hashlib.sha256((p/'error_curves.npz').read_bytes()).hexdigest() for p in [previous,new]},'source_sha':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),'complete_domain':[float(d['t'][0]),float(d['t'][-1])],'no_startup_or_recovery_trim':True}
    (out/'provenance.json').write_text(json.dumps(provenance,indent=2)+'\n')
if __name__=='__main__':
    p=argparse.ArgumentParser()
    for k in ('previous','new','out','title'):p.add_argument('--'+k,required=True)
    plot(**vars(p.parse_args()))
