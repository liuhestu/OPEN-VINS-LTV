"""Plots only; every file belongs to the closeout, never to historical reports."""
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from paper_review import load,savefig
from closeout_analysis import find


def plots(out,dest,completed,entries):
    target=dest/'figures';target.mkdir(exist_ok=True);names=[];plt.rcParams['svg.hashsalt']='ltv-closeout'
    groups={s:[e for e in entries if e['config']['scene']==s] for s in ['REGULAR','FAST','PAPER']}
    for scene,es in groups.items():
        fig,axes=plt.subplots(3,2,figsize=(12,9));end=20 if scene=='PAPER' else 60;tail=end-5 if scene=='PAPER' else 50
        for e in es:
            z=load(Path(e['directory'])/'trace.npz');c=e['config'];label=f"{c['candidate']} ID {c['lifetime']:g}s" if c['lifetime'] else 'C0 fixed ID'
            values=[z['values'][:,0],z['values'][:,1],np.sqrt(np.nanmean(z['landmarks'][:,:,0]**2,axis=1))]
            for row,value in enumerate(values):
                for col in range(2):
                    mask=z['t']>=tail if col else z['t']>=0
                    axes[row,col].plot(z['t'][mask],value[mask],label=label,lw=.8)
        for row,label in enumerate(['Velocity error [m/s]','Gravity vector error [m/s²]','Landmark RMS [m]']):axes[row,0].set_ylabel(label)
        axes[0,0].legend();axes[0,0].set_title(scene+' full');axes[0,1].set_title(f'{tail}–{end} s, independent scale')
        for ax in axes[-1]:ax.set_xlabel('Time [s]')
        name='lifecycle_'+scene;savefig(fig,target,name);names.append(name)
    if 'E1' in completed:
        es=[find(out,impl='CONT_REF',scene='PAPER'),completed['E1'],find(out,impl='CORE',scene='PAPER')]
        fig,axes=plt.subplots(3,2,figsize=(12,9));base=load(Path(es[0]['directory'])/'trace.npz')
        for e in es:
            z=load(Path(e['directory'])/'trace.npz');values=[z['values'][:,0],z['values'][:,1],np.sqrt(np.mean(z['landmarks'][:,:16,0]**2,axis=1))]
            for row,value in enumerate(values):axes[row,0].plot(z['t'],value,label=e['config']['impl'],lw=.8)
        for a,b,name in [(es[2],es[1],'CORE - SPLIT'),(es[1],es[0],'SPLIT - CONT'),(es[2],es[0],'CORE - CONT')]:
            A=load(Path(a['directory'])/'trace.npz');B=load(Path(b['directory'])/'trace.npz');dx=A['x'][:,:54]-B['x'][:,:54]
            vals=[np.linalg.norm(dx[:,48:51],axis=1),np.linalg.norm(dx[:,51:54],axis=1),np.sqrt(np.mean(np.sum(dx[:,:48].reshape(-1,16,3)**2,axis=2),axis=1))]
            for row,v in enumerate(vals):axes[row,1].plot(A['t'],v,label=name,lw=.8)
        for row,label in enumerate(['Velocity [m/s]','Gravity vector [m/s²]','Landmark RMS [m]']):axes[row,0].set_ylabel(label);axes[row,1].set_ylabel(label)
        axes[0,0].legend();axes[0,1].legend();axes[0,0].set_title('PAPER errors');axes[0,1].set_title('Implementation differences')
        for ax in axes[-1]:ax.set_xlabel('Time [s]')
        savefig(fig,target,'PAPER_discretization');names.append('PAPER_discretization')
    if 'E2' in completed:
        e=completed['E2'];z=load(Path(e['directory'])/'trace.npz');cam=load(Path(e['directory'])/'camera.npz');fig,axes=plt.subplots(2,1,figsize=(11,7))
        for ax,lo in zip(axes,[0,20]):
            m=z['t']>=lo;ax.plot(z['t'][m],z['values'][m,0],label='common 200 Hz',lw=.7)
            for side in ['pre','post']:
                m=(cam['side']==side)&(cam['t']>=lo);ax.scatter(cam['t'][m],cam['values'][m,0],s=4,label=side)
            ax.axhline(.05,color='red',ls='--',label='strict 0.05');ax.axhline(.1,color='gray',ls='--',label='wide 0.10');ax.set_ylabel('Velocity error [m/s]');ax.legend(ncol=3)
        axes[-1].set_xlabel('Time [s]');savefig(fig,target,'PAPER_CORE_30s');names.append('PAPER_CORE_30s')
    return names
