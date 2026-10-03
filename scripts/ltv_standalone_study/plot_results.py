import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
plt.rcParams.update({'font.size':9,'axes.grid':True,'grid.alpha':.25})
def make_plots(entries,destination,star):
    destination.mkdir(exist_ok=True)
    paths=[]
    def save(fig,name):
        fig.tight_layout()
        for ext in ['png','svg']: fig.savefig(destination/f'{name}.{ext}',dpi=140)
        plt.close(fig); paths.append(name)
    def find(**kw): return [e for e in entries if all(e['config'].get(k)==v for k,v in kw.items())]
    for scene in ['REGULAR','FAST','PAPER','STATIC']:
        es=find(scene=scene,candidate='C0',noise=0,lifetime=0,initialization='ZERO',tight=False,imu_hz=200,camera_hz=20)
        es=[e for e in es if e['config']['impl']!='EXPERIMENTAL']
        if not es: continue
        fig,ax=plt.subplots(3,2,figsize=(11,8))
        for e in es:
            z=np.load(Path(e['directory'])/'trace.npz'); label=e['config']['impl']
            for row,col in enumerate([0,1,2]):
                for j in range(2):
                    ax[row,j].plot(z['t'],z['values'][:,col],label=label,lw=.8)
                    if j==1: ax[row,j].set_xlim(max(0,z['t'][-1]-10),z['t'][-1])
        for r,title in enumerate(['Velocity error [m/s]','Gravity vector error [m/s²]','Gravity direction error [deg]']):
            for a in ax[r]: a.set_ylabel(title); a.set_xlabel('physical time [s]')
        ax[0,0].legend(); fig.suptitle(scene+' — full horizon and tail'); save(fig,'fixed_'+scene)
    es=find(scene='REGULAR',candidate='C0',noise=0,lifetime=0,initialization='ZERO',tight=False)
    es=[e for e in es if e['config']['impl'] in ['CORE','SPLIT_REF']]
    if es:
        fig,axes=plt.subplots(2,1,figsize=(10,7))
        for e in es:
            z=np.load(Path(e['directory'])/'trace.npz'); c=e['config']; label=f"{c['impl']} {c['imu_hz']}/{c['camera_hz']} Hz"
            for ax,col in zip(axes,[0,1]): ax.plot(z['t'],z['values'][:,col],label=label,lw=.7)
        axes[0].set_ylabel('Velocity error [m/s]'); axes[1].set_ylabel('Gravity vector error [m/s²]'); axes[1].set_xlabel('Time [s]'); axes[0].legend(ncol=2); save(fig,'sampling_rates')
    es=find(scene='REGULAR',noise=0,lifetime=0,initialization='ZERO',tight=False,imu_hz=200,camera_hz=20)
    es=[e for e in es if e['config']['impl']=='EXPERIMENTAL']
    if es:
        fig,ax=plt.subplots(2,2,figsize=(11,7))
        for e in es:
            z=np.load(Path(e['directory'])/'trace.npz')
            for j,col in enumerate([0,1]):
                for k in range(2): ax[j,k].plot(z['t'],z['values'][:,col],label=e['config']['candidate'],lw=.8)
                ax[j,1].set_xlim(50,60)
        ax[0,0].legend(ncol=4); ax[0,0].set_ylabel('Velocity [m/s]'); ax[1,0].set_ylabel('Gravity vector [m/s²]'); save(fig,'gains_full_and_tail')
    es=find(scene='REGULAR',lifetime=0,initialization='ZERO',impl='EXPERIMENTAL')
    es=[e for e in es if e['config']['noise'] in [42,43]]
    if es:
        fig,ax=plt.subplots(2,1,figsize=(10,7))
        for e in es:
            z=np.load(Path(e['directory'])/'trace.npz'); c=e['config']; label=f"{c['candidate']} seed {c['noise']}"
            for a,col in zip(ax,[0,1]): a.plot(z['t'],z['values'][:,col],label=label,lw=.6)
        ax[0].legend(); ax[0].set_ylabel('Velocity [m/s]'); ax[1].set_ylabel('Gravity vector [m/s²]'); ax[1].set_xlabel('Time [s]'); save(fig,'noise')
    for scene in ['REGULAR','FAST']:
        es=find(scene=scene,lifetime=1,noise=0,tight=False)
        if not es: continue
        fig,ax=plt.subplots(3,1,figsize=(11,9)); agefig,ageax=plt.subplots(figsize=(10,5))
        for e in es:
            z=np.load(Path(e['directory'])/'trace.npz'); c=e['config']; label=f"{c['impl']} {c['candidate']} {c['initialization']}"
            for a,col in zip(ax[:2],[0,1]): a.plot(z['t'],z['values'][:,col],label=label,lw=.6)
            ax[2].plot(z['t'],np.nanpercentile(z['landmarks'][:,:,1],95,axis=1),label=label,lw=.6)
            path=Path(e['directory'])/'lifecycle.json'
            if path.exists():
                tracks={v['id']:v for v in json.loads(path.read_text())}; ages=[]; errors=[]
                for k,t in enumerate(z['t']):
                    for j,id in enumerate(z['ids'][k]):
                        if id>=0 and t<=tracks[int(id)]['last_visible']+1e-9:
                            ages.append(t-tracks[int(id)]['observation_birth']); errors.append(z['landmarks'][k,j,1])
                ages=np.array(ages); errors=np.array(errors); bins=np.arange(0,1.025,.025); centers=[]; med=[]
                for lo,hi in zip(bins[:-1],bins[1:]):
                    vals=errors[(ages>=lo)&(ages<hi)]
                    if len(vals): centers.append((lo+hi)/2); med.append(np.median(vals))
                ageax.plot(centers,med,label=label)
        ax[0].legend(ncol=2,fontsize=7); ax[0].set_ylabel('Velocity [m/s]'); ax[1].set_ylabel('Gravity vector [m/s²]'); ax[2].set_ylabel('Landmark relative error P95'); ax[2].set_xlabel('Time [s]'); save(fig,'lifecycle_initialization_'+scene)
        ageax.axhline(.05,color='black',ls='--',label='wide target'); ageax.set_xlabel('Observation age [s], eligible only through last visible'); ageax.set_ylabel('Median relative 3D landmark error'); ageax.legend(fontsize=7); save(agefig,'landmark_age_'+scene)
    return paths
