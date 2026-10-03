"""Standalone exportable figures, generated only after validation unseals."""
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from common import *
from analyze import selected,cache,labels

def make_plots():
 if not read(OUT/'experiment_manifest.json').get('weights_frozen'):raise RuntimeError('validation sealed')
 dest=ROOT/'docs/ltv/evidence/value_study/figures';dest.mkdir(parents=True,exist_ok=True)
 plt.rcParams.update({'font.size':9,'axes.grid':True,'grid.alpha':.25})
 summaries=[]
 for seq in SEQUENCES:
  a=cache(selected(seq));m=a['gt_pose_valid'];t=a['t']-a['t'][m][0]
  fig,axes=plt.subplots(4,1,figsize=(11,10),sharex=True,layout='constrained')
  for name,label,color in [('gt_v_body','Reference','black'),('prior_v','OpenVINS prior','tab:blue'),('ltv_v','LTV (valid)','tab:orange')]:
   y=np.linalg.norm(a[name],axis=1);y[~m]=np.nan
   if name=='ltv_v':y[~np.isfinite(a['ltv_V_error'])]=np.nan
   axes[0].plot(t,y,label=label,color=color,lw=1)
  axes[0].set_ylabel('Speed (m/s)');axes[0].legend(ncol=3);axes[0].set_title(seq+' — Passive, physical IMU time')
  for name,label in [('prior','Prior'),('ltv','LTV')]:
   axes[1].plot(t,a[name+'_V_error'],label=label,lw=1)
   axes[2].plot(t,a[name+'_G_error'],label=label,lw=1)
  axes[1].set_ylabel('Velocity error (m/s)');axes[2].set_ylabel('Gravity error (deg)');axes[1].legend(ncol=2)
  delta=a['shadow_visual_V_error']-a['shadow_V_V_error'];axes[3].plot(t,delta,label='visual error minus visual+V error',lw=.7);axes[3].axhline(0,color='k',lw=.7)
  axes[3].set_ylabel('One-step V benefit (m/s)');axes[3].set_xlabel('Time from reference support start (s)');axes[3].legend()
  for ax in axes:ax.set_xlim(0,t[m][-1])
  for suffix in ['png','pdf']:fig.savefig(dest/(seq+'.'+suffix),dpi=150)
  plt.close(fig)
  ok=np.isfinite(a['ltv_V_error']);good=np.isfinite(a['ltv_G_error'])
  summaries.append((seq,np.sqrt(np.mean(a['prior_V_error'][ok]**2)),np.sqrt(np.mean(a['ltv_V_error'][ok]**2)),np.sqrt(np.mean(a['prior_G_error'][good]**2)),np.sqrt(np.mean(a['ltv_G_error'][good]**2))))
 fig,axes=plt.subplots(2,1,figsize=(12,8),layout='constrained');x=np.arange(len(summaries))
 for j,ax in enumerate(axes):
  ax.bar(x-.2,[s[1+2*j] for s in summaries],.4,label='OpenVINS prior');ax.bar(x+.2,[s[2+2*j] for s in summaries],.4,label='LTV')
  ax.set_yscale('log');ax.set_xticks(x,[s[0] for s in summaries],rotation=25,ha='right');ax.set_ylabel('Velocity RMSE (m/s)' if j==0 else 'Gravity RMSE (deg)');ax.legend()
 for suffix in ['png','pdf']:fig.savefig(dest/('direct_errors.'+suffix),dpi=150)
 plt.close(fig)
if __name__=='__main__':make_plots()
