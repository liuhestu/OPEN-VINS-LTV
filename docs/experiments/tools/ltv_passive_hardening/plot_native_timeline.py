"""Native saved-evidence full timeline; never changes masks or metrics."""
import os
for k in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS'):os.environ[k]='1'
import argparse,json,hashlib
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def main():
 p=argparse.ArgumentParser();p.add_argument('--evaluation',required=True);p.add_argument('--comparison',required=True);p.add_argument('--out',required=True);args=p.parse_args();ev=Path(args.evaluation);prev=Path(args.comparison);out=Path(args.out)
 if out.exists():raise FileExistsError(out)
 m=json.loads((ev/'metrics.json').read_text());pm=json.loads((prev/'metrics.json').read_text());a=dict(np.load(ev/'error_curves.npz'));b=dict(np.load(prev/'error_curves.npz'));np.testing.assert_array_equal(a['camera_ns'],b['camera_ns']);np.testing.assert_array_equal(a['P_NEW_v'],b['P_NEW_v']);np.testing.assert_array_equal(a['P_NEW_eta'],b['P_NEW_eta']);assert m['sequence']==pm['sequence']
 lp=Path(m['provenance']['runs']['P_NEW'])/'features.jsonl';rows=[json.loads(l) for l in lp.read_text().splitlines()];lookup={r['camera_ns']:r for r in rows};r=[lookup[int(k)] for k in a['camera_ns']];origin=a['time'][0];t=a['time']-origin;cad=float(np.median(np.diff(t)));gaps=np.flatnonzero(np.diff(t)>1.5*cad)+1;init_start=(rows[0]['camera_ns']-int(a['camera_ns'][0]))/1e9;end=float(t[-1]);epochs=a['P_NEW_epoch'];breaks=np.r_[False,(np.diff(t)>1.5*cad)|(np.diff(epochs)!=0)]
 def curve(ax,y,label,color,style='-'):
  y=np.asarray(y,float);valid=np.isfinite(y[:-1])&np.isfinite(y[1:])&~breaks[1:];segments=np.stack([np.c_[t[:-1],y[:-1]],np.c_[t[1:],y[1:]]],axis=1);ax.add_collection(LineCollection(segments[valid],colors=color,linewidths=.8,linestyles=style,label=label));ax.plot(t[np.isfinite(y)],y[np.isfinite(y)],'.',ms=1,color=color);ax.autoscale_view()
 def spans(ax,mask,y,color):
  for i in np.flatnonzero(mask):
   width=min(cad,t[i+1]-t[i]) if i+1<len(t) else 0
   if width:ax.broken_barh([(t[i],width)],(y,.65),facecolors=color,linewidth=0)
 fig,axes=plt.subplots(7,1,figsize=(17,17),sharex=True,gridspec_kw={'height_ratios':[2,2,2,1.4,1.2,1.4,1]})
 labels=[('v','Velocity error [m/s]'),('eta','Gravity vector error [m/s²]'),('g_angle_deg','Gravity direction error [deg]')]
 for ax,(q,label) in zip(axes[:3],labels):
  curve(ax,b['P_NEW_error_'+q],'R3 history-compatible raw','#aaaaaa');curve(ax,a['P_NEW_error_'+q],'R3B raw (same numerical estimates)','#2369a6');curve(ax,a['OpenVINS_error_'+q],'OpenVINS main','#df8632');ax.set_ylabel(label);ax.legend(loc='upper right',fontsize=8);ax.set_ylim(bottom=0)
 for j,(key,name,color) in enumerate([('ready_G','G','#339455'),('ready_V','V','#277db5'),('joint_ready','joint','#7555aa')]):spans(axes[3],np.array([bool(x[key]) for x in r]),j,color)
 axes[3].set_yticks([.3,1.3,2.3]);axes[3].set_yticklabels(['G-ready','V-ready','joint-ready']);axes[3].set_ylim(-.1,3)
 for j,key in enumerate(['grace_G','grace_V']):
  mask=np.array([bool(x.get(key,False)) for x in r]);axes[4].scatter(t[mask],np.full(mask.sum(),j),s=8,label=key)
 axes[4].set_yticks([0,1]);axes[4].set_yticklabels(['G grace','V grace']);axes[4].set_ylim(-.5,1.5)
 names=list(dict.fromkeys(x['availability_state'] for x in rows));codes={n:i for i,n in enumerate(names)}
 for name,j in codes.items():spans(axes[5],np.array([x['availability_state']==name for x in r]),j,plt.cm.tab10(j))
 axes[5].set_yticks([j+.3 for j in codes.values()]);axes[5].set_yticklabels(names,fontsize=8);axes[5].set_ylim(-.1,len(names)+.1)
 missing=~(np.isfinite(a['reference_v']).all(axis=1)&np.isfinite(a['reference_eta']).all(axis=1));spans(axes[6],missing,0,'#d34343');axes[6].scatter(t[gaps],np.ones(len(gaps)),marker='|',s=40,color='black',label='packet gap > 1.5 median');ec=np.flatnonzero(np.r_[True,np.diff(epochs)!=0]);axes[6].scatter(t[ec],np.full(len(ec),1.7),marker='v',s=25,color='#72549b');axes[6].set_yticks([.3,1,1.7]);axes[6].set_yticklabels(['reference missing','packet gap','epoch start']);axes[6].set_ylim(-.1,2.1)
 for ax in axes:
  ax.set_xlim(init_start,end);ax.axvspan(init_start,0,color='#dddddd',alpha=.6);ax.axvline(0,color='#555555',lw=.8);ax.grid(alpha=.18)
  for i in gaps:ax.axvspan(t[i-1]+cad,t[i],color='#aaaaaa',alpha=.10,lw=0)
 axes[-1].set_xlabel('Seconds from first evaluated initialized camera packet (0); grey prefix = pre-initialization, not evaluated')
 fig.suptitle(m['sequence']+' — complete saved native timeline; '+m['status']+'\nRaw errors include startup/unavailable periods; missing references remain NaN; lines break at packet gaps',fontsize=13)
 fig.tight_layout(rect=(0,0,1,.955));out.mkdir(parents=True);fig.savefig(out/'native_full_timeline.png',dpi=150);fig.savefig(out/'native_full_timeline.pdf');plt.close(fig)
 record={'sequence':m['sequence'],'status':m['status'],'domain_seconds_relative_initialized':[float(init_start),end],'evaluated_domain_seconds':[0,end],'preinit_packets':sum(not bool(x['initialized']) for x in rows),'initialized_packets':len(t),'packet_gaps_over_1p5_median':len(gaps),'reference_missing_packets':int(missing.sum()),'raw_estimate_comparison':'exact v/eta and integer camera_ns arrays','sources':{str(x):sha(x) for x in [ev/'metrics.json',ev/'error_curves.npz',prev/'metrics.json',prev/'error_curves.npz',lp,Path(__file__)]},'outputs':{str(x):sha(x) for x in out.iterdir()},'limitations':['No interpolation across physical gaps or reference NaNs.','Readiness bars cover at most one nominal outgoing packet interval; no held status invented through gaps.','Pre-initialization is visibly unevaluated; no error values invented.','R3B and prior raw overlap by exact numerical equality; only latest readiness shown.','Scientific decisions remain original evaluator output.']};(out/'provenance.json').write_text(json.dumps(record,indent=2)+'\n');print(json.dumps({k:v for k,v in record.items() if k not in ['sources','outputs']},indent=2))
if __name__=='__main__':main()
