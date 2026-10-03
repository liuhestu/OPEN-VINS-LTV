"""Static, exportable figure of all candidates, with regressions retained."""
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from study import OUT,ROOT,SEQS,CANDIDATES
s=json.loads((OUT/'selection.json').read_text());names=[n for n in CANDIDATES if n!='B']
x=np.array([[s['scores'][n]['per_sequence_percent'][seq] for n in names] for seq in SEQS]);limit=max(1.,np.abs(x).max())
fig,ax=plt.subplots(figsize=(10,6));im=ax.imshow(x,cmap='RdBu_r',vmin=-limit,vmax=limit,aspect='auto')
ax.set_xticks(range(len(names)),names);ax.set_yticks(range(len(SEQS)),SEQS)
for i in range(len(SEQS)):
 for j in range(len(names)):ax.text(j,i,f'{x[i,j]:+.2f}%',ha='center',va='center',color='white' if abs(x[i,j])>.55*limit else 'black',fontsize=9)
ax.set_title('ATE change vs native OpenVINS (negative = improvement)\nAll 10 sequences used for tuning; MH04 excluded; robust EKF, no nonlinear window')
fig.colorbar(im,ax=ax,label='Relative ATE change (%)');fig.tight_layout()
p=ROOT/'docs/ltv/robust_fusion/evidence';p.mkdir(exist_ok=True);fig.savefig(p/'ate_changes.png',dpi=180);plt.close(fig)
