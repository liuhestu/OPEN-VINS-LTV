"""Paired continuous-time block statistics; no independent-frame significance claims."""
import numpy as np
def rmse(x):return float(np.sqrt(np.mean(np.asarray(x)**2)))
def paired(t,a,b):
 """Positive improvement of scalar error b relative to a; resample 1s blocks."""
 good=np.isfinite(a)&np.isfinite(b)&np.isfinite(t);t=np.asarray(t)[good];a=np.asarray(a)[good];b=np.asarray(b)[good]
 if len(t)<2:return {'n':len(t),'improvement':None,'ci95':None}
 labels=np.floor(t-t[0]).astype(int);keys,inv=np.unique(labels,return_inverse=True)
 counts=np.bincount(inv);sa=np.bincount(inv,weights=a*a);sb=np.bincount(inv,weights=b*b)
 rng=np.random.default_rng(42);draw=rng.integers(0,len(keys),(2000,len(keys)))
 ar=np.sqrt(sa[draw].sum(axis=1)/counts[draw].sum(axis=1));br=np.sqrt(sb[draw].sum(axis=1)/counts[draw].sum(axis=1))
 gain=1-br/np.maximum(ar,1e-12)
 return {'n':len(t),'seconds':float(len(t)*np.median(np.diff(t))) if len(t)>1 else 0.,'blocks':len(keys),'baseline_rmse':rmse(a),'method_rmse':rmse(b),'improvement':float(1-rmse(b)/max(rmse(a),1e-12)),'ci95':np.quantile(gain,[.025,.975]).tolist(),'method_win_fraction':float(np.mean(b<a)),'baseline_p95':float(np.quantile(a,.95)),'method_p95':float(np.quantile(b,.95)),'baseline_median':float(np.median(a)),'method_median':float(np.median(b))}
def dependence(a,b):
 good=np.isfinite(a).all(axis=1)&np.isfinite(b).all(axis=1);a=a[good];b=b[good]
 if len(a)<2:return {'n':len(a),'centered_correlation':None}
 cross=a.T@b/len(a);ac=a-a.mean(axis=0);bc=b-b.mean(axis=0);den=np.outer(ac.std(axis=0),bc.std(axis=0))
 corr=np.divide(ac.T@bc/len(a),den,out=np.full_like(den,np.nan),where=den>1e-12)
 return {'n':len(a),'raw_cross_second_moment':cross.tolist(),'centered_correlation':[[float(v) if np.isfinite(v) else None for v in row] for row in corr],'mean_prior_error_dot_correction':float(np.mean(np.einsum('ij,ij->i',a,b-a)))}
def windows(t,a,b):
 good=np.isfinite(a)&np.isfinite(b);result={}
 for label,sign in [('benefit',True),('harm',False)]:
  use=good&((b<a) if sign else (b>a));start=None;items=[]
  for i in range(len(t)+1):
   active=i<len(t) and use[i]
   if active and start is not None and t[i]-t[i-1]>.1:
    items.append((start,i));start=None
   if active and start is None:start=i
   if not active and start is not None:items.append((start,i));start=None
  longest=sorted(items,key=lambda x:t[x[1]-1]-t[x[0]],reverse=True)[:5]
  result[label]=[{'start':float(t[i]),'end':float(t[j-1]),'seconds':float(t[j-1]-t[i]),'baseline_rmse':rmse(a[i:j]),'method_rmse':rmse(b[i:j])} for i,j in longest]
 return result
