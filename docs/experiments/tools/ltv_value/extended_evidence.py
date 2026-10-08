"""Descriptive motion and reference sensitivity, using predeclared thresholds."""
import numpy as np
from scipy.ndimage import uniform_filter1d
from common import *
from analyze import cache,selected
from reference import load_gt

def distribution(x):
    x=np.asarray(x);x=x[np.isfinite(x)]
    return {'n':len(x),'median':float(np.median(x)) if len(x) else None,
            'p75':float(np.quantile(x,.75)) if len(x) else None,
            'p95':float(np.quantile(x,.95)) if len(x) else None,
            'max':float(np.max(x)) if len(x) else None}

def extend():
    if not read(OUT/'experiment_manifest.json').get('validation_unsealed'):raise RuntimeError('validation sealed')
    inputs=read(OUT/'inputs.json');result={}
    for seq in SEQUENCES:
        info=inputs[seq];a=cache(selected(seq));t,p,q,v=load_gt(seq)
        imu=np.loadtxt(Path(info['replay_root'])/'imu0/data.csv',delimiter=',',comments='#');it=imu[:,0]*1e-9
        n=max(1,int(round(1/np.median(np.diff(it)))))
        omega=np.sqrt(uniform_filter1d(np.sum(imu[:,1:4]**2,axis=1),n,mode='nearest'))
        gravity=9.81 if seq in EUROC else 9.8065
        force=np.sqrt(uniform_filter1d((np.linalg.norm(imu[:,4:7],axis=1)-gravity)**2,n,mode='nearest'))
        j=np.searchsorted(t,it).clip(1,len(t)-1)
        supported=(it>=t[0])&(it<=t[-1])&((t[j]-t[j-1])<=.010000001)
        motion={}
        for label,mask in [('all_input',np.ones(len(it),bool)),('GT_pose_support',supported)]:
            motion[label]={'omega_rms_radps':distribution(omega[mask]),'specific_force_variation_mps2':distribution(force[mask]),
                           'seconds':float(mask.sum()*np.median(np.diff(it)))}
        valid=np.isfinite(a['ltv_V_error']);own={}
        for name in ['gt_v_body','prior_v','ltv_v']:
            speed=np.linalg.norm(a[name][valid],axis=1)
            own[name]={'rms_mps':float(np.sqrt(np.mean(speed**2))),'distribution':distribution(speed)}
        r={'motion':motion,'speed_on_common_valid_LTV_support':own,'reference_coverage':{'diagnostic_events':len(a['t']),
           'pose':int(a['gt_pose_valid'].sum()),'velocity':int(a['gt_v_valid'].sum()),
           'G_and_LTV':int(np.isfinite(a['ltv_G_error']).sum()),'V_and_LTV':int(valid.sum())},
           'velocity_reference_sensitivity':{},'timestamp_rounding':{}}
        common=np.isfinite(a['gt_v_body']).all(axis=1)
        for w in [.05,.2]:
            other=a[f'gt_v_body_{w}'];mask=common&np.isfinite(other).all(axis=1)
            r['velocity_reference_sensitivity'][str(w)]={'n':int(mask.sum()),
                'difference_to_0.1s_mps':distribution(np.linalg.norm(other[mask]-a['gt_v_body'][mask],axis=1))}
        if seq not in EUROC:
            from decimal import Decimal,ROUND_HALF_EVEN
            values=[Decimal(line.split()[1])*Decimal(1000000000) for line in (Path(info['source'])/'imu.txt').read_text().splitlines() if line.strip() and not line.startswith('#')]
            r['timestamp_rounding']['max_IMU_ns']=float(max(abs(x-x.to_integral_value(rounding=ROUND_HALF_EVEN)) for x in values))
        result[seq]=r
    write(OUT/'extended_evidence.json',result)
    return result
if __name__=='__main__':extend()
