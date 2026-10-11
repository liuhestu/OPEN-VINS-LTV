#!/usr/bin/env python3
"""Existing persisted MC composite; no sampling or retrospective gate."""
import argparse
import json
from pathlib import Path
import numpy as np
p=argparse.ArgumentParser();p.add_argument('mc',type=Path);p.add_argument('output',type=Path);a=p.parse_args()
if a.output.exists():raise RuntimeError('refusing composite overwrite')
pred=np.loadtxt(a.mc/'predicted_covariance.csv',delimiter=',');emp=np.loadtxt(a.mc/'empirical_covariance.csv',delimiter=',')
nom=np.loadtxt(a.mc/'nominal_mean_error.csv',delimiter=',');mean=np.loadtxt(a.mc/'empirical_mean_error.csv',delimiter=',')
# Fixture current/anchor are t=.1/.05 AFTER stopping at t=0. True initial
# current R_WB=I and generated true omega=0 throughout these intervals.
# Noisy measured/corrected gyro is not the true pose. Thus A_true=I is derived
# from the actual declared truth trajectory, not guessed from the estimates.
a_true=np.eye(3);map=np.zeros((3,33));map[:,21:24]=np.eye(3);map[:,30:33]=-a_true
rp=map@pred@map.T;re=map@emp@map.T;mn=map@nom;me=map@mean
out={'scope':'same recorded controlled truth/law, conditional linear-main; no new MC','current_true_time_s':.1,'anchor_true_time_s':.05,
     'A_true':a_true.tolist(),'A_true_derivation':'true stationary body after t0; initial true current R_WB=I; generated true angular rate zero; seed historic t=-.05 does not change these poses',
     'predicted_R_L_m2':rp.tolist(),'empirical_R_L_m2':re.tolist(),'predicted_trace_m2':float(np.trace(rp)),'empirical_trace_m2':float(np.trace(re)),
     'relative_covariance_frobenius_descriptive':float(np.linalg.norm(re-rp)/np.linalg.norm(rp)),'nominal_composite_mean_m':mn.tolist(),'empirical_composite_mean_m':me.tolist(),
     'nominal_mean_norm_m':float(np.linalg.norm(mn)),'empirical_mean_norm_m':float(np.linalg.norm(me)),
     'predicted_noise_sqrt_trace_m':float(np.sqrt(np.trace(rp))),'mean_norm_over_noise_sqrt_trace':float(np.linalg.norm(me)/np.sqrt(np.trace(rp))),
     'new_acceptance_gate':False,'zero_mean_consistency':'contradicted by controlled mean bias; covariance agreement does not remove it','real_landmark_calibration':'NOT_EVALUATED'}
a.output.write_text(json.dumps(out,indent=2)+'\n');print(json.dumps(out))
