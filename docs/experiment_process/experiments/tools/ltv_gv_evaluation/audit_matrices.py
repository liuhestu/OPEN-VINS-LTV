"""Recompute real joint gains, full Joseph P and native IMU mean from saved raw evidence."""
import argparse
import json
from pathlib import Path
import numpy as np
from scipy.linalg import cho_factor, cho_solve
from run_matrix import write
from evaluate import runs

def main():
    p=argparse.ArgumentParser();p.add_argument("out",type=Path);p.add_argument("report",type=Path)
    args=p.parse_args();summary={}
    for run in sorted(runs(args.out).iterdir()):
        if not (run/"fusion.jsonl").exists():continue
        count=0;max_p=0.;max_mean=0.;max_nis=0.;max_s=0.;max_contribution=0.
        with (run/"fusion.jsonl").open() as events,(run/"matrices.jsonl").open() as matrices:
            for el,ml in zip(events,matrices):
                event,raw=json.loads(el),json.loads(ml)
                assert event["camera_ns"]==raw["camera_ns"]
                if event["submit_reason"]!="joint_applied":continue
                assert event["consumed"] and event["ekf_calls"]==1 and event["G_rows"]+event["V_rows"]>0
                P,H,R,res=[np.asarray(raw[k],dtype=float) for k in ("prior_P","joint_H","joint_R","joint_res")]
                res=res.ravel();actual=np.asarray(raw["joint_post_P"])
                dense=np.zeros((len(res),len(P)));offset=0
                for id_,size in zip(raw["joint_ids"],raw["joint_sizes"]):
                    dense[:,id_:id_+size]=H[:,offset:offset+size];offset+=size
                S=dense@P@dense.T+R
                saved=np.asarray(raw["joint_S"])
                max_s=max(max_s,float(np.linalg.norm(S-saved)/max(1.,np.linalg.norm(saved))))
                factor=cho_factor(saved,lower=False)
                K=cho_solve(factor,(P@dense.T).T).T
                I=np.eye(len(P));A=I-K@dense
                joseph=A@P@A.T+K@R@K.T
                error=float(np.linalg.norm(actual-joseph)/max(1.,np.linalg.norm(P)))
                max_p=max(max_p,error)
                delta=K@res
                prior=np.asarray(raw["prior_imu"]).ravel()
                dq=np.r_[.5*delta[:3],1.];dq/=np.linalg.norm(dq)
                q=prior[:4]
                expected_q=np.r_[dq[3]*q[:3]-np.cross(dq[:3],q[:3])+dq[:3]*q[3],dq[3]*q[3]-dq[:3]@q[:3]]
                if expected_q[3]<0:expected_q*=-1
                expected_q/=np.linalg.norm(expected_q)
                expected=prior.copy();expected[:4]=expected_q;expected[4:]+=delta[3:15]
                actual_mean=np.asarray(raw["joint_post_imu"]).ravel()
                max_mean=max(max_mean,float(np.linalg.norm(expected-actual_mean)))
                nis=float(res@cho_solve(factor,res))
                max_nis=max(max_nis,abs(nis-event["joint_nis"])/max(1.,nis))
                start=event["visual_rows"]
                for branch in ("G","V"):
                    n=event[branch+"_rows"]
                    if n:
                        variance=(10*np.pi/180)**2 if branch=="G" else 1.
                        np.testing.assert_allclose(R[start:start+n,start:start+n],variance*np.eye(n),atol=1e-15,rtol=0.)
                        assert np.count_nonzero(R[start:start+n,:start])==0
                        assert np.count_nonzero(R[start:start+n,start+n:])==0
                        value=float(np.linalg.norm(K[:,start:start+n]@res[start:start+n]))
                        max_contribution=max(max_contribution,abs(value-event[branch+"_update_norm"]))
                        start+=n
                count+=1
        assert max_p<=1e-9 and max_mean<=1e-9, (run,max_p,max_mean)
        assert max_s<=1e-9 and max_nis<=1e-9 and max_contribution<=1e-9
        summary[run.name]=dict(actual_updates_checked=count,max_full_Joseph_relative_error=max_p,
            max_native_IMU_mean_error=max_mean,max_innovation_relative_error=max_s,
            max_joint_NIS_relative_error=max_nis,max_contribution_error=max_contribution,
            block_diagonal_noise_exact=True,tolerance=1e-9,tolerance_source="existing test_ltv_joint_covariance")
        print(run.name,summary[run.name],flush=True)
    write(args.report,summary)

if __name__=="__main__":
    main()
