"""Recover all five native IMU correction blocks and actual joint P changes."""
import argparse
from itertools import zip_longest
import json
from pathlib import Path

import numpy as np
from scipy.linalg import cho_factor, cho_solve
from audit import ROOT, SEQS, sha, stats, write

BLOCKS = dict(attitude_rad=slice(0,3), position_m=slice(3,6),velocity_mps=slice(6,9),gyro_bias_radps=slice(9,12),accel_bias_mps2=slice(12,15))


def records(path):
    with path.open() as f:
        for line in f: yield json.loads(line)


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--evidence',type=Path,default=Path('/home/he/output/ltv_gv_fusion_evaluation_20261005'))
    ap.add_argument('--out',type=Path,default=ROOT/'docs/ltv/landmark_correlation/gv')
    args=ap.parse_args()
    result={}
    for seq in SEQS:
        for mode in ('G','V','GV'):
            run=args.evidence/'runs'/f'{seq}_{mode}_{"01" if mode=="GV" else "03"}'
            values={b:{k:[] for k in BLOCKS} for b in ('G','V')}
            changes={k:[] for k in BLOCKS}
            applied=0
            for event,raw in zip_longest(records(run/'fusion.jsonl'),records(run/'matrices.jsonl')):
                assert event is not None and raw is not None and event['camera_ns']==raw['camera_ns']
                if event['submit_reason']!='joint_applied':continue
                assert event['consumed'] and event['ekf_calls']==1
                applied+=1
                P,H,S=np.asarray(raw['prior_P']),np.asarray(raw['joint_H']),np.asarray(raw['joint_S'])
                dense=np.zeros((H.shape[0],len(P)))
                offset=0
                for id_,size in zip(raw['joint_ids'],raw['joint_sizes']):
                    dense[:,id_:id_+size]=H[:,offset:offset+size];offset+=size
                assert offset==H.shape[1]
                K=cho_solve(cho_factor(S,lower=False),(P@dense.T).T).T
                residual=np.asarray(raw['joint_res']).ravel()
                begin=event['visual_rows']
                for branch in ('G','V'):
                    count=event[branch+'_rows']
                    if count:
                        delta=K[:15,begin:begin+count]@residual[begin:begin+count]
                        for name,block in BLOCKS.items():values[branch][name].append(float(np.linalg.norm(delta[block])))
                    begin+=count
                assert begin==len(residual)
                post=np.asarray(raw['joint_post_P'])
                for name,block in BLOCKS.items():changes[name].append(float(np.trace(post[block,block])-np.trace(P[block,block])))
            result[seq+'/'+mode]=dict(joint_applied=applied,branch_correction_norms={b:{k:stats(v) for k,v in z.items()} for b,z in values.items()},
                actual_joint_P_block_trace_change={k:stats(v) for k,v in changes.items()},
                P_scope='all visual + auxiliary rows jointly; cannot attribute whole contraction to G/V alone',
                matrix_identity=sha(run/'matrices.jsonl'))
            print(seq,mode,applied,flush=True)
    write(args.out/'matrix_blocks.json',result)


if __name__=='__main__':main()
