#!/usr/bin/env python3
"""Verify finite source implementation matrices, without claiming calibration."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import numpy as np


def main():
    p = argparse.ArgumentParser(); p.add_argument('run', type=Path); p.add_argument('output', type=Path); p.add_argument('--name', default='finite.csv')
    a = p.parse_args()
    if a.output.exists(): raise RuntimeError('refusing audit overwrite')
    records = list(csv.DictReader((a.run/a.name).open()))
    events = [json.loads(line) for line in (a.run/(a.name+'.matrices.jsonl')).read_text().splitlines()]
    failures, summaries = [], []
    worst_symmetry = 0.; min_eigen = float('inf')
    with (a.run/(a.name+'.matrices.bin')).open('rb') as f:
        for ordinal, event in enumerate(events):
            matrices = {}
            for name, item in event['matrices'].items():
                f.seek(item['offset']); size = 8*item['rows']*item['cols']; data = f.read(size)
                if len(data) != size: raise RuntimeError('truncated finite covariance stream')
                matrix = np.frombuffer(data, dtype=np.float64).reshape((item['rows'],item['cols']),order='F').copy()
                if not np.isfinite(matrix).all(): failures.append({'event':ordinal,'matrix':name,'reason':'NONFINITE'})
                matrices[name] = matrix
            x = matrices['Sigma_main_local_program']; o = matrices['Sigma_observer_mean']; cross = matrices['C_main_local_program_observer']
            joint = np.block([[x,cross],[cross.T,o]])
            asymmetry = np.linalg.norm(joint-joint.T); scale = max(1.,np.linalg.norm(joint)); worst_symmetry=max(worst_symmetry,asymmetry/scale)
            eigen = np.linalg.eigvalsh(.5*(joint+joint.T)); min_eigen=min(min_eigen,float(eigen[0]))
            if asymmetry > 1e-10*scale or eigen[0] < -1e-10*scale: failures.append({'event':ordinal,'reason':'JOINT_SYMMETRY_OR_PSD','min_eigen':float(eigen[0])})
            anchors = [name for name in matrices if name.startswith('Sigma_anchor_')]
            for anchor in anchors:
                suffix=anchor.removeprefix('Sigma_anchor_'); aa=matrices[anchor]; ta=matrices['Sigma_current_anchor_'+suffix]
                xa=matrices['C_main_local_program_anchor_'+suffix]
                whole=np.block([[x,cross[:,:3],xa],[cross[:,:3].T,o[:3,:3],ta],[xa.T,ta.T,aa]])
                mine=float(np.linalg.eigvalsh(.5*(whole+whole.T))[0]); min_eigen=min(min_eigen,mine)
                if mine < -1e-10*max(1.,np.linalg.norm(whole)): failures.append({'event':ordinal,'anchor':anchor,'reason':'MAIN_CURRENT_ANCHOR_JOINT_PSD','min_eigen':mine})
            summaries.append({'time':event['time'],'feature':event['feature'],'main_dim':len(x),'observer_dim':len(o),
                              'main_observer_cross_norm':float(np.linalg.norm(cross)),'anchors':len(anchors),'missing_events':event['missing_events']})
    result={'run':str(a.run),'events':len(events),'scalar_events':len(records),'features':len({r['feature'] for r in records}),
            'max_factor_columns':max([int(r['factor_columns']) for r in records] or [0]),'max_missing_events':max([int(r['missing_events']) for r in records] or [0]),
            'max_relative_asymmetry':worst_symmetry,'minimum_joint_eigenvalue':None if not events else min_eigen,'failures':failures,
            'implementation_check':'PASS_WITHIN_SCOPE' if events and not failures else 'FAIL' if failures else 'NOT_EVALUATED_NO_POINT_EVENTS',
            'statistics':'NOT_PROVEN; PSD/finite/determinism are implementation checks only','real_landmark_calibration':'NOT_EVALUATED',
            'source_scope':'one-slot conditional program covariance, fresh history subset, declared initial prior/white noises/main H-K schedule',
            'input_sha256':{name:hashlib.sha256((a.run/name).read_bytes()).hexdigest() for name in [a.name,a.name+'.matrices.jsonl']},
            'per_event':summaries}
    a.output.write_text(json.dumps(result,indent=2)+'\n'); print(json.dumps({k:v for k,v in result.items() if k not in ('per_event','input_sha256')}))
    raise SystemExit(bool(failures))


if __name__ == '__main__': main()
