"""Fixed-support tail decomposition; does not alter official alignment/thresholds."""
import argparse
import csv
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import sys
ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'scripts/ltv_gv_evaluation'))
from evaluate import align, nearest


def horn(target, source):
    x = source - source.mean(0); y = target - target.mean(0)
    M = x.T @ y
    xx,xy,xz = M[0]; yx,yy,yz = M[1]; zx,zy,zz = M[2]
    N = np.array([[xx+yy+zz,yz-zy,zx-xz,xy-yx],
                  [yz-zy,xx-yy-zz,xy+yx,zx+xz],
                  [zx-xz,xy+yx,-xx+yy-zz,yz+zy],
                  [xy-yx,zx+xz,yz+zy,-xx-yy+zz]])
    values,vectors=np.linalg.eigh(N);q=vectors[:,np.argmax(values)]
    R=Rotation.from_quat(q[[1,2,3,0]]).as_matrix()
    return R,target.mean(0)-R @ source.mean(0)


def main(off_path, on_path, dataset, output):
    off, on = [np.loadtxt(p / 'trajectory.csv', delimiter=',', skiprows=1, ndmin=2) for p in [off_path, on_path]]
    assert off.shape == on.shape and np.array_equal(off[:, 0], on[:, 0])
    with (off_path / 'audit.csv').open() as f:
        audit = list(csv.DictReader(f))
    dt = json.loads((off_path / 'effective_options.json').read_text())['calib_camimu_dt']
    sensor = np.array([int(x['camera_ns']) * 1e-9 + dt for x in audit])
    sensor = sensor[sensor >= off[0, 0] - 1e-6]
    gt = np.loadtxt(dataset / 'state_groundtruth_estimate0/data.csv', delimiter=',', comments='#')
    gi = nearest(gt[:, 0] * 1e-9, sensor)
    valid = (sensor >= gt[0, 0] * 1e-9) & (sensor <= gt[-1, 0] * 1e-9) & (abs(gt[gi, 0] * 1e-9 - sensor) <= .02)
    times, gp = sensor[valid], gt[gi[valid], 1:4]
    oi, ni = nearest(off[:, 0], times), nearest(on[:, 0], times)
    po, pn = off[oi, 5:8], on[ni, 5:8]
    ro, to = align(gp, po); rn, tn = align(gp, pn)
    rng=np.random.default_rng(20261008);fixture=rng.normal(size=(30,3))
    rotation=Rotation.from_rotvec([.3,.2,-.4]).as_matrix();translation=np.array([.2,.4,-.1])
    check_r,check_t=horn(fixture @ rotation.T+translation,fixture)
    assert np.max(abs(check_r-rotation))<1e-12 and np.max(abs(check_t-translation))<1e-12
    ho,hto=horn(gp,po);hn,htn=horn(gp,pn)
    assert np.max(abs(ho-ro))<1e-10 and np.max(abs(hn-rn))<1e-10
    assert np.max(abs(hto-to))<1e-10 and np.max(abs(htn-tn))<1e-10
    error_off = np.linalg.norm(po @ ro.T + to - gp, axis=1)
    error_own = np.linalg.norm(pn @ rn.T + tn - gp, axis=1)
    error_common = np.linalg.norm(pn @ ro.T + to - gp, axis=1)
    bins = np.floor((times-off[0, 0])/10).astype(int)
    tail = bins == bins.max()
    rms = lambda x: float(np.sqrt(np.mean(x*x)))
    raw = np.linalg.norm(pn-po, axis=1)
    result = dict(scope='descriptive decomposition only; original own-full-SE3 official FAIL retained',
                  tail_bin=int(bins.max()), tail_samples=int(tail.sum()), tail_times=times[tail].tolist(),
                  official_off_tail_rmse_m=rms(error_off[tail]), official_on_tail_rmse_m=rms(error_own[tail]),
                  common_OFF_alignment_on_tail_rmse_m=rms(error_common[tail]),
                  raw_native_position_difference_max_m=float(raw.max()), raw_tail_position_difference_m=raw[tail].tolist(),
                  raw_tail_velocity_difference_m_s=np.linalg.norm(on[ni[tail],8:11]-off[oi[tail],8:11],axis=1).tolist(),
                  alignment_rotation_delta_deg=float(np.degrees(Rotation.from_matrix(rn @ ro.T).magnitude())),
                  alignment_translation_delta_m=float(np.linalg.norm(tn-to)),
                  point_sample_errors_off_m=error_off[tail].tolist(), point_sample_errors_ON_ownfit_m=error_own[tail].tolist(),
                  point_sample_errors_ON_commonfit_m=error_common[tail].tolist(),
                  input_interval='full same native timestamps; no tail/ON-selected fit or support deletion',
                  independent_Horn_off_tail_rmse_m=rms(np.linalg.norm(po @ ho.T+hto-gp,axis=1)[tail]),
                  independent_Horn_on_tail_rmse_m=rms(np.linalg.norm(pn @ hn.T+htn-gp,axis=1)[tail]),
                  independent_Horn_synthetic_rotation_translation_check='PASS')
    with output.open('x') as f: json.dump(result,f,indent=2);f.write('\n')
    print(json.dumps(result,indent=2))


if __name__ == '__main__':
    p=argparse.ArgumentParser();p.add_argument('off',type=Path);p.add_argument('on',type=Path);p.add_argument('dataset',type=Path);p.add_argument('output',type=Path)
    a=p.parse_args();main(a.off,a.on,a.dataset,a.output)
