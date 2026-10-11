"""Evaluator contract checks: rigid alignment, no scale, and visible missing support."""
import csv
import json
from pathlib import Path
import tempfile
import numpy as np
from scipy.spatial.transform import Rotation
from evaluate import align, metrics, nearest

def main():
    rng = np.random.default_rng(42)
    points = rng.normal(size=(100, 3))
    rotation = Rotation.from_rotvec([.2, -.3, .1]).as_matrix()
    truth = points @ rotation.T + [1, 2, 3]
    r, t = align(truth, points)
    np.testing.assert_allclose(r, rotation, atol=1e-12)
    np.testing.assert_allclose(t, [1, 2, 3], atol=1e-12)
    r, t = align(2 * truth, points)
    assert np.linalg.norm(points @ r.T + t - 2 * truth) > 1.
    assert nearest(np.array([0., .01, .02]), np.array([.005, .019])).tolist() == [0, 2]
    with tempfile.TemporaryDirectory(prefix="gv_metrics_") as d:
        out = Path(d)
        off, on = out/"runs/S_OFF_01", out/"runs/S_GV_01"
        off.mkdir(parents=True); on.mkdir()
        times = np.arange(121) * .05
        p = np.c_[np.sin(times), np.cos(times), times*.1]
        x = np.zeros((len(times), 17)); x[:, 0] = times; x[:, 4] = 1.; x[:, 5:8] = p
        gt = np.zeros((len(times), 11)); gt[:, 0] = times*1e9; gt[:, 1:4] = p; gt[:, 4] = 1.
        (out/"state_groundtruth_estimate0").mkdir()
        np.savetxt(out/"state_groundtruth_estimate0/data.csv", gt, delimiter=",")
        for run in (off, on):
            np.savetxt(run/"trajectory.csv", x, delimiter=",", header="trajectory", comments="")
            (run/"effective_options.json").write_text('{"calib_camimu_dt":0}')
            (run/"replay.json").write_text('{"complete":true}')
        with (off/"audit.csv").open("w") as f:
            writer=csv.writer(f);writer.writerow(["camera_ns"])
            writer.writerows([[int(round(t*1e9))] for t in times])
        full, _ = metrics(out,"S","GV",dict(root=str(out)))
        assert full["status"] == "VALID" and full["samples"] == 121
        assert full["ate_rmse_m"] < 1e-12
        for dt in ("1.0","5.0"):
            assert full["rpe"][dt]["translation_rmse_m"] < 1e-12
            assert full["rpe"][dt]["rotation_rmse_deg"] < 1e-12
        np.savetxt(on/"trajectory.csv", x[:-20], delimiter=",", header="trajectory", comments="")
        partial, _ = metrics(out,"S","GV",dict(root=str(out)))
        assert partial["status"] == "FAIL" and partial["target_gt_rows"] == 121
        assert partial["missing_rows"] == 20 and partial["samples"] == 101
        assert partial["raw_metrics_conditional_on_available"] and "ate_rmse_m" in partial
    print(json.dumps(dict(status="PASS_FIXED_SUPPORT_SE3_NO_SCALE_RPE_MISSING_OUTPUT")))

if __name__ == "__main__":
    main()
