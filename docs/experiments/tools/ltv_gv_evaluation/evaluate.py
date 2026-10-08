"""Offline fixed-OFF-support SE(3) metrics, event accounting and divergence evidence."""
import argparse
import csv
from collections import Counter
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from run_matrix import exact, write

def runs(out):
    return out / ("final_runs" if (out / "final_runs").exists() else "runs")

def nearest(t, u):
    hi = np.searchsorted(t, u).clip(0, len(t) - 1)
    lo = np.maximum(0, hi - 1)
    return np.where(abs(t[lo] - u) <= abs(t[hi] - u), lo, hi)

def align(gt, p):
    u, _, vt = np.linalg.svd((p - p.mean(0)).T @ (gt - gt.mean(0)))
    d = np.eye(3)
    d[2, 2] = np.linalg.det(vt.T @ u.T)
    r = vt.T @ d @ u.T
    return r, gt.mean(0) - r @ p.mean(0)

def trajectory(path):
    if not path.exists() or path.stat().st_size == 0:
        return np.empty((0, 17))
    x = np.loadtxt(path, delimiter=",", skiprows=1, ndmin=2)
    if not x.size:
        return np.empty((0, 17))
    assert x.shape[1] == 17 and np.isfinite(x).all() and np.all(np.diff(x[:, 0]) > 0)
    return x

def rows(path):
    return [json.loads(line) for line in path.read_text().splitlines() if line]

def csvrows(path):
    with path.open() as f:
        return list(csv.DictReader(f))

def metrics(out, seq, mode, inp):
    run = runs(out) / f"{seq}_{mode}_01"
    base = trajectory(runs(out) / f"{seq}_OFF_01/trajectory.csv")
    x = trajectory(run / "trajectory.csv")
    gt_path = Path(inp["root"]) / "state_groundtruth_estimate0/data.csv"
    gt_raw = np.loadtxt(gt_path, delimiter=",", comments="#", ndmin=2)
    t, p, q = gt_raw[:, 0] * 1e-9, gt_raw[:, 1:4], gt_raw[:, [5, 6, 7, 4]]
    off_audit = csvrows(runs(out) / f"{seq}_OFF_01/audit.csv")
    offset = json.loads((run / "effective_options.json").read_text())["calib_camimu_dt"]
    camera = np.array([int(r["camera_ns"]) * 1e-9 + offset for r in off_audit])
    sensor = camera[camera >= base[0, 0] - 1e-6]
    gi = nearest(t, sensor)
    inside = (sensor >= t[0]) & (sensor <= t[-1]) & (abs(t[gi] - sensor) <= .02)
    support, gt, gtq = sensor[inside], p[gi[inside]], q[gi[inside]]
    result = dict(sequence=seq, mode=mode, status="FAIL", output_rows=len(x), target_sensor_rows=len(sensor),
                  target_gt_rows=len(support), gt_time_fraction=float(inside.mean()), scale=1.,
                  gt_sha=exact.sha(gt_path), evaluator_sha=exact.sha(__file__),
                  protocol="nearest_GT_20ms; fixed_OFF_support; SE3_no_scale; output_match_1us; >=99pct")
    if not len(x) or len(support) < 3:
        result["reason"] = "missing_output_or_reference"
        return result, None
    j = nearest(x[:, 0], support)
    available = abs(x[j, 0] - support) <= 1e-6
    js = nearest(x[:, 0], sensor)
    sensor_ok = abs(x[js, 0] - sensor) <= 1e-6
    result.update(coverage=float(available.mean()), sensor_coverage=float(sensor_ok.mean()),
                  initialization_delta_s=float(x[0, 0] - base[0, 0]), missing_rows=int((~available).sum()))
    gaps = np.flatnonzero(np.diff(np.r_[False, ~sensor_ok, False]))
    result["longest_missing_packets"] = int(max(gaps[1::2] - gaps[::2], default=0))
    valid = result["coverage"] >= .99 and result["sensor_coverage"] >= .99 and abs(result["initialization_delta_s"]) <= 1e-6
    # Keep raw metrics even for incomplete support, clearly marked conditional.
    est, gt, gtq, times = x[j[available]], gt[available], gtq[available], support[available]
    if len(est) < 3:
        result["reason"] = "insufficient_matching_output"
        return result, None
    r, translation = align(gt, est[:, 5:8])
    error = est[:, 5:8] @ r.T + translation - gt
    result.update(status="VALID" if valid else "FAIL", reason=None if valid else "coverage_or_initialization_mismatch",
                  ate_rmse_m=float(np.sqrt(np.mean(np.sum(error**2, axis=1)))), samples=len(est),
                  alignment_R=r.tolist(), alignment_t=translation.tolist(), raw_metrics_conditional_on_available=not valid)
    rg, re = Rotation.from_quat(gtq), Rotation.from_quat(est[:, 1:5])
    rotation_error = (rg.inv() * (Rotation.from_matrix(r) * re)).magnitude()
    result["rotation_rmse_deg"] = float(np.degrees(np.sqrt(np.mean(rotation_error**2))))
    result["rpe"] = {}
    for dt in (1., 5.):
        end = nearest(times, times + dt)
        ok = abs(times[end] - (times + dt)) <= .025
        a, b = np.flatnonzero(ok), end[ok]
        de = re[a].inv().apply(est[b, 5:8] - est[a, 5:8])
        dg = rg[a].inv().apply(gt[b] - gt[a])
        dr = ((rg[a].inv() * rg[b]).inv() * (re[a].inv() * re[b])).magnitude()
        result["rpe"][str(dt)] = dict(pairs=len(a),
            translation_rmse_m=float(np.sqrt(np.mean(np.sum((de-dg)**2, axis=1)))) if len(a) else None,
            rotation_rmse_deg=float(np.degrees(np.sqrt(np.mean(dr**2)))) if len(a) else None)
    extent = float(np.linalg.norm(np.ptp(gt, axis=0)))
    result.update(gt_extent_m=extent, localization_status="FAIL_GROSS_DRIFT" if result["ate_rmse_m"] > max(5., extent) else "VALID",
                  gross_failure_rule="ATE > max(5m, GT bounding-box diagonal); raw metrics retained")
    if not (run / "replay.json").exists() or not json.loads((run / "replay.json").read_text()).get("complete"):
        result.update(status="FAIL", reason="native_failure_incomplete_input")
    return result, dict(time=times, reference_origin=float(base[0,0]), error=np.linalg.norm(error, axis=1), rotation=np.degrees(rotation_error))

def events(out, seq, mode):
    run = runs(out) / f"{seq}_{mode}_01"
    own, off = rows(run / "fusion.jsonl"), rows(runs(out) / f"{seq}_OFF_01/fusion.jsonl")
    audit = csvrows(run / "audit.csv")
    counts = {}
    first = {}
    for branch in ("G", "V"):
        actual = lambda r: bool(r["consumed"] and r[branch + "_rows"] and r["submit_reason"] == "joint_applied" and r["ekf_calls"] == 1)
        counts[branch] = dict(requests=sum(r["requested_" + branch] for r in own),
                             receipts=sum(r["requested_" + branch] and r["receipt"] for r in own),
                             attempted_gates=sum(r[branch + "_nis"] >= 0 for r in own),
                             gate_accepts=sum(r[branch + "_reason"] == "accepted" for r in own),
                             reasons=dict(Counter(r[branch + "_reason"] if r["receipt"] else "no_update_stage:" + r["trigger_reason"] for r in own if r["requested_" + branch])),
                             actual_updates=sum(actual(r) for r in own),
                             submit_rejects=sum(r[branch + "_reason"] == "accepted" and not actual(r) for r in own))
        submitted = sum(int(r["gravity_submissions" if branch == "G" else "velocity_submissions"]) for r in audit)
        assert submitted == sum(r[branch + "_rows"] > 0 and r["consumed"] for r in own), "audit/receipt mismatch"
    for field in ("main_digest", "covariance_digest", "observer_digest"):
        first[field] = next((dict(camera_ns=a["camera_ns"], actual_fusion=bool(a["consumed"] and (a["G_rows"] or a["V_rows"]) and a["submit_reason"] == "joint_applied"), event=a)
                             for a, b in zip(own, off) if a[field] != b[field]), None)
    first["event"] = next((dict(camera_ns=a["camera_ns"], own=a, off=b) for a, b in zip(own, off)
                           if any(a[k] != b[k] for k in ("receipt", "G_reason", "V_reason", "G_rows", "V_rows", "submit_reason"))), None)
    numeric = min((v for k, v in first.items() if k in ("main_digest","covariance_digest") and v), key=lambda r:r["camera_ns"], default=None)
    if numeric:
        assert numeric["actual_fusion"], "first main divergence without actual fusion"
    if (run / "replay.json").exists():
        meta = json.loads((run / "replay.json").read_text())
        for branch in ("G", "V"):
            assert meta["actual_" + branch + "_submissions"] == counts[branch]["actual_updates"]
    return dict(counts=counts, first_divergence=first, camera_events=len(own),
                actual_joint_updates=sum(r["submit_reason"] == "joint_applied" and r["consumed"] and r["ekf_calls"] == 1 for r in own),
                submit_reasons=dict(Counter(r["submit_reason"] for r in own)))

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out", type=Path)
    ap.add_argument("report", type=Path)
    args = ap.parse_args()
    out, doc = args.out.resolve(), args.report.resolve()
    doc.mkdir(parents=True, exist_ok=True)
    freeze = json.loads((out / "freeze.json").read_text())
    result, diagnostics = [], {}
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    for seq, inp in freeze["inputs"].items():
        fig, ax = plt.subplots(2, 1, figsize=(11, 6), sharex=True)
        for mode in ("OFF", "G", "V", "GV"):
            m, curve = metrics(out, seq, mode, inp)
            result.append(m)
            diagnostics[seq + "/" + mode] = events(out, seq, mode)
            if curve:
                ax[0].plot(curve["time"] - curve["reference_origin"], curve["error"], label=mode)
                ax[1].plot(curve["time"] - curve["reference_origin"], curve["rotation"], label=mode)
                with (out / f"{seq}_{mode}_curve.csv").open("w") as f:
                    np.savetxt(f, np.c_[curve["time"], curve["error"], curve["rotation"]], delimiter=",", header="timestamp,translation_error_m,rotation_error_deg")
        ax[0].set_ylabel("Translation error (m)")
        ax[1].set_ylabel("Rotation error (deg)")
        ax[1].set_xlabel("Seconds after OFF initialization")
        ax[0].legend(); fig.suptitle(seq); fig.tight_layout()
        fig.savefig(doc / f"{seq}_trajectory.png", dpi=160)
        plt.close(fig)
        # Complete timestamped event curves remain external.
        for mode in ("OFF", "G", "V", "GV"):
            data = rows(runs(out) / f"{seq}_{mode}_01/fusion.jsonl")
            fig, ax = plt.subplots(3, 1, figsize=(11, 7), sharex=True)
            t = np.array([r["camera_ns"] for r in data], dtype=np.int64)
            t = (t - t[0]) * 1e-9
            for branch in ("G", "V"):
                ax[0].plot(t, [r[branch+"_nis"] if r[branch+"_nis"] >= 0 else np.nan for r in data], label=branch)
                ax[1].plot(t, [r[branch+"_update_norm"] for r in data], label=branch)
                ax[2].plot(t, [r["ready_"+branch] for r in data], label=branch)
            ax[0].set_ylabel("NIS"); ax[1].set_ylabel("Contribution norm"); ax[2].set_ylabel("Ready")
            ax[2].set_xlabel("Input elapsed seconds"); ax[0].legend(); fig.tight_layout()
            fig.savefig(doc / f"{seq}_{mode}_events.png", dpi=140); plt.close(fig)
    write(doc / "metrics.json", result)
    write(doc / "events.json", diagnostics)
    with (doc / "metrics.csv").open("w") as f:
        writer = csv.writer(f)
        writer.writerow(["sequence", "mode", "ATE_m", "RPE_1s_m", "RPE_1s_deg", "RPE_5s_m", "RPE_5s_deg", "coverage", "samples", "status", "localization"])
        for m in result:
            rpe = m.get("rpe", {})
            writer.writerow([m["sequence"],m["mode"],m.get("ate_rmse_m"),
                             rpe.get("1.0",{}).get("translation_rmse_m"),rpe.get("1.0",{}).get("rotation_rmse_deg"),
                             rpe.get("5.0",{}).get("translation_rmse_m"),rpe.get("5.0",{}).get("rotation_rmse_deg"),
                             m.get("coverage"),m.get("samples"),m["status"],m.get("localization_status")])
    print(json.dumps(result, indent=2))

if __name__ == "__main__":
    main()
