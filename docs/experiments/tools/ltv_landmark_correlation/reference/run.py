#!/usr/bin/env python3
"""Offline LC04-06 reference. No imports from, or writes to, estimator code.

Two paths: explicit independent physical-source maps and covariance-block
elimination. Singular distributions retain their effective subspace unchanged.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.stats import chi2, norm

ROOT = Path(__file__).resolve().parents[5]
CONTRACT = ROOT / "docs/ltv/landmark_correlation/reference/experiment_contract.json"
X, ANCHOR, CURRENT, VISUAL = slice(0, 3), slice(3, 6), slice(6, 9), slice(9, 11)
H = np.array([[0.8, 0.2, -0.1], [-0.1, 0.6, 0.3], [0.2, -0.3, 0.7]])
theta = 0.35
A = np.array([[np.cos(theta), -np.sin(theta), 0], [np.sin(theta), np.cos(theta), 0], [0, 0, 1]])
D = np.column_stack((H, -A, np.eye(3), np.zeros((3, 2))))


def effective_inverse(matrix, tolerance=1e-11):
    """Do not repair incompatible covariance. Drop only negligible PSD modes."""
    matrix = (matrix + matrix.T) / 2
    values, vectors = np.linalg.eigh(matrix)
    threshold = tolerance * max(float(np.max(np.abs(values))), 1.0)
    if values.min() < -threshold:
        raise ValueError("incompatible covariance: negative eigenvalue")
    selected = values > threshold
    inverse = (vectors[:, selected] / values[selected]) @ vectors[:, selected].T
    return inverse, int(selected.sum()), values


def source_map(scene):
    """Rows are consumer errors; columns are uniquely identified unit sources."""
    m = np.zeros((11, 15))
    m[X, :3] = np.diag([0.7, 0.6, 0.5])
    m[ANCHOR, 3:6] = np.diag([0.5, 0.4, 0.3])
    m[CURRENT, 6:9] = np.diag([0.6, 0.5, 0.4])
    m[VISUAL, 9:11] = np.diag([0.45, 0.35])
    if scene == "zero_noise":
        m[:] = 0
    elif scene == "shared_imu_bias":
        m[X, 11:14] = np.diag([0.8, 0.7, 0.6])
        m[ANCHOR, 11:14] = np.eye(3) * 0.45
        m[CURRENT, 11:14] = np.array([[0.65, 0.1, 0], [0, 0.6, 0.15], [0.1, 0, 0.5]])
    elif scene == "shared_bearing":
        m[ANCHOR, 9:11] = np.array([[0.7, 0.1], [-0.2, 0.6], [0.35, -0.3]])
        m[CURRENT, 9:11] = np.array([[0.8, -0.1], [0.1, 0.5], [0.3, 0.2]])
        m[X, 9:11] = np.array([[0.4, 0], [0.2, 0.5], [-0.1, 0.3]])
    elif scene == "seed_feedback":
        # Same historical main errors seed two point estimates and visual rows.
        m[ANCHOR, :3] = np.array([[0.9, 0.1, 0], [0, 0.7, 0.2], [0.15, 0, 0.8]]) @ m[X, :3]
        m[CURRENT, :3] = A @ m[ANCHOR, :3] - 0.65 * H @ m[X, :3]
        m[VISUAL, :3] = np.array([[0.4, 0.1, 0], [0, 0.3, 0.2]]) @ m[X, :3]
    elif scene == "duplicate_information":
        # Exact cancellation: observer/anchor constraint is already determined.
        m[CURRENT, :] = A @ m[ANCHOR, :] - H @ m[X, :]
    elif scene == "near_rank_deficiency":
        # Two exactly duplicate residual rows plus sub-threshold third noise.
        u = np.array([0.7, -0.2, 0.5])
        residual_sources = np.zeros((3, 15))
        residual_sources[0, :3] = u
        residual_sources[1, :3] = 2 * u
        residual_sources[2, 14] = 1e-7
        m[CURRENT, :] = A @ m[ANCHOR, :] - H @ m[X, :] + residual_sources
    elif scene != "independent_sources":
        raise ValueError(scene)
    return m


def block_model(q):
    p = q[X, X]
    r = q[CURRENT, CURRENT] + A @ q[ANCHOR, ANCHOR] @ A.T - A @ q[ANCHOR, CURRENT] - q[CURRENT, ANCHOR] @ A.T
    n = q[X, CURRENT] - q[X, ANCHOR] @ A.T
    r_visual = q[CURRENT, VISUAL] - A @ q[ANCHOR, VISUAL]
    s = H @ p @ H.T + r + H @ n + n.T @ H.T
    b = p @ H.T + n
    inverse, rank, spectrum = effective_inverse(s)
    k = b @ inverse
    posterior = p - b @ inverse @ b.T
    i_kh = np.eye(3) - k @ H
    joseph = i_kh @ p @ i_kh.T + k @ r @ k.T - i_kh @ n @ k.T - k @ n.T @ i_kh.T
    return dict(P=p, R=r, N=n, R_L_visual=r_visual, S=s, B=b, K=k, P_post=posterior, joseph=joseph,
                rank=rank, spectrum=spectrum)


def conditional_information(q):
    # Additional information after the existing visual residual is conditioned.
    rv = np.vstack((D, np.eye(11)[VISUAL]))
    joint = rv @ q @ rv.T
    cv = q[X, VISUAL]
    iv, _, _ = effective_inverse(q[VISUAL, VISUAL])
    baseline = q[X, X] - cv @ iv @ cv.T
    all_cross = q[X, :] @ rv.T
    ij, _, _ = effective_inverse(joint)
    posterior = q[X, X] - all_cross @ ij @ all_cross.T
    delta = (baseline - posterior + (baseline - posterior).T) / 2
    _, rank, spectrum = effective_inverse(delta)
    return delta, rank, spectrum


def verify_algebra(scene, m):
    q = m @ m.T
    b = block_model(q)
    # Independent reference uses noise source coefficients, not the block formula.
    residual_map = D @ m
    nuisance_map = m[CURRENT] - A @ m[ANCHOR]
    ref_s = residual_map @ residual_map.T
    ref_b = m[X] @ residual_map.T
    # Independent reference solves using the source-map SVD, not the covariance
    # eigensolver used by block_model. The same frozen physical rank convention
    # applies to both paths, including an absolute floor for exact duplicates.
    left, sigma, _ = np.linalg.svd(residual_map, full_matrices=False)
    selected = sigma ** 2 > 1e-11 * max(float(np.max(sigma ** 2)), 1.0)
    source_inverse = (left[:, selected] / sigma[selected] ** 2) @ left[:, selected].T
    ref_k = ref_b @ source_inverse
    post_map = m[X] - ref_k @ residual_map
    checks = {
        "R": np.max(np.abs(b["R"] - nuisance_map @ nuisance_map.T)),
        "N": np.max(np.abs(b["N"] - m[X] @ nuisance_map.T)),
        "R_L_visual": np.max(np.abs(b["R_L_visual"] - nuisance_map @ m[VISUAL].T)),
        "S": np.max(np.abs(b["S"] - ref_s)),
        "B": np.max(np.abs(b["B"] - ref_b)),
        "P_conditional": np.max(np.abs(b["P_post"] - post_map @ post_map.T)),
        "generalized_Joseph": np.max(np.abs(b["joseph"] - b["P_post"]))
    }
    # Explicit source projection after conditioning on visual uses row-space SVD.
    visual_sources = m[VISUAL]
    rv_sources = np.vstack((residual_map, visual_sources))
    def source_posterior(rows):
        _, sigma, vh = np.linalg.svd(rows, full_matrices=False)
        selected = sigma ** 2 > 1e-11 * max(float(np.max(sigma ** 2)), 1.0)
        projection = np.eye(m.shape[1]) - vh[selected].T @ vh[selected]
        return m[X] @ projection @ m[X].T
    delta, information_rank, info_spectrum = conditional_information(q)
    source_delta = source_posterior(visual_sources) - source_posterior(rv_sources)
    checks["conditional_information"] = np.max(np.abs(delta - source_delta))
    tolerance = 1e-10
    ok = all(v <= tolerance for v in checks.values())
    if scene == "duplicate_information":
        ok &= b["rank"] == 0 and information_rank == 0 and np.linalg.norm(b["K"]) < tolerance
    return dict(scene=scene, status="PASS" if ok else "FAIL", errors=checks, residual_rank=b["rank"],
                conditional_information_rank=information_rank, information_spectrum=info_spectrum.tolist(),
                matrices={key: value.tolist() for key, value in b.items() if isinstance(value, np.ndarray)})


def approximation(scene, m):
    q = m @ m.T
    independent = np.zeros_like(q)
    for group in [X, ANCHOR, CURRENT, VISUAL]:
        independent[group, group] = q[group, group]
    rows = []
    for name, covariance in [("full", q), ("zero_all_consumer_cross_blocks_DIAGNOSTIC_ONLY", independent)]:
        try:
            model = block_model(covariance)
            delta, rank, spectrum = conditional_information(covariance)
            rows.append(dict(scene=scene, model=name, incompatible=0, residual_rank=model["rank"],
                             S_eigenvalues=json.dumps(model["spectrum"].tolist()), K=json.dumps(model["K"].tolist()),
                             Kr=json.dumps((model["K"] @ np.array([0.1, -0.08, 0.05])).tolist()),
                             prior_trace=np.trace(model["P"]), posterior_trace=np.trace(model["P_post"]),
                             information_trace=np.trace(delta), information_rank=rank,
                             information_eigenvalues=json.dumps(spectrum.tolist())))
        except ValueError as exc:
            rows.append(dict(scene=scene, model=name, incompatible=1, reason=str(exc)))
    return rows


def projections(dimension):
    entries = []
    for i in range(dimension):
        vector = np.eye(dimension)[i]
        entries.append((i, i, vector, None))
        for j in range(i + 1, dimension):
            entries.append((i, j, vector + np.eye(dimension)[j], vector - np.eye(dimension)[j]))
    return entries


def covariance_intervals(samples, q, alpha_per_projection):
    n, dimension = samples.shape
    df = n - 1
    low_quantile = chi2.ppf(alpha_per_projection / 2, df)
    high_quantile = chi2.ppf(1 - alpha_per_projection / 2, df)
    covariance = np.cov(samples, rowvar=False)
    rows = []
    def interval(vector):
        variance = np.var(samples @ vector, ddof=1)
        return df * variance / high_quantile, df * variance / low_quantile
    for i, j, plus, minus in projections(dimension):
        lp, up = interval(plus)
        if minus is None:
            lower, upper = lp, up
        else:
            lm, um = interval(minus)
            lower, upper = (lp - um) / 4, (up - lm) / 4
        predicted = float(q[i, j])
        # Analytic zeros in singular systems can differ by roundoff only.
        pass_entry = lower - 1e-12 <= predicted <= upper + 1e-12
        rows.append(dict(i=i, j=j, predicted=predicted, sample=float(covariance[i, j]),
                         lower=float(lower), upper=float(upper), passes=int(pass_entry)))
    return rows


def nonlinear_seed(rng, n, amplitude):
    # Physical inputs [main depth, horizontal bearing, stereo disparity].
    q = amplitude ** 2 * np.array([[1, 0.25, 0.4], [0.25, 1, -0.15], [0.4, -0.15, 1]])
    noise = rng.multivariate_normal(np.zeros(3), q, size=n)
    baseline, disparity, bearing = 0.12, 0.04, 0.2
    z = baseline / disparity
    noisy_z = baseline / (disparity + noise[:, 2])
    # The x error is true-minus-estimate; seed point uses the same source draw.
    samples = np.column_stack((noise[:, 0], (bearing + noise[:, 1]) * noisy_z - bearing * z + noise[:, 0],
                               noisy_z - z + noise[:, 0]))
    jacobian = np.array([[1, 0, 0], [1, z, -bearing * baseline / disparity ** 2], [1, 0, -baseline / disparity ** 2]])
    return samples, jacobian @ q @ jacobian.T, float(np.min(np.abs(disparity + noise[:, 2])))


def write_csv(path, rows):
    fields = list(dict.fromkeys(key for row in rows for key in row))
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fields)
        writer.writeheader()
        writer.writerows(rows)


def run(output):
    contract = json.loads(CONTRACT.read_text())
    output.mkdir(parents=True, exist_ok=True)
    monte = contract["monte_carlo"]
    n = monte["independent_draws"]
    rng = np.random.default_rng(monte["seed"])
    scenes = monte["scenes"]
    # dimension squared projections per Gaussian distribution, plus nonlinear diagnostic family.
    family_projections = (len(scenes) + 1) * 11 ** 2 + len(monte["local_nonlinear_amplitudes"]) * 3 ** 2
    alpha = monte["family_alpha"] / family_projections
    math_results, diagnostic, summary, intervals, matrices = [], [], [], [], {}
    for scene in scenes:
        m = source_map(scene)
        q = m @ m.T
        math_results.append(verify_algebra(scene, m))
        diagnostic.extend(approximation(scene, m))
        draws = rng.standard_normal((n, m.shape[1])) @ m.T
        rows = covariance_intervals(draws, q, alpha)
        intervals.extend(dict(scene=scene, **r) for r in rows)
        summary.append(dict(scene=scene, draws=n, seed=monte["seed"], expectation="PASS",
                            rejected_entries=sum(1 - r["passes"] for r in rows),
                            status="PASS" if all(r["passes"] for r in rows) else "FAIL",
                            max_absolute_covariance_error=max(abs(r["sample"] - r["predicted"]) for r in rows)))
        matrices[scene] = dict(source_map=m.tolist(), predicted=q.tolist(), sample=np.cov(draws, rowvar=False).tolist(),
                               source_ids=[f"{scene}:physical_source:{i}" for i in range(m.shape[1])])
    # Deliberately violate the physical source identity, preserving marginal blocks.
    m = source_map("shared_imu_bias")
    q = m @ m.T
    broken = np.empty((n, 11))
    for block in [X, ANCHOR, CURRENT, VISUAL]:
        broken[:, block] = rng.standard_normal((n, m.shape[1])) @ m[block].T
    rows = covariance_intervals(broken, q, alpha)
    rejected = sum(1 - r["passes"] for r in rows)
    intervals.extend(dict(scene="broken_shared_source", **r) for r in rows)
    z = norm.ppf(1 - alpha / 2)
    # Covariance error of 0.35 normalized by sqrt(P_ii P_jj); rho=0 under broken sampling.
    shift = monte["minimum_detectable_normalized_covariance_error"] * np.sqrt(n - 1)
    power = float(norm.cdf(-z - shift) + norm.sf(z - shift))
    summary.append(dict(scene="broken_shared_source", draws=n, seed=monte["seed"], expectation="REJECT",
                        rejected_entries=rejected, normal_approximation_power=power,
                        status="PASS" if rejected > 0 and power >= monte["minimum_normal_approximation_power"] else "INCONCLUSIVE"))
    for amplitude in monte["local_nonlinear_amplitudes"]:
        samples, predicted, min_disparity = nonlinear_seed(rng, n, amplitude)
        scene = f"controlled_stereo_seed_amplitude_{amplitude}"
        rows = covariance_intervals(samples, predicted, alpha)
        intervals.extend(dict(scene=scene, **r) for r in rows)
        rejected = sum(1 - r["passes"] for r in rows)
        summary.append(dict(scene=scene, draws=n, seed=monte["seed"], expectation="DIAGNOSTIC",
                            rejected_entries=rejected, status="LINEARIZATION_DEVIATION" if rejected else "NO_DEVIATION_DETECTED",
                            minimum_disparity=min_disparity, independent_landmark_truth="NO"))
        matrices[scene] = dict(predicted=predicted.tolist(), sample=np.cov(samples, rowvar=False).tolist(),
                               mean_error=np.mean(samples, axis=0).tolist())
    result = dict(contract_sha256=hashlib.sha256(CONTRACT.read_bytes()).hexdigest(),
                  implementation_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                  family_projection_tests=family_projections, per_projection_alpha=alpha,
                  math=math_results, monte_carlo=summary,
                  real_landmark_statistical_calibration="NOT_EVALUATED",
                  online_fusion_authorization="NONE")
    (output / "math_and_lifecycle_checks.json").write_text(json.dumps(result, indent=2) + "\n")
    (output / "joint_matrices.json").write_text(json.dumps(matrices, indent=2) + "\n")
    write_csv(output / "monte_carlo_summary.csv", summary)
    write_csv(output / "covariance_confidence_intervals.csv", intervals)
    write_csv(output / "approximation_comparison.csv", diagnostic)
    print(json.dumps(dict(math=[(r["scene"], r["status"]) for r in math_results], monte_carlo=summary), indent=2))
    return 0 if all(r["status"] == "PASS" for r in math_results) and all(r["status"] == "PASS" for r in summary if r["expectation"] != "DIAGNOSTIC") else 1


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    raise SystemExit(run(args.output))
