#!/usr/bin/env python3
"""Offline checks of the proposed contract; no estimator connection or replay."""
import json
from pathlib import Path
import numpy as np


def skew(v):
    x, y, z = v
    return np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])


def exp_so3(v):
    angle = np.linalg.norm(v)
    k = skew(v)
    if angle < 1e-8:
        return np.eye(3) + k + k @ k / 2
    return np.eye(3) + np.sin(angle) / angle * k + (1 - np.cos(angle)) / angle**2 * k @ k


def predict(rt, ra, pt, pa, ell):
    return rt @ (ra.T @ ell + pa - pt)


def main():
    rng = np.random.default_rng(20261005)
    result = {"scope": "offline contract math only; no ON implementation or replay", "seed": 20261005}
    jac_error = gauge_error = 0.
    for _ in range(32):
        rt, ra = exp_so3(rng.normal(size=3)), exp_so3(rng.normal(size=3))
        pt, pa, ell = rng.normal(size=(3, 3))
        h = predict(rt, ra, pt, pa, ell)
        analytical = np.hstack([skew(h), -rt @ ra.T @ skew(ell), -rt, rt])
        numerical = np.empty((3, 12))
        eps = 1e-6
        for j in range(12):
            step = np.zeros(12)
            step[j] = eps
            values = []
            for sign in [1., -1.]:
                d = step * sign
                # Native JPL: R' = Exp(-delta_theta) R; p' = p + delta_p.
                values.append(predict(exp_so3(-d[:3]) @ rt, exp_so3(-d[3:6]) @ ra,
                                      pt + d[6:9], pa + d[9:12], ell))
            numerical[:, j] = (values[0] - values[1]) / (2 * eps)
        jac_error = max(jac_error, float(np.max(np.abs(analytical - numerical))))
        q = exp_so3(rng.normal(size=3))
        translation = rng.normal(size=3)
        transformed = predict(rt @ q.T, ra @ q.T, q @ pt + translation, q @ pa + translation, ell)
        gauge_error = max(gauge_error, float(np.max(np.abs(h - transformed))))
    assert jac_error < 2e-8 and gauge_error < 2e-13
    result["jacobian"] = {"cases": 32, "max_absolute_error": jac_error, "passed": True}
    result["world_gauge"] = {"cases": 32, "max_absolute_error": gauge_error, "passed": True}

    # Joint variables [delta_x, e_anchor, e_current, nu_visual].
    n = 8
    source = rng.normal(size=(n + 9, n + 12))
    joint = source @ source.T + np.eye(n + 9) * .1
    a = exp_so3(rng.normal(size=3))
    transform = np.zeros((6, n + 9))
    transform[:3, n:n+3] = -a
    transform[:3, n+3:n+6] = np.eye(3)
    transform[3:, n+6:n+9] = np.eye(3)
    p = joint[:n, :n]
    r = transform @ joint @ transform.T
    cross = joint[:n] @ transform.T
    saa, stt = joint[n:n+3, n:n+3], joint[n+3:n+6, n+3:n+6]
    sat = joint[n:n+3, n+3:n+6]
    expected_r = stt + a @ saa @ a.T - a @ sat - sat.T @ a.T
    expected_cross = joint[:n, n+3:n+6] - joint[:n, n:n+3] @ a.T
    expected_visual = joint[n+3:n+6, n+6:n+9] - a @ joint[n:n+3, n+6:n+9]
    noise_error = max(float(np.max(np.abs(r[:3, :3] - expected_r))),
                      float(np.max(np.abs(cross[:, :3] - expected_cross))),
                      float(np.max(np.abs(r[:3, 3:] - expected_visual))))
    assert noise_error < 2e-13
    result["joint_noise_elimination"] = {"max_absolute_error": noise_error, "passed": True}

    h = rng.normal(size=(6, n))
    s = h @ p @ h.T + r + h @ cross + cross.T @ h.T
    b = p @ h.T + cross
    k = np.linalg.solve(s, b.T).T
    conditioned = p - b @ np.linalg.solve(s, b.T)
    j = np.eye(n) - k @ h
    joseph = j @ p @ j.T + k @ r @ k.T - j @ cross @ k.T - k @ cross.T @ j.T
    # Independent check: transform the complete source joint distribution to [x, residual].
    measurement_map = transform.copy()
    measurement_map[:, :n] = h
    reference_s = measurement_map @ joint @ measurement_map.T
    reference_b = joint[:n] @ measurement_map.T
    reference = p - reference_b @ np.linalg.solve(reference_s, reference_b.T)
    covariance_error = max(float(np.max(np.abs(conditioned - joseph))),
                           float(np.max(np.abs(conditioned - reference))))
    minimum_eigenvalue = float(np.linalg.eigvalsh((conditioned + conditioned.T) / 2).min())
    assert covariance_error < 2e-12 and minimum_eigenvalue > 0
    result["correlated_update"] = {"max_absolute_error": covariance_error,
                                   "posterior_min_eigenvalue": minimum_eigenvalue, "passed": True}

    # y = x_hat + epsilon contains no new x information: nu = -delta_x + epsilon.
    variance, eps_variance = 2., .3
    duplicate_h, duplicate_n, duplicate_r = 1., -variance, variance + eps_variance
    duplicate_s = duplicate_h**2 * variance + duplicate_r + 2 * duplicate_h * duplicate_n
    duplicate_b = variance * duplicate_h + duplicate_n
    duplicate_p = variance - duplicate_b**2 / duplicate_s
    naive_p = variance - variance**2 / (variance + duplicate_r)
    zero_noise_s = variance + variance - 2 * variance
    assert duplicate_b == 0 and duplicate_p == variance and naive_p < variance and zero_noise_s == 0
    result["duplicate_prior_information"] = {
        "correct_gain": duplicate_b / duplicate_s, "correct_posterior_variance": duplicate_p,
        "naive_independent_posterior_variance": naive_p,
        "exact_duplicate_innovation_rank": 0, "passed": True,
    }
    result["passed"] = True
    output = Path(__file__).with_name("math_checks.json")
    output.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
