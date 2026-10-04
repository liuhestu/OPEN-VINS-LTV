# Two-event local correction operator reconstruction

Analysis297 completes the previously proposed minimum diagnostic:42 original camera events in the user-fixed±0.5s neighborhoods of receipts1413394964105760512 and1413394966255760384. Every event starts independently from its immediately preceding cached **full posterior x and P**. This is not a full observer replay, a new parameter candidate, or a feature-removal intervention. R3C remains on hold.

## Data sufficiency and exact operation sequence

The cache contains previous full x/P and physical-ID slots, every original raw IMU event, current calibration/bias, current managed observations, actual births and post-event slots. The frozen YAML supplies the original Q/V, birth P and spectrum/substep rules. The cache does **not** separately log the current pre-camera P; that matrix must be reconstructed and validated indirectly through the actual final full P. Reusing the preceding posterior P without IMU propagation, or inverting the camera update across spectral clipping, would be incorrect.

The independent new decoder `export_correction_cache.cpp` exports full P only in the two narrow neighborhoods and their immediate anchors. Existing276 binaries, data and formulas stay unchanged; its C++ decoder source subsequently received repository clang-format formatting only (the original source remains in87cb21d). New `local_correction_contributions.py` performs:

1. Original IMU interpolation, calibration/bias, corrected endpoint midpoint inputs and old-state Euler mean propagation. Each covariance step is `P += dt*(A P + P A^T + V)`, **not** `F P F^T + dt V`. Every step runs the original symmetric eigendecomposition, relative-negative rejection, eigenvalue floor reconstruction and final symmetrization, even when no eigenvalue is clamped.
2. Actual post-lifecycle slot ordering, preservation of every retained cross block, deletion of retired coordinates, independent identity birth P blocks and actual cached seed/zero means. Shared v/eta means are unchanged by lifecycle.
3. Camera observations sorted by core slot. With measured normalized camera bearing z, `b=R_BC*z`, `Pi=I-b b^T`; the appropriate landmark block of C is Pi and y is Pi*p_BC. These meter-valued projection innovations are different from the earlier unit-direction/angle residuals.
4. Original adaptive substep count from the largest eigenvalue of `q*C P C^T`. At each correction substep use the same old x/P to form `r=y-Cx`, `K=h*q*P*C^T`, then update x by K*r and P by `-h*q*P*C^T*C*P`. Reapply full spectrum sanitation after each substep and once again after the complete camera update.

The actual-path additive contribution for point i is the sum over substeps of `K_s[:,3i:3i+3]*r_s[3i:3i+3]`. Each term uses the realized shared state/covariance path; all points are evaluated simultaneously from the same substep old state. Terms telescope to the total actual update. This decomposition is not the result of removing i: removal would change C, covariance history, adaptive substeps and all other innovations.

## Predetermined checks and observed agreement

Before execution the limits were fixed at1e-9 for `||x_rebuilt−x_cache||/(1+||x_cache||)` and full Frobenius P relative error, and1e-9 absolute for signed shared Δ reconstruction. No tolerance was widened. Build294 passed. Test295 passed independent old-P simultaneous-update/term-sum checks and full retained cross-block/birth-zero-cross-block checks.

Analysis297 passed all42 events. Maximum normalized state error was `8.040755081333302e-15`; maximum full-P relative error `1.3948271456428435e-13`; maximum signed shared Δ absolute error `1.4835355166553654e-14`. All adaptive camera substep counts matched the cache. Predicted shared means matched the independently validated276 reconstruction exactly. Initial full x/P are the literal decoded preceding posterior; predicted P is reconstructed, not separately observed. Final full x/P are the direct validation oracle. NumPy eigensolve differs from Eigen arithmetic, so this is agreement at the predefined precision, not a bitwise claim.

Every event records the reconstructed full predicted P, after-lifecycle P, final P, full state vectors, slot ordering, every per-substep projection innovation and shared gain block, all per-point shared terms and spectrum diagnostics. The two target events have no eigenvalues clamped at the floor; nevertheless their unconditional eigen-reconstruction operations were retained.

## User-selected target findings

| Event | Camera substeps | Dominant point by actual velocity contribution norm | Point Δv, m/s | Point norm / total update norm |
|---|---:|---|---|---:|
|76.75s|5|27537|[0.0609716,0.1044608,0.1477465]|0.190941 /0.172563|
|78.90s|2|29636|[0.0725236,0.0222381,0.0836174]|0.112899 /0.111615|

At76.75s, ID27537 contributes1.100746 times the total update projected along the total Δv direction, while the other points partly oppose it. The next largest point contribution norm is0.010064m/s (ID28820). Thus this ID is no longer merely correlated with the immediate large correction: its **realized algebraic update term dominates that correction**. This does not, by itself, identify why its geometry/state became inconsistent or prove what an alternative algorithm would do without it. The main event reference review separately established that the combined correction increased true velocity error; no GT is read in this decomposition.

At78.90s, ID29636 contributes1.004789 times the total-update-direction projection. The five points born in that same camera event contribute at most `5.91e-20 m/s` to shared velocity across the actual two substeps. Fresh birth covariance has zero shared cross blocks, and camera-only correction does not create a substantial shared coupling from such an isolated block. Thus a direct same-frame shared correction caused by these new seed writes is inconsistent with the reconstructed operator; the dominant term is an already retained point. This does not exempt later effects of those births after IMU propagation.

These are realized operator facts, not a deletion counterfactual or a new readiness threshold. The approximate seed confidence, observability, possible bearing mismatch and state/covariance dynamics remain separate mechanisms to investigate before proposing a change. No candidate was registered and no threshold, production source or historical evidence was changed.

Evidence: `/home/he/output/ltv_passive_hardening_v2/diagnostics/R3B_local_contributions_01/summary.json` and `events.jsonl`. All42 events and failures policy remain visible. The prior unvalidated R3C freeze drafts were archived as `drafts/r3c_unvalidated_freeze.patch`; the three working files were restored exactly to e792f41 before this diagnostic work.

Formatting follow-up: the original build294 decoder source is archived as `diagnostics/archive/export_correction_cache_build294.cpp` (SHA `c3aa162bc28384464ba35a9e6c65ffbe3236c2d8f1e7dc24e4ef582978693718`). Repository clang-format was then applied; build299 passed. Original `_01` and formatted `_02` decoder executables have identical SHA `0f5cd11f620ada445e839f51a6b6c2489e5aa15ab7bbefa3521789afed129e6c`. Source SHA changed only due to formatting and is not relabeled as build294's source. Analysis297 output remains under its original identities and was not rerun or overwritten.
