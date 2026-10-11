# Independent geometry review, before execution

Reviewed `LtvSeedEstimator`, `LtvSeedUncertainty`, authored geometry test and geometry contract. No executable tests run by reviewer.

The point-line implicit derivative has the correct sign: `A df = Σ[H dc + ((dᵀ(f−c))I+d(f−c)ᵀ) dd]`. Right rotation gives the implemented negative skew direction/lever-arm derivatives. Differentiating the execution transform adds `skew(ell_B)` and `−R_WBᵀ` in the same input columns, correctly retaining correlations when the execution pose is an observation pose. The common-world gauge construction in the test is consistent with these coordinates. Full `J Sigma Jᵀ` uses supplied cross blocks, and no covariance jitter is added. The risk is explicitly a frozen-ray local risk, not a range confidence interval.

Findings sent to owner:

- Degeneracy test had an invalid anchor after resizing observations from six to two (`anchor_observation` remained 4). It exercised INVALID_INPUT instead of DEGENERATE. Set a valid anchor and require the exact rejection reason.
- Public uncertainty methods trust `mean.valid` and index supplied input directly. A stale or forged valid mean paired with another malformed input can cause invalid indexing or an incorrect derivative. Validate association or enforce a checked input/mean interface; test invalid indices and mismatched inputs.
- Canonicalize duplicate camera events/pose identities before forming covariance. The input representation permits duplicated pose timestamps or identical `(pose_index,camera_index)` observations; blindly assigning independent noise to copies would create false information. This is especially relevant to replay/resume and repeated stereo packets.

Remaining acceptance requires actual finite-difference execution, Monte Carlo with shared pose perturbations and frozen solve allocation, new-path call-site convention checks, and covariance-source labelling. Static inspection alone does not certify uncertainty calibration.
