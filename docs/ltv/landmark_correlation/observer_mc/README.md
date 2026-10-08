# Native nonlinear Observer fragment Monte Carlo

The sampling contract was frozen before the run in `contract.json` and copied
with its SHA256 to the raw run directory. This experiment executes the unchanged
production Observer sources with all default numerical parameters and readiness
20; only observer enable is true for the controlled test. Two landmark seed
means are fixed, two actual IMU Euler steps and one actual camera event execute.
The camera updates its real Riccati metric/gain/eigenvalue sanitization. The
same physical source draw drives both IMU steps and the specified bearings.

All three predeclared amplitudes ran 4000 independent draws each. Full two-sided
native finite differences at 1e-6 and 1e-7 predict the 36-dimensional joint output
perturbation covariance across both intermediate states and final camera state.
The finite-difference Jacobians differed by Frobenius norm 5.616e-9. Predicted and
sample covariance relative errors were 2.289%, 3.412%, and 2.778%. Nonlinear
remainder RMS increased from 5.525e-11 through 5.764e-9 to 5.617e-7; all amplitudes
are retained. These are descriptive local linearization diagnostics, with no
Gaussian confidence-interval, consistency PASS or independent truth claim.

The full native camera bearing finite-difference map differs from the shadow's
fixed-gain tangent map by norm 0.0085742 (normalization Jacobian accounted for).
This difference is preserved in `gain_schedule_difference.csv`; it confirms that
nominal fixed-gain shadow maps do not describe the unconditional actual gain
sensitivity. `results.json` provides the relative difference and input/source,
binary, library and raw-file hashes.

The actual “errors” here are perturbed outputs minus a deterministic nominal
output, conditional on fixed seed means, rather than independently measured
estimate-minus-landmark-truth errors. Neither EuRoC landmark truth nor full
historical/main-state cross covariance is supplied. This controlled experiment
therefore extends LC-06 implementation evidence while keeping real landmark
calibration NOT_EVALUATED and full Observer statistical consistency NOT_ACHIEVED.

Raw CSVs are in
`/home/he/open_vins_ltv_ws/src/open_vins_ltv/results/nondegradation_validation/observer_mc_20261008/`.
They include every 8-dimensional draw, centered output errors, uncentered linear
predictions, nonlinear remainders, full predicted/sample 36x36 covariance, full
native Jacobian, and fixed/full camera map comparison. Uncentered output errors
can be reconstructed as linear prediction plus nonlinear remainder. Build and
run commands and all raw-file hashes are in `results.json`. The raw run used
0.57 seconds and peak RSS 8752 KiB on this machine; these local test resource
figures are not candidate production realtime acceptance evidence.
