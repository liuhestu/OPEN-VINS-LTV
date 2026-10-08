# Task C: source-mapped model, revision C1-prevalidation

Baseline: 2417d036002fbfd542ae71d96d0fc734cd6d47e8. Production defaults are unchanged.

Main error is true minus estimate, in the native left JPL quaternion chart and additive world translation/velocity/bias coordinates. Observer and seed error use estimate minus truth. Thus seed error is `-J_main delta_x + J_bearing epsilon`; the supplied-law implementation retains both cross terms in its covariance and main/seed cross block.

The actual observer performs forward Euler IMU mean and Riccati prediction, symmetrizes and spectrally floors P_R, performs controlled lifecycle/seed injection, constructs normalized bearing projection, chooses a camera substep count by ceil of the maximal projected rate, and at each substep updates mean and P_R and spectrally floors P_R. P_R is an internal gain metric; it is never labelled Sigma_L.

`LtvDiscreteSensitivity` differentiates mean and P_R jointly. The correction includes `dP C' r`, `P dC' r`, normalized raw-bearing derivative, measurement projection and the subsequent spectral floor. Floor derivative uses eigenvalue divided differences, with repeated eigenvalues treated by the derivative on their common fixed branch. A floor boundary is explicitly non-smooth. The ceil substep count is locally constant away from a switching boundary; changing the count is a separate event, never differentiated as a smooth function. Fixed calibration/clock is the first model scope. Extrinsic/time and noise-metric derivatives are not yet propagated by runtime callers.

The runtime observer records complete persistent corrected-input-offset and seeded-mean directional sensitivities, and complete same-frame raw-bearing sensitivities including their effect on subsequent P_R/gain substeps. It also retains historical fixed-gain blocks under their original names. The new derivatives are not a physical covariance and do not certify statistical consistency. Seed direction capacity is 32 births per epoch (102 columns including six offsets); reaching it must invalidate any claim of complete source coverage. Retired seed derivative columns are retained because past seeds can influence other states through gain coupling.

`LtvMainCrossShadow` is a separate read-only covariance companion. Actual StateHelper hooks cover initialization, full-state IMU transition, visual injection, clone augmentation and marginalization. Archived auxiliary variables survive main marginalization. Seed Jacobians are mapped from the actual context clone selectors into the full main layout; the runtime emits a bounded conditional seed/main cross component with missing historical bearing cross explicitly recorded. This conditional component must not be consumed by an unconditional correlated updater.

Native JPL injection is normalized `[dx/2,1]`, not Exp(dx). For pre-injection residual chart `delta=dx+r`, differentiation of the native product `q(dx+r) inverse(q(dx))` and rational chart `2 vector/scalar` gives

`J_reset=(I-skew(dx)/2)/(1+||dx||^2/4)`.

Position, velocity and bias error reset is identity. The companion applies this to every actual IMU/pose/quaternion injection without changing stored filter P. It propagates physical Sigma separately and logs `||Sigma_x-P_stored||`. The actual filter does not reset stored P at this site. This is a model distinction requiring evidence, not an automatic failure or excuse to skip source propagation.

Anchor timing must match B: current point is after IMU propagation and before current camera correction; diagnostic anchor is saved after the prior camera transaction. Main visual update follows the LTV camera workflow. A same-time/same-side anchor substitution is therefore invalid.

Remaining runtime dependencies: shared main/Observer raw IMU endpoint source mapping including interpolation, full historical seed/bearing cross, visual nullspace and measurement-compression source maps, calibration/time sensitivity, and an unconditional current/anchor error covariance. The generic seed and visual propagation APIs accept supplied nonzero source crosses; runtime hooks currently retain explicit missing flags instead of asserting those crosses are zero. C2/C3/C4 are not complete yet. Real landmark calibration remains NOT_EVALUATED without independent point truth.
