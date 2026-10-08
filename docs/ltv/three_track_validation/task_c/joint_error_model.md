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

## Program perturbations, true errors and stored P (correction before first finite replay)

Three matrices must remain distinct: stored EKF P; covariance of the program's local estimated-state perturbation; covariance of actual native true-est errors. The finite source engine stores `m=-xi_est`, where `q_est_pert=q(xi_est) q_est_nom` in the local native quaternion chart, with analogous negative additive perturbations for other main variables. It is not automatically the actual true-est error when the nominal attitude error is nonzero.

For `q(dx)=normalize([dx/2,1])`, perturbing the native injection gives

`xi_new = Ad(q(dx)) xi_old + J_dx delta_dx`,

`J_dx=(I-skew(dx)/2)/(1+||dx||^2/4)`,

`Ad(q(dx))=((1-||dx||^2/4) I + dx dx'/2 - skew(dx))/(1+||dx||^2/4)`.

Consequently the finite program factor updates orientation with `Ad * m_old - J_dx * delta_dx_factor`. Multiplying an unconditional program factor `(I-KH) m - K epsilon` by the posterior-centered J_dx alone is wrong at nonzero nominal dx. The external stored-P companion's conditional posterior reset and the finite program factor are separately named.

If the actual nominal true-est rational attitude error is `e0=2 vec(q_true inverse(q_est))/scalar`, then at fixed physical truth

`delta_e_main = M(e0) m`, with `M(e0)=I-skew(e0)/2+e0 e0'/4`.

Position, world velocity and sensor biases are additive, so their mapping is identity at fixed truth. The main/clones pose layout is exported for independent offline GT mapping with the same frozen SE(3) gauge. At e0=0 this mapping is identity; the zero-nominal-main-error controlled experiment does not validate nonzero-error real covariance. Main H/K/FEJ/QR schedule remains a declared nominal linear innovation model; its stochastic gain and Jacobian variation is outside this finite layer. Initial main error covariance is a declared prior law, not a measured true covariance nor a derivation of static-initializer/history-source correlations.

Observer coordinates are physical IMU body vectors at a fixed physical timestamp, with fixed calibration. When true landmark/velocity/gravity vectors are held fixed across the noise ensemble, `delta e_Observer=delta program_mean` for the est-true convention. Thus its program-mean covariance equals the modelled error covariance under that ensemble even when the deterministic mean error is nonzero. This identity does not validate the input noise law, landmark identity, mean bias, non-smooth selection distribution, or real error calibration. Real landmark truth is unavailable, so empirical real calibration is NOT_EVALUATED. The finite one-slot actor also does not inherit the production 30-slot covariance/coupling.
