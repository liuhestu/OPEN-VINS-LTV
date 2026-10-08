# Task C joint model: current implementation and admissible scope

Baseline 2417d036002fbfd542ae71d96d0fc734cd6d47e8. Corrected runtime binary source6982e0473b6963ab95dfb169b585c09f464bfa8e; later analysis/pure-kernel sources have separate recorded OIDs. No production switch, confirm20 or max_slam default changed.

## Three different covariance objects

1. Stored main EKF P and Observer P_R are algorithm design/gain states. Neither is automatically a measured true-error covariance.
2. The finite factor stores main `m=-xi_est`: negative native local program-estimate perturbation about the nominal run. Additive position/velocity/bias variation equals true-est error variation at fixed truth; attitude requires a mean-dependent chart map.
3. At actual nominal attitude error `e0=2 vec(q_true inverse(q_est))/scalar`, true-est error variation is `M(e0) m`, where `M=I-skew(e0)/2+e0 e0'/4`. An actual GT mean or a declared zero-error approximation is necessary. The offline mapper uses a single complete OFF-fitted SE(3) gauge; it does not use GT in online gates/fusion and does not validate the initial/source law.

Observer point/velocity/gravity coordinates are physical IMU body vectors at fixed time and calibration. Under fixed physical truth, their est-true error variation equals program-mean variation even if the deterministic error mean is nonzero. Real input noise, identities, mean bias and empirical landmark-error calibration are not proved by that identity.

## Actual discrete Observer sensitivities

`LtvDiscreteSensitivity.h`, `LtvJointFactors.h` and native `ltv_observer.cpp` hooks differentiate mean and the entire P_R metric together. They include corrected acceleration/gyro, gain dependence, raw-bearing normalization, projection/y, repeated same-frame substeps and spectral flooring. P_R derivatives feed subsequent gain derivatives; no fixed-gain substitution is made in the finite source actor.

Native order is IMU Euler mean/P_R prediction → spectral sanitize → controlled lifecycle/seed → normalized bearing projection → ceil-based adaptive substeps → each camera mean/P_R update and spectral sanitize. The spectral derivative uses divided differences within a fixed branch. Floor/ceil/reset/identity switching are separate events, not globally smooth Jacobians. The historical `LtvErrorShadow` named fixed-gain blocks remain diagnostic and explicitly unqualified; its full-gain offset/seed/raw-bearing derivative blocks are sensitivities, not physical covariance.

## Finite actual source actor

`LtvFiniteJointShadow` runs one independently configured native Observer slot using the same q/V/P_R/floor/substep rules and fixed calibration/clock. This changes its coupling relative to the production30-slot Observer; it cannot be used as a cropped production covariance. Its initial main covariance is an explicitly supplied prior law, not a derivation of actual static-initializer/history-source errors. Main innovation H/K/FEJ/QR schedule is conditioned on the nominal run; stochastic gain/Jacobian variation is not included.

Implemented source paths:

- Actual main mean integrator: two raw endpoints and initial IMU state are differentiated through the selected native discrete/RK4/analytic mean function in a separate local State. Stored G/Q is not substituted for this input map.
- Raw IMU endpoints: timestamps and interpolation weights retain one shared latent draw across adjacent substeps and camera boundaries. Per-sample variance is sigma_density²/raw_dt under the declared white hypothesis. Reusing a source checks dimension/variance; densities cannot change within a model version.
- Prior bias receipt: VioManager captures biases before main propagation. A separate factor is frozen at that exact phase and reused during later Observer integration; current post-propagation bias covariance is not silently substituted.
- Seed history: actual geometry Jacobian, actual full clone selector and pixel-to-bearing tangent maps are used. Already-consumed pixels before registration are rejected; this is the fresh-history subset, not all initialization history.
- Main visual injection: the exact native nullspace/compression Givens rotations also transform registered raw pixel columns. Shared seed/current-bearing/visual sources remain correlated. Unregistered visual remainder is a declared fresh independent white model; duplicate receipt and late registration audits constrain this assumption.
- Clone augmentation and marginalization transform the factor rows. Past effects persist in retained main/Observer/source/anchor factors. Sources expire only under the actual retained-clone and maximum-supported IMU bracket window; reuse after expiry is rejected.
- QR compression stacks ALL retained states, archived anchors and source handles. D'=Q R implies D D'=R' R, so replacing D by R' retains all pairwise covariance and source reuse, without covariance clipping.

Unknown/unqualified: actual initializer-source prior law, already-consumed history outside the registered subset, stochastic main H/K/FEJ/QR effects, true physical process/noise/identity law in EuRoC, changing calibration/time, production30-slot covariance and any real landmark statistical calibration.

## Native injection and true-error reset

For native `q(dx)=normalize([dx/2,1])`, the conditional posterior-centered reset at old error≈dx is

`J_dx=(I-skew(dx)/2)/(1+||dx||²/4)`.

Actual program injection has TWO independent perturbation inputs:

`xi_new=Ad(q(dx)) xi_old + J_dx delta_dx`,

`Ad=((1-||dx||²/4)I+dx dx'/2-skew(dx))/(1+||dx||²/4)`.

Thus the finite program factor uses `Ad*m_old - J_dx*delta_dx_factor`; applying J_dx alone to `(I-KH)m-Kepsilon` is incorrect at nonzero nominal dx. The original stored-P companion keeps its separately named conditional posterior calculation. Corrected code and FD explicitly distinguish these maps. GT mapping M(e0) is a third operation, not a renaming of either matrix.

## Current/anchor residual and correlated solve

Physical/program coordinates must be declared before composing blocks. With current pre-camera point l_t, prior post-camera anchor l_a, R_i=world-to-body and world position p_i:

`r=l_t-R_t(R_a' l_a+p_a-p_t)`, `A=R_t R_a'`.

For main program columns `[theta_t,p_t,theta_a,p_a]`, `m=-xi_est`, the actual partial pose Jacobian is

`H=[skew(predicted), -R_t, -A skew(l_a), R_t]`.

For actual true-error main coordinates, compose H with M(e0)^{-1} for every attitude block. Noise moments at declared nominal A are

`R_L=Sigma_tt+A Sigma_aa A' - Sigma_ta A' - A Sigma_at`,

`N_L=C_xt-C_xa A'`,

`Cov(n_L,n_vis)=Cov(e_t,n_vis)-A Cov(e_a,n_vis)`.

The full residual mean and any random-A/nonzero-anchor-error mean effects must be accounted for before assigning zero-mean statistical interpretation. Independent real landmark truth is unavailable; those means are not empirically closed.

`LtvCorrelatedLandmark.h` supplies pure algebra for R_L/N_L/visual cross and a stable LDLT solve:

`S=H P H'+R+H N+N' H'`,

`K=(P H'+N) S^{-1}`, `Pplus=P-(P H'+N) S^{-1}(H P+N')`.

It verifies joint state/noise compatibility and rejects singular duplicated-information innovation. Correction input is explicitly innovation minus a declared mean. It does not mutate State and does not use an independent Joseph formula to conceal N. Formula/pose-FD/solve tests are separate from real runtime qualification.

## Concrete STOP for production correlated ON / C4

No qualified production30-slot R_L/N_L/visual-cross interface exists. The finite actor has different gain/coupling; actual initial/history/noise law and main stochastic schedule remain unvalidated; actual error charts require mean mapping; landmark residual mean is unclosed. In controlled known-truth evidence, deterministic Observer mean error norm9.85903 remains despite covariance agreement0.496%. Treating covariance agreement as zero-mean consistent fusion would therefore be a false inference. Production correlated ON and C4 replacement are STOP/NOT_RUN pending those extensions. B's independently authorized engineering approximate ON has separate empirical qualification.
