# Task C validation (in progress; no production correlated qualification)

Reference baseline: 2417d036002fbfd542ae71d96d0fc734cd6d47e8. Production switches and confirm20 remain unchanged.

## Verified evidence

| Scope | Actual run | Evidence / result |
|---|---|---|
| All camera mean/P_R inputs, raw bearing normalization, spectral floor, conditional native reset | c_c1_c2_light_checks_attempt1_20261008T104157_6bd13168 | Camera FD max 1.31459e-10 < 1e-8; fixed spectral branch passed; floor boundary explicitly NONSMOOTH |
| Actual default native Observer repeated IMU/camera/seed/bias/raw-bearing directions | c_finite_c_unit_checks_attempt1_20261008T115555_b159a62e | q=1e-4, V_o=1e6: max FD 7.12404e-10; bounded q17/V_o=.02: 5.87584e-10 |
| Rejected synthetic excitation retained | c_c1_native_fd_attempt1_20261008T104352_fcc7cbfb, later independent reject fixture | First attempt failed before FD. Actual CovarianceFailure reset 7, required substeps 85036 > cap100. Not a smooth derivative PASS |
| Generic seed/main/visual same-source propagation | c_finite_c_unit_checks_attempt1_20261008T115555_b159a62e | Main/seed sign, pixel receipt, visual reuse and re-seed nonzero cross identities passed; known-law low-dimensional MC error 1.983% <5% |
| Known independent stationary world-point truth, actual geometric seed + actual native Observer, conditioned linear main | c_native_joint_truth_mc_attempt1_20261008T120009_8c85103e | 12000 complete samples, seed20261008, joint covariance error0.496% <5%; IMU endpoint reuse, seed history, prior-bias/main-pixel/anchor sources and QR passed |
| Repeated-anchor information counterexample | same known-truth MC | Full residual point-noise trace9.64098e-8; omitted anchor cross trace1.80023e-3 |
| Nonzero program injection and true-error chart mean mapping | c_program_injection_true_error_fd_attempt1_20261008T124940_76fa6df0 | Actual two-input program Ad/J_dx and M(e0) FD passed under1e-8; conditional posterior reset tested separately |
| Actual ROS native builds | c_c_native_build_attempt1_20261008T104403_3b58bfc7; c_finite_joint_native_build_attempt1_20261008T113534_2b39eafe | Both EXIT0. First ordinary build omitted phase0 flag and proves production compilation only. Second explicitly enabled phase0/C-related targets and installed10 verification binaries + raw runner |

Raw outputs are in the shared session's isolated `tasks/task_c/results/<run-id>` directories. Each has launcher manifests, exact source/binary/config identities, process ownership and resource locks. These paths are local evidence, not publicly hosted data.

## Statistical limits

The native Observer's known deterministic mean error in the controlled experiment is 9.85903 (dominated by zero gravity initialization against true gravity). Empirical mean minus nominal-model mean is7.50908e-5. Covariance calibration around this biased mean does not prove zero-mean NEES/NIS consistency or useful fusion. No independent real landmark truth exists; real landmark calibration is NOT_EVALUATED.

The controlled main model is conditional linear H/K with nominal zero injection. That experiment does not validate nonlinear real-main injection, stochastic main gain/FEJ/QR variation, actual static-initializer source error law, gate-conditioned distributions, or production30-slot coupling. Its PASS applies only to the recorded known-truth covariance experiment.

## Runtime finite implementation boundaries

One independently configured native Observer slot, fixed calibration/clock, supplied initial main error law, white raw endpoint/pixel hypothesis, nominal main H/K schedule. The old stored P companion, main program perturbation factor and actual true-error chart covariance are distinct. New code exports nominal main/clone chart layout for offline GT mapping; no GT is used in online fusion or gates.

Main source receipts previously consumed before registration are rejected for seed selection; this is a fresh-history subset. Expired sources are removed only within the documented native clone/IMU window. Duplicate/reused/pruned identities and covariance-law changes are audited or rejected. The model must not be copied into the production30-slot covariance by deleting other points.

C4 replacement of B's approximate model is NOT_RUN: no qualified production covariance interface exists. Engineering approximate ON in B is authorized independently and does not inherit statistical/model qualification from C.

## Pending evidence

Corrected6982 native unit suite, short finite-OFF/ON exact main state/full P/Observer/lifecycle regression and source audit; then matched full V2_02/V2_03 finite replay and offline true-error mapping. Current documentation does not mark these pending items complete.


### Composite residual truth diagnostic limitation (precise)

Absence of independent point truth prevents single-point Sigma_tt real calibration; it does not make every composite noise diagnostic impossible. For a static point and verified same physical identity, GT poses cancel its unknown world position:

`n_L=ellhat_t-A_GT ellhat_a-Rt_GT(pa_GT-pt_GT)=e_t-A_GT e_a`.

Thus main pose GT could support an offline composite mean/predicted-R_L diagnostic without point GT. Current finite logs persist covariance/cross blocks and main/clone nominal poses, but not the independent finite actor's current and saved-anchor mean point vectors. Production30-slot means in other logs cannot replace them. The actual missing evidence is those synchronized finite mean vectors/identity receipts, not point truth alone. Minimal next instrumentation is to retain pre-camera current and post-camera anchor ellhat/identity/time, then evaluate the composite with the frozen GT/gauge/bracket protocol. It remains a DEV diagnostic; the existing contract still does not grant real landmark calibration without independent truth. C4 STOP additionally rests on differing30-slot coupling, prior/H-K/source/mean validity and unclassified missing events.
