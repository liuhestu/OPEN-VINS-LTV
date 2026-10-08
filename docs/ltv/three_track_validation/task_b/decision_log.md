# B decision log

## Before causal gate analysis

Observation: historical mature LTV prediction outperforms weak anchor propagation, but strong past-bearing geometry has confidence intervals spanning zero. V2_03 normal is a counterexample to uniform gain, and startup has greater error.

Judgment: the uncertainty is predictive value and tail control, not exclusively missing full cross covariance. Blindly implementing approximate ON or scanning its R cannot resolve this uncertainty.

Change: test only three predeclared causal gates using reusable verified full development-sequence evidence. Keep ungated mature/all-candidate denominators, common gated support, every model's all-available gated support, and startup/normal/recovery statistics. Do not inspect or tune on the holdout sequences.

Next verification: the main agent freezes the gate admission contract before this analysis runs. Only a candidate passing both development sequences advances to native residual/Jacobian and matched ON experiments. Failure is preserved as STOP with no fallback to the weak control.

## Cross-track model finding before gate results

C's source audit identifies native finite JPL injection without an explicit chart-reset covariance transform. This is a model convention gap, not build pollution and not evidence of observed divergence. Keep the existing baseline algorithm, disclose native-P/physical-Sigma distinction in any B updater, and check actual native injection/FEJ Jacobians. Do not cite PSD or stored-P equality as statistical consistency.

## B gate v1, batch b_gate_v1_dev_historical_20261008T104150_b4de5501

Observation: all three candidates fail admission. B2 passes gain confidence and every frozen tail check on both development sequences, but mature supply is only 13.51%/8.895%, below the frozen 20% requirement. B0 has adequate supply but V2_03 gain is 0.335% with interval crossing zero and recovery-tail failure.

Judgment: these results reject the first three candidates. B2's failure is not a full-correlation blocker and not build pollution: raw/cache/source identities and immutable-source checks passed. The coverage loss might be a fixed 0.5s window-design issue, so refusing all further research would be premature. No ON, noise-weight scan, acceptance relaxation or holdout inspection is justified.

Next verification: a descriptive (non-admission) anchor-age/source-supply audit around 0.5 and 0.55 seconds distinguishes numerical endpoint effects, actual clone-window occupancy, and model failure. Any replacement candidate must be separately frozen before its results, keep the original admission criteria, and rerun affected checks. The result is not a qualification of a post-hoc subgroup.

## Source-supply diagnostic, batch b_supply_source_audit_v1_20261008T105015_b7c4f69e

Observation: 0.55±1µs anchors represent 78.4%/68.2% of ready-mature supply, and B1/B2 discarded all of them. With the unchanged relative agreement score, newly included .55s points have strong-geometry p95/p99 ratios 0.789/0.478 (V2_02) and 0.774/1.040 (V2_03), with fewer bad angles. V2_03 also has 0.6–1.1s live-anchor spans: nominal camera period does not imply uninterrupted clone supply. Those long-age agreement tails are worse and must remain visible.

Judgment: the fixed .5s gate was inconsistent with ordinary pre-marginalization twelve-clone supply, rather than evidence that all predictive supply is unusable. The diagnostic supports testing B3, without qualifying B3. V2_03 .55s ungated RMS is slightly worse than geometry, so broadening age may dilute gain. Both numerical boundary effects and physical-age effects are explicit.

Change: preregister exactly one lifecycle-derived B3, retaining prior readiness, maturity, relative agreement 0.02, and fixed nominal .55s age cap (plus 1µs machine-clock tolerance). Do not replace that cap with arbitrary live-window length: .6–1.1s counterexamples reject such a shortcut. Preserve all B0/B1/B2 failures and every original admission criterion/denominator.

Next verification: main-agent publication freezes B3 before execution. Re-evaluate both complete development sequences against original strong-geometry paired support and all required phase-tail/supply tests. No R scan, ON, holdout tuning or acceptance relaxation until predictive admission. Raw source/cache/prediction/feature files were SHA-verified before and after this descriptive batch; external disk cleanup did not change their identities.

## B3 admission and single B4 mechanism registration

B3's lifecycle fix raises mature supply to 65.07%/36.77%, and every tail/bad-angle condition passes. V2_02 improves 56.20% with positive confidence. V2_03 improves 4.0328%, below the unchanged 5% threshold; its interval [0.00885%,12.15%] is retained. V2_03 normal gain is only 1.126% with interval spanning zero, while recovery gain is 9.331%. The coverage problem was an implementation-design issue; residual insufficiency versus strong geometry is now a model-value issue. No input/build pollution was found.

Main authorizes exactly one additional mechanism test: B4 asks whether the immediately previous successful camera0 consumption's comparative predictive quality causally identifies useful current points. It adds no numeric score threshold, R tuning, age scan or acceptance change. The ledger uses camera0 actual `corrected_ids`, strict preceding complete frame and full identity, commits staged scores only for the following frame, and shares one point selection across both camera evaluations. Offline angle columns alone are not an online implementation.

Synthetic prefix-causality, row-order, consumption, entered-generation and missing-previous-packet tests pass under `b_lagged_causality_synthetic_20261008T111042_a543b21c`. Current/future target mutation leaves current gates unchanged; an unconsumed or stale score is rejected. These checks validate ledger semantics only, not predictive admission or actual ON.

## Explicit engineering-admission amendment

B4 retains its predictive FAIL: V2_03 gain 3.791% is below the original 5% threshold. It improves no aggregate gain over B3, lowers supply 36.77%→21.44%, and requires online strong-geometry fitting. All risk/coverage tests pass on both candidates. The main agent relayed explicit user steering: the retained 5% predictive-benefit failure is no longer the sole engineering ON precondition. This expands the authorized engineering experiment without altering any predictive result or engineering acceptance threshold. The earlier STOP decision is superseded for the engineering experiment only.

Choose B3 for engineering: its risk qualification and greater supply passed, and it avoids B4's unsupported online geometry cost. Implement a separately default-OFF native B3 updater, with the registered engineering R approximation and bounded prior-information/point supply. Preserve existing visual updates. Validate native JPL/FEJ finite differences, default-OFF and same-diagnostic OFF exact behavior, then short real actual ON and complete matched development/holdout runs. No further shadow optimization seeks a 5% PASS.

## First actual ON audit exposes a receipt-log ordering bug

The true V2_02 start-to40s ON command exited 0, but the independent audit failed and is preserved. Selected points are present and actual main state/P and trajectory means differ in 603 frames, starting at 1413393895.8557606. Frame/point logs nevertheless show zero consumed rows and zero contribution: the native manager called finish_frame before copying the live receipt's post-update diagnostics. This is an implementation/logging phase defect, not a demonstrated model failure or input/build pollution.

Fix only the logger's location to after the existing receipt copy. Keep R, H, gates, point/information budgets, native visual update and injection unchanged. A new source/artifact identity must build and repeat the affected native OFF and real ON safety checks. The old bad logs and FAILED_RETAINED audit are not repaired retrospectively or counted as action qualification. Exit0 and a state/P difference alone do not prove each declared row's actual consumption/component.

## Whole-clone visual layout counterexample

Source-level joint-capture audit found a second implementation problem before complete ON experiments: native MSCKF uses whole 6D clone PoseJPL types, while the first LM block used separate anchor q/p subtypes. The generic merger correctly rejects overlapping distinct Type pointers. That can cause LM to be discarded on visual-update frames even while auxiliary-only frames truly change the main filter. The initial state/P difference does not establish successful visual-plus-LM joint integration.

Use whole current IMU pose6D and whole anchor clone pose6D in the LM order, keeping the identical 3x12 physical H, residual, noise, selection and budgets. Add a real native test preserving the old partial-layout overlap counterexample, proving corrected whole-clone merge and one consumed joint native EKF call with nonzero LM component. The live immutable v3 logging-fix build is allowed to finish without artifact mutation; v4 uses a new complete source/artifact identity. Both defects are implementation issues, not evidence to tune R or weaken engineering thresholds.

Before v4 freezes, the main agent approves the hybrid metadata order: current q3/p3 subtypes plus whole anchor PoseJPL6. Its physical H columns and covariance entries are unchanged, while it shares the current q object with G/V and the whole anchor object with native visual rows. This avoids a second latent integration-only overlap without changing A's model. Native fixture tests check visual+LM and visual+G metadata+LM joint H/P/S, actual receipt consumption and equivalence to one directly stacked native EKF call. Those interface tests do not grant combined-mode engineering qualification. Current experiments keep G/V OFF.

## V4 native joint action proof and safe prefix

Fresh v4 source5cdedc0 and twelve native checks pass. Corrected hybrid metadata passes the old overlap counterexample, true visual+LM native merge/application and G/V-metadata interface one-EKF equivalence. This is interface evidence, not combined-mode qualification. The original OFF and corrected shadow OFF are prefix-exact; diagnostic ON and scalar ON are likewise exact, proving extra matrix capture read-only in the measured prefix.

Actual V2_02 start-to40s: 401 applied/nonzero frames, 802 points, one EKF per actual receipt; first full-state/P and current-IMU mean difference maps exactly to the first LM application. Independent reconstruction from real recorded priorP/H/S/res/layout verifies native K_L*r_L total and bias components and gives physical-unit current rotation/position/velocity/bias blocks. Maximum theta=1.95e-7rad, p=2.23e-6m, v=2.60e-6m/s, bg=1.24e-8rad/s, ba=1.45e-6m/s². The full norm mixes error-state units. Action is real and small; gain is not established by its presence.

V2_03 start-to40s similarly gives 190 actual nonzero frames/379 points, with first state/P/mean difference aligned to first applied receipt. No new numeric or identity failure occurs; point/information budgets hold. These are two complete original input prefixes, not full-sequence engineering or statistical qualification. Preserve prior wrong-phase logs/audit failure. No H/R/gate/parameter scan is justified by the small action.

Next: complete matched development OFF/ON under the shared strict metric/window contract, keeping sparse and empty support visible. A's reproducible three-sample tail failure demonstrates why a favourable full ATE cannot erase local regression; B uses exactly those rules. Independent holdouts and production-style exclusive timing follow only an eligible development candidate. Correlation model remains explicitly approximate; C's incomplete runtime blocks are not consumed.
