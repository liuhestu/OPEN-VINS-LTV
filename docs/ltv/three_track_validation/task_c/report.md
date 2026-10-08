# Task C result: finite source implementation, production correlated STOP

C implements and tests an independent one-slot native Observer joint source model. Both complete DEV streams finish with finite diagnostic enabled and leave the original main state/full P, production Observer x/P_R and lifecycle/readiness/cache exactly unchanged. This is a read-only implementation result, not ON fusion benefit or statistical consistency. Production correlated fusion/C4 is STOP; B's engineering approximate candidate has separate qualification.

Runtime source6982e0473b6963ab95dfb169b585c09f464bfa8e was built fresh with phase0/C-related targets, then native units and prefix/full checks ran against its frozen install. Analysis scripts/kernel/probes have separately recorded OIDs. Changed C++ was clang-format -style=file checked at finalization, with no formatting delta; no frozen source or running artifact was edited/rebuilt. Default confirm20, G/V OFF and max_slam0 remain unchanged.

## Actual delivered scope

Full mean/P_R/gain/bearing/spectral sensitivities in native Observer; source-identified shared raw IMU endpoints/interpolation; actual selected main mean integrator FD in a separate local State; pre-propagation prior-bias receipt; actual seed/history/clone selectors and pixel-to-bearing maps; actual visual nullspace/compression source columns; native two-input program injection; clone/marginalization/history factors; pre-camera current/post-camera saved anchors; all-retained-state/source QR compression. Stored P, program perturbation covariance and actual true-error chart covariance are distinct. Independent offline GT mapping uses the frozen shared OFF-support SE(3) gauge and ≤10ms GT interpolation brackets.

The pure correlated kernel implements R_L/N_L/visual cross and stable S/K/Pplus with explicit same-chart/mean inputs, input finite/symmetry/joint PSD checks, posterior raw minimum eigenvalue and singular duplicate rejection. Native pose Jacobian FD max3.79246e-10. It has no State mutation or production switch; no qualified online30-slot covariance/mean interface is delivered.

## Complete raw replay evidence

| Sequence | Matched camera / IMU / output rows | Complete original exact OFF/shadow | Matrix events / actual indexed point IDs | Max factor cols | Joint min eig | GT mapped / NT |
|---|---:|---|---:|---:|---:|---:|
| V2_02_medium | 2348 / 23490 / 2266 | PASS, tolerance0 | 2171 / 82 | 445 | -1.83e-16 | 2158 / 13 |
| V2_03_difficult | 1921 / 23370 / 1806 | PASS, tolerance0 | 1685 / 99 | 505 | -1.09e-16 | 1678 / 7 |

V2_03's one unmatched left and415 unmatched right images are retained (416 records); both modes use the identical full matched roster. No successful overlap subset replaces the full streams. Index matrices are finite; symmetry/PSD checks pass only as numerical implementation checks. They do not prove the source/noise law or empirical real calibration.

Raw runs and analyses, in the shared session `tasks/task_c/results/`:

- V2_02 OFF `c_V2_02_medium_finite_OFF_full_attempt1_20261008T133634_af0b4936`; shadow `c_V2_02_medium_finite_ON_full_attempt1_20261008T133946_0d8a58ad`; full exact/audit/map `c_V2_02_full_exact_source_true_error_map_attempt1_20261008T134716_214657c8`.
- V2_03 OFF `c_V2_03_difficult_finite_OFF_full_attempt1_20261008T134020_02ba2df8`; shadow `c_V2_03_difficult_finite_ON_full_attempt1_20261008T134809_9eb2b516`; full exact/audit/map `c_V2_03_full_exact_source_true_error_map_attempt1_20261008T135211_326d5290`.
- The10s prefix native/cache equality and finite matrix/source receipt repeat are independently zero-tolerance/byte-exact. The first attempt to use the FULL comparator on a prefix remains FAILED_RETAINED with Complete real input required; no full criterion was relaxed. Prefix repeat is not independent sampling or full covariance-repeat qualification.

## Source validity gaps are retained

The finite logger has a cumulative unclassified missing-event counter:35 on V2_02 and22 on V2_03. Only9/2171 and81/1685 indexed events respectively have counter0. This does not identify the later events as invalid individually, nor show the counted events are benign. Sites include finite Observer reset/rejection, missing/mismatched metadata, invalid/late seed input and unmatched visual source dimensions. Normal retirement alone does not increment it. Current logs do not distinguish the sites; guessing a cause would be unsupported. Source completeness therefore remains INCONCLUSIVE despite numerical PSD and exact read-only parity.

The physical assumptions are supplied initial error law (not static-initializer/source calibration), fixed calibration/clock, white endpoint/pixel hypotheses, nominal conditioned main H/K/FEJ/QR schedule, and fresh registered seed history. Already-consumed seed pixels are rejected. Noise-law/pruning reuse is checked. Actual30-slot gain/cross-point coupling and full old-history/initializer correlations remain unclosed.

## Controlled covariance and mean evidence

Original and one explicitly authorized same-seed/same-law reproduction use12000 complete samples, seed20261008; no parameters or samples were changed. The aggregate mixed-unit covariance Frobenius error is0.0049601733598021404 against the frozen0.05 limit. Persisted predicted/empirical33×33 covariance and33-component means now make it auditable. Physical-block relative errors are descriptive, not new retrospective gates: main position2.298%, remaining diagonal blocks approximately0.496%–1.868%. That does not confer every-block statistical consistency or real calibration.

The deterministic native model's body error means and empirical reproduced mean norms are:

| Block | Nominal error vector | Nominal norm | Empirical norm |
|---|---|---:|---:|
| Landmark (m) | [-.00103366, .00051683, -.04408040] | .04409554m | .04404252m |
| Velocity (m/s) | [.000046664, -.000023332, .980997083] | .980997085m/s | .980997070m/s |
| Gravity (m/s²) | [1.2453e-6, -6.2263e-7, 9.809999922] | 9.809999922m/s² | 9.809999922m/s² |

9.859026 is a9D norm mixing metres, m/s and m/s², not9.859m. The original7.509e-5 empirical/model difference is likewise a33D mixed-unit aggregate; original component data were not saved, and the reproduction now supplies them without changing the old result. Known deterministic bias is substantial, so covariance agreement cannot erase the mean error or establish zero-mean NEES/NIS consistency.

Complete anchor cross gives point residual-noise trace9.64098e-8; setting anchor cross to zero gives1.80023e-3. This is a concrete repeated-information counterexample, not evidence of real ON gain.

## Layer decisions and minimal next work

- C0/C1: formula/source mapping and fixed-branch native sensitivities validated within documented tests; non-smooth floor/substep/reject branches remain separate.
- C2: reduced known-truth covariance benchmark reproduced with actual geometry/native Observer/shared sources; main model is conditioned linear/nominal-zero injection. Mean consistency and global/nonlinear statistical qualification are not proved.
- C3: actual complete finite source replay/read-only exact parity and offline chart mapping delivered. Full physical source validity/calibration is INCONCLUSIVE/NOT_EVALUATED; numerical PSD cannot close the unclassified counter or assumptions.
- C4: STOP/NOT_RUN. No production correlated ON. Limits include independent1-slot coupling, unvalidated initial/history/noise/identity law, conditioned main schedule, unclosed mean/chart requirements and unavailable independent real landmark truth.

Minimal extensions are reason-specific source-event journaling (without deleting failures), actual initializer/previously-consumed-history error/source law, calibrated input/identity/selection model, native main stochastic schedule/mean mapping, and a scalable full production Observer source graph. Then independent truth-controlled scenarios and matched approximate/correlated comparisons can test mean and information gain. Increasing a Riccati P or multiplying R does not close those dependencies.

See `model_interface_v2.json`, `joint_error_model.md`, `joint_model_validation.md`, evidence summaries and complete run manifest. Large raw evidence paths are local resources, not public hosted datasets.
