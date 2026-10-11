# Independent final scope review at confirmation launch

Reviewed after freeze attempt 151, SHA `f992d6d2e4220f0bb70dcd4e02b7ce2823d42cdcd6b6068a372116e651c1ec97`. Confirmation is still executing; this document does **not** certify final outcomes. Review used current source diff, frozen development evidence and the decision log only. No additional GT, confirmation outputs or heavy computation was accessed. No numerical source or frozen configuration was changed by this review.

## What changed and what did not

The statement “production files were unchanged” would be inaccurate for this task. Existing `VioManager.{h,cpp}`, `LtvAdapter.{h,cpp}`, `LtvOptions.h` and `ltv_observer.{h,cpp}` were changed within the authorized scope. They add a default-OFF read-only feature context, an explicit controlled admission/one-time seed interface, passive audit/cache integration and readiness metadata. ROS1/ROS2 build lists and the LTV test build list register the added modules and executable tests. New history, geometry, uncertainty, quality, lifecycle, pipeline and cache modules implement the opt-in path.

The current tracked diff contains no edits under `ov_core/src`, `ov_init/src`, the MSCKF state/update algorithm directories, `config/`, historical standalone-v2 reports, or historical handover/golden directories. In particular, native tracker, StateHelper, Propagator, UpdaterMSCKF, FeatureInitializer and existing G/V fusion equations are not modified. C0 Q/V/P0 and the 30 maximum / 15 minimum feature settings remain enforced for the new path. The final read-only SHA audit remains the authoritative integrity evidence; a clean diff alone is not a substitute for that audit.

## Controlled seed and persistent state contract

- A caller must opt in after starting a fresh observer and before its first camera event. Legacy callers follow the old lifecycle/update implementation through a null-control path.
- Controlled input validates epoch, physical time, calibration, retained IDs, visible observations and births before state mutation. A birth requires a current observation, finite body mean and an ID never admitted in that epoch; active and retired IDs cannot be reseeded.
- Lifecycle rebuilding copies every surviving old state and full covariance subblock. New points retain the C0 design block with zero new cross blocks. A seed only writes the newly admitted landmark mean, **after lifecycle and before that frame's correction**. It does not overwrite v/eta or substitute Sigma_seed into the Riccati P.
- Rejected/immature candidates remain outside the active observer and cannot contribute innovation. Empty/insufficient admitted observations still pass through physical IMU propagation and camera timing; low feature count does not call reset/pause. Genuine input-time faults retain the original reset behavior and restart the manager under a new epoch.
- Current clone means and their complete joint P are copied together before native visual updates. Native JPL error maps to right-positive body rotation plus additive world position; the native perturbation test and full-cross-block tests cover this convention. No GT or future lifetime enters the bridge.
- Default-OFF exact regression, actual short/full B/OLD/NEW main-estimate equality, actual zero auxiliary receipts/submissions and cache replay have independent evidence recorded in the freeze test groups. These demonstrate the tested paths/scopes; they do not prove closed-loop fusion suitability.

## Preserved negative and positive development evidence

The old noisy temporal calibration did not justify a calibrated probability interval: along-ray bias and finite-sample coverage were inadequate, particularly REGULAR. Risk remains explicitly a local risk score. The temporal candidate improved seed reliability but had approximately 2% readiness in REGULAR 1 s and harmed raw speed RMSE relative to OLD; those results were not hidden or relabelled.

Frozen hybrid synthetic development accepted 1515/1824 REGULAR 1 s opportunities with 96.83% reliable seeds and 830/3618 REGULAR 0.5 s opportunities with 93.25% reliability. The latter had zero readiness, preserving a clear short-lifetime limitation despite acceptable seed selection. FAST 1 s and 0.5 s seed reliability was approximately 97.68% and 98.17%. In these far-point synthetic examples the hybrid chose temporal fallback; stereo information alone cannot be credited for those gains.

All three frozen hybrid **development** real sequences met the declared output/coverage rule and exact main-estimate bypass checks. These are development results, not a substitute for the ongoing three-sequence confirmation and prescribed repeats. The selected hybrid is unchanged on entering confirmation; no sixth candidate, gain change, lowered minimum count or relaxed quality threshold was introduced.

Earlier compilation/linkage errors, the offline repeated-NPZ decompression failure, presentation retry and freeze metadata preflight failures remain recorded. Repairs changed tools/instrumentation, not the scientific acceptance criteria. Interrupted analysis reused saved simulation arrays rather than consuming a fictitious new successful observer run.

## Limits to retain in final reporting

- Real landmarks have no three-dimensional GT; source admission and risk diagnostics are not correctness labels.
- Temporal seeds use OpenVINS poses and joint covariance; unavailable pose-bearing correlations remain an approximation. Stereo adds right-camera information and fixed-calibration assumptions. Neither source establishes independence from OpenVINS.
- Sigma_seed is a local risk model, while P_L is a Riccati design matrix. No calibrated 95% claim follows from either.
- Mature support can remain insufficient under short identity lifetimes even when admitted seeds are reliable. All raw intervals, packet coverage, time coverage and longest gaps must remain visible.
- Preserve V1_01 attitude-reference limitations and UZH v3 finite reference support/fixed derivative-window limitations.
- Byte-equal native trajectories establish passive noninterference, not odometry improvement. G/V injection remains OFF; no automatic closed-loop or ATE optimization is authorized.

Final status must come from the completed frozen confirmation evidence, full input counts, repeat/cache audits, budget ledger and final integrity check. This review intentionally leaves that claim open.
