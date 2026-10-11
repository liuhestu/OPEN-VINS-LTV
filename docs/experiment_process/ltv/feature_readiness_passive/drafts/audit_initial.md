# Independent audit draft: initial interfaces and evidence gaps

Role C, static inspection only. No build, replay, geometric Monte Carlo, broad data hashing, or confirmation output evaluation was performed for this draft. This is not a PASS certificate.

## Available inputs and commands

The old sensor-only mirror `/home/he/output/openvins_ltv_value_study_20261003/inputs/` contains `V1_01_easy`, `V2_02_medium`, `V2_03_difficult`, `indoor_forward_3`, and `indoor_forward_6`. Each named mirror root exists. Its old manifest is `../inputs.json`. Reuse sensor files read-only, not the old result/threshold classifications.

`V1_03_difficult` is absent from that manifest, but its original `/home/he/datasets/euroc/ASL/V1_03_difficult/mav0/` has both camera CSVs, IMU CSV, and official state GT. Only existence and file sizes were checked; image completeness remains to be validated by the budgeted runner/preflight.

EuRoC GT: `/home/he/datasets/euroc/ASL/<sequence>/mav0/state_groundtruth_estimate0/data.csv`. UZH GT: `/home/he/datasets/uzhfpv/archives/<sequence>_snapdragon_with_gt/groundtruth.txt`. The historical release verification names UZH v3 and hashes; it is provenance, not new validation output.

Existing configuration roots: `config/ltv_value/ltv_euroc` (offset 0) and `config/ltv_value/uzhfpv_indoor` (offset −0.016684572091862235 s). Copy into the new task's output; do not run old prepare.py because it rewrites tracked configuration.

Native CLI: `run_ltv_value config.yaml sensor_ASL_root output_dir [max_camera_packets]`. It does not load GT and synchronizes exact stereo timestamps, records unmatched images, consumes IMU with one cached right endpoint, then consumes remaining IMU after full replay. Confirm complete count fields, not merely exit zero. New task uses its own compiled binary and identity.

Do not use legacy `scripts/ltv_value/run.py` as dispatcher: its limits are 80 full/20 short, short means up to 30 seconds, and it permits injection modes and legacy weights. New short budget requires at most 10 seconds of actual input span and attempts must be reserved before startup.

## Read-only bridge and covariance contract

Current VioManager freezes ba/bg before propagation, propagates/augments clone, then obtains cam0 observations before feature selection and cleanup. This is the intended causal sampling location. Camera extrinsic conversion is `R_BC=R_ItoC.transpose()`, `p_BC=-R_BC*p_IinC`; estimator stores JPL G-to-I while explicit geometry should receive `R_WB=state->_imu->Rot().transpose()`.

Deep-copy observations and all context before native feature mutation; no Feature pointers in delayed history/cache. A temporal seed context must refresh all retained clone means and one matching joint P at this event rather than mixing stored old means with current P. `StateHelper::get_marginal_covariance(state, ordered_clone_types)` preserves cross blocks; active state `_variables` is private. No StateHelper algorithm change is needed to read joint P.

`LtvAdapter::process` already propagates with empty bearings. `pause` resets observer and ID map. A rejected/immature candidate set must take the former path, not pause. Existing timing tests cover interpolated endpoints and persistence, but do not exercise the new seed path. Replay cache must reproduce exact raw IMU delivery order, calibration/bias version, camera event order, and reset/pause events, not just 20 Hz bearings.

## Existing audit does not prove the new hard gate

`ValueDiagnostics` serializes native IMU mean/FEJ and optionally prior P plus counterfactual update blocks. It omits full state/FEJ and visual IDs, and expensive counterfactual shadow builds are unnecessary to establish passive equality.

The existing runner digest covers timestamp, IMU mean/FEJ, full P, clone mean/FEJ, sorted tracker IDs/delete flags, and good_features_MSCKF positions. Strengthen it with all public state families (calibration, SLAM values/FEJ and type IDs), camera models, full tracker observation history or an independent observation digest, and visual candidate/accepted ID order captured before cleanup. Current post-cleanup IDs cannot prove identical visual sets.

State public fields expose all current families without private-memory access: IMU, ordered clones, SLAM features, dt, IMU-to-camera poses, camera intrinsics, gyro/accel intrinsic vectors/rotations. Canonically sort unordered maps; include type id/size to verify the full P layout. Compare per-camera event digests and byte-equal trajectory, not only a terminal digest or ATE.

Actual zero injection requires per-frame receipt diagnostics/counters at the native call site. No receipt may be created in B/P_OLD/P_NEW; accepted_G, accepted_V, and extra EKF submissions must remain zero. A YAML OFF flag is necessary but insufficient. Audit startup options and all events including no-visual-update and low-feature frames.

## Independent acceptance coverage

1. Disabled new path: exact full x/P and slot/event parity against old core on identical deterministic event sequence; no modification of golden fixtures.
2. One-time seed: only just-admitted point mean before correction; duplicate, stale time, stale epoch rejected; old x and all old P subblocks exact; no seed accepted without corresponding current bearing.
3. Candidate isolation/lifecycle: rejected seeds never participate in correction, missing bearings never copied, finite candidate TTL and bounded histories, correct reappearance identity handling, nontrivial cross-block preservation through compaction, low count preserves propagation and epoch.
4. Geometry: nonidentity rotations/lever arms, positive ray versus optical Z, execution-time transform, shared rigid gauge invariance, temporal correlations and singular joint P support. Check ray uncertainty definition does not claim complete distance uncertainty when it freezes ray direction.
5. Cache equivalence: same C++ module; compare input event content first, then decisions/reason/seed source and coordinates, exact x/P, slots, readiness and reset events. Equality in summary RMSE does not establish equivalence. Save comparison tolerance before observing differences; same binary/exact serialized doubles should be exact.
6. Synthetic reliability: denominators include every legal track with an initialization opportunity before final risk gate; birth-track denominator separately; scene-specific minimum 100 accepted, ≥90% reliable, ≥20% opportunity acceptance; time/track block dependence not independent per point.
7. Real evaluation: fixed all-initialized-camera denominator including absent/not-ready output; per-branch coverage and valid GT seconds; NEW/OLD/OpenVINS compared on identical ready support, raw common finite support plus missing intervals, no fitted alignment or offset. V1_03 must be classified EuRoC explicitly (old `common.EUROC` omits it).
8. GT helper returns unit gravity, not eta: multiply by configured gravity (EuRoC 9.81, UZH 9.8065). UZH velocity uses frozen 0.1 s cubic local derivative and original gap mask; do not retune the window. No landmark accuracy labels for real input.
9. Resource/resume: full cache replay counts as real full attempt; interrupted attempts retained; partial outputs never reused; a paused/incomplete run is not PASS; seed/IMU replay persistence tested or restart-from-beginning explicitly counted as a new attempt.

Independent inspection found no local dataset-path blocker. Engineering/cache/selector acceptance remains unproven until implemented and exercised.
