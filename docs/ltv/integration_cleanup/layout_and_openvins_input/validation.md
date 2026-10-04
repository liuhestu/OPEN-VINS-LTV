# Physical layout and OpenVINS input cleanup verification

Parent: `df5b67d11d6f32c0b6d137db36e975d2cb19ca67`.
Behavior reference: original `f3e055ab2dc185d3a5910c9fa60c505dd912671d`.
Branch: `feature/ltv-openvins-integration-cleanup`.

## Changes

Actual source directories now implement the requested landmark_adapter, observer
and fusion boundaries; config and diagnostics are separate. UpdaterLTV moved out
of update/ into ltv/fusion/. LtvFeaturePipeline.cpp is now LtvLandmarkAdapter.cpp.
The ltv root contains only README.md and sources.cmake. ROS 1, ROS 2 and current
standalone checks share that production source list.

OpenVINS event conversion moved from VioManager to observer/LtvOpenVinsInput.
The bridge reads existing tracks, clone poses/full joint covariance, calibration
and time offset. Current left observations are read once for both bearings and
history; camera-0 extrinsics are converted once. Bias sampling remains before
propagation, all other inputs remain after propagation and before MSCKF updates.
No Feature/State pointer is retained by the returned value snapshot.

LTV short history remains necessary for epoch, protected IDs, bounded retention
and transaction rollback. No triangulation, seed, quality, identity, active
consistency, Riccati, admission, readiness or G/V residual algorithm changes.
The migration audit proves all 34 moved files retain their non-include contents
(the exact count is in source_audit.json). Production calibration/configuration,
ov_core, ov_init, State and trajectories are unchanged.

Repository callers and the old parity harness have include-path changes only.
Historical frozen experiment validators/manifests keep their old hashes and
must run from their original refs. The legacy FeaturePipeline alias remains
inside landmark_adapter; old flat external include paths require migration.

## Executed results

- Full native ov_msckf_lib, ENABLE_ROS=OFF, Release: built successfully, no
  compiler warnings/errors. Existing assertions-enabled tests: 15/15 passed.
- Original baseline vs final native-library adapter replay: modes 0, 4 and
  Passive C02 mode 5, each 201 camera frames / 2001 IMU samples. Entire x/P,
  slots and non-wall-clock event JSON are byte-identical. C02 retained
  323 mean writes, 29 consistency skips and 20 retirements. Libraries ran
  in separate spawned processes to prevent SONAME reuse.
- Native OpenVINS Simulator, Passive C02: 80 camera frames / 3239 IMU samples.
  76 process calls and 4 pauses; 76 available events; observer and mature
  landmark pools reached 30; 12 ready events and 1 soft-grace event.
  Main state/covariance/FEJ digests are byte-identical to baseline.
  Lossless input caches are byte-identical under the numerical comparator:
  all observations, calibration, clones and their joint covariance, timestamps,
  biases, observer full x/P, lifecycle and ID/slot mappings match. This directly
  exercises the changed VioManager-to-OpenVINS bridge.
- Native default-off simulation: 40 camera frames / 1639 IMU samples; full
  estimator state/covariance/FEJ digests remain byte-identical to baseline.
- Source/include migration audit and git diff --check passed.

Simulator tests use the upstream GT initialization and deterministic rpng_sim
fixture. The Passive fixture disables online calibration, as required by existing
LTV validation, and applies the frozen C02 settings identically to both libraries.
Only test configurations change; production YAML files do not.

## Reproduction

Build the full library with the existing native CMake entrypoint:
`cmake -S ov_msckf -B build-native -DENABLE_ROS=OFF -DCMAKE_BUILD_TYPE=Release`.
Configure `ov_msckf/tests/ltv/standalone` with
`-DLTV_NATIVE_LIBRARY=/absolute/build-native/libov_msckf_lib.so`, build, then
run CTest. Configure the same standalone project with LTV_SOURCE_ROOT pointing
to the independent frozen baseline to compare its full library.

Use the input hashes and frozen C02 test YAML overrides in
validation_manifest.json. Run both native harnesses with:
`test_ltv_native_default_off <passive-yaml> <digest-file> --passive-c02`.
The harness writes <digest-file>.ltvcache and enforces disabled G/V.
Compare digest files with cmp and caches with compare_cache_numerical.

Run scripts/ltv_integration_cleanup/compare_adapter.py with --before/--after
native synthetic wrappers, the unchanged input directory and --output.
Its original fixture identity remains the same; native_adapter_parity.json
records library hashes and exact replay results.

## Limits

No ROS installation or real V2_02/V2_03 dataset/cache is present here.
ROS/colcon and real EuRoC Passive C02 regressions are not executed.
These results establish equality on the tested inputs, not real-data acceptance.
R4 NOT_ACHIEVED and its existing unmet contracts are unchanged. G/V feedback
remains disabled; moving Fusion does not enable it.
