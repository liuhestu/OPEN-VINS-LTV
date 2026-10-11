# Cleanup verification and remaining limits

Base: `feature/ltv-passive-hardening-v2@f3e055ab2dc185d3a5910c9fa60c505dd912671d`.
Delivery branch: `feature/ltv-openvins-integration-cleanup`.
Status: **boundary cleanup implemented; exact equivalence on tested inputs;
real EuRoC regression remains unverified**.

## Executed

- Full original `ov_msckf_lib` built from independent baseline and cleanup
  checkouts with `ENABLE_ROS=OFF`, Release, GCC 13.3.0, Eigen 3.4.0,
  OpenCV 4.6.0, Boost 1.83.0 and Ceres 2.2.0 on Ubuntu 24.04.
  No compiler warnings or errors in the successful full-library builds.
- Independent target builds the actual 12 LTV production translation units,
  plus the shared printer. Both checkouts passed all 14 existing standalone
  feature/core/health contracts. With the native timing test, both passed 15/15.
  Assertions remain enabled in Release tests (`-UNDEBUG`).
- Cache writer/replayer selftests passed for both checkouts. The two generated
  caches compared exactly: 33 IMU records, 3 process events and 1 pause event;
  full state/P, slot mappings, births/retirements and non-wall-clock metadata
  matched. Negative selftests reject truncation, changed seed behavior and
  overwrite attempts.
- A native main-estimator simulation with **LTV entirely disabled** processed
  40 camera updates and 1639 IMU samples. Ordered SHA-1 state digests include
  full covariance, IMU mean/FEJ, clone/SLAM states, calibration states/FEJ and
  camera parameters. Baseline and final cleanup digest files are byte-identical,
  SHA-256 `dd388151369850961ba8e24ad8fb9bc279d646c0e8bed7c13592414dc22cece7`.
  This uses upstream simulator GT initialization; it is not a real-data result.
- Exact adapter comparison on one fixed, input-only, 10-second REGULAR
  WRONG_MATCH fixture, seed 42, lifetime 1 s, using the unchanged input generator
  and EuRoC calibration. Each checkout/mode consumed 201 camera events and
  2001 endpoint IMU samples. Legacy Passive (0), Active Consistency OFF (4),
  frozen C02 (5) compare full x/P, slots and all non-wall-clock event/health/
  lifecycle JSON. Only `compute_time_ms` is excluded; no numerical tolerance.
  In C02 the fixture writes 323 means and exercises 29 skips and 20 retirements.
  This is a numerical regression fixture, not a FAST study, threshold search,
  recovery verification or new statistical performance claim.
- Adapter comparisons cover both the independent build and wrappers linked
  against the full, separately compiled OpenVINS libraries. Each library runs
  in its own spawned process so the dynamic linker cannot reuse the other
  version through their identical native library SONAME. Machine-readable
  identities and results are in `adapter_parity.json` and
  `native_adapter_parity.json`.
- `source_audit.json` compares 294 protected files byte-for-byte to the base;
  no configuration, core equation, parameter, readiness algorithm, main VIO,
  R4 evidence or R4 script/ledger changed. The three modified implementation
  files differ only by the declared include/class-name substitutions.
- Changed C++ files pass repository clang-format checks and `git diff --check`.

## Not executed

- ROS 2 Humble / colcon build and ROS launch: no ROS installation in this cloud
  environment. Native full-library compilation does not establish ROS transport
  or package-install behavior.
- V2_02_medium / V2_03_difficult real replay and complete real C02 cache parity:
  dataset images/IMU and native binary caches are absent. R4 stores those at
  `/home/he/output/ltv_active_landmark_consistency_r4/`, which is not present
  here. The tested ETH dataset URL returned HTTP 502 through the available
  network. Repository plots and summary tables are not replay inputs and were
  not substituted for new results.

Therefore this branch must not be labeled as fully real-regression-validated,
`PASSIVE_READY_EUROC`, or authorization to enable G/V feedback. Its module
contracts are frozen; the real-input confirmation remains a separate pending
validation step, not further algorithm work.

## Environment and retried setup checks

System apt installation was unavailable; Ubuntu packages were downloaded and
extracted to a scratch dependency prefix. Configuration initially needed the
relocated BLAS/LAPACK and glog module paths. An early link missed Armadillo's
non-multiarch library path; an early native-test link ran before its baseline
library was complete. These environment/dependency-order failures were resolved
without source or parameter changes. One executable initially lacked execute
permission and passed after chmod. The first event-JSON comparison included the
known wall-clock field and failed; the comparator now excludes that single
field explicitly. An input-generation call with an unregistered 4-second
lifetime was rejected before execution; the final fixture uses the registered
1-second lifetime. The original R4 ledgers and acceptance artifacts were not
modified, and these attempts are not presented as new R4 confirmations.

## Reproduce the native checks

Build both full libraries with the same compiler, flags and dependencies:

```bash
cmake -S ov_msckf -B /tmp/ov-native -DENABLE_ROS=OFF -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/ov-native --target ov_msckf_lib -j2
cmake -S ov_msckf/tests/ltv/standalone -B /tmp/ltv-cleanup \
  -DCMAKE_BUILD_TYPE=Release -DLTV_NATIVE_LIBRARY=/tmp/ov-native/libov_msckf_lib.so
cmake --build /tmp/ltv-cleanup -j2
ctest --test-dir /tmp/ltv-cleanup --output-on-failure
```

Use the same setup for the frozen checkout via `LTV_SOURCE_ROOT`. For the native
main-state check, copy `config/rpng_sim` to a temporary directory and resolve
its `sim_traj_path` to the repository's
`ov_data/sim/tum_corridor1_512_16_okvis.txt`. Feed the identical temporary config
to both `test_ltv_native_default_off CONFIG OUTPUT` binaries and compare their
outputs byte-for-byte. Dataset/production YAML files should not be edited.
