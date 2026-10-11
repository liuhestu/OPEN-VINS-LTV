# Local ROS build and real Passive C02 regression

Result: **PASS_REAL_ROS_PASSIVE_C02_CLEANUP_EQUIVALENCE**, 2026-10-05.
Current source `f1df9135a89056ebd38b981deffcd930ca7c42c4` includes the requested
cleanup commit; baseline `f3e055ab2dc185d3a5910c9fa60c505dd912671d` remains in a
clean detached worktree. Initial current worktree was clean. This follow-up adds
only comparison tools and evidence; no production repair was necessary.

## Environment and build

External output root (`OUT` below):
`/home/he/output/ltv_integration_cleanup_real_20261005`.
The independent baseline is `$OUT/baseline_f3e055ab`. Each revision has its own
`build`, `install` and `log` under `$OUT/baseline` or `$OUT/current`.

Actual build command for each revision, after clearing inherited workspace
prefix/library/Python paths and sourcing `/opt/ros/humble/setup.bash`:

```bash
colcon --log-base "$WS/log" build --base-paths "$SOURCE" \
  --packages-up-to ov_msckf --executor sequential \
  --build-base "$WS/build" --install-base "$WS/install" \
  --cmake-args -DENABLE_ROS=ON -DBUILD_LTV_PHASE0_TESTS=ON \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
  -DPYTHON_EXECUTABLE=/usr/bin/python3
```

Both builds exit 0 and compile ov_core, ov_init, ov_msckf and ROS2 runtime tools.
GCC 11.4.0, ROS Humble, toolchain/dependency cache entries and normalized actual
compiler flags are equal. Dependency-selected `-std=c++17` is present in both;
the project standard setting was left untouched. There are 13 identical existing
warnings per build (Eigen, Ceres deprecation, upstream unused variables and ROS
obsolete headers), with no errors. The current 15-entry LTV production list is
compiled exactly once, including LtvOpenVinsInput and the moved UpdaterLTV.
All 34 moved files retain their non-include contents against the cleanup parent.
ov_core, ov_init and production configuration are unchanged against the baseline.

Each revision ran **24 installed related tests, all passed**, with assertions
enabled on LTV test targets. The exact test names, commands and exit codes are in
[baseline_tests.json](real_20261005/baseline_tests.json) and
[current_tests.json](real_20261005/current_tests.json). They cover observer
lifecycle, timing, layout/cross covariance, FEJ, G/V Jacobians/covariance,
controlled features, history/identity, seed pipeline, readiness, hardening and
Active Consistency. Tests do not enable production G/V feedback.

The initial build-identity checker accidentally included the upstream
`test_sim_meas` in its LTV assertion check. Its selector was corrected to LTV test
targets; flags and numerical criteria did not change. The failed checker command
and fix are saved in [checker_scope_fix.json](real_20261005/checker_scope_fix.json).

## Frozen real inputs and execution

Real images and IMU CSVs are under
`/home/he/datasets/euroc/ASL/{V2_02_medium,V2_03_difficult}/mav0`.
The existing R4 ON C02 configuration was copied byte-for-byte to external input
directories. Both revisions use the same files. Estimator YAML SHA256:
`c2250d9d5da69235cdbfbf46ae8a0f15384eecfb4583664c25d5516662ba8b34`.
[real_inputs.json](real_20261005/real_inputs.json) records all calibration/CSV
hashes, the original frozen identity paths/hashes and image aggregate hashes.

All 4696 V2_02 and 4258 V2_03 referenced image files were hashed before and after
replay; aggregates remained unchanged. Full per-image identities remain external
in `$OUT/images_identity.json`. V2_03 contains 1922 left and 2336 right raw images;
the existing exact-timestamp pairing yields 1921 packets and 416 unmatched images
(1 left, 415 right). Baseline/current unmatched records match exactly. No new
filtering or pairing policy was introduced.

The unchanged runner uses C0 Q/V/P0, original initialization, C02 history/seed/
consistency/readiness settings and disabled G/V. Its existing deterministic
overrides set OpenCV to one thread and disable multithreaded subscriptions and
publication. Audit/cache capture is enabled identically. BLAS/OMP/MKL/NUMEXPR are
single-threaded. At most two compute processes ran concurrently. The command is:

```bash
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
cd "$RUN_OUTPUT"
"$WS/install/ov_msckf/lib/ov_msckf/run_ltv_feature_passive" \
  "$FROZEN_CONFIG" "$MAV0" "$RUN_OUTPUT" P_NEW
```

Four baseline runs (two per sequence) finished first, followed by two current
runs. All six exit 0, use no short limit and consume the complete input IMU and
camera packet lists. Separate processes load their own installed libraries;
`ldd` paths and SHA256 identities verify no cross-workspace SONAME reuse.
[baseline_run_jobs.json](real_20261005/baseline_run_jobs.json) and
[current_run_jobs.json](real_20261005/current_run_jobs.json) preserve each exact
command, environment controls, source/binary/input identity and exit code.

## Exact comparisons and exercised events

Both baseline repeats are exact; both current comparisons are exact. The checker
uses zero tolerance and reports the first differing row/field or binary event
and payload byte. Its tampering self-test rejects changed state/covariance,
signed zero, nested timing fields, Active Consistency suffixes, truncated caches
and identical-but-incomplete evidence. It accepts only the specified top-level
wall-clock field difference.

| Evidence | Scope / comparison |
|---|---|
| audit.csv | Every camera timestamp, main state families and FEJ, full covariance/layout, clones, SLAM, calibration, camera models, tracker/visual digests, auxiliary submission counts; exact literal rows |
| trajectory.csv | Every emitted main trajectory sample; exact literal rows |
| cache.bin | Entire serialized inputs and observer full x/P, all cross blocks, IDs/core IDs/slots, controlled births/retirements, seed diagnostics and Active Consistency suffix; exact bytes |
| features.jsonl | Entire event tree, including readiness, lifecycle, corrections and consistency; exact types/float bits and canonical JSON after excluding only top-level compute_time_ms |
| replay.json / effective_options.json | Complete input consumption, exact parsed options, zero G/V submissions and flags |
| unmatched_camera.csv | Full unmatched image records; exact literal rows |

Main state/covariance equality uses the existing runner's SHA1 digest of raw
values, FEJ and the complete matrix with dimensions/layout; raw main matrices are
not exported individually. LTV equality compares raw complete serialized matrices.
Checks cover camera-event boundaries and the complete IMU input stream, rather
than separately exporting internal integration substeps. Physical timestamps and
readiness ages are always retained. Process elapsed times in command records and
build/test logs are metadata, not numerical outputs.

| Sequence | Packets / IMU | Process / pause | Mean writes | Consistency skips / retires | ready G / V |
|---|---|---|---:|---|---|
| V2_02 | 2348 / 23490 | 2266 / 82 | 2574 | 147 / 69 | 1606 / 1571 |
| V2_03 | 1921 / 23370 | 1806 / 115 | 2784 | 260 / 160 | 706 / 637 |

Counts match across all three runs of each sequence. The observer pool reaches
30 points in both. All retirement event records (including candidate lifecycle
records) total 6882 and 6828 respectively. Detailed coverage and exact comparison
file/cache hashes are in [event_coverage.json](real_20261005/event_coverage.json)
and the four repeat/regression JSON files.

## Evidence, reproduction and limits

[manifest.json](real_20261005/manifest.json) binds archived evidence and external
logs to SHA256 identities. Full caches, arrays, logs, builds and datasets stay
outside Git. The archived [commands directory](real_20261005/commands) preserves
the executed orchestration scripts and build commands. Those scripts use their
external output root as their directory; inspect/copy them into a fresh external
root with updated source/input identities for a new study. They intentionally
refuse to reuse existing run directories.

For saved-result verification:

```bash
python3 scripts/ltv_integration_cleanup/test_real_output_checker.py
python3 scripts/ltv_integration_cleanup/check_real_outputs.py \
  "$OUT/runs/baseline_V2_02_medium_01" \
  "$OUT/runs/current_V2_02_medium_01" --out /tmp/new-v2-02-comparison.json
```

Apply the same comparison to V2_03 and the two baseline repeats. Existing
comparison output files are never silently overwritten.

Frozen-input fallback replay was not required because native repeats and branch
comparisons are exact. Live ROS topic/bag launch, ROS1 compilation, other datasets,
ATE, unrelated upstream simulator tests and intentional-failure/golden harnesses
were not executed in this follow-up. No parameters, seed/history algorithms or
architecture were changed. Production G/V remain disabled. These results prove
cleanup equivalence on the tested real inputs; **R4 NOT_ACHIEVED remains unchanged**.
