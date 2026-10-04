# Frozen Passive LTV module boundary

Base: `f3e055ab2dc185d3a5910c9fa60c505dd912671d`.
This branch reorganizes the existing R4 implementation. It does not change its
algorithms, parameters, C02 thresholds, working domain or acceptance status.
G/V feedback remains disabled. The R4 evidence and its unmet contracts remain
historical records, not a claim of new acceptance.

| Component | Owns | Does not own |
|---|---|---|
| OpenVINS frontend / estimator | Tracking, feature IDs, calibration, poses and covariance | LTV lifecycle or Riccati state |
| `ltv::LtvLandmarkAdapter` | History, identity guard, stereo/temporal seed and uncertainty, admission, lifecycle, active consistency | Observer x/P, readiness or main estimator writes |
| `ltv::LtvObserver` | Mathematical state, Riccati propagation/correction and explicit controlled slot application | Tracker, seed geometry, quality gates, policy retirement decisions or readiness |
| `ov_msckf::LtvAdapter` | IMU timing/calibration, local ID mapping, staged transaction and epoch reset; owns the other components | Feature policy implementation or OpenVINS state writes |
| `ltv::LtvReadiness` | Availability and G/V output health | Observation rejection or core-state mutation |
| `UpdaterLTV` | Existing auxiliary residual/update path | Enabled feedback in this branch |

`LtvLandmarkContext.h` holds the caller-supplied causal geometry value types. It
allows Active Consistency to depend on context alone rather than including the
whole pipeline. The context is passed by const reference; no estimator/tracker
handles cross this boundary.

`ltv_controlled_features.h` holds the value-only retained IDs, once-only births
and controlled result contract. The Observer applies these commands and checks
invariants; it does not decide which physical features should be admitted or
retired. Retaining those core checks preserves the existing behavior.

The existing transaction in `LtvAdapterHardened.cpp` is unchanged: evaluate a
copy of the landmark adapter, stage IDs and full observer x/P, then commit or
roll back the whole event. The new class remains copyable with the same members
and member order. All current seed, quality, identity, lifecycle, consistency,
maturity, readiness, warmup and grace mechanisms are retained.

## Compatibility

`LtvFeaturePipeline.h` remains a forwarding header with the type alias
`LtvFeaturePipeline = LtvLandmarkAdapter`. Existing source callers and research
tools need no changes. `feature_pipeline()` remains as an accessor alias for
`landmark_adapter()`. Existing context/config/frame names are preserved.
The implementation remains at `LtvFeaturePipeline.cpp` to avoid unnecessary
source-list changes in the ROS 1/ROS 2 builds and historical tools.

Renaming the concrete class changes its C++ mangled symbols. Rebuild the library
and dependent binaries together; this is source compatibility, not compatibility
with previously compiled pipeline callers.

No experimental branch or diagnostic tool was deleted or promoted into the
production path. Consolidating seed classes or changing readiness/lifecycle is
outside this cleanup.

## Verification commands

The standalone target compiles the actual production sources and registers
existing assert-enabled tests without requiring ROS:

```bash
cmake -S ov_msckf/tests/ltv/standalone -B /tmp/ltv-cleanup -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/ltv-cleanup -j2
ctest --test-dir /tmp/ltv-cleanup --output-on-failure
```

The same CMake project can build the frozen checkout by setting
`-DLTV_SOURCE_ROOT=/absolute/path/to/f3e055ab-checkout`. With identical existing
CSR inputs, compare the two production wrappers:

```bash
python scripts/ltv_integration_cleanup/compare_adapter.py \
  --before /tmp/ltv-baseline/libltv_synthetic_wrapper.so \
  --after /tmp/ltv-cleanup/libltv_synthetic_wrapper.so \
  --input-dir /absolute/path/to/input-directory \
  --output /tmp/adapter-parity.json
```

Each library is loaded in a separate spawned process. This also prevents two
full OpenVINS libraries with the same SONAME from being silently reused by the
dynamic linker. This requires numpy and the existing input runner's Python dependencies. It
compares all camera-event x/P and slot arrays plus lifecycle, geometry and health
JSON. Only `compute_time_ms` is excluded. Modes 0, 4 and 5 cover Legacy Passive,
Active Consistency OFF and frozen C02 respectively. The caller must identify the
baseline checkout; shared-library hashes alone do not establish its source SHA.

True default LTV-disabled parity and real V2_02/V2_03 regressions are separate
checks. A synthetic fixture cannot substitute for either. See validation.md for
what was actually executed and what remains unavailable.
