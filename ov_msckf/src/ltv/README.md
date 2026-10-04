# Passive LTV module boundary

This is the implementation layout, not a proposed grouping. Production and
standalone builds use `sources.cmake`; no flat production headers remain here.

| Directory | Responsibility | Entry / components |
|---|---|---|
| `landmark_adapter/` | Bounded history, identity, seed geometry/uncertainty/quality, admission and lifecycle, active consistency | `LtvLandmarkAdapter`; `LtvFeatureHistory`, `LtvSeedEstimator`, `LtvSeedUncertainty`, `LtvFeatureQuality`, `LtvLandmarkManager`, `LtvIdentityGuard`, `LtvActiveConsistency` |
| `observer/` | Observer x/P; OpenVINS input conversion; IMU/camera timing, transactional application, readiness | `ltv_observer`, `ltv_types`, `ltv_controlled_features`, `LtvOpenVinsInput`, `LtvAdapter`, `LtvReadiness` |
| `fusion/` | Existing G/V residual and EKF update implementation | `UpdaterLTV`; G/V injection stays disabled in the Passive configuration |
| `diagnostics/` | Cache capture/replay, bounded audit and value diagnostics | `LtvPassiveCache`, `LtvBoundedDiagnostics`, `ValueDiagnostics` |
| `config/` | Existing option loading and validation | `LtvOptions` |

## OpenVINS integration

`VioManager` queries the existing FeatureDatabase at the camera timestamp and
passes the current features and State to `makeLtvOpenVinsInput`. This read-only
bridge produces one event-local value snapshot. It queries each feature/camera
observation once, using the left-camera bearing for both the observer and landmark
context, and reuses the left-camera extrinsics already converted for calibration.
It retains no Feature/State pointers after conversion.

Clones, extrinsics, time offset, IMU calibration and the full joint clone
covariance come from OpenVINS. The execution clone, ordering, JPL error convention
and camera-to-IMU timestamps are unchanged. Biases still come from the existing
pre-propagation capture in VioManager. LtvAdapter owns IMU timing and the
landmark/observer transaction; the input bridge only converts OpenVINS values.

LtvLandmarkAdapter owns policy/history, never observer x/P or estimator state.
LtvObserver applies controlled retained IDs and once-only births, preserving its
invariant checks. UpdaterLTV is the sole existing auxiliary fusion implementation;
neither adapter writes the main estimator.

## Deliberately retained

The short causal LTV history has epoch, bounded retention, protected IDs and
transaction rollback semantics. OpenVINS FeatureDatabase cleans tracks according
to estimator marginalization, so it cannot replace that history without changing
behavior. OpenVINS triangulated landmarks/FeatureInitializer are not substituted
for the frozen stereo/temporal seed algorithm.

`LtvFeaturePipeline.h` is a legacy type alias inside landmark_adapter, not a
second implementation. Its old flat include path is removed; repository callers
use the new paths. Historical experiment manifests retain their original refs
and hashes; frozen studies must be replayed from those refs.

See `docs/ltv/integration_cleanup/layout_and_openvins_input/validation.md` for
the exact tested scope. Algorithms, parameters and default-off options are frozen.
