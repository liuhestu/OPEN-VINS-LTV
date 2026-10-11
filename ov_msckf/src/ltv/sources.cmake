# Mathematical observer, feature processing, integration and fusion.
set(LTV_PRODUCTION_SOURCES
    src/ltv/observer/LtvObserver.cpp
    src/ltv/features/LtvSeedGeometry.cpp
    src/ltv/features/LtvFeatureManager.cpp
    src/ltv/features/LtvFeatureFrontend.cpp
    src/ltv/runtime/LtvAvailabilityPolicy.cpp
    src/ltv/runtime/LtvLandmarkHistory.cpp
    src/ltv/runtime/LtvRuntime.cpp
    src/ltv/runtime/LtvOpenVinsInput.cpp
    src/ltv/fusion/UpdaterLTV.cpp
)
