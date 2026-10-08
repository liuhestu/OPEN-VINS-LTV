#pragma once
#include "ltv/observer/LtvAdapter.h"
#include <memory>
namespace ov_core {
class Feature;
}
namespace ov_msckf {
class State;
// A camera-event value snapshot. OpenVINS owns source state and tracks;
// no source pointers survive this conversion and no estimator state is written.
struct LtvOpenVinsInput {
  LtvCalibration calibration;
  ltv::FeaturePipelineContext feature_context;
  std::vector<LtvBearing> bearings;
};
LtvOpenVinsInput makeLtvOpenVinsInput(const std::shared_ptr<State> &state, const std::vector<std::shared_ptr<ov_core::Feature>> &features,
                                      double camera_time, uint64_t version, bool managed_features, double pixel_noise_variance = 1.);
} // namespace ov_msckf
