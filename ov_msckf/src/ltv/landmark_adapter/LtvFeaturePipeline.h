#pragma once

#include "ltv/landmark_adapter/LtvLandmarkAdapter.h"

namespace ltv {
// Source compatibility for existing study tools and downstream callers.
using LtvFeaturePipeline = LtvLandmarkAdapter;
} // namespace ltv
