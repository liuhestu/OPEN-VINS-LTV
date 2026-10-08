// Optional read-only interception of the actual core's feature selection, for diagnosis only.
#include "ltv/observer/ltv_observer.h"
#include <cstdio>
#include <cstdlib>
#include <dlfcn.h>
#include <map>
namespace ltv {
LtvSnapshot LtvObserver::updateFeatures(double frame, double imu, const std::vector<LtvFeatureObservation> &observations,
                                        const Eigen::Matrix3d &Rbc, const Eigen::Vector3d &pc) {
  using Function = LtvSnapshot (*)(LtvObserver *, double, double, const std::vector<LtvFeatureObservation> &, const Eigen::Matrix3d &,
                                   const Eigen::Vector3d &);
  static auto original =
      reinterpret_cast<Function>(dlsym(RTLD_NEXT, "_ZN3ltv11LtvObserver14updateFeaturesEddRKSt6vectorINS_21LtvFeatureObservationESaIS2_"
                                                  "EERKN5Eigen6MatrixIdLi3ELi3ELi0ELi3ELi3EEERKNS8_IdLi3ELi1ELi0ELi3ELi1EEE"));
  if (!original)
    std::abort();
  auto result = original(this, frame, imu, observations, Rbc, pc);
  static FILE *out = [] {
    const char *path = std::getenv("LTV_LIFECYCLE_TRACE");
    FILE *f = path ? std::fopen(path, "w") : nullptr;
    if (!f)
      std::abort();
    std::fprintf(f, "t,event,id,slot,resident_age,range,ray_depth,projection_residual\n");
    return f;
  }();
  static std::map<const LtvObserver *, std::map<int, double>> births;
  auto &active = births[this];
  for (auto it = active.begin(); it != active.end();) {
    if (slotForFeature(it->first) < 0) {
      std::fprintf(out, "%.17g,death,%d,-1,%.17g,0,0,0\n", frame, it->first, frame - it->second);
      it = active.erase(it);
    } else
      ++it;
  }
  for (const auto &feature : observations) {
    const int slot = slotForFeature(feature.feature_id);
    if (slot < 0)
      continue;
    const auto added = active.emplace(feature.feature_id, frame);
    const Eigen::Vector3d point = state().segment<3>(3 * slot) - pc;
    const Eigen::Vector3d bearing = Rbc * feature.normalized_coordinate.normalized();
    const double depth = bearing.dot(point), residual = (point - bearing * depth).norm();
    std::fprintf(out, "%.17g,%s,%d,%d,%.17g,%.17g,%.17g,%.17g\n", frame, added.second ? "birth" : "observation", feature.feature_id, slot,
                 frame - added.first->second, point.norm(), depth, residual);
  }
  return result;
}
} // namespace ltv
