// Standalone C ABI: Python owns input generation and files; the actual core owns estimation.
#include "ltv_observer.h"
#include <Eigen/Geometry>
#include <algorithm>
#ifdef EXPERIMENTAL
#include <functional>
#endif
using V3 = Eigen::Vector3d;
using M3 = Eigen::Matrix3d;
extern "C" {
void *create(double q, double v, double p) {
  auto *o = new ltv::LtvObserver;
  ltv::LtvConfig c;
  c.enable = true;
  c.q_landmark = q;
  c.v_landmark = c.v_velocity = c.v_gravity = v;
  c.initial_p_landmark = c.initial_p_velocity = c.initial_p_gravity = p;
  o->configure(c);
  o->start(0);
  return o;
}
void destroy(void *o) { delete static_cast<ltv::LtvObserver *>(o); }
int predict(void *ptr, double dt, const double *a, const double *w) {
  auto &o = *static_cast<ltv::LtvObserver *>(ptr);
  o.propagateImu(dt, Eigen::Map<const V3>(a), Eigen::Map<const V3>(w), V3::Zero(), V3::Zero());
  return o.started();
}
int camera(void *ptr, double t, int n, const int *ids, const double *z, const double *r, const double *p, int ns, const int *seed_ids,
           const double *seeds, const double *exact) {
  auto &o = *static_cast<ltv::LtvObserver *>(ptr);
  std::vector<ltv::LtvFeatureObservation> observations;
  for (int j = 0; j < n; ++j) {
    ltv::LtvFeatureObservation f;
    f.feature_id = ids[j];
    f.normalized_coordinate = Eigen::Map<const V3>(z + 3 * j);
    observations.push_back(f);
  }
#ifdef EXPERIMENTAL
  o.mean_hook = [=](Eigen::VectorXd &x, const std::unordered_map<int, int> &slots) {
    for (int j = 0; j < ns; ++j) {
      const auto it = slots.find(seed_ids[j]);
      if (it != slots.end())
        x.segment<3>(3 * it->second) = Eigen::Map<const V3>(seeds + 3 * j);
    }
    if (exact)
      x = Eigen::Map<const Eigen::VectorXd>(exact, x.size());
  };
#endif
  o.updateFeatures(t, t, observations, Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(r), Eigen::Map<const V3>(p));
  return o.started();
}
int read_state(void *ptr, double *x, double *p, int *ids, double *diag) {
  auto &o = *static_cast<ltv::LtvObserver *>(ptr);
  const int d = o.state().size();
  std::copy(o.state().data(), o.state().data() + d, x);
  Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(p, d, d) = o.covariance();
  for (int j = 0; j < (d - 6) / 3; ++j)
    ids[j] = -1;
  // Reverse lookup is restricted to caller's supplied observed/history IDs via slot().
  auto s = o.snapshot();
  diag[0] = s.camera_substeps;
  diag[1] = int(s.last_reset_reason);
  diag[2] = s.update_time_ms;
#ifdef EXPERIMENTAL
  diag[3] = o.protection_count;
  diag[4] = o.protection_norm;
  diag[5] = o.minimum_before_protection;
#endif
  return d;
}
int slot(void *ptr, int id) { return static_cast<ltv::LtvObserver *>(ptr)->slotForFeature(id); }
void truth_cpp(double t, int scene, double *out) {
  M3 R;
  V3 p, v, a, w;
  if (scene == 2) {
    const V3 axis(-1, 3, 0);
    R = (Eigen::AngleAxisd(t * axis.norm(), axis.normalized()) * Eigen::AngleAxisd(-2 * t, V3::UnitY())).toRotationMatrix();
    p = 2 * V3(std::sin(t), std::sin(t) * std::cos(t), 1);
    v = V3(2 * std::cos(t), 2 * std::cos(2 * t), 0);
    a = V3(-2 * std::sin(t), -4 * std::sin(2 * t), 0);
    w = V3(-std::cos(2 * t), 1, std::sin(2 * t));
  } else {
    const bool moving = scene != 3;
    const double scale = scene == 1 ? 3 : 1;
    const V3 axis = V3(.3, -.4, .8).normalized();
    R = (Eigen::AngleAxisd(.23, V3::UnitX()) * Eigen::AngleAxisd(-.18, V3::UnitY()) *
         Eigen::AngleAxisd(moving ? .6 * std::sin(.8 * t) : 0, axis))
            .toRotationMatrix();
    p = scale * V3(1.8 * std::sin(.7 * t), 1.2 * std::sin(1.1 * t), .7 * std::sin(.9 * t));
    v = scale * V3(1.26 * std::cos(.7 * t), 1.32 * std::cos(1.1 * t), .63 * std::cos(.9 * t));
    a = scale * V3(-.882 * std::sin(.7 * t), -1.452 * std::sin(1.1 * t), -.567 * std::sin(.9 * t));
    w = moving ? (.48 * std::cos(.8 * t) * axis).eval() : V3::Zero();
    if (!moving) {
      p.setZero();
      v.setZero();
      a.setZero();
    }
  }
  Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> mapped_rotation(out);
  mapped_rotation = R;
  Eigen::Map<V3>(out + 9) = p;
  Eigen::Map<V3>(out + 12) = v;
  Eigen::Map<V3>(out + 15) = a;
  Eigen::Map<V3>(out + 18) = w;
}
}
