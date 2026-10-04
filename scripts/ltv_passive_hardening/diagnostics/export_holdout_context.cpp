#include "ltv/LtvPassiveCache.h"
#include <cstring>
#include <iomanip>
#include <iostream>
#include <set>
#include <sstream>
#include <stdexcept>
#include <type_traits>
namespace ov_msckf {
namespace {
const char MAGIC[] = "LTVPCACHE001";
constexpr uint64_t MAX_BYTES = 256ULL * 1024 * 1024;
struct Binary {
  std::istream *in = nullptr;
  std::ostream *out = nullptr;
  explicit Binary(std::istream &s) : in(&s) {}
  explicit Binary(std::ostream &s) : out(&s) {}
  template <typename T> void scalar(T &v) {
    static_assert(std::is_arithmetic<T>::value, "scalar arithmetic only");
    if (in)
      in->read(reinterpret_cast<char *>(&v), sizeof(v));
    else
      out->write(reinterpret_cast<const char *>(&v), sizeof(v));
    if ((in && !*in) || (out && !*out))
      throw std::runtime_error("cache I/O error");
  }
  void count(size_t &v, uint64_t limit = 1000000) {
    uint64_t n = v;
    scalar(n);
    if (n > limit)
      throw std::runtime_error("cache dimension limit");
    v = static_cast<size_t>(n);
  }
  void text(std::string &s) {
    size_t n = s.size();
    count(n, MAX_BYTES);
    if (in) {
      s.resize(n);
      if (n)
        in->read(&s[0], n);
    } else if (n)
      out->write(s.data(), n);
    if ((in && !*in) || (out && !*out))
      throw std::runtime_error("cache string I/O error");
  }
  template <typename Derived> void matrix(Eigen::MatrixBase<Derived> &base) {
    auto &m = base.derived();
    size_t rows = m.rows(), cols = m.cols();
    count(rows, 10000);
    count(cols, 10000);
    if (rows * cols > MAX_BYTES / sizeof(double))
      throw std::runtime_error("cache matrix too large");
    if ((Derived::RowsAtCompileTime != Eigen::Dynamic && rows != static_cast<size_t>(Derived::RowsAtCompileTime)) ||
        (Derived::ColsAtCompileTime != Eigen::Dynamic && cols != static_cast<size_t>(Derived::ColsAtCompileTime)))
      throw std::runtime_error("cache matrix shape mismatch");
    if (in)
      m.resize(rows, cols);
    for (size_t j = 0; j < cols; ++j)
      for (size_t i = 0; i < rows; ++i)
        scalar(m(i, j));
  }
};
void platform() {
  uint32_t one = 1;
  if (sizeof(double) != 8 || *reinterpret_cast<char *>(&one) != 1)
    throw std::runtime_error("cache requires little endian IEEE754 host");
}
void sample(Binary &b, ov_core::ImuData &s) {
  b.scalar(s.timestamp);
  b.matrix(s.wm);
  b.matrix(s.am);
}
void observations(Binary &b, std::vector<ltv::HistoryObservation> &os) {
  size_t n = os.size();
  b.count(n);
  if (b.in)
    os.resize(n);
  for (auto &o : os) {
    b.count(o.feature_id, UINT64_MAX);
    b.scalar(o.camera_id);
    b.matrix(o.bearing);
    b.scalar(o.match_valid);
  }
}
void context(Binary &b, ltv::FeaturePipelineContext &c) {
  b.scalar(c.epoch);
  b.scalar(c.version);
  b.scalar(c.time);
  b.count(c.execution_pose_index, 10000);
  size_t n = c.poses.size();
  b.count(n, 1000);
  if (b.in)
    c.poses.resize(n);
  for (auto &p : c.poses) {
    b.matrix(p.R_WB);
    b.matrix(p.p_WB);
    b.scalar(p.t);
  }
  b.matrix(c.pose_covariance);
  n = c.cameras.size();
  b.count(n, 16);
  if (b.in)
    c.cameras.resize(n);
  for (auto &cam : c.cameras) {
    b.matrix(cam.R_BC);
    b.matrix(cam.p_BC);
  }
  observations(b, c.observations);
}
void calibration(Binary &b, LtvCalibration &c) {
  b.matrix(c.Da);
  b.matrix(c.Dw);
  b.matrix(c.Tg);
  b.matrix(c.R_ACCtoIMU);
  b.matrix(c.R_GYROtoIMU);
  b.matrix(c.R_BC);
  b.matrix(c.p_BC);
  b.scalar(c.offset);
}
void seed(Binary &b, ltv::FeatureSeedCandidate &s) {
  b.count(s.feature_id, UINT64_MAX);
  b.scalar(s.epoch);
  b.scalar(s.time);
  b.matrix(s.landmark_B);
  int source = static_cast<int>(s.source);
  b.scalar(source);
  b.scalar(s.geometry_valid);
  b.scalar(s.independent_check_available);
  b.scalar(s.min_ray);
  b.scalar(s.condition);
  b.scalar(s.parallax_rad);
  b.scalar(s.residual_rad);
  b.scalar(s.relative_risk);
  b.scalar(s.check_residual_rad);
}
template <typename D> void array(const Eigen::MatrixBase<D> &v) {
  std::cout << "[";
  for (int i = 0; i < v.rows(); ++i)
    for (int j = 0; j < v.cols(); ++j) {
      if (i || j)
        std::cout << ",";
      std::cout << v(i, j);
    }
  std::cout << "]";
}

void emit(double camera_time, const ltv::FeaturePipelineContext &c) {
  std::cout << "{\"camera_time\":" << camera_time << ",\"time\":" << c.time << ",\"epoch\":" << c.epoch << ",\"version\":" << c.version
            << ",\"execution_pose_index\":" << c.execution_pose_index << ",\"poses\":[";
  for (size_t i = 0; i < c.poses.size(); ++i) {
    if (i)
      std::cout << ",";
    auto &p = c.poses[i];
    std::cout << "{\"time\":" << p.t << ",\"R_WB\":";
    array(p.R_WB);
    std::cout << ",\"p_WB\":";
    array(p.p_WB);
    std::cout << "}";
  }
  std::cout << "],\"cameras\":[";
  for (size_t i = 0; i < c.cameras.size(); ++i) {
    if (i)
      std::cout << ",";
    std::cout << "{\"R_BC\":";
    array(c.cameras[i].R_BC);
    std::cout << ",\"p_BC\":";
    array(c.cameras[i].p_BC);
    std::cout << "}";
  }
  std::cout << "],\"observations\":[";
  bool first = true;
  for (const auto &o : c.observations) {
    if (o.feature_id != 27537 && o.feature_id != 29636)
      continue;
    if (!first)
      std::cout << ",";
    first = false;
    std::cout << "{\"id\":" << o.feature_id << ",\"camera\":" << o.camera_id << ",\"valid\":" << o.match_valid << ",\"bearing\":";
    array(o.bearing);
    std::cout << "}";
  }
  std::cout << "]}\n";
}
} // namespace
} // namespace ov_msckf
int main(int argc, char **argv) {
  try {
    using namespace ov_msckf;
    if (argc != 2)
      return 2;
    std::cout << std::setprecision(17);
    std::ifstream in(argv[1], std::ios::binary);
    char magic[sizeof(MAGIC)];
    in.read(magic, sizeof(magic));
    if (std::memcmp(magic, MAGIC, sizeof(magic)))
      throw std::runtime_error("magic");
    Binary outer(static_cast<std::istream &>(in));
    for (;;) {
      uint8_t type;
      std::string payload;
      outer.scalar(type);
      outer.text(payload);
      if (!type) {
        if (in.peek() != EOF)
          throw std::runtime_error("trailing bytes");
        break;
      }
      if (type != 2)
        continue;
      std::istringstream data(payload, std::ios::binary);
      Binary b(static_cast<std::istream &>(data));
      double time;
      uint64_t version;
      b.scalar(time);
      b.scalar(version);
      bool selected = false;
      for (double target : {1413394964105760512LL * 1e-9, 1413394966255760384LL * 1e-9})
        if (time >= target - 1.000001 && time <= target + 1e-6)
          selected = true;
      if (!selected)
        continue;
      size_t n = 0;
      b.count(n);
      for (size_t i = 0; i < n; ++i) {
        size_t id = 0;
        Eigen::Vector3d v;
        b.count(id, UINT64_MAX);
        b.matrix(v);
      }
      LtvCalibration c;
      calibration(b, c);
      Eigen::Vector3d ba, bg;
      b.matrix(ba);
      b.matrix(bg);
      bool present;
      b.scalar(present);
      ltv::FeaturePipelineContext ctx;
      if (present) {
        context(b, ctx);
        emit(time, ctx);
      } else
        throw std::runtime_error("Missing context");
    }
    return 0;
  } catch (const std::exception &e) {
    std::cerr << e.what() << "\n";
    return 1;
  }
}
