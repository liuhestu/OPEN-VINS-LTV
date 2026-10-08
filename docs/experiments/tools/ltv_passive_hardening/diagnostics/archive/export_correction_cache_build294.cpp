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
template<typename D> void array(const Eigen::MatrixBase<D>&v){std::cout<<"[";for(int i=0;i<v.rows();++i)for(int j=0;j<v.cols();++j){if(i||j)std::cout<<",";std::cout<<v(i,j);}std::cout<<"]";}
void decode(const std::string &expected,const LtvCalibration &c,const Eigen::Vector3d &ba,const Eigen::Vector3d &bg){
 std::istringstream input(expected,std::ios::binary);Binary b(static_cast<std::istream&>(input));LtvFrame f;
#define FIELD(x) b.scalar(f.x)
  FIELD(epoch);
  FIELD(version);
  FIELD(sequence);
  FIELD(camera_ns);
  FIELD(camera_time);
  FIELD(imu_time);
  FIELD(cursor);
  FIELD(available);
  FIELD(ready_G);
  FIELD(ready_V);
  b.count(f.mature_features);
  b.text(f.reason);
  FIELD(integrated_steps);
  FIELD(integrated_seconds);
#undef FIELD
  auto &s = f.snapshot;
#define FIELD(x) b.scalar(s.x)
  FIELD(frame_timestamp);
  FIELD(imu_timestamp);
  b.matrix(s.velocity_body);
  b.matrix(s.gravity_body);
  FIELD(valid);
  FIELD(velocity_valid);
  FIELD(gravity_valid);
  FIELD(state_features);
  FIELD(observed_features);
  FIELD(healthy_camera_updates);
  FIELD(innovation_norm);
  FIELD(covariance_trace);
  FIELD(covariance_min_diagonal);
  FIELD(covariance_max_diagonal);
  FIELD(camera_substeps);
  int reset = static_cast<int>(s.last_reset_reason);
  b.scalar(reset);
  FIELD(snapshot_time_error);
  FIELD(gravity_angle_ltv_vs_vins);
  FIELD(gravity_factor_residual_norm);
  FIELD(gravity_factor_weighted_residual_norm);
  FIELD(gravity_factor_added);
  FIELD(gravity_gate_base_eligible);
  FIELD(gravity_gate_feature_ok);
  FIELD(gravity_gate_eta_norm_ok);
  FIELD(gravity_gate_innovation_ok);
  FIELD(gravity_gate_reset_ok);
  FIELD(gravity_gate_pass);
  FIELD(gravity_gate_reason_mask);
  b.matrix(s.vins_velocity_body);
  b.matrix(s.velocity_factor_residual);
  FIELD(velocity_factor_residual_norm);
  FIELD(velocity_factor_weighted_residual_norm);
  FIELD(velocity_factor_added);
  FIELD(velocity_factor_base_eligible);
  FIELD(velocity_gate_feature_ok);
  FIELD(velocity_gate_innovation_ok);
  FIELD(velocity_gate_disagreement_ok);
  FIELD(velocity_gate_reset_ok);
  FIELD(velocity_gate_pass);
  FIELD(velocity_gate_reason_mask);
  FIELD(velocity_oracle_mask_loaded);
  FIELD(velocity_oracle_mask_hit);
  FIELD(velocity_oracle_pass);
#undef FIELD


 Eigen::VectorXd x;Eigen::MatrixXd P;b.matrix(x);b.matrix(P);size_t n=0;b.count(n);
 std::vector<std::pair<size_t,int>> slots;
 for(size_t i=0;i<n;++i){size_t id=0;int core,slot;b.count(id,UINT64_MAX);b.scalar(core);b.scalar(slot);slots.emplace_back(id,slot);}
 ltv::LandmarkManagerFrame m;b.scalar(m.accepted_input);b.text(m.reason);b.scalar(m.epoch);b.scalar(m.time);
 for(auto *v:{&m.retained_ids,&m.retired_ids}){n=0;b.count(n);v->resize(n);for(auto &id:*v)b.count(id,UINT64_MAX);}
 observations(b,m.observations);n=0;b.count(n);m.births.resize(n);for(auto &birth:m.births){b.count(birth.feature_id,UINT64_MAX);b.scalar(birth.apply_seed);seed(b,birth.seed);}
 if(std::abs(f.camera_time-1413394964105760512LL*1e-9)>.550001 && std::abs(f.camera_time-1413394966255760384LL*1e-9)>.550001)return;
 std::cout<<"{\"kind\":\"camera\",\"time\":"<<f.camera_time<<",\"target\":"<<f.imu_time<<",\"cursor\":"<<f.cursor<<",\"epoch\":"<<f.epoch<<",\"sequence\":"<<f.sequence<<",\"available\":"<<f.available<<",\"camera_substeps\":"<<s.camera_substeps<<",\"x\":";array(x);std::cout<<",\"P\":";array(P);
 std::cout<<",\"slots\":[";for(size_t i=0;i<slots.size();++i){if(i)std::cout<<",";std::cout<<"["<<slots[i].first<<","<<slots[i].second<<"]";}std::cout<<"],\"observations\":[";
 for(size_t i=0;i<m.observations.size();++i){if(i)std::cout<<",";auto&o=m.observations[i];std::cout<<"{\"id\":"<<o.feature_id<<",\"camera\":"<<o.camera_id<<",\"bearing\":";array(o.bearing);std::cout<<"}";}
 std::cout<<"],\"births\":[";for(size_t i=0;i<m.births.size();++i){if(i)std::cout<<",";auto&birth=m.births[i];std::cout<<"{\"id\":"<<birth.feature_id<<",\"apply_seed\":"<<birth.apply_seed<<",\"mean\":";array(birth.seed.landmark_B);std::cout<<"}";}
 std::cout<<"],\"calibration\":{";
#define MAT(name) std::cout<<"\"" #name "\":";array(c.name);std::cout<<",";
 MAT(Da) MAT(Dw) MAT(Tg) MAT(R_ACCtoIMU) MAT(R_GYROtoIMU) MAT(R_BC)
#undef MAT
 std::cout<<"\"p_BC\":";array(c.p_BC);std::cout<<",\"offset\":"<<c.offset<<"},\"ba\":";array(ba);std::cout<<",\"bg\":";array(bg);std::cout<<"}\n";
}
}}
int main(int argc,char**argv){try{using namespace ov_msckf;if(argc!=2)return 2;std::cout<<std::setprecision(17);std::ifstream in(argv[1],std::ios::binary);char magic[sizeof(MAGIC)];in.read(magic,sizeof(magic));if(std::memcmp(magic,MAGIC,sizeof(magic)))throw std::runtime_error("magic");Binary outer(static_cast<std::istream&>(in));for(;;){uint8_t type;std::string payload;outer.scalar(type);outer.text(payload);if(!type){if(in.peek()!=EOF)throw std::runtime_error("trailing cache bytes");break;}std::istringstream data(payload,std::ios::binary);Binary b(static_cast<std::istream&>(data));
 if(type==1){ov_core::ImuData s;sample(b,s);std::cout<<"{\"kind\":\"imu\",\"time\":"<<s.timestamp<<",\"am\":";array(s.am);std::cout<<",\"wm\":";array(s.wm);std::cout<<"}\n";continue;}
 if(type==3){std::cout<<"{\"kind\":\"pause\"}\n";continue;}
 if(type!=2)throw std::runtime_error("unknown event");double time;uint64_t version;b.scalar(time);b.scalar(version);size_t n=0;b.count(n);for(size_t i=0;i<n;++i){size_t id=0;Eigen::Vector3d v;b.count(id,UINT64_MAX);b.matrix(v);}LtvCalibration c;calibration(b,c);Eigen::Vector3d ba,bg;b.matrix(ba);b.matrix(bg);bool present;b.scalar(present);ltv::FeaturePipelineContext ctx;if(present)context(b,ctx);std::string expected;b.text(expected);decode(expected,c,ba,bg);
 }return 0;}catch(const std::exception&e){std::cerr<<e.what()<<"\n";return 1;}}
