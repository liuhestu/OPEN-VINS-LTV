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

struct Evidence {std::string xp,life;std::map<size_t,std::pair<int,int>> slots;uint64_t epoch,sequence;double camera,imu,cursor;size_t ignored_ids=0;};
Evidence decode(const std::string &expected){std::istringstream input(expected,std::ios::binary);Binary b(static_cast<std::istream&>(input));LtvFrame f;
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


 Evidence e;e.epoch=f.epoch;e.sequence=f.sequence;e.camera=f.camera_time;e.imu=f.imu_time;e.cursor=f.cursor;
 size_t begin=input.tellg();Eigen::VectorXd x;Eigen::MatrixXd P;b.matrix(x);b.matrix(P);e.xp=expected.substr(begin,size_t(input.tellg())-begin);
 size_t n=0;b.count(n);for(size_t i=0;i<n;++i){size_t id=0;int core,slot;b.count(id,UINT64_MAX);b.scalar(core);b.scalar(slot);if(slot>=0){if(!e.slots.emplace(id,std::make_pair(core,slot)).second)throw std::runtime_error("duplicate slot ID");}else ++e.ignored_ids;}
 ltv::LandmarkManagerFrame m;b.scalar(m.accepted_input);b.text(m.reason);b.scalar(m.epoch);b.scalar(m.time);begin=input.tellg();
 for(auto *v:{&m.retained_ids,&m.retired_ids}){n=0;b.count(n);v->resize(n);for(auto &id:*v)b.count(id,UINT64_MAX);}
 observations(b,m.observations);n=0;b.count(n);m.births.resize(n);for(auto &birth:m.births){b.count(birth.feature_id,UINT64_MAX);b.scalar(birth.apply_seed);seed(b,birth.seed);}e.life=expected.substr(begin,size_t(input.tellg())-begin);return e;
}
struct Record{uint8_t type;std::string input,expected;};
Record next(std::ifstream &in){Binary outer(static_cast<std::istream&>(in));Record r;outer.scalar(r.type);std::string payload;outer.text(payload);if(r.type==0){if(!payload.empty()||in.peek()!=EOF)throw std::runtime_error("trailing cache bytes");return r;}if(r.type==1){r.input=payload;return r;}std::istringstream stream(payload,std::ios::binary);Binary b(static_cast<std::istream&>(stream));double time;uint64_t version;b.scalar(time);b.scalar(version);
 if(r.type==2){size_t n=0;b.count(n);for(size_t i=0;i<n;++i){size_t id=0;Eigen::Vector3d v;b.count(id,UINT64_MAX);b.matrix(v);}LtvCalibration c;calibration(b,c);Eigen::Vector3d ba,bg;b.matrix(ba);b.matrix(bg);bool present;b.scalar(present);ltv::FeaturePipelineContext ctx;if(present)context(b,ctx);}
 else if(r.type==3){std::string reason;b.text(reason);}else throw std::runtime_error("invalid type");
 r.input=payload.substr(0,size_t(stream.tellg()));b.text(r.expected);if(stream.peek()!=EOF)throw std::runtime_error("record trailing bytes");return r;
}
}}
int main(int argc,char**argv){try{using namespace ov_msckf;if(argc==5 && std::string(argv[1])=="--fixture") {std::ifstream in(argv[2],std::ios::binary);in.seekg(sizeof(MAGIC));Record r;do{r=next(in);if(!r.type)throw std::runtime_error("no process fixture");}while(r.type!=2);auto e=decode(r.expected);size_t at=r.expected.find(e.xp);if(at==std::string::npos||e.xp.size()<24)throw std::runtime_error("fixture x/P absent");for(int k=0;k<2;++k){std::ofstream out(argv[3+k],std::ios::binary);out.write(MAGIC,sizeof(MAGIC));std::string expected=r.expected;if(k){double z;std::memcpy(&z,&expected[at+16],8);z+=1.;std::memcpy(&expected[at+16],&z,8);}std::ostringstream encoded(std::ios::binary);encoded.write(r.input.data(),r.input.size());Binary inner(static_cast<std::ostream&>(encoded));inner.text(expected);std::string payload=encoded.str();Binary outer(static_cast<std::ostream&>(out));outer.scalar(r.type);outer.text(payload);uint8_t zero=0;std::string empty;outer.scalar(zero);outer.text(empty);}return 0;}if(argc!=3)return 2;std::ifstream a(argv[1],std::ios::binary),b(argv[2],std::ios::binary);for(auto*p:{&a,&b}){char magic[sizeof(MAGIC)];p->read(magic,sizeof(magic));if(std::memcmp(magic,MAGIC,sizeof(magic)))throw std::runtime_error("header");}size_t index=0,frames=0,imu=0,metadata=0,retiredmap=0;for(;;){auto x=next(a),y=next(b);++index;auto fail=[&](const char*s){throw std::runtime_error(std::string(s)+" at record "+std::to_string(index));};if(x.type!=y.type)fail("event type");if(x.input!=y.input)fail("input bytes");if(!x.type)break;if(x.type==1){++imu;continue;}auto p=decode(x.expected),q=decode(y.expected);if(p.camera!=q.camera||p.imu!=q.imu||p.cursor!=q.cursor)fail("physical timestamps");if(p.xp!=q.xp)fail("full x/P");if(p.slots!=q.slots)fail("active full ID/core slot mapping");if(p.life!=q.life)fail("retained/retired/observations/births");metadata+=x.expected!=y.expected;retiredmap+=p.ignored_ids!=q.ignored_ids;++frames;}
 std::cout<<"{\"status\":\"PASS_FULL_NUMERICAL_CACHE\",\"frames\":"<<frames<<",\"imu\":"<<imu<<",\"metadata_difference_frames\":"<<metadata<<",\"inactive_mapping_difference_frames\":"<<retiredmap<<"}\n";
 }catch(const std::exception&e){std::cerr<<e.what()<<"\n";return 1;}}
