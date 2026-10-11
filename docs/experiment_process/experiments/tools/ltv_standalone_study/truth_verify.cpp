// Algebra-only sensor contract probe. No observer is constructed or propagated.
#include <Eigen/Geometry>
#include <iomanip>
#include <iostream>
extern "C" void truth_cpp(double t, int scene, double *out);
int main() {
  using V3 = Eigen::Vector3d;
  std::cout << std::setprecision(17);
  for (int scene = 0; scene < 4; ++scene) {
    const int n = scene == 2 ? 16 : 30;
    Eigen::Matrix3d Rbc = Eigen::Matrix3d::Identity();
    if (scene != 2)
      Rbc = Eigen::AngleAxisd(.2, V3::UnitY()).toRotationMatrix();
    const V3 pc = scene == 2 ? V3(.02, .06, .01) : V3(.1, -.04, .06);
    for (double t : {0., .13, 1.7, 7.3}) {
      double raw[21];
      truth_cpp(t, scene, raw);
      const Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> R(raw);
      const Eigen::Map<const V3> p(raw + 9), v(raw + 12), a(raw + 15), w(raw + 18);
      const V3 ab = R.transpose() * (a - V3(0, 0, -9.81));
      const V3 vb = R.transpose() * v, eta = R.transpose() * V3(0, 0, -9.81);
      for (int j = 0; j < n; ++j) {
        const V3 point = scene == 2 ? V3(j / 4 - 1.5, j % 4 - 1.5, 0) : V3((j % 6 - 2.5) * 1.4, (j / 6 - 2) * 1.3, 12 + .17 * j);
        const V3 ell = R.transpose() * (point - p);
        const V3 z = (Rbc.transpose() * (ell - pc)).normalized();
        std::cout << scene << ' ' << t << ' ' << j;
        for (const V3 &value : {ab, V3(w), ell, vb, eta, z})
          for (int k = 0; k < 3; ++k)
            std::cout << ' ' << value[k];
        std::cout << '\n';
      }
    }
  }
}
