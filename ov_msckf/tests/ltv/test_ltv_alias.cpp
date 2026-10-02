#include <Eigen/Dense>
#include <iostream>

// A nonzero exit is an intentional acceptance failure, not a probe crash.
int main() {
  Eigen::MatrixXd original(5, 5);
  for (int r = 0; r < original.rows(); ++r)
    for (int c = 0; c < original.cols(); ++c)
      original(r, c) = 10 * r + c;
  const Eigen::MatrixXd expected = (0.5 * (original + original.transpose())).eval();
  Eigen::MatrixXd actual = original;
  actual = 0.5 * (actual + actual.transpose());
  const double error = (actual - expected).cwiseAbs().maxCoeff();
  const double asymmetry = (actual - actual.transpose()).cwiseAbs().maxCoeff();
  std::cout << "max_error=" << error << " max_asymmetry=" << asymmetry << std::endl;
  return error <= 1e-12 && asymmetry <= 1e-12 ? 0 : 1;
}
