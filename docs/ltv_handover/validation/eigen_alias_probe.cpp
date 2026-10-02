#include <eigen3/Eigen/Dense>
#include <iostream>

int main()
{
    Eigen::MatrixXd original(5, 5);
    for (int row = 0; row < 5; ++row)
        for (int column = 0; column < 5; ++column)
            original(row, column) = 10 * row + column;
    Eigen::MatrixXd inPlace = original;
    const Eigen::MatrixXd expected = 0.5 * (original + original.transpose());
    inPlace = 0.5 * (inPlace + inPlace.transpose());
    const double error = (inPlace - expected).cwiseAbs().maxCoeff();
    Eigen::MatrixXd a = original / 100;
    Eigen::MatrixXd p = original * original.transpose() + Eigen::MatrixXd::Identity(5, 5);
    const Eigen::MatrixXd oldP = p;
    const Eigen::MatrixXd noise = Eigen::MatrixXd::Identity(5, 5);
    const Eigen::MatrixXd expectedPropagation = oldP + .001 * (a * oldP + oldP * a.transpose() + noise);
    p += .001 * (a * p + p * a.transpose() + noise);
    const double propagationError = (p - expectedPropagation).cwiseAbs().maxCoeff();
    Eigen::VectorXd state = Eigen::VectorXd::LinSpaced(5, 1, 5);
    const Eigen::VectorXd expectedState = state + .001 * (a * state + Eigen::VectorXd::Ones(5));
    state += .001 * (a * state + Eigen::VectorXd::Ones(5));
    const double stateError = (state - expectedState).cwiseAbs().maxCoeff();
    p = oldP;
    const Eigen::MatrixXd c = original.topRows(3) / 100;
    const Eigen::MatrixXd pct = p * c.transpose();
    const Eigen::MatrixXd expectedCorrection = oldP - .001 * pct * c * oldP;
    p -= .001 * pct * c * p;
    const double correctionError = (p - expectedCorrection).cwiseAbs().maxCoeff();
    std::cout << "{\"nonsymmetric_in_place_symmetrization_max_error\":" << error
              << ",\"frozen_old_P_propagation_max_error\":" << propagationError
              << ",\"frozen_old_state_propagation_max_error\":" << stateError
              << ",\"frozen_old_P_correction_max_error\":" << correctionError << "}\n";
    if (propagationError > 1e-10 || stateError > 1e-10 || correctionError > 1e-10)
        return 1;
    return 0;
}
