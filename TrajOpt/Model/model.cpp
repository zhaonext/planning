#include "model.h"

/* (s ,n ,alpha, x, y, phi) (v, delta)
StateVector Model::dynamics(const State &X, const Input &U) {
    StateVector odes{};

    const double x = X.x, y = X.y, phi = X.phi, s = X.s, n = X.n, alpha = X.alpha, v = X.v;
    const double acc = U.acceleration, delta = U.delta;

    const double cos_alpha = std::cos(alpha), sin_alpha = std::sin(alpha);
    const double cos_phi = std::cos(phi), sin_phi = std::sin(phi);
    const double tan_delta = std::tan(delta);

    double kappa = curve_.calKappaByS(s);
    const double denominator = 1 - n * kappa;

    // Compute ODEs
    odes(0) = v * cos_alpha / denominator;
    odes(1) = v * sin_alpha;
    odes(2) = v * tan_delta / WHEELBASE - v * kappa * cos_alpha / denominator;
    odes(3) = v * cos_phi;
    odes(4) = v * sin_phi;
    odes(5) = v * tan_delta / WHEELBASE;
    odes(6) = v;

    return odes;
}

Eigen::MatrixXd Model::getModelJacobian(const State &X, const Input &U) {
    Eigen::MatrixXd jac = Eigen::MatrixXd::Zero(NUM_STATE, NUM_STATE + NUM_INPUT);

    const double x = X.x, y = X.y, phi = X.phi, s = X.s, n = X.n, alpha = X.alpha;
    const double v = U.acceleration, delta = U.delta;

    const double cos_alpha = std::cos(alpha), sin_alpha = std::sin(alpha);
    const double cos_phi = std::cos(phi), sin_phi = std::sin(phi);

    double kappa = curve_.calKappaByS(s);
    const double one_minus_kappa_n = 1 - kappa * n;
    const double one_minus_kappa_n_sq = std::pow(one_minus_kappa_n, 2);
    const double sec_delta = 1 / std::cos(delta);
    const double sec_delta_sq = std::pow(sec_delta, 2);

    // Frenet frame state
    jac(0, 1) = (kappa * v * cos_alpha) / one_minus_kappa_n_sq;
    jac(0, 2) = -(v * sin_alpha) / one_minus_kappa_n;
    jac(0, 6) = cos_alpha / one_minus_kappa_n;

    jac(1, 2) = v * cos_alpha;
    jac(1, 6) = sin_alpha;

    jac(2, 1) = -(std::pow(kappa, 2) * v * cos_alpha) / one_minus_kappa_n_sq;
    jac(2, 2) = (kappa * v * sin_alpha) / one_minus_kappa_n;
    jac(2, 6) = std::tan(delta) / WHEELBASE - (kappa * cos_alpha) / one_minus_kappa_n;
    jac(2, 7) = (v * sec_delta_sq) / WHEELBASE;

    // Cartesian frame state
    jac(3, 5) = -v * sin_phi;
    jac(3, 6) = cos_phi;

    jac(4, 5) = v * cos_phi;
    jac(4, 6) = sin_phi;

    jac(5, 6) = std::tan(delta) / WHEELBASE;
    jac(5, 7) = v * sec_delta_sq / WHEELBASE;

    return jac;
}*/

// (s ,n ,alpha, x, y, v, phi) (acc, phidot)
StateVector Model::dynamics(const State &X, const Input &U) {
    StateVector odes{};

    const double x = X.x, y = X.y, phi = X.phi, s = X.s, n = X.n, alpha = X.alpha, v = X.v;
    const double a = U.acceleration, phidot = U.phidot;

    const double cos_alpha = std::cos(alpha), sin_alpha = std::sin(alpha);
    const double cos_phi = std::cos(phi), sin_phi = std::sin(phi);

    //        double kappa = curve_->calKappaByS(s);
    double kappa;
    ref_path_->GetKappaFromS(s, &kappa);

    const double denominator = 1 - n * kappa;

    // Frenet ODE
    odes(0) = v * cos_alpha / denominator;
    odes(1) = v * sin_alpha;
    odes(2) = phidot - v * kappa * cos_alpha / denominator;
    // Cartesian ODE
    odes(3) = v * cos_phi;
    odes(4) = v * sin_phi;
    odes(5) = a;
    odes(6) = phidot;

    return odes;
}

Eigen::MatrixXd Model::getModelJacobian(const State &X, const Input &U) {
    Eigen::MatrixXd jac = Eigen::MatrixXd::Zero(NUM_STATE, NUM_STATE + NUM_INPUT);

    const double x = X.x, y = X.y, phi = X.phi, s = X.s, n = X.n, alpha = X.alpha, v = X.v;

    //    double kappa = curve_->calKappaByS(s);
    double kappa;
    ref_path_->GetKappaFromS(s, &kappa);

    // Frenet frame state
    jac(0, 1) = (kappa * v * Cos(alpha)) / Power(1 - kappa * n, 2);
    jac(0, 2) = -((v * Sin(alpha)) / (1 - kappa * n));
    jac(0, 5) = Cos(alpha) / (1 - kappa * n);

    jac(1, 2) = v * Cos(alpha);
    jac(1, 5) = Sin(alpha);

    jac(2, 1) = -((Power(kappa, 2) * v * Cos(alpha)) / Power(1 - kappa * n, 2));
    jac(2, 2) = (kappa * v * Sin(alpha)) / (1 - kappa * n);
    jac(2, 5) = -((kappa * Cos(alpha)) / (1 - kappa * n));
    jac(2, 8) = 1;

    // Cartesian frame state
    jac(3, 5) = Cos(phi);
    jac(3, 6) = -(v * Sin(phi));

    jac(4, 5) = Sin(phi);
    jac(4, 6) = v * Cos(phi);

    jac(5, 7) = 1;
    jac(6, 8) = 1;

    return jac;
}
