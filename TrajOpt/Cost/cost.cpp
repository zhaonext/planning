#include "cost.h"
double Cost::calCostFunction(const State &X, const Input &U, const double desired_speed, const std::vector<std::pair<ObstacleType, TrajectoryPoint>> &obs_list_info, int time_index) {
    double n_ego = X.n, v_ego = X.v, phidot = U.phidot;
    double a = U.acceleration;

    double err_n = n_ego - 0;
    double err_v = v_ego - desired_speed;

    // calculate cost
    double cost = 0.0;

    double dis_to_obs_cost = disToObsCost(X, obs_list_info, time_index);
    cost += weight_distance_obstacle * dis_to_obs_cost;

    cost += weight_n_offset * err_n * err_n;
    if (err_v > 0) {
        cost += weight_positive_velocity * err_v * err_v;
    } else {
        cost += weight_negative_velocity * err_v * err_v;
    }

    cost += weight_acceleration * a * a;
    cost += weight_phidot * phidot * phidot;

    //    ADEBUG << "Cost Function Errors:"
    //           << "\n - Lateral Offset Error (err_n): " << err_n
    //           << "\n - Velocity Error (err_v): " << err_v
    //           << "\n - Distance to Obstacle (err_dis): " << dis_to_obs_cost
    //           << "\n - Acceleration (a): " << a
    //           << "\n - Steering Rate (phidot): " << phidot;

    //    ADEBUG << "Cost Function Calculation:"
    //           << "\n - Distance to Obstacle Cost: " << dis_to_obs_cost
    //           << "\n - Lateral Offset Cost: " << (weight_n_offset * err_n * err_n)
    //           << "\n - Velocity Error Cost: "
    //           << ((err_v > 0) ? ("Positive (" + std::to_string(weight_positive_velocity * err_v * err_v) + ")") : ("Negative (" + std::to_string(weight_negative_velocity * err_v * err_v) + ")"))
    //           << "\n - Acceleration Cost: " << (weight_acceleration * a * a)
    //           << "\n - Steering Rate Cost: " << (weight_phidot * phidot * phidot)
    //           << "\n - Total Cost: " << cost;

    return cost;
}

Eigen::VectorXd Cost::calJacobian(const State &X, const Input &U, const double desired_speed, const std::vector<std::pair<ObstacleType, TrajectoryPoint>> &obs_list_info, int time_index) {
    Eigen::VectorXd jac(NUM_STATE + NUM_INPUT);
    double s_ego = X.s, n_ego = X.n, v_ego = X.v;
    double a = U.acceleration, phidot = U.phidot;

    //    double err_s = s_ego - s_ref;
    double err_n = n_ego - 0;
    double err_v = v_ego - desired_speed;

    jac(0) = 0;                            // ds
    jac(1) = 2 * weight_n_offset * err_n;  // dn
    jac(2) = 0;                            // dalpha

    // 添加与障碍物的距离的Jacobian
    Eigen::VectorXd dis_to_obs_cost_jacobian = disToObsCostJacobian(X, obs_list_info, time_index);
    jac(3) = weight_distance_obstacle * dis_to_obs_cost_jacobian(0);  // dx
    jac(4) = weight_distance_obstacle * dis_to_obs_cost_jacobian(1);  // dy

    if (err_v > 0) {  // d(v)
        jac(5) = 2 * weight_positive_velocity * err_v;
    } else {
        jac(5) = 2 * weight_negative_velocity * err_v;
    }

    jac(6) = 0;  // d(phi)

    jac(7) = 2 * weight_acceleration * a;  // d(acc)
    jac(8) = 2 * weight_phidot * phidot;   // d(delta)

    //    ADEBUG << "Cost Jacobian: " << jac.transpose();

    return jac;
}

Eigen::MatrixXd Cost::calHessian(const State &X, const Input &U, const double desired_speed, const std::vector<std::pair<ObstacleType, TrajectoryPoint>> &obs_list_info, int time_index) {
    double s_ego = X.s, n_ego = X.n, v_ego = X.v;
    double err_v = v_ego - desired_speed;

    Eigen::MatrixXd hess(NUM_STATE + NUM_INPUT, NUM_STATE + NUM_INPUT);
    hess.setZero();

    hess(0, 0) = 0;                    // dsds
    hess(1, 1) = 2 * weight_n_offset;  // dndn
    hess(2, 2) = 0;                    // d(alpha)d(alpha)

    // 添加与障碍物的距离的Hessian
    Eigen::MatrixXd dis_to_obs_cost_hessian = disToObsCostHessian(X, obs_list_info, time_index);
    hess(3, 3) = weight_distance_obstacle * dis_to_obs_cost_hessian(0, 0);  // dxdx
    hess(4, 4) = weight_distance_obstacle * dis_to_obs_cost_hessian(1, 1);  // dydy
    hess(3, 4) = weight_distance_obstacle * dis_to_obs_cost_hessian(0, 1);  // dxdy
    hess(4, 3) = weight_distance_obstacle * dis_to_obs_cost_hessian(1, 0);  // dydx

    if (err_v > 0) {
        hess(5, 5) = 2 * weight_positive_velocity;
    } else {
        hess(5, 5) = 2 * weight_negative_velocity;
    }

    hess(6, 6) = 0;  // d(phi)d(phi)

    hess(7, 7) = 2 * weight_acceleration;  // d(acc)d(acc)
    hess(8, 8) = 2 * weight_phidot;        // d(phidot)d(phidot)

    //    ADEBUG << "Cost hessian: \n"
    //           << hess.transpose();
    return hess;
}

double Cost::calDisToObs(const EgoVehicleInfo &ego, ObstacleType type, const TrajectoryPoint &obs, int time) {
    double xobs = obs.x, yobs = obs.y, phiobs = obs.phi, vobs = obs.v;
    double xego = ego.x, yego = ego.y, phiego = ego.phi, vego = ego.v;

    double dx = xego - xobs;
    double dy = yego - yobs;
    double cos_phi = cos(phiobs);
    double sin_phi = sin(phiobs);

    // ellipse param
    double a;
    double b;

    if (type == ObstacleType::DYNAMIC_VEHICLE) {
        //  a = l + v * t_safe + s_safe + ego_r
        a = VEHICLE_LENGTH + fabs(vobs * cos(phiobs)) * DEFAULT_T_SAFE + DEFAULT_S_SAFE;
        //  b = w + s_safe + ego_r
        b = VEHICLE_WIDTH + DEFAULT_S_SAFE;
    } else if (type == ObstacleType::PEDESTRIAN) {
        // TODO :: pedestrian ellipse param
        a = PEDESTRIAN_ENVELOPE_ELLIPSE_PARAM_A;
        b = PEDESTRIAN_ENVELOPE_ELLIPSE_PARAM_B;
    }

    double asquare = pow(a, 2);
    double bsquare = pow(b, 2);

    double A = dy * cos_phi + dx * sin_phi;
    double B = dx * cos_phi - dy * sin_phi;

    //    double cost = Power(Power((-yego + yobs) * Cos(phiobs) + (-xego + xobs) * Sin(phiobs), 2) / bsquare + Power((-xego + xobs) * Cos(phiobs) - (-yego + yobs) * Sin(phiobs), 2) / asquare, -2);

    double cost = 1 / (Power(Exp, time / gama) * Power(Power((-yego + yobs) * Cos(phiobs) + (-xego + xobs) * Sin(phiobs), 2) / bsquare +
                                                           Power((-xego + xobs) * Cos(phiobs) - (-yego + yobs) * Sin(phiobs), 2) / asquare,
                                                       2));
    return cost;
}

Eigen::Vector2d Cost::calDisToObsGrad(const EgoVehicleInfo &ego, ObstacleType type, const TrajectoryPoint &obs, int time) {
    Eigen::Vector2d grad;
    double xobs = obs.x, yobs = obs.y, phiobs = obs.phi, vobs = obs.v;
    double xego = ego.x, yego = ego.y, phiego = ego.phi, vego = ego.v;

    double dx = xego - xobs;
    double dy = yego - yobs;
    double cos_phi = cos(phiobs);
    double sin_phi = sin(phiobs);

    // ellipse param
    double a;
    double b;

    if (type == ObstacleType::DYNAMIC_VEHICLE) {
        //  a = l + v * t_safe + s_safe + ego_r
        a = VEHICLE_LENGTH + fabs(vobs * cos(phiobs)) * DEFAULT_T_SAFE + DEFAULT_S_SAFE;
        //  b = w + s_safe + ego_r
        b = VEHICLE_WIDTH + DEFAULT_S_SAFE;
    } else if (type == ObstacleType::PEDESTRIAN) {
        // TODO :: pedestrian ellipse param
        a = PEDESTRIAN_ENVELOPE_ELLIPSE_PARAM_A;
        b = PEDESTRIAN_ENVELOPE_ELLIPSE_PARAM_B;
    }

    double asquare = pow(a, 2);
    double bsquare = pow(b, 2);

    double A = Power(dy * cos_phi + dx * sin_phi, 2) / bsquare;
    double B = Power(dx * cos_phi - dy * sin_phi, 2) / asquare;
    double denominator = asquare * bsquare * Power(A + B, 3);

    //    double grad_x = -4 * (bsquare * dx * Power(cos_phi, 2) + (asquare - bsquare) * dy * cos_phi * sin_phi + asquare * dx * Power(sin_phi, 2)) / denominator;
    //    double grad_y = -4 * (asquare * dy * Power(cos_phi, 2) + (asquare - bsquare) * dx * cos_phi * sin_phi + bsquare * dy * Power(sin_phi, 2)) / denominator;

    double grad_x = (-4 * (bsquare * (xego - xobs) * Power(Cos(phiobs), 2) + (asquare - bsquare) * (yego - yobs) * Cos(phiobs) * Sin(phiobs) + asquare * (xego - xobs) * Power(Sin(phiobs), 2))) /
                    (asquare * bsquare * Power(Exp, time / gama) * Power(Power((-yego + yobs) * Cos(phiobs) + (-xego + xobs) * Sin(phiobs), 2) / bsquare + Power((xego - xobs) * Cos(phiobs) + (-yego + yobs) * Sin(phiobs), 2) / asquare, 3));

    double grad_y = (-4 * (asquare * (yego - yobs) * Power(Cos(phiobs), 2) + (asquare - bsquare) * (xego - xobs) * Cos(phiobs) * Sin(phiobs) + bsquare * (yego - yobs) * Power(Sin(phiobs), 2))) /
                    (asquare * bsquare * Power(Exp, time / gama) * Power(Power((-yego + yobs) * Cos(phiobs) + (-xego + xobs) * Sin(phiobs), 2) / bsquare + Power((xego - xobs) * Cos(phiobs) + (-yego + yobs) * Sin(phiobs), 2) / asquare, 3));

    grad(0) = grad_x;
    grad(1) = grad_y;

    return grad;
}

Eigen::Matrix2d Cost::calDisToObsCostHessian(const EgoVehicleInfo &ego, ObstacleType type, const TrajectoryPoint &obs, int time) {
    Eigen::Matrix2d hessian;

    double xobs = obs.x, yobs = obs.y, phiobs = obs.phi, vobs = obs.v;
    double xego = ego.x, yego = ego.y, phiego = ego.phi, vego = ego.v;

    double dx = xego - xobs;
    double dy = yego - yobs;
    double cos_phi = cos(phiobs);
    double sin_phi = sin(phiobs);

    // ellipse param
    double a;
    double b;

    if (type == ObstacleType::DYNAMIC_VEHICLE) {
        //  a = l + v * t_safe + s_safe + ego_r
        a = VEHICLE_LENGTH + fabs(vobs * cos(phiobs)) * DEFAULT_T_SAFE + DEFAULT_S_SAFE;
        //  b = w + s_safe + ego_r
        b = VEHICLE_WIDTH + DEFAULT_S_SAFE;
    } else if (type == ObstacleType::PEDESTRIAN) {
        // TODO :: pedestrian ellipse param
        a = PEDESTRIAN_ENVELOPE_ELLIPSE_PARAM_A;
        b = PEDESTRIAN_ENVELOPE_ELLIPSE_PARAM_B;
    }

    double asquare = pow(a, 2);
    double bsquare = pow(b, 2);

    //    double common_denominator = Power(asquare, 2) * Power(bsquare, 2) * Power(Power(dy * cos_phi + dx * sin_phi, 2) / bsquare + Power(dx * cos_phi - dy * sin_phi, 2) / asquare, 4);
    //
    //    double term1_xx = bsquare * dx * Power(cos_phi, 2) + (asquare - bsquare) * dy * cos_phi * sin_phi + asquare * dx * Power(sin_phi, 2);
    //    double term1_xy = -bsquare * dx * Power(cos_phi, 2) - (asquare - bsquare) * dy * cos_phi * sin_phi + asquare * -dx * Power(sin_phi, 2);
    //    double term1_yy = asquare * dy * Power(cos_phi, 2) + (asquare - bsquare) * dx * cos_phi * sin_phi + bsquare * dy * Power(sin_phi, 2);
    //
    //    double term2_xx = bsquare * Power(cos_phi, 2) + asquare * Power(sin_phi, 2);
    //    double term2_yy = asquare * Power(cos_phi, 2) + bsquare * Power(sin_phi, 2);
    //
    //    double common_numerator_xx = asquare * Power(dy * cos_phi + dx * sin_phi, 2) + bsquare * Power(dx * cos_phi - dy * sin_phi, 2);
    //    double common_numerator_xy = (asquare - bsquare) * cos_phi * sin_phi * common_numerator_xx;

    //    double hessian_xx = 4 * (6 * Power(term1_xx, 2) - term2_xx * common_numerator_xx) / common_denominator;
    //    double hessian_yy = 4 * (6 * Power(term1_yy, 2) - term2_yy * common_numerator_xx) / common_denominator;
    //    double hessian_xy = 4 * (6 * term1_xy * term1_yy - common_numerator_xy) / common_denominator;

    double hessian_xx = (4 * (6 * Power(bsquare * (xego - xobs) * Power(Cos(phiobs), 2) + (asquare - bsquare) * (yego - yobs) * Cos(phiobs) * Sin(phiobs) + asquare * (xego - xobs) * Power(Sin(phiobs), 2), 2) -
                              (bsquare * Power(Cos(phiobs), 2) + asquare * Power(Sin(phiobs), 2)) *
                                  (asquare * Power((-yego + yobs) * Cos(phiobs) + (-xego + xobs) * Sin(phiobs), 2) + bsquare * Power((xego - xobs) * Cos(phiobs) + (-yego + yobs) * Sin(phiobs), 2)))) /
                        (Power(asquare, 2) * Power(bsquare, 2) * Power(Exp, time / gama) * Power(Power((-yego + yobs) * Cos(phiobs) + (-xego + xobs) * Sin(phiobs), 2) / bsquare + Power((xego - xobs) * Cos(phiobs) + (-yego + yobs) * Sin(phiobs), 2) / asquare, 4));

    double hessian_yy = (4 * (6 * Power(asquare * (yego - yobs) * Power(Cos(phiobs), 2) + (asquare - bsquare) * (xego - xobs) * Cos(phiobs) * Sin(phiobs) + bsquare * (yego - yobs) * Power(Sin(phiobs), 2), 2) -
                              (asquare * Power(Cos(phiobs), 2) + bsquare * Power(Sin(phiobs), 2)) *
                                  (asquare * Power((-yego + yobs) * Cos(phiobs) + (-xego + xobs) * Sin(phiobs), 2) + bsquare * Power((xego - xobs) * Cos(phiobs) + (-yego + yobs) * Sin(phiobs), 2)))) /
                        (Power(asquare, 2) * Power(bsquare, 2) * Power(Exp, time / gama) * Power(Power((-yego + yobs) * Cos(phiobs) + (-xego + xobs) * Sin(phiobs), 2) / bsquare + Power((xego - xobs) * Cos(phiobs) + (-yego + yobs) * Sin(phiobs), 2) / asquare, 4));

    double hessian_xy = (4 * (6 * (-(bsquare * (xego - xobs) * Power(Cos(phiobs), 2)) - (asquare - bsquare) * (yego - yobs) * Cos(phiobs) * Sin(phiobs) + asquare * (-xego + xobs) * Power(Sin(phiobs), 2)) *
                                  (-(asquare * (yego - yobs) * Power(Cos(phiobs), 2)) - (asquare - bsquare) * (xego - xobs) * Cos(phiobs) * Sin(phiobs) + bsquare * (-yego + yobs) * Power(Sin(phiobs), 2)) -
                              (asquare - bsquare) * Cos(phiobs) * Sin(phiobs) * (asquare * Power((-yego + yobs) * Cos(phiobs) + (-xego + xobs) * Sin(phiobs), 2) + bsquare * Power((xego - xobs) * Cos(phiobs) + (-yego + yobs) * Sin(phiobs), 2)))) /
                        (Power(asquare, 2) * Power(bsquare, 2) * Power(Exp, time / gama) * Power(Power((-yego + yobs) * Cos(phiobs) + (-xego + xobs) * Sin(phiobs), 2) / bsquare + Power((xego - xobs) * Cos(phiobs) + (-yego + yobs) * Sin(phiobs), 2) / asquare, 4));

    hessian(0, 0) = hessian_xx;
    hessian(1, 1) = hessian_yy;
    hessian(0, 1) = hessian_xy;
    hessian(1, 0) = hessian_xy;

    return hessian;
}

double Cost::disToObsCost(const State &X, const std::vector<std::pair<ObstacleType, TrajectoryPoint>> &obs_list_info, int time) {
    double cost = 0.0;
    double xego = X.x, yego = X.y, phiego = X.phi, vego = X.v;
    EgoVehicleInfo ego{xego, yego, vego, phiego};

    for (const auto &obs : obs_list_info) {
        //        obs.print_point();
        cost += calDisToObs(ego, obs.first, obs.second, time);
    }

    return cost;
}

Eigen::Vector2d Cost::disToObsCostJacobian(const State &X, const std::vector<std::pair<ObstacleType, TrajectoryPoint>> &obs_list_info, int time) {
    Eigen::Vector2d jac_obs(0, 0);

    double xego = X.x, yego = X.y, vego = X.v, phiego = X.phi;
    EgoVehicleInfo ego{xego, yego, vego, phiego};

    for (const auto &obs : obs_list_info) {
        jac_obs += calDisToObsGrad(ego, obs.first, obs.second, time);
    }

    return jac_obs;
}

Eigen::Matrix2d Cost::disToObsCostHessian(const State &X, const std::vector<std::pair<ObstacleType, TrajectoryPoint>> &obs_list_info, int time) {
    Eigen::Matrix2d hessian_obs;
    hessian_obs.setZero();

    double xego = X.x, yego = X.y, vego = X.v, phiego = X.phi;
    EgoVehicleInfo ego{xego, yego, vego, phiego};

    for (const auto &obs : obs_list_info) {
        hessian_obs += calDisToObsCostHessian(ego, obs.first, obs.second, time);
    }

    return hessian_obs;
}