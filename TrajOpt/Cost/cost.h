#pragma once
#include <Eigen/Dense>
#include <map>

#include "Common/common.h"
#include "Map/curve.h"
#include "Prediction/prediction_trajectory.h"

class Cost {
public:
    using CostFunction = std::function<double(const double *x, const double *u)>;
    using CostJacobian = std::function<void(double *dx, double *du, const double *x, const double *u)>;
    using CostHessian = std::function<void(double *ddx, double *ddu, double *dudx, const double *x, const double *u)>;

    static CostFunction CreateCostFunction(const double desired_speed, const std::vector<std::pair<ObstacleType, TrajectoryPoint>> &obs_list_info, int time_index) {
        return CostFunction{[desired_speed, obs_list_info, time_index](const double *x, const double *u) -> double {
            State state = pointerToState(x);
            Input input = pointerToInput(u);

            return calCostFunction(state, input, desired_speed, obs_list_info, time_index);
        }};
    }
    static CostJacobian CreateCostJacobian(const double desired_speed, const std::vector<std::pair<ObstacleType, TrajectoryPoint>> &obs_list_info, int time_index) {
        return CostJacobian{[desired_speed, obs_list_info, time_index](double *dx, double *du, const double *x, const double *u) -> void {
            State state = pointerToState(x);
            Input input = pointerToInput(u);

            Eigen::VectorXd jac = calJacobian(state, input, desired_speed, obs_list_info, time_index);
            //            ADEBUG << "Jacobian calculation for cost:" << jac.transpose();

            for (int i = 0; i < NUM_STATE; ++i) {
                dx[i] = jac(i);
            }
            for (int i = 0; i < NUM_INPUT; ++i) {
                du[i] = jac(i + NUM_STATE);
            }
        }};
    }
    static CostHessian CreatCostHessian(const double desired_speed, const std::vector<std::pair<ObstacleType, TrajectoryPoint>> &obs_list_info, int time_index) {
        return CostHessian{[desired_speed, obs_list_info, time_index](double *ddx, double *ddu, double *dudx, const double *x, const double *u) -> void {
            State state = pointerToState(x);
            Input input = pointerToInput(u);

            Eigen::MatrixXd hess = calHessian(state, input, desired_speed, obs_list_info, time_index);
            //            ADEBUG << "Hessian calculation for cost: \n"
            //                   << hess;

            // Hxx
            Eigen::Map<Eigen::Matrix<double, NUM_STATE, NUM_STATE>> Hxx(ddx);
            Hxx(0, 0) = hess(0, 0);  // dsds
            Hxx(1, 1) = hess(1, 1);  // dndn
            Hxx(2, 2) = hess(2, 2);  // d(alpha)d(alpha)

            Hxx(3, 3) = hess(3, 3);  // dxdx
            Hxx(4, 4) = hess(4, 4);  // dydy
            Hxx(3, 4) = hess(3, 4);  // dxdy
            Hxx(4, 3) = hess(4, 3);  // dydx

            Hxx(5, 5) = hess(5, 5);  // dvdv
            Hxx(6, 6) = hess(6, 6);  // d(phi)d(phi)

            // Huu
            Eigen::Map<Eigen::Matrix<double, NUM_INPUT, NUM_INPUT>> Huu(ddu);
            Huu.setZero();
            Huu(0, 0) = hess(7, 7);  // d(acc)d(acc)
            Huu(1, 1) = hess(8, 8);  // d(phidot)d(phidot)

            // Hux is zero value
            Eigen::Map<Eigen::Matrix<double, NUM_INPUT, NUM_STATE>> Hux(dudx);
            Hux.setZero();
        }};
    }

public:
    Cost() = default;

private:
    static double calCostFunction(const State &X, const Input &U, double desired_speed, const std::vector<std::pair<ObstacleType, TrajectoryPoint>> &obs_list_info, int time_index);
    static Eigen::VectorXd calJacobian(const State &X, const Input &U, double desired_speed, const std::vector<std::pair<ObstacleType, TrajectoryPoint>> &obs_list_info, int time_index);
    static Eigen::MatrixXd calHessian(const State &X, const Input &U, double desired_speed, const std::vector<std::pair<ObstacleType, TrajectoryPoint>> &obs_list_info, int time_index);

    static double calDisToObs(const EgoVehicleInfo &ego, ObstacleType type, const TrajectoryPoint &obs, int time);
    static Eigen::Vector2d calDisToObsGrad(const EgoVehicleInfo &ego, ObstacleType type, const TrajectoryPoint &obs, int time);
    static Eigen::Matrix2d calDisToObsCostHessian(const EgoVehicleInfo &ego, ObstacleType type, const TrajectoryPoint &obs, int time);
    static Eigen::Vector2d disToObsCostJacobian(const State &X, const std::vector<std::pair<ObstacleType, TrajectoryPoint>> &obs_list_info, int time);
    static double disToObsCost(const State &X, const std::vector<std::pair<ObstacleType, TrajectoryPoint>> &obs_list_info, int time);
    static Eigen::Matrix2d disToObsCostHessian(const State &X, const std::vector<std::pair<ObstacleType, TrajectoryPoint>> &obs_list_info, int time);

    static double decayFactor(int time_step, double lambda = 50) {
        return exp(-static_cast<double>(time_step) / lambda);
    }

private:
    static constexpr double k = 1, z1 = 1, z2 = 5;
    static constexpr double gama = 50;
};
