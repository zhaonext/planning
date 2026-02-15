#pragma once
#include <cmath>
#include <functional>
#include <vector>

#include "Common/common.h"
#include "Eigen/Eigen"

template <int NDISK>
class CollisionRiskEvaluator {
public:
    using CollisionFunction = std::function<void(double* val, const double* x, const double* u)>;
    using CollisionJacobian = std::function<void(double* jac, const double* x, const double* u)>;

    static CollisionFunction CreateCollisionFunction(const std::vector<std::pair<ObstacleType, TrajectoryPoint>>& obs_list_info) {
        return CollisionFunction{[obs_list_info](double* val, const double* x, const double* u) -> void {
            const State state = pointerToState(x);
            calCollisionFunction(val, state, obs_list_info);
        }};
    }

    static CollisionJacobian CreateCollisionJacobian(const std::vector<std::pair<ObstacleType, TrajectoryPoint>>& obs_list_info) {
        return CollisionJacobian{[obs_list_info](double* jac, const double* x, const double* u) -> void {
            const State state = pointerToState(x);
            calCollisionJacobian(jac, state, obs_list_info);
        }};
    }

    explicit CollisionRiskEvaluator() = default;

private:
    static void calCollisionFunction(double* val, const State& X, const std::vector<std::pair<ObstacleType, TrajectoryPoint>>& obs_list_info) {
        const double xego = X.x, yego = X.y, phiego = X.phi;
        const double common_term = VEHICLE_LENGTH / (2 * NDISK);
        for (const auto& obs_info : obs_list_info) {
            auto type = obs_info.first;
            auto obs_point = obs_info.second;
            const double xobs = obs_point.x, yobs = obs_point.y, phiobs = obs_point.phi, vobs = obs_point.v;

            // ego disc radius
            // double EGO_DISC_RADIUS = pow(pow(VEHICLE_LENGTH / (2 * NUM_EGO_ENVELOPE_CIRCLES), 2) + pow(VEHICLE_WIDTH / 2, 2), 0.5);

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

            for (int i = 0; i < NDISK; ++i) {
                double factor = (2 * i + 1) * common_term - REAR_SUSPENSION;
                val[i] += 1 - Power((xego - xobs) * Cos(phiobs) + factor * Cos(phiego + phiobs) + (-yego + yobs) * Sin(phiobs), 2) / asquare - Power((yego - yobs) * Cos(phiobs) + (xego - xobs) * Sin(phiobs) + factor * Sin(phiego + phiobs), 2) / bsquare;
            }
        }
    }

    static void calCollisionJacobian(double* jac, const State& X, const std::vector<std::pair<ObstacleType, TrajectoryPoint>>& obs_list_info) {
        double xego = X.x, yego = X.y, phiego = X.phi;
        const double common_term = VEHICLE_LENGTH / (2 * NDISK);
        Eigen::Map<Eigen::Matrix<double, NDISK, NUM_STATE + NUM_INPUT>> J(jac);
        J.setZero();
        for (const auto& obs_info : obs_list_info) {
            auto type = obs_info.first;
            auto obs_point = obs_info.second;
            const double xobs = obs_point.x, yobs = obs_point.y, phiobs = obs_point.phi, vobs = obs_point.v;

            // ego disc radius
            double EGO_DISC_RADIUS = pow(pow(VEHICLE_LENGTH / (2 * NUM_EGO_ENVELOPE_CIRCLES), 2) + pow(VEHICLE_WIDTH / 2, 2), 0.5);

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

            for (int i = 0; i < NDISK; ++i) {
                double factor = (2 * i + 1) * common_term - REAR_SUSPENSION;
                // d(collision) / dx
                J(i, 3) += (-2 * (bsquare * (xego - xobs + factor * Cos(phiego)) * Power(Cos(phiobs), 2) + (asquare - bsquare) * Cos(phiobs) * (yego - yobs + factor * Sin(phiego)) * Sin(phiobs) + asquare * (xego - xobs + factor * Cos(phiego)) * Power(Sin(phiobs), 2))) / (asquare * bsquare);
                // d(collision) / dy
                J(i, 4) += (-2 * (asquare * Power(Cos(phiobs), 2) * (yego - yobs + factor * Sin(phiego)) + (asquare - bsquare) * (xego - xobs + factor * Cos(phiego)) * Cos(phiobs) * Sin(phiobs) + bsquare * (yego - yobs + factor * Sin(phiego)) * Power(Sin(phiobs), 2))) / (asquare * bsquare);
                // d(collision) / d(phi)
                J(i, 6) += (2 * factor * ((xego - xobs) * Cos(phiobs) + factor * Cos(phiego + phiobs) + (-yego + yobs) * Sin(phiobs)) * Sin(phiego + phiobs)) / asquare - (2 * factor * Cos(phiego + phiobs) * ((yego - yobs) * Cos(phiobs) + (xego - xobs) * Sin(phiobs) + factor * Sin(phiego + phiobs))) / bsquare;
                //            std::cout << "collision jac:" << std::endl;
                //            std::cout << J << std::endl;
            }
        }
    }

    /*
     CollisionFunction collision_function = [this](double* val, const double* x, const double* u) {
        double xobs = obs_.x, yobs = obs_.y, phiobs = obs_.phi, vobs = obs_.velocity;
        double xego = x[3], yego = x[4], phiego = x[5], vego = u[0];

    UpdateEgoStatus(xego, yego, phiego, vego);
    const double common_term = (REAR_SUSPENSION + WHEELBASE + FRONT_SUSPENSION) / (2 * NDISK);
    const double common_radius = sqrt(pow(common_term, 2) + pow(VEHICLE_WIDTH / 2, 2));

    obs_ellipse_.UpdateEllipseParam(vobs, phiobs, common_radius);
    double asquare = pow(obs_ellipse_.GetA(), 2);
    double bsquare = pow(obs_ellipse_.GetB(), 2);

    for (int i = 0; i < NDISK; ++i) {
        double factor = (2 * i + 1) * common_term - REAR_SUSPENSION;
        val[i] = 1 - Power((xego - xobs) * Cos(phiobs) + factor * Cos(phiego + phiobs) + (-yego + yobs) * Sin(phiobs), 2) / asquare - Power((yego - yobs) * Cos(phiobs) + (xego - xobs) * Sin(phiobs) + factor * Sin(phiego + phiobs), 2) / bsquare;
    }
};
CollisionJacobian collision_jacobian = [this](double* jac, const double* x, const double* u) {
    double xobs = obs_.x, yobs = obs_.y, phiobs = obs_.phi, vobs = obs_.velocity;
    double xego = x[3], yego = x[4], phiego = x[5], vego = u[0];

    UpdateEgoStatus(xego, yego, phiego, vego);
    const double common_term = (REAR_SUSPENSION + WHEELBASE + FRONT_SUSPENSION) / (2 * NDISK);
    const double common_radius = sqrt(pow(common_term, 2) + pow(VEHICLE_WIDTH / 2, 2));

    obs_ellipse_.UpdateEllipseParam(vobs, phiobs, common_radius);
    double asquare = pow(obs_ellipse_.GetA(), 2);
    double bsquare = pow(obs_ellipse_.GetB(), 2);

    Eigen::Map<Eigen::Matrix<double, NDISK, NUM_STATE + NUM_INPUT>> J(jac);
    J.setZero();
    for (int i = 0; i < NDISK; ++i) {
        double factor = (2 * i + 1) * common_term - REAR_SUSPENSION;
        // d(collision) / dx
        J(i, 3) = (-2 * (bsquare * (xego - xobs + factor * Cos(phiego)) * Power(Cos(phiobs), 2) + (asquare - bsquare) * Cos(phiobs) * (yego - yobs + factor * Sin(phiego)) * Sin(phiobs) + asquare * (xego - xobs + factor * Cos(phiego)) * Power(Sin(phiobs), 2))) / (asquare * bsquare);
        // d(collision) / dy
        J(i, 4) = (-2 * (asquare * Power(Cos(phiobs), 2) * (yego - yobs + factor * Sin(phiego)) + (asquare - bsquare) * (xego - xobs + factor * Cos(phiego)) * Cos(phiobs) * Sin(phiobs) + bsquare * (yego - yobs + factor * Sin(phiego)) * Power(Sin(phiobs), 2))) / (asquare * bsquare);
        // d(collision) / d(phi)
        J(i, 5) = (2 * factor * ((xego - xobs) * Cos(phiobs) + factor * Cos(phiego + phiobs) + (-yego + yobs) * Sin(phiobs)) * Sin(phiego + phiobs)) / asquare - (2 * factor * Cos(phiego + phiobs) * ((yego - yobs) * Cos(phiobs) + (xego - xobs) * Sin(phiobs) + factor * Sin(phiego + phiobs))) / bsquare;
    }
};*/
};