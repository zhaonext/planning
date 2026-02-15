#pragma once

#include <cmath>
#include <iostream>
#include <utility>

#include "Common/common.h"

class ObstacleVehiclePredictiveModel {
public:
    explicit ObstacleVehiclePredictiveModel(ObstacleInfo obstacle_) : obstacle_(std::move(obstacle_)) {}

    Trajectory predict_trajectory(double horizon, double time_step) {
        if (horizon < 0 || time_step <= 0) {
            AERROR << "Error: Invalid horizon or time_step values.";
            return {};  // 返回空轨迹
        }

        Trajectory trajectory;
        for (int i = 0; i <= horizon; ++i) {
            if (!predict(time_step)) {
                AERROR << "Prediction failed at step " << i << ".";
                break;
            }
            TrajectoryPoint point{obstacle_.x, obstacle_.y, obstacle_.t, obstacle_.v, obstacle_.phi};
            trajectory.points.push_back(point);
        }
        return trajectory;
    }

private:
    bool predict(double time_step) {
        if (time_step <= 0) {
            AERROR << "Error: time_step should be a positive value.";
            return false;
        }

        switch (obstacle_.type) {
            case DYNAMIC_VEHICLE:
                return predict_dynamic_vehicle(time_step);
            case PEDESTRIAN:
            case BICYCLE:
                return predict_pedestrian_or_bicycle(time_step);
            case STATIC_VEHICLE:
                return predict_static(time_step);
            case TYPE_UNKNOWN:
                // 对于未知类型，可能需要一个默认的行为或者直接返回错误
                AERROR << "Unknown obstacle_ type.";
                return false;
            default:
                AERROR << "Unhandled obstacle_ type.";
                return false;
        }
    }

    bool predict_dynamic_vehicle(double time_step, bool use_ctrv = false, bool use_ctra = true) {
        if (time_step <= 0) {
            AERROR << "Error: time_step should be a positive value in dynamic vehicle prediction.";
            return false;
        }

        const double epsilon = 1e-5;
        if (std::fabs(obstacle_.psi_dot) < epsilon) {
            update_linear_motion(time_step);
        } else {
            if (use_ctrv && !use_ctra) {
                // 使用CTRV模型
                update_ctrv_motion(time_step);
            } else if (use_ctra) {
                // 使用CTRA模型
                update_ctra_motion(time_step);
            } else {
                AERROR << "Error: No motion model selected for dynamic vehicle with non-zero psi_dot.";
                return false;
            }
        }

        obstacle_.t += time_step;
        return true;
    }

    bool predict_pedestrian_or_bicycle(double time_step) {
        // 行人和自行车通常没有显著的加速度和转向，可以使用简单的线性模型
        if (time_step <= 0) {
            AERROR << "Error: time_step should be a positive value for pedestrian or bicycle prediction.";
            return false;
        }
        update_linear_motion(time_step);
        obstacle_.t += time_step;
        return true;
    }

    bool predict_static(double time_step) {
        // 静态障碍物不移动，不需要更新位置，但是仍然要更新时间，以保持时间的连续性
        if (time_step <= 0) {
            AERROR << "Error: time_step should be a positive value for static prediction.";
            return false;
        }
        obstacle_.t += time_step;
        return true;
    }

    // model
    void update_linear_motion(double time_step) {
        obstacle_.x += obstacle_.v * std::cos(obstacle_.phi) * time_step;
        obstacle_.y += obstacle_.v * std::sin(obstacle_.phi) * time_step;
        obstacle_.v += obstacle_.a * time_step;
    }
    void update_ctrv_motion(double time_step) {
        double R = obstacle_.v / obstacle_.psi_dot;
        double beta = obstacle_.psi_dot * time_step;
        obstacle_.x += R * (std::sin(obstacle_.phi + beta) - std::sin(obstacle_.phi));
        obstacle_.y += R * (-std::cos(obstacle_.phi + beta) + std::cos(obstacle_.phi));
        obstacle_.phi += beta;
        // 注意：CTRV模型不考虑加速度
    }
    void update_ctra_motion(double time_step) {
        double R = obstacle_.v / obstacle_.psi_dot;
        double beta = obstacle_.psi_dot * time_step;
        obstacle_.x += R * (std::sin(obstacle_.phi + beta) - std::sin(obstacle_.phi)) + 0.5 * obstacle_.a * time_step * time_step * std::cos(obstacle_.phi + beta / 2.0);
        obstacle_.y += R * (-std::cos(obstacle_.phi + beta) + std::cos(obstacle_.phi)) + 0.5 * obstacle_.a * time_step * time_step * std::sin(obstacle_.phi + beta / 2.0);
        obstacle_.v += obstacle_.a * time_step;
        obstacle_.phi += beta;
    }

private:
    ObstacleInfo obstacle_;
};