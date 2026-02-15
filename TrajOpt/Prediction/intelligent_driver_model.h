#pragma once

#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>

#include "Common/common.h"

class IntelligentDriverModel {
public:
    struct IDMState {
        double x;  // 自车横向位置，单位：m
        double y;  // 自车纵向位置，单位：m
        double phi;
        double v;        // 自车速度，单位：m/s
        double x_front;  // 前车横向位置，单位：m
        double y_front;  // 前车纵向位置，单位：m
        double phi_front;
        double v_front;  // 前车速度，单位：m/s
    };

    struct IDMParam {
        double desired_velocity = DESIRED_SPEED;        // 期望速度，单位：m/s
        double minimum_spacing = 5.0;                   // 最小间隔，单位：m
        double desired_headway = 1.0;                   // 期望车头时距，单位：s
        double max_acceleration = 2.0;                  // 最大加速度，单位：m/s^2
        double comfortable_braking_deceleration = 3.0;  // 舒适减速度，单位：m/s^2
        double hard_braking_deceleration = 5.0;         // 紧急减速度，单位：m/s^2
        int exponent = 3;                               // 加速度指数
    };

    /**
     * @brief 计算IDM期望加速度的方法
     * @param param
     * @param state
     * @return
     */
    static std::pair<double, double> calIdmDesiredAccelerationAndVelocity(const IDMParam &param, const IDMState &state, const double dt) {
        if (dt < 0.0) {
            AERROR << "Invalid timestep: dt must be non-negative.";
            return {0.0, state.v};
        }

        const double DISTANCE_THRESHOLD = 100.0;  // Safe distance threshold in meters
        const double LATERAL_THRESHOLD = MAXIMUM_LATERAL_OFFSET / 2 + VEHICLE_WIDTH / 2;

        const double dx = state.x_front - state.x;
        const double dy = state.y_front - state.y;
        const double x_relative = (dx * cos(state.phi) + dy * sin(state.phi)) - (state.v_front * cos(state.phi_front) * dt);
        const double y_relative = (-dx * sin(state.phi) + dy * cos(state.phi)) - (state.v_front * sin(state.phi_front) * dt);

        const bool in_front = x_relative > 0 && x_relative <= DISTANCE_THRESHOLD;
        const bool on_side = std::abs(y_relative) <= LATERAL_THRESHOLD;

        if (!in_front || !on_side) {
            // 当前方没有障碍物或障碍物距离超过阈值时，逐渐加速到期望速度
            double acceleration_to_desired_velocity = (param.desired_velocity - state.v) / param.desired_headway;
            acceleration_to_desired_velocity = std::min(acceleration_to_desired_velocity, param.max_acceleration);
            const double new_velocity = std::max(0.0, state.v + acceleration_to_desired_velocity * dt);
            return {acceleration_to_desired_velocity, new_velocity};
        }

        const double safe_distance = param.minimum_spacing + state.v * param.desired_headway +
                                     state.v * (state.v - state.v_front) / (2 * std::sqrt(param.max_acceleration * param.comfortable_braking_deceleration));
        const double s = x_relative;

        double acceleration = param.max_acceleration * (1 - std::pow(state.v / param.desired_velocity, param.exponent) - std::pow(safe_distance / s, 2));
        acceleration = std::min(acceleration, param.max_acceleration);
        acceleration = std::max(acceleration, -param.hard_braking_deceleration);

        const double new_velocity = std::max(0.0, state.v + acceleration * dt);

        return {acceleration, new_velocity};
    }
};
