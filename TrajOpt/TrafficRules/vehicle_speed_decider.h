#pragma once
#include <cmath>

#include "Common/common.h"

class ObstacleVehicleSpeedDecision {
public:
    static double CalculateDynamicVehicleSafetySpeed(double localX, double ego_v, double obs_v) {
        constexpr double DELTA_T = 2.0;                 // 时间差参数
        constexpr double SAFETY_DISTANCE_FACTOR = 2.0;  // 安全距离基准因子
        constexpr double MAX_FACTOR = 3.5;              // 最大调整因子
        constexpr double MIN_FACTOR = -2.0;             // 最小调整因子

        // 安全距离
        double targetSafetyDistance = SAFETY_DISTANCE_FACTOR + ego_v * DELTA_T;

        double L = MAX_FACTOR - MIN_FACTOR;
        double k = 0.1;                    // 曲线的陡峭程度，需要根据实际情况调整
        double x0 = targetSafetyDistance;  // 中点设为目标安全距离

        // 计算速度调整因子
        double factor = L / (1 + exp(-k * (localX - x0))) + MIN_FACTOR;
        return obs_v + factor;
    }

    static double CalculateStaticObstacleSafetySpeed(double distance, double ego_v) {
        const double safetyDistance = 30.0;                              // 安全距离
        const double maxDeceleration = std::fabs(MINIMUM_ACCELERATION);  // 最大减速度，单位：m/s^2

        if (distance < safetyDistance) {
            // 计算减速度，距离越近，减速度越大
            double decelerationFactor = (safetyDistance - distance) / safetyDistance;
            double deceleration = std::min(maxDeceleration * decelerationFactor, ego_v);

            // 调整速度
            double adjustedSpeed = std::max(ego_v - deceleration, 0.0);  // 确保速度不低于0
            return adjustedSpeed;
        }

        return DESIRED_SPEED;  // 如果距离足够远，保持期望速度
    }
};
