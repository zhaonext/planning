#pragma once
#include <cmath>
#include <iostream>
#include <vector>

#include "Common/log.h"
#include "Common/util.h"
#include "PathSegment/path.h"
#include "PathSegment/vec2d.h"
#include "TrafficRules/traffic_light.h"
#include "TrafficRules/util.h"

class CrosswalkDecision {
public:
    CrosswalkDecision() = default;

    static std::pair<double, bool> CalculateCrosswalkSafetySpeed(const EgoVehicleInfo& ego,
                                                                 const ObstacleInfo& obstacles) {
        double minSafeSpeed = DESIRED_SPEED;
        bool buildWall = false;  // 是否需要停车
        int triggeringObstacleId = -1;

        double adjustedSpeed = DESIRED_SPEED;

        if (obstacles.type == ObstacleType::PEDESTRIAN) {
            // 对行人进行速度调整
            adjustedSpeed = adjustSpeedForPedestrian(ego, obstacles);
            buildWall = adjustedSpeed == 0.0;  // 如果需要完全停车，则建立障碍墙
        }

        if (adjustedSpeed < minSafeSpeed) {
            minSafeSpeed = adjustedSpeed;
            triggeringObstacleId = obstacles.id;
        }

        if (triggeringObstacleId != -1) {
            AWARN << "Calculated safe speed for crosswalk = " << minSafeSpeed * 3.6 << " km/h, due to obstacle ID " << triggeringObstacleId;
        }

        return {minSafeSpeed, buildWall};
    }

private:
    static double adjustSpeedForPedestrian(const EgoVehicleInfo& ego, const ObstacleInfo& obs) {
        const double LATERAL_SAFETY_DISTANCE = LANE_WIDTH * 1.5 + ego.v * 0.1;  // 横向安全距离

        // 使用toLocalCoordinates函数计算障碍物的局部坐标
        auto [localX, localY] = toLocalCoordinates(ego, obs);

        // 仅考虑自车前方一定范围内的行人
        if (localX <= 0 || std::fabs(localY) > LATERAL_SAFETY_DISTANCE) {
            return DESIRED_SPEED;
        }

        // 如果障碍物在刹车距离之内，开始减速
        const double sigmoidSteepness = -0.2;  // Sigmoid 曲线的陡峭程度,绝对值越小越平缓
        double speed = ego.v * 3.6;

        double someFactor;
        if (speed <= 10.0) {
            someFactor = 0.2;
        } else if (speed <= 20.0) {
            someFactor = 0.4;
        } else if (speed <= 30.0) {
            someFactor = 0.6;
        } else if (speed <= 40.0) {
            someFactor = 0.8;
        } else if (speed <= 50.0) {
            someFactor = 1.0;
        } else if (speed <= 60.0) {
            someFactor = 1.4;
        } else if (speed <= 70.0) {
            someFactor = 1.2;
        } else {
            someFactor = 2.0;
        }

        const double sigmoidMidpoint = localX * someFactor;  // Sigmoid 曲线的对称点,理解为一旦在对称点右边，惩罚因子将会快速逼近与1

        // 使用 Sigmoid 函数计算减速因子
        double decelerationFactor = 1.0 / (1.0 + exp(sigmoidSteepness * (localX - sigmoidMidpoint)));

        // 计算调整后的速度
        double adjustedSpeed = ego.v * decelerationFactor;

        return adjustedSpeed;
    }
};