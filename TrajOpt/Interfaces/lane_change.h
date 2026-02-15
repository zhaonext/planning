#pragma once
#include "Common/common.h"

class LaneChangeDecider {
private:
    std::chrono::steady_clock::time_point lastLaneChangeTime;                 // 上一次换道的时间点
    const std::chrono::seconds laneChangeCooldown = std::chrono::seconds(5);  // 定义换道冷却时间，例如30秒

public:
    LaneChangeDecider() {
        lastLaneChangeTime = std::chrono::steady_clock::now();  // 初始化上次换道时间为一个过去的时间点
    }

    // 检查是否超过了冷却时间
    bool isCooldownElapsed() {
        return (std::chrono::steady_clock::now() - lastLaneChangeTime) > laneChangeCooldown;
    }

    // 更新上次换道时间
    void updateLaneChangeTime() {
        lastLaneChangeTime = std::chrono::steady_clock::now();
    }

    std::tuple<bool, bool, bool> shouldChangeLane(const EgoVehicleInfo& ego, const std::vector<ObstacleInfo>& obstacles, int currentLaneIndex, int totalLanes) {
        if (!isCooldownElapsed()) {
            return {false, false, false};  // 如果还在冷却时间内，保持当前车道
        }
        auto decision = analyzeObstacles(ego, obstacles, currentLaneIndex, totalLanes);
        // 如果决定换道，则更新换道时间
        if (std::get<1>(decision) || std::get<2>(decision)) {
            updateLaneChangeTime();
        }
        return decision;
    }

private:
    static std::tuple<bool, bool, bool> analyzeObstacles(const EgoVehicleInfo& ego, const std::vector<ObstacleInfo>& obstacles, int currentLaneIndex, int totalLanes) {
        double slow_speed_threshold = LANE_SPEED_LIMIT * 0.3;
        double forward_threshold = 50.0;
        double lateral_threshold = LANE_WIDTH + VEHICLE_WIDTH * 0.3;

        bool slowVehicleAhead = false;
        bool safeToChangeLeft = currentLaneIndex + 1 < totalLanes;
        bool safeToChangeRight = currentLaneIndex - 1 >= 0;

        // 用于跟踪最近障碍物的变量
        double nearestObstacleDistance = forward_threshold;
        int nearestObstacleId = -1;

        for (const auto& obs : obstacles) {
            auto [localX, localY] = toLocalCoordinates(ego, obs);

            // 寻找最近的障碍物
            if (localX > 0 && localX < nearestObstacleDistance) {
                nearestObstacleDistance = localX;
                nearestObstacleId = obs.id;
                slowVehicleAhead = obs.v < slow_speed_threshold;

                // 判断障碍物是否在左侧或右侧车道
                if (localY < 0 && std::fabs(localY) < lateral_threshold && safeToChangeLeft) {
                    safeToChangeRight = false;
                } else if (localY > 0 && localY < lateral_threshold && safeToChangeRight) {
                    safeToChangeLeft = false;
                }
            }
        }

        // 如果左右都可以换道，优先向左换道
        if (slowVehicleAhead && safeToChangeLeft && safeToChangeRight) {
            safeToChangeRight = false;
        }
        if (nearestObstacleId != -1) {
            if (safeToChangeLeft) {
                AWARN << "Left lane change triggered by obstacle ID: " << nearestObstacleId;
            }
            if (safeToChangeRight) {
                AWARN << "Right lane change triggered by obstacle ID: " << nearestObstacleId;
            }
        }

        return {slowVehicleAhead, safeToChangeLeft, safeToChangeRight};
    }

    // 将全局坐标转换为车辆局部坐标
    static std::pair<double, double> toLocalCoordinates(const EgoVehicleInfo& ego, const ObstacleInfo& obs) {
        double dx = obs.x - ego.x;
        double dy = obs.y - ego.y;
        double localX = dx * std::cos(-ego.phi) - dy * std::sin(-ego.phi);
        double localY = dx * std::sin(-ego.phi) + dy * std::cos(-ego.phi);
        return {localX, localY};
    }
};
