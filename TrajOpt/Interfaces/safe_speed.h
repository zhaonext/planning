#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <vector>

#include "Common/common.h"
#include "Common/util.h"
#include "TrafficRules/crosswalk.h"
#include "TrafficRules/traffic_light.h"
#include "TrafficRules/vehicle_speed_decider.h"

class SafetySpeed {
public:
    SafetySpeed() = default;

    static std::pair<double, bool> calSafeSpeed(const EgoVehicleInfo& ego,
                                                const std::vector<ObstacleInfo>& obstacles,
                                                const std::shared_ptr<Path>& reference_line_info,
                                                const Vec2d& stop_line,
                                                const TrafficLightInfo& traffic_light_info,
                                                const std::string& driving_type) {
        const double DISTANCE_THRESHOLD = 100.0;
        const double LATERAL_THRESHOLD = LANE_WIDTH + VEHICLE_WIDTH * 0.5 + ego.v * 0.1;
        double minSafeSpeed = DESIRED_SPEED;
        bool buildWall = false;
        int triggeringObstacleId = -1;

        auto [localX, localY] = toLocalCoordinates(ego, stop_line);

        // turn left || turn right
        if (driving_type == "turn_left" || driving_type == "turn_right") {
            const double MIN_SAFE_SPEED_TURN = 20 / 3.6;
            const double SPEED_ADJUSTMENT_FACTOR_TURN = 0.65;
            if (localX > 0 && localX < 50) {
                minSafeSpeed = std::max(ego.v * SPEED_ADJUSTMENT_FACTOR_TURN, MIN_SAFE_SPEED_TURN);
            }
        }

        // 交通灯逻辑
        const double kApproachThreshold = 100.0;
        TrafficLightState traffic_light_state = traffic_light_info.getState();
        if (traffic_light_state == TrafficLightState::RED || traffic_light_state == TrafficLightState::YELLOW) {
            if (localX > 0 && localX < kApproachThreshold) {
                auto [adjustedSpeed, stopRequired] = TrafficLightDecision::MakeDecisions(ego, reference_line_info, stop_line, traffic_light_info);
                minSafeSpeed = std::min(minSafeSpeed, adjustedSpeed);
                buildWall = stopRequired;
                if (stopRequired) {
                    AWARN << "Red light and close to stop line, adjusted speed = " << minSafeSpeed << ", build wall = true";
                    return {minSafeSpeed, true};
                }
            }
        }
        // 障碍物逻辑
        for (const auto& obs : obstacles) {
            auto [localX, localY] = toLocalCoordinates(ego, obs);

            if (std::fabs(localY) >= LATERAL_THRESHOLD || localX <= 0 || localX >= DISTANCE_THRESHOLD) {
                continue;
            }

            double adjustedSpeed = DESIRED_SPEED;
            switch (obs.type) {
                case ObstacleType::PEDESTRIAN: {
                    auto [AdjustedSpeed, BuildWall] = CrosswalkDecision::CalculateCrosswalkSafetySpeed(ego, obs);
                    adjustedSpeed = AdjustedSpeed;
                    buildWall = BuildWall;
                    break;
                }
                case ObstacleType::DYNAMIC_VEHICLE:
                    adjustedSpeed = ObstacleVehicleSpeedDecision::CalculateDynamicVehicleSafetySpeed(localX, ego.v, obs.v);
                    break;
                case ObstacleType::STATIC_VEHICLE:
                    adjustedSpeed = ObstacleVehicleSpeedDecision::CalculateStaticObstacleSafetySpeed(localX, ego.v);
                    break;
            }

            if (adjustedSpeed < minSafeSpeed) {
                minSafeSpeed = adjustedSpeed;
                triggeringObstacleId = obs.id;
            }
        }

        if (triggeringObstacleId != -1) {
            AWARN << "Calculated safe speed = " << minSafeSpeed * 3.6 << " km/h, due to obstacle ID " << triggeringObstacleId;
        }

        return {minSafeSpeed, buildWall};
    }
};