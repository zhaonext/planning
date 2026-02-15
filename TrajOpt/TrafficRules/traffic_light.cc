#include "traffic_light.h"

#include <cmath>
#include <iostream>

#include "Common/log.h"
#include "Common/util.h"
#include "TrafficRules/util.h"

std::pair<double, bool> TrafficLightDecision::MakeDecisions(const EgoVehicleInfo& ego_state,
                                                            const std::shared_ptr<Path>& reference_line_info,
                                                            const Vec2d& stop_line,
                                                            const TrafficLightInfo& traffic_light_info) {
    static constexpr double kBaseApproachThreshold = 20.0;  // 基础阈值
    static constexpr double kMaxApproachThreshold = 80.0;   // 最大阈值
    double someFactor;                                      // 根据速度选择someFactor

    double speed = ego_state.v * 3.6;
    AINFO << "Current speed (km/h): " << speed;  // 输出当前速度

    // 选择someFactor
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
        someFactor = 1.2;
    } else {
        someFactor = 1.5;
    }
    AWARN << "Selected someFactor: " << someFactor;  // 输出选定的someFactor

    // 计算动态阈值
    double dynamicApproachThreshold = std::min(kBaseApproachThreshold + ego_state.v * someFactor, kMaxApproachThreshold);
    AWARN << "Dynamic approach threshold: " << dynamicApproachThreshold;  // 输出动态阈值

    TrafficLightState traffic_light_state = traffic_light_info.getState();
    auto adc_front_edge_s = ego_state.center_to_front_edge();

    auto [localX, localY] = toLocalCoordinates(ego_state, stop_line);

    // 如果接近停车线
    if (localX > 0 && localX <= dynamicApproachThreshold) {
        if (traffic_light_state == TrafficLightState::GREEN) {
            // 绿灯逻辑：适当减速
            double adjustedSpeed = std::max(ego_state.v * calculateDecelerationFactor(localX, dynamicApproachThreshold), MINIMUM_SPEED);
            AWARN << "Green light and approaching stop line, adjusted speed: " << adjustedSpeed;
            return {adjustedSpeed, false};
        } else {
            // 非绿灯逻辑（红灯或黄灯）：停车
            double stop_deceleration = GetADCStopDeceleration(ego_state, adc_front_edge_s, stop_line);
            if (stop_deceleration <= RED_LIGHT_MAXIMUM_DECELERATION) {
                double adjustedSpeed = std::max(ego_state.v * calculateDecelerationFactor(localX, dynamicApproachThreshold), MINIMUM_SPEED);
                AWARN << "Adjusted speed for stopping: " << adjustedSpeed;  // 输出调整后的速度
                return {adjustedSpeed, true};
            } else {
                AERROR << "Unable to decelerate safely for red light, current deceleration: " << stop_deceleration;
                return {ego_state.v, false};
            }
        }
    }

    AWARN << "Proceeding at desired speed: " << DESIRED_SPEED;  // 未接近停车线或绿灯状态，继续以期望速度行驶
    return {DESIRED_SPEED, false};
}

double TrafficLightDecision::calculateDecelerationFactor(double distance, double threshold) {
    const double midPoint = threshold * 0.9;  // 函数的中点设置为阈值的 90%
    const double steepness = -0.5;            // 增加陡峭程度，以便在接近停车线时快速减速

    // Sigmoid 函数计算减速因子
    double decelerationFactor = 1.0 / (1.0 + exp(steepness * (distance - midPoint)));
    decelerationFactor = std::max(decelerationFactor, 0.75);  // 考虑最小减速因子

    AINFO << "Deceleration factor: " << decelerationFactor;  // 输出减速因子
    return decelerationFactor;
}
