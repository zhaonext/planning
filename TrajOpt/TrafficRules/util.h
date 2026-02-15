#pragma once
#include <cmath>
#include <limits>

#include "Common/common.h"
#include "traffic_rule_parse.h"

static double GetADCStopDeceleration(const EgoVehicleInfo& vehicle_state,
                                     const Vec2d& adc_front_edge,
                                     const Vec2d& stop_line) {
    double adc_speed = vehicle_state.v;
    const double max_adc_stop_speed = RED_LIGHT_MAXIMUM_DECELERATION;

    // 确保车辆速度高于最大停车速度
    if (adc_speed < max_adc_stop_speed) {
        return 0.0;
    }

    // 计算车辆前方方向的单位向量
    Vec2d front_direction(std::cos(vehicle_state.phi), std::sin(vehicle_state.phi));

    // 计算 stop_line 和 adc_front_edge 的相对位置向量
    Vec2d relative_position = stop_line - adc_front_edge;

    // 判断 stop_line 是否在 adc_front_edge 的前方
    if (front_direction.InnerProd(relative_position) <= 0) {
        // stop_line 不在 adc_front_edge 的前方
        return std::numeric_limits<double>::max();
    }

    double stop_distance = relative_position.Length();

    // 停车线与车辆非常接近，无需减速
    if (stop_distance < 1e-5) {
        return std::numeric_limits<double>::max();
    }

    // 计算减速度
    return (adc_speed * adc_speed) / (2 * stop_distance);
}
