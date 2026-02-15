#pragma once

#include <cmath>
#include <utility>

#include "Common/common.h"

static std::pair<double, double> toLocalCoordinates(const EgoVehicleInfo& ego, const ObstacleInfo& object) {
    // 获取车辆前端的位置
    Vec2d front_edge = ego.center_to_front_edge();

    // 计算障碍物相对于车辆前端的位置
    double dx = object.x - front_edge.x();
    double dy = object.y - front_edge.y();

    // 考虑车辆朝向，将障碍物坐标转换为车辆局部坐标
    double localX = dx * std::cos(-ego.phi) - dy * std::sin(-ego.phi);
    double localY = dx * std::sin(-ego.phi) + dy * std::cos(-ego.phi);

    return {localX, localY};
}

static std::pair<double, double> toLocalCoordinates(const EgoVehicleInfo& ego, const Vec2d& global_position) {
    // 获取车辆前端的位置
    Vec2d front_edge = ego.center_to_front_edge();

    // 计算障碍物相对于车辆前端的位置
    double dx = global_position.x() - front_edge.x();
    double dy = global_position.y() - front_edge.y();

    // 考虑车辆朝向，将障碍物坐标转换为车辆局部坐标
    double local_x = dx * cos(-ego.phi) - dy * sin(-ego.phi);
    double local_y = dx * sin(-ego.phi) + dy * cos(-ego.phi);

    return {local_x, local_y};
}
