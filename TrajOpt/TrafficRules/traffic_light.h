#pragma once

#include <memory>

#include "PathSegment/path.h"
#include "PathSegment/vec2d.h"
#include "TrafficRules/traffic_light.h"
#include "TrafficRules/util.h"

class TrafficLightDecision {
public:
    static std::pair<double, bool> MakeDecisions(const EgoVehicleInfo& ego_state,
                                                 const std::shared_ptr<Path>& reference_line_info,
                                                 const Vec2d& stop_line,
                                                 const TrafficLightInfo& traffic_light_info);

    static double calculateDecelerationFactor(double distance, double threshold);
};