#pragma once
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>

#include "Common/common.h"

// Traffic rule
//  1. crosswalk
//  2. traffic light
inline bool TRAFFIC_LIGHT_ENABLE = 1;
inline double RED_LIGHT_STOPPING_DISTANCE = 0.0;
inline double RED_LIGHT_MAXIMUM_DECELERATION = 0.0;

class TrafficRuleManager {
public:
    static bool loadParam(const std::string &filepath) {
        std::ifstream file(filepath);
        if (!file.is_open()) {
            AERROR << "Failed to open the file: " + filepath;
            return false;
        }

        nlohmann::json j;
        file >> j;

        auto traffic_rule = j.at("traffic_rule");
        for (const auto &rule : traffic_rule) {
            if (rule["name"] == "traffic_light") {
                if (rule.contains("config")) {
                    TRAFFIC_LIGHT_ENABLE = rule["config"]["enabled"].get<double>();
                    RED_LIGHT_STOPPING_DISTANCE = rule["config"]["stop_distance"].get<double>();
                    RED_LIGHT_MAXIMUM_DECELERATION = rule["config"]["max_stop_deceleration"].get<double>();
                }
                return true;
            }
        }
        return false;
    }
};
