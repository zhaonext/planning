#pragma once
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>

#include "Common/common.h"

// vehicle param
inline double DESIRED_SPEED = 0.0;
inline double MINIMUM_SPEED = 0.0;
inline double MAXIMUM_SPEED = 0.0;

inline double MINIMUM_ACCELERATION = 0.0;
inline double MAXIMUM_ACCELERATION = 0.0;

inline double MINIMUM_FRONT_WHEEL_TURNING_ANGLE = 0.0;
inline double MAXIMUM_FRONT_WHEEL_TURNING_ANGLE = 0.0;

inline double MINIMUM_LATERAL_OFFSET = 0.0;
inline double MAXIMUM_LATERAL_OFFSET = 0.0;

// Weight
inline double weight_acceleration = 0.0;
inline double weight_positive_velocity = 0.0;
inline double weight_negative_velocity = 0.0;
inline double weight_phidot = 0.0;
inline double weight_s_offset = 0.0;
inline double weight_n_offset = 0.0;
inline double weight_distance_obstacle = 0.0;

// Random obstacles
inline int NUMBER_OF_OBSTACLE_SAMPLES = 0;
inline int RANDOM_SEED = 0;
inline double SAMPLE_START_SECTION = 0.0;
inline double SAMPLE_END_SECTION = 0.0;

// Pedestrian envelope ellipse
inline double PEDESTRIAN_ENVELOPE_ELLIPSE_PARAM_A = 0.0;
inline double PEDESTRIAN_ENVELOPE_ELLIPSE_PARAM_B = 0.0;

// Road
inline double LANE_WIDTH = 0.0;
inline double LANE_SPEED_LIMIT = 0.0;

// Init state
inline double INIT_STATE_S = 0.0;
inline double INIT_STATE_N = 0.0;
inline double INIT_STATE_ALPHA = 0.0;
inline double INIT_STATE_X = 0.0;
inline double INIT_STATE_Y = 0.0;
inline double INIT_STATE_V = 0.0;
inline double INIT_STATE_PHI = 0.0;

class ScenarioManager {
public:
    static bool loadParam(const std::string &filepath, const std::string &scenario_name, std::vector<ObstacleInfo> &obstacles) {
        std::ifstream file(filepath);
        if (!file.is_open()) {
            AERROR << "Failed to open the file: " + filepath;
            return false;
        }

        nlohmann::json j;
        file >> j;

        auto scenarios = j.at("scenarios");
        for (const auto &scenario : scenarios) {
            if (scenario["name"] == scenario_name) {
                if (scenario.contains("obstacles")) {
                    for (const auto &obsJson : scenario["obstacles"]) {
                        ObstacleInfo obs;
                        obs.from_json(obsJson);
                        obstacles.push_back(obs);
                    }
                }
                if (scenario.contains("speed_limits")) {
                    DESIRED_SPEED = scenario["speed_limits"]["desired"].get<double>() / 3.6;  // km/h -> m/s
                    MINIMUM_SPEED = scenario["speed_limits"]["minimum"].get<double>() / 3.6;
                    MAXIMUM_SPEED = scenario["speed_limits"]["maximum"].get<double>() / 3.6;
                }
                if (scenario.contains("acceleration_limits")) {
                    MINIMUM_ACCELERATION = scenario["acceleration_limits"]["minimum"];
                    MAXIMUM_ACCELERATION = scenario["acceleration_limits"]["maximum"];
                }
                if (scenario.contains("front_wheel_turning_angle")) {
                    MINIMUM_FRONT_WHEEL_TURNING_ANGLE = scenario["front_wheel_turning_angle"]["minimum"];  // 0.52rad = 30°; 0.61rad = 35°; 0.7rad = 40°
                    MAXIMUM_FRONT_WHEEL_TURNING_ANGLE = scenario["front_wheel_turning_angle"]["maximum"];
                }
                if (scenario.contains("lateral_offset")) {
                    MINIMUM_LATERAL_OFFSET = scenario["lateral_offset"]["minimum"];
                    MAXIMUM_LATERAL_OFFSET = scenario["lateral_offset"]["maximum"];
                }
                if (scenario.contains("weights")) {
                    weight_acceleration = scenario["weights"]["acceleration"];
                    weight_positive_velocity = scenario["weights"]["positive_velocity"];
                    weight_negative_velocity = scenario["weights"]["negative_velocity"];
                    weight_phidot = scenario["weights"]["phidot"];
                    weight_s_offset = scenario["weights"]["s_offset"];
                    weight_n_offset = scenario["weights"]["n_offset"];
                    weight_distance_obstacle = scenario["weights"]["distance_obstacle"];
                }
                if (scenario.contains("sample_obstacles")) {
                    NUMBER_OF_OBSTACLE_SAMPLES = scenario["sample_obstacles"]["number_of_obstacle_samples"];
                    RANDOM_SEED = scenario["sample_obstacles"]["random_seed"];
                    SAMPLE_START_SECTION = scenario["sample_obstacles"]["start_section"];
                    SAMPLE_END_SECTION = scenario["sample_obstacles"]["end_section"];
                }
                if (scenario.contains("pedestrian_envelope_ellipse")) {
                    PEDESTRIAN_ENVELOPE_ELLIPSE_PARAM_A = scenario["pedestrian_envelope_ellipse"]["ellipse_param_a"];
                    PEDESTRIAN_ENVELOPE_ELLIPSE_PARAM_B = scenario["pedestrian_envelope_ellipse"]["ellipse_param_a"];
                }
                if (scenario.contains("road")) {
                    LANE_WIDTH = scenario["road"]["lane_width"];
                    LANE_SPEED_LIMIT = scenario["road"]["lane_speed_limit"].get<double>() / 3.6;  // km/h -> m/s;
                }
                if (scenario.contains("init_state")) {
                    INIT_STATE_S = scenario["init_state"]["s"];
                    INIT_STATE_N = scenario["init_state"]["n"];
                    INIT_STATE_ALPHA = scenario["init_state"]["alpha"];
                    INIT_STATE_X = scenario["init_state"]["x"];
                    INIT_STATE_Y = scenario["init_state"]["y"];
                    INIT_STATE_V = scenario["init_state"]["v"].get<double>() / 3.6;
                    INIT_STATE_PHI = scenario["init_state"]["phi"];
                }
                return true;
            }
        }

        AERROR << "Scenario name " << scenario_name << " not found in the file: " + filepath;
        return false;
    }
    static void print_weight_bound_info() {
        ADEBUG << "==== Speed and Lateral Offset Limits ====\n";
        ADEBUG << "Desired Speed: " << DESIRED_SPEED * 3.6 << " km/h\n";
        ADEBUG << "Minimum Speed: " << MINIMUM_SPEED * 3.6 << " km/h\n";
        ADEBUG << "Maximum Speed: " << MAXIMUM_SPEED * 3.6 << " km/h\n";
        ADEBUG << "Minimum Lateral Offset: " << MINIMUM_LATERAL_OFFSET << " m\n";
        ADEBUG << "Maximum Lateral Offset: " << MAXIMUM_LATERAL_OFFSET << " m\n\n";

        ADEBUG << "==== Acceleration and Steering angle Limits ====\n";
        ADEBUG << "Minimum Acceleration: " << MINIMUM_ACCELERATION << " m/s^2\n";
        ADEBUG << "Maximum Acceleration: " << MAXIMUM_ACCELERATION << " m/s^2\n";
        ADEBUG << "Min Front Wheel Angle: " << MINIMUM_FRONT_WHEEL_TURNING_ANGLE * 180 / M_PI << " degree\n";
        ADEBUG << "Max Front Wheel Angle: " << MAXIMUM_FRONT_WHEEL_TURNING_ANGLE * 180 / M_PI << " degree\n\n";

        ADEBUG << "==== Cost Weights ====\n";
        ADEBUG << "Acceleration Weight: " << weight_acceleration << "\n";
        ADEBUG << "Positive Velocity Weight: " << weight_positive_velocity << "\n";
        ADEBUG << "Negative Velocity Weight: " << weight_negative_velocity << "\n";
        ADEBUG << "Phidot Weight: " << weight_phidot << "\n";
        ADEBUG << "S Offset Weight: " << weight_s_offset << "\n";
        ADEBUG << "N Offset Weight: " << weight_n_offset << "\n";
        ADEBUG << "Distance to Obstacle Weight: " << weight_distance_obstacle << "\n\n"
               << "=======================\n";
    }
};
