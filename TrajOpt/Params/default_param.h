#pragma once
#include <fstream>
#include <iostream>

#include "nlohmann/json.hpp"

// Vehicle param
inline double FRONT_SUSPENSION = 0.0;
inline double REAR_SUSPENSION = 0.0;
inline double WHEELBASE = 0.0;
inline double VEHICLE_WIDTH = 0.0;
inline double VEHICLE_HEIGHT = 0.0;
inline double VEHICLE_LENGTH = 0.0;

class ParametersLoader {
public:
    explicit ParametersLoader(const std::string& filePath) {
        std::ifstream file(filePath);
        if (!file.is_open()) {
            AERROR << "Error opening file: " << filePath;
        }
        file >> jsonData;
        loadParam();
    }

    static void print_all_params() {
        ADEBUG << "Vehicle Parameters:"
               << "\n"
               << "-------------------------"
               << "\n"
               << "Front Suspension Length (m): " << FRONT_SUSPENSION << "\n"
               << "Rear Suspension Length (m): " << REAR_SUSPENSION << "\n"
               << "Wheelbase Length (m): " << WHEELBASE << "\n"
               << "Vehicle Width (m): " << VEHICLE_WIDTH << "\n"
               << "Vehicle Height (m): " << VEHICLE_HEIGHT << "\n"
               << "Total Vehicle Length (m): " << VEHICLE_LENGTH << "\n"
               << "\n-------------------------";
    }

private:
    void loadParam() {
        FRONT_SUSPENSION = getParameter("vehicle", "front_suspension");   // 前轴到车身前端的距离
        REAR_SUSPENSION = getParameter("vehicle", "rear_suspension");     // 后轴到车身后端的距离
        WHEELBASE = getParameter("vehicle", "wheelbase");                 // 轮距
        VEHICLE_WIDTH = getParameter("vehicle", "width");                 // 车身宽度
        VEHICLE_HEIGHT = getParameter("vehicle", "height");               // 车身高度
        VEHICLE_LENGTH = FRONT_SUSPENSION + REAR_SUSPENSION + WHEELBASE;  // 车身长度
    }

    double getParameter(const std::string& category, const std::string& param) const {
        if (jsonData.contains(category) && jsonData[category].contains(param)) {
            return jsonData[category][param];
        }
        AERROR << "Parameter not found: " + category + "::" + param;
        return -1;
    }

private:
    nlohmann::json jsonData;
};
