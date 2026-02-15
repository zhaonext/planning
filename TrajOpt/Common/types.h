#pragma once
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>

#include "Eigen/Dense"
#include "Common/log.h"
#include "Params/default_param.h"
#include "PathSegment/vec2d.h"
#include "nlohmann/json.hpp"

#define Power(x, y) std::pow(x, y)
#define Cos(theta)  std::cos(theta)
#define Sin(theta)  std::sin(theta)
#define Sqrt(x)     std::sqrt(x)
#define Exp         std::exp(1.0)

static constexpr double INF = 1E8;

// Stata and Control volume dimension
static constexpr int NUM_STATE = 7;  // s, n, alpha, x, y, phi, v
static constexpr int NUM_INPUT = 2;  // acc, delta

// Reference line
static constexpr int BEZIER_ORDER = 2;

// Envelope circles param
constexpr int NUM_EGO_ENVELOPE_CIRCLES = 4;  // 自车包络圆个数
constexpr double DEFAULT_T_SAFE = 0.1;       // 障碍物包络椭圆的默认安全时间
constexpr double DEFAULT_S_SAFE = 0.0;       // 障碍物包络椭圆的默认安全距离

// Traffic light
enum class TrafficLightState {
    RED,
    YELLOW,
    GREEN
};
enum class TrafficLightOrientation {
    HORIZONTAL,
    VERTICAL
};
struct TrafficLightInfo {
    TrafficLightInfo() = default;

public:
    int getId() const { return id_; }
    TrafficLightState getState() const { return state_; }
    const Vec2d &getPosition() const { return position_; }
    double getHeight() const { return height_; }
    TrafficLightOrientation getOrientation() const { return orientation_; }

    void setId(int id) { id_ = id; }
    void setState(TrafficLightState state) { state_ = state; }
    void setPosition(const Vec2d &position) { position_ = position; }
    void setHeight(double height) { height_ = height; }
    void setOrientation(TrafficLightOrientation orientation) { orientation_ = orientation; }

private:
    int id_{};
    TrafficLightState state_{};
    TrafficLightOrientation orientation_{};
    Vec2d position_{};
    double height_{};
};

const Eigen::VectorXd Q = Eigen::VectorXd::Constant(NUM_STATE, 1e-3);
const Eigen::VectorXd R = Eigen::VectorXd::Constant(NUM_INPUT, 1e-4);
const Eigen::VectorXd Qf = Eigen::VectorXd::Constant(NUM_STATE, 1e-5);
const Eigen::VectorXd Rf = Eigen::VectorXd::Constant(NUM_INPUT, 1e-5);

struct State {
    double s;      // 0
    double n;      // 1
    double alpha;  // 2
    double x;      // 3
    double y;      // 4
    double v;      // 5
    double phi;    // 6

    void setZero() {
        s = 0.0;
        n = 0.0;
        alpha = 0.0;
        x = 0.0;
        y = 0.0;
        v = 0.0;
        phi = 0.0;
    }
    void print_info() const {
        std::cout << "State Information:\n"
                  << " - s (Arc Length): " << s << "\n"
                  << " - n (Lateral Offset): " << n << "\n"
                  << " - alpha (Orientation Angle): " << alpha << "\n"
                  << " - x (X Coordinate): " << x << "\n"
                  << " - y (Y Coordinate): " << y << "\n"
                  << " - v (Velocity): " << v << "\n"
                  << " - phi (Heading Angle): " << phi
                  << std::endl;
    }
};
struct Input {
    double acceleration;
    double phidot;

    void setZero() {
        acceleration = 0.0;
        phidot = 0.0;
    }
};

typedef Eigen::Matrix<double, NUM_STATE, 1> StateVector;
typedef Eigen::Matrix<double, NUM_INPUT, 1> InputVector;

typedef Eigen::Matrix<double, NUM_STATE, 1> Bounds_s;
typedef Eigen::Matrix<double, NUM_INPUT, 1> Bounds_u;
typedef Eigen::Matrix<double, NUM_STATE + NUM_INPUT, 1> Bounds_su;  // lower + upper

StateVector stateToVector(const State &S);
InputVector inputToVector(const Input &U);

State vectorToState(const StateVector &X);
Input vectorToInput(const InputVector &U);

State pointerToState(const double *X);
Input pointerToInput(const double *U);

StateVector pointerToStateVector(const double *X);
InputVector pointerToInputVector(const double *U);

enum ObstacleType {
    PEDESTRIAN,       // 行人
    BICYCLE,          // 自行车
    STATIC_VEHICLE,   // 静态车辆
    DYNAMIC_VEHICLE,  // 动态车辆
    TYPE_UNKNOWN      // 未定义或未知
};
enum ObstacleIntent {
    LF,             // 直行
    LL,             // 左转
    LR,             // 右转
    STOP,           // 停止
    ACC,            // 加速
    DEC,            // 减速
    PARKING,        // 停车
    INTENT_UNKNOWN  // 未定义或未知
};

struct EgoVehicleInfo {
    double x{};
    double y{};
    double v{};
    double phi{};
    double t{};

    Vec2d center_to_front_edge() const {
        double front_to_axle = VEHICLE_LENGTH - REAR_SUSPENSION;
        Vec2d front;
        front.set_x(x + front_to_axle * std::cos(phi));
        front.set_y(y + front_to_axle * std::sin(phi));
        return front;
    }

    Vec2d center_to_rear_edge() const {
        Vec2d rear;
        rear.set_x(x - REAR_SUSPENSION * std::cos(phi));
        rear.set_y(y - REAR_SUSPENSION * std::sin(phi));
        return rear;
    }
    void print_info() const {
        std::cout << "x = " << x << ", y = " << y << ", v = " << v << ", phi = " << phi << std::endl;
    }
};

struct ObstacleInfo {
    int id{};
    double x{};
    double y{};
    double v{};
    double phi{};
    double t{};
    double psi_dot{};
    double a{};

    double priority;  // 优先级：行人：高; 动态障碍物: 较高; 静态障碍物：低
    ObstacleType type = TYPE_UNKNOWN;
    ObstacleIntent intent = INTENT_UNKNOWN;

    const std::map<std::string, ObstacleType> typeMap = {
        {"PEDESTRIAN", PEDESTRIAN},
        {"BICYCLE", BICYCLE},
        {"STATIC_VEHICLE", STATIC_VEHICLE},
        {"DYNAMIC_VEHICLE", DYNAMIC_VEHICLE}};

    const std::map<std::string, ObstacleIntent> intentMap = {
        {"LF", LF},
        {"LL", LL},
        {"LR", LR},
        {"STOP", STOP},
        {"ACC", ACC},
        {"DEC", DEC},
        {"PARKING", PARKING}};

    void from_json(const nlohmann::json &j) {
        id = j.at("id");
        x = j.at("x");
        y = j.at("y");
        v = j.at("v").get<double>() / 3.6;
        phi = j.at("phi");

        if (j.contains("t")) t = j.at("t");
        if (j.contains("psi_dot")) psi_dot = j.at("psi_dot");
        if (j.contains("a")) a = j.at("a");

        // 使用映射查找对应的枚举值
        type = typeMap.find(j.at("type").get<std::string>()) != typeMap.end() ? typeMap.at(j.at("type").get<std::string>()) : TYPE_UNKNOWN;
        intent = intentMap.find(j.at("intent").get<std::string>()) != intentMap.end() ? intentMap.at(j.at("intent").get<std::string>()) : INTENT_UNKNOWN;
    }

    void print_info() const {
        std::cout << "Obstacle ID: " << id
                  << ", Pos: (" << x << ", " << y << ")"
                  << ", Vel: " << v * 3.6
                  << ", Type: " << obstacleTypeToString(type)
                  << ", Intent: " << obstacleIntentToString(intent)
                  << std::endl;
    }

    static std::string obstacleTypeToString(const ObstacleType type) {
        switch (type) {
            case PEDESTRIAN:
                return "PEDESTRIAN";
            case BICYCLE:
                return "BICYCLE";
            case STATIC_VEHICLE:
                return "STATIC_VEHICLE";
            case DYNAMIC_VEHICLE:
                return "DYNAMIC_VEHICLE";
            default:
                return "TYPE_UNKNOWN";
        }
    }
    static std::string obstacleIntentToString(const ObstacleIntent intent) {
        switch (intent) {
            case LF:
                return "LF";
            case LL:
                return "LL";
            case LR:
                return "LR";
            case STOP:
                return "STOP";
            case ACC:
                return "ACC";
            case DEC:
                return "DEC";
            case PARKING:
                return "PARKING";
            default:
                return "INTENT_UNKNOWN";
        }
    }

    void update(const double time_step) {
        x += v * cos(phi) * time_step;
        y += v * sin(phi) * time_step;
        t += time_step;
    }
};

struct TrajectoryPoint {
    double x;
    double y;
    double t;

    double v;
    double phi;

    void print_point() const {
        std::cout << "Trajectory point: x = " << x << ", y = " << y << ", t = " << t << ", v = " << v << ", phi = " << phi << std::endl;
    }
};
struct Trajectory {
    std::vector<TrajectoryPoint> points;
};
struct ObstacleTrajectory {
    int id;
    ObstacleType type;
    std::unordered_map<ObstacleIntent, std::pair<double, Trajectory>> multimode_trajectories;  // pair of Trajectory and probability
};

EgoVehicleInfo StateToEgoVehicleInfo(const State &state);
Vec2d StateToVec2d(const State &state);