#pragma once
#include <random>
#include <string>

#include "Map/RoadNet.h"
#include "Map/road.h"
#include "Map/routing.h"
#include "PathSegment/path.h"
#include "TrafficRules/traffic_light_controller.h"
#include "TrafficRules/traffic_light_node.h"
#include "TrafficRules/traffic_rule_parse.h"
#include "Visualization/plot.h"
#include "Visualization/visualization.h"
#include "lane_change.h"
#include "safe_speed.h"
#include "solver_interface.h"

static std::string getBasePath() {
    char result[PATH_MAX];
    ssize_t count = readlink("/proc/self/exe", result, PATH_MAX);
    std::string full_path(result, (count > 0) ? count : 0);
    std::string basePath = full_path.substr(0, full_path.find_last_of('/') + 1);

    // 假设构建目录是 "cmake-build-debug"，则去除它
    size_t debug = basePath.find("/build/");
    size_t release = basePath.find("/release/");
    size_t pos_debug = basePath.find("/cmake-build-debug/");
    size_t pos_release = basePath.find("/cmake-build-release/");

    if (debug != std::string::npos) {
        basePath = basePath.substr(0, debug);
    }
    if (release != std::string::npos) {
        basePath = basePath.substr(0, release);
    }
    if (pos_debug != std::string::npos) {
        basePath = basePath.substr(0, pos_debug);
    }
    if (pos_release != std::string::npos) {
        basePath = basePath.substr(0, pos_release);
    }

    return basePath;
}
struct PlannerParams {
    std::string basePath;
    std::string default_file_path = "/Params/default_param.json";
    std::string scenarios_file_path = "/Scenarios/scenarios.json";
    std::string scenarios_name = "turn_left_param";
    std::string roadNet_file_path = "/Map/RoadNet.xml";
    std::string traffic_rule_path = "/TrafficRules/traffic_rule.json";
    std::string driving_route = "turn_left";
    float time_step = 0.1;
    int horizon = 41;
};
struct SolverOptionsManager {
    static altro::AltroOptions getDefaultOptions() {
        altro::AltroOptions opts;
        // opts.verbose = altro::Verbosity::Outer;
        opts.throw_errors = true;
        opts.iterations_max = 50;
        opts.tol_primal_feasibility = 1e-12;
        opts.penalty_scaling = 500;
        opts.tol_meritfun_gradient = 1e-10;
        opts.use_backtracking_linesearch = true;
        return opts;
    }
};

class Planner {
    using ControlSequence = std::tuple<double, double, double, double>;  // acc, phidot, delta, kappa

public:
    explicit Planner(PlannerParams params) : params_(std::move(params)) { loadDefaultParam(); }
    bool planFirstFrame();
    bool plan();
    void getVisualData(double &times);

    void addVehicleObstacle(ObstacleInfo obs);
    void addPedestrianObstacle(ObstacleInfo obs);
    void updateTrafficLightInfo(const TrafficLightInfo &traffic_light_info) { current_traffic_light_info_ = traffic_light_info; }

    const Trajectory &getEgoTrajectory() const { return ego_trajectory_; }
    const Eigen::VectorXd &getEgoInitState() const { return state_init_; }
    const Planner::ControlSequence &getControlSequence() const { return control_sequence_; }
    const std::vector<ObstacleInfo> &getTargetObstacleInitState() const { return obstacles_manager_->GetTargetObstacleInitState(); }
    const std::vector<ObstacleInfo> &getNoTargetObstacleInitState() const { return obstacles_manager_->GetNoTargetObstacleInitState(); }
    const std::vector<std::tuple<int, std::string, Trajectory>> getAllObstaclePredictionTrajectory() const { return all_obs_prediction_traj_; }
    const std::shared_ptr<RoadNetwork> &getRoadNetwork() const { return road_network_; }
    bool isReachStopLine() const { return reach_stop_line_; }
    const TrafficLightInfo &getCurrentTrafficLightInfo() const { return current_traffic_light_info_; }
    const vector<std::tuple<double, double, double>> &getSpeedProfile() const { return speed_profile_; }
    const Vec2d &getStopLine() const { return stop_line_; }
    const std::shared_ptr<Path> &getRefLine() const { return ref_line_; }

private:
    // init
    bool loadScenarios(const std::string &scenarios_name);
    void loadDefaultParam() const;
    bool setReferenceLine();
    bool setInitState();
    bool initializeDecider();
    bool updateRefLineAndStateProjectionPoints();

    // solve
    bool initializeSolver();
    bool resetSolver();
    bool setConstraints();
    bool setCostFunction();
    bool solveProblem();
    bool solve();
    // get next frame data
    bool setNextFrameInitStateAndInput();
    bool getNextFrameObstaclePredictionTrajectories();
    std::pair<Eigen::VectorXd, Eigen::VectorXd> getStateAndInputAIndex(int index);
    // safe speed
    bool setSafeSpeed();
    // road
    bool loadRoadAndSampleObstacles(double offset, int horizon, int number_of_obstacle_samples, int random_seed);

    // print
    static std::string SolveStatusToString(altro::SolveStatus status);

    // lane change
    //    bool executeLaneChange();

private:
    double safeSpeed_{};
    PlannerParams params_;
    Eigen::VectorXd state_init_ = Eigen::VectorXd::Zero(NUM_STATE);
    Eigen::VectorXd state_final_ = Eigen::VectorXd::Zero(NUM_STATE);
    Eigen::VectorXd u_init_ = Eigen::VectorXd::Zero(NUM_INPUT);
    Eigen::VectorXd u_final_ = Eigen::VectorXd::Zero(NUM_INPUT);
    std::shared_ptr<altro::ALTROSolver> solver_;
    std::vector<ObstacleInfo> obstacles_;
    std::shared_ptr<ObstaclesManager> obstacles_manager_;
    //    std::shared_ptr<ReferenceBezierCurve<BEZIER_ORDER>> reference_line_;
    std::shared_ptr<Road> road_;
    std::shared_ptr<RoadNetwork> road_network_;
    std::shared_ptr<Path> ref_line_;

public:
private:
    std::shared_ptr<Routing> routing_;

private:
    // flag
    std::shared_ptr<LaneChangeDecider> lane_change_decider_;

    // Visualize data
    Trajectory ego_trajectory_;
    ControlSequence control_sequence_;
    std::vector<std::tuple<int, std::string, Trajectory>> all_obs_prediction_traj_;
    std::vector<std::tuple<double, double, double>> speed_profile_;

    // traffic light
    bool reach_stop_line_{};
    Vec2d stop_line_;
    TrafficLightInfo current_traffic_light_info_{};
};

class PlannerNode {
public:
    explicit PlannerNode(ros::NodeHandle &nh) : nh_(nh), is_first_frame_(true), cycle_times_(0) {
        PlannerParams params;
        params.basePath = getBasePath();
        planner_ = std::make_shared<Planner>(params);
        traffic_light_controller_ = std::make_shared<TrafficLightController>();
        timer_ = nh_.createTimer(ros::Duration(1.0), &PlannerNode::traffic_lights_callback, this);
        pedestrian_sub_ = nh_.subscribe("/move_base_simple/goal", 10, &PlannerNode::pedestrian_callback, this);
        vehicle_sub_ = nh_.subscribe("/initialpose", 10, &PlannerNode::dynamic_vehicle_callback, this);
        static_obs_sub_ = nh_.subscribe("/publish_point", 10, &PlannerNode::static_obs_callback, this);
    }
    void planCallback();

    void pedestrian_callback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void dynamic_vehicle_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
    void static_obs_callback(const geometry_msgs::PointStamped::ConstPtr &msg);
    void traffic_lights_callback(const ros::TimerEvent &);

private:
    static double generateRandomSpeed(double min_speed, double max_speed);

    void visualization();

private:
    ros::NodeHandle nh_;
    ros::Timer timer_;
    ros::Subscriber pedestrian_sub_;
    ros::Subscriber vehicle_sub_;
    ros::Subscriber static_obs_sub_;

    std::shared_ptr<Planner> planner_;
    bool is_first_frame_ = true;
    double times = 0.0;
    int cycle_times_ = 0;

    std::shared_ptr<TrafficLightController> traffic_light_controller_;

    // visualization
    Visualization vis;
    bool canvas_settings_flag = true;
    std::vector<double> t_s;
    std::vector<double> velocity;
    std::vector<double> delta;
    std::vector<double> acc;
    std::vector<double> kappa;
};