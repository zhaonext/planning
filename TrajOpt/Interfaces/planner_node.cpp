#include "planner_node.h"

#include "Map/RoadNet.h"

bool Planner::planFirstFrame() {
    if (!loadScenarios(params_.scenarios_name)) return false;
    if (!setInitState()) return false;
    if (!setReferenceLine()) return false;
    if (!initializeDecider()) return false;
    if (!solve()) return false;
    return true;
}
bool Planner::plan() {
    if (!setNextFrameInitStateAndInput()) return false;
    if (!updateRefLineAndStateProjectionPoints()) return false;
    if (!getNextFrameObstaclePredictionTrajectories()) return false;
    //    if (!executeLaneChange()) return false;
    if (!setSafeSpeed()) return false;
    if (!solve()) return false;
    return true;
}

bool Planner::setInitState() {
    State state{};
    state.s = INIT_STATE_S;
    state.n = INIT_STATE_N;
    state.alpha = INIT_STATE_ALPHA;
    state.x = INIT_STATE_X;
    state.y = INIT_STATE_Y;
    state.v = INIT_STATE_V;
    state.phi = INIT_STATE_PHI;
    state_init_ = stateToVector(state);
    return true;
}

bool Planner::setReferenceLine() {
    const std::string driving_routing = params_.driving_route;
    auto connections = road_network_->getConnections();
    auto conn_it = connections.find(driving_routing);
    if (conn_it != connections.end() && conn_it->second) {
        auto routing_lane = conn_it->second->getRoutingLane();
        routing_ = std::make_shared<Routing>(routing_lane);
    } else {
        AERROR << "Routing setting failed.";
        return false;
    }
    ref_line_ = routing_->getReferenceLine(StateToVec2d(vectorToState(state_init_)));

    if (!ref_line_) {
        AERROR << "Reference line setting failed.";
        return false;
    }

    auto initState = ref_line_->calInitState(vectorToState(state_init_));
    state_init_ = stateToVector(initState);
    return true;
}

bool Planner::initializeDecider() {
    lane_change_decider_ = std::make_shared<LaneChangeDecider>();
    if (!lane_change_decider_) {
        AERROR << "Failed to initialize LaneChangeDecider.";
        return false;
    }
    return true;
}

bool Planner::loadScenarios(const std::string &scenarios_name) {
    // basic configuration file
    const std::string scenarios_path = params_.basePath + params_.scenarios_file_path;
    const bool scenario_load = ScenarioManager::loadParam(scenarios_path, scenarios_name, obstacles_);
    if (!scenario_load) {
        AERROR << "Failed to load parameters from " << scenarios_path;
        return false;
    }

    const std::string traffic_rule_path = params_.basePath + params_.traffic_rule_path;
    const bool traffic_rule_load = TrafficRuleManager::loadParam(traffic_rule_path);
    if (!traffic_rule_load) {
        AERROR << "Failed to load traffic rule ";
        return false;
    }

    road_network_ = std::make_shared<RoadNetwork>();
    const std::string road_network_path = params_.basePath + params_.roadNet_file_path;
    if (!road_network_->loadFromXML(road_network_path)) {
        AERROR << "Failed to load road network from XML.";
        return false;
    }

    const bool success = loadRoadAndSampleObstacles(LANE_WIDTH * 0.5, params_.horizon, NUMBER_OF_OBSTACLE_SAMPLES, RANDOM_SEED);
    if (!success) {
        AERROR << "Failed to load road and sample obstacles.";
        return false;
    }

    return true;
}

bool Planner::loadRoadAndSampleObstacles(double offset, int horizon, int number_of_obstacle_samples, int random_seed) {
    road_ = road_network_->getRoads().at(0);
    if (!road_) {
        AERROR << "Failed to create Road object.";
        return false;
    }
    auto ref_lines = road_->getLanes();

    obstacles_manager_ = std::make_shared<ObstaclesManager>(obstacles_, params_.horizon, params_.time_step);
    if (!obstacles_manager_) {
        AERROR << "Failed to create ObstaclesManager.";
        return false;
    }

    for (const auto &ref : ref_lines) {
        const auto obstacles = ref.getReferenceLine()->sampleObstaclesAlongBezierCurve(number_of_obstacle_samples, random_seed++);
        AINFO << "Sampling obstacles for Lane ID " << ref.getLaneID() << ": " << obstacles.size() << " obstacles sampled.";
        for (const auto &obs : obstacles) {
            addVehicleObstacle(obs);
        }
    }

    return true;
}
// bool Planner::executeLaneChange() {
//     auto refLines = road_->getLanes();
//     if (refLines.empty()) {
//         AWARN << "No lanes available in road.";
//         return false;
//     }
//
//     int currentLaneIndex = ref_index;                   // 当前车辆所在车道索引
//     int laneCount = static_cast<int>(refLines.size());  // 车道总数
//
//     if (!obstacles_manager_) {
//         AWARN << "Invalid obstacles manager.";
//         return false;
//     }
//
//     const auto obs_info = obstacles_manager_->GetTargetObstacleInitState();
//     auto ego_info = StateToEgoVehicleInfo(vectorToState(state_init_));
//     auto [slowVehicleAhead, safeToChangeLeft, safeToChangeRight] = lane_change_decider_->shouldChangeLane(ego_info, obs_info, currentLaneIndex, laneCount);
//
//     reference_line_.reset();
//     reference_line_ = nullptr;
//
//     if (slowVehicleAhead) {
//         if (safeToChangeRight && currentLaneIndex - 1 >= 0) {
//             // 向右换道
//             ref_index = currentLaneIndex - 1;
//             AWARN << "Current lane index: " << currentLaneIndex << ", Change Right lane index: " << ref_index;
//             reference_line_ = refLines.at(ref_index).getReferenceLine();
//         }
//         if (safeToChangeLeft && currentLaneIndex + 1 < laneCount) {
//             // 向左换道
//             ref_index = currentLaneIndex + 1;
//             AWARN << "Current lane index: " << currentLaneIndex << ", Change left lane index: " << ref_index;
//             reference_line_ = refLines.at(ref_index).getReferenceLine();
//         }
//     }
//
//     // 如果无法换道，保持当前车道
//     if (!reference_line_) {
//         reference_line_ = refLines.at(currentLaneIndex).getReferenceLine();
//         if (!reference_line_) {
//             AWARN << "Failed to get reference line for current lane.";
//             return false;
//         }
//     }
//
//     state_init_ = stateToVector(reference_line_->calInitState(vectorToState(state_init_)));
//
//     return true;
// }

void Planner::addVehicleObstacle(ObstacleInfo obs) {
    if (!obstacles_manager_) {
        AERROR << "Failed to create ObstaclesManager.";
    }
    int max_id = 0;
    if (!obstacles_.empty()) {
        // 找到最大的id
        const auto max_it = std::max_element(obstacles_.begin(), obstacles_.end(), [](const ObstacleInfo &a, const ObstacleInfo &b) { return a.id < b.id; });
        max_id = max_it->id;
    }
    obs.id = max_id + 1;
    obs.t = 0.0;
    // 根据速度设置类型和意图
    if (obs.v > 0) {
        obs.type = DYNAMIC_VEHICLE;
        obs.intent = LF;
        obs.priority = 0.8;

    } else {
        obs.type = STATIC_VEHICLE;
        obs.intent = STOP;  // 静态障碍物没有移动意图
        obs.priority = 0.4;
    }

    obstacles_.emplace_back(obs);
    obstacles_manager_->addObstacleToList(obs);
}
void Planner::addPedestrianObstacle(ObstacleInfo obs) {
    int max_id = 0;
    if (!obstacles_.empty()) {
        // 找到最大的id
        const auto max_it = std::max_element(obstacles_.begin(), obstacles_.end(), [](const ObstacleInfo &a, const ObstacleInfo &b) { return a.id < b.id; });
        max_id = max_it->id;
    }
    obs.id = max_id + 1;
    obs.t = 0.0;
    obs.type = ObstacleType::PEDESTRIAN;
    obs.priority = 1.0;
    // 根据速度设置类型和意图
    if (obs.v > 0) {
        obs.intent = ObstacleIntent::LF;  // 假设动态障碍物都是前进的
    } else {
        obs.type = ObstacleType::PEDESTRIAN;
        obs.intent = ObstacleIntent::STOP;  // 静态障碍物没有移动意图
    }
    AINFO << "Adding pedestrian with ID: ";
    obs.print_info();
    obstacles_.emplace_back(obs);
    obstacles_manager_->addObstacleToList(obs);
}
void Planner::loadDefaultParam() const { ParametersLoader load(params_.basePath + params_.default_file_path); }

bool Planner::initializeSolver() {
    altro::ErrorCodes error_codes;
    solver_ = std::make_shared<altro::ALTROSolver>(params_.horizon);
    error_codes = solver_->SetDimension(NUM_STATE, NUM_INPUT);
    if (error_codes != altro::ErrorCodes::NoError) {
        AERROR << "Failed to set dimension";
        return false;
    }
    error_codes = solver_->SetTimeStep(params_.time_step);
    if (error_codes != altro::ErrorCodes::NoError) {
        AERROR << "Failed to set time step";
        return false;
    }
    return true;
}
bool Planner::resetSolver() {
    solver_.reset();
    return solver_ == nullptr;
}
bool Planner::setConstraints() {
    if (!ref_line_) {
        AERROR << "Invalid reference line data.";
        return false;
    }
    // dynamics
    const auto dynamics_manager = std::make_shared<DynamicsManager>(ref_line_);
    if (!dynamics_manager->setDynamicsFunction(solver_, params_.horizon)) {
        AERROR << "Failed to set dynamics function";
        return false;
    }
    // bound
    const auto bounds_constraint_manager = std::make_shared<BoundsConstraintManager>();
    if (!bounds_constraint_manager->setBoundsConstraint(solver_, params_.horizon)) {
        AERROR << "Failed to set bounds constraint";
        return false;
    }
    // collision
    const auto collision_risk_evaluator = std::make_shared<CollisionConstraintManager<NUM_EGO_ENVELOPE_CIRCLES>>(obstacles_manager_);
    if (!collision_risk_evaluator->setCollisionConstraints(solver_, params_.horizon)) {
        AERROR << "Failed to set collision constraint";
        return false;
    }
    return true;
}
bool Planner::setCostFunction() {
    const auto cost_function_manager = std::make_shared<CostFunctionManager>(params_.horizon, obstacles_manager_);
    if (!cost_function_manager->setCostFunction(solver_, safeSpeed_)) {
        AERROR << "Failed to set cost function";
        return false;
    }
    if (!cost_function_manager->setFinalStateInputCost(solver_, vectorToState(state_final_), vectorToInput(u_final_))) {
        AERROR << "Failed to set final state input cost";
        return false;
    }
    return true;
}
bool Planner::solveProblem() {
    altro::ErrorCodes error_codes;
    error_codes = solver_->Initialize();
    if (error_codes != altro::ErrorCodes::NoError) {
        AERROR << "Failed to initialize solver";
        return false;
    }
    error_codes = solver_->SetInitialState(state_init_.data(), NUM_STATE);
    if (error_codes != altro::ErrorCodes::NoError) {
        AERROR << "Failed to set initial state";
        return false;
    }
    error_codes = solver_->SetInput(u_init_.data(), NUM_INPUT);
    if (error_codes != altro::ErrorCodes::NoError) {
        AERROR << "Failed to set input";
        return false;
    }

    altro::AltroOptions opts = SolverOptionsManager::getDefaultOptions();
    solver_->SetOptions(opts);

    auto start_time = std::chrono::high_resolution_clock::now();
    altro::SolveStatus status_code = solver_->Solve();
    auto end_time = std::chrono::high_resolution_clock::now();
    auto solving_duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
    if (status_code != altro::SolveStatus::Success) {
        AERROR << "Solving failed: " << SolveStatusToString(status_code);
        return false;
    } else {
        AINFO << "Solving succeeded. Solving time: " << solving_duration << " ms.";
        return true;
    }
}
bool Planner::solve() {
    if (!resetSolver()) return false;
    if (!initializeSolver()) return false;
    if (!setCostFunction()) return false;
    if (!setConstraints()) return false;
    if (!solveProblem()) return false;
    return true;
}

bool Planner::setNextFrameInitStateAndInput() {
    auto final_state_and_input = getStateAndInputAIndex(2);

    if (final_state_and_input.first.size() != NUM_STATE ||
        final_state_and_input.second.size() != NUM_INPUT) {
        AERROR << "Invalid state or input size.";
        return false;
    }

    auto initialState = final_state_and_input.first;
    state_init_ = initialState;
    ADEBUG << "Next frame initial state and input set:\n"
           << "  Initial State: " << state_init_.transpose() << "\n"
           << "  Initial Input: " << u_init_.transpose();

    return true;
}
bool Planner::updateRefLineAndStateProjectionPoints() {
    ref_line_ = routing_->getReferenceLine(StateToVec2d(vectorToState(state_init_)));
    if (!ref_line_) {
        AERROR << "Failed to update Reference Line";
        return false;
    }
    state_init_ = stateToVector(ref_line_->calInitState(vectorToState(state_init_)));
    return true;
}
bool Planner::getNextFrameObstaclePredictionTrajectories() {
    if (!obstacles_manager_->updateObstacleState(vectorToState(state_init_), 2)) {
        return false;
    }
    const auto obs_list = obstacles_manager_->GetTargetObstacleInitState();
    return true;
}
bool Planner::setSafeSpeed() {
    auto ego_state = StateToEgoVehicleInfo(vectorToState(state_init_));
    const auto obs_list_info = getTargetObstacleInitState();
    // 计算安全速度
    // TODO:: stop_line
    stop_line_ = road_network_->getRoads().at(0)->getLanes().at(3).getLaneEndPoints();
    auto [safeSpeed, reachStopLine] = SafetySpeed::calSafeSpeed(ego_state, obs_list_info, ref_line_, stop_line_, current_traffic_light_info_, params_.driving_route);
    safeSpeed_ = safeSpeed;
    reach_stop_line_ = reachStopLine;

    if (safeSpeed_ < 0) {
        ADEBUG << "Failed to calculate safe speed.";
        return false;
    }
    return true;
}

std::pair<Eigen::VectorXd, Eigen::VectorXd> Planner::getStateAndInputAIndex(int index) {
    if (index < 0 || index > params_.horizon) {
        AERROR << "Invalid time step: " << std::to_string(index);
        return {Eigen::VectorXd(), Eigen::VectorXd()};  // return empty vectors
    }

    double state[NUM_STATE];
    solver_->GetState(state, index);
    Eigen::VectorXd state_at_t = Eigen::Map<Eigen::VectorXd>(state, NUM_STATE);
    double u[NUM_INPUT];
    Eigen::VectorXd input_at_t = Eigen::Map<Eigen::VectorXd>(u, NUM_INPUT);
    if (index == params_.horizon) {
        solver_->GetInput(u, index - 1);
        input_at_t = Eigen::Map<Eigen::VectorXd>(u, NUM_INPUT);
    }
    return std::make_pair(state_at_t, input_at_t);
}

std::string Planner::SolveStatusToString(altro::SolveStatus status) {
    switch (status) {
        case altro::SolveStatus::Success:
            return "Success";
        case altro::SolveStatus::Unsolved:
            return "Unsolved";
        case altro::SolveStatus::MaxIterations:
            return "MaxIterations";
        case altro::SolveStatus::MaxObjectiveExceeded:
            return "MaxObjectiveExceeded";
        case altro::SolveStatus::StateOutOfBounds:
            return "StateOutOfBounds";
        case altro::SolveStatus::InputOutOfBounds:
            return "InputOutOfBounds";
        case altro::SolveStatus::MeritFunGradientTooSmall:
            return "MeritFunGradientTooSmall";
        default:
            return "Unknown SolveStatus";
    }
}

void Planner::getVisualData(double &times) {
    // ego data
    const int horizon = params_.horizon;
    ego_trajectory_.points.clear();
    ego_trajectory_.points.reserve(horizon);
    speed_profile_.clear();
    speed_profile_.reserve(horizon);

    // control data
    for (int i = 0; i <= horizon; i++) {
        double state[NUM_STATE];
        solver_->GetState(state, i);
        const double x_ego = state[3], y_ego = state[4], v_ego = state[5], phi_ego = state[6];
        TrajectoryPoint point{x_ego, y_ego, times, v_ego, phi_ego};  // x , y , t , v , phi
        speed_profile_.emplace_back(x_ego, y_ego, v_ego);
        ego_trajectory_.points.emplace_back(point);

        if (i == 0) {
            double input[NUM_INPUT];
            solver_->GetInput(input, 0);
            const double acc_val = input[0];
            const double phidot_val = input[1];
            const double delta_val = std::atan2(phidot_val * WHEELBASE, v_ego) * 180 / M_PI;

            double acc = acc_val;
            double phidot = phidot_val;
            double delta = delta_val;
            double kappa = phidot_val / v_ego;  //  kappa = phidot / v
            control_sequence_ = std::make_tuple(acc, phidot, delta, kappa);
        }
    }

    // obs data
    all_obs_prediction_traj_.clear();
    auto target_obstacle_trajectories = obstacles_manager_->GetTargetObstaclePredictionTrajectories();
    auto no_target_obstacle_trajectories = obstacles_manager_->GetNoTargetObstaclePredictionTrajectories();

    for (const auto &obs_traj : target_obstacle_trajectories) {
        double max_probability = -INF;
        Trajectory max_traj;

        // 寻找概率最大的轨迹
        for (const auto &mode_traj_pair : obs_traj.multimode_trajectories) {
            double probability = mode_traj_pair.second.first;
            if (probability > max_probability) {
                max_probability = probability;
                max_traj = mode_traj_pair.second.second;
            }
        }

        if (max_probability > -INF) {
            all_obs_prediction_traj_.emplace_back(obs_traj.id, "target", max_traj);
        }
    }

    for (const auto &obs_traj : no_target_obstacle_trajectories) {
        double max_probability = -INF;
        Trajectory max_traj;

        // 寻找概率最大的轨迹
        for (const auto &mode_traj_pair : obs_traj.multimode_trajectories) {
            double probability = mode_traj_pair.second.first;
            if (probability > max_probability) {
                max_probability = probability;
                max_traj = mode_traj_pair.second.second;
            }
        }

        if (max_probability > -INF) {
            all_obs_prediction_traj_.emplace_back(obs_traj.id, "no_target", max_traj);
        }
    }

    times += params_.time_step;
}

void PlannerNode::planCallback() {
    AINFO << "Planning cycle " << cycle_times_ << " started.";
    cycle_times_++;

    if (is_first_frame_) {
        if (!planner_->planFirstFrame()) {
            AERROR << "Failed to process the first frame in cycle " << cycle_times_;
        } else {
            visualization();
            is_first_frame_ = false;
        }
    } else {
        if (!planner_->plan()) {
            AERROR << "Failed to execute planning in cycle " << cycle_times_;
        }
        visualization();
    }
}

void PlannerNode::pedestrian_callback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    static tf2_ros::Buffer tfBuffer;
    static tf2_ros::TransformListener tfListener(tfBuffer);

    if (tfBuffer.canTransform("map", msg->header.frame_id, ros::Time(0), ros::Duration(0.01))) {
        geometry_msgs::TransformStamped transformStamped;
        transformStamped = tfBuffer.lookupTransform("map", msg->header.frame_id, ros::Time(0));

        // 将目标点转换到机器人的本地坐标系
        geometry_msgs::PoseStamped pose_transformed;
        tf2::doTransform(*msg, pose_transformed, transformStamped);

        double x = pose_transformed.pose.position.x;
        double y = pose_transformed.pose.position.y;

        // 使用tf2::Quaternion获取偏航角
        tf2::Quaternion q(pose_transformed.pose.orientation.x,
                          pose_transformed.pose.orientation.y,
                          pose_transformed.pose.orientation.z,
                          pose_transformed.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        // 生成随机速度
        double v = generateRandomSpeed(0.5, 3.0);

        ObstacleInfo obs;
        obs.x = x;
        obs.y = y;
        obs.phi = yaw;  // 使用tf2获取的偏航角
        obs.v = v;

        planner_->addPedestrianObstacle(obs);
    } else {
        AWARN << "Cannot transform from " << msg->header.frame_id.c_str() << " to map";
    }
}

void PlannerNode::dynamic_vehicle_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg) {
    static tf2_ros::Buffer tfBuffer;
    static tf2_ros::TransformListener tfListener(tfBuffer);

    if (tfBuffer.canTransform("map", msg->header.frame_id, ros::Time(0), ros::Duration(0.01))) {
        geometry_msgs::TransformStamped transformStamped;
        transformStamped = tfBuffer.lookupTransform("map", msg->header.frame_id, ros::Time(0));

        // 将估计的初始位置转换到机器人的本地坐标系
        geometry_msgs::PoseStamped pose_transformed;
        pose_transformed.header = msg->header;
        pose_transformed.pose = msg->pose.pose;
        tf2::doTransform(pose_transformed, pose_transformed, transformStamped);

        double x = pose_transformed.pose.position.x;
        double y = pose_transformed.pose.position.y;

        // 使用tf2::Quaternion获取偏航角
        tf2::Quaternion q(pose_transformed.pose.orientation.x,
                          pose_transformed.pose.orientation.y,
                          pose_transformed.pose.orientation.z,
                          pose_transformed.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        // 生成随机速度
        double v = generateRandomSpeed(30 / 3.6, 50 / 3.6);

        ObstacleInfo obs;
        obs.x = x;
        obs.y = y;
        obs.phi = yaw;  // 使用tf2获取的偏航角
        obs.v = v;
        planner_->addVehicleObstacle(obs);

    } else {
        AWARN << "Cannot transform from " << msg->header.frame_id.c_str() << " to map";
    }
}
void PlannerNode::static_obs_callback(const geometry_msgs::PointStamped::ConstPtr &msg) {
//    static tf2_ros::Buffer tfBuffer;
//    static tf2_ros::TransformListener tfListener(tfBuffer);
//
//    if (tfBuffer.canTransform("map", msg->header.frame_id, ros::Time(0), ros::Duration(0.01))) {
//        geometry_msgs::TransformStamped transformStamped;
//        transformStamped = tfBuffer.lookupTransform("map", msg->header.frame_id, ros::Time(0));
//
//        // 将接收到的点转换到机器人的本地坐标系
//        geometry_msgs::PointStamped point_transformed;
//        tf2::doTransform(*msg, point_transformed, transformStamped);
//
//        double x = point_transformed.point.x;
//        double y = point_transformed.point.y;
//
//        ObstacleInfo obs;
//        obs.x = x;
//        obs.y = y;
//        obs.phi = 0;  // 点没有朝向，偏航角设为0
//        obs.v = 0;    // 静态障碍物，速度设为0
//
////        planner_->addVehicleObstacle(obs);
//    } else {
//        AWARN << "Cannot transform from " << msg->header.frame_id.c_str() << " to map";
//    }
}
void PlannerNode::traffic_lights_callback(const ros::TimerEvent &) {
    TrafficLightInfo trafficLightInfo;
    const auto &roadNetwork = planner_->getRoadNetwork();
    traffic_light_controller_->update(1);  // 假设每次更新代表1秒
    for (const auto &light : roadNetwork->getTrafficLights()) {
        auto [x, y, z] = light->getPosition();
        int light_id = light->getId();
        auto orientationStr = light->getOrientation();
        auto orientation = roadNetwork->stringToTrafficLightOrientation(orientationStr);

        // 获取当前交通灯状态
        auto currentState = traffic_light_controller_->getStateForLight(light_id);

        trafficLightInfo.setId(light_id);
        trafficLightInfo.setOrientation(orientation);
        trafficLightInfo.setHeight(z);
        trafficLightInfo.setPosition(Vec2d(x, y));
        trafficLightInfo.setState(currentState);
        //            vis.publishTrafficLightMarker(light_id, x, y, z, currentState, orientation);
    }
    planner_->updateTrafficLightInfo(trafficLightInfo);
}

double PlannerNode::generateRandomSpeed(double min_speed, double max_speed) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(min_speed, max_speed);  // 假设速度范围在0.5到3.0米/秒之间
    return dis(gen);
}

void PlannerNode::visualization() {
    planner_->getVisualData(times);
    const auto &roadNetwork = planner_->getRoadNetwork();
    const auto &ego_state = vectorToState(planner_->getEgoInitState());
    const auto &ego_trajectory = planner_->getEgoTrajectory();
    const auto &target_obs_init_state = planner_->getTargetObstacleInitState();
    const auto &no_target_obs_init_state = planner_->getNoTargetObstacleInitState();
    const auto &control_sequence = planner_->getControlSequence();
    const auto &all_obs_prediction_traj = planner_->getAllObstaclePredictionTrajectory();
    const auto &build_barrier_wall = planner_->isReachStopLine();
    const auto &reference_line = planner_->getRefLine();
    const auto &traffic_light_info = planner_->getCurrentTrafficLightInfo();
    const auto &speed_profile = planner_->getSpeedProfile();
    const auto &stop_line = planner_->getStopLine();
    const auto &[acc_val, phidot_val, delta_val, kappa_val] = control_sequence;

#define plot_flag 0
#define rviz_flag 1

#if plot_flag
    if (canvas_settings_flag) {
        plt::figure_size(1024, 680);
        canvas_settings_flag = false;
    }
    plt::clf();

    // 第一行的图
    int row = 4, col = 1, num = 0;
    plt::subplot(row, col, ++num);

#endif
    // road
    for (const auto &road : roadNetwork->getRoads()) {
        auto road_id = road->getId();
        auto lanes = road->getLanes();
        for (const auto &lane : lanes) {
            auto lane_id = lane.getLaneID();
            auto lane_data = lane.getLaneData();
            if (lane_data.find("reference_line") != lane_data.end() && lane_data.find("left_boundary") != lane_data.end() && lane_data.find("right_boundary") != lane_data.end()) {
                auto [x_center, y_center] = lane_data.at("reference_line");
                auto [x_left, y_left] = lane_data.at("left_boundary");
                auto [x_right, y_right] = lane_data.at("right_boundary");
#if rviz_flag
                vis.publishRoadMarkerWithScale(x_center, y_center, x_left, y_left, x_right, y_right, "Road" + std::to_string(road_id) + "_lane" + std::to_string(lane_id), GRAY);
#endif
            }
        }
    }

    // zebra crossings
    for (const auto &crossing : roadNetwork->getZebraCrossings()) {
        double length, yaw;
        roadNetwork->calculateZebraCrossingParameters(crossing, length, yaw);

        auto [x_start, y_start] = crossing->getStartPoint();
        int crossing_id = crossing->getId();  // 获取斑马线的ID
#if rviz_flag
        vis.publishZebraCrossingMarker(crossing_id, x_start, y_start, length, 3, 1, 1, yaw);
#endif
    }

    // traffic light
#if rviz_flag
    vis.publishTrafficLightMarker(traffic_light_info.getId(),
                                  traffic_light_info.getPosition().x(),
                                  traffic_light_info.getPosition().y(),
                                  traffic_light_info.getHeight(),
                                  traffic_light_info.getState(),
                                  traffic_light_info.getOrientation());
#endif
    // connection
    for (const auto &connection : roadNetwork->getConnections()) {
        auto [direction, connection_path] = connection;
        auto lane_data = connection_path->getConnectionLane().getLaneData();
        if (lane_data.find("reference_line") != lane_data.end() && lane_data.find("left_boundary") != lane_data.end() && lane_data.find("right_boundary") != lane_data.end()) {
            auto [x_center, y_center] = lane_data.at("reference_line");
            auto [x_left, y_left] = lane_data.at("left_boundary");
            auto [x_right, y_right] = lane_data.at("right_boundary");
#if rviz_flag
            vis.publishRoadMarkerWithScale(x_center, y_center, x_left, y_left, x_right, y_right, "connection" + direction, GRAY);
#endif
        }
    }

#if plot_flag
    Plot::draw_road(x_center, y_center, x_left, y_left, x_right, y_right, "grey");
#endif

#if rviz_flag
    // ego marker
    vis.publishStaticAxes();
    vis.publishEgoStateMarker(TrajectoryPoint{ego_state.x, ego_state.y, times, ego_state.v, ego_state.phi}, "ego_state", LIGHT_BLUE);
    vis.publishTimeAndGearOverlay(std::to_string(times), "D");
    vis.publishEgoTrajectory(ego_trajectory.points, "ego_trajectory", GREEN);
    vis.publishEgoSpeedAndSteeringAngleAndAcceleration(ego_state.v * 3.6, delta_val * 180 / M_PI, acc_val);
    vis.publishSpeedProfile(speed_profile, "speed_profile", 0, GREEN);
    // stop line
    vis.publishStopLineMarker(build_barrier_wall, stop_line.x(), stop_line.y(), 0, 0.8, 15, 8, MAROON);
    // reference line
    vis.publishEgoReferenceLine(reference_line->path_points(), "reference_line", PINK);
#endif

#if plot_flag
    Plot::drawZebraCrossing(100, -MAXIMUM_LATERAL_OFFSET + 0.45 * 0.5, MAXIMUM_LATERAL_OFFSET * (double)road->getLanes().size() * 2, 3, 0.45, 0.45, 1.57);
    Plot::draw_optimize_trajectory(ego_trajectory, "g");
    Plot::draw_ego_rectangle(ego_state.x, ego_state.y, ego_state.phi, "b");
#endif

    //  obs marker
    for (const auto &obs_traj : all_obs_prediction_traj) {
        auto [id, target, traj] = obs_traj;
        if (target == "target") {
            vis.publishAllObstacleTrajectory(traj.points, id, "obs_trajectory" + std::to_string(id), RED);
        }
        if (target == "no_target") {
            vis.publishAllObstacleTrajectory(traj.points, id, "obs_trajectory" + std::to_string(id), RED, 0.25);
        }
    }

    for (const auto &obs : target_obs_init_state) {
        TrajectoryPoint obs_point{};
        obs_point.x = obs.x;
        obs_point.y = obs.y;
        obs_point.v = obs.v;
        obs_point.phi = obs.phi;
        obs_point.t = times;
        double vobs = obs.v;
        double phiobs = obs.phi;
        double asquare, bsquare;
        if (obs.type == ObstacleType::DYNAMIC_VEHICLE) {
            asquare = VEHICLE_LENGTH + fabs(vobs * cos(phiobs)) * DEFAULT_T_SAFE + DEFAULT_S_SAFE;
            bsquare = VEHICLE_WIDTH + DEFAULT_S_SAFE;

        } else if (obs.type == ObstacleType::PEDESTRIAN) {
            // TODO::optimize
            asquare = PEDESTRIAN_ENVELOPE_ELLIPSE_PARAM_A;
            bsquare = PEDESTRIAN_ENVELOPE_ELLIPSE_PARAM_B;
        }
        std::pair<double, double> ellipse_param = std::make_pair(asquare, bsquare);
#if rviz_flag
        vis.publishTargetObstacleMarker(obs_point, obs.id, obs.type, obs.intent, obs.priority, "obs_state", ellipse_param, 1.0);
#endif

#if plot_flag
        Plot::draw_obs_rectangle(obs_point, obs.id, obs.type, obs.intent, obs.priority, ellipse_param);
#endif
    }

    for (const auto &obs : no_target_obs_init_state) {
        TrajectoryPoint obs_point{};
        obs_point.x = obs.x;
        obs_point.y = obs.y;
        obs_point.v = obs.v;
        obs_point.phi = obs.phi;
        obs_point.t = times;
        double vobs = obs.v;
        double phiobs = obs.phi;
        double asquare, bsquare;
        if (obs.type == ObstacleType::DYNAMIC_VEHICLE) {
            asquare = VEHICLE_LENGTH + fabs(vobs * cos(phiobs)) * DEFAULT_T_SAFE + DEFAULT_S_SAFE;
            bsquare = VEHICLE_WIDTH + DEFAULT_S_SAFE;

        } else if (obs.type == ObstacleType::PEDESTRIAN) {
            // TODO::optimize
            asquare = PEDESTRIAN_ENVELOPE_ELLIPSE_PARAM_A;
            bsquare = PEDESTRIAN_ENVELOPE_ELLIPSE_PARAM_B;
        }
        std::pair<double, double> ellipse_param = std::make_pair(asquare, bsquare);
        //        vis.publishNoTargetObstacleMarker(obs_point, obs.id, obs.type, obs.intent, obs.priority, "obs_state", ellipse_param, 0.5);
#if rviz_flag
        vis.publishTargetObstacleMarker(obs_point, obs.id, obs.type, obs.intent, obs.priority, "obs_state", ellipse_param, 0.25);
#endif

#if plot_flag
        Plot::draw_obs_rectangle(obs_point, obs.id, obs.type, obs.intent, obs.priority, ellipse_param);
#endif
    }

#if plot_flag
    double limit_width = 160.0;
    plt::xlim(ego_state.x - limit_width / 2, ego_state.x + limit_width / 2);
#endif

    //===============================Control Sequence_==========================================//

#if plot_flag
    t_s.emplace_back(0.1 * cycle_times_);
    // Velocity subplot
    plt::subplot(row, col, ++num);
    velocity.emplace_back(ego_state.v);
    Plot::draw_velocity_curve(t_s, velocity);

    // Delta subplot
    //    plt::subplot(row, col, ++num);
    //    delta.emplace_back(delta_val);
    //    Plot::draw_delta_curve(t_s, delta);

    // Acceleration subplot
    plt::subplot(row, col, ++num);
    acc.emplace_back(acc_val);
    Plot::draw_acc_curve(t_s, acc);

    // kappa subplot
    //    plt::subplot(row, col, ++num);
    //    kappa.emplace_back(kappa_val);
    //    Plot::draw_kappa_curve(t_s, kappa);

    // plot config
    std::map<std::string, double> top;
    top["top"] = 0.965;
    std::map<std::string, double> left;
    left["left"] = 0.110;
    std::map<std::string, double> right;
    right["right"] = 0.970;
    plt::subplots_adjust(top);
    plt::subplots_adjust(left);
    plt::subplots_adjust(right);

    plt::show(false);
    plt::pause(0.0001);
#endif
}
